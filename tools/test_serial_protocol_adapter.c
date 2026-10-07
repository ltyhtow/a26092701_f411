#include "serial_protocol_adapter.h"
#include "serial_transport.h"
#include "app_clock.h"
#include "semphr.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static uint8_t tx_storage[256], rx_storage[256];
static lwrb_t tx, rx;
static bool fail_mutex, fail_task, fail_transport, fail_take, locked;
static unsigned mutex_created, mutex_deleted, task_created, task_deleted;
static unsigned transport_inits, transport_deinits, polls, takes, gives, critical_depth, callbacks;
static TickType_t last_timeout;
static BaseType_t scheduler;
static TaskHandle_t current_task;
static void (*created_task)(void *);
static jmp_buf end_iteration;
static int task_token, mutex_token, app_token;
static bool reset_pending;
static uint32_t link_generation;
static bool interrupt_after_snapshot;

void test_enter_critical(void) { ++critical_depth; }
void test_exit_critical(void) {
    assert(critical_depth > 0);
    if (--critical_depth == 0 && interrupt_after_snapshot) {
        interrupt_after_snapshot = false;
        ++link_generation;
        reset_pending = true;
    }
}
BaseType_t xTaskGetSchedulerState(void) { return scheduler; }
uint32_t app_clock_now_ms(void) { return 100000U; }
TickType_t app_clock_period_ticks(uint32_t milliseconds) { return (milliseconds + 9U) / 10U; }
SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    if (fail_mutex) return NULL;
    ++mutex_created; return &mutex_token;
}
void vSemaphoreDelete(SemaphoreHandle_t handle) {
    assert(handle == &mutex_token && !locked); ++mutex_deleted;
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t timeout) {
    assert(handle == &mutex_token); ++takes; last_timeout = timeout;
    if (fail_take) return pdFALSE;
    assert(!locked); locked = true; return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t handle) {
    assert(handle == &mutex_token && locked); ++gives; locked = false; return pdTRUE;
}
BaseType_t xTaskCreate(void (*task)(void *), const char *name, uint32_t stack,
                       void *argument, UBaseType_t priority, TaskHandle_t *handle) {
    assert(strcmp(name, "Protocol") == 0 && stack >= 512 && argument == NULL && priority == 2);
    if (fail_task) return pdFAIL;
    created_task = task; *handle = &task_token; ++task_created; return pdPASS;
}
void vTaskDelete(TaskHandle_t handle) { assert(handle == &task_token); ++task_deleted; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return current_task; }
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks) {
    assert(clear == pdTRUE && ticks == 1U && !locked);
    longjmp(end_iteration, 1);
}
BaseType_t serial_transport_init(TaskHandle_t notify_task) {
    assert(notify_task == &task_token); ++transport_inits;
    if (fail_transport) return pdFAIL;
    assert(lwrb_init(&tx, tx_storage, sizeof(tx_storage)));
    assert(lwrb_init(&rx, rx_storage, sizeof(rx_storage)));
    return pdPASS;
}
void serial_transport_deinit(void) { ++transport_deinits; }
lwrb_t *serial_transport_rx_buffer(void) { return &rx; }
lwrb_t *serial_transport_tx_buffer(void) { return &tx; }
void serial_transport_poll_tx(void) { ++polls; }
bool serial_transport_take_rx_reset(void) {
    const bool result = reset_pending;
    reset_pending = false;
    return result;
}
uint32_t serial_transport_link_generation(void) { return link_generation; }

static bool on_packet(uint32_t command, const serial_message_t *message, void *context) {
    assert(context == &app_token && command == SERIAL_CMD_SYSTEM && message->system.action == 2);
    const unsigned before = takes;
    assert(locked);
    assert(serial_protocol_try_send(command, &message->system, sizeof(message->system)) == SERIAL_PROTOCOL_OK);
    assert(takes == before); /* callback response doesn't deadlock on own mutex */
    ++callbacks;
    return false;
}

int main(void) {
    current_task = &app_token;
    system_command_t command = {1,7,2,0};
    assert(serial_protocol_try_send(SERIAL_CMD_SYSTEM, &command, sizeof(command)) == SERIAL_PROTOCOL_NOT_READY);
    fail_mutex = true;
    assert(!serial_service_start(on_packet, &app_token));
    assert(mutex_created == 0 && task_created == 0);
    fail_mutex = false; fail_task = true;
    assert(!serial_service_start(on_packet, &app_token));
    assert(mutex_created == 1 && mutex_deleted == 1 && task_created == 0);
    fail_task = false; fail_transport = true;
    assert(!serial_service_start(on_packet, &app_token));
    assert(task_created == 1 && task_deleted == 1 && mutex_created == mutex_deleted);
    assert(transport_inits == 1 && transport_deinits == 1);
    fail_transport = false;
    scheduler = taskSCHEDULER_RUNNING;
    assert(!serial_service_start(on_packet, &app_token));
    scheduler = taskSCHEDULER_NOT_STARTED;
    assert(serial_service_start(on_packet, &app_token));
    assert(!serial_service_start(on_packet, &app_token));
    fail_take = true;
    assert(serial_protocol_try_send(SERIAL_CMD_SYSTEM, &command, sizeof(command)) == SERIAL_PROTOCOL_BUSY);
    assert(last_timeout == 0 && polls == 0);
    assert(serial_protocol_send(SERIAL_CMD_SYSTEM, &command, sizeof(command)) == SERIAL_PROTOCOL_BUSY);
    assert(last_timeout == 1); /* 10 ms converted at 100 Hz */
    fail_take = false;
    assert(serial_protocol_try_send(SERIAL_CMD_SYSTEM, &command, sizeof(command)) == SERIAL_PROTOCOL_OK);
    assert(last_timeout == 0 && !locked && polls == 2);
    uint8_t frame[128]; const size_t length = lwrb_read(&tx, frame, sizeof(frame));
    assert(length == 9 && frame[0] == 0xAA && frame[1] == 3 && frame[2] == 4 && frame[8] == 0x55);
    assert(serial_protocol_try_send(SERIAL_CMD_RESPONSE, &command, sizeof(command)) == SERIAL_PROTOCOL_INVALID_ARGUMENT);
    assert(!locked);
    assert(lwrb_write(&rx, frame, length) == length);
    current_task = &task_token; scheduler = taskSCHEDULER_RUNNING;
    if (setjmp(end_iteration) == 0) created_task(NULL);
    assert(callbacks == 1 && !locked);
    serial_protocol_stats_t stats;
    serial_protocol_get_stats(&stats);
    assert(stats.queue_overruns == 1 && stats.rx_timeouts == 0 && critical_depth == 0);
    /* A prefix consumed before disconnect cannot combine with a suffix from
     * another USB session. A full fresh frame still dispatches afterwards. */
    assert(lwrb_write(&rx, frame, 4) == 4);
    if (setjmp(end_iteration) == 0) created_task(NULL);
    assert(callbacks == 1);
    ++link_generation;
    reset_pending = true;
    assert(lwrb_write(&rx, frame + 4, length - 4) == length - 4);
    if (setjmp(end_iteration) == 0) created_task(NULL);
    assert(callbacks == 1);
    assert(lwrb_write(&rx, frame, length) == length);
    if (setjmp(end_iteration) == 0) created_task(NULL);
    assert(callbacks == 2 && critical_depth == 0);
    assert(lwrb_write(&rx, frame, length) == length);
    interrupt_after_snapshot = true;
    if (setjmp(end_iteration) == 0) created_task(NULL);
    assert(callbacks == 2 && reset_pending && critical_depth == 0);
    if (setjmp(end_iteration) == 0) created_task(NULL);
    assert(!reset_pending);
    const unsigned deleted_before = task_deleted;
    serial_service_stop(); /* runtime stop is forbidden, resources stay alive */
    assert(task_deleted == deleted_before);
    scheduler = taskSCHEDULER_NOT_STARTED;
    serial_service_stop(); serial_service_stop();
    assert(task_deleted == task_created && mutex_deleted == mutex_created && transport_deinits == transport_inits);
    assert(serial_protocol_try_send(SERIAL_CMD_SYSTEM, &command, sizeof(command)) == SERIAL_PROTOCOL_NOT_READY);
    assert(serial_service_start(NULL, NULL)); /* retry after full cleanup */
    serial_protocol_get_stats(&stats);
    assert(stats.queue_overruns == 0);
    serial_service_stop();
    puts("PASS: protocol lifecycle failure cleanup, 100 Hz timing, nonblocking send and reentrant callback");
    return 0;
}
