#include "serial_protocol_adapter.h"
#include "serial_transport.h"
#include "app_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

#define PROTOCOL_TASK_STACK_WORDS 512U
#define PROTOCOL_TASK_PERIOD_MS 5U

static serial_protocol_engine_t engine;
static TaskHandle_t protocol_task_handle;
static SemaphoreHandle_t protocol_mutex;
static bool transport_started;
static serial_packet_handler_t application_callback;
static void *application_context;
static uint32_t receive_generation;

static bool deliver_packet(uint32_t command, const serial_message_t *message, void *context) {
    (void)context;
    /* An interrupt may reset USB while the parser is consuming staged bytes.
     * A frame is accepted only in the session in which this poll began.
     * Application callbacks only use bounded, nonblocking queue/reply paths.
     * USB event processing is deferred when a reply polls inside this section.
     */
    taskENTER_CRITICAL();
    const bool accepted = serial_transport_link_generation() == receive_generation &&
        (application_callback == NULL || application_callback(command, message, application_context));
    taskEXIT_CRITICAL();
    return accepted;
}

static void reset_receive_if_needed(void) {
    taskENTER_CRITICAL();
    if (serial_transport_take_rx_reset()) serial_engine_reset_rx(&engine);
    taskEXIT_CRITICAL();
}

static void protocol_task(void *argument) {
    (void)argument;
    for (;;) {
        (void)xSemaphoreTake(protocol_mutex, portMAX_DELAY);
        serial_transport_poll_tx();
        taskENTER_CRITICAL();
        if (serial_transport_take_rx_reset()) serial_engine_reset_rx(&engine);
        receive_generation = serial_transport_link_generation();
        taskEXIT_CRITICAL();
        const bool pending = serial_engine_poll(&engine, app_clock_now_ms());
        serial_transport_poll_tx();
        (void)xSemaphoreGive(protocol_mutex);
        /* Release the mutex after each bounded poll. Pending ISR notifications
         * may wake this wait immediately; higher priority control can preempt. */
        (void)ulTaskNotifyTake(pdTRUE, app_clock_period_ticks(pending ? 1U : PROTOCOL_TASK_PERIOD_MS));
    }
}

bool serial_service_start(serial_packet_handler_t callback, void *context) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED ||
        protocol_mutex != NULL || protocol_task_handle != NULL) return false;
    protocol_mutex = xSemaphoreCreateMutex();
    if (protocol_mutex == NULL) return false;
    if (xTaskCreate(protocol_task, "Protocol", PROTOCOL_TASK_STACK_WORDS, NULL, 2U,
                    &protocol_task_handle) != pdPASS) {
        serial_service_stop();
        return false;
    }
    /* Mark attempted startup too: partially initialized DMA needs cleanup. */
    transport_started = true;
    application_callback = callback;
    application_context = context;
    if (serial_transport_init(protocol_task_handle) != pdPASS ||
        !serial_engine_init(&engine, serial_transport_tx_buffer(), serial_transport_rx_buffer(),
                            deliver_packet, NULL)) {
        serial_service_stop();
        return false;
    }
    return true;
}

void serial_service_stop(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) return;
    serial_engine_deinit(&engine);
    application_callback = NULL;
    application_context = NULL;
    if (transport_started) {
        serial_transport_deinit();
        transport_started = false;
    }
    if (protocol_task_handle != NULL) {
        vTaskDelete(protocol_task_handle);
        protocol_task_handle = NULL;
    }
    if (protocol_mutex != NULL) {
        vSemaphoreDelete(protocol_mutex);
        protocol_mutex = NULL;
    }
}

static serial_protocol_result_t send_with_timeout(uint32_t command, const void *data,
                                                   size_t native_size, TickType_t timeout) {
    if (!engine.ready || protocol_mutex == NULL) return SERIAL_PROTOCOL_NOT_READY;
    const bool already_locked = xTaskGetCurrentTaskHandle() == protocol_task_handle;
    if (!already_locked && xSemaphoreTake(protocol_mutex, timeout) != pdTRUE)
        return SERIAL_PROTOCOL_BUSY;
    serial_transport_poll_tx();
    reset_receive_if_needed();
    const serial_protocol_result_t result = serial_engine_send(&engine, command, data, native_size);
    serial_transport_poll_tx();
    if (!already_locked) (void)xSemaphoreGive(protocol_mutex);
    return result;
}

serial_protocol_result_t serial_protocol_send(uint32_t command, const void *data, size_t length) {
    return send_with_timeout(command, data, length, app_clock_period_ticks(10U));
}

serial_protocol_result_t serial_protocol_try_send(uint32_t command, const void *data, size_t length) {
    return send_with_timeout(command, data, length, 0U);
}

void serial_protocol_get_stats(serial_protocol_stats_t *stats) {
    if (stats != NULL) {
        taskENTER_CRITICAL();
        *stats = engine.stats;
        taskEXIT_CRITICAL();
    }
}
