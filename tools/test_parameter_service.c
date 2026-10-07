/* Public-API integration test: actual worker, profile and transactional store.
 * Only EEPROM, balance-task completion, FreeRTOS and the framed send API are
 * replaced. No production .c inclusion and no access to private service state. */
#include "parameter_service.h"
#include "parameter_store.h"
#include "eeprom_port.h"
#include "balance_task.h"
#include "app_clock.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct parameter_test_queue { unsigned capacity, size, count; uint8_t items[4][128]; };
static QueueHandle_t request_queue, completion_queue;
static TaskFunction_t task_entry;
static BaseType_t scheduler;
static jmp_buf finished;
static const char *scenario;
static unsigned step, critical_depth, queue_creates, queue_deletes, task_deletes;
static unsigned fail_queue, begin_calls, end_calls, responses, writes, reads, init_calls;
static unsigned apply_calls, apply_responses, saved_writes;
static bool fail_task, maintenance, missing_eeprom, fail_next_read, ignore_writes;
static int write_budget = -1;
static uint32_t eeprom_error, revision;
static balance_state_t state = BALANCE_STATE_DISARMED;
static balance_controller_config_t board_defaults, active_config, pending_config;
static balance_config_result_fn completion;
static void *completion_context;
static uint32_t completion_cookie;
static balance_config_result_t application_result = BALANCE_CONFIG_OK;
static uint8_t nvm[EEPROM_PORT_CAPACITY_BYTES];
static command_response_t response;

static void drive(void);
static parameter_command_t command(uint8_t operation, uint16_t key, int value, uint8_t sequence) {
    return (parameter_command_t){.version = SERIAL_PROTOCOL_VERSION, .sequence = sequence,
        .operation = operation, .key = key, .value_q16_16 = value * 65536};
}
static void submit(uint8_t operation, uint16_t key, int value, uint8_t sequence) {
    parameter_command_t request = command(operation, key, value, sequence);
    assert(parameter_service_submit(&request, NULL));
}
static void expect(uint8_t sequence, serial_response_status_t status) {
    assert(response.sequence == sequence && response.status == status);
    assert(response.config_revision == revision && response.request_command == SERIAL_CMD_PARAMETER);
    assert(response.version == SERIAL_PROTOCOL_VERSION && response.timestamp_ms == 1234U);
    assert(!maintenance && completion == NULL && critical_depth == 0U);
}
void parameter_test_enter_critical(void) { ++critical_depth; }
void parameter_test_exit_critical(void) { assert(critical_depth != 0U); --critical_depth; }
BaseType_t xTaskGetSchedulerState(void) { return scheduler; }
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, uint16_t stack,
    void *argument, UBaseType_t priority, TaskHandle_t *handle) {
    assert(maintenance && strcmp(name, "Parameters") == 0 && stack >= 768U);
    assert(argument == NULL && priority == 1U);
    if (fail_task) return pdFAIL;
    task_entry = entry; *handle = (void *)1;
    return pdPASS;
}
void vTaskDelete(TaskHandle_t handle) { assert(handle == (void *)1); ++task_deletes; }
QueueHandle_t xQueueCreate(UBaseType_t capacity, UBaseType_t size) {
    assert(maintenance && capacity <= 4U && size <= 128U);
    if (++queue_creates == fail_queue) return NULL;
    QueueHandle_t queue = calloc(1U, sizeof(*queue));
    assert(queue != NULL); queue->capacity = capacity; queue->size = size;
    if (capacity == 4U) request_queue = queue; else { assert(capacity == 1U); completion_queue = queue; }
    return queue;
}
void vQueueDelete(QueueHandle_t queue) { assert(queue != NULL); ++queue_deletes; free(queue); }
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait) {
    assert(queue == request_queue && wait == 0U);
    if (queue->count == queue->capacity) return pdFAIL;
    memcpy(queue->items[queue->count++], item, queue->size); return pdPASS;
}
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item) {
    assert(queue == completion_queue && queue->capacity == 1U && maintenance);
    memcpy(queue->items[0], item, queue->size); queue->count = 1U; return pdPASS;
}
BaseType_t xQueueReset(QueueHandle_t queue) { queue->count = 0U; return pdPASS; }
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t wait) {
    assert(wait == portMAX_DELAY && critical_depth == 0U);
    if (queue == request_queue && queue->count == 0U) drive();
    if (queue == completion_queue && queue->count == 0U) {
        assert(completion != NULL && maintenance && responses == apply_responses);
        /* Queries while the worker awaits real application see the old revision
         * and mark maintenance busy; queue acceptance has not emitted an ACK. */
        const uint32_t before = revision;
        const float old_kp = active_config.balance_loop.kp;
        submit(PARAMETER_OP_GET, PARAMETER_KEY_BALANCE_KP, 0, 0xEEU);
        assert(response.config_revision == before && response.value_q16_16 == (int32_t)(old_kp * 65536.0f));
        assert((response.storage_flags & SERIAL_STORAGE_BUSY) != 0U);
        if (application_result == BALANCE_CONFIG_OK) { active_config = pending_config; ++revision; }
        balance_config_result_fn callback = completion; completion = NULL;
        callback(application_result, completion_cookie, revision, completion_context);
        assert(maintenance && queue->count == 1U);
    }
    assert(queue->count != 0U);
    memcpy(item, queue->items[0], queue->size);
    for (unsigned i = 1U; i < queue->count; ++i) memcpy(queue->items[i - 1U], queue->items[i], queue->size);
    --queue->count; return pdTRUE;
}
uint32_t app_clock_now_ms(void) { return 1234U; }
bool balance_task_begin_maintenance(void) {
    ++begin_calls;
    if (maintenance || state != BALANCE_STATE_DISARMED) return false;
    maintenance = true; return true;
}
void balance_task_end_maintenance(void) {
    assert(maintenance && completion == NULL); maintenance = false; ++end_calls;
}
balance_state_t balance_task_get_state(void) { return state; }
void balance_task_get_snapshot(balance_runtime_snapshot_t *snapshot) {
    memset(snapshot, 0, sizeof(*snapshot)); snapshot->state = state;
}
void balance_task_get_config_snapshot(balance_controller_config_t *config, uint32_t *version) {
    *config = active_config; if (version) *version = revision;
}
balance_config_result_t balance_task_request_config(const balance_controller_config_t *config,
    uint32_t cookie, balance_config_result_fn callback, void *context) {
    assert(maintenance && completion == NULL && !config->auto_recovery_enabled);
    pending_config = *config; completion = callback; completion_context = context;
    completion_cookie = cookie; apply_responses = responses; ++apply_calls;
    return BALANCE_CONFIG_OK;
}
bool balance_task_request_arm(void) { assert(!"Parameter service must never request arm"); return false; }
serial_protocol_result_t serial_protocol_try_send(uint32_t type, const void *data, size_t length) {
    assert(type == SERIAL_CMD_RESPONSE && length == sizeof(response) && critical_depth == 0U);
    memcpy(&response, data, length); ++responses; return SERIAL_PROTOCOL_OK;
}
bool eeprom_port_init(void) {
    assert(maintenance && critical_depth == 0U); ++init_calls;
    eeprom_error = missing_eeprom ? EEPROM_PORT_ERROR_NOT_CONFIGURED : EEPROM_PORT_ERROR_NONE;
    return !missing_eeprom;
}
uint32_t eeprom_port_last_error(void) { return eeprom_error; }
bool eeprom_port_read(uint16_t address, uint8_t *data, size_t length) {
    assert(maintenance && !missing_eeprom && critical_depth == 0U && address + length <= sizeof(nvm));
    ++reads;
    if (fail_next_read) { fail_next_read = false; eeprom_error = EEPROM_PORT_ERROR_READ; return false; }
    memcpy(data, nvm + address, length); return true;
}
bool eeprom_port_write(uint16_t address, const uint8_t *data, size_t length) {
    assert(maintenance && !missing_eeprom && critical_depth == 0U && address + length <= sizeof(nvm));
    ++writes;
    if (ignore_writes) return true;
    for (size_t i = 0U; i < length; ++i) {
        if (write_budget == 0) { eeprom_error = EEPROM_PORT_ERROR_WRITE; return false; }
        nvm[address + i] = data[i]; if (write_budget > 0) --write_budget;
    }
    return true;
}
static bool raw_read(uint16_t address, uint8_t *data, uint16_t length, void *context) {
    (void)context; memcpy(data, nvm + address, length); return true;
}
static bool raw_write(uint16_t address, const uint8_t *data, uint16_t length, void *context) {
    (void)context; memcpy(nvm + address, data, length); return true;
}
static void seed(float kp) {
    parameter_store_t storage; parameter_profile_t profile;
    assert(parameter_store_init(&storage, raw_read, raw_write, NULL, &board_defaults));
    parameter_profile_defaults(&profile, &board_defaults);
    assert(parameter_profile_set(&profile, PARAMETER_KEY_BALANCE_KP, kp) == PARAMETER_OK);
    assert(parameter_store_save(&storage, &profile) == PARAMETER_STORE_OK);
}
static float stored_kp(void) {
    parameter_store_t storage; parameter_profile_t profile;
    assert(parameter_store_init(&storage, raw_read, raw_write, NULL, &board_defaults));
    assert(parameter_store_load(&storage, &profile) == PARAMETER_STORE_OK);
    return profile.controller.balance_loop.kp;
}
static void done(void) {
    assert(!maintenance && completion == NULL && state == BALANCE_STATE_DISARMED);
    longjmp(finished, 1);
}
static void drive_disabled(void) {
    switch (step++) {
        case 0:
            assert(init_calls == 1U && writes == 0U && apply_calls == 0U);
            submit(PARAMETER_OP_STATUS, 9U, 0, 250U); expect(250U, SERIAL_RESPONSE_INVALID);
            submit(PARAMETER_OP_SET, PARAMETER_KEY_BALANCE_KP, 181, 1U); break;
        case 1:
            expect(1U, SERIAL_RESPONSE_OK);
            assert(response.value_q16_16 == 181 * 65536 && revision == 1U);
            assert((response.storage_flags & (SERIAL_STORAGE_CONFIGURED | SERIAL_STORAGE_READY | SERIAL_STORAGE_ERROR)) == 0U);
            submit(PARAMETER_OP_GET, PARAMETER_KEY_BALANCE_KP, 0, 2U);
            expect(2U, SERIAL_RESPONSE_OK); assert(response.value_q16_16 == 181 * 65536);
            submit(PARAMETER_OP_STATUS, 0U, 0, 3U); expect(3U, SERIAL_RESPONSE_OK);
            assert(response.value_q16_16 == EEPROM_PORT_ERROR_NOT_CONFIGURED * 65536);
            submit(PARAMETER_OP_SAVE, 0U, 0, 4U); break;
        default:
            expect(4U, SERIAL_RESPONSE_NOT_READY); assert(writes == 0U && reads == 0U && revision == 1U);
            done();
    }
}
static void drive_save(void) {
    switch (step++) {
        case 0: submit(PARAMETER_OP_SET, PARAMETER_KEY_BALANCE_KP, 181, 1U); break;
        case 1: expect(1U, SERIAL_RESPONSE_OK); submit(PARAMETER_OP_SAVE, 0U, 0, 2U); break;
        case 2:
            expect(2U, SERIAL_RESPONSE_OK); assert(stored_kp() == 181.0f);
            assert((response.storage_flags & SERIAL_STORAGE_VALID) != 0U);
            assert((response.storage_flags & (SERIAL_STORAGE_DIRTY | SERIAL_STORAGE_ERROR)) == 0U);
            saved_writes = writes; submit(PARAMETER_OP_SAVE, 0U, 0, 3U); break;
        case 3:
            expect(3U, SERIAL_RESPONSE_OK); assert(writes == saved_writes);
            submit(PARAMETER_OP_SET, PARAMETER_KEY_BALANCE_KP, 182, 4U); break;
        case 4:
            expect(4U, SERIAL_RESPONSE_OK); write_budget = 9;
            submit(PARAMETER_OP_SAVE, 0U, 0, 5U); break;
        case 5:
            expect(5U, SERIAL_RESPONSE_STORAGE_ERROR); assert(stored_kp() == 181.0f && active_config.balance_loop.kp == 182.0f);
            assert((response.storage_flags & (SERIAL_STORAGE_DIRTY | SERIAL_STORAGE_ERROR)) == (SERIAL_STORAGE_DIRTY | SERIAL_STORAGE_ERROR));
            write_budget = -1; submit(PARAMETER_OP_SAVE, 0U, 0, 6U); break;
        case 6:
            expect(6U, SERIAL_RESPONSE_OK); assert(stored_kp() == 182.0f);
            fail_next_read = true; submit(PARAMETER_OP_SAVE, 0U, 0, 7U); break;
        case 7:
            expect(7U, SERIAL_RESPONSE_STORAGE_ERROR);
            assert((response.storage_flags & SERIAL_STORAGE_READY) == 0U);
            submit(PARAMETER_OP_SET, PARAMETER_KEY_BALANCE_KP, 183, 8U); break;
        case 8:
            expect(8U, SERIAL_RESPONSE_OK); ignore_writes = true;
            submit(PARAMETER_OP_SAVE, 0U, 0, 9U); break;
        case 9:
            expect(9U, SERIAL_RESPONSE_STORAGE_ERROR); assert(stored_kp() == 182.0f);
            ignore_writes = false; submit(PARAMETER_OP_LOAD, 0U, 0, 10U); break;
        default:
            expect(10U, SERIAL_RESPONSE_OK); assert(active_config.balance_loop.kp == 182.0f && revision == 4U);
            done();
    }
}
static void drive_unsafe(void) {
    switch (step++) {
        case 0:
            state = BALANCE_STATE_ARMED; submit(PARAMETER_OP_SET, PARAMETER_KEY_BALANCE_KP, 181, 1U); break;
        case 1:
            expect(1U, SERIAL_RESPONSE_UNSAFE); assert(apply_calls == 0U && writes == 0U);
            state = BALANCE_STATE_FALLEN; submit(PARAMETER_OP_SAVE, 0U, 0, 2U); break;
        case 2:
            expect(2U, SERIAL_RESPONSE_UNSAFE); assert(writes == 0U);
            state = BALANCE_STATE_DISARMED; application_result = BALANCE_CONFIG_UNSAFE;
            submit(PARAMETER_OP_SET, PARAMETER_KEY_BALANCE_KP, 182, 3U); break;
        case 3:
            expect(3U, SERIAL_RESPONSE_UNSAFE); assert(revision == 0U && active_config.balance_loop.kp == board_defaults.balance_loop.kp);
            submit(PARAMETER_OP_SET, PARAMETER_KEY_MAX_PWM, 5000, 4U); break;
        default:
            expect(4U, SERIAL_RESPONSE_INVALID); assert(revision == 0U && apply_calls == 1U);
            done();
    }
}
static void drive(void) {
    assert(!maintenance && critical_depth == 0U);
    if (strcmp(scenario, "disabled_ram") == 0) drive_disabled();
    else if (strcmp(scenario, "save_failures") == 0) drive_save();
    else if (strcmp(scenario, "unsafe") == 0) drive_unsafe();
    else if (strcmp(scenario, "boot_load") == 0) {
        assert(active_config.balance_loop.kp == 222.0f && revision == 1U && apply_calls == 1U);
        assert(!active_config.auto_recovery_enabled && writes == 0U);
        submit(PARAMETER_OP_GET, PARAMETER_KEY_BALANCE_KP, 0, 1U); expect(1U, SERIAL_RESPONSE_OK);
        assert(response.value_q16_16 == 222 * 65536);
        assert((response.storage_flags & SERIAL_STORAGE_VALID) != 0U && (response.storage_flags & SERIAL_STORAGE_DIRTY) == 0U);
        done();
    } else assert(!"Unknown worker scenario");
}

int main(int argc, char **argv) {
    assert(argc == 2); scenario = argv[1];
    balance_controller_default_config(&board_defaults);
    board_defaults.max_pwm = 4500.0f; board_defaults.auto_recovery_enabled = false;
    active_config = board_defaults; memset(nvm, 0xFF, sizeof(nvm));
    if (strcmp(scenario, "disabled_ram") == 0) missing_eeprom = true;
    if (strcmp(scenario, "boot_load") == 0) seed(222.0f);
    if (strcmp(scenario, "queue_fail_1") == 0) fail_queue = 1U;
    if (strcmp(scenario, "queue_fail_2") == 0) fail_queue = 2U;
    if (strcmp(scenario, "task_fail") == 0) fail_task = true;
    parameter_command_t before_start = command(PARAMETER_OP_STATUS, 0U, 0, 0U);
    assert(!parameter_service_submit(&before_start, NULL));
    if (fail_queue || fail_task) {
        assert(!parameter_service_start(&board_defaults));
        assert(!maintenance && begin_calls == 1U && end_calls == 1U);
        assert(queue_deletes == (fail_queue ? 1U : 2U) && task_deletes == 0U);
        assert(!parameter_service_submit(&before_start, NULL));
        assert(init_calls == 0U && writes == 0U);
        puts("PASS parameter-service failed start releases gate and queues"); return 0;
    }
    assert(parameter_service_start(&board_defaults) && maintenance && task_entry != NULL);
    assert(!parameter_service_start(&board_defaults));
    scheduler = taskSCHEDULER_RUNNING;
    if (setjmp(finished) == 0) task_entry(NULL);
    assert(completion == NULL && !maintenance && begin_calls == end_calls + (strcmp(scenario, "unsafe") == 0 ? 2U : 0U));
    scheduler = taskSCHEDULER_NOT_STARTED; parameter_service_stop();
    assert(queue_deletes == 2U && task_deletes == 1U);
    printf("PASS parameter-service %s: actual worker/profile/store, maintenance, application-before-ACK, no auto-arm\n", scenario);
    return 0;
}
