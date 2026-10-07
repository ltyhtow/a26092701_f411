#include "parameter_service.h"
#include "parameter_store.h"
#include "eeprom_port.h"
#include "balance_task.h"
#include "app_clock.h"
#include "queue.h"
#include "task.h"
#include <limits.h>
#include <math.h>
#include <string.h>

typedef struct { balance_config_result_t result; uint32_t cookie, revision; } apply_result_t;
static QueueHandle_t requests, completions;
static TaskHandle_t worker_task;
static balance_controller_config_t defaults;
static parameter_store_t store;
static bool started, configured, ready, saved_valid, working;
static uint16_t last_error;
static uint8_t saved_bytes[PARAMETER_PROFILE_ENCODED_SIZE];
static uint32_t next_cookie;

static bool storage_read(uint16_t address, uint8_t *data, uint16_t length, void *context) {
    (void)context; return eeprom_port_read(address, data, length);
}
static bool storage_write(uint16_t address, const uint8_t *data, uint16_t length, void *context) {
    (void)context; return eeprom_port_write(address, data, length);
}
static void set_working(bool value) {
    taskENTER_CRITICAL(); working = value; taskEXIT_CRITICAL();
}
static bool ensure_storage(void) {
    if (ready) return true;
    bool available = eeprom_port_init();
    uint32_t error = eeprom_port_last_error();
    taskENTER_CRITICAL();
    ready = available;
    configured = error != EEPROM_PORT_ERROR_NOT_CONFIGURED;
    last_error = available ? 0U : (uint16_t)error;
    taskEXIT_CRITICAL();
    return available;
}
static void remember_saved(const parameter_profile_t *profile) {
    uint8_t bytes[PARAMETER_PROFILE_ENCODED_SIZE];
    if (!parameter_profile_encode(profile, bytes, sizeof(bytes))) return;
    taskENTER_CRITICAL();
    memcpy(saved_bytes, bytes, sizeof(bytes));
    saved_valid = true; last_error = 0;
    taskEXIT_CRITICAL();
}
static void remember_store_error(parameter_store_result_t result) {
    taskENTER_CRITICAL();
    last_error = (result == PARAMETER_STORE_OK || result == PARAMETER_STORE_UNCHANGED ||
                  result == PARAMETER_STORE_EMPTY) ? 0U : (uint16_t)(0x100U + result);
    if (result == PARAMETER_STORE_EMPTY || result == PARAMETER_STORE_INVALID_RECORD) saved_valid = false;
    if (result == PARAMETER_STORE_IO_ERROR) ready = false; /* Re-probe on the next explicit I/O request. */
    taskEXIT_CRITICAL();
}
static void active_profile(parameter_profile_t *profile, uint32_t *revision) {
    parameter_profile_defaults(profile, &defaults);
    balance_task_get_config_snapshot(&profile->controller, revision);
}
static uint16_t storage_flags(const parameter_profile_t *active) {
    uint8_t bytes[PARAMETER_PROFILE_ENCODED_SIZE];
    bool encoded = parameter_profile_encode(active, bytes, sizeof(bytes));
    taskENTER_CRITICAL();
    uint16_t flags = (configured ? SERIAL_STORAGE_CONFIGURED : 0U) |
                     (ready ? SERIAL_STORAGE_READY : 0U) |
                     (saved_valid ? SERIAL_STORAGE_VALID : 0U) |
                     (working ? SERIAL_STORAGE_BUSY : 0U) |
                     (last_error != 0U && last_error != EEPROM_PORT_ERROR_NOT_CONFIGURED ? SERIAL_STORAGE_ERROR : 0U);
    if (!saved_valid || !encoded || memcmp(bytes, saved_bytes, sizeof(bytes))) flags |= SERIAL_STORAGE_DIRTY;
    taskEXIT_CRITICAL();
    return flags;
}
static int32_t to_q16(float value) {
    if (!isfinite(value)) return 0;
    double scaled = (double)value * 65536.0;
    if (scaled >= INT32_MAX) return INT32_MAX;
    if (scaled <= INT32_MIN) return INT32_MIN;
    return (int32_t)llround(scaled);
}
static void respond_snapshot(const parameter_command_t *command, serial_response_status_t status, float value,
                              const parameter_profile_t *active, uint32_t revision) {
    balance_runtime_snapshot_t snapshot;
    balance_task_get_snapshot(&snapshot);
    command_response_t response = {
        .version = SERIAL_PROTOCOL_VERSION, .sequence = command->sequence,
        .request_command = SERIAL_CMD_PARAMETER, .status = (uint8_t)status, .key = command->key,
        .controller_state = (uint8_t)snapshot.state, .storage_flags = storage_flags(active),
        .value_q16_16 = to_q16(value), .fault_flags = snapshot.fault_flags,
        .config_revision = revision, .timestamp_ms = app_clock_now_ms()
    };
    (void)serial_protocol_try_send(SERIAL_CMD_RESPONSE, &response, sizeof(response));
}
static void respond(const parameter_command_t *command, serial_response_status_t status, float value) {
    parameter_profile_t active;
    uint32_t revision;
    active_profile(&active, &revision);
    respond_snapshot(command, status, value, &active, revision);
}
static void config_completed(balance_config_result_t result, uint32_t cookie,
                              uint32_t revision, void *context) {
    (void)context;
    apply_result_t completion = {result, cookie, revision};
    (void)xQueueOverwrite(completions, &completion);
}
static serial_response_status_t apply_profile(const parameter_profile_t *profile) {
    uint32_t cookie = ++next_cookie;
    (void)xQueueReset(completions);
    balance_config_result_t result = balance_task_request_config(&profile->controller, cookie, config_completed, NULL);
    if (result == BALANCE_CONFIG_OK) {
        apply_result_t completion = {0};
        /* Only the storage worker waits. Never unlock maintenance before actual
         * control completion: a stalled controller keeps outputs inhibited. */
        for (;;) {
            if (xQueueReceive(completions, &completion, portMAX_DELAY) == pdTRUE && completion.cookie == cookie) break;
        }
        result = completion.result;
    }
    switch (result) {
        case BALANCE_CONFIG_OK: return SERIAL_RESPONSE_OK;
        case BALANCE_CONFIG_BUSY: return SERIAL_RESPONSE_BUSY;
        case BALANCE_CONFIG_UNSAFE: return SERIAL_RESPONSE_UNSAFE;
        case BALANCE_CONFIG_INVALID: return SERIAL_RESPONSE_INVALID;
        default: return SERIAL_RESPONSE_NOT_READY;
    }
}
static void handle_job(const parameter_command_t *command) {
    if (!balance_task_begin_maintenance()) {
        respond(command, balance_task_get_state() == BALANCE_STATE_DISARMED ? SERIAL_RESPONSE_BUSY : SERIAL_RESPONSE_UNSAFE, 0);
        return;
    }
    set_working(true);
    parameter_profile_t candidate;
    uint32_t revision;
    active_profile(&candidate, &revision);
    serial_response_status_t status = SERIAL_RESPONSE_OK;
    float value = 0;
    if (command->operation == PARAMETER_OP_SET) {
        parameter_result_t result = parameter_profile_set(&candidate, command->key,
                                                          (float)command->value_q16_16 / 65536.0f);
        if (result != PARAMETER_OK) status = result == PARAMETER_UNKNOWN_KEY ? SERIAL_RESPONSE_UNSUPPORTED : SERIAL_RESPONSE_INVALID;
        else {
            status = apply_profile(&candidate);
            if (status == SERIAL_RESPONSE_OK) (void)parameter_profile_get(&candidate, command->key, &value);
        }
    } else if (command->operation == PARAMETER_OP_DEFAULTS) {
        parameter_profile_defaults(&candidate, &defaults);
        status = apply_profile(&candidate);
    } else if (!ensure_storage()) status = SERIAL_RESPONSE_NOT_READY;
    else if (command->operation == PARAMETER_OP_SAVE) {
        parameter_store_result_t result = parameter_store_save(&store, &candidate);
        remember_store_error(result);
        if (result == PARAMETER_STORE_OK || result == PARAMETER_STORE_UNCHANGED) remember_saved(&candidate);
        else status = SERIAL_RESPONSE_STORAGE_ERROR;
    } else {
        parameter_store_result_t result = parameter_store_load(&store, &candidate);
        remember_store_error(result);
        if (result == PARAMETER_STORE_OK) {
            remember_saved(&candidate);
            status = apply_profile(&candidate);
        } else status = result == PARAMETER_STORE_EMPTY ? SERIAL_RESPONSE_NO_SAVED : SERIAL_RESPONSE_STORAGE_ERROR;
    }
    /* Capture the completed revision before releasing the gate; a later PID
     * update must not relabel this request's value with a different revision. */
    parameter_profile_t completed;
    active_profile(&completed, &revision);
    balance_task_end_maintenance();
    set_working(false);
    respond_snapshot(command, status, value, &completed, revision);
}
static void worker(void *argument) {
    (void)argument;
    if (ensure_storage()) {
        parameter_profile_t loaded;
        parameter_profile_defaults(&loaded, &defaults);
        parameter_store_result_t result = parameter_store_load(&store, &loaded);
        remember_store_error(result);
        if (result == PARAMETER_STORE_OK) {
            remember_saved(&loaded);
            serial_response_status_t applied = apply_profile(&loaded);
            if (applied != SERIAL_RESPONSE_OK) {
                taskENTER_CRITICAL(); last_error = (uint16_t)(0x200U + applied); taskEXIT_CRITICAL();
            }
        }
    }
    balance_task_end_maintenance();
    set_working(false);
    for (;;) {
        parameter_command_t command;
        if (xQueueReceive(requests, &command, portMAX_DELAY) == pdTRUE) handle_job(&command);
    }
}
bool parameter_service_submit(const parameter_command_t *command, void *context) {
    (void)context;
    if (!started || !command) return false;
    if (command->version != SERIAL_PROTOCOL_VERSION) { respond(command, SERIAL_RESPONSE_INVALID, 0); return true; }
    if (command->operation == PARAMETER_OP_GET) {
        parameter_profile_t profile;
        uint32_t revision;
        active_profile(&profile, &revision);
        float value = 0;
        parameter_result_t result = parameter_profile_get(&profile, command->key, &value);
        respond_snapshot(command, result == PARAMETER_OK ? SERIAL_RESPONSE_OK : SERIAL_RESPONSE_UNSUPPORTED,
                          value, &profile, revision);
        return true;
    }
    if (command->operation == PARAMETER_OP_STATUS) {
        if (command->key != 0U) { respond(command, SERIAL_RESPONSE_INVALID, 0); return true; }
        taskENTER_CRITICAL(); uint16_t error = last_error; taskEXIT_CRITICAL();
        respond(command, SERIAL_RESPONSE_OK, (float)error);
        return true;
    }
    if (command->operation < PARAMETER_OP_SET || command->operation > PARAMETER_OP_DEFAULTS) {
        respond(command, SERIAL_RESPONSE_UNSUPPORTED, 0); return true;
    }
    if (command->operation != PARAMETER_OP_SET && command->key != 0U) {
        respond(command, SERIAL_RESPONSE_INVALID, 0); return true;
    }
    return xQueueSend(requests, command, 0U) == pdPASS;
}
bool parameter_service_start(const balance_controller_config_t *board_defaults) {
    if (started || !board_defaults || xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) return false;
    if (!balance_task_begin_maintenance()) return false;
    defaults = *board_defaults;
    configured = ready = saved_valid = false;
    working = true; last_error = 0; next_cookie = 0;
    if (!parameter_store_init(&store, storage_read, storage_write, NULL, &defaults)) goto failed;
    requests = xQueueCreate(4U, sizeof(parameter_command_t));
    completions = xQueueCreate(1U, sizeof(apply_result_t));
    if (!requests || !completions) goto failed;
    started = true;
    if (xTaskCreate(worker, "Parameters", 768U, NULL, 1U, &worker_task) == pdPASS) return true;
failed:
    parameter_service_stop();
    return false;
}
void parameter_service_stop(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) return;
    if (worker_task) vTaskDelete(worker_task);
    if (working) balance_task_end_maintenance();
    if (requests) vQueueDelete(requests);
    if (completions) vQueueDelete(completions);
    worker_task = NULL; requests = completions = NULL; started = working = false;
}
