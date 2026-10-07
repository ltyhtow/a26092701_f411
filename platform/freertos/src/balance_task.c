/* FreeRTOS owner and port boundary; all control policy lives in balance_runtime. */
#include "balance_task.h"
#include "app_clock.h"
#include <string.h>

static balance_task_config_t s_config;
static balance_runtime_t s_runtime;
static TaskHandle_t s_task_handle;
static bool s_initialized;
static uint32_t s_pending_requests, s_cycle_counter;
static balance_runtime_snapshot_t s_published_snapshot;
static balance_controller_t s_published_controller;
static safety_fsm_t s_published_fsm;
static uint32_t s_config_revision;
static bool s_maintenance;
static UBaseType_t s_rejected_pid_count;
static struct {
    balance_controller_config_t config;
    balance_config_result_fn callback;
    void *context;
    uint32_t cookie;
    bool pending;
} s_config_request;

static void dispatch_motor(int16_t left, int16_t right) {
    if (s_config.motor_output_hook != NULL) s_config.motor_output_hook(left, right);
}
static UBaseType_t queued_count(QueueHandle_t queue) {
    return queue == NULL ? 0U : uxQueueMessagesWaiting(queue);
}
void balance_task_run_cycle(void) {
    if (!s_initialized) return;
    balance_runtime_begin_cycle(&s_runtime, app_clock_now_ms());
    balance_encoder_read_fn encoder_hook;
    taskENTER_CRITICAL();
    balance_runtime_request_flags(&s_runtime, s_pending_requests | (s_maintenance ? BALANCE_REQUEST_DISARM : 0U));
    s_pending_requests = 0U;
    encoder_hook = s_config.encoder_read_hook;
    taskEXIT_CRITICAL();
    attitude_sample_t attitude;
    for (UBaseType_t n = queued_count(s_config.attitude_queue); n > 0U; --n) {
        if (xQueueReceive(s_config.attitude_queue, &attitude, 0U) != pdTRUE) break;
        balance_runtime_accept_attitude(&s_runtime, &attitude);
    }
    system_request_t system;
    for (UBaseType_t n = queued_count(s_config.system_command_queue); n > 0U; --n) {
        if (xQueueReceive(s_config.system_command_queue, &system, 0U) != pdTRUE) break;
        balance_runtime_request_system(&s_runtime, &system);
    }
    motion_request_t motion;
    for (UBaseType_t n = queued_count(s_config.motion_command_queue); n > 0U; --n) {
        if (xQueueReceive(s_config.motion_command_queue, &motion, 0U) != pdTRUE) break;
        balance_runtime_accept_motion(&s_runtime, &motion);
    }
    float left_counts = 0.0f, right_counts = 0.0f;
    if (encoder_hook != NULL) encoder_hook(&left_counts, &right_counts);
    balance_runtime_finish_cycle(&s_runtime, left_counts, right_counts);
    if (s_runtime.snapshot.calibration_request) {
        const bool accepted = s_config.calibration_request_hook != NULL && s_config.calibration_request_hook();
        balance_runtime_calibration_result(&s_runtime, accepted);
    }
    /* Apply after system/motion actions: an ARM or calibration in this same
     * cycle must not race a tuning update. Handle one legacy PID per cycle to
     * bound work and keep each completion tied to one configuration revision. */
    pid_request_t pid;
    const bool have_pid = s_config.pid_config_queue != NULL &&
        xQueueReceive(s_config.pid_config_queue, &pid, 0U) == pdTRUE;
    balance_config_result_t pid_result = BALANCE_CONFIG_UNSAFE;
    balance_config_result_t config_result = BALANCE_CONFIG_UNSAFE;
    balance_config_result_fn config_callback = NULL;
    void *config_context = NULL;
    uint32_t config_cookie = 0U, config_revision = 0U, pid_revision = 0U;
    balance_telemetry_fn telemetry_hook;
    bool telemetry;
    uint8_t divisor;
    /* Time is re-read here: a hook or preemption may consume the remaining
     * sensor/control budget. A concurrent stop wins over this PWM write. */
    taskENTER_CRITICAL();
    balance_runtime_prepare_output(&s_runtime, app_clock_now_ms(), s_pending_requests);
    const bool safe_config = s_runtime.snapshot.state == BALANCE_STATE_DISARMED &&
        (s_pending_requests & (BALANCE_REQUEST_ARM | BALANCE_REQUEST_ESTOP | BALANCE_REQUEST_CALIBRATE)) == 0U;
    const bool have_config = s_config_request.pending;
    if (have_config) {
        config_callback = s_config_request.callback;
        config_context = s_config_request.context;
        config_cookie = s_config_request.cookie;
        if (safe_config) {
            config_result = balance_runtime_apply_config(&s_runtime, &s_config_request.config) ?
                BALANCE_CONFIG_OK : BALANCE_CONFIG_INVALID;
            if (config_result == BALANCE_CONFIG_OK) ++s_config_revision;
        }
        config_revision = s_config_revision;
        s_config_request.pending = false;
    }
    if (have_pid) {
        /* A full profile and a legacy PID request must never silently overwrite
         * each other in one cycle. The host can retry the rejected legacy PID. */
        if (have_config || s_maintenance || s_rejected_pid_count > 0U) pid_result = BALANCE_CONFIG_BUSY;
        else if (safe_config) {
            pid_result = balance_runtime_accept_pid(&s_runtime, &pid) ? BALANCE_CONFIG_OK : BALANCE_CONFIG_INVALID;
            if (pid_result == BALANCE_CONFIG_OK) ++s_config_revision;
        }
        pid_revision = s_config_revision;
        if (s_rejected_pid_count > 0U) --s_rejected_pid_count;
    }
    dispatch_motor(s_runtime.snapshot.outputs.left_pwm, s_runtime.snapshot.outputs.right_pwm);
    s_published_snapshot = s_runtime.snapshot;
    s_published_controller = s_runtime.controller;
    s_published_fsm = s_runtime.safety;
    telemetry = s_config.enable_telemetry;
    divisor = s_config.telemetry_divisor;
    telemetry_hook = s_config.telemetry_hook;
    taskEXIT_CRITICAL();
    if (config_callback != NULL) config_callback(config_result, config_cookie, config_revision, config_context);
    if (have_pid && s_config.pid_result_hook != NULL) s_config.pid_result_hook(&pid, pid_result, pid_revision);
    if (telemetry && telemetry_hook != NULL && divisor > 0U && s_cycle_counter % divisor == 0U)
        telemetry_hook(&s_runtime.snapshot);
    ++s_cycle_counter;
}
static void balance_task_entry(void *argument) {
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = app_clock_period_ticks(BALANCE_TASK_PERIOD_MS);
    for (;;) {
        vTaskDelayUntil(&wake, period);
        const TickType_t current = xTaskGetTickCount();
        if ((TickType_t)(current - wake) >= period) wake = current;
        balance_task_run_cycle();
    }
}
void balance_task_default_config(balance_task_config_t *config) {
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->priority = BALANCE_TASK_DEFAULT_PRIORITY;
    config->stack_depth_words = BALANCE_TASK_STACK_WORDS;
    config->telemetry_divisor = BALANCE_TELEMETRY_DEFAULT_DIVISOR;
}
BaseType_t balance_task_start(const balance_task_config_t *config, TaskHandle_t *task_handle) {
    if (config == NULL || config->attitude_queue == NULL || config->motor_output_hook == NULL || s_initialized) return pdFAIL;
    s_config = *config;
    if (s_config.priority == 0U) s_config.priority = BALANCE_TASK_DEFAULT_PRIORITY;
    if (s_config.stack_depth_words == 0U) s_config.stack_depth_words = BALANCE_TASK_STACK_WORDS;
    if (s_config.telemetry_divisor == 0U) s_config.telemetry_divisor = BALANCE_TELEMETRY_DEFAULT_DIVISOR;
    balance_runtime_init(&s_runtime, config->controller_config);
    s_config.controller_config = NULL;
    s_pending_requests = s_cycle_counter = 0U;
    s_config_revision = 0U;
    s_maintenance = false;
    s_rejected_pid_count = 0U;
    memset(&s_config_request, 0, sizeof(s_config_request));
    s_published_snapshot = s_runtime.snapshot;
    s_published_controller = s_runtime.controller;
    s_published_fsm = s_runtime.safety;
    taskENTER_CRITICAL();
    dispatch_motor(0, 0);
    s_initialized = true;
    taskEXIT_CRITICAL();
    const BaseType_t result = xTaskCreate(balance_task_entry, "BalanceTask", s_config.stack_depth_words,
                                        NULL, (UBaseType_t)s_config.priority, &s_task_handle);
    if (result != pdPASS) {
        taskENTER_CRITICAL();
        s_initialized = false;
        dispatch_motor(0, 0);
        taskEXIT_CRITICAL();
    } else if (task_handle != NULL) *task_handle = s_task_handle;
    return result;
}
BaseType_t balance_task_init_and_start(QueueHandle_t attitude, QueueHandle_t pid, QueueHandle_t motion,
    balance_motor_output_fn hook, TaskHandle_t *task_handle) {
    balance_task_config_t config;
    balance_task_default_config(&config);
    config.attitude_queue = attitude;
    config.pid_config_queue = pid;
    config.motion_command_queue = motion;
    config.motor_output_hook = hook;
    return balance_task_start(&config, task_handle);
}
void balance_task_set_motor_output_hook(balance_motor_output_fn hook) {
    /* A missing output is never accepted: retain the working cutoff path. */
    if (hook == NULL) return;
    taskENTER_CRITICAL();
    dispatch_motor(0, 0);
    s_config.motor_output_hook = hook;
    s_pending_requests |= BALANCE_REQUEST_DISARM;
    dispatch_motor(0, 0);
    taskEXIT_CRITICAL();
}
void balance_task_set_encoder_read_hook(balance_encoder_read_fn hook) {
    taskENTER_CRITICAL();
    s_config.encoder_read_hook = hook;
    s_pending_requests |= BALANCE_REQUEST_DISARM;
    dispatch_motor(0, 0);
    taskEXIT_CRITICAL();
}
static bool post_request(uint32_t request) {
    taskENTER_CRITICAL();
    const bool accepted = s_initialized && !(s_maintenance && (request & BALANCE_REQUEST_ARM) != 0U);
    if (accepted) {
        s_pending_requests |= request;
        if ((request & BALANCE_REQUEST_STOP_MASK) != 0U) dispatch_motor(0, 0);
    }
    taskEXIT_CRITICAL();
    return accepted;
}
bool balance_task_request_arm(void) { return post_request(BALANCE_REQUEST_ARM); }
void balance_task_request_disarm(void) { (void)post_request(BALANCE_REQUEST_DISARM); }
bool balance_task_reset_fault(void) { return post_request(BALANCE_REQUEST_RESET); }
void balance_task_emergency_stop(void) { (void)post_request(BALANCE_REQUEST_ESTOP); }
balance_state_t balance_task_get_state(void) {
    taskENTER_CRITICAL();
    const balance_state_t state = s_published_snapshot.state;
    taskEXIT_CRITICAL();
    return state;
}
void balance_task_get_controller(balance_controller_t *controller) {
    if (controller == NULL) return;
    taskENTER_CRITICAL();
    *controller = s_published_controller;
    taskEXIT_CRITICAL();
}
void balance_task_get_safety_fsm(safety_fsm_t *fsm) {
    if (fsm == NULL) return;
    taskENTER_CRITICAL();
    *fsm = s_published_fsm;
    taskEXIT_CRITICAL();
}
void balance_task_get_latest_outputs(balance_controller_outputs_t *outputs) {
    if (outputs == NULL) return;
    taskENTER_CRITICAL();
    *outputs = s_published_snapshot.outputs;
    taskEXIT_CRITICAL();
}
void balance_task_set_telemetry_enabled(bool enable, uint8_t divisor) {
    taskENTER_CRITICAL();
    s_config.enable_telemetry = enable;
    s_config.telemetry_divisor = divisor > 0U ? divisor : BALANCE_TELEMETRY_DEFAULT_DIVISOR;
    taskEXIT_CRITICAL();
}
balance_config_result_t balance_task_request_config(const balance_controller_config_t *config,
    uint32_t cookie, balance_config_result_fn callback, void *context) {
    if (callback == NULL || !balance_controller_config_is_valid(config)) return BALANCE_CONFIG_INVALID;
    taskENTER_CRITICAL();
    balance_config_result_t result = BALANCE_CONFIG_OK;
    if (!s_initialized) result = BALANCE_CONFIG_NOT_READY;
    else if (s_config_request.pending) result = BALANCE_CONFIG_BUSY;
    else {
        s_config_request.config = *config;
        s_config_request.callback = callback;
        s_config_request.context = context;
        s_config_request.cookie = cookie;
        s_config_request.pending = true;
    }
    taskEXIT_CRITICAL();
    return result;
}
uint32_t balance_task_get_config_revision(void) {
    taskENTER_CRITICAL();
    const uint32_t revision = s_config_revision;
    taskEXIT_CRITICAL();
    return revision;
}
void balance_task_get_config_snapshot(balance_controller_config_t *config, uint32_t *revision) {
    taskENTER_CRITICAL();
    if (config != NULL) *config = s_published_controller.config;
    if (revision != NULL) *revision = s_config_revision;
    taskEXIT_CRITICAL();
}
void balance_task_get_snapshot(balance_runtime_snapshot_t *snapshot) {
    if (snapshot == NULL) return;
    taskENTER_CRITICAL();
    *snapshot = s_published_snapshot;
    taskEXIT_CRITICAL();
}
bool balance_task_begin_maintenance(void) {
    taskENTER_CRITICAL();
    const bool accepted = s_initialized && !s_maintenance && !s_config_request.pending &&
        s_published_snapshot.state == BALANCE_STATE_DISARMED &&
        (s_pending_requests & (BALANCE_REQUEST_ARM | BALANCE_REQUEST_ESTOP | BALANCE_REQUEST_CALIBRATE)) == 0U;
    if (accepted) {
        s_maintenance = true;
        s_pending_requests |= BALANCE_REQUEST_DISARM;
        dispatch_motor(0, 0);
    }
    taskEXIT_CRITICAL();
    return accepted;
}
void balance_task_end_maintenance(void) {
    taskENTER_CRITICAL();
    if (s_maintenance) {
        /* Drain commands received during maintenance rather than letting an
         * old ARM/enable take effect after the lock is released. Immediate
         * disarm requests survive this flush in s_pending_requests. */
        if (s_config.system_command_queue != NULL) (void)xQueueReset(s_config.system_command_queue);
        if (s_config.motion_command_queue != NULL) (void)xQueueReset(s_config.motion_command_queue);
        /* Legacy PID keeps its FIFO completion, but all entries already queued
         * at exit must receive BUSY even if consumed in a later cycle. */
        s_rejected_pid_count = queued_count(s_config.pid_config_queue);
        s_maintenance = false;
        s_pending_requests |= BALANCE_REQUEST_DISARM;
        dispatch_motor(0, 0);
    }
    taskEXIT_CRITICAL();
}
bool balance_task_is_maintenance(void) {
    taskENTER_CRITICAL();
    const bool active = s_maintenance;
    taskEXIT_CRITICAL();
    return active;
}
