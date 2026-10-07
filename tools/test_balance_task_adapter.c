/* Real RTOS adapter linked normally; deterministic queue, clock and motor ports. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "balance_task.h"
#include "app_clock.h"

struct test_queue { size_t item_size; unsigned count; unsigned char items[64][128]; };
static struct test_queue attitude_queue, system_queue, motion_queue, pid_queue;
static uint32_t now_ms, encoder_delay_ms;
static int critical_depth;
static int16_t motor_left, motor_right;
static unsigned telemetry_count, calibration_count;
static bool inject_stop, calibration_ok = true, creation_ok = true;
static float encoder_left, encoder_right;
static balance_runtime_snapshot_t last_snapshot;
static unsigned config_completions, pid_completions;
static uint32_t last_cookie, last_revision;
static balance_config_result_t last_config_result, last_pid_result;

void test_enter_critical(void) { ++critical_depth; }
void test_exit_critical(void) { assert(critical_depth > 0); --critical_depth; }
uint32_t app_clock_now_ms(void) { return now_ms; }
TickType_t app_clock_period_ticks(uint32_t ms) { return ms > 0U ? ms : 1U; }
TickType_t xTaskGetTickCount(void) { return now_ms; }
void vTaskDelayUntil(TickType_t *previous, TickType_t increment) { *previous += increment; now_ms = *previous; }
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, uint16_t stack,
                       void *argument, UBaseType_t priority, TaskHandle_t *handle) {
    (void)entry; (void)name; (void)argument;
    assert(stack > 0U && priority > 0U);
    *handle = (TaskHandle_t)1;
    return creation_ok ? pdPASS : pdFAIL;
}
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue) { return queue->count; }
BaseType_t xQueueReset(QueueHandle_t queue) {
    assert(critical_depth > 0);
    queue->count = 0U;
    return pdPASS;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait) {
    assert(wait == 0U && critical_depth == 0);
    if (queue->count == 0U) return pdFALSE;
    memcpy(out, queue->items[0], queue->item_size);
    --queue->count;
    memmove(queue->items[0], queue->items[1], queue->count * sizeof(queue->items[0]));
    return pdTRUE;
}
static void push(struct test_queue *queue, const void *item) {
    assert(queue->count < 64U && queue->item_size <= 128U);
    memcpy(queue->items[queue->count++], item, queue->item_size);
}
static void motor_hook(int16_t left, int16_t right) {
    assert(critical_depth > 0);
    motor_left = left; motor_right = right;
}
static void encoder_hook(float *left, float *right) {
    assert(critical_depth == 0);
    now_ms += encoder_delay_ms;
    *left = encoder_left; *right = encoder_right;
    if (inject_stop) {
        inject_stop = false;
        balance_task_emergency_stop();
        assert(motor_left == 0 && motor_right == 0);
    }
}
static void telemetry_hook(const balance_runtime_snapshot_t *snapshot) {
    assert(critical_depth == 0);
    last_snapshot = *snapshot;
    ++telemetry_count;
}
static bool calibration_hook(void) { assert(critical_depth == 0); ++calibration_count; return calibration_ok; }
static void config_completed(balance_config_result_t result, uint32_t cookie, uint32_t revision, void *context) {
    assert(critical_depth == 0 && context == &config_completions);
    last_config_result = result; last_cookie = cookie; last_revision = revision;
    ++config_completions;
    assert(balance_task_get_config_revision() == revision); /* Published before completion. */
}
static void pid_completed(const pid_request_t *request, balance_config_result_t result, uint32_t revision) {
    assert(critical_depth == 0 && request->token == 17U);
    last_pid_result = result; ++pid_completions;
    assert(balance_task_get_config_revision() == revision);
}
static balance_task_config_t configuration(void) {
    attitude_queue.item_size = sizeof(attitude_sample_t);
    system_queue.item_size = sizeof(system_request_t);
    motion_queue.item_size = sizeof(motion_request_t);
    pid_queue.item_size = sizeof(pid_request_t);
    balance_task_config_t config;
    balance_task_default_config(&config);
    config.attitude_queue = &attitude_queue;
    config.system_command_queue = &system_queue;
    config.motion_command_queue = &motion_queue;
    config.pid_config_queue = &pid_queue;
    config.pid_result_hook = pid_completed;
    config.motor_output_hook = motor_hook;
    config.encoder_read_hook = encoder_hook;
    config.calibration_request_hook = calibration_hook;
    config.telemetry_hook = telemetry_hook;
    config.enable_telemetry = true;
    config.telemetry_divisor = 1U;
    return config;
}
static void action(system_action_t action) {
    const system_request_t request = {.action = action};
    push(&system_queue, &request);
}
#define HEALTHY (ATTITUDE_STATUS_CALIBRATED | ATTITUDE_STATUS_ATTITUDE_VALID)
static void tick(uint16_t status, uint32_t interval) {
    now_ms += interval;
    const attitude_sample_t sample = {.timestamp_ms = now_ms, .status_flags = status, .pitch_deg = 1.0f};
    push(&attitude_queue, &sample);
    balance_task_run_cycle();
    assert(critical_depth == 0);
}
static void qualify(void) { for (unsigned n = 0; n < 41U; ++n) tick(HEALTHY, 5); }
static void arm(void) {
    qualify(); action(SYSTEM_ACTION_ARM); tick(HEALTHY, 5);
    assert(balance_task_get_state() == BALANCE_STATE_ARMED);
    assert(motor_left != 0 || motor_right != 0);
}
static balance_config_result_t request_config(const balance_controller_config_t *config, uint32_t cookie) {
    return balance_task_request_config(config, cookie, config_completed, &config_completions);
}
static void test_tuning(void) {
    balance_controller_t controller;
    balance_task_get_controller(&controller);
    const float original_kp = controller.config.balance_loop.kp;
    balance_controller_config_t candidate = controller.config;
    candidate.balance_loop.kp = 44.0f;
    assert(request_config(&candidate, 100U) == BALANCE_CONFIG_OK);
    tick(HEALTHY, 5);
    assert(config_completions == 1U && last_cookie == 100U && last_revision == 0U);
    assert(last_config_result == BALANCE_CONFIG_UNSAFE);
    balance_task_get_controller(&controller);
    assert(controller.config.balance_loop.kp == original_kp);
    const pid_request_t pid = {.loop_id = 0, .kp = 55, .kd = 0.5f, .token = 17U};
    push(&pid_queue, &pid); tick(HEALTHY, 5);
    assert(pid_completions == 1U && last_pid_result == BALANCE_CONFIG_UNSAFE);
    balance_task_request_disarm(); tick(HEALTHY, 5);
    candidate.velocity_loop.lpf_alpha = NAN;
    assert(request_config(&candidate, 101U) == BALANCE_CONFIG_INVALID);
    candidate.velocity_loop.lpf_alpha = 0.75f;
    assert(request_config(&candidate, 102U) == BALANCE_CONFIG_OK);
    candidate.balance_loop.kp = 99.0f; /* Caller storage must have been copied. */
    assert(request_config(&candidate, 103U) == BALANCE_CONFIG_BUSY);
    assert(balance_task_get_config_revision() == 0U);
    push(&pid_queue, &pid); tick(HEALTHY, 5);
    assert(last_config_result == BALANCE_CONFIG_OK && last_cookie == 102U && last_revision == 1U);
    assert(last_pid_result == BALANCE_CONFIG_BUSY);
    balance_task_get_controller(&controller);
    assert(controller.config.balance_loop.kp == 44.0f && !controller.state.is_enabled);
    push(&pid_queue, &pid); tick(HEALTHY, 5);
    assert(last_pid_result == BALANCE_CONFIG_OK && balance_task_get_config_revision() == 2U);
    balance_task_get_controller(&controller);
    assert(controller.config.balance_loop.kp == 55.0f);
    pid_request_t invalid = pid; invalid.integral_limit = -1.0f;
    push(&pid_queue, &invalid); tick(HEALTHY, 5);
    assert(last_pid_result == BALANCE_CONFIG_INVALID && balance_task_get_config_revision() == 2U);
    assert(balance_task_begin_maintenance());
    assert(balance_task_is_maintenance() && !balance_task_begin_maintenance());
    assert(!balance_task_request_arm());
    action(SYSTEM_ACTION_ARM);
    push(&pid_queue, &pid);
    tick(HEALTHY, 5);
    assert(balance_task_get_state() == BALANCE_STATE_DISARMED && last_pid_result == BALANCE_CONFIG_BUSY);
    /* Full configuration application is allowed for the lock-owning worker. */
    assert(request_config(&candidate, 103U) == BALANCE_CONFIG_OK);
    tick(HEALTHY, 5);
    assert(last_config_result == BALANCE_CONFIG_OK && last_revision == 3U);
    /* ARM/enable/legacy PID queued just before release cannot become deferred
     * commands after the maintenance gate disappears. */
    action(SYSTEM_ACTION_ARM);
    const motion_request_t deferred_motion = {.enable = true, .linear_counts_per_5ms = 5};
    push(&motion_queue, &deferred_motion);
    push(&pid_queue, &pid); push(&pid_queue, &pid);
    balance_task_end_maintenance();
    assert(!balance_task_is_maintenance());
    assert(system_queue.count == 0U && motion_queue.count == 0U);
    tick(HEALTHY, 5); tick(HEALTHY, 5);
    assert(balance_task_get_state() == BALANCE_STATE_DISARMED && last_pid_result == BALANCE_CONFIG_BUSY);
    assert(balance_task_get_config_revision() == 3U);
    /* Same-cycle ARM wins over a pending tuning request. */
    assert(request_config(&candidate, 104U) == BALANCE_CONFIG_OK);
    action(SYSTEM_ACTION_ARM); tick(HEALTHY, 5);
    assert(last_config_result == BALANCE_CONFIG_UNSAFE && last_revision == 3U);
    assert(balance_task_get_state() == BALANCE_STATE_ARMED);
    assert(!balance_task_begin_maintenance());
    balance_task_emergency_stop(); tick(HEALTHY, 5);
    assert(request_config(&candidate, 105U) == BALANCE_CONFIG_OK);
    tick(HEALTHY, 5);
    assert(last_config_result == BALANCE_CONFIG_UNSAFE);
    action(SYSTEM_ACTION_RESET); tick(HEALTHY, 5);
    action(SYSTEM_ACTION_CALIBRATE); tick(HEALTHY, 5);
    assert(request_config(&candidate, 106U) == BALANCE_CONFIG_OK);
    tick(ATTITUDE_STATUS_CALIBRATING, 5);
    assert(last_config_result == BALANCE_CONFIG_UNSAFE);
    qualify(); arm();
}
int main(int argc, char **argv) {
    assert(argc == 2);
    balance_task_config_t config = configuration();
    if (strcmp(argv[1], "invalid_config") == 0) {
        config.motor_output_hook = NULL;
        assert(balance_task_start(&config, NULL) == pdFAIL);
        assert(!balance_task_request_arm());
        return 0;
    }
    if (strcmp(argv[1], "create_failure") == 0) {
        creation_ok = false;
        assert(balance_task_start(&config, NULL) == pdFAIL);
        assert(motor_left == 0 && motor_right == 0 && !balance_task_request_arm());
        creation_ok = true;
    }
    assert(balance_task_start(&config, NULL) == pdPASS);
    assert(balance_task_start(&config, NULL) == pdFAIL);
    assert(motor_left == 0 && motor_right == 0);
    arm();
    if (strcmp(argv[1], "normal") == 0 || strcmp(argv[1], "create_failure") == 0) {
        test_tuning();
        encoder_left = 20; encoder_right = -20;
        tick(HEALTHY, 10);
        assert(last_snapshot.left_counts_per_5ms == 10 && last_snapshot.right_counts_per_5ms == -10);
        assert(telemetry_count > 0);
        balance_task_request_disarm();
        assert(motor_left == 0 && motor_right == 0);
        assert(balance_task_request_arm());
        tick(HEALTHY, 5);
        assert(balance_task_get_state() == BALANCE_STATE_DISARMED);
        balance_task_set_motor_output_hook(NULL);
        action(SYSTEM_ACTION_ARM); tick(HEALTHY, 5);
        assert(balance_task_get_state() == BALANCE_STATE_ARMED);
        balance_task_set_encoder_read_hook(encoder_hook);
        assert(motor_left == 0 && motor_right == 0);
        tick(HEALTHY, 5);
        assert(balance_task_get_state() == BALANCE_STATE_DISARMED);
    } else if (strcmp(argv[1], "preemption") == 0) {
        inject_stop = true; tick(HEALTHY, 5);
        assert(motor_left == 0 && motor_right == 0 && last_snapshot.outputs.safety_active);
        tick(HEALTHY, 5);
        assert(balance_task_get_state() == BALANCE_STATE_FALLEN);
        assert(last_snapshot.fault_flags & SAFETY_FAULT_EMERGENCY_STOP);
    } else if (strcmp(argv[1], "deadline") == 0) {
        encoder_delay_ms = 25; tick(HEALTHY, 5);
        assert(motor_left == 0 && motor_right == 0);
        assert(balance_task_get_state() == BALANCE_STATE_FALLEN);
        assert(last_snapshot.fault_flags & SAFETY_FAULT_SENSOR_INVALID);
    } else if (strcmp(argv[1], "calibration") == 0) {
        action(SYSTEM_ACTION_CALIBRATE); tick(HEALTHY, 5);
        assert(calibration_count == 1U && balance_task_get_state() == BALANCE_STATE_CALIBRATING);
        assert(motor_left == 0 && motor_right == 0);
        tick(ATTITUDE_STATUS_CALIBRATING, 5); qualify();
        action(SYSTEM_ACTION_ARM); tick(HEALTHY, 5);
        assert(balance_task_get_state() == BALANCE_STATE_ARMED);
        calibration_ok = false; action(SYSTEM_ACTION_CALIBRATE); tick(HEALTHY, 5);
        assert(balance_task_get_state() == BALANCE_STATE_DISARMED);
        assert(last_snapshot.fault_flags & SAFETY_FAULT_CALIBRATION_FAIL);
    } else assert(!"Unknown adapter scenario");
    assert(critical_depth == 0);
    puts("PASS: FreeRTOS balance adapter scenario");
    return 0;
}
