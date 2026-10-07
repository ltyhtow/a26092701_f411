#include "wheel_test_task.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "board_diagnostics.h"
#include "serial_protocol.h"
#include "app_clock.h"
#include <math.h>
#include <string.h>

static wheel_test_t test;
static bool initialized, stop_before_start, sample_valid;
static uint32_t previous_sample_ms, dropped, errors;
static uint8_t sequence;

static void stop_if_terminal(void) {
    if (wheel_test_terminal(&test)) {
        motor_driver_emergency_stop();
        test.snapshot.flags |= WHEEL_TEST_FLAG_MOTOR_LATCHED;
    }
}

bool wheel_test_task_command(uint8_t action) {
    taskENTER_CRITICAL();
    bool accepted = false;
    if (!initialized) {
        if (action == WHEEL_TEST_STOP) {
            stop_before_start = true;
            motor_driver_emergency_stop();
            accepted = true;
        }
    } else {
        const uint32_t now = app_clock_now_ms();
        if (motor_driver_fault_latched()) wheel_test_abort(&test, WHEEL_TEST_REASON_DRIVER_FAULT, now);
        accepted = wheel_test_command(&test, action, now);
        stop_if_terminal();
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void wheel_test_task_get_snapshot(wheel_test_snapshot_t *snapshot) {
    if (snapshot == NULL) return;
    taskENTER_CRITICAL();
    *snapshot = test.snapshot;
    taskEXIT_CRITICAL();
}

static void count_drop(serial_protocol_result_t result) {
    if (result != SERIAL_PROTOCOL_OK && dropped != UINT32_MAX) ++dropped;
}

void wheel_test_task_run_cycle(void) {
    if (!initialized) return;
    const uint32_t cycle_ms = app_clock_now_ms();
    taskENTER_CRITICAL();
    const bool stage_change = test.started && !wheel_test_terminal(&test) &&
        wheel_test_phase_at_elapsed(cycle_ms - test.start_ms) != test.snapshot.phase;
    taskEXIT_CRITICAL();
    const bool report = stage_change || cycle_ms - previous_sample_ms >= WHEEL_TEST_SAMPLE_MS;
    encoder_driver_snapshot_t sample = {0};
    encoder_test_telemetry_t encoder = {0};
    if (report) {
        float left, right;
        encoder_driver_read_speed(&left, &right);
        encoder_driver_get_snapshot(&sample);
        sample_valid = sample.valid && isfinite(left) && isfinite(right);
        if (!sample_valid && errors != UINT32_MAX) ++errors;
        const uint32_t sample_ms = app_clock_now_ms();
        encoder.timestamp_ms = sample_ms;
        encoder.sample_period_ms = sample_ms - previous_sample_ms;
        previous_sample_ms = sample_ms;
        bool invert_left, invert_right;
        encoder_driver_get_polarity(&invert_left, &invert_right);
        encoder.flags = (sample_valid ? ENCODER_TEST_VALID : 0U) |
                        (encoder_driver_is_ready() ? ENCODER_TEST_READY : 0U) |
                        (invert_left ? ENCODER_TEST_INVERT_LEFT : 0U) |
                        (invert_right ? ENCODER_TEST_INVERT_RIGHT : 0U);
        encoder.raw_left = sample.raw_left; encoder.raw_right = sample.raw_right;
        encoder.delta_left = sample.delta_left; encoder.delta_right = sample.delta_right;
        encoder.total_left = sample.total_left; encoder.total_right = sample.total_right;
        encoder.gpio_levels = board_encoder_levels();
        encoder.sample_errors = errors;
    }
    /* Sampling and packet submission never hold this lock. Commands, the final
     * fresh deadline check, and the actuator write share this short lock. */
    taskENTER_CRITICAL();
    const uint32_t now = app_clock_now_ms();
    if (report && !sample_valid) wheel_test_abort(&test, WHEEL_TEST_REASON_SENSOR_ERROR, now);
    wheel_test_step(&test, now, encoder_driver_is_ready(), motor_driver_fault_latched(), sample_valid);
    stop_if_terminal();
    if (!wheel_test_terminal(&test)) {
        motor_driver_set_output(test.snapshot.left_pwm, test.snapshot.right_pwm);
    }
    const wheel_test_snapshot_t state = test.snapshot;
    taskEXIT_CRITICAL();
    if (report) {
        encoder.version = SERIAL_PROTOCOL_VERSION;
        encoder.sequence = sequence;
        encoder.tx_dropped = dropped;
        if ((state.flags & WHEEL_TEST_FLAG_MOTOR_LATCHED) != 0U) encoder.flags |= ENCODER_TEST_MOTOR_LOCKED;
        count_drop(serial_protocol_try_send(SERIAL_CMD_ENCODER_TEST, &encoder, sizeof(encoder)));
        const wheel_test_telemetry_t status = {
            .version = SERIAL_PROTOCOL_VERSION, .sequence = sequence++,
            .phase = (uint8_t)state.phase, .reason = (uint8_t)state.reason,
            .timestamp_ms = state.timestamp_ms, .phase_elapsed_ms = state.phase_elapsed_ms,
            .left_pwm = state.left_pwm, .right_pwm = state.right_pwm,
            .heartbeat_age_ms = state.heartbeat_age_ms, .flags = state.flags,
            .reserved = 0U, .remaining_ms = state.remaining_ms,
            .test_elapsed_ms = state.test_elapsed_ms
        };
        count_drop(serial_protocol_try_send(SERIAL_CMD_WHEEL_TEST, &status, sizeof(status)));
    }
}

static void wheel_test_task_entry(void *argument) {
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, app_clock_period_ticks(WHEEL_TEST_PERIOD_MS));
        wheel_test_task_run_cycle();
        /* Missed releases do not cause a burst of catch-up writes. */
        if (xTaskGetTickCount() - wake >= app_clock_period_ticks(WHEEL_TEST_PERIOD_MS)) wake = xTaskGetTickCount();
    }
}

BaseType_t wheel_test_task_start(TaskHandle_t *task_handle) {
    if (task_handle == NULL || initialized) return pdFAIL;
    motor_driver_init();
    motor_driver_stop(); /* Zero effort also if the board initialized the port earlier. */
    const bool ready = encoder_driver_init();
    taskENTER_CRITICAL();
    previous_sample_ms = app_clock_now_ms();
    wheel_test_init(&test, previous_sample_ms);
    if (stop_before_start) wheel_test_abort(&test, WHEEL_TEST_REASON_USER_STOP, previous_sample_ms);
    wheel_test_step(&test, previous_sample_ms, ready, motor_driver_fault_latched(), false);
    stop_if_terminal();
    initialized = true;
    taskEXIT_CRITICAL();
    const BaseType_t result = xTaskCreate(wheel_test_task_entry, "WheelTest", 512U, NULL, 3U, task_handle);
    if (result != pdPASS) {
        taskENTER_CRITICAL();
        wheel_test_abort(&test, WHEEL_TEST_REASON_DRIVER_FAULT, app_clock_now_ms());
        stop_if_terminal();
        taskEXIT_CRITICAL();
    }
    return result;
}
