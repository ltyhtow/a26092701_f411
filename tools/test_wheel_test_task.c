/* Link production task and sequencer normally; only replace platform contracts. */
#include "wheel_test_task.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "serial_protocol.h"
#include "app_clock.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t now_ms, sample_delay;
static unsigned critical, motor_inits, encoder_inits, output_calls, sample_count, encoder_frames, status_frames;
static bool latch, encoder_ready = true, create_ok = true, valid = true, nan_sample, stop_in_sample;
static bool transport_full, heartbeat_during_sample;
static int16_t output_left, output_right;
static encoder_test_telemetry_t encoder_frame;
static wheel_test_telemetry_t status_frame;
static TaskFunction_t captured_entry;
void test_enter_critical(void) { ++critical; }
void test_exit_critical(void) { assert(critical > 0U); --critical; }
uint32_t app_clock_now_ms(void) { return now_ms; }
TickType_t app_clock_period_ticks(uint32_t ms) { return ms; }
TickType_t xTaskGetTickCount(void) { return now_ms; }
void vTaskDelayUntil(TickType_t *wake, TickType_t interval) { *wake += interval; now_ms = *wake; }
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, unsigned stack,
                      void *argument, UBaseType_t priority, TaskHandle_t *handle) {
    assert(critical == 0U && motor_inits == 1U && encoder_inits == 1U);
    assert(strcmp(name, "WheelTest") == 0 && stack >= 512U && priority == 3U && argument == NULL);
    captured_entry = entry;
    *handle = (void *)1;
    return create_ok ? pdPASS : pdFAIL;
}
void motor_driver_init(void) { ++motor_inits; output_left = output_right = 0; }
void motor_driver_stop(void) { output_left = output_right = 0; }
void motor_driver_set_output(int16_t left, int16_t right) {
    assert(critical > 0U && !latch);
    output_left = left; output_right = right; ++output_calls;
}
void motor_driver_emergency_stop(void) { assert(critical > 0U); latch = true; output_left = output_right = 0; }
bool motor_driver_fault_latched(void) { return latch; }
bool encoder_driver_init(void) { ++encoder_inits; return encoder_ready; }
bool encoder_driver_is_ready(void) { return encoder_ready; }
void encoder_driver_get_polarity(bool *left, bool *right) { *left = false; *right = true; }
void encoder_driver_read_speed(float *left, float *right) {
    assert(critical == 0U);
    ++sample_count;
    now_ms += sample_delay;
    *left = nan_sample ? NAN : 0.0f; *right = 0.0f; /* No pulses must remain observable, not fatal. */
    if (stop_in_sample) {
        stop_in_sample = false;
        assert(wheel_test_task_command(WHEEL_TEST_STOP));
        assert(latch && output_left == 0 && output_right == 0);
    }
    if (heartbeat_during_sample) {
        heartbeat_during_sample = false;
        assert(!wheel_test_task_command(WHEEL_TEST_HEARTBEAT));
    }
}
void encoder_driver_get_snapshot(encoder_driver_snapshot_t *snapshot) {
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->valid = valid;
}
uint32_t board_encoder_levels(void) { return 5U; }
serial_protocol_result_t serial_protocol_try_send(uint32_t command, const void *data, size_t size) {
    assert(critical == 0U);
    if (command == SERIAL_CMD_ENCODER_TEST) {
        assert(size == sizeof(encoder_frame)); memcpy(&encoder_frame, data, size); ++encoder_frames;
    } else {
        assert(command == SERIAL_CMD_WHEEL_TEST && size == sizeof(status_frame));
        memcpy(&status_frame, data, size); ++status_frames;
    }
    return transport_full ? SERIAL_PROTOCOL_TX_FULL : SERIAL_PROTOCOL_OK;
}
static wheel_test_snapshot_t snapshot(void) {
    wheel_test_snapshot_t state;
    wheel_test_task_get_snapshot(&state);
    assert(critical == 0U);
    return state;
}
static void tick(uint32_t increment) {
    now_ms += increment;
    wheel_test_task_run_cycle();
    assert(critical == 0U);
}
static void advance(uint32_t ms, bool heartbeat) {
    for (uint32_t elapsed = 0; elapsed < ms; elapsed += 10) {
        if (heartbeat && elapsed % 200U == 0U) assert(wheel_test_task_command(WHEEL_TEST_HEARTBEAT));
        tick(10U);
    }
}
static void begin(void) {
    assert(!wheel_test_task_command(WHEEL_TEST_START));
    advance(40U, false);
    assert(!wheel_test_task_command(WHEEL_TEST_START));
    tick(10U);
    assert(sample_count == 1U && status_frame.phase == WHEEL_TEST_WAIT);
    assert(encoder_frame.flags == (ENCODER_TEST_VALID | ENCODER_TEST_READY | ENCODER_TEST_INVERT_RIGHT));
    assert(wheel_test_task_command(WHEEL_TEST_START));
    assert(!wheel_test_task_command(WHEEL_TEST_START));
    advance(5500U, true);
    assert(output_left == 1200 && output_right == 0 && !latch);
}
static void verify_stopped(wheel_test_reason_t reason) {
    assert(latch && output_left == 0 && output_right == 0);
    assert(snapshot().phase == WHEEL_TEST_ABORTED && snapshot().reason == reason);
    const unsigned prior_outputs = output_calls;
    assert(!wheel_test_task_command(WHEEL_TEST_START));
    assert(!wheel_test_task_command(WHEEL_TEST_HEARTBEAT));
    advance(100U, false);
    assert(output_calls == prior_outputs && output_left == 0 && output_right == 0);
    assert(status_frame.reason == reason && (status_frame.flags & WHEEL_TEST_FLAG_MOTOR_LATCHED));
}
int main(int argc, char **argv) {
    assert(argc == 2);
    const char *mode = argv[1];
    if (strcmp(mode, "init_failure") == 0) encoder_ready = false;
    if (strcmp(mode, "driver_failure") == 0) latch = true;
    if (strcmp(mode, "create_failure") == 0) create_ok = false;
    if (strcmp(mode, "wrap") == 0) now_ms = UINT32_MAX - 10000U;
    if (strcmp(mode, "stop_before_start") == 0) assert(wheel_test_task_command(WHEEL_TEST_STOP));
    assert(wheel_test_task_start(NULL) == pdFAIL && motor_inits == 0U);
    TaskHandle_t handle = NULL;
    assert(wheel_test_task_start(&handle) == (create_ok ? pdPASS : pdFAIL));
    assert(wheel_test_task_start(&handle) == pdFAIL);
    assert(captured_entry != NULL && output_left == 0 && output_right == 0);
    if (!encoder_ready) verify_stopped(WHEEL_TEST_REASON_SENSOR_ERROR);
    else if (!create_ok || strcmp(mode, "driver_failure") == 0) verify_stopped(WHEEL_TEST_REASON_DRIVER_FAULT);
    else if (strcmp(mode, "stop_before_start") == 0) verify_stopped(WHEEL_TEST_REASON_USER_STOP);
    else if (strcmp(mode, "invalid_first_sample") == 0) {
        valid = false; advance(50U, false); verify_stopped(WHEEL_TEST_REASON_SENSOR_ERROR);
    } else if (strcmp(mode, "misaligned_start") == 0) {
        advance(50U, false);
        now_ms += 3U; /* Command arrives between 10 ms task releases. */
        assert(wheel_test_task_command(WHEEL_TEST_START));
        now_ms += 7U;
        wheel_test_task_run_cycle();
        advance(5000U, true);
        assert(status_frame.phase == WHEEL_TEST_LEFT_FORWARD);
        assert(status_frame.left_pwm == 50 && status_frame.right_pwm == 0);
        assert(encoder_frame.timestamp_ms == 5060U && encoder_frame.sample_period_ms == 10U);
        assert(wheel_test_task_command(WHEEL_TEST_STOP));
        verify_stopped(WHEEL_TEST_REASON_USER_STOP);
    } else {
        begin();
        if (strcmp(mode, "normal") == 0 || strcmp(mode, "wrap") == 0 || strcmp(mode, "tx_full") == 0) {
            transport_full = strcmp(mode, "tx_full") == 0;
            advance(15500U, true);
            assert(snapshot().phase == WHEEL_TEST_DONE && latch);
            assert(output_left == 0 && output_right == 0 && status_frame.remaining_ms == 0U);
            assert(status_frame.test_elapsed_ms == 21000U && status_frame.reason == 0U);
            assert(!wheel_test_task_command(WHEEL_TEST_START));
            assert(!wheel_test_task_command(WHEEL_TEST_HEARTBEAT));
            assert(encoder_frames == status_frames && encoder_frames == 421U);
            if (transport_full) assert(encoder_frame.tx_dropped > 0U);
        } else if (strcmp(mode, "stop") == 0) {
            assert(wheel_test_task_command(WHEEL_TEST_STOP)); verify_stopped(WHEEL_TEST_REASON_USER_STOP);
        } else if (strcmp(mode, "preemption") == 0) {
            stop_in_sample = true; advance(50U, true); verify_stopped(WHEEL_TEST_REASON_USER_STOP);
        } else if (strcmp(mode, "timeout") == 0) {
            advance(750U, false); verify_stopped(WHEEL_TEST_REASON_LINK_TIMEOUT);
        } else if (strcmp(mode, "deadline") == 0) {
            tick(101U); verify_stopped(WHEEL_TEST_REASON_DEADLINE);
        } else if (strcmp(mode, "sample_deadline") == 0) {
            sample_delay = 101U; advance(50U, false); sample_delay = 0U; verify_stopped(WHEEL_TEST_REASON_DEADLINE);
        } else if (strcmp(mode, "late_heartbeat") == 0) {
            sample_delay = 751U; heartbeat_during_sample = true;
            advance(50U, false); sample_delay = 0U; verify_stopped(WHEEL_TEST_REASON_LINK_TIMEOUT);
        } else if (strcmp(mode, "nan") == 0 || strcmp(mode, "invalid") == 0) {
            if (strcmp(mode, "nan") == 0) nan_sample = true; else valid = false;
            advance(50U, false); verify_stopped(WHEEL_TEST_REASON_SENSOR_ERROR);
            assert(encoder_frame.sample_errors > 0U && !(encoder_frame.flags & ENCODER_TEST_VALID));
        } else assert(!"Unknown scenario");
    }
    printf("PASS wheel task %s\n", mode);
    return 0;
}
