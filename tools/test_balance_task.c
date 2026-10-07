/* Public portable runtime API: no scheduler, wire types, HAL, or production .c inclusion. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "balance_runtime.h"

static balance_runtime_t runtime;
static balance_runtime_snapshot_t snapshot;
static attitude_sample_t attitudes[64];
static system_request_t systems[64];
static motion_request_t motions[64];
static unsigned attitude_count, system_count, motion_count, calibration_count;
static uint32_t test_now, pending_requests, dispatch_delay;
static bool calibration_ok;
static float encoder_left_counts, encoder_right_counts;
#define HEALTHY (ATTITUDE_STATUS_CALIBRATED | ATTITUDE_STATUS_ATTITUDE_VALID)
#define ENABLE 1U
#define CLEAR_FAULT 2U

static void setup(void) {
    balance_controller_config_t config;
    balance_controller_default_config(&config);
    config.velocity_loop.lpf_alpha = 0.0f;
    balance_runtime_init(&runtime, &config);
    attitude_count = system_count = motion_count = calibration_count = 0;
    test_now = pending_requests = dispatch_delay = 0U;
    calibration_ok = true;
    encoder_left_counts = encoder_right_counts = 0.0f;
    balance_runtime_get_snapshot(&runtime, &snapshot);
    assert(snapshot.outputs.safety_active);
}
static void system_action(system_action_t action) {
    assert(system_count < 64U);
    systems[system_count++] = (system_request_t){.action = action};
}
static void motion_action(uint16_t flags, float speed) {
    assert(motion_count < 64U);
    motions[motion_count++] = (motion_request_t){.enable = (flags & ENABLE) != 0U,
        .clear_fault = (flags & CLEAR_FAULT) != 0U, .linear_counts_per_5ms = speed, .timeout_ms = 500};
}
static void sample(uint32_t timestamp, uint16_t flags, float pitch) {
    assert(attitude_count < 64U);
    attitudes[attitude_count++] = (attitude_sample_t){.timestamp_ms = timestamp, .status_flags = flags, .pitch_deg = pitch};
}
static void cycle(void) {
    balance_runtime_begin_cycle(&runtime, test_now);
    balance_runtime_request_flags(&runtime, pending_requests);
    pending_requests = 0U;
    for (unsigned n = 0; n < attitude_count; ++n) balance_runtime_accept_attitude(&runtime, &attitudes[n]);
    for (unsigned n = 0; n < system_count; ++n) balance_runtime_request_system(&runtime, &systems[n]);
    for (unsigned n = 0; n < motion_count; ++n) balance_runtime_accept_motion(&runtime, &motions[n]);
    attitude_count = system_count = motion_count = 0U;
    balance_runtime_finish_cycle(&runtime, encoder_left_counts, encoder_right_counts);
    balance_runtime_get_snapshot(&runtime, &snapshot);
    if (snapshot.calibration_request) {
        ++calibration_count;
        balance_runtime_calibration_result(&runtime, calibration_ok);
    }
    balance_runtime_prepare_output(&runtime, test_now + dispatch_delay, 0U);
    balance_runtime_get_snapshot(&runtime, &snapshot);
}
static void tick(uint16_t flags, float pitch) { test_now += 5; sample(test_now, flags, pitch); cycle(); }
static void qualify(void) { for (unsigned i = 0; i < 41; ++i) tick(HEALTHY, 1.0f); }
static void arm(void) {
    qualify(); system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1.0f);
    assert(snapshot.state == BALANCE_STATE_ARMED);
    assert(snapshot.outputs.left_pwm != 0 || snapshot.outputs.right_pwm != 0);
}
static void stopped(void) {
    assert(snapshot.state != BALANCE_STATE_ARMED);
    assert(snapshot.outputs.left_pwm == 0 && snapshot.outputs.right_pwm == 0);
}
static void test_boot_and_quality(void) {
    setup(); system_action(SYSTEM_ACTION_ARM); test_now += 5; cycle(); stopped();
    for (unsigned i = 0; i < 45; ++i) { system_action(SYSTEM_ACTION_ARM); tick(ATTITUDE_STATUS_ATTITUDE_VALID, 1); stopped(); }
    for (unsigned i = 0; i < 45; ++i) { system_action(SYSTEM_ACTION_ARM); tick(HEALTHY | ATTITUDE_STATUS_STARTUP, 1); stopped(); }
    for (unsigned i = 0; i < 20; ++i) tick(HEALTHY, 1);
    system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1); stopped();
    arm();
}
static void test_invalid_age_duplicates_and_wrap(void) {
    const uint16_t invalid[] = {HEALTHY | ATTITUDE_STATUS_INPUT_INVALID, ATTITUDE_STATUS_CALIBRATED, HEALTHY | ATTITUDE_STATUS_STARTUP};
    for (unsigned j = 0; j < sizeof(invalid)/sizeof(invalid[0]); ++j) {
        setup(); arm(); tick(invalid[j], 1); stopped();
        assert(snapshot.state == BALANCE_STATE_FALLEN);
        qualify(); system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1); stopped();
        system_action(SYSTEM_ACTION_RESET); system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1); stopped();
        system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1); assert(snapshot.state == BALANCE_STATE_ARMED);
    }
    setup(); arm();
    const uint32_t stamp = test_now;
    for (unsigned i = 0; i < 5; ++i) { test_now += 5; sample(stamp, HEALTHY, 1); cycle(); }
    stopped();
    setup(); test_now = UINT32_MAX - 150U; arm();
    assert(test_now < 150U); tick(HEALTHY, 1); assert(snapshot.state == BALANCE_STATE_ARMED);
}
static void test_stop_precedence_and_old_motion(void) {
    setup(); arm(); system_action(SYSTEM_ACTION_DISARM); motion_action(ENABLE, 1);
    tick(HEALTHY, 1); stopped();
    motion_action(ENABLE, 1); tick(HEALTHY, 1); stopped();
    system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1);
    assert(snapshot.state == BALANCE_STATE_ARMED && runtime.safety.target_linear_cmd == 0);
    motion_action(ENABLE, 1); motion_action(0, 0); motion_action(ENABLE, 1); tick(HEALTHY, 1); stopped();
    arm(); pending_requests = BALANCE_REQUEST_DISARM | BALANCE_REQUEST_ARM; tick(HEALTHY, 1); stopped();
    arm(); system_action(SYSTEM_ACTION_RESET); tick(HEALTHY, 20); stopped();
    setup(); arm(); tick(HEALTHY, 36); qualify();
    assert(snapshot.state == BALANCE_STATE_FALLEN);
    pending_requests = BALANCE_REQUEST_DISARM;
    motion_action(CLEAR_FAULT, 0); tick(HEALTHY, 1); stopped();
    assert(snapshot.state == BALANCE_STATE_DISARMED && snapshot.fault_flags == 0U);
}
static void test_deadline_encoder_and_preemption(void) {
    setup(); arm(); test_now += 25; sample(test_now, HEALTHY, 1); cycle(); stopped();
    setup(); arm(); cycle(); stopped();
    setup(); arm(); encoder_left_counts = NAN; tick(HEALTHY, 1); stopped();
    setup(); arm(); tick(HEALTHY, NAN); stopped();
    setup(); arm(); dispatch_delay = 25; tick(HEALTHY, 1); stopped();
    assert(snapshot.fault_flags & SAFETY_FAULT_SENSOR_INVALID);
    setup(); arm();
    balance_runtime_prepare_output(&runtime, test_now, BALANCE_REQUEST_ESTOP);
    balance_runtime_get_snapshot(&runtime, &snapshot);
    assert(snapshot.outputs.left_pwm == 0 && snapshot.outputs.right_pwm == 0);
    pending_requests = BALANCE_REQUEST_ESTOP; tick(HEALTHY, 1); stopped();
}
static void test_encoder_interval_normalization(void) {
    setup(); arm(); encoder_left_counts = 20; encoder_right_counts = -20;
    test_now += 10; sample(test_now, HEALTHY, 1); cycle();
    assert(snapshot.state == BALANCE_STATE_ARMED);
    assert(snapshot.left_counts_per_5ms == 10 && snapshot.right_counts_per_5ms == -10);
    assert(fabsf(runtime.controller.state.velocity.filtered_speed_left - 10.0f) < 0.001f);
    assert(fabsf(runtime.controller.state.velocity.filtered_speed_right + 10.0f) < 0.001f);
    encoder_left_counts = 10; encoder_right_counts = -10; tick(HEALTHY, 1);
    assert(snapshot.left_counts_per_5ms == 10 && snapshot.right_counts_per_5ms == -10);
}
static void test_calibration_handshake(void) {
    setup(); arm(); system_action(SYSTEM_ACTION_CALIBRATE); tick(HEALTHY, 1); stopped();
    assert(calibration_count == 1U && snapshot.state == BALANCE_STATE_CALIBRATING);
    qualify(); system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1); stopped();
    tick(ATTITUDE_STATUS_CALIBRATING, 0); qualify(); assert(snapshot.state == BALANCE_STATE_DISARMED);
    system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1); assert(snapshot.state == BALANCE_STATE_ARMED);
    setup(); calibration_ok = false; system_action(SYSTEM_ACTION_CALIBRATE); tick(HEALTHY, 1); stopped();
    qualify(); system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1); stopped();
    setup(); qualify(); system_action(SYSTEM_ACTION_CALIBRATE); tick(HEALTHY, 1);
    for (unsigned i = 0; i < 3001; ++i) tick(HEALTHY, 1);
    stopped(); assert(snapshot.state == BALANCE_STATE_DISARMED && (snapshot.fault_flags & SAFETY_FAULT_CALIBRATION_FAIL));
    system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1); stopped();
    system_action(SYSTEM_ACTION_CALIBRATE); tick(HEALTHY, 1);
    tick(ATTITUDE_STATUS_CALIBRATING, 0); qualify();
    system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1); assert(snapshot.state == BALANCE_STATE_ARMED);
}
static void test_independent_contexts_and_domain_validation(void) {
    setup(); arm();
    balance_runtime_t second;
    balance_runtime_snapshot_t second_snapshot;
    balance_runtime_init(&second, NULL);
    balance_runtime_begin_cycle(&second, test_now);
    balance_runtime_finish_cycle(&second, 0, 0);
    balance_runtime_get_snapshot(&second, &second_snapshot);
    assert(second_snapshot.state == BALANCE_STATE_DISARMED && snapshot.state == BALANCE_STATE_ARMED);
    const pid_request_t gains = {.loop_id = 0, .kp = 40, .kd = 0.6f};
    const float before = runtime.controller.config.balance_loop.kp;
    assert(!balance_runtime_accept_pid(&runtime, &gains));
    assert(runtime.controller.config.balance_loop.kp == before); /* Armed update rejected. */
    system_action(SYSTEM_ACTION_DISARM); tick(HEALTHY, 1);
    assert(balance_runtime_accept_pid(&runtime, &gains));
    assert(runtime.controller.config.balance_loop.kp == 40.0f);
    pid_request_t invalid = gains; invalid.kp = NAN;
    assert(!balance_runtime_accept_pid(&runtime, &invalid));
    assert(runtime.controller.config.balance_loop.kp == 40.0f);
    motion_action(ENABLE, NAN); tick(HEALTHY, 1); stopped();
    system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1);
    assert(snapshot.state == BALANCE_STATE_ARMED);
    motion_action(ENABLE, 0); motions[motion_count - 1U].timeout_ms = UINT16_MAX;
    tick(HEALTHY, 1); assert(runtime.safety.config.cmd_timeout_ms == SAFETY_FSM_DEFAULT_CMD_TIMEOUT_MS);
}
static void test_exact_freshness_and_stop_boundaries(void) {
    setup(); arm();
    balance_runtime_prepare_output(&runtime, test_now + 20U, 0U);
    balance_runtime_get_snapshot(&runtime, &snapshot);
    assert(snapshot.state == BALANCE_STATE_ARMED && !snapshot.outputs.safety_active);
    balance_runtime_prepare_output(&runtime, test_now + 21U, 0U);
    balance_runtime_get_snapshot(&runtime, &snapshot); stopped();
    setup(); arm();
    test_now += 5U; sample(test_now + 1U, HEALTHY, 1); cycle(); stopped();
    setup(); arm();
    /* Out-of-order samples cannot replace the most recent timestamp. */
    const uint32_t latest = test_now;
    test_now += 5U; sample(latest - 5U, HEALTHY, 1); cycle();
    assert(snapshot.attitude.timestamp_ms == latest);
    test_now += 20U; sample(latest - 5U, HEALTHY, 1); cycle(); stopped();
    setup(); arm();
    /* Reset plus enable clears a fault while remaining stopped, then a separate
     * explicit arm is required; repeated enabled motion is not an arm edge. */
    tick(HEALTHY, 36); qualify(); motion_action(ENABLE | CLEAR_FAULT, 1);
    tick(HEALTHY, 1); stopped(); assert(snapshot.fault_flags == 0U);
    motion_action(ENABLE, 1); tick(HEALTHY, 1); stopped();
    system_action(SYSTEM_ACTION_ARM); tick(HEALTHY, 1);
    assert(snapshot.state == BALANCE_STATE_ARMED);
}
int main(void) {
    test_boot_and_quality(); test_invalid_age_duplicates_and_wrap();
    test_stop_precedence_and_old_motion(); test_deadline_encoder_and_preemption();
    test_encoder_interval_normalization(); test_calibration_handshake();
    test_independent_contexts_and_domain_validation();
    test_exact_freshness_and_stop_boundaries();
    puts("PASS: 8 portable balance runtime suites (public API, no RTOS or wire-format dependencies)");
    return 0;
}
