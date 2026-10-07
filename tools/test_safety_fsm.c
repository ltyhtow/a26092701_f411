/** Host behavior regressions for defaults, qualification, stops and command expiry. */
#include "safety_fsm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define NEAR(a, b) assert(fabsf((a) - (b)) < 0.011f)
static uint32_t now;
static void tick(safety_fsm_t *fsm, float pitch, float roll, float gyro) {
    now += 5U;
    safety_fsm_update_attitude(fsm, pitch, roll, gyro, 0.0f, 0.0f);
    safety_fsm_step(fsm, 0.005f, now);
}
static void qualify(safety_fsm_t *fsm) {
    for (unsigned i = 0; i < 40; ++i) tick(fsm, 0, 0, 0);
    assert(safety_fsm_is_steady(fsm));
}
static void calibrated(safety_fsm_t *fsm) {
    now = 0;
    safety_fsm_init(fsm, NULL);
    assert(safety_fsm_request_calibration(fsm));
    safety_fsm_notify_calibration_done(fsm, true);
    qualify(fsm);
}
static void test_init_and_defaults(void) {
    safety_fsm_t f;
    safety_fsm_init(&f, NULL);
    assert(f.state == BALANCE_STATE_DISARMED);
    assert(!safety_fsm_is_motor_enabled(&f));
    assert(!safety_fsm_is_upright(&f));
    assert(!safety_fsm_is_steady(&f));
    assert(!safety_fsm_is_calibrated(&f));
    assert(f.fault_flags == SAFETY_FAULT_NONE);
    assert(f.config.require_calibration && !f.config.auto_rearm_enable);
    NEAR(f.config.pitch_fall_limit_deg, 35);
    NEAR(f.config.roll_fall_limit_deg, 30);
    NEAR(f.config.pitch_recovery_limit_deg, 5);
    NEAR(f.config.roll_recovery_limit_deg, 10);
    assert(f.config.cmd_timeout_ms == 500U);
    assert(!safety_fsm_request_arm(&f));
}
static void test_calibration_lifecycle(void) {
    safety_fsm_t f;
    now = 0;
    safety_fsm_init(&f, NULL);
    qualify(&f);
    assert(!safety_fsm_request_arm(&f));
    assert(safety_fsm_request_calibration(&f));
    assert(f.state == BALANCE_STATE_CALIBRATING);
    assert(!safety_fsm_is_motor_enabled(&f));
    assert(!safety_fsm_request_calibration(&f));
    safety_fsm_notify_calibration_done(&f, true);
    assert(f.state == BALANCE_STATE_DISARMED && f.is_calibrated);
    assert(!safety_fsm_request_arm(&f)); /* Need fresh steady qualification after calibration. */
    qualify(&f);
    assert(safety_fsm_request_arm(&f));
    assert(safety_fsm_is_motor_enabled(&f));
}
static void test_arm_and_disarm(void) {
    safety_fsm_t f;
    calibrated(&f);
    assert(safety_fsm_request_arm(&f));
    safety_fsm_feed_motion_cmd(&f, 1, 1, now);
    tick(&f, 0, 0, 0);
    safety_fsm_request_disarm(&f);
    assert(f.state == BALANCE_STATE_DISARMED && !safety_fsm_is_motor_enabled(&f));
    assert(f.target_linear_cmd == 0 && f.target_yaw_cmd == 0);
    qualify(&f); /* Original bug re-armed immediately in this interval. */
    assert(f.state == BALANCE_STATE_DISARMED);
    assert(safety_fsm_request_arm(&f));
    tick(&f, 0, 0, 0);
    NEAR(f.ramped_linear_vel, 0);
}
static void test_arm_rejection_when_tilted(void) {
    safety_fsm_t f;
    calibrated(&f);
    tick(&f, 10, 0, 0);
    assert(!safety_fsm_can_rearm(&f) && !safety_fsm_request_arm(&f));
    tick(&f, 2, 0, 0);
    assert(!safety_fsm_request_arm(&f));
    qualify(&f);
    assert(safety_fsm_can_rearm(&f) && safety_fsm_request_arm(&f));
}
static void test_fall_protection_pitch_and_roll(void) {
    safety_fsm_t f;
    calibrated(&f);
    assert(safety_fsm_request_arm(&f));
    tick(&f, 36, 0, 0);
    assert(f.state == BALANCE_STATE_FALLEN && f.fall_event_count == 1U);
    assert(!safety_fsm_is_motor_enabled(&f) && safety_fsm_has_fault(&f, SAFETY_FAULT_FALL_PITCH));
    tick(&f, 20, 0, 0);
    assert(!safety_fsm_reset_fault(&f) && !safety_fsm_request_arm(&f));
    qualify(&f);
    assert(f.state == BALANCE_STATE_FALLEN && !safety_fsm_request_arm(&f));
    assert(safety_fsm_reset_fault(&f));
    assert(f.state == BALANCE_STATE_DISARMED && !safety_fsm_has_fault(&f, SAFETY_FAULT_FALL_PITCH));
    assert(safety_fsm_request_arm(&f));
    tick(&f, 0, -32, 0);
    assert(f.state == BALANCE_STATE_FALLEN && !safety_fsm_is_motor_enabled(&f));
    assert(safety_fsm_has_fault(&f, SAFETY_FAULT_FALL_ROLL));
}
static void test_watchdog_timeout_and_ramp_to_zero(void) {
    safety_fsm_t f;
    calibrated(&f);
    assert(safety_fsm_request_arm(&f));
    for (unsigned i = 0; i < 100; ++i) {
        safety_fsm_feed_motion_cmd(&f, 1, 0, now + 5U);
        tick(&f, 0, 0, 0);
    }
    NEAR(f.ramped_linear_vel, 1);
    for (unsigned i = 0; i < 90; ++i) tick(&f, 0, 0, 0);
    assert(!f.watchdog_tripped);
    for (unsigned i = 0; i < 12; ++i) tick(&f, 0, 0, 0);
    assert(f.watchdog_tripped && safety_fsm_has_fault(&f, SAFETY_FAULT_CMD_TIMEOUT));
    assert(f.state == BALANCE_STATE_ARMED && safety_fsm_is_motor_enabled(&f));
    for (unsigned i = 0; i < 100; ++i) tick(&f, 0, 0, 0);
    NEAR(f.ramped_linear_vel, 0);
    safety_fsm_feed_heartbeat(&f, now);
    assert(!f.watchdog_tripped && !safety_fsm_has_fault(&f, SAFETY_FAULT_CMD_TIMEOUT));
    tick(&f, 0, 0, 0);
    NEAR(f.ramped_linear_vel, 0); /* Heartbeat cannot revive an old motion target. */
}
static void test_slew_rate_limiter(void) {
    safety_fsm_t f;
    calibrated(&f);
    assert(safety_fsm_request_arm(&f));
    safety_fsm_feed_motion_cmd(&f, 1, 0, now);
    for (unsigned i = 0; i < 20; ++i) tick(&f, 0, 0, 0);
    NEAR(f.ramped_linear_vel, 0.2f);
    for (unsigned i = 0; i < 20; ++i) tick(&f, 0, 0, 0);
    NEAR(f.ramped_linear_vel, 0.4f);
    safety_fsm_feed_motion_cmd(&f, 0, 0, now);
    for (unsigned i = 0; i < 10; ++i) tick(&f, 0, 0, 0);
    NEAR(f.ramped_linear_vel, 0.2f);
    for (unsigned i = 0; i < 10; ++i) tick(&f, 0, 0, 0);
    NEAR(f.ramped_linear_vel, 0);
}
static void test_emergency_stop_and_invalid_inputs(void) {
    safety_fsm_t f;
    calibrated(&f);
    assert(safety_fsm_request_arm(&f));
    safety_fsm_emergency_stop(&f);
    assert(f.state == BALANCE_STATE_FALLEN && !safety_fsm_is_motor_enabled(&f));
    assert(safety_fsm_has_fault(&f, SAFETY_FAULT_EMERGENCY_STOP));
    assert(!safety_fsm_request_arm(&f));
    assert(safety_fsm_reset_fault(&f));
    assert(f.state == BALANCE_STATE_DISARMED && !safety_fsm_has_fault(&f, SAFETY_FAULT_EMERGENCY_STOP));
    assert(safety_fsm_request_arm(&f));
    safety_fsm_update_attitude(&f, NAN, 0, 0, 0, 0);
    assert(f.state == BALANCE_STATE_FALLEN && !safety_fsm_is_motor_enabled(&f));
    assert(!safety_fsm_can_rearm(&f) && !safety_fsm_request_arm(&f));
    safety_fsm_request_disarm(&f);
    qualify(&f);
    assert(f.state == BALANCE_STATE_FALLEN); /* Disarm cannot bypass a fault reset. */
    assert(safety_fsm_has_fault(&f, SAFETY_FAULT_SENSOR_INVALID));
    assert(safety_fsm_reset_fault(&f));
    assert(safety_fsm_request_arm(&f));
    safety_fsm_feed_motion_cmd(&f, NAN, 0, now);
    assert(!safety_fsm_is_motor_enabled(&f));
}
static void test_fresh_qualification_auto_policy_and_wrap(void) {
    safety_fsm_t f;
    now = 0;
    safety_fsm_init(&f, NULL);
    f.config.auto_rearm_enable = true;
    qualify(&f);
    assert(f.state == BALANCE_STATE_DISARMED && !safety_fsm_request_arm(&f));
    assert(safety_fsm_request_calibration(&f));
    safety_fsm_notify_calibration_done(&f, true);
    safety_fsm_update_attitude(&f, 0, 0, 0, 0, 0);
    for (unsigned i = 0; i < 100; ++i) safety_fsm_step(&f, 0.005f, now += 5U);
    assert(!f.is_steady && !safety_fsm_request_arm(&f));
    qualify(&f);
    assert(safety_fsm_request_arm(&f));
    tick(&f, 36, 0, 0);
    f.is_calibrated = false;
    qualify(&f);
    assert(f.state == BALANCE_STATE_FALLEN); /* Auto recovery must respect calibration. */
    f.is_calibrated = true;
    qualify(&f);
    assert(f.state == BALANCE_STATE_ARMED);
    now = UINT32_MAX - 10U;
    safety_fsm_feed_motion_cmd(&f, 1, 0, now);
    for (unsigned i = 0; i < 6; ++i) tick(&f, 0, 0, 0);
    assert(!f.watchdog_tripped);
    tick(&f, 36, 0, 0);
    safety_fsm_request_disarm(&f);
    qualify(&f);
    assert(f.state == BALANCE_STATE_FALLEN); /* Even opt-in auto recovery cannot undo manual stop. */
}
int main(void) {
    test_init_and_defaults(); test_calibration_lifecycle(); test_arm_and_disarm();
    test_arm_rejection_when_tilted(); test_fall_protection_pitch_and_roll();
    test_watchdog_timeout_and_ramp_to_zero(); test_slew_rate_limiter();
    test_emergency_stop_and_invalid_inputs(); test_fresh_qualification_auto_policy_and_wrap();
    puts("PASS: 9 safety FSM behavior suites");
    return 0;
}
