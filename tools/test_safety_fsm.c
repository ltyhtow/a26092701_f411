/**
 * @file test_safety_fsm.c
 * @brief Standalone unit test suite for Safety & Interaction FSM.
 */

#include "safety_fsm.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        printf("[FAIL] Line %d: Assertion '%s' failed.\n", __LINE__, #cond); \
        assert(cond); \
    } \
} while (0)

#define ASSERT_FLOAT_NEAR(val, exp, tol) do { \
    if (fabsf((val) - (exp)) > (tol)) { \
        printf("[FAIL] Line %d: Expected %f, got %f (diff %f > %f)\n", \
               __LINE__, (double)(exp), (double)(val), (double)fabsf((val) - (exp)), (double)(tol)); \
        assert(fabsf((val) - (exp)) <= (tol)); \
    } \
} while (0)

static void test_init_and_defaults(void) {
    safety_fsm_t fsm;
    safety_fsm_init(&fsm, NULL);

    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_DISARMED);
    ASSERT_TRUE(!safety_fsm_is_motor_enabled(&fsm));
    ASSERT_TRUE(safety_fsm_is_upright(&fsm));
    ASSERT_TRUE(!safety_fsm_is_steady(&fsm));
    ASSERT_TRUE(!safety_fsm_is_calibrated(&fsm));
    ASSERT_TRUE(safety_fsm_get_fault_flags(&fsm) == SAFETY_FAULT_NONE);

    ASSERT_FLOAT_NEAR(fsm.config.pitch_fall_limit_deg, 35.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(fsm.config.roll_fall_limit_deg, 30.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(fsm.config.pitch_recovery_limit_deg, 5.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(fsm.config.roll_recovery_limit_deg, 10.0f, 1e-4f);
    ASSERT_TRUE(fsm.config.cmd_timeout_ms == 500U);

    printf("  [PASS] test_init_and_defaults\n");
}

static void test_calibration_lifecycle(void) {
    safety_fsm_t fsm;
    safety_fsm_init(&fsm, NULL);

    /* Cannot arm before calibration if required */
    fsm.config.require_calibration = true;
    safety_fsm_update_attitude(&fsm, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    /* Make steady */
    for (int i = 0; i < 50; i++) {
        safety_fsm_step(&fsm, 0.005f, (uint32_t)(i * 5));
    }
    ASSERT_TRUE(safety_fsm_is_steady(&fsm));
    ASSERT_TRUE(!safety_fsm_request_arm(&fsm)); /* rejected */

    /* Enter calibration */
    ASSERT_TRUE(safety_fsm_request_calibration(&fsm));
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_CALIBRATING);
    ASSERT_TRUE(!safety_fsm_is_motor_enabled(&fsm));

    /* Cannot calibrate while calibrating */
    ASSERT_TRUE(!safety_fsm_request_calibration(&fsm));

    /* Calibration done with success */
    safety_fsm_notify_calibration_done(&fsm, true);
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_DISARMED);
    ASSERT_TRUE(safety_fsm_is_calibrated(&fsm));

    /* Now arm should succeed */
    ASSERT_TRUE(safety_fsm_request_arm(&fsm));
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_ARMED);
    ASSERT_TRUE(safety_fsm_is_motor_enabled(&fsm));

    printf("  [PASS] test_calibration_lifecycle\n");
}

static void test_arm_and_disarm(void) {
    safety_fsm_t fsm;
    safety_fsm_init(&fsm, NULL);

    safety_fsm_update_attitude(&fsm, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.01f, 10);

    /* Arm */
    ASSERT_TRUE(safety_fsm_request_arm(&fsm));
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_ARMED);
    ASSERT_TRUE(safety_fsm_is_motor_enabled(&fsm));

    /* Disarm */
    safety_fsm_request_disarm(&fsm);
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_DISARMED);
    ASSERT_TRUE(!safety_fsm_is_motor_enabled(&fsm));

    printf("  [PASS] test_arm_and_disarm\n");
}

static void test_arm_rejection_when_tilted(void) {
    safety_fsm_t fsm;
    safety_fsm_init(&fsm, NULL);

    /* Pitch is 10 deg, recovery threshold is 5 deg */
    safety_fsm_update_attitude(&fsm, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.01f, 10);

    ASSERT_TRUE(!safety_fsm_can_rearm(&fsm));
    ASSERT_TRUE(!safety_fsm_request_arm(&fsm));
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_DISARMED);

    /* Now bring upright to 2.0 deg */
    safety_fsm_update_attitude(&fsm, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.01f, 20);

    ASSERT_TRUE(safety_fsm_can_rearm(&fsm));
    ASSERT_TRUE(safety_fsm_request_arm(&fsm));
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_ARMED);

    printf("  [PASS] test_arm_rejection_when_tilted\n");
}

static void test_fall_protection_pitch_and_roll(void) {
    safety_fsm_t fsm;
    safety_fsm_init(&fsm, NULL);

    safety_fsm_update_attitude(&fsm, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.01f, 10);
    ASSERT_TRUE(safety_fsm_request_arm(&fsm));
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_ARMED);

    /* 1. Pitch exceeds fall threshold (|36| > 35) */
    safety_fsm_update_attitude(&fsm, 36.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.005f, 15);

    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_FALLEN);
    ASSERT_TRUE(!safety_fsm_is_motor_enabled(&fsm));
    ASSERT_TRUE(safety_fsm_has_fault(&fsm, SAFETY_FAULT_FALL_PITCH));
    ASSERT_TRUE(fsm.fall_event_count == 1U);

    /* Recovery: Robot still tilted (pitch = 20 deg), cannot re-arm */
    safety_fsm_update_attitude(&fsm, 20.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.005f, 20);
    ASSERT_TRUE(!safety_fsm_reset_fault(&fsm));
    ASSERT_TRUE(!safety_fsm_request_arm(&fsm));
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_FALLEN);

    /* Recovery: Robot placed upright (|pitch| = 3.0 deg < 5.0 deg) */
    safety_fsm_update_attitude(&fsm, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.005f, 25);
    ASSERT_TRUE(safety_fsm_is_upright(&fsm));

    /* Reset fault transitions from FALLEN to DISARMED */
    ASSERT_TRUE(safety_fsm_reset_fault(&fsm));
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_DISARMED);
    ASSERT_TRUE(!safety_fsm_has_fault(&fsm, SAFETY_FAULT_FALL_PITCH));

    /* Arm again */
    ASSERT_TRUE(safety_fsm_request_arm(&fsm));
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_ARMED);

    /* 2. Roll exceeds fall threshold (|32| > 30) */
    safety_fsm_update_attitude(&fsm, 0.0f, -32.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.005f, 30);
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_FALLEN);
    ASSERT_TRUE(!safety_fsm_is_motor_enabled(&fsm));
    ASSERT_TRUE(safety_fsm_has_fault(&fsm, SAFETY_FAULT_FALL_ROLL));

    printf("  [PASS] test_fall_protection_pitch_and_roll\n");
}

static void test_watchdog_timeout_and_ramp_to_zero(void) {
    safety_fsm_t fsm;
    safety_fsm_init(&fsm, NULL);

    safety_fsm_update_attitude(&fsm, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.005f, 0);
    safety_fsm_request_arm(&fsm);

    /* Send motion command at t = 100ms: target linear = 1.0 */
    safety_fsm_feed_motion_cmd(&fsm, 1.0f, 0.0f, 100);

    /* Step for 500ms while keeping watchdog alive */
    uint32_t t = 100;
    for (int i = 0; i < 100; i++) {
        t += 5;
        safety_fsm_feed_motion_cmd(&fsm, 1.0f, 0.0f, t);
        safety_fsm_step(&fsm, 0.005f, t);
    }

    /* Output should have ramped up to 1.0 */
    float out_linear = 0.0f, out_yaw = 0.0f;
    safety_fsm_get_motion_output(&fsm, &out_linear, &out_yaw);
    ASSERT_FLOAT_NEAR(out_linear, 1.0f, 0.01f);
    ASSERT_TRUE(!safety_fsm_has_fault(&fsm, SAFETY_FAULT_CMD_TIMEOUT));

    /* Now STOP sending commands. Run until watchdog trips (timeout is 500ms) */
    /* t currently is 600ms, last command was at 600ms */
    for (int i = 0; i < 90; i++) { /* 450ms elapse */
        t += 5;
        safety_fsm_step(&fsm, 0.005f, t);
    }
    ASSERT_TRUE(!fsm.watchdog_tripped); /* 450ms < 500ms */

    /* Advance another 60ms (total 510ms since last command) */
    for (int i = 0; i < 12; i++) {
        t += 5;
        safety_fsm_step(&fsm, 0.005f, t);
    }

    /* Watchdog must have tripped */
    ASSERT_TRUE(fsm.watchdog_tripped);
    ASSERT_TRUE(safety_fsm_has_fault(&fsm, SAFETY_FAULT_CMD_TIMEOUT));
    /* But robot should STILL be armed balancing upright in place! */
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_ARMED);
    ASSERT_TRUE(safety_fsm_is_motor_enabled(&fsm));

    /* Continue stepping so ramp controller smoothly decelerates speed to 0 */
    for (int i = 0; i < 100; i++) {
        t += 5;
        safety_fsm_step(&fsm, 0.005f, t);
    }

    safety_fsm_get_motion_output(&fsm, &out_linear, &out_yaw);
    ASSERT_FLOAT_NEAR(out_linear, 0.0f, 0.01f);

    /* Feed heartbeat at t -> clears watchdog */
    safety_fsm_feed_heartbeat(&fsm, t);
    ASSERT_TRUE(!fsm.watchdog_tripped);
    ASSERT_TRUE(!safety_fsm_has_fault(&fsm, SAFETY_FAULT_CMD_TIMEOUT));

    printf("  [PASS] test_watchdog_timeout_and_ramp_to_zero\n");
}

static void test_slew_rate_limiter(void) {
    safety_fsm_t fsm;
    safety_fsm_init(&fsm, NULL);

    /* Configure max_linear_accel = 2.0 units/s^2, max_linear_decel = 4.0 units/s^2 */
    fsm.config.max_linear_accel = 2.0f;
    fsm.config.max_linear_decel = 4.0f;

    safety_fsm_update_attitude(&fsm, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.01f, 0);
    safety_fsm_request_arm(&fsm);

    /* Command sudden jump to 1.0 */
    safety_fsm_feed_motion_cmd(&fsm, 1.0f, 0.0f, 100);

    /* In dt = 0.1s, acceleration step = 2.0 * 0.1 = 0.2 */
    safety_fsm_step(&fsm, 0.1f, 100);
    float out_linear = 0.0f;
    safety_fsm_get_motion_output(&fsm, &out_linear, NULL);
    ASSERT_FLOAT_NEAR(out_linear, 0.2f, 0.001f);

    /* Next 0.1s: 0.2 + 0.2 = 0.4 */
    safety_fsm_step(&fsm, 0.1f, 200);
    safety_fsm_get_motion_output(&fsm, &out_linear, NULL);
    ASSERT_FLOAT_NEAR(out_linear, 0.4f, 0.001f);

    /* Now command sudden brake to 0.0 */
    safety_fsm_feed_motion_cmd(&fsm, 0.0f, 0.0f, 250);
    /* In dt = 0.05s, deceleration step = 4.0 * 0.05 = 0.2 */
    safety_fsm_step(&fsm, 0.05f, 250);
    safety_fsm_get_motion_output(&fsm, &out_linear, NULL);
    ASSERT_FLOAT_NEAR(out_linear, 0.2f, 0.001f);

    safety_fsm_step(&fsm, 0.05f, 300);
    safety_fsm_get_motion_output(&fsm, &out_linear, NULL);
    ASSERT_FLOAT_NEAR(out_linear, 0.0f, 0.001f);

    printf("  [PASS] test_slew_rate_limiter\n");
}

static void test_emergency_stop_and_invalid_inputs(void) {
    safety_fsm_t fsm;
    safety_fsm_init(&fsm, NULL);

    safety_fsm_update_attitude(&fsm, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    safety_fsm_step(&fsm, 0.01f, 10);
    ASSERT_TRUE(safety_fsm_request_arm(&fsm));

    /* Emergency stop */
    safety_fsm_emergency_stop(&fsm);
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_FALLEN);
    ASSERT_TRUE(!safety_fsm_is_motor_enabled(&fsm));
    ASSERT_TRUE(safety_fsm_has_fault(&fsm, SAFETY_FAULT_EMERGENCY_STOP));

    /* Cannot re-arm while E-stop active */
    ASSERT_TRUE(!safety_fsm_request_arm(&fsm));

    /* Reset fault clears E-stop */
    ASSERT_TRUE(safety_fsm_reset_fault(&fsm));
    ASSERT_TRUE(safety_fsm_get_state(&fsm) == BALANCE_STATE_DISARMED);
    ASSERT_TRUE(!safety_fsm_has_fault(&fsm, SAFETY_FAULT_EMERGENCY_STOP));

    /* Invalid NaN sensor inputs */
    safety_fsm_update_attitude(&fsm, (float)NAN, 0.0f, 0.0f, 0.0f, 0.0f);
    ASSERT_TRUE(safety_fsm_has_fault(&fsm, SAFETY_FAULT_SENSOR_INVALID));
    ASSERT_TRUE(!safety_fsm_can_rearm(&fsm));
    ASSERT_TRUE(!safety_fsm_request_arm(&fsm));

    printf("  [PASS] test_emergency_stop_and_invalid_inputs\n");
}

int main(void) {
    printf("Starting Safety & Interaction FSM Test Suite...\n");

    test_init_and_defaults();
    test_calibration_lifecycle();
    test_arm_and_disarm();
    test_arm_rejection_when_tilted();
    test_fall_protection_pitch_and_roll();
    test_watchdog_timeout_and_ramp_to_zero();
    test_slew_rate_limiter();
    test_emergency_stop_and_invalid_inputs();

    printf("ALL 8 TEST SUITES PASSED CLEANLY!\n");
    return 0;
}
