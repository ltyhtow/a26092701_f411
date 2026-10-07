/**
 * @file test_balance_controller.c
 * @brief Comprehensive standalone unit test suite for Balance Controller module.
 */

#include "balance_controller.h"

#include <assert.h>
#include <float.h>
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
    const float actual_value = (val); \
    const float expected_value = (exp); \
    const float allowed_error = (tol); \
    if (!isfinite(actual_value) || !isfinite(expected_value) || \
        !(fabsf(actual_value - expected_value) <= allowed_error)) { \
        printf("[FAIL] Line %d: Expected %f, got %f (diff %f > %f)\n", \
               __LINE__, (double)expected_value, (double)actual_value, \
               (double)fabsf(actual_value - expected_value), (double)allowed_error); \
        assert(isfinite(actual_value) && isfinite(expected_value) && \
               fabsf(actual_value - expected_value) <= allowed_error); \
    } \
} while (0)

static void test_initialization_and_defaults(void) {
    balance_controller_t ctrl;
    balance_controller_config_t cfg;
    balance_controller_default_config(&cfg);

    ASSERT_FLOAT_NEAR(cfg.balance_loop.kp, 180.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(cfg.balance_loop.kd, 1.2f, 1e-4f);
    ASSERT_FLOAT_NEAR(cfg.velocity_loop.kp, 0.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(cfg.velocity_loop.ki, 0.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(cfg.velocity_loop.integral_limit, 2000.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(cfg.velocity_loop.lpf_alpha, 0.75f, 1e-4f);
    ASSERT_FLOAT_NEAR(cfg.turn_loop.kp, 15.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(cfg.turn_loop.kd, 0.1f, 1e-4f);
    ASSERT_FLOAT_NEAR(cfg.mechanical_zero_pitch, 0.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(cfg.max_pitch_angle, 35.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(cfg.max_pwm, 4999.0f, 1e-4f);
    ASSERT_TRUE(!cfg.auto_recovery_enabled);
    ASSERT_TRUE(cfg.velocity_coupling_mode == BALANCE_VEL_COUPLE_PARALLEL);

    memset(&ctrl, 0xa5, sizeof(ctrl));
    balance_controller_init(&ctrl, &cfg);
    ASSERT_TRUE(!balance_controller_is_enabled(&ctrl));
    ASSERT_TRUE(ctrl.state.tip_over_count == 0U);
    ASSERT_TRUE(!ctrl.state.velocity.is_filter_initialized);
    ASSERT_TRUE(!balance_controller_is_tipped_over(&ctrl));
    printf("[PASS] test_initialization_and_defaults\n");
}

static void test_speed_filter(void) {
    float f = 0.0f;
    /* First step with alpha = 0.8: y = 0.8 * 0 + 0.2 * 100 = 20.0 */
    f = balance_controller_filter_speed(f, 100.0f, 0.8f);
    ASSERT_FLOAT_NEAR(f, 20.0f, 1e-4f);

    /* Second step: y = 0.8 * 20 + 0.2 * 100 = 36.0 */
    f = balance_controller_filter_speed(f, 100.0f, 0.8f);
    ASSERT_FLOAT_NEAR(f, 36.0f, 1e-4f);

    /* Test NaN handling: should preserve current filtered value */
    float f_nan = balance_controller_filter_speed(f, NAN, 0.8f);
    ASSERT_FLOAT_NEAR(f_nan, 36.0f, 1e-4f);

    printf("[PASS] test_speed_filter\n");
}

static void test_deadband_and_limits(void) {
    /* Zero input */
    float z = balance_controller_apply_deadband_and_limit(0.0f, 200.0f, 4999.0f);
    ASSERT_FLOAT_NEAR(z, 0.0f, 1e-4f);

    /* Positive input inside limit */
    float p = balance_controller_apply_deadband_and_limit(50.0f, 200.0f, 4999.0f);
    ASSERT_FLOAT_NEAR(p, 250.0f, 1e-4f);

    /* Negative input inside limit */
    float n = balance_controller_apply_deadband_and_limit(-50.0f, 200.0f, 4999.0f);
    ASSERT_FLOAT_NEAR(n, -250.0f, 1e-4f);

    /* Saturation positive */
    float sat_p = balance_controller_apply_deadband_and_limit(4900.0f, 200.0f, 4999.0f);
    ASSERT_FLOAT_NEAR(sat_p, 4999.0f, 1e-4f);

    /* Saturation negative */
    float sat_n = balance_controller_apply_deadband_and_limit(-4900.0f, 200.0f, 4999.0f);
    ASSERT_FLOAT_NEAR(sat_n, -4999.0f, 1e-4f);

    printf("[PASS] test_deadband_and_limits\n");
}

static void test_balance_loop_pd(void) {
    balance_pid_params_t params = { .kp = 180.0f, .kd = 1.2f };

    /* Upright equilibrium */
    float u0 = balance_controller_calc_balance_pd(&params, 0.0f, 0.0f);
    ASSERT_FLOAT_NEAR(u0, 0.0f, 1e-4f);

    /* 2 degrees error, 0 gyro */
    float u_ang = balance_controller_calc_balance_pd(&params, 2.0f, 0.0f);
    ASSERT_FLOAT_NEAR(u_ang, 360.0f, 1e-4f);

    /* 0 degrees error, 10 dps gyro angular velocity */
    float u_gyro = balance_controller_calc_balance_pd(&params, 0.0f, 10.0f);
    ASSERT_FLOAT_NEAR(u_gyro, 12.0f, 1e-4f);

    /* Combined */
    float u_comb = balance_controller_calc_balance_pd(&params, 2.0f, 10.0f);
    ASSERT_FLOAT_NEAR(u_comb, 372.0f, 1e-4f);

    printf("[PASS] test_balance_loop_pd\n");
}

static void test_velocity_loop_pi_anti_windup(void) {
    velocity_pid_params_t params = {
        .kp = 8.0f,
        .ki = 0.4f,
        .integral_limit = 50.0f,
        .lpf_alpha = 0.75f,
        .max_output = 500.0f
    };
    velocity_state_t state;
    memset(&state, 0, sizeof(state));

    /* Normal step: error = 10, dt = 0.1s -> integral = 1.0 */
    float u = balance_controller_calc_velocity_pi(&params, &state, 10.0f, 0.1f);
    ASSERT_FLOAT_NEAR(state.integral, 1.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(u, 8.0f * 10.0f + 0.4f * 1.0f, 1e-4f);

    /* Saturate integral */
    for (int i = 0; i < 200; i++) {
        balance_controller_calc_velocity_pi(&params, &state, 10.0f, 0.1f);
    }
    ASSERT_FLOAT_NEAR(state.integral, 50.0f, 1e-4f);

    printf("[PASS] test_velocity_loop_pi_anti_windup\n");
}

static void test_turn_loop_pd(void) {
    turn_pid_params_t params = { .kp = 15.0f, .kd = 0.1f, .max_output = 1000.0f };
    turn_state_t state;
    memset(&state, 0, sizeof(state));

    /* First step: dt = 0.005s, target = 20 dps, meas = 0 dps */
    float u1 = balance_controller_calc_turn_pd(&params, &state, 20.0f, 0.0f, 0.005f);
    /* On first sample, d_term is suppressed to prevent spike */
    ASSERT_FLOAT_NEAR(u1, 15.0f * 20.0f, 1e-4f);

    /* Second step with changing error -> derivative active */
    float u2 = balance_controller_calc_turn_pd(&params, &state, 10.0f, 0.0f, 0.005f);
    /* error = 10, derivative = (10 - 20) / 0.005 = -2000, d_term = 0.1 * -2000 = -200 */
    ASSERT_FLOAT_NEAR(u2, 15.0f * 10.0f - 200.0f, 1e-4f);

    printf("[PASS] test_turn_loop_pd\n");
}

static void test_tip_over_protection(void) {
    balance_controller_t ctrl;
    balance_controller_init(&ctrl, NULL);

    /* Safety integration owns explicit enable; reset alone never enables. */
    balance_controller_set_enabled(&ctrl, true);

    balance_controller_inputs_t in;
    balance_controller_outputs_t out;
    memset(&in, 0, sizeof(in));
    in.dt_s = 0.005f;

    /* Upright input */
    in.pitch_deg = 2.0f;
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_TRUE(!out.safety_active);
    ASSERT_TRUE(!balance_controller_is_tipped_over(&ctrl));
    ASSERT_TRUE(out.left_pwm != 0);

    /* Exceed tip-over angle (e.g. 40 deg > 35 deg) */
    in.pitch_deg = 40.0f;
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_TRUE(out.safety_active);
    ASSERT_TRUE(out.left_pwm == 0);
    ASSERT_TRUE(out.right_pwm == 0);
    ASSERT_TRUE(balance_controller_is_tipped_over(&ctrl));

    /* Robot brought back upright, but auto_recovery_enabled is false (latched) */
    in.pitch_deg = 0.0f;
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_TRUE(out.safety_active);
    ASSERT_TRUE(out.left_pwm == 0);
    ASSERT_TRUE(balance_controller_is_tipped_over(&ctrl));

    /* Reset re-enables */
    balance_controller_reset(&ctrl);
    ASSERT_TRUE(!balance_controller_is_tipped_over(&ctrl));
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_TRUE(!out.safety_active);

    printf("[PASS] test_tip_over_protection\n");
}

static void test_gain_update_helpers(void) {
    balance_controller_t ctrl;
    balance_controller_init(&ctrl, NULL);

    balance_controller_set_mechanical_zero(&ctrl, -1.5f);
    ASSERT_FLOAT_NEAR(balance_controller_get_mechanical_zero(&ctrl), -1.5f, 1e-4f);

    ASSERT_TRUE(balance_controller_set_loop_gains(&ctrl, BALANCE_LOOP_ID_BALANCE, 200.0f, 0.0f, 2.5f, 0.0f));
    ASSERT_FLOAT_NEAR(ctrl.config.balance_loop.kp, 200.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(ctrl.config.balance_loop.kd, 2.5f, 1e-4f);

    ASSERT_TRUE(balance_controller_set_loop_gains(&ctrl, BALANCE_LOOP_ID_VELOCITY, 12.0f, 0.5f, 0.0f, 1500.0f));
    ASSERT_FLOAT_NEAR(ctrl.config.velocity_loop.kp, 12.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(ctrl.config.velocity_loop.ki, 0.5f, 1e-4f);
    ASSERT_FLOAT_NEAR(ctrl.config.velocity_loop.integral_limit, 1500.0f, 1e-4f);

    ASSERT_TRUE(balance_controller_set_loop_gains(&ctrl, BALANCE_LOOP_ID_TURN, 20.0f, 0.0f, 0.3f, 800.0f));
    ASSERT_FLOAT_NEAR(ctrl.config.turn_loop.kp, 20.0f, 1e-4f);
    ASSERT_FLOAT_NEAR(ctrl.config.turn_loop.kd, 0.3f, 1e-4f);
    ASSERT_FLOAT_NEAR(ctrl.config.turn_loop.max_output, 800.0f, 1e-4f);

    ASSERT_TRUE(!balance_controller_set_loop_gains(&ctrl, 99, 1.0f, 1.0f, 1.0f, 1.0f));

    printf("[PASS] test_gain_update_helpers\n");
}

static void test_velocity_coupling_modes(void) {
    balance_controller_t ctrl;
    balance_controller_init(&ctrl, NULL);
    balance_controller_set_velocity_gains(&ctrl, 8.0f, 0.4f, 2000.0f);
    balance_controller_set_enabled(&ctrl, true);

    balance_controller_inputs_t in;
    balance_controller_outputs_t out;
    memset(&in, 0, sizeof(in));
    in.dt_s = 0.005f;

    /* 1. Parallel mode: Robot is pushed forward (speed = 50.0, target = 0) */
    /* Upright angle pitch = 0, gyro = 0 */
    in.pitch_deg = 0.0f;
    in.gyro_pitch_dps = 0.0f;
    in.measured_speed_left = 50.0f;
    in.measured_speed_right = 50.0f;
    in.target_speed = 0.0f;
    ctrl.config.velocity_coupling_mode = BALANCE_VEL_COUPLE_PARALLEL;

    balance_controller_update(&ctrl, &in, &out);

    /* In parallel mode, forward movement must command POSITIVE wheel effort (drive forward to brake) */
    ASSERT_TRUE(out.left_pwm > 0);
    ASSERT_TRUE(out.right_pwm > 0);

    /* 2. Cascade mode: Desired forward movement (target = 50.0, speed = 0) */
    balance_controller_reset(&ctrl);
    in.measured_speed_left = 0.0f;
    in.measured_speed_right = 0.0f;
    in.target_speed = 50.0f;
    ctrl.config.velocity_loop.max_output = 10.0f; /* degrees in cascade */
    ctrl.config.velocity_coupling_mode = BALANCE_VEL_COUPLE_TILT_CASCADE;

    balance_controller_update(&ctrl, &in, &out);

    /* In cascade mode, velocity PI outputs dynamic tilt offset (velocity_effort > 0).
     * To catch this forward target tilt, wheels momentarily reverse or accelerate accordingly. */
    ASSERT_TRUE(out.velocity_effort > 0.0f);

    printf("[PASS] test_velocity_coupling_modes\n");
}

static void assert_stopped(const balance_controller_outputs_t *out) {
    ASSERT_TRUE(out->safety_active);
    ASSERT_TRUE(out->left_pwm == 0 && out->right_pwm == 0);
    ASSERT_FLOAT_NEAR(out->balance_effort, 0.0f, 0.0f);
    ASSERT_FLOAT_NEAR(out->velocity_effort, 0.0f, 0.0f);
}

static void test_disabled_and_restart_history(void) {
    balance_controller_t ctrl;
    balance_controller_init(&ctrl, NULL);
    balance_controller_inputs_t in = {.dt_s = 0.005f, .pitch_deg = 2.0f};
    balance_controller_outputs_t out;
    balance_controller_update(&ctrl, &in, &out);
    assert_stopped(&out);
    balance_controller_set_enabled(&ctrl, true);
    balance_controller_set_velocity_gains(&ctrl, 1.0f, 1.0f, 100.0f);
    in.target_speed = 20.0f;
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_TRUE(ctrl.state.velocity.integral > 0.0f);
    balance_controller_set_enabled(&ctrl, false);
    ASSERT_FLOAT_NEAR(ctrl.state.velocity.integral, 0.0f, 0.0f);
    ASSERT_TRUE(!ctrl.state.velocity.is_filter_initialized);
    balance_controller_reset(&ctrl);
    ASSERT_TRUE(!balance_controller_is_enabled(&ctrl));
    balance_controller_set_enabled(&ctrl, true);
    in.measured_speed_left = 20.0f;
    in.measured_speed_right = 20.0f;
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_FLOAT_NEAR(out.velocity_effort, 0.0f, 0.0f);
    printf("[PASS] test_disabled_and_restart_history\n");
}

static void test_zero_gains_never_accumulate(void) {
    balance_controller_t ctrl;
    balance_controller_init(&ctrl, NULL);
    balance_controller_set_enabled(&ctrl, true);
    balance_controller_inputs_t in = {.dt_s = 0.005f, .target_speed = 100.0f};
    balance_controller_outputs_t out;
    for (unsigned i = 0; i < 10000; ++i) {
        balance_controller_update(&ctrl, &in, &out);
    }
    ASSERT_FLOAT_NEAR(ctrl.state.velocity.integral, 0.0f, 0.0f);
    ASSERT_FLOAT_NEAR(out.velocity_effort, 0.0f, 0.0f);
    balance_controller_set_velocity_gains(&ctrl, 0.0f, 2.0f, 100.0f);
    in.target_speed = 0.0f;
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_FLOAT_NEAR(out.velocity_effort, 0.0f, 0.0f);
    ctrl.state.velocity.integral = 99.0f;
    balance_controller_set_velocity_gains(&ctrl, 2.0f, 0.0f, 100.0f);
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_FLOAT_NEAR(ctrl.state.velocity.integral, 0.0f, 0.0f);
    printf("[PASS] test_zero_gains_never_accumulate\n");
}

static void test_pi_output_saturation_and_unwind(void) {
    velocity_pid_params_t params = {.kp = 2.0f, .ki = 1.0f,
        .integral_limit = 100.0f, .max_output = 5.0f};
    velocity_state_t state = {0};
    for (unsigned i = 0; i < 100; ++i) {
        ASSERT_FLOAT_NEAR(balance_controller_calc_velocity_pi(&params, &state, 10.0f, 0.1f), 5.0f, 0.0f);
    }
    ASSERT_FLOAT_NEAR(state.integral, 0.0f, 0.0f);
    state.integral = 10.0f;
    ASSERT_FLOAT_NEAR(balance_controller_calc_velocity_pi(&params, &state, -1.0f, 0.1f), 5.0f, 0.0f);
    ASSERT_FLOAT_NEAR(state.integral, 9.9f, 1e-4f);
    params.integral_limit = 0.0f;
    balance_controller_calc_velocity_pi(&params, &state, 10.0f, 0.1f);
    ASSERT_FLOAT_NEAR(state.integral, 0.0f, 0.0f);
    params.max_output = 0.0f;
    ASSERT_FLOAT_NEAR(balance_controller_calc_velocity_pi(&params, &state, 10.0f, 0.1f), 0.0f, 0.0f);
    printf("[PASS] test_pi_output_saturation_and_unwind\n");
}

static void test_mixed_pwm_anti_windup(void) {
    balance_controller_t ctrl;
    balance_controller_init(&ctrl, NULL);
    ctrl.config.max_pwm = 100.0f;
    ctrl.config.deadband_left = 10.0f;
    ctrl.config.deadband_right = 10.0f;
    balance_controller_set_balance_gains(&ctrl, 20.0f, 0.0f);
    balance_controller_set_velocity_gains(&ctrl, 0.0f, 1.0f, 100.0f);
    balance_controller_set_enabled(&ctrl, true);
    balance_controller_inputs_t in = {.dt_s = 0.1f, .pitch_deg = 5.0f, .target_speed = -10.0f};
    balance_controller_outputs_t out;
    for (unsigned i = 0; i < 100; ++i) {
        balance_controller_update(&ctrl, &in, &out);
        ASSERT_TRUE(out.left_pwm == 100 && out.right_pwm == 100);
    }
    ASSERT_FLOAT_NEAR(ctrl.state.velocity.integral, 0.0f, 0.0f);
    in.target_speed = 10.0f; /* opposite contribution is allowed to unwind saturation */
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_FLOAT_NEAR(ctrl.state.velocity.integral, 1.0f, 1e-4f);

    balance_controller_set_enabled(&ctrl, false);
    ctrl.config.velocity_coupling_mode = BALANCE_VEL_COUPLE_TILT_CASCADE;
    ctrl.config.velocity_loop.max_output = 10.0f;
    balance_controller_reset(&ctrl);
    balance_controller_set_enabled(&ctrl, true);
    in.target_speed = -10.0f;
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_TRUE(!out.safety_active && out.left_pwm == 100);
    ASSERT_FLOAT_NEAR(ctrl.state.velocity.integral, 0.0f, 0.0f);
    printf("[PASS] test_mixed_pwm_anti_windup\n");
}

static void test_all_nonfinite_inputs_fail_closed(void) {
    const float bad_values[] = {NAN, INFINITY, -INFINITY};
    for (unsigned value = 0; value < 3; ++value) {
        for (unsigned field = 0; field < 8; ++field) {
            balance_controller_t ctrl;
            balance_controller_init(&ctrl, NULL);
            balance_controller_set_enabled(&ctrl, true);
            balance_controller_inputs_t in = {.dt_s = 0.005f, .pitch_deg = 2.0f};
            float *fields[] = {&in.pitch_deg, &in.gyro_pitch_dps, &in.gyro_yaw_dps,
                &in.measured_speed_left, &in.measured_speed_right, &in.target_speed,
                &in.target_yaw_rate_dps, &in.dt_s};
            *fields[field] = bad_values[value];
            ctrl.state.velocity.integral = 50.0f;
            balance_controller_outputs_t out;
            memset(&out, 0xff, sizeof(out));
            balance_controller_update(&ctrl, &in, &out);
            assert_stopped(&out);
            ASSERT_FLOAT_NEAR(ctrl.state.velocity.integral, 0.0f, 0.0f);
        }
    }
    balance_controller_outputs_t out;
    memset(&out, 0xff, sizeof(out));
    balance_controller_update(NULL, NULL, &out);
    assert_stopped(&out);
    printf("[PASS] test_all_nonfinite_inputs_fail_closed\n");
}

static void test_parameter_rejection_and_overflow(void) {
    balance_controller_t ctrl;
    balance_controller_init(&ctrl, NULL);
    ASSERT_TRUE(balance_controller_config_is_valid(&ctrl.config));
    ASSERT_TRUE(!balance_controller_set_loop_gains(&ctrl, BALANCE_LOOP_ID_VELOCITY, NAN, 1.0f, 0.0f, 20.0f));
    ASSERT_FLOAT_NEAR(ctrl.config.velocity_loop.kp, 0.0f, 0.0f);
    balance_controller_set_mechanical_zero(&ctrl, INFINITY);
    ASSERT_FLOAT_NEAR(ctrl.config.mechanical_zero_pitch, 0.0f, 0.0f);
    balance_controller_set_velocity_gains(&ctrl, 1.0f, 1.0f, -10.0f);
    ASSERT_FLOAT_NEAR(ctrl.config.velocity_loop.kp, 0.0f, 0.0f);
    balance_controller_inputs_t in = {.dt_s = 0.005f, .pitch_deg = 2.0f};
    balance_controller_outputs_t out;
    ctrl.config.max_pwm = 40000.0f;
    balance_controller_set_enabled(&ctrl, true);
    ASSERT_TRUE(!balance_controller_is_enabled(&ctrl));
    balance_controller_update(&ctrl, &in, &out);
    assert_stopped(&out);
    ctrl.config.max_pwm = 4999.0f;
    balance_controller_set_enabled(&ctrl, true);
    ctrl.config.balance_loop.kp = FLT_MAX;
    balance_controller_update(&ctrl, &in, &out);
    assert_stopped(&out);
    ctrl.config.balance_loop.kp = 180.0f;
    in.dt_s = 0.0f;
    balance_controller_update(&ctrl, &in, &out);
    assert_stopped(&out);
    in.dt_s = 0.6f;
    balance_controller_update(&ctrl, &in, &out);
    assert_stopped(&out);
    ASSERT_FLOAT_NEAR(balance_controller_apply_deadband_and_limit(5.0f, 10.0f, 0.0f), 0.0f, 0.0f);
    ASSERT_FLOAT_NEAR(balance_controller_apply_deadband_and_limit(NAN, 10.0f, 100.0f), 0.0f, 0.0f);
    printf("[PASS] test_parameter_rejection_and_overflow\n");
}

static void test_turn_gain_transition(void) {
    balance_controller_t ctrl;
    balance_controller_init(&ctrl, NULL);
    balance_controller_set_enabled(&ctrl, true);
    balance_controller_inputs_t in = {.dt_s = 0.005f, .target_yaw_rate_dps = 20.0f};
    balance_controller_outputs_t out;
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_TRUE(out.left_pwm < 0 && out.right_pwm > 0);
    balance_controller_set_turn_gains(&ctrl, 1.0f, 10.0f);
    in.target_yaw_rate_dps = -20.0f;
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_FLOAT_NEAR(out.turn_effort, -20.0f, 1e-4f); /* no stale derivative kick */
    printf("[PASS] test_turn_gain_transition\n");
}

static void test_config_and_recovery_boundaries(void) {
    balance_controller_t ctrl;
    balance_controller_init(&ctrl, NULL);
    ctrl.config.velocity_coupling_mode = BALANCE_VEL_COUPLE_TILT_CASCADE;
    ASSERT_TRUE(!balance_controller_config_is_valid(&ctrl.config)); /* PWM limit is not tilt degrees */
    ctrl.config.velocity_loop.max_output = 10.0f;
    ASSERT_TRUE(balance_controller_config_is_valid(&ctrl.config));
    ctrl.config.deadband_left = ctrl.config.max_pwm + 1.0f;
    ASSERT_TRUE(!balance_controller_config_is_valid(&ctrl.config));
    ctrl.config.deadband_left = 0.0f;
    ctrl.config.turn_loop.max_output = NAN;
    ASSERT_TRUE(!balance_controller_config_is_valid(&ctrl.config));

    balance_controller_init(&ctrl, NULL);
    ctrl.config.auto_recovery_enabled = true;
    ctrl.config.max_pitch_angle = 2.0f;
    balance_controller_set_enabled(&ctrl, true);
    balance_controller_inputs_t in = {.dt_s = 0.005f, .pitch_deg = 3.0f};
    balance_controller_outputs_t out;
    balance_controller_update(&ctrl, &in, &out);
    assert_stopped(&out);
    ASSERT_TRUE(ctrl.state.tip_over_count == 1U);
    in.pitch_deg = 1.9f; /* even small cutoffs retain a recovery margin */
    balance_controller_update(&ctrl, &in, &out);
    assert_stopped(&out);
    in.pitch_deg = 0.5f;
    balance_controller_update(&ctrl, &in, &out);
    ASSERT_TRUE(!out.safety_active);
    ASSERT_TRUE(ctrl.state.tip_over_count == 1U);
    printf("[PASS] test_config_and_recovery_boundaries\n");
}

int main(void) {
    printf("=== Starting Balance Controller Unit Tests ===\n");
    test_initialization_and_defaults();
    test_speed_filter();
    test_deadband_and_limits();
    test_balance_loop_pd();
    test_velocity_loop_pi_anti_windup();
    test_turn_loop_pd();
    test_tip_over_protection();
    test_gain_update_helpers();
    test_velocity_coupling_modes();
    test_disabled_and_restart_history();
    test_zero_gains_never_accumulate();
    test_pi_output_saturation_and_unwind();
    test_mixed_pwm_anti_windup();
    test_all_nonfinite_inputs_fail_closed();
    test_parameter_rejection_and_overflow();
    test_turn_gain_transition();
    test_config_and_recovery_boundaries();
    printf("=== All Balance Controller Unit Tests PASSED Successfully! ===\n");
    return 0;
}
