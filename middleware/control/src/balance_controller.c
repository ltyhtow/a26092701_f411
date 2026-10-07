/**
 * @file balance_controller.c
 * @brief Platform-independent, decoupled balance controller implementation.
 */

#include "balance_controller.h"

#include <math.h>
#include <string.h>

/* Floating point deadband threshold to prevent jitter around exact zero */
#define FLOAT_ZERO_EPSILON      1e-4f

/* Safe limits for numerical protection */
#define MIN_DT_SECONDS          1e-4f
#define MAX_DT_SECONDS          0.5f

/* Pitch recovery hysteresis in degrees */
#define TIP_OVER_HYSTERESIS_DEG 5.0f

static float clamp_magnitude(float value, float limit)
{
    return fminf(limit, fmaxf(-limit, value));
}

static bool valid_dt(float dt)
{
    return isfinite(dt) && dt >= MIN_DT_SECONDS && dt <= MAX_DT_SECONDS;
}

bool balance_controller_config_is_valid(const balance_controller_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    return isfinite(config->balance_loop.kp) && isfinite(config->balance_loop.kd) &&
        isfinite(config->velocity_loop.kp) && isfinite(config->velocity_loop.ki) &&
        isfinite(config->velocity_loop.integral_limit) && config->velocity_loop.integral_limit >= 0.0f &&
        isfinite(config->velocity_loop.lpf_alpha) && config->velocity_loop.lpf_alpha >= 0.0f &&
        config->velocity_loop.lpf_alpha <= 1.0f &&
        isfinite(config->velocity_loop.max_output) && config->velocity_loop.max_output >= 0.0f &&
        isfinite(config->turn_loop.kp) && isfinite(config->turn_loop.kd) &&
        isfinite(config->turn_loop.max_output) && config->turn_loop.max_output >= 0.0f &&
        isfinite(config->mechanical_zero_pitch) && isfinite(config->max_pitch_angle) &&
        config->max_pitch_angle > 0.0f && config->max_pitch_angle <= 180.0f &&
        fabsf(config->mechanical_zero_pitch) < config->max_pitch_angle &&
        isfinite(config->max_pwm) && config->max_pwm > 0.0f && config->max_pwm <= INT16_MAX &&
        isfinite(config->deadband_left) && config->deadband_left >= 0.0f &&
        config->deadband_left <= config->max_pwm &&
        isfinite(config->deadband_right) && config->deadband_right >= 0.0f &&
        config->deadband_right <= config->max_pwm &&
        (config->velocity_coupling_mode == BALANCE_VEL_COUPLE_PARALLEL ||
         (config->velocity_coupling_mode == BALANCE_VEL_COUPLE_TILT_CASCADE &&
          config->velocity_loop.max_output < config->max_pitch_angle - fabsf(config->mechanical_zero_pitch)));
}

static void clear_loop_history(balance_controller_t *ctrl)
{
    memset(&ctrl->state.velocity, 0, sizeof(ctrl->state.velocity));
    memset(&ctrl->state.turn, 0, sizeof(ctrl->state.turn));
}

static void reject_update(balance_controller_t *ctrl, balance_controller_outputs_t *outputs)
{
    clear_loop_history(ctrl);
    memset(outputs, 0, sizeof(*outputs));
    outputs->safety_active = true;
}

void balance_controller_default_config(balance_controller_config_t *config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));

    /* 1. Balance PD Loop Defaults
     * Typical tuning: Kp drives restoring torque proportional to angle error;
     * Kd provides angular velocity damping directly from gyro. */
    config->balance_loop.kp = 180.0f;
    config->balance_loop.kd = 1.2f;

    /* 2. Velocity loop remains disabled pending encoder validation and tuning.
     * Encoder acquisition itself remains active; zero gains do not certify it. */
    config->velocity_loop.kp = 0.0f;
    config->velocity_loop.ki = 0.0f;
    config->velocity_loop.integral_limit = 2000.0f;
    config->velocity_loop.lpf_alpha = 0.75f;
    config->velocity_loop.max_output = 1500.0f;

    /* 3. Turn PD Loop Defaults
     * Controls yaw rate differential effort. */
    config->turn_loop.kp = 15.0f;
    config->turn_loop.kd = 0.1f;
    config->turn_loop.max_output = 1000.0f;

    /* 4. Physical Chassis & Hardware Limits */
    config->mechanical_zero_pitch = 0.0f;
    config->max_pitch_angle = 35.0f;
    config->deadband_left = 200.0f;
    config->deadband_right = 200.0f;
    config->max_pwm = 4999.0f;
    config->auto_recovery_enabled = false; /* Safe latching default */

    /* Default coupling mode: Parallel summation */
    config->velocity_coupling_mode = BALANCE_VEL_COUPLE_PARALLEL;
}

void balance_controller_init(balance_controller_t *ctrl, const balance_controller_config_t *config)
{
    if (ctrl == NULL) {
        return;
    }

    if (config != NULL) {
        ctrl->config = *config;
    } else {
        balance_controller_default_config(&ctrl->config);
    }

    /* init accepts uninitialized storage: never preserve bytes from state here. */
    memset(&ctrl->state, 0, sizeof(ctrl->state));
    /* Explicit enable is required after the caller's safety checks. */
}

void balance_controller_reset(balance_controller_t *ctrl)
{
    if (ctrl == NULL) {
        return;
    }

    bool was_enabled = ctrl->state.is_enabled;
    uint32_t tip_count = ctrl->state.tip_over_count;

    memset(&ctrl->state, 0, sizeof(ctrl->state));

    ctrl->state.is_enabled = was_enabled;
    ctrl->state.tip_over_count = tip_count;
    ctrl->state.is_tipped_over = false;
}

void balance_controller_reset_integrators(balance_controller_t *ctrl)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->state.velocity.integral = 0.0f;
    ctrl->state.velocity.last_speed_error = 0.0f;
    ctrl->state.turn.last_yaw_rate_error = 0.0f;
    ctrl->state.turn.has_previous_yaw_error = false;
}

void balance_controller_set_enabled(balance_controller_t *ctrl, bool enable)
{
    if (ctrl == NULL) {
        return;
    }

    if (enable != ctrl->state.is_enabled) {
        clear_loop_history(ctrl);
    }
    if (enable && !ctrl->state.is_enabled) {
        /* Clear tipped over flag on manual enable */
        ctrl->state.is_tipped_over = false;
    }

    ctrl->state.is_enabled = enable && balance_controller_config_is_valid(&ctrl->config);
}

bool balance_controller_is_enabled(const balance_controller_t *ctrl)
{
    return (ctrl != NULL) ? ctrl->state.is_enabled : false;
}

bool balance_controller_is_tipped_over(const balance_controller_t *ctrl)
{
    return (ctrl != NULL) ? ctrl->state.is_tipped_over : true;
}

void balance_controller_set_mechanical_zero(balance_controller_t *ctrl, float zero_pitch_deg)
{
    if (ctrl == NULL || !isfinite(zero_pitch_deg) ||
        fabsf(zero_pitch_deg) >= ctrl->config.max_pitch_angle) {
        return;
    }

    ctrl->config.mechanical_zero_pitch = zero_pitch_deg;
}

float balance_controller_get_mechanical_zero(const balance_controller_t *ctrl)
{
    return (ctrl != NULL) ? ctrl->config.mechanical_zero_pitch : 0.0f;
}

void balance_controller_set_balance_gains(balance_controller_t *ctrl, float kp, float kd)
{
    if (ctrl == NULL || !isfinite(kp) || !isfinite(kd)) {
        return;
    }

    ctrl->config.balance_loop.kp = kp;
    ctrl->config.balance_loop.kd = kd;
}

void balance_controller_set_velocity_gains(balance_controller_t *ctrl, float kp, float ki, float integral_limit)
{
    if (ctrl == NULL || !isfinite(kp) || !isfinite(ki) ||
        !isfinite(integral_limit) || integral_limit < 0.0f) {
        return;
    }

    ctrl->config.velocity_loop.kp = kp;
    ctrl->config.velocity_loop.ki = ki;
    ctrl->config.velocity_loop.integral_limit = integral_limit;
    memset(&ctrl->state.velocity, 0, sizeof(ctrl->state.velocity));
}

void balance_controller_set_turn_gains(balance_controller_t *ctrl, float kp, float kd)
{
    if (ctrl == NULL || !isfinite(kp) || !isfinite(kd)) {
        return;
    }

    ctrl->config.turn_loop.kp = kp;
    ctrl->config.turn_loop.kd = kd;
    memset(&ctrl->state.turn, 0, sizeof(ctrl->state.turn));
}

bool balance_controller_set_loop_gains(balance_controller_t *ctrl,
                                       uint8_t loop_id,
                                       float kp,
                                       float ki,
                                       float kd,
                                       float limit)
{
    if (ctrl == NULL || !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        !isfinite(limit) || limit < 0.0f) {
        return false;
    }

    switch (loop_id) {
    case BALANCE_LOOP_ID_BALANCE:
        balance_controller_set_balance_gains(ctrl, kp, kd);
        return true;

    case BALANCE_LOOP_ID_VELOCITY:
        balance_controller_set_velocity_gains(ctrl, kp, ki, limit);
        return true;

    case BALANCE_LOOP_ID_TURN:
        balance_controller_set_turn_gains(ctrl, kp, kd);
        if (limit > 0.0f) {
            ctrl->config.turn_loop.max_output = limit;
        }
        return true;

    default:
        return false;
    }
}

float balance_controller_filter_speed(float current_filtered, float raw_sample, float alpha)
{
    if (!isfinite(raw_sample)) {
        return isfinite(current_filtered) ? current_filtered : 0.0f;
    }
    if (!isfinite(current_filtered) || !isfinite(alpha)) {
        return raw_sample;
    }

    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    return (alpha * current_filtered) + ((1.0f - alpha) * raw_sample);
}

float balance_controller_calc_balance_pd(const balance_pid_params_t *params,
                                         float pitch_error_deg,
                                         float gyro_pitch_dps)
{
    if (params == NULL) {
        return 0.0f;
    }

    /* Standard PD:
     * Gyro angular velocity is used directly as derivative term to avoid
     * amplifying high-frequency noise inherent in numerical differentiation. */
    return (params->kp * pitch_error_deg) + (params->kd * gyro_pitch_dps);
}

float balance_controller_calc_velocity_pi(const velocity_pid_params_t *params,
                                          velocity_state_t *state,
                                          float speed_error,
                                          float dt_s)
{
    if (params == NULL || state == NULL) {
        return 0.0f;
    }

    if (!isfinite(params->kp) || !isfinite(params->ki) || !isfinite(speed_error) ||
        !isfinite(params->integral_limit) || params->integral_limit < 0.0f ||
        !isfinite(params->max_output) || params->max_output < 0.0f ||
        !isfinite(state->integral) || !valid_dt(dt_s)) {
        state->integral = 0.0f;
        return NAN;
    }

    /* A disabled I term never accumulates hidden charge for later tuning. */
    if (params->ki == 0.0f || params->integral_limit == 0.0f || params->max_output == 0.0f) {
        state->integral = 0.0f;
    }
    float old_integral = clamp_magnitude(state->integral, params->integral_limit);
    float candidate = old_integral;
    if (params->ki != 0.0f && params->max_output > 0.0f) {
        candidate = clamp_magnitude(old_integral + speed_error * dt_s, params->integral_limit);
    }
    float p_term = params->kp * speed_error;
    float effort = p_term + params->ki * candidate;
    if (!isfinite(effort)) {
        state->integral = 0.0f;
        return NAN;
    }

    /* Freeze only integration that pushes farther into the PI output limit;
     * allow the integral to unwind when its contribution opposes saturation. */
    float delta_effort = params->ki * (candidate - old_integral);
    if ((effort > params->max_output && delta_effort > 0.0f) ||
        (effort < -params->max_output && delta_effort < 0.0f)) {
        candidate = old_integral;
        effort = p_term + params->ki * candidate;
    }
    state->integral = candidate;
    state->last_speed_error = speed_error;
    return clamp_magnitude(effort, params->max_output);
}

float balance_controller_calc_turn_pd(const turn_pid_params_t *params,
                                      turn_state_t *state,
                                      float target_yaw_rate_dps,
                                      float measured_yaw_rate_dps,
                                      float dt_s)
{
    if (params == NULL || state == NULL) {
        return 0.0f;
    }

    if (!isfinite(params->kp) || !isfinite(params->kd) ||
        !isfinite(params->max_output) || params->max_output < 0.0f ||
        !isfinite(target_yaw_rate_dps) || !isfinite(measured_yaw_rate_dps) ||
        !isfinite(state->last_yaw_rate_error) || !valid_dt(dt_s)) {
        memset(state, 0, sizeof(*state));
        return NAN;
    }
    if (params->max_output == 0.0f || (params->kp == 0.0f && params->kd == 0.0f)) {
        memset(state, 0, sizeof(*state));
        return 0.0f;
    }

    float error = target_yaw_rate_dps - measured_yaw_rate_dps;
    float p_term = params->kp * error;
    float d_term = 0.0f;

    if (valid_dt(dt_s)) {
        if (state->has_previous_yaw_error) {
            if (params->kd != 0.0f) {
                float derivative = (error - state->last_yaw_rate_error) / dt_s;
                d_term = params->kd * derivative;
            }
        }
        state->has_previous_yaw_error = true;
    }

    state->last_yaw_rate_error = error;

    float effort = p_term + d_term;

    if (!isfinite(effort)) {
        memset(state, 0, sizeof(*state));
        return NAN;
    }
    return clamp_magnitude(effort, params->max_output);
}

float balance_controller_apply_deadband_and_limit(float effort, float deadband, float max_pwm)
{
    if (!isfinite(effort) || !isfinite(deadband) || !isfinite(max_pwm) ||
        deadband < 0.0f || max_pwm < 0.0f || max_pwm > INT16_MAX) {
        return 0.0f;
    }
    float deadband_mag = deadband;
    float max_pwm_mag = max_pwm;

    if (fabsf(effort) <= FLOAT_ZERO_EPSILON) {
        return 0.0f;
    }

    float compensated = effort;
    if (effort > 0.0f) {
        compensated += deadband_mag;
    } else {
        compensated -= deadband_mag;
    }

    return clamp_magnitude(compensated, max_pwm_mag);
}

static bool mix_efforts(const balance_controller_t *ctrl,
                        const balance_controller_inputs_t *inputs,
                        balance_controller_outputs_t *outputs)
{
    float target_pitch = ctrl->config.mechanical_zero_pitch;
    if (ctrl->config.velocity_coupling_mode == BALANCE_VEL_COUPLE_TILT_CASCADE) {
        target_pitch += outputs->velocity_effort;
    }
    outputs->balance_effort = balance_controller_calc_balance_pd(&ctrl->config.balance_loop,
        inputs->pitch_deg - target_pitch, inputs->gyro_pitch_dps);
    float linear_effort = outputs->balance_effort;
    if (ctrl->config.velocity_coupling_mode == BALANCE_VEL_COUPLE_PARALLEL) {
        linear_effort -= outputs->velocity_effort;
    }
    outputs->left_effort_raw = linear_effort - outputs->turn_effort;
    outputs->right_effort_raw = linear_effort + outputs->turn_effort;
    return isfinite(outputs->balance_effort) && isfinite(outputs->velocity_effort) &&
        isfinite(outputs->turn_effort) && isfinite(outputs->left_effort_raw) &&
        isfinite(outputs->right_effort_raw);
}

static bool pushes_saturation(float raw, float deadband, float max_pwm, float change)
{
    float raw_limit = max_pwm - deadband;
    return (raw > raw_limit && change > 0.0f) || (raw < -raw_limit && change < 0.0f);
}

void balance_controller_update(balance_controller_t *ctrl,
                              const balance_controller_inputs_t *inputs,
                              balance_controller_outputs_t *outputs)
{
    if (outputs == NULL) {
        return;
    }
    memset(outputs, 0, sizeof(*outputs));
    outputs->safety_active = true;
    if (ctrl == NULL || inputs == NULL) {
        return;
    }

    /* 1. Numerical Sanity Check */
    if (!balance_controller_config_is_valid(&ctrl->config) ||
        !isfinite(inputs->pitch_deg) || !isfinite(inputs->gyro_pitch_dps) ||
        !isfinite(inputs->gyro_yaw_dps) || !isfinite(inputs->measured_speed_left) ||
        !isfinite(inputs->measured_speed_right) || !isfinite(inputs->target_speed) ||
        !isfinite(inputs->target_yaw_rate_dps) || !valid_dt(inputs->dt_s) ||
        !isfinite(ctrl->state.velocity.integral)) {
        reject_update(ctrl, outputs);
        return;
    }

    /* 2. Tip-over Safety Check */
    float abs_pitch = fabsf(inputs->pitch_deg);
    float max_pitch = ctrl->config.max_pitch_angle;

    if (abs_pitch > max_pitch) {
        if (!ctrl->state.is_tipped_over) {
            ctrl->state.is_tipped_over = true;
            ctrl->state.tip_over_count++;
        }
    } else if (ctrl->state.is_tipped_over && ctrl->config.auto_recovery_enabled) {
        /* Hysteresis: only clear tip-over when comfortably within safety margin */
        float recovery_threshold = fmaxf(max_pitch - TIP_OVER_HYSTERESIS_DEG, 0.5f * max_pitch);
        if (abs_pitch < recovery_threshold) {
            ctrl->state.is_tipped_over = false;
        }
    }

    /* If tipped over or disabled, cut motor output immediately and reset integrators */
    if (ctrl->state.is_tipped_over || !ctrl->state.is_enabled) {
        reject_update(ctrl, outputs);
        return;
    }

    /* The caller must provide a measured valid interval, never a fabricated dt. */
    float dt = inputs->dt_s;

    /* 3. Velocity Loop Calculation
     * 3.1. Filter supplied wheel speeds (F411: X4 counts per nominal 5 ms). */
    float alpha = ctrl->config.velocity_loop.lpf_alpha;
    if (!ctrl->state.velocity.is_filter_initialized) {
        ctrl->state.velocity.filtered_speed_left = inputs->measured_speed_left;
        ctrl->state.velocity.filtered_speed_right = inputs->measured_speed_right;
        ctrl->state.velocity.is_filter_initialized = true;
    } else {
        ctrl->state.velocity.filtered_speed_left =
            balance_controller_filter_speed(ctrl->state.velocity.filtered_speed_left,
                                            inputs->measured_speed_left,
                                            alpha);

        ctrl->state.velocity.filtered_speed_right =
            balance_controller_filter_speed(ctrl->state.velocity.filtered_speed_right,
                                            inputs->measured_speed_right,
                                            alpha);
    }

    ctrl->state.velocity.filtered_speed_avg =
        0.5f * ctrl->state.velocity.filtered_speed_left + 0.5f * ctrl->state.velocity.filtered_speed_right;

    /* 3.2. Speed error: Target speed minus filtered forward speed */
    float speed_error = inputs->target_speed - ctrl->state.velocity.filtered_speed_avg;

    float previous_integral = ctrl->state.velocity.integral;
    outputs->velocity_effort = balance_controller_calc_velocity_pi(&ctrl->config.velocity_loop,
                                                                  &ctrl->state.velocity,
                                                                  speed_error,
                                                                  dt);

    /* 5. Turn Loop Calculation */
    outputs->turn_effort = balance_controller_calc_turn_pd(&ctrl->config.turn_loop,
                                                           &ctrl->state.turn,
                                                           inputs->target_yaw_rate_dps,
                                                           inputs->gyro_yaw_dps,
                                                           dt);

    /* 6. Output Mixer
     * Combine balance, velocity, and turn efforts.
     * In parallel mode, velocity effort is subtracted so that when measured speed
     * exceeds target (speed_error < 0), the resulting positive effort drives wheels
     * forward to induce restoring backward tilt and brake the vehicle. */
    if (!mix_efforts(ctrl, inputs, outputs)) {
        reject_update(ctrl, outputs);
        return;
    }

    /* The actuator limit also includes balance, turn and stiction compensation.
     * Undo this step's integration if it pushes either wheel farther into a
     * saturated command; recompute outputs with the retained integral. */
    float integral_change = ctrl->state.velocity.integral - previous_integral;
    float motor_change = -ctrl->config.velocity_loop.ki * integral_change;
    if (ctrl->config.velocity_coupling_mode == BALANCE_VEL_COUPLE_TILT_CASCADE) {
        motor_change *= ctrl->config.balance_loop.kp;
    }
    if (ctrl->config.velocity_loop.ki != 0.0f &&
        ctrl->config.velocity_loop.integral_limit > 0.0f &&
        ctrl->config.velocity_loop.max_output > 0.0f &&
        (pushes_saturation(outputs->left_effort_raw, ctrl->config.deadband_left,
                           ctrl->config.max_pwm, motor_change) ||
         pushes_saturation(outputs->right_effort_raw, ctrl->config.deadband_right,
                           ctrl->config.max_pwm, motor_change))) {
        ctrl->state.velocity.integral = clamp_magnitude(previous_integral,
            ctrl->config.velocity_loop.integral_limit);
        float retained_effort = ctrl->config.velocity_loop.kp * speed_error +
            ctrl->config.velocity_loop.ki * ctrl->state.velocity.integral;
        if (!isfinite(retained_effort)) {
            reject_update(ctrl, outputs);
            return;
        }
        outputs->velocity_effort = clamp_magnitude(retained_effort, ctrl->config.velocity_loop.max_output);
        if (!mix_efforts(ctrl, inputs, outputs)) {
            reject_update(ctrl, outputs);
            return;
        }
    }

    /* 7. Deadband Compensation & Maximum PWM Saturation */
    float left_final = balance_controller_apply_deadband_and_limit(outputs->left_effort_raw,
                                                                  ctrl->config.deadband_left,
                                                                  ctrl->config.max_pwm);

    float right_final = balance_controller_apply_deadband_and_limit(outputs->right_effort_raw,
                                                                   ctrl->config.deadband_right,
                                                                   ctrl->config.max_pwm);

    outputs->left_pwm = (int16_t)roundf(left_final);
    outputs->right_pwm = (int16_t)roundf(right_final);
    outputs->safety_active = false;
}
