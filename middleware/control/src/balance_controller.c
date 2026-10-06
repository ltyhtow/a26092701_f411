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
#define DEFAULT_FALLBACK_DT     0.005f

/* Pitch recovery hysteresis in degrees */
#define TIP_OVER_HYSTERESIS_DEG 5.0f

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

    /* 2. Velocity PI Loop Defaults
     * Note: Encoders are disabled / zeroed due to 2.0V rail degradation.
     * Kp and Ki set to 0.0f to operate in pure upright balance PD mode
     * and prevent integrator windup from missing pulse feedback. */
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

    balance_controller_reset(ctrl);
    ctrl->state.is_enabled = true;
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

    if (!enable && ctrl->state.is_enabled) {
        balance_controller_reset_integrators(ctrl);
    } else if (enable && !ctrl->state.is_enabled) {
        /* Clear tipped over flag on manual enable */
        ctrl->state.is_tipped_over = false;
        balance_controller_reset_integrators(ctrl);
    }

    ctrl->state.is_enabled = enable;
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
    if (ctrl == NULL) {
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
    if (ctrl == NULL) {
        return;
    }

    ctrl->config.balance_loop.kp = kp;
    ctrl->config.balance_loop.kd = kd;
}

void balance_controller_set_velocity_gains(balance_controller_t *ctrl, float kp, float ki, float integral_limit)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->config.velocity_loop.kp = kp;
    ctrl->config.velocity_loop.ki = ki;
    ctrl->config.velocity_loop.integral_limit = fabsf(integral_limit);
}

void balance_controller_set_turn_gains(balance_controller_t *ctrl, float kp, float kd)
{
    if (ctrl == NULL) {
        return;
    }

    ctrl->config.turn_loop.kp = kp;
    ctrl->config.turn_loop.kd = kd;
}

bool balance_controller_set_loop_gains(balance_controller_t *ctrl,
                                       uint8_t loop_id,
                                       float kp,
                                       float ki,
                                       float kd,
                                       float limit)
{
    if (ctrl == NULL) {
        return false;
    }

    switch (loop_id) {
    case BALANCE_LOOP_ID_BALANCE:
        ctrl->config.balance_loop.kp = kp;
        ctrl->config.balance_loop.kd = kd;
        return true;

    case BALANCE_LOOP_ID_VELOCITY:
        ctrl->config.velocity_loop.kp = kp;
        ctrl->config.velocity_loop.ki = ki;
        if (limit > 0.0f) {
            ctrl->config.velocity_loop.integral_limit = limit;
        }
        return true;

    case BALANCE_LOOP_ID_TURN:
        ctrl->config.turn_loop.kp = kp;
        ctrl->config.turn_loop.kd = kd;
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
    if (isnan(raw_sample) || isinf(raw_sample)) {
        return current_filtered;
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

    float p_term = params->kp * speed_error;

    /* Integrate with time step */
    if (dt_s > MIN_DT_SECONDS && dt_s < MAX_DT_SECONDS) {
        state->integral += speed_error * dt_s;
    }

    /* Anti-windup clamping */
    float limit = fabsf(params->integral_limit);
    if (limit > FLOAT_ZERO_EPSILON) {
        if (state->integral > limit) {
            state->integral = limit;
        } else if (state->integral < -limit) {
            state->integral = -limit;
        }
    }

    float i_term = params->ki * state->integral;
    float effort = p_term + i_term;

    /* Output saturation limit */
    if (params->max_output > FLOAT_ZERO_EPSILON) {
        if (effort > params->max_output) {
            effort = params->max_output;
        } else if (effort < -params->max_output) {
            effort = -params->max_output;
        }
    }

    state->last_speed_error = speed_error;
    return effort;
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

    float error = target_yaw_rate_dps - measured_yaw_rate_dps;
    float p_term = params->kp * error;
    float d_term = 0.0f;

    if (dt_s > MIN_DT_SECONDS && dt_s < MAX_DT_SECONDS) {
        if (state->has_previous_yaw_error) {
            float derivative = (error - state->last_yaw_rate_error) / dt_s;
            d_term = params->kd * derivative;
        }
        state->has_previous_yaw_error = true;
    }

    state->last_yaw_rate_error = error;

    float effort = p_term + d_term;

    /* Output saturation limit */
    if (params->max_output > FLOAT_ZERO_EPSILON) {
        if (effort > params->max_output) {
            effort = params->max_output;
        } else if (effort < -params->max_output) {
            effort = -params->max_output;
        }
    }

    return effort;
}

float balance_controller_apply_deadband_and_limit(float effort, float deadband, float max_pwm)
{
    float deadband_mag = fabsf(deadband);
    float max_pwm_mag = fabsf(max_pwm);

    if (fabsf(effort) <= FLOAT_ZERO_EPSILON) {
        return 0.0f;
    }

    float compensated = effort;
    if (effort > 0.0f) {
        compensated += deadband_mag;
    } else {
        compensated -= deadband_mag;
    }

    if (max_pwm_mag > FLOAT_ZERO_EPSILON) {
        if (compensated > max_pwm_mag) {
            compensated = max_pwm_mag;
        } else if (compensated < -max_pwm_mag) {
            compensated = -max_pwm_mag;
        }
    }

    return compensated;
}

void balance_controller_update(balance_controller_t *ctrl,
                              const balance_controller_inputs_t *inputs,
                              balance_controller_outputs_t *outputs)
{
    if (ctrl == NULL || inputs == NULL || outputs == NULL) {
        return;
    }

    memset(outputs, 0, sizeof(*outputs));

    /* 1. Numerical Sanity Check */
    if (isnan(inputs->pitch_deg) || isnan(inputs->gyro_pitch_dps) || isnan(inputs->gyro_yaw_dps) ||
        isinf(inputs->pitch_deg) || isinf(inputs->gyro_pitch_dps) || isinf(inputs->gyro_yaw_dps)) {
        outputs->safety_active = true;
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
        float recovery_threshold = max_pitch - TIP_OVER_HYSTERESIS_DEG;
        if (recovery_threshold < 5.0f) {
            recovery_threshold = 5.0f;
        }
        if (abs_pitch < recovery_threshold) {
            ctrl->state.is_tipped_over = false;
        }
    }

    /* If tipped over or disabled, cut motor output immediately and reset integrators */
    if (ctrl->state.is_tipped_over || !ctrl->state.is_enabled) {
        balance_controller_reset_integrators(ctrl);
        outputs->safety_active = true;
        return;
    }

    /* Sanitize loop delta time */
    float dt = inputs->dt_s;
    if (dt < MIN_DT_SECONDS || dt > MAX_DT_SECONDS || isnan(dt)) {
        dt = DEFAULT_FALLBACK_DT;
    }

    /* 3. Velocity Loop Calculation
     * 3.1. 1st-order Low-Pass Filter on raw wheel encoder speeds */
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
        0.5f * (ctrl->state.velocity.filtered_speed_left + ctrl->state.velocity.filtered_speed_right);

    /* 3.2. Speed error: Target speed minus filtered forward speed */
    float speed_error = inputs->target_speed - ctrl->state.velocity.filtered_speed_avg;

    outputs->velocity_effort = balance_controller_calc_velocity_pi(&ctrl->config.velocity_loop,
                                                                  &ctrl->state.velocity,
                                                                  speed_error,
                                                                  dt);

    /* 4. Balance Loop Calculation
     * Determine target pitch based on coupling mode */
    float target_pitch = ctrl->config.mechanical_zero_pitch;

    if (ctrl->config.velocity_coupling_mode == BALANCE_VEL_COUPLE_TILT_CASCADE) {
        /* In cascade mode, velocity PI outputs dynamic tilt angle offset.
         * Leaning forward (positive target angle) drives wheels forward to catch tilt. */
        target_pitch += outputs->velocity_effort;
    }

    float pitch_error = inputs->pitch_deg - target_pitch;

    outputs->balance_effort = balance_controller_calc_balance_pd(&ctrl->config.balance_loop,
                                                                pitch_error,
                                                                inputs->gyro_pitch_dps);

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
    float linear_effort = outputs->balance_effort;
    if (ctrl->config.velocity_coupling_mode == BALANCE_VEL_COUPLE_PARALLEL) {
        linear_effort -= outputs->velocity_effort;
    }

    /* Differential steering: Left = Linear - Turn, Right = Linear + Turn */
    outputs->left_effort_raw = linear_effort - outputs->turn_effort;
    outputs->right_effort_raw = linear_effort + outputs->turn_effort;

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
