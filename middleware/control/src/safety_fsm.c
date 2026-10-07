/**
 * @file safety_fsm.c
 * @brief Implementation of Safety & Interaction FSM for self-balancing robot.
 */

#include "safety_fsm.h"

#include <math.h>
#include <string.h>

/**
 * @brief Helper: Apply ramp slew-rate limiter.
 * @param current Current value.
 * @param target Desired target value.
 * @param max_accel Maximum acceleration rate limit (rate when speed magnitude increases).
 * @param max_decel Maximum deceleration rate limit (rate when speed magnitude decreases).
 * @param dt_s Elapsed time step in seconds.
 * @return New ramped value.
 */
static float apply_ramp_filter(float current, float target, float max_accel, float max_decel, float dt_s) {
    if (dt_s <= 0.0f) {
        return current;
    }
    if (max_accel <= 0.0f && max_decel <= 0.0f) {
        return target;
    }

    float diff = target - current;
    if (fabsf(diff) < 1e-5f) {
        return target;
    }

    /* Identify deceleration (braking towards 0) vs acceleration (away from 0) */
    bool is_braking = ((current > 0.0f) && (target < current)) ||
                      ((current < 0.0f) && (target > current));

    float rate;
    if (is_braking && max_decel > 0.0f) {
        rate = max_decel;
    } else if (max_accel > 0.0f) {
        rate = max_accel;
    } else if (max_decel > 0.0f) {
        rate = max_decel;
    } else {
        return target;
    }

    float max_step = rate * dt_s;
    if (diff > max_step) {
        return current + max_step;
    } else if (diff < -max_step) {
        return current - max_step;
    } else {
        return target;
    }
}

safety_fsm_config_t safety_fsm_default_config(void) {
    safety_fsm_config_t cfg;
    cfg.pitch_fall_limit_deg       = SAFETY_FSM_DEFAULT_FALL_PITCH_DEG;
    cfg.roll_fall_limit_deg        = SAFETY_FSM_DEFAULT_FALL_ROLL_DEG;
    cfg.pitch_recovery_limit_deg   = SAFETY_FSM_DEFAULT_RECOVERY_PITCH_DEG;
    cfg.roll_recovery_limit_deg    = SAFETY_FSM_DEFAULT_RECOVERY_ROLL_DEG;
    cfg.gyro_steady_limit_dps      = SAFETY_FSM_DEFAULT_STEADY_GYRO_DPS;
    cfg.steady_time_threshold_ms   = SAFETY_FSM_DEFAULT_STEADY_TIME_MS;
    cfg.cmd_timeout_ms             = SAFETY_FSM_DEFAULT_CMD_TIMEOUT_MS;
    cfg.max_linear_accel           = SAFETY_FSM_DEFAULT_MAX_LINEAR_ACCEL;
    cfg.max_linear_decel           = SAFETY_FSM_DEFAULT_MAX_LINEAR_DECEL;
    cfg.max_yaw_accel              = SAFETY_FSM_DEFAULT_MAX_YAW_ACCEL;
    cfg.require_calibration        = true;
    cfg.auto_rearm_enable          = false;
    return cfg;
}

void safety_fsm_init(safety_fsm_t *fsm, const safety_fsm_config_t *config) {
    if (fsm == NULL) {
        return;
    }

    memset(fsm, 0, sizeof(*fsm));

    if (config != NULL) {
        fsm->config = *config;
    } else {
        fsm->config = safety_fsm_default_config();
    }

    fsm->state          = BALANCE_STATE_DISARMED;
    fsm->previous_state = BALANCE_STATE_DISARMED;
    fsm->is_upright     = false;
    fsm->is_steady      = false;
}

balance_state_t safety_fsm_get_state(const safety_fsm_t *fsm) {
    if (fsm == NULL) {
        return BALANCE_STATE_DISARMED;
    }
    return fsm->state;
}

const char *safety_fsm_state_to_str(balance_state_t state) {
    switch (state) {
        case BALANCE_STATE_DISARMED:    return "DISARMED";
        case BALANCE_STATE_CALIBRATING: return "CALIBRATING";
        case BALANCE_STATE_ARMED:       return "ARMED";
        case BALANCE_STATE_FALLEN:      return "FALLEN";
        default:                        return "UNKNOWN";
    }
}

bool safety_fsm_is_motor_enabled(const safety_fsm_t *fsm) {
    return (fsm != NULL && fsm->state == BALANCE_STATE_ARMED);
}

bool safety_fsm_is_upright(const safety_fsm_t *fsm) {
    return (fsm != NULL && fsm->is_upright);
}

bool safety_fsm_is_steady(const safety_fsm_t *fsm) {
    return (fsm != NULL && fsm->is_steady);
}

bool safety_fsm_is_calibrated(const safety_fsm_t *fsm) {
    return (fsm != NULL && fsm->is_calibrated);
}

bool safety_fsm_can_rearm(const safety_fsm_t *fsm) {
    if (fsm == NULL) {
        return false;
    }

    if (fsm->emergency_stop_active) {
        return false;
    }

    /* Communication loss stops motion but retains upright balancing. */
    if (!fsm->attitude_valid ||
        (fsm->fault_flags & ~SAFETY_FAULT_CMD_TIMEOUT) != 0U) {
        return false;
    }

    /* Reject if calibration is required but not performed */
    if (fsm->config.require_calibration && !fsm->is_calibrated) {
        return false;
    }

    /* Must be within upright recovery limits */
    return fsm->is_upright && fsm->is_steady;
}

uint32_t safety_fsm_get_fault_flags(const safety_fsm_t *fsm) {
    if (fsm == NULL) {
        return 0U;
    }
    return fsm->fault_flags;
}

bool safety_fsm_has_fault(const safety_fsm_t *fsm, uint32_t fault_mask) {
    if (fsm == NULL) {
        return false;
    }
    return ((fsm->fault_flags & fault_mask) != 0U);
}

void safety_fsm_feed_heartbeat(safety_fsm_t *fsm, uint32_t timestamp_ms) {
    if (fsm == NULL) {
        return;
    }
    fsm->last_cmd_timestamp_ms = timestamp_ms;
    fsm->has_received_cmd      = true;
    fsm->watchdog_tripped      = false;
    fsm->fault_flags          &= ~SAFETY_FAULT_CMD_TIMEOUT;
}

void safety_fsm_feed_motion_cmd(safety_fsm_t *fsm, float target_linear, float target_yaw, uint32_t timestamp_ms) {
    if (fsm == NULL) {
        return;
    }

    if (isfinite(target_linear) && isfinite(target_yaw)) {
        fsm->target_linear_cmd = (fsm->state == BALANCE_STATE_ARMED) ? target_linear : 0.0f;
        fsm->target_yaw_cmd    = (fsm->state == BALANCE_STATE_ARMED) ? target_yaw : 0.0f;
    } else {
        fsm->target_linear_cmd = 0.0f;
        fsm->target_yaw_cmd    = 0.0f;
        safety_fsm_set_fault(fsm, SAFETY_FAULT_SENSOR_INVALID);
    }

    fsm->last_cmd_timestamp_ms = timestamp_ms;
    fsm->has_received_cmd      = true;
    fsm->watchdog_tripped      = false;
    fsm->fault_flags          &= ~SAFETY_FAULT_CMD_TIMEOUT;
}

void safety_fsm_update_attitude(safety_fsm_t *fsm, float pitch_deg, float roll_deg,
                                float gyro_pitch_dps, float gyro_roll_dps, float gyro_yaw_dps) {
    if (fsm == NULL) {
        return;
    }

    if (!isfinite(pitch_deg) || !isfinite(roll_deg) ||
        !isfinite(gyro_pitch_dps) || !isfinite(gyro_roll_dps) || !isfinite(gyro_yaw_dps)) {
        safety_fsm_set_fault(fsm, SAFETY_FAULT_SENSOR_INVALID);
        return;
    }

    fsm->attitude_valid = true;
    fsm->attitude_sample_pending = true;
    /* Recover health, but a running sensor failure remains latched until reset. */
    if (fsm->state != BALANCE_STATE_FALLEN) {
        fsm->fault_flags &= ~SAFETY_FAULT_SENSOR_INVALID;
    }

    fsm->current_pitch_deg      = pitch_deg;
    fsm->current_roll_deg       = roll_deg;
    fsm->current_gyro_pitch_dps = gyro_pitch_dps;
    fsm->current_gyro_roll_dps  = gyro_roll_dps;
    fsm->current_gyro_yaw_dps   = gyro_yaw_dps;
    fsm->is_upright = fabsf(pitch_deg) <= fsm->config.pitch_recovery_limit_deg &&
                      fabsf(roll_deg) <= fsm->config.roll_recovery_limit_deg;
    if (!fsm->is_upright || fabsf(gyro_pitch_dps) > fsm->config.gyro_steady_limit_dps ||
        fabsf(gyro_roll_dps) > fsm->config.gyro_steady_limit_dps ||
        fabsf(gyro_yaw_dps) > fsm->config.gyro_steady_limit_dps) {
        fsm->is_steady = false;
        fsm->steady_duration_ms = 0U;
    }
}

void safety_fsm_get_motion_output(const safety_fsm_t *fsm, float *out_linear, float *out_yaw) {
    float linear = 0.0f;
    float yaw    = 0.0f;

    if (fsm != NULL && fsm->state == BALANCE_STATE_ARMED) {
        linear = fsm->ramped_linear_vel;
        yaw    = fsm->ramped_yaw_vel;
    }

    if (out_linear != NULL) {
        *out_linear = linear;
    }
    if (out_yaw != NULL) {
        *out_yaw = yaw;
    }
}

bool safety_fsm_request_arm(safety_fsm_t *fsm) {
    if (fsm == NULL) {
        return false;
    }

    if (!safety_fsm_can_rearm(fsm)) {
        return false;
    }

    if (fsm->state == BALANCE_STATE_ARMED) {
        return true;
    }

    /* A fault reset never doubles as permission to restart. */
    if (fsm->state == BALANCE_STATE_DISARMED) {
        /* Clear fall tilt faults on successful arming */
        fsm->fault_flags &= ~(SAFETY_FAULT_FALL_PITCH | SAFETY_FAULT_FALL_ROLL);
        fsm->previous_state = fsm->state;
        fsm->state = BALANCE_STATE_ARMED;
        fsm->state_transition_count++;
        fsm->ramped_linear_vel = 0.0f;
        fsm->ramped_yaw_vel    = 0.0f;
        fsm->target_linear_cmd = 0.0f;
        fsm->target_yaw_cmd = 0.0f;
        fsm->manual_disarm_latched = false;
        return true;
    }

    return false;
}

void safety_fsm_request_disarm(safety_fsm_t *fsm) {
    if (fsm == NULL) {
        return;
    }
    fsm->manual_disarm_latched = true;

    if (fsm->state != BALANCE_STATE_DISARMED && fsm->state != BALANCE_STATE_FALLEN) {
        fsm->previous_state = fsm->state;
        fsm->state = BALANCE_STATE_DISARMED;
        fsm->state_transition_count++;
    }

    fsm->ramped_linear_vel = 0.0f;
    fsm->ramped_yaw_vel    = 0.0f;
    fsm->target_linear_cmd = 0.0f;
    fsm->target_yaw_cmd = 0.0f;
}

bool safety_fsm_request_calibration(safety_fsm_t *fsm) {
    if (fsm == NULL) {
        return false;
    }

    /* Only allowed from DISARMED state */
    if (fsm->state == BALANCE_STATE_DISARMED) {
        fsm->previous_state = fsm->state;
        fsm->state = BALANCE_STATE_CALIBRATING;
        fsm->state_transition_count++;
        fsm->is_calibrated = false;
        fsm->ramped_linear_vel = 0.0f;
        fsm->ramped_yaw_vel    = 0.0f;
        fsm->target_linear_cmd = 0.0f;
        fsm->target_yaw_cmd = 0.0f;
        fsm->steady_duration_ms = 0U;
        fsm->is_steady = false;
        return true;
    }

    return false;
}

void safety_fsm_notify_calibration_done(safety_fsm_t *fsm, bool success) {
    if (fsm == NULL || fsm->state != BALANCE_STATE_CALIBRATING) {
        return;
    }

    if (success) {
        fsm->is_calibrated = true;
        fsm->fault_flags  &= ~SAFETY_FAULT_CALIBRATION_FAIL;
    } else {
        fsm->is_calibrated = false;
        fsm->fault_flags  |= SAFETY_FAULT_CALIBRATION_FAIL;
    }

    fsm->previous_state = fsm->state;
    fsm->state = BALANCE_STATE_DISARMED;
    fsm->state_transition_count++;
    fsm->ramped_linear_vel = 0.0f;
    fsm->ramped_yaw_vel    = 0.0f;
    fsm->steady_duration_ms = 0U;
    fsm->is_steady = false;
}

bool safety_fsm_reset_fault(safety_fsm_t *fsm) {
    if (fsm == NULL) {
        return false;
    }

    if (!fsm->attitude_valid || !fsm->is_upright || !fsm->is_steady ||
        fsm->state == BALANCE_STATE_CALIBRATING) {
        return false;
    }

    if (fsm->state == BALANCE_STATE_FALLEN) {
        /* Recovery check: must be upright to reset from FALLEN */
        bool upright = (fabsf(fsm->current_pitch_deg) <= fsm->config.pitch_recovery_limit_deg) &&
                       (fabsf(fsm->current_roll_deg) <= fsm->config.roll_recovery_limit_deg);
        if (!upright) {
            return false;
        }

        /* Clear fall faults and transition to DISARMED */
        fsm->fault_flags &= ~(SAFETY_FAULT_FALL_PITCH | SAFETY_FAULT_FALL_ROLL);
        fsm->previous_state = fsm->state;
        fsm->state = BALANCE_STATE_DISARMED;
        fsm->state_transition_count++;
        fsm->ramped_linear_vel = 0.0f;
        fsm->ramped_yaw_vel    = 0.0f;
    }

    /* Clear transient and operator-resettable faults */
    fsm->fault_flags &= ~(SAFETY_FAULT_CMD_TIMEOUT | SAFETY_FAULT_CALIBRATION_FAIL |
                          SAFETY_FAULT_EMERGENCY_STOP | SAFETY_FAULT_SENSOR_INVALID);
    fsm->emergency_stop_active = false;
    safety_fsm_request_disarm(fsm);
    return true;
}

void safety_fsm_set_fault(safety_fsm_t *fsm, uint32_t fault_flag) {
    if (fsm == NULL) {
        return;
    }
    fsm->fault_flags |= fault_flag;
    if ((fault_flag & SAFETY_FAULT_SENSOR_INVALID) != 0U) {
        fsm->attitude_valid = false;
        fsm->attitude_sample_pending = false;
        fsm->is_upright = false;
        fsm->is_steady = false;
        fsm->steady_duration_ms = 0U;
    }
    if ((fault_flag & ~SAFETY_FAULT_CMD_TIMEOUT) != 0U) {
        if (fsm->state == BALANCE_STATE_ARMED) {
            fsm->previous_state = fsm->state;
            fsm->state = BALANCE_STATE_FALLEN;
            fsm->state_transition_count++;
        }
        fsm->target_linear_cmd = 0.0f;
        fsm->target_yaw_cmd = 0.0f;
        fsm->ramped_linear_vel = 0.0f;
        fsm->ramped_yaw_vel = 0.0f;
    }
}

void safety_fsm_clear_fault(safety_fsm_t *fsm, uint32_t fault_flag) {
    if (fsm == NULL) {
        return;
    }
    fsm->fault_flags &= ~fault_flag;
}

void safety_fsm_emergency_stop(safety_fsm_t *fsm) {
    if (fsm == NULL) {
        return;
    }

    fsm->emergency_stop_active = true;
    fsm->fault_flags |= SAFETY_FAULT_EMERGENCY_STOP;

    if (fsm->state == BALANCE_STATE_ARMED || fsm->state == BALANCE_STATE_CALIBRATING) {
        fsm->previous_state = fsm->state;
        fsm->state = BALANCE_STATE_FALLEN;
        fsm->state_transition_count++;
    }

    fsm->ramped_linear_vel = 0.0f;
    fsm->ramped_yaw_vel    = 0.0f;
    fsm->target_linear_cmd = 0.0f;
    fsm->target_yaw_cmd = 0.0f;
}

void safety_fsm_step(safety_fsm_t *fsm, float dt_s, uint32_t current_time_ms) {
    if (fsm == NULL) {
        return;
    }

    if (dt_s <= 0.0f || !isfinite(dt_s)) {
        dt_s = 0.005f; /* Default fallback to 200Hz interval */
    } else if (dt_s > 0.1f) {
        dt_s = 0.1f;   /* Clamp to prevent massive jumps on task stall/debug */
    }

    /* 1. Evaluate Upright Status */
    fsm->is_upright = fsm->attitude_valid &&
                      (fabsf(fsm->current_pitch_deg) <= fsm->config.pitch_recovery_limit_deg) &&
                      (fabsf(fsm->current_roll_deg) <= fsm->config.roll_recovery_limit_deg);

    /* 2. Evaluate Steady Condition */
    bool rates_low = (fabsf(fsm->current_gyro_pitch_dps) <= fsm->config.gyro_steady_limit_dps) &&
                     (fabsf(fsm->current_gyro_roll_dps) <= fsm->config.gyro_steady_limit_dps) &&
                     (fabsf(fsm->current_gyro_yaw_dps) <= fsm->config.gyro_steady_limit_dps);

    if (rates_low && fsm->is_upright && fsm->attitude_sample_pending) {
        uint32_t dt_ms = (uint32_t)(dt_s * 1000.0f);
        if (dt_ms == 0U) {
            dt_ms = 1U;
        }
        if (fsm->steady_duration_ms < fsm->config.steady_time_threshold_ms) {
            fsm->steady_duration_ms += dt_ms;
        }
        if (fsm->steady_duration_ms >= fsm->config.steady_time_threshold_ms) {
            fsm->is_steady = true;
        }
    } else {
        fsm->steady_duration_ms = 0U;
        fsm->is_steady = false;
    }
    fsm->attitude_sample_pending = false;

    /* 3. Evaluate Fall Protection */
    bool fall_detected = false;
    if (fabsf(fsm->current_pitch_deg) > fsm->config.pitch_fall_limit_deg) {
        fsm->fault_flags |= SAFETY_FAULT_FALL_PITCH;
        fall_detected = true;
    }
    if (fabsf(fsm->current_roll_deg) > fsm->config.roll_fall_limit_deg) {
        fsm->fault_flags |= SAFETY_FAULT_FALL_ROLL;
        fall_detected = true;
    }

    if (fall_detected && (fsm->state == BALANCE_STATE_ARMED || fsm->state == BALANCE_STATE_CALIBRATING)) {
        fsm->previous_state = fsm->state;
        fsm->state = BALANCE_STATE_FALLEN;
        fsm->state_transition_count++;
        fsm->fall_event_count++;
        fsm->ramped_linear_vel = 0.0f;
        fsm->ramped_yaw_vel    = 0.0f;
        fsm->target_linear_cmd = 0.0f;
        fsm->target_yaw_cmd = 0.0f;
    }

    /* If not tilted and in DISARMED state, clear transient fall flags */
    if (!fall_detected && fsm->state == BALANCE_STATE_DISARMED && fsm->is_upright) {
        fsm->fault_flags &= ~(SAFETY_FAULT_FALL_PITCH | SAFETY_FAULT_FALL_ROLL);
    }

    /* 4. Evaluate Communication Watchdog */
    if (fsm->config.cmd_timeout_ms > 0U) {
        bool timeout = false;
        if (!fsm->has_received_cmd) {
            if (current_time_ms > fsm->config.cmd_timeout_ms) {
                timeout = true;
            }
        } else {
            uint32_t elapsed_ms = current_time_ms - fsm->last_cmd_timestamp_ms;
            if (elapsed_ms > fsm->config.cmd_timeout_ms) {
                timeout = true;
            }
        }

        if (timeout) {
            if (!fsm->watchdog_tripped) {
                fsm->watchdog_tripped = true;
                fsm->watchdog_trip_count++;
            }
            fsm->fault_flags |= SAFETY_FAULT_CMD_TIMEOUT;
            /* A subsequent heartbeat must not resurrect an expired target. */
            fsm->target_linear_cmd = 0.0f;
            fsm->target_yaw_cmd = 0.0f;
        } else {
            fsm->watchdog_tripped = false;
            fsm->fault_flags &= ~SAFETY_FAULT_CMD_TIMEOUT;
        }
    }

    /* 5. Motion Slew Rate Limiter (Ramp Controller) */
    if (fsm->state == BALANCE_STATE_ARMED) {
        /* When watchdog expires, target velocities are forced to zero to smoothly stop */
        float effective_linear = fsm->watchdog_tripped ? 0.0f : fsm->target_linear_cmd;
        float effective_yaw    = fsm->watchdog_tripped ? 0.0f : fsm->target_yaw_cmd;

        fsm->ramped_linear_vel = apply_ramp_filter(fsm->ramped_linear_vel,
                                                   effective_linear,
                                                   fsm->config.max_linear_accel,
                                                   fsm->config.max_linear_decel,
                                                   dt_s);

        fsm->ramped_yaw_vel = apply_ramp_filter(fsm->ramped_yaw_vel,
                                                effective_yaw,
                                                fsm->config.max_yaw_accel,
                                                fsm->config.max_yaw_accel,
                                                dt_s);
    } else {
        /* Motors disabled: immediate cutoff to prevent runaway or jump on re-arm */
        fsm->ramped_linear_vel = 0.0f;
        fsm->ramped_yaw_vel    = 0.0f;
    }

    /* 6. Auto-recovery handling (optional policy) */
    if (fsm->state == BALANCE_STATE_FALLEN && fsm->config.auto_rearm_enable &&
        !fsm->manual_disarm_latched) {
        if (fsm->is_upright && fsm->is_steady &&
            !fsm->emergency_stop_active &&
            fsm->attitude_valid &&
            (!fsm->config.require_calibration || fsm->is_calibrated) &&
            ((fsm->fault_flags & ~(SAFETY_FAULT_FALL_PITCH | SAFETY_FAULT_FALL_ROLL |
                                   SAFETY_FAULT_CMD_TIMEOUT)) == 0U)) {
            (void)safety_fsm_reset_fault(fsm);
            (void)safety_fsm_request_arm(fsm);
        }
    }

    fsm->last_step_timestamp_ms = current_time_ms;
}
