#include "balance_runtime.h"
#include <limits.h>
#include <math.h>
#include <string.h>

static bool healthy_attitude(const attitude_sample_t *attitude, uint32_t now) {
    const uint16_t required = ATTITUDE_STATUS_ATTITUDE_VALID | ATTITUDE_STATUS_CALIBRATED;
    const uint16_t rejected = ATTITUDE_STATUS_INPUT_INVALID | ATTITUDE_STATUS_STARTUP | ATTITUDE_STATUS_CALIBRATING;
    return (uint32_t)(now - attitude->timestamp_ms) <= BALANCE_RUNTIME_MAX_AGE_MS &&
           (attitude->status_flags & required) == required && (attitude->status_flags & rejected) == 0U &&
           isfinite(attitude->pitch_deg) && isfinite(attitude->roll_deg) &&
           isfinite(attitude->gyro_dps[0]) && isfinite(attitude->gyro_dps[1]) && isfinite(attitude->gyro_dps[2]);
}
static void stopped_outputs(balance_runtime_t *runtime) {
    memset(&runtime->snapshot.outputs, 0, sizeof(runtime->snapshot.outputs));
    runtime->snapshot.outputs.safety_active = true;
}
static void publish_state(balance_runtime_t *runtime) {
    runtime->snapshot.attitude = runtime->attitude;
    runtime->snapshot.timestamp_ms = runtime->cycle_ms;
    runtime->snapshot.state = safety_fsm_get_state(&runtime->safety);
    runtime->snapshot.fault_flags = safety_fsm_get_fault_flags(&runtime->safety);
}
void balance_runtime_init(balance_runtime_t *runtime, const balance_controller_config_t *config) {
    if (runtime == NULL) return;
    memset(runtime, 0, sizeof(*runtime));
    balance_controller_init(&runtime->controller, config);
    safety_fsm_init(&runtime->safety, NULL);
    stopped_outputs(runtime);
    publish_state(runtime);
}
void balance_runtime_begin_cycle(balance_runtime_t *runtime, uint32_t now_ms) {
    runtime->elapsed_ms = runtime->have_cycle ? now_ms - runtime->last_cycle_ms : BALANCE_RUNTIME_PERIOD_MS;
    runtime->deadline_missed = runtime->elapsed_ms == 0U || runtime->elapsed_ms > BALANCE_RUNTIME_MAX_AGE_MS;
    runtime->dt_s = runtime->deadline_missed ? 0.005f : (float)runtime->elapsed_ms * 0.001f;
    runtime->cycle_ms = runtime->last_cycle_ms = now_ms;
    runtime->have_cycle = true;
    runtime->requests = 0U;
    runtime->new_attitude = runtime->bad_sample = runtime->have_motion = false;
    runtime->snapshot.calibration_request = false;
    if (runtime->calibration_pending &&
        (uint32_t)(now_ms - runtime->calibration_requested_ms) > BALANCE_RUNTIME_CALIBRATION_TIMEOUT_MS) {
        runtime->calibration_pending = runtime->calibration_started = false;
        safety_fsm_request_disarm(&runtime->safety);
        safety_fsm_set_fault(&runtime->safety, SAFETY_FAULT_CALIBRATION_FAIL);
    }
}
void balance_runtime_accept_attitude(balance_runtime_t *runtime, const attitude_sample_t *sample) {
    if (sample == NULL) return;
    if (!healthy_attitude(sample, runtime->cycle_ms)) runtime->bad_sample = true;
    const uint32_t delta = sample->timestamp_ms - runtime->attitude.timestamp_ms;
    if (runtime->have_attitude && (delta == 0U || delta > INT32_MAX)) return;
    runtime->attitude = *sample;
    runtime->have_attitude = runtime->new_attitude = true;
    if ((uint32_t)(runtime->cycle_ms - sample->timestamp_ms) <= BALANCE_RUNTIME_MAX_AGE_MS &&
        (sample->status_flags & ATTITUDE_STATUS_CALIBRATING) != 0U &&
        (!runtime->calibration_pending ||
         (uint32_t)(sample->timestamp_ms - runtime->calibration_requested_ms) <= INT32_MAX))
        runtime->calibration_started = true;
}
bool balance_runtime_apply_config(balance_runtime_t *runtime, const balance_controller_config_t *config) {
    if (runtime == NULL || safety_fsm_get_state(&runtime->safety) != BALANCE_STATE_DISARMED ||
        runtime->calibration_pending || !balance_controller_config_is_valid(config)) return false;
    balance_controller_init(&runtime->controller, config);
    stopped_outputs(runtime);
    return true;
}
bool balance_runtime_accept_pid(balance_runtime_t *runtime, const pid_request_t *request) {
    if (runtime == NULL || request == NULL || !isfinite(request->kp) || !isfinite(request->ki) ||
        !isfinite(request->kd) || !isfinite(request->integral_limit) || request->integral_limit < 0.0f) return false;
    balance_controller_config_t candidate = runtime->controller.config;
    switch (request->loop_id) {
        case 0U: candidate.balance_loop.kp = request->kp; candidate.balance_loop.kd = request->kd; break;
        case 1U: candidate.velocity_loop.kp = request->kp; candidate.velocity_loop.ki = request->ki;
                 candidate.velocity_loop.integral_limit = request->integral_limit; break;
        case 2U: candidate.turn_loop.kp = request->kp; candidate.turn_loop.kd = request->kd; break;
        default: return false;
    }
    return balance_runtime_apply_config(runtime, &candidate);
}
void balance_runtime_request_flags(balance_runtime_t *runtime, uint32_t requests) { runtime->requests |= requests; }
void balance_runtime_request_system(balance_runtime_t *runtime, const system_request_t *request) {
    if (request == NULL) return;
    switch (request->action) {
        case SYSTEM_ACTION_ARM: runtime->requests |= BALANCE_REQUEST_ARM; break;
        case SYSTEM_ACTION_DISARM: runtime->requests |= BALANCE_REQUEST_DISARM; break;
        case SYSTEM_ACTION_RESET: runtime->requests |= BALANCE_REQUEST_RESET; break;
        case SYSTEM_ACTION_CALIBRATE: runtime->requests |= BALANCE_REQUEST_CALIBRATE; break;
        default: break;
    }
}
void balance_runtime_accept_motion(balance_runtime_t *runtime, const motion_request_t *request) {
    if (request == NULL) return;
    /* Even an invalid motion value cannot discard an explicit stop/reset. */
    if (!request->enable) runtime->requests |= BALANCE_REQUEST_DISARM;
    else if (!runtime->motion_enable_seen) runtime->requests |= BALANCE_REQUEST_ARM;
    runtime->motion_enable_seen = request->enable;
    if (request->clear_fault) runtime->requests |= BALANCE_REQUEST_RESET;
    if (!isfinite(request->linear_counts_per_5ms) || !isfinite(request->yaw_rate_dps)) {
        runtime->requests |= BALANCE_REQUEST_DISARM;
        return;
    }
    runtime->motion = *request;
    runtime->have_motion = true;
}
void balance_runtime_finish_cycle(balance_runtime_t *runtime, float left_counts, float right_counts) {
    safety_fsm_t *safety = &runtime->safety;
    runtime->attitude_valid = runtime->have_attitude && healthy_attitude(&runtime->attitude, runtime->cycle_ms) && !runtime->deadline_missed;
    if (runtime->bad_sample || !runtime->attitude_valid) safety_fsm_set_fault(safety, SAFETY_FAULT_SENSOR_INVALID);
    if (runtime->new_attitude && runtime->attitude_valid) {
        if (runtime->calibration_pending) {
            if (runtime->calibration_started) {
                safety_fsm_notify_calibration_done(safety, true);
                runtime->calibration_pending = false;
            }
        } else safety->is_calibrated = true;
        safety_fsm_update_attitude(safety, runtime->attitude.pitch_deg, runtime->attitude.roll_deg,
                                  runtime->attitude.gyro_dps[1], runtime->attitude.gyro_dps[0], runtime->attitude.gyro_dps[2]);
    }
    if (!runtime->attitude_valid) safety->is_calibrated = false;
    if (runtime->elapsed_ms > 0U) {
        const float scale = (float)BALANCE_RUNTIME_PERIOD_MS / (float)runtime->elapsed_ms;
        left_counts *= scale;
        right_counts *= scale;
    }
    runtime->snapshot.left_counts_per_5ms = left_counts;
    runtime->snapshot.right_counts_per_5ms = right_counts;
    if (!isfinite(left_counts) || !isfinite(right_counts)) safety_fsm_set_fault(safety, SAFETY_FAULT_SENSOR_INVALID);
    safety_fsm_step(safety, runtime->dt_s, runtime->cycle_ms);

    const uint32_t requests = runtime->requests;
    if ((requests & BALANCE_REQUEST_ESTOP) != 0U) safety_fsm_emergency_stop(safety);
    else if ((requests & BALANCE_REQUEST_DISARM) != 0U) {
        safety_fsm_request_disarm(safety);
        if ((requests & BALANCE_REQUEST_RESET) != 0U) (void)safety_fsm_reset_fault(safety);
    } else if ((requests & BALANCE_REQUEST_CALIBRATE) != 0U) {
        safety_fsm_request_disarm(safety);
        runtime->calibration_pending = runtime->calibration_started = false;
        safety->is_calibrated = false;
        if (safety_fsm_request_calibration(safety)) {
            runtime->calibration_pending = true;
            runtime->calibration_requested_ms = runtime->cycle_ms;
            runtime->snapshot.calibration_request = true;
        } else safety_fsm_set_fault(safety, SAFETY_FAULT_CALIBRATION_FAIL);
    } else if ((requests & BALANCE_REQUEST_RESET) != 0U) {
        safety_fsm_request_disarm(safety);
        (void)safety_fsm_reset_fault(safety);
    } else if ((requests & BALANCE_REQUEST_ARM) != 0U) {
        if (safety_fsm_request_arm(safety)) safety_fsm_feed_heartbeat(safety, runtime->cycle_ms);
    }
    if (runtime->have_motion && (requests & BALANCE_REQUEST_STOP_MASK) == 0U) {
        const motion_request_t *motion = &runtime->motion;
        safety->config.cmd_timeout_ms = motion->timeout_ms > 0U && motion->timeout_ms < SAFETY_FSM_DEFAULT_CMD_TIMEOUT_MS ?
                                       motion->timeout_ms : SAFETY_FSM_DEFAULT_CMD_TIMEOUT_MS;
        safety_fsm_feed_motion_cmd(safety, motion->linear_counts_per_5ms, motion->yaw_rate_dps, runtime->cycle_ms);
    }
    stopped_outputs(runtime);
    if (safety_fsm_is_motor_enabled(safety) && runtime->attitude_valid) {
        float linear, yaw;
        safety_fsm_get_motion_output(safety, &linear, &yaw);
        const balance_controller_inputs_t inputs = {
            .pitch_deg = runtime->attitude.pitch_deg, .gyro_pitch_dps = runtime->attitude.gyro_dps[1],
            .gyro_yaw_dps = runtime->attitude.gyro_dps[2],
            .measured_speed_left = left_counts, .measured_speed_right = right_counts,
            .target_speed = linear, .target_yaw_rate_dps = yaw, .dt_s = runtime->dt_s
        };
        balance_controller_set_enabled(&runtime->controller, true);
        balance_controller_update(&runtime->controller, &inputs, &runtime->snapshot.outputs);
        if (runtime->snapshot.outputs.safety_active || balance_controller_is_tipped_over(&runtime->controller)) {
            safety_fsm_set_fault(safety, SAFETY_FAULT_SENSOR_INVALID);
            stopped_outputs(runtime);
        }
    }
    if (!safety_fsm_is_motor_enabled(safety)) {
        balance_controller_set_enabled(&runtime->controller, false);
        balance_controller_reset_integrators(&runtime->controller);
    }
    publish_state(runtime);
}
void balance_runtime_calibration_result(balance_runtime_t *runtime, bool accepted) {
    if (!runtime->snapshot.calibration_request) return;
    runtime->snapshot.calibration_request = false;
    if (!accepted) {
        runtime->calibration_pending = runtime->calibration_started = false;
        safety_fsm_notify_calibration_done(&runtime->safety, false);
        stopped_outputs(runtime);
        publish_state(runtime);
    }
}
void balance_runtime_prepare_output(balance_runtime_t *runtime, uint32_t dispatch_ms, uint32_t pending_requests) {
    if ((uint32_t)(dispatch_ms - runtime->cycle_ms) > BALANCE_RUNTIME_MAX_AGE_MS || !runtime->have_attitude ||
        (uint32_t)(dispatch_ms - runtime->attitude.timestamp_ms) > BALANCE_RUNTIME_MAX_AGE_MS) {
        safety_fsm_set_fault(&runtime->safety, SAFETY_FAULT_SENSOR_INVALID);
        balance_controller_set_enabled(&runtime->controller, false);
        balance_controller_reset_integrators(&runtime->controller);
        runtime->attitude_valid = false;
        stopped_outputs(runtime);
    }
    if ((pending_requests & BALANCE_REQUEST_STOP_MASK) != 0U) stopped_outputs(runtime);
    publish_state(runtime);
}
void balance_runtime_get_snapshot(const balance_runtime_t *runtime, balance_runtime_snapshot_t *snapshot) {
    if (runtime != NULL && snapshot != NULL) *snapshot = runtime->snapshot;
}
