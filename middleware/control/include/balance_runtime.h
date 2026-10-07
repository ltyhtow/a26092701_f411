#ifndef BALANCE_RUNTIME_H
#define BALANCE_RUNTIME_H

#include "balance_controller.h"
#include "safety_fsm.h"
#include "robot_types.h"

#define BALANCE_RUNTIME_PERIOD_MS 5U
#define BALANCE_RUNTIME_MAX_AGE_MS 20U
#define BALANCE_RUNTIME_CALIBRATION_TIMEOUT_MS 15000U
#define BALANCE_REQUEST_ARM       (1U << 0)
#define BALANCE_REQUEST_DISARM    (1U << 1)
#define BALANCE_REQUEST_RESET     (1U << 2)
#define BALANCE_REQUEST_ESTOP     (1U << 3)
#define BALANCE_REQUEST_CALIBRATE (1U << 4)
#define BALANCE_REQUEST_STOP_MASK (BALANCE_REQUEST_DISARM | BALANCE_REQUEST_RESET | BALANCE_REQUEST_ESTOP | BALANCE_REQUEST_CALIBRATE)

typedef struct {
    attitude_sample_t attitude;
    uint32_t timestamp_ms;
    uint32_t fault_flags;
    balance_state_t state;
    float left_counts_per_5ms, right_counts_per_5ms;
    balance_controller_outputs_t outputs;
    bool calibration_request;
} balance_runtime_snapshot_t;

/* One explicit owner per context. All time arguments are monotonic milliseconds
 * modulo uint32_t, and successive observations must be less than half the range
 * apart. The platform serializes stop requests and final output writes.
 *
 * Lifecycle for each bounded cycle:
 *   begin_cycle -> accept queued samples/commands -> finish_cycle(wheel counts)
 *   -> handle calibration_request and report result -> prepare_output -> write PWM.
 * Call prepare_output with freshly read time while holding the same exclusion
 * that protects the platform's immediate stop. It is never sufficient to write
 * the output computed by finish_cycle without that last age/stop check.
 * Public context storage enables static allocation and multiple independent
 * robots; treat its fields as read-only outside this module. */
typedef struct {
    balance_controller_t controller;
    safety_fsm_t safety;
    balance_runtime_snapshot_t snapshot;
    attitude_sample_t attitude;
    motion_request_t motion;
    uint32_t last_cycle_ms, cycle_ms, elapsed_ms, requests;
    uint32_t calibration_requested_ms;
    float dt_s;
    bool have_cycle, have_attitude, attitude_valid, motion_enable_seen;
    bool calibration_pending, calibration_started;
    bool new_attitude, bad_sample, deadline_missed, have_motion;
} balance_runtime_t;

void balance_runtime_init(balance_runtime_t *runtime, const balance_controller_config_t *config);
void balance_runtime_begin_cycle(balance_runtime_t *runtime, uint32_t now_ms);
void balance_runtime_accept_attitude(balance_runtime_t *runtime, const attitude_sample_t *sample);
/* Configuration changes are permitted only while DISARMED. Full validation is
 * atomic: rejection leaves the controller and its integrators untouched. */
bool balance_runtime_apply_config(balance_runtime_t *runtime, const balance_controller_config_t *config);
bool balance_runtime_accept_pid(balance_runtime_t *runtime, const pid_request_t *request);
void balance_runtime_accept_motion(balance_runtime_t *runtime, const motion_request_t *request);
void balance_runtime_request_system(balance_runtime_t *runtime, const system_request_t *request);
void balance_runtime_request_flags(balance_runtime_t *runtime, uint32_t requests);
/* Wheel values are counts accumulated since the preceding cycle. */
void balance_runtime_finish_cycle(balance_runtime_t *runtime, float left_counts, float right_counts);
/* Call after dispatching the snapshot.calibration_request effect, before output. */
void balance_runtime_calibration_result(balance_runtime_t *runtime, bool accepted);
/* Required immediately before writing PWM. A pending stop suppresses this output
 * and must also be posted to the following begin/finish cycle by the platform. */
void balance_runtime_prepare_output(balance_runtime_t *runtime, uint32_t dispatch_ms, uint32_t pending_requests);
void balance_runtime_get_snapshot(const balance_runtime_t *runtime, balance_runtime_snapshot_t *snapshot);

#endif
