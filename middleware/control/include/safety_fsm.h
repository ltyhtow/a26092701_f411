/**
 * @file safety_fsm.h
 * @brief Safety & Interaction Finite State Machine (FSM) for two-wheel self-balancing robot.
 * @details Implements a 4-state safety machine with fall/tilt protection,
 *          upright recovery qualification, communication watchdog, and motion
 *          slew-rate limiting (ramp controller).
 *
 * Platform-independent, pure C99 with zero dynamic memory allocation.
 */

#ifndef SAFETY_FSM_H
#define SAFETY_FSM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default Configuration Constants */
#define SAFETY_FSM_DEFAULT_FALL_PITCH_DEG        35.0f   /**< Fall shutdown pitch limit (|pitch| > 35 deg) */
#define SAFETY_FSM_DEFAULT_FALL_ROLL_DEG         30.0f   /**< Fall shutdown roll limit (|roll| > 30 deg) */
#define SAFETY_FSM_DEFAULT_RECOVERY_PITCH_DEG    5.0f    /**< Upright recovery pitch threshold (|pitch| < 5 deg) */
#define SAFETY_FSM_DEFAULT_RECOVERY_ROLL_DEG     10.0f   /**< Upright recovery roll threshold (|roll| < 10 deg) */
#define SAFETY_FSM_DEFAULT_STEADY_GYRO_DPS       10.0f   /**< Steady detection max angular speed in deg/s */
#define SAFETY_FSM_DEFAULT_STEADY_TIME_MS        200U    /**< Continuous steady time required (ms) */
#define SAFETY_FSM_DEFAULT_CMD_TIMEOUT_MS        500U    /**< Communication watchdog timeout (ms) */
#define SAFETY_FSM_DEFAULT_MAX_LINEAR_ACCEL      2.0f    /**< Max linear acceleration limit (units/s^2) */
#define SAFETY_FSM_DEFAULT_MAX_LINEAR_DECEL      4.0f    /**< Max linear deceleration limit (units/s^2) */
#define SAFETY_FSM_DEFAULT_MAX_YAW_ACCEL         10.0f   /**< Max angular acceleration limit (units/s^2) */

/* Safety Fault Bitmask Definitions */
#define SAFETY_FAULT_NONE               0x00000000U
#define SAFETY_FAULT_FALL_PITCH         (1U << 0) /**< Pitch tilt exceeded safety threshold */
#define SAFETY_FAULT_FALL_ROLL          (1U << 1) /**< Roll tilt exceeded safety threshold */
#define SAFETY_FAULT_CMD_TIMEOUT        (1U << 2) /**< Communication command watchdog expired */
#define SAFETY_FAULT_SENSOR_INVALID     (1U << 3) /**< IMU / sensor telemetry invalid or NaN */
#define SAFETY_FAULT_CALIBRATION_FAIL   (1U << 4) /**< Sensor calibration failed */
#define SAFETY_FAULT_EMERGENCY_STOP     (1U << 5) /**< Software or hardware E-stop active */

/**
 * @brief Robot operational states.
 */
typedef enum {
    BALANCE_STATE_DISARMED = 0, /**< Motors disabled, PWM=0. Standby / safe state. */
    BALANCE_STATE_CALIBRATING,  /**< Waiting for sensor / zero point calibration. */
    BALANCE_STATE_ARMED,        /**< Normal active balance closed-loop operation. */
    BALANCE_STATE_FALLEN        /**< Fall/tilt protection triggered, motors immediately cut off. */
} balance_state_t;

/**
 * @brief Safety FSM configuration parameters.
 */
typedef struct {
    /* Fall protection limits */
    float pitch_fall_limit_deg;         /**< Pitch angle threshold for fall trigger (degrees) */
    float roll_fall_limit_deg;          /**< Roll angle threshold for fall trigger (degrees) */

    /* Recovery thresholds */
    float pitch_recovery_limit_deg;     /**< Max pitch angle to qualify as upright (degrees) */
    float roll_recovery_limit_deg;      /**< Max roll angle to qualify as upright (degrees) */
    float gyro_steady_limit_dps;        /**< Max gyro rate on all axes for steady detection (deg/s) */
    uint32_t steady_time_threshold_ms;  /**< Required continuous steady duration (ms) */

    /* Communication Watchdog */
    uint32_t cmd_timeout_ms;            /**< Motion command watchdog timeout (ms) */

    /* Motion Slew Rate Limiter (Ramp Controller) */
    float max_linear_accel;             /**< Linear acceleration limit (units/s^2, <=0 disables) */
    float max_linear_decel;             /**< Linear deceleration limit (units/s^2, <=0 disables) */
    float max_yaw_accel;                /**< Yaw angular acceleration limit (units/s^2, <=0 disables) */

    /* Behavioral policy flags */
    bool require_calibration;           /**< If true, arming is rejected until calibration succeeds */
    bool auto_rearm_enable;             /**< If true, automatically re-arms from FALLEN when upright & steady */
} safety_fsm_config_t;

/**
 * @brief Safety FSM runtime context.
 */
typedef struct {
    /* State */
    balance_state_t state;              /**< Current active balance state */
    balance_state_t previous_state;     /**< Previous balance state */

    /* Configuration */
    safety_fsm_config_t config;         /**< Active configuration */

    /* Fault Flags */
    uint32_t fault_flags;               /**< Active fault bitmask (SAFETY_FAULT_*) */

    /* Status flags */
    bool is_calibrated;                 /**< IMU/zero calibration completed successfully */
    bool is_upright;                    /**< Current attitude within recovery bounds */
    bool is_steady;                     /**< Gyro rates below steady threshold for required duration */
    bool watchdog_tripped;              /**< True when motion command watchdog has timed out */
    bool emergency_stop_active;         /**< True when E-stop condition is latched */
    bool attitude_valid;               /**< A finite, healthy attitude has been supplied */
    bool attitude_sample_pending;      /**< Only newly supplied samples qualify steady time */
    bool manual_disarm_latched;        /**< Explicit stop inhibits optional automatic fall recovery */

    /* Attitude inputs */
    float current_pitch_deg;            /**< Latest pitch angle (deg) */
    float current_roll_deg;             /**< Latest roll angle (deg) */
    float current_gyro_pitch_dps;       /**< Latest pitch gyro rate (deg/s) */
    float current_gyro_roll_dps;        /**< Latest roll gyro rate (deg/s) */
    float current_gyro_yaw_dps;         /**< Latest yaw gyro rate (deg/s) */

    /* Timing & Counters */
    uint32_t last_cmd_timestamp_ms;     /**< Timestamp of last valid motion command or heartbeat (ms) */
    uint32_t last_step_timestamp_ms;    /**< Timestamp of previous step (ms) */
    uint32_t steady_duration_ms;        /**< Elapsed time satisfying steady conditions (ms) */
    bool has_received_cmd;              /**< True if at least one motion command or heartbeat was received */

    /* Motion Command Inputs (Raw) */
    float target_linear_cmd;            /**< Raw commanded linear velocity */
    float target_yaw_cmd;               /**< Raw commanded yaw rate */

    /* Ramp Controller Output (Slew Rate Limited) */
    float ramped_linear_vel;            /**< Slew-rate filtered linear velocity output */
    float ramped_yaw_vel;               /**< Slew-rate filtered yaw rate output */

    /* Diagnostic counters */
    uint32_t state_transition_count;    /**< Total state transitions */
    uint32_t fall_event_count;          /**< Total fall protection triggers */
    uint32_t watchdog_trip_count;       /**< Total watchdog timeout events */
} safety_fsm_t;

/**
 * @brief Retrieve default FSM configuration.
 * @return Default safety_fsm_config_t structure.
 */
safety_fsm_config_t safety_fsm_default_config(void);

/**
 * @brief Initialize the Safety FSM context.
 * @param[out] fsm Pointer to safety_fsm_t structure to initialize.
 * @param[in]  config Pointer to configuration, or NULL to use defaults.
 */
void safety_fsm_init(safety_fsm_t *fsm, const safety_fsm_config_t *config);

/* State & Status Queries */

/**
 * @brief Get the current balance state.
 * @param[in] fsm Pointer to safety_fsm_t.
 * @return Current balance_state_t.
 */
balance_state_t safety_fsm_get_state(const safety_fsm_t *fsm);

/**
 * @brief Get string representation of a balance state.
 * @param[in] state Balance state enum.
 * @return Constant string (e.g., "ARMED", "FALLEN").
 */
const char *safety_fsm_state_to_str(balance_state_t state);

/**
 * @brief Check if motor drive outputs should be enabled.
 * @details Only true in BALANCE_STATE_ARMED. In all other states (DISARMED,
 *          CALIBRATING, FALLEN), motors must be cut off (PWM = 0).
 * @param[in] fsm Pointer to safety_fsm_t.
 * @return true if motors are permitted to run, false otherwise.
 */
bool safety_fsm_is_motor_enabled(const safety_fsm_t *fsm);

/**
 * @brief Check if robot is currently within upright recovery limits.
 * @param[in] fsm Pointer to safety_fsm_t.
 * @return true if |pitch| < pitch_recovery_limit and |roll| < roll_recovery_limit.
 */
bool safety_fsm_is_upright(const safety_fsm_t *fsm);

/**
 * @brief Check if robot is currently steady (gyro rates low for duration).
 * @param[in] fsm Pointer to safety_fsm_t.
 * @return true if steady conditions are met.
 */
bool safety_fsm_is_steady(const safety_fsm_t *fsm);

/**
 * @brief Check if calibration has completed.
 * @param[in] fsm Pointer to safety_fsm_t.
 * @return true if calibrated.
 */
bool safety_fsm_is_calibrated(const safety_fsm_t *fsm);

/**
 * @brief Check if conditions allow re-arming the robot.
 * @details Evaluates whether the robot is upright, has no blocking faults,
 *          and is calibrated if required.
 * @param[in] fsm Pointer to safety_fsm_t.
 * @return true if re-arming is permitted.
 */
bool safety_fsm_can_rearm(const safety_fsm_t *fsm);

/**
 * @brief Get the active fault flags bitmask.
 * @param[in] fsm Pointer to safety_fsm_t.
 * @return Bitmask of active SAFETY_FAULT_* flags.
 */
uint32_t safety_fsm_get_fault_flags(const safety_fsm_t *fsm);

/**
 * @brief Check if any specified fault flags are active.
 * @param[in] fsm Pointer to safety_fsm_t.
 * @param[in] fault_mask Bitmask of faults to test.
 * @return true if any tested fault is set.
 */
bool safety_fsm_has_fault(const safety_fsm_t *fsm, uint32_t fault_mask);

/* Input Feed APIs */

/**
 * @brief Feed heartbeat to keep the communication watchdog alive.
 * @param[in,out] fsm Pointer to safety_fsm_t.
 * @param[in]     timestamp_ms Current system timestamp (ms).
 */
void safety_fsm_feed_heartbeat(safety_fsm_t *fsm, uint32_t timestamp_ms);

/**
 * @brief Feed motion command targets and refresh the communication watchdog.
 * @param[in,out] fsm Pointer to safety_fsm_t.
 * @param[in]     target_linear Commanded forward linear velocity (m/s or normalized).
 * @param[in]     target_yaw Commanded yaw angular rate (rad/s, deg/s, or normalized).
 * @param[in]     timestamp_ms Current system timestamp (ms).
 */
void safety_fsm_feed_motion_cmd(safety_fsm_t *fsm, float target_linear, float target_yaw, uint32_t timestamp_ms);

/**
 * @brief Update attitude and angular rate telemetry from the IMU.
 * @param[in,out] fsm Pointer to safety_fsm_t.
 * @param[in]     pitch_deg Current pitch angle in degrees.
 * @param[in]     roll_deg Current roll angle in degrees.
 * @param[in]     gyro_pitch_dps Current pitch angular velocity in deg/s.
 * @param[in]     gyro_roll_dps Current roll angular velocity in deg/s.
 * @param[in]     gyro_yaw_dps Current yaw angular velocity in deg/s.
 */
void safety_fsm_update_attitude(safety_fsm_t *fsm, float pitch_deg, float roll_deg,
                                float gyro_pitch_dps, float gyro_roll_dps, float gyro_yaw_dps);

/**
 * @brief Query the current slew-rate limited (ramped) motion outputs.
 * @param[in]  fsm Pointer to safety_fsm_t.
 * @param[out] out_linear Pointer to store ramped linear velocity (can be NULL).
 * @param[out] out_yaw Pointer to store ramped yaw rate (can be NULL).
 */
void safety_fsm_get_motion_output(const safety_fsm_t *fsm, float *out_linear, float *out_yaw);

/* State Transition & Command APIs */

/**
 * @brief Request transition to BALANCE_STATE_ARMED.
 * @details Allowed from DISARMED if upright and steady, and no blocking faults.
 *          FALLEN requires an explicit successful reset first (default policy).
 * @param[in,out] fsm Pointer to safety_fsm_t.
 * @return true if transitioned to ARMED, false if rejected due to safety violations.
 */
bool safety_fsm_request_arm(safety_fsm_t *fsm);

/**
 * @brief Request transition to BALANCE_STATE_DISARMED.
 * @param[in,out] fsm Pointer to safety_fsm_t.
 */
void safety_fsm_request_disarm(safety_fsm_t *fsm);

/**
 * @brief Request transition to BALANCE_STATE_CALIBRATING.
 * @details Only allowed from BALANCE_STATE_DISARMED.
 * @param[in,out] fsm Pointer to safety_fsm_t.
 * @return true if calibration state entered, false otherwise.
 */
bool safety_fsm_request_calibration(safety_fsm_t *fsm);

/**
 * @brief Notify FSM that calibration process has finished.
 * @param[in,out] fsm Pointer to safety_fsm_t.
 * @param[in]     success true if calibration succeeded, false if failed.
 */
void safety_fsm_notify_calibration_done(safety_fsm_t *fsm, bool success);

/**
 * @brief Reset faults and allow recovery from FALLEN state.
 * @details In FALLEN state, transitions to DISARMED if upright conditions are met.
 *          Clears fall and timeout faults.
 * @param[in,out] fsm Pointer to safety_fsm_t.
 * @return true if fault was reset successfully, false if robot is still tilted.
 */
bool safety_fsm_reset_fault(safety_fsm_t *fsm);

/**
 * @brief Set external fault flag (e.g. sensor lost, actuator fault).
 * @param[in,out] fsm Pointer to safety_fsm_t.
 * @param[in]     fault_flag SAFETY_FAULT_* bitmask to set.
 */
void safety_fsm_set_fault(safety_fsm_t *fsm, uint32_t fault_flag);

/**
 * @brief Clear external fault flag.
 * @param[in,out] fsm Pointer to safety_fsm_t.
 * @param[in]     fault_flag SAFETY_FAULT_* bitmask to clear.
 */
void safety_fsm_clear_fault(safety_fsm_t *fsm, uint32_t fault_flag);

/**
 * @brief Trigger an emergency stop immediately.
 * @param[in,out] fsm Pointer to safety_fsm_t.
 */
void safety_fsm_emergency_stop(safety_fsm_t *fsm);

/* Periodic Execution Step */

/**
 * @brief Main periodic FSM evaluation step.
 * @details Evaluates fall protection, communication watchdog, steady detection,
 *          auto-recovery (if enabled), and motion slew-rate filtering.
 * @param[in,out] fsm Pointer to safety_fsm_t.
 * @param[in]     dt_s Elapsed time since last call in seconds (e.g., 0.005f for 200Hz).
 * @param[in]     current_time_ms Current system timestamp in milliseconds.
 */
void safety_fsm_step(safety_fsm_t *fsm, float dt_s, uint32_t current_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_FSM_H */
