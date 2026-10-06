/**
 * @file balance_controller.h
 * @brief Platform-independent, decoupled balance controller for two-wheel self-balancing robots.
 *
 * @note Pure C99 implementation without hardware dependencies, registers, or RTOS calls.
 *       All functions are re-entrant and thread-safe when operating on separate contexts.
 *
 * Control Topology:
 *   1. Balance Loop (PD):
 *      - Controls chassis pitch attitude around equilibrium (mechanical zero theta_0).
 *      - Uses pitch angle error (theta - theta_target) for proportional feedback.
 *      - Uses Gyro_Y angular velocity (dps) directly as derivative feedback to prevent
 *        noise amplification from numerical differentiation.
 *   2. Velocity Loop (PI):
 *      - Regulates linear wheel speed against target speed.
 *      - Employs a 1st-order Low-Pass Filter (LPF) on measured speed (alpha ~ 0.7 - 0.8)
 *        to attenuate discrete encoder pulse quantization noise.
 *      - Includes anti-windup clamping on the integral accumulator.
 *      - Supports both parallel effort summation and cascaded dynamic tilt angle modes.
 *   3. Turn Loop (PD):
 *      - Controls yaw rate using target yaw rate and Gyro_Z angular velocity.
 *      - Provides directional steering and active yaw stabilization against drift.
 *   4. Output Mixer:
 *      - Computes differential PWM efforts for Left and Right motors.
 *      - Applies deadband compensation to overcome static friction (stiction).
 *      - Implements tip-over shutdown and saturation limiting [-max_pwm, +max_pwm].
 */

#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Loop identifiers for multi-loop parameter configuration.
 */
typedef enum {
    BALANCE_LOOP_ID_BALANCE  = 1U,  /**< Balance PD loop */
    BALANCE_LOOP_ID_VELOCITY = 2U,  /**< Velocity PI loop */
    BALANCE_LOOP_ID_TURN     = 3U   /**< Turn PD loop */
} balance_loop_id_t;

/**
 * @brief Velocity control coupling topology.
 */
typedef enum {
    /**
     * @brief Parallel summation mode:
     *        Linear Effort = Balance_Effort - Velocity_Effort
     *        where Velocity_Effort = Kp*(Target - Measured).
     *        When moving forward (Measured > Target), the negative velocity effort produces
     *        forward motor torque, driving wheels under the chassis to tilt it back and brake.
     */
    BALANCE_VEL_COUPLE_PARALLEL = 0U,

    /**
     * @brief Cascaded tilt angle mode:
     *        Velocity PI outputs a dynamic pitch offset:
     *        theta_target = theta_0 + dynamic_pitch_offset
     *        Balance loop then drives robot to catch the induced tilt.
     */
    BALANCE_VEL_COUPLE_TILT_CASCADE = 1U
} balance_velocity_coupling_mode_t;

/**
 * @brief Parameters for the Balance (PD) loop.
 */
typedef struct {
    float kp;                   /**< Proportional gain (effort / degree) */
    float kd;                   /**< Derivative gain on Gyro_Y (effort / (deg/s)) */
} balance_pid_params_t;

/**
 * @brief Parameters for the Velocity (PI) loop.
 */
typedef struct {
    float kp;                   /**< Proportional gain (effort / (speed_unit)) */
    float ki;                   /**< Integral gain (effort / (speed_unit * s)) */
    float integral_limit;       /**< Maximum anti-windup integral accumulator magnitude */
    float lpf_alpha;            /**< First-order LPF factor: y[k] = alpha*y[k-1] + (1-alpha)*x[k] */
    float max_output;           /**< Output saturation limit (PWM effort or max tilt degrees) */
} velocity_pid_params_t;

/**
 * @brief Parameters for the Turn (PD) loop.
 */
typedef struct {
    float kp;                   /**< Proportional gain on yaw rate error */
    float kd;                   /**< Derivative gain on yaw rate error derivative */
    float max_output;           /**< Maximum differential effort for turning */
} turn_pid_params_t;

/**
 * @brief Complete configuration parameters for the balance controller.
 */
typedef struct {
    balance_pid_params_t balance_loop;          /**< Balance PD parameters */
    velocity_pid_params_t velocity_loop;        /**< Velocity PI parameters */
    turn_pid_params_t turn_loop;                /**< Turn PD parameters */

    float mechanical_zero_pitch;                /**< Mechanical equilibrium pitch offset theta_0 (deg) */
    float max_pitch_angle;                      /**< Tip-over cutoff angle (deg); shutdown if |pitch| > max */
    float deadband_left;                        /**< Left motor deadband PWM boost (stiction compensation) */
    float deadband_right;                       /**< Right motor deadband PWM boost (stiction compensation) */
    float max_pwm;                              /**< Maximum allowed PWM magnitude (e.g. 4999.0f) */
    bool auto_recovery_enabled;                 /**< If true, auto-recover from tip-over when upright again; if false, latch until reset */

    balance_velocity_coupling_mode_t velocity_coupling_mode; /**< Parallel summation or cascade tilt */
} balance_controller_config_t;

/**
 * @brief Runtime state variables for the velocity loop.
 */
typedef struct {
    float filtered_speed_left;      /**< Low-pass filtered left wheel speed */
    float filtered_speed_right;     /**< Low-pass filtered right wheel speed */
    float filtered_speed_avg;       /**< Filtered average chassis forward speed */
    float integral;                 /**< Velocity integral accumulator */
    float last_speed_error;         /**< Previous speed error */
    bool is_filter_initialized;     /**< True once first samples initialize filter values */
} velocity_state_t;

/**
 * @brief Runtime state variables for the turn loop.
 */
typedef struct {
    float last_yaw_rate_error;      /**< Previous yaw rate error for derivative estimation */
    bool has_previous_yaw_error;    /**< True once first error sample has been recorded */
} turn_state_t;

/**
 * @brief Overall runtime state of the balance controller.
 */
typedef struct {
    velocity_state_t velocity;      /**< Velocity loop runtime state */
    turn_state_t turn;              /**< Turn loop runtime state */

    bool is_enabled;                /**< Controller output enabled */
    bool is_tipped_over;            /**< True if robot exceeded max pitch angle */
    uint32_t tip_over_count;        /**< Historical counter of tip-over events */
} balance_controller_state_t;

/**
 * @brief Controller input sensor and target measurements.
 */
typedef struct {
    float pitch_deg;                /**< Current fused pitch angle from IMU/AHRS (degrees) */
    float gyro_pitch_dps;           /**< Gyro angular velocity around pitch axis (deg/s) */
    float gyro_yaw_dps;             /**< Gyro angular velocity around yaw/turn axis (deg/s) */

    float measured_speed_left;      /**< Measured left wheel speed (pulses/period, mm/s, or RPM) */
    float measured_speed_right;     /**< Measured right wheel speed (pulses/period, mm/s, or RPM) */

    float target_speed;             /**< Desired linear forward speed (same unit as measured_speed) */
    float target_yaw_rate_dps;      /**< Desired turning yaw rate (deg/s, positive = turn left/CCW) */

    float dt_s;                     /**< Loop update interval in seconds (e.g. 0.005f for 200 Hz) */
} balance_controller_inputs_t;

/**
 * @brief Controller computed intermediate efforts and final motor outputs.
 */
typedef struct {
    float balance_effort;           /**< Balance loop effort */
    float velocity_effort;          /**< Velocity loop effort (or dynamic tilt in cascade mode) */
    float turn_effort;              /**< Turn loop differential effort */

    float left_effort_raw;          /**< Left motor effort before deadband and saturation */
    float right_effort_raw;         /**< Right motor effort before deadband and saturation */

    int16_t left_pwm;               /**< Final clamped integer PWM command for left motor */
    int16_t right_pwm;              /**< Final clamped integer PWM command for right motor */

    bool safety_active;             /**< True if outputs forced to zero (tipped over or disabled) */
} balance_controller_outputs_t;

/**
 * @brief Balance controller context structure encapsulating configuration and state.
 */
typedef struct {
    balance_controller_config_t config;
    balance_controller_state_t state;
} balance_controller_t;

/* ========================================================================= */
/*                          Public API Functions                             */
/* ========================================================================= */

/**
 * @brief Populate configuration struct with standard, safe default parameters.
 * @param[out] config Pointer to configuration struct to populate.
 */
void balance_controller_default_config(balance_controller_config_t *config);

/**
 * @brief Initialize the balance controller instance with given configuration.
 * @param[in,out] ctrl Pointer to controller instance.
 * @param[in]     config Pointer to configuration parameters (NULL for default config).
 */
void balance_controller_init(balance_controller_t *ctrl, const balance_controller_config_t *config);

/**
 * @brief Reset all controller runtime state (filters, integrators, flags) to zero.
 * @param[in,out] ctrl Pointer to controller instance.
 */
void balance_controller_reset(balance_controller_t *ctrl);

/**
 * @brief Reset only integrator accumulators (useful on stance transitions or re-enabling).
 * @param[in,out] ctrl Pointer to controller instance.
 */
void balance_controller_reset_integrators(balance_controller_t *ctrl);

/**
 * @brief Enable or disable motor output from controller.
 * @param[in,out] ctrl Pointer to controller instance.
 * @param[in]     enable True to enable, false to disable.
 */
void balance_controller_set_enabled(balance_controller_t *ctrl, bool enable);

/**
 * @brief Check if controller is enabled.
 * @param[in] ctrl Pointer to controller instance.
 * @return True if enabled.
 */
bool balance_controller_is_enabled(const balance_controller_t *ctrl);

/**
 * @brief Check if controller has tripped the tip-over safety shutdown.
 * @param[in] ctrl Pointer to controller instance.
 * @return True if currently in tipped-over state.
 */
bool balance_controller_is_tipped_over(const balance_controller_t *ctrl);

/**
 * @brief Set mechanical equilibrium zero pitch angle.
 * @param[in,out] ctrl Pointer to controller instance.
 * @param[in]     zero_pitch_deg Mechanical zero angle (deg).
 */
void balance_controller_set_mechanical_zero(balance_controller_t *ctrl, float zero_pitch_deg);

/**
 * @brief Get current mechanical equilibrium zero pitch angle.
 * @param[in] ctrl Pointer to controller instance.
 * @return Mechanical zero pitch angle (deg).
 */
float balance_controller_get_mechanical_zero(const balance_controller_t *ctrl);

/**
 * @brief Set Balance loop PD gains.
 * @param[in,out] ctrl Pointer to controller instance.
 * @param[in]     kp Proportional gain.
 * @param[in]     kd Derivative gain on gyro rate.
 */
void balance_controller_set_balance_gains(balance_controller_t *ctrl, float kp, float kd);

/**
 * @brief Set Velocity loop PI gains.
 * @param[in,out] ctrl Pointer to controller instance.
 * @param[in]     kp Proportional gain.
 * @param[in]     ki Integral gain.
 * @param[in]     integral_limit Anti-windup limit for integral accumulator.
 */
void balance_controller_set_velocity_gains(balance_controller_t *ctrl, float kp, float ki, float integral_limit);

/**
 * @brief Set Turn loop PD gains.
 * @param[in,out] ctrl Pointer to controller instance.
 * @param[in]     kp Proportional gain.
 * @param[in]     kd Derivative gain.
 */
void balance_controller_set_turn_gains(balance_controller_t *ctrl, float kp, float kd);

/**
 * @brief Set controller gains by loop identifier (convenient for remote serial tuning).
 * @param[in,out] ctrl Pointer to controller instance.
 * @param[in]     loop_id Loop identifier (BALANCE_LOOP_ID_*).
 * @param[in]     kp Proportional gain.
 * @param[in]     ki Integral gain (applicable to velocity loop).
 * @param[in]     kd Derivative gain (applicable to balance/turn loops).
 * @param[in]     limit Output/integral limit.
 * @return True if loop_id was recognized and updated, false otherwise.
 */
bool balance_controller_set_loop_gains(balance_controller_t *ctrl,
                                       uint8_t loop_id,
                                       float kp,
                                       float ki,
                                       float kd,
                                       float limit);

/**
 * @brief Main control step function. Executes balance, velocity, and turn loops,
 *        mixes outputs, applies deadband compensation and saturation limits.
 *
 * @param[in,out] ctrl Pointer to controller instance.
 * @param[in]     inputs Sensor measurements and target references.
 * @param[out]    outputs Computed loop efforts and final PWM commands.
 */
void balance_controller_update(balance_controller_t *ctrl,
                              const balance_controller_inputs_t *inputs,
                              balance_controller_outputs_t *outputs);

/* ========================================================================= */
/*                   Decoupled Unit Calculator Functions                     */
/* ========================================================================= */

/**
 * @brief Calculate Balance PD loop effort.
 *        Effort = Kp * pitch_error + Kd * gyro_pitch_dps
 * @param[in] params Balance PD parameters.
 * @param[in] pitch_error_deg Angle error in degrees (measured - target).
 * @param[in] gyro_pitch_dps Angular velocity in degrees/s.
 * @return Calculated balance effort.
 */
float balance_controller_calc_balance_pd(const balance_pid_params_t *params,
                                         float pitch_error_deg,
                                         float gyro_pitch_dps);

/**
 * @brief Calculate Velocity PI loop effort with 1st-order LPF and anti-windup.
 * @param[in]     params Velocity PI parameters.
 * @param[in,out] state Velocity loop runtime state.
 * @param[in]     speed_error Speed error (target - filtered_speed).
 * @param[in]     dt_s Loop time step in seconds.
 * @return Calculated velocity effort (or dynamic tilt angle).
 */
float balance_controller_calc_velocity_pi(const velocity_pid_params_t *params,
                                          velocity_state_t *state,
                                          float speed_error,
                                          float dt_s);

/**
 * @brief Calculate Turn PD loop effort.
 * @param[in]     params Turn PD parameters.
 * @param[in,out] state Turn loop runtime state.
 * @param[in]     target_yaw_rate_dps Desired yaw rate in deg/s.
 * @param[in]     measured_yaw_rate_dps Measured Gyro_Z in deg/s.
 * @param[in]     dt_s Loop time step in seconds.
 * @return Calculated differential turn effort.
 */
float balance_controller_calc_turn_pd(const turn_pid_params_t *params,
                                      turn_state_t *state,
                                      float target_yaw_rate_dps,
                                      float measured_yaw_rate_dps,
                                      float dt_s);

/**
 * @brief First-order Low-Pass Filter step:
 *        y[k] = alpha * y[k-1] + (1 - alpha) * x[k]
 * @param[in] current_filtered Previous filtered value y[k-1].
 * @param[in] raw_sample New sample x[k].
 * @param[in] alpha Filter weight between 0.0f and 1.0f (typically 0.7 - 0.8).
 * @return New filtered value y[k].
 */
float balance_controller_filter_speed(float current_filtered, float raw_sample, float alpha);

/**
 * @brief Apply deadband feedforward compensation and clamp output to [-max_pwm, +max_pwm].
 *        If effort > 0: output = min(max_pwm, effort + deadband)
 *        If effort < 0: output = max(-max_pwm, effort - deadband)
 *        If effort == 0: output = 0
 * @param[in] effort Uncompensated effort.
 * @param[in] deadband Static friction deadband compensation value.
 * @param[in] max_pwm Maximum allowed PWM magnitude.
 * @return Compensated and clamped effort.
 */
float balance_controller_apply_deadband_and_limit(float effort, float deadband, float max_pwm);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_CONTROLLER_H */
