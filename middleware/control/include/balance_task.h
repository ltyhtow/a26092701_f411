/**
 * @file balance_task.h
 * @brief FreeRTOS 200 Hz periodic balance control task for two-wheel self-balancing robot.
 * @details Integrates balance controller, safety FSM, IMU attitude queue, and protocol queues:
 *          - Runs at 200 Hz (5 ms period via vTaskDelayUntil).
 *          - Consumes attitude telemetry from imu_attitude_queue.
 *          - Dynamically consumes PID runtime updates from pid_config_queue.
 *          - Consumes motion targets and arm/disarm flags from motion_command_queue.
 *          - Feeds watchdog and drives targets through safety slew limiter.
 *          - Updates Safety FSM; outputs PWM only when ARMED, 0 otherwise.
 *          - Dispatches motor output commands through a decoupled hook/callback.
 *
 * Platform-independent middleware layer: decoupled from STM32 hardware registers.
 */

#ifndef BALANCE_TASK_H
#define BALANCE_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "balance_controller.h"
#include "safety_fsm.h"
#include "serial_protocol.h"
#include "imu_fusion.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Timing and Task Configuration */
#define BALANCE_TASK_PERIOD_MS          5U      /**< 200 Hz periodic interval (5 ms) */
#define BALANCE_TASK_PERIOD_S           0.005f  /**< Time step in seconds */
#define BALANCE_TASK_STACK_WORDS        512U    /**< Stack depth in 32-bit words (2048 bytes) */
#define BALANCE_TASK_DEFAULT_PRIORITY   3U      /**< Task execution priority */

/* Protocol Motion Command Flag Bitmasks */
#define BALANCE_MOTION_FLAG_ENABLE      0x0001U /**< Bit 0: Arm / Enable motors */
#define BALANCE_MOTION_FLAG_CLEAR_FAULT 0x0002U /**< Bit 1: Clear fault / Recover from FALLEN */

/* Telemetry Defaults */
#define BALANCE_TELEMETRY_DEFAULT_DIVISOR 4U    /**< 200 Hz / 4 = 50 Hz default telemetry */

/**
 * @brief Function pointer type for decoupled motor drive output hook.
 * @param left_pwm Clamped left motor PWM effort (-max_pwm to +max_pwm).
 * @param right_pwm Clamped right motor PWM effort (-max_pwm to +max_pwm).
 */
typedef void (*balance_motor_output_fn)(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief Function pointer type for wheel encoder speed feedback hook.
 * @param[out] left_speed Measured left wheel forward velocity.
 * @param[out] right_speed Measured right wheel forward velocity.
 */
typedef void (*balance_encoder_read_fn)(float *left_speed, float *right_speed);

/**
 * @brief Balance control task creation and runtime configuration.
 */
typedef struct {
    QueueHandle_t attitude_queue;               /**< Queue for imu_fusion_output_t (Required) */
    QueueHandle_t pid_config_queue;             /**< Queue for pid_config_command_t (Optional) */
    QueueHandle_t motion_command_queue;         /**< Queue for motion_command_t (Optional) */
    QueueHandle_t system_command_queue;         /**< Queue for system_command_t (Optional) */
    balance_motor_output_fn motor_output_hook;  /**< Motor output callback (Optional) */
    balance_encoder_read_fn encoder_read_hook;  /**< Wheel speed encoder callback (Optional) */
    uint32_t priority;                          /**< Task priority (0 uses default 3U) */
    uint16_t stack_depth_words;                 /**< Stack depth (0 uses default 512U) */
    bool enable_telemetry;                      /**< Send SERIAL_CMD_TELEMETRY over protocol */
    uint8_t telemetry_divisor;                  /**< Telemetry downsample ratio (e.g. 4 for 50 Hz) */
} balance_task_config_t;

/**
 * @brief Default hook function for motor output.
 * @note Implemented as weak function in balance_task.c; board drivers can override it directly.
 * @param left_pwm Clamped left motor PWM.
 * @param right_pwm Clamped right motor PWM.
 */
void balance_motor_set_output(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief Populate default task configuration struct.
 * @param[out] config Pointer to config structure to initialize.
 */
void balance_task_default_config(balance_task_config_t *config);

/**
 * @brief Create and start the FreeRTOS Balance Control Task.
 * @param[in]  config Pointer to task configuration.
 * @param[out] task_handle Optional pointer to receive created task handle.
 * @return pdPASS on success, pdFAIL on invalid arguments or task creation failure.
 */
BaseType_t balance_task_start(const balance_task_config_t *config,
                              TaskHandle_t *task_handle);

/**
 * @brief Convenience start API using standard queue handles.
 * @param attitude_queue Handle to imu_attitude_queue.
 * @param pid_config_queue Handle to pid_config_queue.
 * @param motion_command_queue Handle to motion_command_queue.
 * @param motor_hook Motor drive callback.
 * @param task_handle Pointer to receive created task handle.
 * @return pdPASS on success, pdFAIL on failure.
 */
BaseType_t balance_task_init_and_start(QueueHandle_t attitude_queue,
                                      QueueHandle_t pid_config_queue,
                                      QueueHandle_t motion_command_queue,
                                      balance_motor_output_fn motor_hook,
                                      TaskHandle_t *task_handle);

/**
 * @brief Set or update motor output hook callback dynamically at runtime.
 * @param hook Function pointer to callback.
 */
void balance_task_set_motor_output_hook(balance_motor_output_fn hook);

/**
 * @brief Set or update encoder read hook callback dynamically at runtime.
 * @param hook Function pointer to callback.
 */
void balance_task_set_encoder_read_hook(balance_encoder_read_fn hook);

/**
 * @brief Request robot arming (from DISARMED or FALLEN).
 * @return true if transitioned to ARMED, false if rejected due to tilt or motion.
 */
bool balance_task_request_arm(void);

/**
 * @brief Request robot disarming (safe cutoff, PWM=0).
 */
void balance_task_request_disarm(void);

/**
 * @brief Clear safety faults and attempt recovery from FALLEN state.
 * @return true if fault was reset and robot is upright, false otherwise.
 */
bool balance_task_reset_fault(void);

/**
 * @brief Trigger immediate emergency stop (latches E-stop fault, cuts motors).
 */
void balance_task_emergency_stop(void);

/**
 * @brief Query current robot operational balance state.
 * @return Current balance_state_t (DISARMED, CALIBRATING, ARMED, FALLEN).
 */
balance_state_t balance_task_get_state(void);

/**
 * @brief Get read-only pointer to internal balance controller context.
 * @return Pointer to balance_controller_t.
 */
const balance_controller_t *balance_task_get_controller(void);

/**
 * @brief Get read-only pointer to internal safety FSM context.
 * @return Pointer to safety_fsm_t.
 */
const safety_fsm_t *balance_task_get_safety_fsm(void);

/**
 * @brief Query latest computed control outputs.
 * @param[out] outputs Destination structure to receive output snapshot.
 */
void balance_task_get_latest_outputs(balance_controller_outputs_t *outputs);

/**
 * @brief Enable or disable periodic serial telemetry publication.
 * @param enable true to enable, false to disable.
 * @param divisor Decimation ratio relative to 200 Hz (e.g. 4 for 50 Hz).
 */
void balance_task_set_telemetry_enabled(bool enable, uint8_t divisor);

/**
 * @brief Calibration task entry function to inspect IMU pitch polarity, mechanical zero, and encoders.
 * @param argument QueueHandle_t to imu_attitude_queue.
 */
void balance_calibration_task_entry(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_TASK_H */
