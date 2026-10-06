/**
 * @file balance_task.c
 * @brief FreeRTOS 200 Hz periodic balance control task implementation.
 * @details Integrates:
 *          1. IMU attitude reception from imu_attitude_queue.
 *          2. Dynamic PID parameter updates from pid_config_queue.
 *          3. Motion commands, watchdog feeding, and slew rate limiting from motion_command_queue.
 *          4. Safety state machine evaluation (ARMED -> output PWM, otherwise 0).
 *          5. Motor output dispatch via registered hook or weak fallback.
 *          6. Periodic motion telemetry reporting via serial protocol.
 */

#include "balance_task.h"

#include <limits.h>
#include <math.h>
#include <string.h>

/* Task context and runtime structures */
static balance_task_config_t s_config;
static balance_controller_t   s_controller;
static safety_fsm_t          s_safety_fsm;
static TaskHandle_t          s_task_handle;
static bool                  s_initialized = false;

/* Attitude processing state */
static imu_fusion_output_t   s_latest_attitude;
static bool                  s_attitude_valid = false;

/* Outputs and telemetry state */
static balance_controller_outputs_t s_latest_outputs;
static uint8_t                      s_telemetry_seq = 0U;
static uint32_t                     s_cycle_counter = 0U;

/**
 * @brief Default weak implementation of motor output hook.
 *        Can be overridden in board application layer.
 */
__attribute__((weak)) void balance_motor_set_output(int16_t left_pwm, int16_t right_pwm) {
    (void)left_pwm;
    (void)right_pwm;
}

/**
 * @brief Convert floating-point value to signed Q16.16 fixed-point format with saturation.
 */
static inline int32_t float_to_q16_16(float val) {
    if (isnan(val)) {
        return 0;
    }
    const float scaled = val * 65536.0f;
    if (scaled >= 2147483647.0f) {
        return INT32_MAX;
    }
    if (scaled <= -2147483648.0f) {
        return INT32_MIN;
    }
    return (int32_t)lroundf(scaled);
}

/**
 * @brief Balance control periodic task entry function.
 */
static void balance_task_entry(void *argument) {
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(BALANCE_TASK_PERIOD_MS);

    for (;;) {
        /* Maintain precise 200 Hz periodic schedule (5 ms) */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint32_t current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        imu_fusion_output_t attitude;
        memset(&attitude, 0, sizeof(attitude));

        /* ----------------------------------------------------------------- */
        /* 1. Receive attitude message from imu_attitude_queue               */
        /* ----------------------------------------------------------------- */
        if (s_config.attitude_queue != NULL) {
            if (xQueueReceive(s_config.attitude_queue, &attitude, 0U) == pdTRUE) {
                s_latest_attitude = attitude;
                s_attitude_valid = true;
            } else if (s_attitude_valid) {
                /* Use most recent attitude sample if queue was momentarily empty */
                attitude = s_latest_attitude;
            }
        }

        if (s_attitude_valid) {
            /* Directly use calibrated gyro angular rates from IMU fusion filter
             * without numerical differentiation to prevent phase lag and noise amplification.
             * Axis mapping: [0]=Roll (X), [1]=Pitch (Y), [2]=Yaw (Z). */
            float gyro_pitch = attitude.gyro_dps[1];
            float gyro_roll  = attitude.gyro_dps[0];
            float gyro_yaw   = attitude.gyro_dps[2];

            /* Check sensor health / calibration flags */
            if ((attitude.status_flags & IMU_FUSION_STATUS_INPUT_INVALID) != 0U) {
                safety_fsm_set_fault(&s_safety_fsm, SAFETY_FAULT_SENSOR_INVALID);
            } else {
                safety_fsm_clear_fault(&s_safety_fsm, SAFETY_FAULT_SENSOR_INVALID);
                safety_fsm_update_attitude(&s_safety_fsm,
                                           attitude.pitch_deg,
                                           attitude.roll_deg,
                                           gyro_pitch,
                                           gyro_roll,
                                           gyro_yaw);
            }

            if ((attitude.status_flags & IMU_FUSION_STATUS_CALIBRATING) != 0U) {
                if (safety_fsm_get_state(&s_safety_fsm) == BALANCE_STATE_DISARMED) {
                    (void)safety_fsm_request_calibration(&s_safety_fsm);
                }
            } else if ((attitude.status_flags & IMU_FUSION_STATUS_CALIBRATED) != 0U) {
                if (safety_fsm_get_state(&s_safety_fsm) == BALANCE_STATE_CALIBRATING) {
                    safety_fsm_notify_calibration_done(&s_safety_fsm, true);
                }
            }
        }

        /* ----------------------------------------------------------------- */
        /* 2. Poll pid_config_queue: update controller parameters at runtime */
        /* ----------------------------------------------------------------- */
        if (s_config.pid_config_queue != NULL) {
            pid_config_command_t pid_cmd;
            while (xQueueReceive(s_config.pid_config_queue, &pid_cmd, 0U) == pdTRUE) {
                float kp = (float)pid_cmd.kp_q16_16 / 65536.0f;
                float ki = (float)pid_cmd.ki_q16_16 / 65536.0f;
                float kd = (float)pid_cmd.kd_q16_16 / 65536.0f;
                float limit = (float)pid_cmd.integral_limit_q16_16 / 65536.0f;

                switch (pid_cmd.loop_id) {
                case 0U: /* Loop 0: Balance PD */
                    balance_controller_set_balance_gains(&s_controller, kp, kd);
                    break;
                case 1U: /* Loop 1: Velocity PI */
                    balance_controller_set_velocity_gains(&s_controller, kp, ki, limit);
                    break;
                case 2U: /* Loop 2: Turn PD */
                    balance_controller_set_turn_gains(&s_controller, kp, kd);
                    break;
                default:
                    /* Fallback supporting BALANCE_LOOP_ID_* enums (1: balance, 2: velocity, 3: turn) */
                    (void)balance_controller_set_loop_gains(&s_controller, pid_cmd.loop_id, kp, ki, kd, limit);
                    break;
                }
            }
        }

        /* ----------------------------------------------------------------- */
        /* 2.5 Poll system_command_queue: handle Arm, Disarm, Reset, Calibrate */
        /* ----------------------------------------------------------------- */
        if (s_config.system_command_queue != NULL) {
            system_command_t sys_cmd;
            while (xQueueReceive(s_config.system_command_queue, &sys_cmd, 0U) == pdTRUE) {
                switch (sys_cmd.action) {
                case 1U: /* Arm */
                    (void)safety_fsm_request_arm(&s_safety_fsm);
                    break;
                case 2U: /* Disarm */
                    safety_fsm_request_disarm(&s_safety_fsm);
                    break;
                case 3U: /* Reset Fault / Recover from Fallen */
                    (void)safety_fsm_reset_fault(&s_safety_fsm);
                    break;
                case 4U: /* Calibrate */
                    (void)safety_fsm_request_calibration(&s_safety_fsm);
                    break;
                default:
                    break;
                }
            }
        }

        /* ----------------------------------------------------------------- */
        /* 3. Poll motion_command_queue: feed watchdog, handle flags, ramp  */
        /* ----------------------------------------------------------------- */
        if (s_config.motion_command_queue != NULL) {
            motion_command_t motion_cmd;
            while (xQueueReceive(s_config.motion_command_queue, &motion_cmd, 0U) == pdTRUE) {
                float target_linear = (float)motion_cmd.linear_q16_16 / 65536.0f;
                float target_yaw    = (float)motion_cmd.yaw_q16_16 / 65536.0f;

                /* Dynamic watchdog timeout configuration */
                if (motion_cmd.timeout_ms > 0U) {
                    s_safety_fsm.config.cmd_timeout_ms = motion_cmd.timeout_ms;
                }

                /* Clear fault flag */
                if ((motion_cmd.flags & BALANCE_MOTION_FLAG_CLEAR_FAULT) != 0U) {
                    (void)safety_fsm_reset_fault(&s_safety_fsm);
                }

                /* Enable / Disarm flag handling */
                if ((motion_cmd.flags & BALANCE_MOTION_FLAG_ENABLE) != 0U) {
                    if (safety_fsm_get_state(&s_safety_fsm) != BALANCE_STATE_ARMED) {
                        (void)safety_fsm_request_arm(&s_safety_fsm);
                    }
                } else {
                    safety_fsm_request_disarm(&s_safety_fsm);
                }

                /* Feed safety watchdog and set commanded velocities */
                safety_fsm_feed_motion_cmd(&s_safety_fsm, target_linear, target_yaw, current_time_ms);
            }
        }

        /* ----------------------------------------------------------------- */
        /* 4. Update Safety FSM and calculate controller outputs            */
        /* ----------------------------------------------------------------- */
        safety_fsm_step(&s_safety_fsm, BALANCE_TASK_PERIOD_S, current_time_ms);

        balance_state_t fsm_state = safety_fsm_get_state(&s_safety_fsm);
        int16_t left_pwm = 0;
        int16_t right_pwm = 0;
        float speed_left = 0.0f;
        float speed_right = 0.0f;

        /* Measure wheel speed if encoder callback is provided */
        if (s_config.encoder_read_hook != NULL) {
            s_config.encoder_read_hook(&speed_left, &speed_right);
        }

        if (fsm_state == BALANCE_STATE_ARMED && s_attitude_valid) {
            balance_controller_set_enabled(&s_controller, true);

            /* Retrieve targets filtered by motion slew rate limiter */
            float slew_linear = 0.0f;
            float slew_yaw    = 0.0f;
            safety_fsm_get_motion_output(&s_safety_fsm, &slew_linear, &slew_yaw);

            balance_controller_inputs_t inputs;
            inputs.pitch_deg            = attitude.pitch_deg;
            inputs.gyro_pitch_dps       = attitude.gyro_dps[1];
            inputs.gyro_yaw_dps         = attitude.gyro_dps[2];
            inputs.measured_speed_left  = speed_left;
            inputs.measured_speed_right = speed_right;
            inputs.target_speed         = slew_linear;
            inputs.target_yaw_rate_dps  = slew_yaw;
            inputs.dt_s                 = BALANCE_TASK_PERIOD_S;

            balance_controller_outputs_t outputs;
            balance_controller_update(&s_controller, &inputs, &outputs);
            s_latest_outputs = outputs;

            if (outputs.safety_active || balance_controller_is_tipped_over(&s_controller)) {
                /* Tip-over shutdown */
                left_pwm  = 0;
                right_pwm = 0;
                safety_fsm_set_fault(&s_safety_fsm, SAFETY_FAULT_FALL_PITCH);
                safety_fsm_emergency_stop(&s_safety_fsm);
            } else {
                left_pwm  = outputs.left_pwm;
                right_pwm = outputs.right_pwm;
            }
        } else {
            /* DISARMED, CALIBRATING, or FALLEN: Output must be 0 */
            balance_controller_set_enabled(&s_controller, false);
            balance_controller_reset_integrators(&s_controller);
            left_pwm  = 0;
            right_pwm = 0;
            memset(&s_latest_outputs, 0, sizeof(s_latest_outputs));
            s_latest_outputs.safety_active = true;
        }

        /* ----------------------------------------------------------------- */
        /* 5. Dispatch Motor Output Hook or Callback                        */
        /* ----------------------------------------------------------------- */
        if (s_config.motor_output_hook != NULL) {
            s_config.motor_output_hook(left_pwm, right_pwm);
        } else {
            balance_motor_set_output(left_pwm, right_pwm);
        }

        /* ----------------------------------------------------------------- */
        /* 6. Periodic Motion Telemetry Reporting (Optional)                */
        /* ----------------------------------------------------------------- */
        if (s_config.enable_telemetry && s_config.telemetry_divisor > 0U) {
            if ((s_cycle_counter % s_config.telemetry_divisor) == 0U) {
                motion_telemetry_t telem;
                memset(&telem, 0, sizeof(telem));
                telem.version          = SERIAL_PROTOCOL_VERSION;
                telem.sequence         = s_telemetry_seq++;
                telem.status_flags     = (uint16_t)fsm_state;
                telem.angle_q16_16     = float_to_q16_16(attitude.pitch_deg);
                telem.gyro_q16_16      = float_to_q16_16(attitude.gyro_dps[1]);
                telem.left_speed_q16_16  = float_to_q16_16(speed_left);
                telem.right_speed_q16_16 = float_to_q16_16(speed_right);
                telem.left_pwm         = left_pwm;
                telem.right_pwm        = right_pwm;
                telem.fault_code       = (uint16_t)safety_fsm_get_fault_flags(&s_safety_fsm);
                telem.timestamp_ms     = current_time_ms;

                (void)serial_protocol_send(SERIAL_CMD_TELEMETRY, &telem, sizeof(telem));
            }
        }

        s_cycle_counter++;
    }
}

/* ========================================================================= */
/*                          Public API Implementation                        */
/* ========================================================================= */

void balance_task_default_config(balance_task_config_t *config) {
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->attitude_queue       = NULL;
    config->pid_config_queue     = NULL;
    config->motion_command_queue = NULL;
    config->system_command_queue = NULL;
    config->motor_output_hook    = NULL;
    config->encoder_read_hook    = NULL;
    config->priority             = BALANCE_TASK_DEFAULT_PRIORITY;
    config->stack_depth_words    = BALANCE_TASK_STACK_WORDS;
    config->enable_telemetry     = false;
    config->telemetry_divisor    = BALANCE_TELEMETRY_DEFAULT_DIVISOR;
}

BaseType_t balance_task_start(const balance_task_config_t *config,
                              TaskHandle_t *task_handle) {
    if (config == NULL || config->attitude_queue == NULL) {
        return pdFAIL;
    }

    s_config = *config;
    if (s_config.priority == 0U) {
        s_config.priority = BALANCE_TASK_DEFAULT_PRIORITY;
    }
    if (s_config.stack_depth_words == 0U) {
        s_config.stack_depth_words = BALANCE_TASK_STACK_WORDS;
    }
    if (s_config.telemetry_divisor == 0U) {
        s_config.telemetry_divisor = BALANCE_TELEMETRY_DEFAULT_DIVISOR;
    }

    /* Initialize controller and safety FSM */
    balance_controller_init(&s_controller, NULL);
    safety_fsm_init(&s_safety_fsm, NULL);

    /* Reset internal state */
    s_attitude_valid             = false;
    s_telemetry_seq              = 0U;
    s_cycle_counter              = 0U;
    memset(&s_latest_outputs, 0, sizeof(s_latest_outputs));

    BaseType_t ret = xTaskCreate(balance_task_entry,
                                 "BalanceTask",
                                 s_config.stack_depth_words,
                                 NULL,
                                 (UBaseType_t)s_config.priority,
                                 &s_task_handle);

    if (ret == pdPASS) {
        s_initialized = true;
        if (task_handle != NULL) {
            *task_handle = s_task_handle;
        }
    }

    return ret;
}

BaseType_t balance_task_init_and_start(QueueHandle_t attitude_queue,
                                      QueueHandle_t pid_config_queue,
                                      QueueHandle_t motion_command_queue,
                                      balance_motor_output_fn motor_hook,
                                      TaskHandle_t *task_handle) {
    balance_task_config_t cfg;
    balance_task_default_config(&cfg);
    cfg.attitude_queue       = attitude_queue;
    cfg.pid_config_queue     = pid_config_queue;
    cfg.motion_command_queue = motion_command_queue;
    cfg.motor_output_hook    = motor_hook;

    return balance_task_start(&cfg, task_handle);
}

void balance_task_set_motor_output_hook(balance_motor_output_fn hook) {
    s_config.motor_output_hook = hook;
}

void balance_task_set_encoder_read_hook(balance_encoder_read_fn hook) {
    s_config.encoder_read_hook = hook;
}

bool balance_task_request_arm(void) {
    if (!s_initialized) {
        return false;
    }
    return safety_fsm_request_arm(&s_safety_fsm);
}

void balance_task_request_disarm(void) {
    if (!s_initialized) {
        return;
    }
    safety_fsm_request_disarm(&s_safety_fsm);
}

bool balance_task_reset_fault(void) {
    if (!s_initialized) {
        return false;
    }
    return safety_fsm_reset_fault(&s_safety_fsm);
}

void balance_task_emergency_stop(void) {
    if (!s_initialized) {
        return;
    }
    safety_fsm_emergency_stop(&s_safety_fsm);
}

balance_state_t balance_task_get_state(void) {
    return safety_fsm_get_state(&s_safety_fsm);
}

const balance_controller_t *balance_task_get_controller(void) {
    return &s_controller;
}

const safety_fsm_t *balance_task_get_safety_fsm(void) {
    return &s_safety_fsm;
}

void balance_task_get_latest_outputs(balance_controller_outputs_t *outputs) {
    if (outputs != NULL) {
        *outputs = s_latest_outputs;
    }
}

void balance_task_set_telemetry_enabled(bool enable, uint8_t divisor) {
    s_config.enable_telemetry  = enable;
    s_config.telemetry_divisor = (divisor > 0U) ? divisor : BALANCE_TELEMETRY_DEFAULT_DIVISOR;
}
