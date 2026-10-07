#include "app_protocol.h"
#include "app_config.h"
#if WHEEL_TEST_ENABLED
#include "wheel_test_task.h"
#endif
#include "balance_task.h"
#include "serial_protocol.h"
#include <limits.h>
#include <math.h>

static void respond(uint32_t command, uint8_t sequence, uint8_t request_command,
                    uint8_t key, serial_response_status_t status, uint32_t revision) {
    balance_runtime_snapshot_t snapshot;
    balance_task_get_snapshot(&snapshot);
    const command_response_t response = {
        .version = SERIAL_PROTOCOL_VERSION, .sequence = sequence,
        .request_command = request_command, .status = (uint8_t)status, .key = key,
        .controller_state = (uint8_t)snapshot.state, .fault_flags = snapshot.fault_flags,
        .config_revision = revision, .timestamp_ms = snapshot.timestamp_ms
    };
    (void)serial_protocol_try_send(command, &response, sizeof(response));
}
void app_protocol_pid_result(const pid_request_t *request, balance_config_result_t result, uint32_t revision) {
    if (request == NULL) return;
    serial_response_status_t status = SERIAL_RESPONSE_INVALID;
    switch (result) {
        case BALANCE_CONFIG_OK: status = SERIAL_RESPONSE_OK; break;
        case BALANCE_CONFIG_BUSY: status = SERIAL_RESPONSE_BUSY; break;
        case BALANCE_CONFIG_UNSAFE: status = SERIAL_RESPONSE_UNSAFE; break;
        case BALANCE_CONFIG_NOT_READY: status = SERIAL_RESPONSE_NOT_READY; break;
        default: break;
    }
    respond(SERIAL_CMD_PID_ACK, (uint8_t)request->token, SERIAL_CMD_PID_CONFIG,
            request->loop_id, status, revision);
}

bool app_protocol_receive(uint32_t command, const serial_message_t *message, void *context) {
#if WHEEL_TEST_ENABLED
    (void)context;
    if (message == NULL) return false;
    if (command == SERIAL_CMD_WHEEL_CONTROL) {
        (void)wheel_test_task_command(message->wheel_control.action);
    } else if ((command == SERIAL_CMD_SYSTEM && message->system.action == SYSTEM_ACTION_DISARM) ||
               (command == SERIAL_CMD_MOTION && (message->motion.flags & 1U) == 0U)) {
        (void)wheel_test_task_command(WHEEL_TEST_STOP);
    }
    return true; /* Status stream reports phase/stop reason; all other commands are ignored. */
#elif ENCODER_PUSH_TEST_ENABLED || MOTOR_POLARITY_TEST_ENABLED
    (void)command; (void)message; (void)context;
    return true; /* Passive/diagnostic modes never arm a closed-loop controller. */
#else
    app_command_queues_t *queues = context;
    if (!queues || !message) return false;
    if (balance_task_is_maintenance() &&
        ((command == SERIAL_CMD_MOTION && (message->motion.flags & 1U) != 0U) ||
         (command == SERIAL_CMD_SYSTEM && message->system.action != SYSTEM_ACTION_DISARM))) {
        respond(SERIAL_CMD_RESPONSE, message->motion.sequence, (uint8_t)command, 0U,
                SERIAL_RESPONSE_BUSY, balance_task_get_config_revision());
        return true;
    }
    switch (command) {
        case SERIAL_CMD_MOTION: {
            motion_request_t request = {
                .linear_counts_per_5ms = (float)message->motion.linear_q16_16 / 65536.0f,
                .yaw_rate_dps = (float)message->motion.yaw_q16_16 / 65536.0f,
                .timeout_ms = message->motion.timeout_ms,
                .enable = (message->motion.flags & 1U) != 0U,
                .clear_fault = (message->motion.flags & 2U) != 0U
            };
            if (!request.enable) balance_task_request_disarm();
            return xQueueOverwrite(queues->motion, &request) == pdPASS;
        }
        case SERIAL_CMD_PID_CONFIG: {
            pid_request_t request = {
                .loop_id = message->pid.loop_id,
                .kp = (float)message->pid.kp_q16_16 / 65536.0f,
                .ki = (float)message->pid.ki_q16_16 / 65536.0f,
                .kd = (float)message->pid.kd_q16_16 / 65536.0f,
                .integral_limit = (float)message->pid.integral_limit_q16_16 / 65536.0f,
                .token = message->pid.sequence
            };
            if (balance_task_is_maintenance()) {
                app_protocol_pid_result(&request, BALANCE_CONFIG_BUSY, balance_task_get_config_revision());
                return true;
            }
            /* Match the persisted profile policy without coupling the control
             * library to the parameter or protocol implementation. */
            if (fabsf(request.kp) > 10000.0f || fabsf(request.ki) > 10000.0f ||
                fabsf(request.kd) > 10000.0f || request.integral_limit < 0.0f ||
                request.integral_limit > 30000.0f) {
                app_protocol_pid_result(&request, BALANCE_CONFIG_INVALID, balance_task_get_config_revision());
                return true;
            }
            if (xQueueSend(queues->pid, &request, 0U) == pdPASS) return true;
            app_protocol_pid_result(&request, BALANCE_CONFIG_BUSY, balance_task_get_config_revision());
            return false;
        }
        case SERIAL_CMD_PARAMETER:
            if (queues->parameter_handler != NULL &&
                queues->parameter_handler(&message->parameter, queues->parameter_context)) return true;
            respond(SERIAL_CMD_RESPONSE, message->parameter.sequence, SERIAL_CMD_PARAMETER,
                    message->parameter.key,
                    queues->parameter_handler == NULL ? SERIAL_RESPONSE_NOT_READY : SERIAL_RESPONSE_BUSY,
                    balance_task_get_config_revision());
            return false;
        case SERIAL_CMD_SYSTEM: {
            system_request_t request = {.action = (system_action_t)message->system.action};
            if (request.action == SYSTEM_ACTION_DISARM) balance_task_request_disarm();
            return xQueueSend(queues->system, &request, 0U) == pdPASS;
        }
        default: return true; /* Known device-to-host frames are ignored, not queue failures. */
    }
#endif
}
static int32_t q16(float value) {
    if (!isfinite(value)) return 0;
    const double scaled = (double)value * 65536.0;
    if (scaled >= INT32_MAX) return INT32_MAX;
    if (scaled <= INT32_MIN) return INT32_MIN;
    return (int32_t)llround(scaled);
}
void app_protocol_telemetry(const balance_runtime_snapshot_t *snapshot) {
    static uint8_t sequence;
    if (!snapshot) return;
    motion_telemetry_t message = {
        .version = SERIAL_PROTOCOL_VERSION, .sequence = sequence++,
        .status_flags = (uint16_t)snapshot->state,
        .angle_q16_16 = q16(snapshot->attitude.pitch_deg),
        .gyro_q16_16 = q16(snapshot->attitude.gyro_dps[1]),
        .left_speed_q16_16 = q16(snapshot->left_counts_per_5ms),
        .right_speed_q16_16 = q16(snapshot->right_counts_per_5ms),
        .left_pwm = snapshot->outputs.left_pwm, .right_pwm = snapshot->outputs.right_pwm,
        .fault_code = (uint16_t)snapshot->fault_flags,
        .timestamp_ms = snapshot->timestamp_ms
    };
    (void)serial_protocol_try_send(SERIAL_CMD_TELEMETRY, &message, sizeof(message));
}
