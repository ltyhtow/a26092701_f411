#include "serial_codec.h"
#include <limits.h>
#include <string.h>

static void put16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}
static void put32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4U; ++i) p[i] = (uint8_t)(v >> (8U * i));
}
static void put64(uint8_t *p, uint64_t v) {
    for (unsigned i = 0; i < 8U; ++i) p[i] = (uint8_t)(v >> (8U * i));
}
static uint16_t get16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}
static uint32_t get32(const uint8_t *p) {
    uint32_t v = 0U;
    for (unsigned i = 0; i < 4U; ++i) v |= (uint32_t)p[i] << (8U * i);
    return v;
}
static uint64_t get64(const uint8_t *p) {
    uint64_t v = 0U;
    for (unsigned i = 0; i < 8U; ++i) v |= (uint64_t)p[i] << (8U * i);
    return v;
}
/* Avoid implementation-defined conversion of unsigned values above INT_MAX. */
static int16_t signed16(uint16_t v) {
    return v <= INT16_MAX ? (int16_t)v : (int16_t)(-1 - (int32_t)(UINT16_MAX - v));
}
static int32_t signed32(uint32_t v) {
    return v <= INT32_MAX ? (int32_t)v : -1 - (int32_t)(UINT32_MAX - v);
}
static int64_t signed64(uint64_t v) {
    return v <= INT64_MAX ? (int64_t)v : -1 - (int64_t)(UINT64_MAX - v);
}

size_t serial_codec_wire_size(uint32_t command) {
    switch (command) {
        case SERIAL_CMD_MOTION: return 16U;
        case SERIAL_CMD_PID_CONFIG: return 20U;
        case SERIAL_CMD_SYSTEM: return 4U;
        case SERIAL_CMD_PARAMETER: return 8U;
        case SERIAL_CMD_WHEEL_CONTROL: return 4U;
        case SERIAL_CMD_WHEEL_TEST: return 32U;
        case SERIAL_CMD_PID_ACK:
        case SERIAL_CMD_RESPONSE: return 24U;
        case SERIAL_CMD_TELEMETRY: return 32U;
        case SERIAL_CMD_IMU_STATUS: return 24U;
        case SERIAL_CMD_IMU_DIAGNOSTIC: return 24U;
        case SERIAL_CMD_IMU_ATTITUDE: return 52U;
        case SERIAL_CMD_ENCODER_TEST: return 56U;
        default: return 0U;
    }
}

size_t serial_codec_native_size(uint32_t command) {
    switch (command) {
        case SERIAL_CMD_MOTION: return sizeof(motion_command_t);
        case SERIAL_CMD_PID_CONFIG: return sizeof(pid_config_command_t);
        case SERIAL_CMD_SYSTEM: return sizeof(system_command_t);
        case SERIAL_CMD_PARAMETER: return sizeof(parameter_command_t);
        case SERIAL_CMD_WHEEL_CONTROL: return sizeof(wheel_test_command_t);
        case SERIAL_CMD_WHEEL_TEST: return sizeof(wheel_test_telemetry_t);
        case SERIAL_CMD_PID_ACK:
        case SERIAL_CMD_RESPONSE: return sizeof(command_response_t);
        case SERIAL_CMD_TELEMETRY: return sizeof(motion_telemetry_t);
        case SERIAL_CMD_IMU_STATUS: return sizeof(imu_telemetry_t);
        case SERIAL_CMD_IMU_DIAGNOSTIC: return sizeof(imu_diagnostic_t);
        case SERIAL_CMD_IMU_ATTITUDE: return sizeof(imu_attitude_telemetry_t);
        case SERIAL_CMD_ENCODER_TEST: return sizeof(encoder_test_telemetry_t);
        default: return 0U;
    }
}

bool serial_codec_encode(uint32_t command, const void *native, size_t native_size,
                         uint8_t *wire, size_t capacity, size_t *written) {
    const size_t size = serial_codec_wire_size(command);
    if (written != NULL) *written = 0U;
    if (size == 0U || native == NULL || wire == NULL || capacity < size ||
        native_size != serial_codec_native_size(command)) return false;
    /* version/sequence are the common initial bytes of every DTO. */
    const uint8_t *common = native;
    if (common[0] != SERIAL_PROTOCOL_VERSION) return false;
    memset(wire, 0, size);
    wire[0] = common[0]; wire[1] = common[1];
    switch (command) {
        case SERIAL_CMD_WHEEL_CONTROL: {
            const wheel_test_command_t *v = native;
            if (v->action < 1U || v->action > 3U) return false;
            wire[2] = v->action;
            break;
        }
        case SERIAL_CMD_WHEEL_TEST: {
            const wheel_test_telemetry_t *v = native;
            if (v->phase > 10U || v->reason > 5U || (v->flags & ~0xFU)) return false;
            wire[2] = v->phase; wire[3] = v->reason;
            put32(wire + 4, v->timestamp_ms); put32(wire + 8, v->phase_elapsed_ms);
            put16(wire + 12, (uint16_t)v->left_pwm); put16(wire + 14, (uint16_t)v->right_pwm);
            put32(wire + 16, v->heartbeat_age_ms); put16(wire + 20, v->flags);
            put32(wire + 24, v->remaining_ms); put32(wire + 28, v->test_elapsed_ms);
            break;
        }
        case SERIAL_CMD_MOTION: {
            const motion_command_t *v = native;
            if ((v->flags & ~0x0003U) != 0U) return false;
            put32(wire + 4, (uint32_t)v->linear_q16_16);
            put32(wire + 8, (uint32_t)v->yaw_q16_16);
            put16(wire + 12, v->timeout_ms); put16(wire + 14, v->flags);
            break;
        }
        case SERIAL_CMD_PID_CONFIG: {
            const pid_config_command_t *v = native;
            if (v->loop_id > 2U) return false;
            wire[2] = v->loop_id;
            put32(wire + 4, (uint32_t)v->kp_q16_16);
            put32(wire + 8, (uint32_t)v->ki_q16_16);
            put32(wire + 12, (uint32_t)v->kd_q16_16);
            put32(wire + 16, (uint32_t)v->integral_limit_q16_16);
            break;
        }
        case SERIAL_CMD_SYSTEM: {
            const system_command_t *v = native;
            if (v->action < 1U || v->action > 4U) return false;
            wire[2] = v->action;
            break;
        }
        case SERIAL_CMD_PARAMETER: {
            const parameter_command_t *v = native;
            wire[2] = v->operation; wire[3] = v->key;
            put32(wire + 4, (uint32_t)v->value_q16_16);
            break;
        }
        case SERIAL_CMD_PID_ACK:
        case SERIAL_CMD_RESPONSE: {
            const command_response_t *v = native;
            wire[2] = v->request_command; wire[3] = v->status;
            wire[4] = v->key; wire[5] = v->controller_state;
            put16(wire + 6, v->storage_flags);
            put32(wire + 8, (uint32_t)v->value_q16_16);
            put32(wire + 12, v->fault_flags); put32(wire + 16, v->config_revision);
            put32(wire + 20, v->timestamp_ms);
            break;
        }
        case SERIAL_CMD_TELEMETRY: {
            const motion_telemetry_t *v = native;
            put16(wire + 2, v->status_flags);
            put32(wire + 4, (uint32_t)v->angle_q16_16);
            put32(wire + 8, (uint32_t)v->gyro_q16_16);
            put32(wire + 12, (uint32_t)v->left_speed_q16_16);
            put32(wire + 16, (uint32_t)v->right_speed_q16_16);
            put16(wire + 20, (uint16_t)v->left_pwm); put16(wire + 22, (uint16_t)v->right_pwm);
            put16(wire + 24, v->battery_mv); put16(wire + 26, v->fault_code);
            put32(wire + 28, v->timestamp_ms);
            break;
        }
        case SERIAL_CMD_IMU_STATUS: {
            const imu_telemetry_t *v = native;
            put16(wire + 2, v->status_flags);
            for (unsigned i = 0; i < 3U; ++i) {
                put16(wire + 4 + 2U * i, (uint16_t)v->accel_raw[i]);
                put16(wire + 10 + 2U * i, (uint16_t)v->gyro_raw[i]);
            }
            put16(wire + 16, v->error_code); put32(wire + 20, v->timestamp_ms);
            break;
        }
        case SERIAL_CMD_IMU_DIAGNOSTIC: {
            const imu_diagnostic_t *v = native;
            wire[2] = v->address_8bit; wire[3] = v->who_am_i;
            put16(wire + 4, v->init_result);
            put32(wire + 8, v->hal_status); put32(wire + 12, v->hal_error_codes);
            /* Bytes 18/19 were ARM ABI padding; preserve the 24-byte legacy
             * payload while making these bytes deterministic reserved zeroes. */
            put32(wire + 20, v->timestamp_ms);
            break;
        }
        case SERIAL_CMD_IMU_ATTITUDE: {
            const imu_attitude_telemetry_t *v = native;
            put16(wire + 2, v->status_flags); put16(wire + 4, v->calibration_samples);
            put32(wire + 8, (uint32_t)v->roll_q16_16);
            put32(wire + 12, (uint32_t)v->pitch_q16_16);
            put32(wire + 16, (uint32_t)v->yaw_q16_16);
            put32(wire + 20, (uint32_t)v->gyro_bias_x_q16_16);
            put32(wire + 24, (uint32_t)v->gyro_bias_y_q16_16);
            put32(wire + 28, (uint32_t)v->gyro_bias_z_q16_16);
            put32(wire + 32, (uint32_t)v->quaternion_w_q30);
            put32(wire + 36, (uint32_t)v->quaternion_x_q30);
            put32(wire + 40, (uint32_t)v->quaternion_y_q30);
            put32(wire + 44, (uint32_t)v->quaternion_z_q30);
            put32(wire + 48, v->timestamp_ms);
            break;
        }
        case SERIAL_CMD_ENCODER_TEST: {
            const encoder_test_telemetry_t *v = native;
            put16(wire + 2, v->flags); put32(wire + 4, v->timestamp_ms);
            put32(wire + 8, v->sample_period_ms); put32(wire + 12, v->raw_left);
            put32(wire + 16, v->raw_right); put32(wire + 20, (uint32_t)v->delta_left);
            put32(wire + 24, (uint32_t)v->delta_right); put32(wire + 28, v->gpio_levels);
            put64(wire + 32, (uint64_t)v->total_left); put64(wire + 40, (uint64_t)v->total_right);
            put32(wire + 48, v->tx_dropped); put32(wire + 52, v->sample_errors);
            break;
        }
        default: return false;
    }
    if (written != NULL) *written = size;
    return true;
}

bool serial_codec_decode(uint32_t command, const uint8_t *wire, size_t length,
                         void *native, size_t native_capacity) {
    const size_t size = serial_codec_wire_size(command);
    const size_t native_size = serial_codec_native_size(command);
    if (size == 0U || wire == NULL || native == NULL || length != size ||
        native_capacity < native_size || wire[0] != SERIAL_PROTOCOL_VERSION) return false;
    /* Validate first; failed decode must leave the destination untouched. */
    switch (command) {
        case SERIAL_CMD_WHEEL_CONTROL:
            if (wire[2] < 1U || wire[2] > 3U || wire[3]) return false;
            break;
        case SERIAL_CMD_WHEEL_TEST:
            if (wire[2] > 10U || wire[3] > 5U || (get16(wire + 20) & ~0xFU) || wire[22] || wire[23]) return false;
            break;
        case SERIAL_CMD_MOTION:
            if (wire[2] || wire[3] || (get16(wire + 14) & ~0x0003U)) return false;
            break;
        case SERIAL_CMD_PID_CONFIG:
            if (wire[2] > 2U || wire[3]) return false;
            break;
        case SERIAL_CMD_SYSTEM:
            if (wire[2] < 1U || wire[2] > 4U || wire[3]) return false;
            break;
        case SERIAL_CMD_IMU_STATUS:
            if (wire[18] || wire[19]) return false;
            break;
        case SERIAL_CMD_IMU_DIAGNOSTIC:
            if (wire[6] || wire[7] || wire[16] || wire[17] || wire[18] || wire[19]) return false;
            break;
        case SERIAL_CMD_IMU_ATTITUDE:
            if (wire[6] || wire[7]) return false;
            break;
        default: break;
    }
    memset(native, 0, native_size);
    uint8_t *common = native;
    common[0] = wire[0]; common[1] = wire[1];
    switch (command) {
        case SERIAL_CMD_WHEEL_CONTROL: ((wheel_test_command_t *)native)->action = wire[2]; break;
        case SERIAL_CMD_WHEEL_TEST: {
            wheel_test_telemetry_t *v = native;
            v->phase = wire[2]; v->reason = wire[3];
            v->timestamp_ms = get32(wire + 4); v->phase_elapsed_ms = get32(wire + 8);
            v->left_pwm = signed16(get16(wire + 12)); v->right_pwm = signed16(get16(wire + 14));
            v->heartbeat_age_ms = get32(wire + 16); v->flags = get16(wire + 20);
            v->remaining_ms = get32(wire + 24); v->test_elapsed_ms = get32(wire + 28);
            break;
        }
        case SERIAL_CMD_MOTION: {
            motion_command_t *v = native;
            v->linear_q16_16 = signed32(get32(wire + 4));
            v->yaw_q16_16 = signed32(get32(wire + 8));
            v->timeout_ms = get16(wire + 12); v->flags = get16(wire + 14); break;
        }
        case SERIAL_CMD_PID_CONFIG: {
            pid_config_command_t *v = native;
            v->loop_id = wire[2]; v->kp_q16_16 = signed32(get32(wire + 4));
            v->ki_q16_16 = signed32(get32(wire + 8)); v->kd_q16_16 = signed32(get32(wire + 12));
            v->integral_limit_q16_16 = signed32(get32(wire + 16)); break;
        }
        case SERIAL_CMD_SYSTEM: ((system_command_t *)native)->action = wire[2]; break;
        case SERIAL_CMD_PARAMETER: {
            parameter_command_t *v = native;
            v->operation = wire[2]; v->key = wire[3];
            v->value_q16_16 = signed32(get32(wire + 4)); break;
        }
        case SERIAL_CMD_PID_ACK:
        case SERIAL_CMD_RESPONSE: {
            command_response_t *v = native;
            v->request_command = wire[2]; v->status = wire[3];
            v->key = wire[4]; v->controller_state = wire[5];
            v->storage_flags = get16(wire + 6);
            v->value_q16_16 = signed32(get32(wire + 8));
            v->fault_flags = get32(wire + 12); v->config_revision = get32(wire + 16);
            v->timestamp_ms = get32(wire + 20); break;
        }
        case SERIAL_CMD_TELEMETRY: {
            motion_telemetry_t *v = native;
            v->status_flags = get16(wire + 2); v->angle_q16_16 = signed32(get32(wire + 4));
            v->gyro_q16_16 = signed32(get32(wire + 8));
            v->left_speed_q16_16 = signed32(get32(wire + 12));
            v->right_speed_q16_16 = signed32(get32(wire + 16));
            v->left_pwm = signed16(get16(wire + 20)); v->right_pwm = signed16(get16(wire + 22));
            v->battery_mv = get16(wire + 24); v->fault_code = get16(wire + 26);
            v->timestamp_ms = get32(wire + 28); break;
        }
        case SERIAL_CMD_IMU_STATUS: {
            imu_telemetry_t *v = native;
            v->status_flags = get16(wire + 2);
            for (unsigned i = 0; i < 3U; ++i) {
                v->accel_raw[i] = signed16(get16(wire + 4 + 2U * i));
                v->gyro_raw[i] = signed16(get16(wire + 10 + 2U * i));
            }
            v->error_code = get16(wire + 16); v->timestamp_ms = get32(wire + 20); break;
        }
        case SERIAL_CMD_IMU_DIAGNOSTIC: {
            imu_diagnostic_t *v = native;
            v->address_8bit = wire[2]; v->who_am_i = wire[3]; v->init_result = get16(wire + 4);
            v->hal_status = get32(wire + 8); v->hal_error_codes = get32(wire + 12);
            v->timestamp_ms = get32(wire + 20); break;
        }
        case SERIAL_CMD_IMU_ATTITUDE: {
            imu_attitude_telemetry_t *v = native;
            v->status_flags = get16(wire + 2); v->calibration_samples = get16(wire + 4);
            v->roll_q16_16 = signed32(get32(wire + 8)); v->pitch_q16_16 = signed32(get32(wire + 12));
            v->yaw_q16_16 = signed32(get32(wire + 16));
            v->gyro_bias_x_q16_16 = signed32(get32(wire + 20));
            v->gyro_bias_y_q16_16 = signed32(get32(wire + 24));
            v->gyro_bias_z_q16_16 = signed32(get32(wire + 28));
            v->quaternion_w_q30 = signed32(get32(wire + 32));
            v->quaternion_x_q30 = signed32(get32(wire + 36));
            v->quaternion_y_q30 = signed32(get32(wire + 40));
            v->quaternion_z_q30 = signed32(get32(wire + 44));
            v->timestamp_ms = get32(wire + 48); break;
        }
        case SERIAL_CMD_ENCODER_TEST: {
            encoder_test_telemetry_t *v = native;
            v->flags = get16(wire + 2); v->timestamp_ms = get32(wire + 4);
            v->sample_period_ms = get32(wire + 8); v->raw_left = get32(wire + 12);
            v->raw_right = get32(wire + 16); v->delta_left = signed32(get32(wire + 20));
            v->delta_right = signed32(get32(wire + 24)); v->gpio_levels = get32(wire + 28);
            v->total_left = signed64(get64(wire + 32)); v->total_right = signed64(get64(wire + 40));
            v->tx_dropped = get32(wire + 48); v->sample_errors = get32(wire + 52); break;
        }
        default: return false;
    }
    return true;
}
