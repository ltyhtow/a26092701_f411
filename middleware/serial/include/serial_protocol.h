#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LwPKT command values used by the application protocol. */
#define SERIAL_CMD_MOTION       0x01U
#define SERIAL_CMD_PID_CONFIG   0x02U
#define SERIAL_CMD_SYSTEM       0x03U
#define SERIAL_CMD_TELEMETRY    0x81U
#define SERIAL_CMD_PID_ACK      0x82U
#define SERIAL_CMD_RESPONSE     0x83U
#define SERIAL_CMD_IMU_STATUS   0x84U
#define SERIAL_CMD_IMU_DIAGNOSTIC 0x85U
#define SERIAL_CMD_IMU_ATTITUDE 0x86U

#define SERIAL_PROTOCOL_VERSION 1U

typedef enum {
    SERIAL_PROTOCOL_OK = 0,
    SERIAL_PROTOCOL_NOT_READY = -1,
    SERIAL_PROTOCOL_BUSY = -2,
    SERIAL_PROTOCOL_INVALID_ARGUMENT = -3,
    SERIAL_PROTOCOL_TX_FULL = -4,
    SERIAL_PROTOCOL_ERROR = -5,
} serial_protocol_result_t;

typedef struct {
    uint32_t rx_crc_errors;
    uint32_t rx_stop_errors;
    uint32_t rx_memory_errors;
    uint32_t rx_timeouts;
    uint32_t unknown_commands;
    uint32_t queue_overruns;
} serial_protocol_stats_t;

typedef struct {
    uint8_t  version;
    uint8_t  sequence;
    uint8_t  reserved[2];
    int32_t  linear_q16_16;
    int32_t  yaw_q16_16;
    uint16_t timeout_ms;
    uint16_t flags;
} motion_command_t;

typedef struct {
    uint8_t  version;
    uint8_t  sequence;
    uint8_t  loop_id;
    uint8_t  reserved;
    int32_t  kp_q16_16;
    int32_t  ki_q16_16;
    int32_t  kd_q16_16;
    int32_t  integral_limit_q16_16;
} pid_config_command_t;

typedef struct {
    uint8_t version;
    uint8_t sequence;
    uint8_t action;
    uint8_t reserved;
} system_command_t;

typedef struct {
    uint8_t  version;
    uint8_t  sequence;
    uint16_t status_flags;
    int32_t  angle_q16_16;
    int32_t  gyro_q16_16;
    int32_t  left_speed_q16_16;
    int32_t  right_speed_q16_16;
    int16_t  left_pwm;
    int16_t  right_pwm;
    uint16_t battery_mv;
    uint16_t fault_code;
    uint32_t timestamp_ms;
} motion_telemetry_t;

typedef struct {
    uint8_t  version;
    uint8_t  sequence;
    uint16_t status_flags;      /* bit0: sample valid */
    int16_t  accel_raw[3];
    int16_t  gyro_raw[3];
    uint16_t error_code;
    uint16_t reserved;
    uint32_t timestamp_ms;
} imu_telemetry_t;

typedef struct {
    uint8_t  version;
    uint8_t  sequence;
    uint8_t  address_8bit;
    uint8_t  who_am_i;
    uint16_t init_result;
    uint16_t reserved0;
    uint32_t hal_status;
    uint32_t hal_error_codes;
    uint16_t reserved1;
    uint32_t timestamp_ms;
} imu_diagnostic_t;

typedef struct {
    uint8_t  version;
    uint8_t  sequence;
    uint16_t status_flags;
    uint16_t calibration_samples;
    uint16_t reserved;
    int32_t  roll_q16_16;
    int32_t  pitch_q16_16;
    int32_t  yaw_q16_16;
    int32_t  gyro_bias_x_q16_16;
    int32_t  gyro_bias_y_q16_16;
    int32_t  gyro_bias_z_q16_16;
    int32_t  quaternion_w_q30;
    int32_t  quaternion_x_q30;
    int32_t  quaternion_y_q30;
    int32_t  quaternion_z_q30;
    uint32_t timestamp_ms;
} imu_attitude_telemetry_t;

_Static_assert(sizeof(motion_command_t) == 16U, "motion command wire size");
_Static_assert(sizeof(pid_config_command_t) == 20U, "pid command wire size");
_Static_assert(sizeof(system_command_t) == 4U, "system command wire size");
_Static_assert(sizeof(motion_telemetry_t) == 32U, "motion telemetry wire size");
_Static_assert(sizeof(imu_telemetry_t) == 24U, "imu telemetry wire size");
_Static_assert(sizeof(imu_diagnostic_t) == 24U, "imu diagnostic wire size");
_Static_assert(sizeof(imu_attitude_telemetry_t) == 52U, "imu attitude wire size");

/* Thread-safe packet submission API for FreeRTOS task context. */
serial_protocol_result_t serial_protocol_send(uint32_t command, const void *data, size_t length);
void serial_protocol_get_stats(serial_protocol_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PROTOCOL_H */
