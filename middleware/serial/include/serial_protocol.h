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
#define SERIAL_CMD_PARAMETER    0x04U
#define SERIAL_CMD_WHEEL_CONTROL 0x06U
#define SERIAL_CMD_TELEMETRY    0x81U
#define SERIAL_CMD_PID_ACK      0x82U
#define SERIAL_CMD_RESPONSE     0x83U
#define SERIAL_CMD_IMU_STATUS   0x84U
#define SERIAL_CMD_IMU_DIAGNOSTIC 0x85U
#define SERIAL_CMD_IMU_ATTITUDE 0x86U
#define SERIAL_CMD_ENCODER_TEST 0x87U
#define SERIAL_CMD_WHEEL_TEST   0x88U

typedef struct {
    uint8_t version, sequence, action, reserved; /* START=1, STOP=2, heartbeat=3 */
} wheel_test_command_t;
typedef struct {
    uint8_t version, sequence, phase, reason;
    uint32_t timestamp_ms, phase_elapsed_ms;
    int16_t left_pwm, right_pwm;
    uint32_t heartbeat_age_ms;
    uint16_t flags, reserved;
    uint32_t remaining_ms, test_elapsed_ms;
} wheel_test_telemetry_t;

#define ENCODER_TEST_VALID          (1U << 0)
#define ENCODER_TEST_READY          (1U << 1)
#define ENCODER_TEST_MOTOR_LOCKED    (1U << 2)
#define ENCODER_TEST_INVERT_LEFT     (1U << 3)
#define ENCODER_TEST_INVERT_RIGHT    (1U << 4)

#define SERIAL_PROTOCOL_VERSION 1U

#define SERIAL_STORAGE_CONFIGURED (1U << 0)
#define SERIAL_STORAGE_READY      (1U << 1)
#define SERIAL_STORAGE_VALID      (1U << 2)
#define SERIAL_STORAGE_DIRTY      (1U << 3)
#define SERIAL_STORAGE_BUSY       (1U << 4)
#define SERIAL_STORAGE_ERROR      (1U << 5)

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
    uint32_t rx_invalid_payloads;
} serial_protocol_stats_t;

typedef struct {
    uint8_t  version;
    uint8_t  sequence;
    uint8_t  reserved[2];
    int32_t  linear_q16_16; /* Quadrature counts per 5 ms control sample. */
    int32_t  yaw_q16_16;    /* Degrees per second. */
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

typedef enum {
    PARAMETER_OP_GET = 1, PARAMETER_OP_SET = 2, PARAMETER_OP_SAVE = 3,
    PARAMETER_OP_LOAD = 4, PARAMETER_OP_DEFAULTS = 5, PARAMETER_OP_STATUS = 6
} parameter_operation_t;

typedef enum {
    SERIAL_RESPONSE_OK = 0, SERIAL_RESPONSE_BUSY = 1,
    SERIAL_RESPONSE_INVALID = 2, SERIAL_RESPONSE_UNSUPPORTED = 3,
    SERIAL_RESPONSE_UNSAFE = 4, SERIAL_RESPONSE_STORAGE_ERROR = 5,
    SERIAL_RESPONSE_NO_SAVED = 6, SERIAL_RESPONSE_NOT_READY = 7
} serial_response_status_t;

/* 0x04: eight wire bytes. Unsupported operation/key values reach the application
 * so it can return an explicit rejection with the original request sequence. */
typedef struct {
    uint8_t version, sequence, operation, key;
    int32_t value_q16_16;
} parameter_command_t;

/* 0x82 (legacy PID completion) and 0x83 (parameter/status result): 24 bytes.
 * sequence identifies the original request; key is loop_id for a PID result.
 * OK means the operation completed, never merely entered a queue. */
typedef struct {
    uint8_t version, sequence, request_command, status;
    uint8_t key, controller_state;
    uint16_t storage_flags;
    int32_t value_q16_16;
    uint32_t fault_flags, config_revision, timestamp_ms;
} command_response_t;

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

/* Native DTO only. serial_codec defines the fixed little-endian wire layout. */
typedef struct {
    uint8_t version;
    uint8_t sequence;
    uint16_t flags;
    uint32_t timestamp_ms;
    uint32_t sample_period_ms;
    uint32_t raw_left;
    uint32_t raw_right;
    int32_t delta_left;
    int32_t delta_right;
    uint32_t gpio_levels; /* bit0=L A, bit1=L B, bit2=R A, bit3=R B */
    int64_t total_left;
    int64_t total_right;
    uint32_t tx_dropped;
    uint32_t sample_errors;
} encoder_test_telemetry_t;

/* Task-context adapter API: data points to the command's native DTO; length is
 * sizeof that DTO (validation only). The codec, never struct layout, determines
 * the wire bytes. Commands without a defined payload are rejected. */
serial_protocol_result_t serial_protocol_send(uint32_t command, const void *data, size_t length);
/* Nonblocking submission for control tasks: BUSY/TX_FULL drops this frame. */
serial_protocol_result_t serial_protocol_try_send(uint32_t command, const void *data, size_t length);
void serial_protocol_get_stats(serial_protocol_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PROTOCOL_H */
