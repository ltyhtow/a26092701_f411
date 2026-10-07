#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* In-process domain contracts. Never transmit these structures as wire bytes. */
#define ATTITUDE_STATUS_CALIBRATING    0x0001U
#define ATTITUDE_STATUS_CALIBRATED     0x0002U
#define ATTITUDE_STATUS_ATTITUDE_VALID 0x0004U
#define ATTITUDE_STATUS_STATIONARY     0x0008U
#define ATTITUDE_STATUS_ACCEL_IGNORED  0x0010U
#define ATTITUDE_STATUS_STARTUP        0x0020U
#define ATTITUDE_STATUS_INPUT_INVALID  0x0040U

typedef struct { float w, x, y, z; } attitude_quaternion_t;
typedef struct {
    uint32_t timestamp_ms;
    uint16_t status_flags;
    uint16_t calibration_samples;
    float roll_deg, pitch_deg, yaw_deg;
    float gyro_dps[3];
    float gyro_bias_dps[3];
    attitude_quaternion_t quaternion;
} attitude_sample_t;

typedef struct {
    float linear_counts_per_5ms;
    float yaw_rate_dps;
    uint16_t timeout_ms;
    bool enable;
    bool clear_fault;
} motion_request_t;

typedef struct {
    uint8_t loop_id;
    float kp, ki, kd, integral_limit;
    uint32_t token; /* Opaque request correlation; never interpreted by control. */
} pid_request_t;

typedef enum {
    SYSTEM_ACTION_ARM = 1,
    SYSTEM_ACTION_DISARM = 2,
    SYSTEM_ACTION_RESET = 3,
    SYSTEM_ACTION_CALIBRATE = 4
} system_action_t;
typedef struct { system_action_t action; } system_request_t;

#endif
