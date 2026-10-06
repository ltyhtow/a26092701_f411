#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <stdint.h>

#include "FusionAhrs.h"
#include "FusionBias.h"
#include "FusionRemap.h"
#include "imu_port.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_FUSION_STATUS_CALIBRATING        0x0001U
#define IMU_FUSION_STATUS_CALIBRATED         0x0002U
#define IMU_FUSION_STATUS_ATTITUDE_VALID     0x0004U
#define IMU_FUSION_STATUS_STATIONARY         0x0008U
#define IMU_FUSION_STATUS_ACCEL_IGNORED      0x0010U
#define IMU_FUSION_STATUS_STARTUP            0x0020U
#define IMU_FUSION_STATUS_INPUT_INVALID      0x0040U

typedef struct {
    FusionAhrs ahrs;
    FusionBias bias;
    FusionRemapAlignment alignment;
    FusionVector gyro_calibration_sum;
    FusionVector calibration_previous_gyro;
    FusionVector calibration_previous_accel;
    uint32_t calibration_samples;
    uint32_t last_timestamp_ms;
    uint8_t calibrated;
    uint8_t has_timestamp;
    uint8_t calibration_has_previous;
} imu_fusion_t;

typedef struct {
    uint32_t timestamp_ms;
    uint16_t status_flags;
    uint16_t calibration_samples;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_bias_dps[3];
    FusionQuaternion quaternion;
} imu_fusion_output_t;

void imu_fusion_init(imu_fusion_t *fusion);

void imu_fusion_set_alignment(imu_fusion_t *fusion,
                              FusionRemapAlignment alignment);

uint8_t imu_fusion_update(imu_fusion_t *fusion,
                          const imu_sample_t *sample,
                          uint32_t timestamp_ms,
                          imu_fusion_output_t *output);

uint8_t imu_fusion_is_calibrated(const imu_fusion_t *fusion);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FUSION_H */
