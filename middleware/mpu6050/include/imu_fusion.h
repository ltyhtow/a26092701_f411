#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <stdint.h>
#include <stddef.h>

#include "imu_port.h"
#include "robot_types.h"

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

typedef struct imu_fusion_context imu_fusion_t;
typedef attitude_sample_t imu_fusion_output_t;
/* Caller owns malloc-aligned storage of this size; no allocation in the core. */
size_t imu_fusion_context_size(void);

void imu_fusion_init(imu_fusion_t *fusion);

void imu_fusion_set_alignment(imu_fusion_t *fusion,
                              uint8_t alignment);

uint8_t imu_fusion_update(imu_fusion_t *fusion,
                          const imu_sample_t *sample,
                          uint32_t timestamp_ms,
                          imu_fusion_output_t *output);

uint8_t imu_fusion_is_calibrated(const imu_fusion_t *fusion);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FUSION_H */
