#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

/* Keep the raw sampler active while leaving bring-up UART traffic opt-in. */
#define IMU_SAMPLE_PERIOD_MS     5U
#ifndef IMU_UART_TEST_ENABLED
#define IMU_UART_TEST_ENABLED    0U
#endif
#define IMU_UART_TEST_PERIOD_MS  50U

/* Fusion runs on the raw sample stream and keeps its own latest-value output. */
#define IMU_FUSION_SAMPLE_RATE_HZ             (1000.0f / (float)IMU_SAMPLE_PERIOD_MS)
#define IMU_FUSION_CALIBRATION_SAMPLES       400U
#define IMU_FUSION_CALIBRATION_GYRO_DELTA_DPS 0.75f
#define IMU_FUSION_CALIBRATION_ACCEL_DELTA_G  0.03f
#define IMU_FUSION_CALIBRATION_ACCEL_TOLERANCE_G 0.15f
#define IMU_FUSION_BIAS_STATIONARY_LIMIT_DPS  2.0f
#define IMU_FUSION_BIAS_STATIONARY_PERIOD_S  3.0f
#define IMU_FUSION_ALIGNMENT_INDEX            0U /* +X+Y+Z; see FusionRemap.h */
#define IMU_FUSION_AHRS_GAIN                  0.5f
#define IMU_FUSION_ACCEL_REJECTION_DEG       10.0f
#define IMU_FUSION_REJECTION_TIMEOUT_S        5.0f

#endif /* IMU_CONFIG_H */
