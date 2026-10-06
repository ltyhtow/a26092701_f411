#include "imu_fusion.h"

#include "imu_config.h"

#include <math.h>
#include <string.h>

static FusionVector sample_accelerometer(const imu_sample_t *sample,
                                         const imu_fusion_t *fusion) {
    const FusionVector sensor = {
        .axis = {
            .x = sample->accel_g[0],
            .y = sample->accel_g[1],
            .z = sample->accel_g[2],
        },
    };
    return FusionRemap(sensor, fusion->alignment);
}

static FusionVector sample_gyroscope(const imu_sample_t *sample,
                                     const imu_fusion_t *fusion) {
    const FusionVector sensor = {
        .axis = {
            .x = sample->gyro_dps[0],
            .y = sample->gyro_dps[1],
            .z = sample->gyro_dps[2],
        },
    };
    return FusionRemap(sensor, fusion->alignment);
}

static float max_abs_axis(const FusionVector value) {
    float result = fabsf(value.axis.x);
    const float y = fabsf(value.axis.y);
    const float z = fabsf(value.axis.z);
    if (y > result) {
        result = y;
    }
    if (z > result) {
        result = z;
    }
    return result;
}

static uint8_t acceleration_is_level(const FusionVector accelerometer) {
    const float acceleration_norm = FusionVectorNorm(accelerometer);
    return (fabsf(acceleration_norm - 1.0f) <=
                IMU_FUSION_CALIBRATION_ACCEL_TOLERANCE_G)
               ? 1U
               : 0U;
}

static uint8_t is_stationary(const FusionVector gyroscope,
                             const FusionVector accelerometer) {
    return (max_abs_axis(gyroscope) <= IMU_FUSION_BIAS_STATIONARY_LIMIT_DPS &&
            acceleration_is_level(accelerometer) != 0U) ? 1U : 0U;
}

/* Startup calibration accepts a stable non-zero gyro output and averages it
 * as the bias. Requiring the raw rate to be near zero would reject the very
 * offset this phase is intended to measure. */
static uint8_t calibration_is_stable(imu_fusion_t *fusion,
                                     const FusionVector gyroscope,
                                     const FusionVector accelerometer) {
    uint8_t stable = 0U;

    if (acceleration_is_level(accelerometer) == 0U) {
        fusion->calibration_has_previous = 0U;
        return 0U;
    }
    if (fusion->calibration_has_previous == 0U) {
        stable = 1U;
    } else {
        const FusionVector gyro_delta =
            FusionVectorSubtract(gyroscope, fusion->calibration_previous_gyro);
        const FusionVector accel_delta =
            FusionVectorSubtract(accelerometer, fusion->calibration_previous_accel);
        stable = (max_abs_axis(gyro_delta) <=
                      IMU_FUSION_CALIBRATION_GYRO_DELTA_DPS &&
                  max_abs_axis(accel_delta) <=
                      IMU_FUSION_CALIBRATION_ACCEL_DELTA_G)
                     ? 1U
                     : 0U;
    }
    fusion->calibration_previous_gyro = gyroscope;
    fusion->calibration_previous_accel = accelerometer;
    fusion->calibration_has_previous = 1U;
    return stable;
}

static float sample_period(const imu_fusion_t *fusion, uint32_t timestamp_ms) {
    float period = 1.0f / IMU_FUSION_SAMPLE_RATE_HZ;

    if (fusion->has_timestamp != 0U) {
        const uint32_t elapsed_ms = timestamp_ms - fusion->last_timestamp_ms;
        if (elapsed_ms >= 1U && elapsed_ms <= 250U) {
            period = (float)elapsed_ms / 1000.0f;
        }
    }
    return period;
}

static void write_output(const imu_fusion_t *fusion,
                         const FusionVector gyroscope,
                         const FusionVector accelerometer,
                         uint32_t timestamp_ms,
                         uint16_t status_flags,
                         imu_fusion_output_t *output) {
    if (output == NULL) {
        return;
    }

    const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&fusion->ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(quaternion);
    const FusionVector offset = FusionBiasGetOffset(&fusion->bias);

    output->timestamp_ms = timestamp_ms;
    output->status_flags = status_flags;
    output->calibration_samples =
        (fusion->calibration_samples > UINT16_MAX)
            ? UINT16_MAX
            : (uint16_t)fusion->calibration_samples;
    output->roll_deg = euler.angle.roll;
    output->pitch_deg = euler.angle.pitch;
    output->yaw_deg = euler.angle.yaw;
    output->gyro_bias_dps[0] = offset.axis.x;
    output->gyro_bias_dps[1] = offset.axis.y;
    output->gyro_bias_dps[2] = offset.axis.z;
    output->quaternion = quaternion;

    if (is_stationary(gyroscope, accelerometer) != 0U) {
        output->status_flags |= IMU_FUSION_STATUS_STATIONARY;
    }
}

void imu_fusion_init(imu_fusion_t *fusion) {
    FusionAhrsSettings ahrs_settings;
    FusionBiasSettings bias_settings;

    if (fusion == NULL) {
        return;
    }

    memset(fusion, 0, sizeof(*fusion));
    fusion->alignment = (FusionRemapAlignment)IMU_FUSION_ALIGNMENT_INDEX;

    FusionAhrsInitialise(&fusion->ahrs);
    ahrs_settings = fusionAhrsDefaultSettings;
    ahrs_settings.sampleRate = IMU_FUSION_SAMPLE_RATE_HZ;
    ahrs_settings.gain = IMU_FUSION_AHRS_GAIN;
    ahrs_settings.gyroscopeRange = 500.0f;
    ahrs_settings.accelerationRejection = IMU_FUSION_ACCEL_REJECTION_DEG;
    ahrs_settings.rejectionTimeout = IMU_FUSION_REJECTION_TIMEOUT_S;
    FusionAhrsSetSettings(&fusion->ahrs, &ahrs_settings);

    FusionBiasInitialise(&fusion->bias);
    bias_settings = fusionBiasDefaultSettings;
    bias_settings.sampleRate = IMU_FUSION_SAMPLE_RATE_HZ;
    bias_settings.stationaryThreshold = IMU_FUSION_BIAS_STATIONARY_LIMIT_DPS;
    bias_settings.stationaryPeriod = IMU_FUSION_BIAS_STATIONARY_PERIOD_S;
    FusionBiasSetSettings(&fusion->bias, &bias_settings);
}

void imu_fusion_set_alignment(imu_fusion_t *fusion,
                              FusionRemapAlignment alignment) {
    if (fusion != NULL && fusion->alignment != alignment) {
        fusion->alignment = alignment;
        fusion->gyro_calibration_sum = FUSION_VECTOR_ZERO;
        fusion->calibration_previous_gyro = FUSION_VECTOR_ZERO;
        fusion->calibration_previous_accel = FUSION_VECTOR_ZERO;
        fusion->calibration_samples = 0U;
        fusion->calibrated = 0U;
        fusion->calibration_has_previous = 0U;
        fusion->last_timestamp_ms = 0U;
        fusion->has_timestamp = 0U;
        {
            const FusionBiasSettings bias_settings = fusion->bias.settings;
            FusionBiasInitialise(&fusion->bias);
            FusionBiasSetSettings(&fusion->bias, &bias_settings);
        }
        FusionAhrsRestart(&fusion->ahrs);
    }
}

uint8_t imu_fusion_update(imu_fusion_t *fusion,
                          const imu_sample_t *sample,
                          uint32_t timestamp_ms,
                          imu_fusion_output_t *output) {
    FusionVector accelerometer;
    FusionVector gyroscope;
    float period;
    uint16_t status_flags = 0U;

    if (fusion == NULL || sample == NULL) {
        if (output != NULL) {
            memset(output, 0, sizeof(*output));
            output->timestamp_ms = timestamp_ms;
            output->status_flags = IMU_FUSION_STATUS_INPUT_INVALID;
        }
        return 0U;
    }

    accelerometer = sample_accelerometer(sample, fusion);
    gyroscope = sample_gyroscope(sample, fusion);

    if (fusion->calibrated == 0U) {
        status_flags = IMU_FUSION_STATUS_CALIBRATING;
        const uint8_t calibration_stable =
            calibration_is_stable(fusion, gyroscope, accelerometer);
        if (calibration_stable != 0U) {
            fusion->gyro_calibration_sum =
                FusionVectorAdd(fusion->gyro_calibration_sum, gyroscope);
            fusion->calibration_samples++;
        } else {
            fusion->gyro_calibration_sum = FUSION_VECTOR_ZERO;
            fusion->calibration_samples = 0U;
        }

        if (fusion->calibration_samples >= IMU_FUSION_CALIBRATION_SAMPLES) {
            const float count = (float)fusion->calibration_samples;
            const FusionVector offset = FusionVectorScale(
                fusion->gyro_calibration_sum, 1.0f / count);
            FusionBiasSetOffset(&fusion->bias, offset);
            FusionAhrsRestart(&fusion->ahrs);
            fusion->calibrated = 1U;
            status_flags = IMU_FUSION_STATUS_CALIBRATED;
        } else {
            if (output != NULL) {
                memset(output, 0, sizeof(*output));
                output->timestamp_ms = timestamp_ms;
                output->status_flags = status_flags;
                output->calibration_samples =
                    (fusion->calibration_samples > UINT16_MAX)
                        ? UINT16_MAX
                        : (uint16_t)fusion->calibration_samples;
                output->gyro_bias_dps[0] =
                    fusion->calibration_samples == 0U
                        ? 0.0f
                        : fusion->gyro_calibration_sum.axis.x /
                              (float)fusion->calibration_samples;
                output->gyro_bias_dps[1] =
                    fusion->calibration_samples == 0U
                        ? 0.0f
                        : fusion->gyro_calibration_sum.axis.y /
                              (float)fusion->calibration_samples;
                output->gyro_bias_dps[2] =
                    fusion->calibration_samples == 0U
                        ? 0.0f
                        : fusion->gyro_calibration_sum.axis.z /
                              (float)fusion->calibration_samples;
                if (calibration_stable != 0U) {
                    output->status_flags |= IMU_FUSION_STATUS_STATIONARY;
                }
            }
            fusion->last_timestamp_ms = timestamp_ms;
            fusion->has_timestamp = 1U;
            return 1U;
        }
    }

    period = sample_period(fusion, timestamp_ms);
    FusionAhrsSetSamplePeriod(&fusion->ahrs, period);
    {
        FusionBiasSettings bias_settings = fusion->bias.settings;
        bias_settings.sampleRate = 1.0f / period;
        FusionBiasSetSettings(&fusion->bias, &bias_settings);
    }
    gyroscope = FusionBiasUpdate(&fusion->bias, gyroscope);
    FusionAhrsUpdateNoMagnetometer(&fusion->ahrs, gyroscope, accelerometer);

    {
        const FusionAhrsInternalStates internal =
            FusionAhrsGetInternalStates(&fusion->ahrs);
        const FusionAhrsFlags flags = FusionAhrsGetFlags(&fusion->ahrs);
        status_flags |= IMU_FUSION_STATUS_CALIBRATED |
                        IMU_FUSION_STATUS_ATTITUDE_VALID;
        if (internal.accelerometerIgnored) {
            status_flags |= IMU_FUSION_STATUS_ACCEL_IGNORED;
        }
        if (flags.startup) {
            status_flags |= IMU_FUSION_STATUS_STARTUP;
        }
    }

    write_output(fusion, gyroscope, accelerometer, timestamp_ms, status_flags,
                 output);
    fusion->last_timestamp_ms = timestamp_ms;
    fusion->has_timestamp = 1U;
    return 1U;
}

uint8_t imu_fusion_is_calibrated(const imu_fusion_t *fusion) {
    return (fusion != NULL && fusion->calibrated != 0U) ? 1U : 0U;
}
