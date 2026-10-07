/* Uses only public headers; no HAL, FreeRTOS, Fusion types or test substitutes. */
#include "imu_fusion.h"
#include "imu_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void near(float actual, float expected, float tolerance) {
    assert(isfinite(actual) && fabsf(actual - expected) <= tolerance);
}

static uint32_t calibrate(imu_fusion_t *fusion, const imu_sample_t *sample, uint32_t start,
                          imu_fusion_output_t *output) {
    for (uint32_t i = 1; i <= IMU_FUSION_CALIBRATION_SAMPLES; ++i) {
        const uint32_t now = start + i * IMU_SAMPLE_PERIOD_MS;
        assert(imu_fusion_update(fusion, sample, now, output));
        assert(output->timestamp_ms == now && output->calibration_samples == i);
        if (i < IMU_FUSION_CALIBRATION_SAMPLES) {
            assert(!imu_fusion_is_calibrated(fusion));
            assert((output->status_flags & IMU_FUSION_STATUS_CALIBRATING) != 0U);
            assert((output->status_flags & IMU_FUSION_STATUS_ATTITUDE_VALID) == 0U);
        }
    }
    assert(imu_fusion_is_calibrated(fusion));
    assert((output->status_flags & IMU_FUSION_STATUS_CALIBRATED) != 0U);
    assert((output->status_flags & IMU_FUSION_STATUS_ATTITUDE_VALID) != 0U);
    for (unsigned axis = 0; axis < 3; ++axis) {
        near(output->gyro_bias_dps[axis], sample->gyro_dps[axis], 0.0001f);
        near(output->gyro_dps[axis], 0, 0.0001f);
    }
    near(output->quaternion.w, 1.0f, 0.001f);
    near(output->quaternion.x, 0.0f, 0.001f);
    near(output->quaternion.y, 0.0f, 0.001f);
    near(output->quaternion.z, 0.0f, 0.001f);
    near(output->roll_deg, 0.0f, 0.001f);
    near(output->pitch_deg, 0.0f, 0.001f);
    return output->timestamp_ms;
}

static void reject_nonfinite(imu_fusion_t *fusion, const imu_sample_t *healthy,
                             uint32_t timestamp_ms) {
    const float invalid_values[] = {NAN, INFINITY, -INFINITY};
    /* Opaque storage was calloc/malloc allocated and fully initialized, so its
     * byte representation can verify the documented no-mutation contract
     * without exposing any estimator implementation types. */
    const size_t size = imu_fusion_context_size();
    void *before = malloc(size);
    assert(before != NULL);
    memcpy(before, fusion, size);
    for (unsigned field = 0; field < 2U; ++field) {
        for (unsigned axis = 0; axis < 3U; ++axis) {
            for (unsigned value = 0; value < 3U; ++value) {
                imu_sample_t invalid = *healthy;
                imu_fusion_output_t output;
                memset(&output, 0xA5, sizeof(output));
                if (field == 0U) invalid.accel_g[axis] = invalid_values[value];
                else invalid.gyro_dps[axis] = invalid_values[value];
                assert(!imu_fusion_update(fusion, &invalid, timestamp_ms, &output));
                assert(output.timestamp_ms == timestamp_ms);
                assert(output.status_flags == IMU_FUSION_STATUS_INPUT_INVALID);
                assert(output.calibration_samples == 0U);
                near(output.quaternion.w, 0, 0);
                near(output.gyro_dps[axis], 0, 0);
                assert(memcmp(before, fusion, size) == 0);
                assert(!imu_fusion_update(fusion, &invalid, timestamp_ms, NULL));
                assert(memcmp(before, fusion, size) == 0);
            }
        }
    }
    free(before);
}

int main(void) {
    assert(imu_fusion_context_size() > 0U);
    imu_fusion_t *fusion = malloc(imu_fusion_context_size());
    assert(fusion != NULL);
    imu_fusion_init(NULL);
    imu_fusion_set_alignment(NULL, 0);
    assert(!imu_fusion_is_calibrated(NULL));
    imu_fusion_init(fusion);
    assert(!imu_fusion_is_calibrated(fusion));
    imu_sample_t sample = {.accel_g = {0,0,1}, .gyro_dps = {1.25f,-0.5f,0.25f}};
    imu_fusion_output_t output;
    memset(&output, 0xA5, sizeof(output));
    assert(!imu_fusion_update(NULL, &sample, 10, &output));
    assert(output.timestamp_ms == 10 && output.status_flags == IMU_FUSION_STATUS_INPUT_INVALID);
    assert(!imu_fusion_update(fusion, NULL, 11, &output));
    assert(output.timestamp_ms == 11 && output.status_flags == IMU_FUSION_STATUS_INPUT_INVALID);
    assert(!imu_fusion_update(NULL, NULL, 0, NULL));
    reject_nonfinite(fusion, &sample, 1000U);
    assert(!imu_fusion_is_calibrated(fusion));

    uint32_t now = calibrate(fusion, &sample, UINT32_MAX - 1000U, &output);
    reject_nonfinite(fusion, &sample, now + IMU_SAMPLE_PERIOD_MS);
    assert(imu_fusion_is_calibrated(fusion));
    /* Public quaternion/Euler data remains usable after filter startup. */
    for (unsigned i = 0; i < 1000U; ++i) {
        now += IMU_SAMPLE_PERIOD_MS;
        assert(imu_fusion_update(fusion, &sample, now, &output));
    }
    assert((output.status_flags & IMU_FUSION_STATUS_STARTUP) == 0U);
    near(output.quaternion.w, 1, 0.001f);
    near(output.yaw_deg, 0, 0.001f);
    imu_fusion_set_alignment(fusion, 255); /* invalid alignment preserves state */
    assert(imu_fusion_is_calibrated(fusion));
    imu_fusion_set_alignment(fusion, IMU_FUSION_ALIGNMENT_INDEX);
    assert(imu_fusion_is_calibrated(fusion));
    imu_fusion_set_alignment(fusion, 1);
    assert(!imu_fusion_is_calibrated(fusion));
    imu_fusion_init(fusion); /* Restores configured alignment, bias and counters. */
    assert(!imu_fusion_is_calibrated(fusion));
    now = calibrate(fusion, &sample, 0, &output);
    assert(imu_fusion_update(fusion, &sample, now + IMU_SAMPLE_PERIOD_MS, NULL));

    imu_fusion_init(fusion);
    for (unsigned i = 1; i < 20; ++i) assert(imu_fusion_update(fusion, &sample, i * 5U, &output));
    imu_sample_t moving = sample; moving.accel_g[2] = 2.0f;
    assert(imu_fusion_update(fusion, &moving, 100, &output));
    assert(output.calibration_samples == 0 && !imu_fusion_is_calibrated(fusion));
    (void)calibrate(fusion, &sample, 100, &output);
    free(fusion);
    puts("PASS: opaque Fusion, 400-sample bias, quaternion, wrap, reset and nonfinite-input recovery");
    return 0;
}
