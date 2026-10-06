#include "imu_uart_test_task.h"

#include "imu_fusion.h"
#include "imu_task.h"
#include "serial_protocol.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#define IMU_UART_TEST_STACK_DEPTH_WORDS 256U

static QueueHandle_t imu_uart_source_queue;
static QueueHandle_t imu_uart_attitude_queue;

static int32_t imu_float_to_q16_16(float value) {
    const float scaled = value * 65536.0f;
    if (scaled >= 2147483647.0f) {
        return INT32_MAX;
    }
    if (scaled <= -2147483648.0f) {
        return INT32_MIN;
    }
    return (int32_t)lroundf(scaled);
}

static int32_t imu_float_to_q30(float value) {
    const float scaled = value * 1073741824.0f;
    if (scaled >= 2147483647.0f) {
        return INT32_MAX;
    }
    if (scaled <= -2147483648.0f) {
        return INT32_MIN;
    }
    return (int32_t)lroundf(scaled);
}

static void fill_attitude_telemetry(const imu_fusion_output_t *source,
                                    uint8_t sequence,
                                    imu_attitude_telemetry_t *telemetry) {
    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->version = SERIAL_PROTOCOL_VERSION;
    telemetry->sequence = sequence;
    telemetry->status_flags = source->status_flags;
    telemetry->calibration_samples = source->calibration_samples;
    telemetry->roll_q16_16 = imu_float_to_q16_16(source->roll_deg);
    telemetry->pitch_q16_16 = imu_float_to_q16_16(source->pitch_deg);
    telemetry->yaw_q16_16 = imu_float_to_q16_16(source->yaw_deg);
    telemetry->gyro_bias_x_q16_16 = imu_float_to_q16_16(source->gyro_bias_dps[0]);
    telemetry->gyro_bias_y_q16_16 = imu_float_to_q16_16(source->gyro_bias_dps[1]);
    telemetry->gyro_bias_z_q16_16 = imu_float_to_q16_16(source->gyro_bias_dps[2]);
    telemetry->quaternion_w_q30 = imu_float_to_q30(source->quaternion.element.w);
    telemetry->quaternion_x_q30 = imu_float_to_q30(source->quaternion.element.x);
    telemetry->quaternion_y_q30 = imu_float_to_q30(source->quaternion.element.y);
    telemetry->quaternion_z_q30 = imu_float_to_q30(source->quaternion.element.z);
    telemetry->timestamp_ms = source->timestamp_ms;
}

static void imu_uart_test_task_entry(void *argument) {
    imu_sample_message_t message;
    imu_telemetry_t telemetry;
    imu_diagnostic_t diagnostic;
    imu_fusion_output_t attitude;
    imu_attitude_telemetry_t attitude_telemetry;
    uint8_t sequence = 0U;

    (void)argument;
    for (;;) {
        if (xQueueReceive(imu_uart_source_queue, &message,
                          pdMS_TO_TICKS(IMU_UART_TEST_PERIOD_MS)) != pdTRUE) {
            memset(&message, 0, sizeof(message));
            message.timestamp_ms = (uint32_t)xTaskGetTickCount();
            message.error_code = 0xFFFFU;
        }

        memset(&telemetry, 0, sizeof(telemetry));
        telemetry.version = SERIAL_PROTOCOL_VERSION;
        telemetry.sequence = sequence++;
        telemetry.status_flags = message.status_flags;
        memcpy(telemetry.accel_raw, message.sample.accel_raw,
               sizeof(telemetry.accel_raw));
        memcpy(telemetry.gyro_raw, message.sample.gyro_raw,
               sizeof(telemetry.gyro_raw));
        telemetry.error_code = message.error_code;
        telemetry.timestamp_ms = message.timestamp_ms;
        if (message.status_flags == 0U && message.error_code != 0xFFFFU) {
            memset(&diagnostic, 0, sizeof(diagnostic));
            diagnostic.version = SERIAL_PROTOCOL_VERSION;
            diagnostic.sequence = sequence;
            diagnostic.address_8bit = message.diagnostics.address;
            diagnostic.who_am_i = message.diagnostics.chip_id;
            diagnostic.init_result = message.diagnostics.init_result;
            diagnostic.hal_status = message.diagnostics.hal_status;
            diagnostic.hal_error_codes = message.diagnostics.hal_error_codes;
            diagnostic.timestamp_ms = message.timestamp_ms;
            (void)serial_protocol_send(SERIAL_CMD_IMU_DIAGNOSTIC,
                                        &diagnostic, sizeof(diagnostic));
        }
        (void)serial_protocol_send(SERIAL_CMD_IMU_STATUS,
                                   &telemetry, sizeof(telemetry));
        if (xQueuePeek(imu_uart_attitude_queue, &attitude, 0U) == pdTRUE) {
            fill_attitude_telemetry(&attitude, sequence++, &attitude_telemetry);
            (void)serial_protocol_send(SERIAL_CMD_IMU_ATTITUDE,
                                        &attitude_telemetry,
                                        sizeof(attitude_telemetry));
        }
    }
}

BaseType_t imu_uart_test_task_start(QueueHandle_t sample_queue,
                                    QueueHandle_t attitude_queue,
                                    TaskHandle_t *task_handle) {
    if (sample_queue == NULL || attitude_queue == NULL || task_handle == NULL) {
        return pdFAIL;
    }
    imu_uart_source_queue = sample_queue;
    imu_uart_attitude_queue = attitude_queue;
    return xTaskCreate(imu_uart_test_task_entry, "IMUTx",
                       IMU_UART_TEST_STACK_DEPTH_WORDS, NULL, 1U, task_handle);
}
