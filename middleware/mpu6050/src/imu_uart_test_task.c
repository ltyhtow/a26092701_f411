#include "imu_uart_test_task.h"

#include "imu_config.h"
#include "imu_fusion.h"
#include "imu_task.h"
#include "serial_protocol.h"
#include "serial_transport.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define IMU_UART_TEST_STACK_DEPTH_WORDS 384U

static QueueHandle_t imu_uart_source_queue;
static QueueHandle_t imu_uart_attitude_queue;

#if IMU_UART_ASCII_OUTPUT
static void format_fixed_float(char *out, size_t max_len, float val) {
    const char *sign = (val < 0.0f) ? "-" : " ";
    float abs_val = (val < 0.0f) ? -val : val;
    int int_part = (int)abs_val;
    int frac_part = (int)((abs_val - (float)int_part) * 100.0f);
    if (frac_part >= 100) { frac_part = 99; }
    snprintf(out, max_len, "%s%3d.%02d", sign, int_part, frac_part);
}
#else
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
#endif

static void imu_uart_test_task_entry(void *argument) {
    imu_sample_message_t message;
    imu_fusion_output_t attitude;
#if !IMU_UART_ASCII_OUTPUT
    imu_telemetry_t telemetry;
    imu_diagnostic_t diagnostic;
    imu_attitude_telemetry_t attitude_telemetry;
    uint8_t sequence = 0U;
#endif

    (void)argument;
    for (;;) {
        if (xQueueReceive(imu_uart_source_queue, &message,
                          pdMS_TO_TICKS(IMU_UART_TEST_PERIOD_MS)) != pdTRUE) {
            memset(&message, 0, sizeof(message));
            message.timestamp_ms = (uint32_t)xTaskGetTickCount();
            message.error_code = 0xFFFFU;
        }

#if IMU_UART_ASCII_OUTPUT
        char line_buf[100];
        if (message.status_flags == 0U) {
            snprintf(line_buf, sizeof(line_buf),
                     "[MPU6050 WAIT] WHO_AM_I=0x%02X Init=%u Err=0x%lX\r\n",
                     message.diagnostics.chip_id, message.diagnostics.init_result,
                     (unsigned long)message.diagnostics.hal_error_codes);
            serial_transport_send_string(line_buf);
        } else if (xQueuePeek(imu_uart_attitude_queue, &attitude, 0U) == pdTRUE) {
            if ((attitude.status_flags & IMU_FUSION_STATUS_CALIBRATING) != 0U) {
                snprintf(line_buf, sizeof(line_buf),
                         "[MPU6050 CALIB] Samples: %3u / 400 (keep robot still)\r\n",
                         attitude.calibration_samples);
                serial_transport_send_string(line_buf);
            } else {
                char s_pitch[16], s_roll[16], s_yaw[16];
                format_fixed_float(s_pitch, sizeof(s_pitch), attitude.pitch_deg);
                format_fixed_float(s_roll, sizeof(s_roll), attitude.roll_deg);
                format_fixed_float(s_yaw, sizeof(s_yaw), attitude.yaw_deg);
                snprintf(line_buf, sizeof(line_buf),
                         "[IMU] Pitch:%s | Roll:%s | Yaw:%s (deg)\r\n",
                         s_pitch, s_roll, s_yaw);
                serial_transport_send_string(line_buf);
            }
        }
#else
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
#endif
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
