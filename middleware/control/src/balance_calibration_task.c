/**
 * @file balance_calibration_task.c
 * @brief Standalone safe calibration task to measure:
 *        1. IMU pitch polarity (leaning forward should increase pitch)
 *        2. Mechanical equilibrium zero point theta_0
 *        3. Encoder pulse counting direction (pushing forward must be positive)
 *
 * Motors remain 100% disabled during calibration.
 */

#include "balance_task.h"
#include "encoder_driver.h"
#include "imu_fusion.h"
#include "serial_transport.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

static void format_float(char *buf, size_t len, float val) {
    const char *sign = (val < 0.0f) ? "-" : "+";
    float abs_val = (val < 0.0f) ? -val : val;
    int int_part = (int)abs_val;
    int frac_part = (int)((abs_val - (float)int_part) * 100.0f);
    if (frac_part >= 100) { frac_part = 99; }
    snprintf(buf, len, "%s%2d.%02d", sign, int_part, frac_part);
}

void balance_calibration_task_entry(void *argument) {
    QueueHandle_t attitude_q = (QueueHandle_t)argument;
    imu_fusion_output_t attitude;
    char text[128];
    char s_pitch[16], s_roll[16];

    encoder_driver_init();
    encoder_driver_reset_totals();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100U)); /* 10 Hz 采样打印 */

        if (attitude_q != NULL && xQueuePeek(attitude_q, &attitude, 0U) == pdTRUE) {
            format_float(s_pitch, sizeof(s_pitch), attitude.pitch_deg);
            format_float(s_roll, sizeof(s_roll), attitude.roll_deg);

            int64_t l_tot = 0, r_tot = 0;
            encoder_driver_get_totals(&l_tot, &r_tot);

            /* 读取瞬时单周期增量 */
            float l_spd = 0.0f, r_spd = 0.0f;
            encoder_driver_read_speed(&l_spd, &r_spd);

            snprintf(text, sizeof(text),
                     "[CALIB] Pitch:%s deg | Roll:%s deg | L_Enc:%+6d | R_Enc:%+6d\r\n",
                     s_pitch, s_roll, (int)l_tot, (int)r_tot);

            serial_transport_send_string(text);
        }
    }
}
