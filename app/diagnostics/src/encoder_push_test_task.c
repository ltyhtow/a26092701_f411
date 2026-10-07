#include "encoder_push_test_task.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "serial_protocol.h"
#include "board_diagnostics.h"
#include "app_clock.h"
#include <string.h>

static void encoder_push_test_task(void *argument) {
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    uint32_t previous_ms = app_clock_now_ms();
    bool invert_left, invert_right;
    encoder_driver_get_polarity(&invert_left, &invert_right);
    uint32_t dropped = 0, errors = 0;
    uint8_t sequence = 0;
    for (;;) {
        vTaskDelayUntil(&wake, app_clock_period_ticks(ENCODER_PUSH_TEST_PERIOD_MS));
        uint32_t now = app_clock_now_ms();
        if (now - previous_ms > ENCODER_PUSH_TEST_PERIOD_MS) {
            /* Do not emit a burst of zero-length catch-up samples after a stall. */
            wake = xTaskGetTickCount();
        }
        float left, right;
        encoder_driver_read_speed(&left, &right);
        encoder_driver_snapshot_t sample;
        encoder_driver_get_snapshot(&sample);
        if (!sample.valid && errors != UINT32_MAX) ++errors;
        encoder_test_telemetry_t packet;
        memset(&packet, 0, sizeof(packet));
        packet.version = SERIAL_PROTOCOL_VERSION;
        packet.sequence = sequence++;
        packet.flags = (sample.valid ? ENCODER_TEST_VALID : 0U) |
                       (encoder_driver_is_ready() ? ENCODER_TEST_READY : 0U) |
                       (motor_driver_fault_latched() ? ENCODER_TEST_MOTOR_LOCKED : 0U) |
                       (invert_left ? ENCODER_TEST_INVERT_LEFT : 0U) |
                       (invert_right ? ENCODER_TEST_INVERT_RIGHT : 0U);
        packet.timestamp_ms = now;
        packet.sample_period_ms = now - previous_ms;
        previous_ms = now;
        packet.raw_left = sample.raw_left;
        packet.raw_right = sample.raw_right;
        packet.delta_left = sample.delta_left;
        packet.delta_right = sample.delta_right;
        packet.total_left = sample.total_left;
        packet.total_right = sample.total_right;
        /* Instantaneous input levels, not a measurement of every edge. */
        packet.gpio_levels = board_encoder_levels();
        packet.tx_dropped = dropped;
        packet.sample_errors = errors;
        /* The normal protocol API calls lwpkt_write -> LwRB -> UART DMA. */
        if (serial_protocol_try_send(SERIAL_CMD_ENCODER_TEST, &packet, sizeof(packet)) != SERIAL_PROTOCOL_OK &&
            dropped != UINT32_MAX) ++dropped;
    }
}

BaseType_t encoder_push_test_task_start(TaskHandle_t *task_handle) {
    if (task_handle == NULL) return pdFAIL;
    motor_driver_emergency_stop(); /* Latched: incoming Arm commands cannot power wheels. */
    (void)encoder_driver_init(); /* Keep reporting READY=0 if initialization fails. */
    return xTaskCreate(encoder_push_test_task, "EncoderPush", 384U, NULL, 1U, task_handle);
}
