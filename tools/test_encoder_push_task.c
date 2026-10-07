/* Real encoder_push_test_task.c + encoder_driver.c; mock only RTOS, HAL IO,
 * the motor latch (covered by motor hardware tests), and packet submission.
 * Each scenario runs in a fresh process to preserve driver initialization rules.
 */
#include "encoder_push_test_task.h"
#include "encoder_driver.h"
#include "encoder_config.h"
#include "app_clock.h"
#include "motor_driver.h"
#include "serial_protocol.h"
#include "tim.h"
#include "main.h"

#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#define FRAME_COUNT 4U
TIM_HandleTypeDef htim2, htim5;
GPIO_TypeDef test_gpioa, test_gpiob;
static TaskFunction_t captured_entry;
static jmp_buf stop_task;
static bool motor_locked, create_failure, initialization_failure;
static unsigned lock_calls, frame_count, delay_count;
static TickType_t now_tick = 100U;
static encoder_test_telemetry_t frames[FRAME_COUNT];
static const uint32_t raw_left[FRAME_COUNT] = {12U, 20U, UINT32_MAX - 2U, 5U};
static const uint32_t raw_right[FRAME_COUNT] = {UINT32_MAX - 8U, UINT32_MAX - 12U, 3U, 10U};
static const int32_t native_delta_left[FRAME_COUNT] = {12, 8, -23, 8};
static const int32_t native_delta_right[FRAME_COUNT] = {-9, -4, 16, 7};
static const uint32_t input_levels[FRAME_COUNT] = {5U, 10U, 15U, 0U};

void motor_driver_emergency_stop(void) {
    motor_locked = true;
    ++lock_calls;
}

bool motor_driver_fault_latched(void) { return motor_locked; }

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channels) {
    assert(motor_locked && lock_calls == 1U); /* Motor lock must precede acquisition. */
    assert(channels == TIM_CHANNEL_ALL);
    ++timer->start_calls;
    if (timer->start_result == HAL_OK) timer->running = true;
    return timer->start_result;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *timer, uint32_t channels) {
    assert(channels == TIM_CHANNEL_ALL);
    ++timer->stop_calls;
    timer->running = false;
    return HAL_OK;
}

BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, unsigned stack,
                      void *argument, UBaseType_t priority, TaskHandle_t *handle) {
    assert(motor_locked && lock_calls == 1U);
    assert(htim2.start_calls == 1U);
    assert(strcmp(name, "EncoderPush") == 0 && stack >= 384U);
    assert(argument == NULL && priority == 1U);
    if (create_failure) return pdFAIL;
    captured_entry = entry;
    *handle = (void *)1;
    return pdPASS;
}

TickType_t xTaskGetTickCount(void) { return now_tick; }
uint32_t app_clock_now_ms(void) { return now_tick; }
TickType_t app_clock_period_ticks(uint32_t period_ms) { return period_ms; }

void vTaskDelayUntil(TickType_t *wake, TickType_t interval) {
    assert(interval == 50U); /* 20 Hz, not the closed-loop 200 Hz period. */
    if (frame_count == FRAME_COUNT) longjmp(stop_task, 1);
    ++delay_count;
    *wake += interval;
    now_tick = *wake;
    htim2.counter = raw_left[frame_count];
    htim5.counter = raw_right[frame_count];
    const uint32_t levels = input_levels[frame_count];
    GPIOA->IDR = 0x80000000U | ((levels & 1U) << 5U) |
                 ((levels >> 2U) & 1U) | (((levels >> 3U) & 1U) << 1U);
    GPIOB->IDR = 0x80000000U | (((levels >> 1U) & 1U) << 3U);
}

serial_protocol_result_t serial_protocol_try_send(uint32_t command, const void *data, size_t size) {
    assert(command == 0x87U && command == SERIAL_CMD_ENCODER_TEST);
    /* Submission is a native DTO; the real codec owns the 56-byte wire size. */
    assert(size == sizeof(encoder_test_telemetry_t));
    assert(motor_locked && frame_count < FRAME_COUNT);
    memcpy(&frames[frame_count], data, size);
    ++frame_count;
    return frame_count == 2U ? SERIAL_PROTOCOL_BUSY : SERIAL_PROTOCOL_OK;
}

static void verify_frames(void) {
    assert(frame_count == FRAME_COUNT && delay_count == FRAME_COUNT);
    int64_t total_left = 0, total_right = 0;
    for (unsigned i = 0U; i < FRAME_COUNT; ++i) {
        const encoder_test_telemetry_t *frame = &frames[i];
        assert(frame->version == SERIAL_PROTOCOL_VERSION && frame->sequence == i);
        assert(frame->timestamp_ms == 150U + i * 50U);
        assert(frame->sample_period_ms == 50U);
        assert(frame->gpio_levels == input_levels[i]);
        assert(frame->tx_dropped == (i < 2U ? 0U : 1U));
        const uint16_t fixed_flags = ENCODER_TEST_MOTOR_LOCKED |
            (ENCODER_INVERT_LEFT ? ENCODER_TEST_INVERT_LEFT : 0U) |
            (ENCODER_INVERT_RIGHT ? ENCODER_TEST_INVERT_RIGHT : 0U);
        if (initialization_failure) {
            assert(frame->flags == fixed_flags);
            assert(frame->sample_errors == i + 1U);
            assert(frame->raw_left == 0U && frame->raw_right == 0U);
            assert(frame->delta_left == 0 && frame->delta_right == 0);
            assert(frame->total_left == 0 && frame->total_right == 0);
        } else {
            const int32_t delta_left = ENCODER_INVERT_LEFT ? -native_delta_left[i] : native_delta_left[i];
            const int32_t delta_right = ENCODER_INVERT_RIGHT ? -native_delta_right[i] : native_delta_right[i];
            total_left += delta_left;
            total_right += delta_right;
            assert(frame->flags == (fixed_flags | ENCODER_TEST_READY | ENCODER_TEST_VALID));
            assert(frame->sample_errors == 0U);
            assert(frame->raw_left == raw_left[i] && frame->raw_right == raw_right[i]);
            assert(frame->delta_left == delta_left && frame->delta_right == delta_right);
            assert(frame->total_left == total_left && frame->total_right == total_right);
        }
    }
    if (initialization_failure) assert(!htim2.running && !htim5.running);
    else assert(htim2.running && htim5.running);
}

int main(int argc, char **argv) {
    assert(argc == 2);
    if (strcmp(argv[1], "fail_left") == 0) {
        htim2.start_result = HAL_ERROR;
        initialization_failure = true;
    } else if (strcmp(argv[1], "fail_right") == 0) {
        htim5.start_result = HAL_ERROR;
        initialization_failure = true;
    } else if (strcmp(argv[1], "create_failure") == 0) {
        create_failure = true;
    } else assert(strcmp(argv[1], "normal") == 0);
    assert(encoder_push_test_task_start(NULL) == pdFAIL);
    assert(lock_calls == 0U);
    TaskHandle_t task_handle = NULL;
    const BaseType_t result = encoder_push_test_task_start(&task_handle);
    assert(motor_locked && lock_calls == 1U);
    if (create_failure) {
        assert(result == pdFAIL && captured_entry == NULL && frame_count == 0U);
    } else {
        assert(result == pdPASS && captured_entry != NULL);
        if (setjmp(stop_task) == 0) captured_entry(NULL);
        verify_frames();
    }
    printf("PASS encoder push task: %s\n", argv[1]);
    return 0;
}
