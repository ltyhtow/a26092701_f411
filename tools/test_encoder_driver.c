/** Host tests for acquisition failure, direction, modulo counts and reset semantics. */
#include "encoder_driver.h"
#include "encoder_config.h"
#include "tim.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channels) {
    assert(channels == TIM_CHANNEL_ALL);
    ++timer->start_calls;
    if (timer->start_result == HAL_OK) {
        timer->running = true;
    }
    return timer->start_result;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *timer, uint32_t channels) {
    assert(channels == TIM_CHANNEL_ALL);
    ++timer->stop_calls;
    timer->running = false;
    return HAL_OK;
}

static float left_feedback(float counts) {
    return ENCODER_INVERT_LEFT ? -counts : counts;
}

static float right_feedback(float counts) {
    return ENCODER_INVERT_RIGHT ? -counts : counts;
}

static void test_start_failure_and_retry(void) {
    float left = 0.0f;
    float right = 0.0f;
    assert(!encoder_driver_is_ready());
    encoder_driver_read_speed(&left, &right);
    assert(isnan(left) && isnan(right));
    htim2.start_result = HAL_ERROR;
    assert(!encoder_driver_init());
    assert(!encoder_driver_is_ready());
    assert(!htim2.running && !htim5.running);
    assert(htim2.stop_calls == 1 && htim5.stop_calls == 1);

    htim2.start_result = HAL_OK;
    htim5.start_result = HAL_ERROR;
    assert(!encoder_driver_init());
    assert(!htim2.running && !htim5.running);
    assert(htim2.stop_calls == 2 && htim5.stop_calls == 2);
    encoder_driver_read_speed(&left, &right);
    assert(isnan(left) && isnan(right));

    htim5.start_result = HAL_OK;
    assert(encoder_driver_init());
    assert(encoder_driver_is_ready() && htim2.running && htim5.running);
    htim2.counter = 10U;
    htim5.counter = 10U;
    assert(encoder_driver_init()); /* idempotent init cannot erase acquired pulses */
    encoder_driver_read_speed(&left, &right);
    assert(left == left_feedback(10.0f) && right == right_feedback(10.0f));
    puts("[PASS] encoder startup failure, retry and idempotence");
}

static void test_counts_polarity_and_wrap(void) {
    float left;
    float right;
    int64_t total_left;
    int64_t total_right;
    htim2.counter = UINT32_MAX - 1U; /* 10 -> -2, difference -12 */
    htim5.counter = UINT32_MAX - 1U;
    encoder_driver_read_speed(&left, &right);
    assert(left == left_feedback(-12.0f) && right == right_feedback(-12.0f));
    htim2.counter = 3U; /* -2 -> +3 crosses wrap, difference +5 */
    htim5.counter = 3U;
    encoder_driver_read_speed(&left, &right);
    assert(left == left_feedback(5.0f) && right == right_feedback(5.0f));
    encoder_driver_get_totals(&total_left, &total_right);
    assert(total_left == (int64_t)left_feedback(3.0f));
    assert(total_right == (int64_t)right_feedback(3.0f));
    encoder_driver_read_speed(&left, &right);
    assert(left == 0.0f && right == 0.0f);
    puts("[PASS] encoder signed X4 counts/sample, polarity and counter wrap");
}

static void test_totals_reset_keeps_speed_baseline(void) {
    float left;
    float right;
    int64_t total_left;
    int64_t total_right;
    encoder_driver_reset_totals();
    encoder_driver_get_totals(&total_left, &total_right);
    assert(total_left == 0 && total_right == 0);
    htim2.counter = 8U;
    htim5.counter = 8U;
    encoder_driver_read_speed(&left, &right);
    assert(left == left_feedback(5.0f) && right == right_feedback(5.0f));
    encoder_driver_get_totals(&total_left, &total_right);
    assert(total_left == (int64_t)left_feedback(5.0f));
    assert(total_right == (int64_t)right_feedback(5.0f));
    puts("[PASS] odometry reset preserves speed baseline");
}

static void test_ambiguous_delta_fails_closed(void) {
    float left;
    float right;
    int64_t total_left;
    int64_t total_right;
    htim2.counter += UINT32_C(0x80000000);
    encoder_driver_read_speed(&left, &right);
    assert(isnan(left) && isnan(right));
    encoder_driver_get_totals(&total_left, &total_right);
    assert(total_left == (int64_t)left_feedback(5.0f));
    assert(total_right == (int64_t)right_feedback(5.0f));
    htim5.counter += UINT32_C(0x80000000);
    encoder_driver_read_speed(&left, &right);
    assert(isnan(left) && isnan(right));
    encoder_driver_read_speed(&left, &right);
    assert(left == 0.0f && right == 0.0f);
    encoder_driver_read_speed(NULL, NULL);
    encoder_driver_get_totals(NULL, NULL);
    puts("[PASS] ambiguous half-range deltas are invalid feedback");
}

int main(void) {
    test_start_failure_and_retry();
    test_counts_polarity_and_wrap();
    test_totals_reset_keeps_speed_baseline();
    test_ambiguous_delta_fails_closed();
    return 0;
}
