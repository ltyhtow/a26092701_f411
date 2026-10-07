/* TIM2/TIM5 acquisition adapter. Counter arithmetic lives in encoder_counter. */
#include "encoder_driver.h"
#include "encoder_config.h"
#include "tim.h"

#include <math.h>
#include <stddef.h>

static bool s_encoder_inited;
static encoder_counter_t s_counter;

void encoder_driver_get_snapshot(encoder_driver_snapshot_t *snapshot) {
    encoder_counter_get_snapshot(&s_counter, snapshot);
}

void encoder_driver_get_polarity(bool *invert_left, bool *invert_right) {
    if (invert_left != NULL) *invert_left = ENCODER_INVERT_LEFT != 0U;
    if (invert_right != NULL) *invert_right = ENCODER_INVERT_RIGHT != 0U;
}

bool encoder_driver_init(void) {
    if (s_encoder_inited) return true;
    /* Baseline precedes starting counters, so startup pulses are retained. */
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim5, 0U);
    encoder_counter_init(&s_counter, ENCODER_INVERT_LEFT != 0U,
                         ENCODER_INVERT_RIGHT != 0U, 0U, 0U);
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL) != HAL_OK) {
        (void)HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
        (void)HAL_TIM_Encoder_Stop(&htim5, TIM_CHANNEL_ALL);
        return false;
    }
    s_encoder_inited = true;
    return true;
}

bool encoder_driver_is_ready(void) { return s_encoder_inited; }

void encoder_driver_read_speed(float *left_speed, float *right_speed) {
    encoder_counter_invalidate(&s_counter);
    if (s_encoder_inited) {
        (void)encoder_counter_sample(&s_counter, __HAL_TIM_GET_COUNTER(&htim2),
                                    __HAL_TIM_GET_COUNTER(&htim5));
    }
    const encoder_counter_snapshot_t *sample = &s_counter.snapshot;
    if (left_speed != NULL) *left_speed = sample->valid ? (float)sample->delta_left : NAN;
    if (right_speed != NULL) *right_speed = sample->valid ? (float)sample->delta_right : NAN;
}

void encoder_driver_get_totals(int64_t *left_total, int64_t *right_total) {
    if (left_total != NULL) *left_total = s_counter.snapshot.total_left;
    if (right_total != NULL) *right_total = s_counter.snapshot.total_right;
}

void encoder_driver_reset_totals(void) { encoder_counter_reset_totals(&s_counter); }
