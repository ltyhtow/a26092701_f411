/**
 * @file encoder_driver.c
 * @brief Implementation of dual-channel quadrature encoder driver.
 */

#include "encoder_driver.h"
#include "tim.h"

static bool s_encoder_inited = false;
static uint32_t s_last_cnt_left = 0U;
static uint32_t s_last_cnt_right = 0U;

static int64_t s_total_cnt_left = 0;
static int64_t s_total_cnt_right = 0;

void encoder_driver_init(void) {
    if (s_encoder_inited) {
        return;
    }

    /* 启动 TIM2 (左轮 PA5/PB3) 和 TIM5 (右轮 PA0/PA1) 的正交编码器接口 */
    (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    /* 计数器初始清零 */
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim5, 0U);

    s_last_cnt_left = 0U;
    s_last_cnt_right = 0U;
    s_total_cnt_left = 0;
    s_total_cnt_right = 0;

    s_encoder_inited = true;
}

void encoder_driver_read_speed(float *left_speed, float *right_speed) {
    if (!s_encoder_inited) {
        encoder_driver_init();
    }

    /* 读取当前硬件 32 位计数器值 (TIM2 & TIM5 均为 32 位定时器) */
    uint32_t curr_left = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t curr_right = __HAL_TIM_GET_COUNTER(&htim5);

    /* 32 位无符号整数差值自动处理回绕溢出 (Unsigned wrap-around) */
    int32_t delta_left = (int32_t)(curr_left - s_last_cnt_left);
    int32_t delta_right = (int32_t)(curr_right - s_last_cnt_right);

    s_last_cnt_left = curr_left;
    s_last_cnt_right = curr_right;

    /* 极性修正：推动小车向前时，左右轮增量均应为正 */
#if ENCODER_INVERT_LEFT
    delta_left = -delta_left;
#endif

#if ENCODER_INVERT_RIGHT
    delta_right = -delta_right;
#endif

    /* 更新累计总里程脉冲 */
    s_total_cnt_left += (int64_t)delta_left;
    s_total_cnt_right += (int64_t)delta_right;

    /* 返回给平衡控制器的速度反馈 (单位: 脉冲数/控制周期，在 200Hz 下采样) */
    if (left_speed != NULL) {
        *left_speed = (float)delta_left;
    }
    if (right_speed != NULL) {
        *right_speed = (float)delta_right;
    }
}

void encoder_driver_get_totals(int64_t *left_total, int64_t *right_total) {
    if (left_total != NULL) {
        *left_total = s_total_cnt_left;
    }
    if (right_total != NULL) {
        *right_total = s_total_cnt_right;
    }
}

void encoder_driver_reset_totals(void) {
    s_total_cnt_left = 0;
    s_total_cnt_right = 0;
}
