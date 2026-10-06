/**
 * @file encoder_driver.h
 * @brief Dual-channel quadrature encoder driver for STM32F411 balance robot.
 * @details Reads TIM2 (Left wheel: PA5/PB3) and TIM5 (Right wheel: PA0/PA1)
 *          in 32-bit 4x quadrature encoder mode (TIM_ENCODERMODE_TI12).
 */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 编码器极性反相配置 (推动车轮向前时，脉冲增量应为正数) */
#ifndef ENCODER_INVERT_LEFT
#define ENCODER_INVERT_LEFT     0U  /**< 1: 左轮反相; 0: 正常 */
#endif

#ifndef ENCODER_INVERT_RIGHT
#define ENCODER_INVERT_RIGHT    1U  /**< 1: 右轮反相; 0: 正常 (两轮镜像对称) */
#endif

/**
 * @brief Initialize and start TIM2 and TIM5 hardware encoder counters.
 */
void encoder_driver_init(void);

/**
 * @brief Read encoder speed delta feedback (called periodically by balance control task).
 * @param[out] left_speed  Speed feedback for left wheel (pulses per second or delta pulses).
 * @param[out] right_speed Speed feedback for right wheel (pulses per second or delta pulses).
 */
void encoder_driver_read_speed(float *left_speed, float *right_speed);

/**
 * @brief Get cumulative total encoder counts (for odometry / distance estimation).
 * @param[out] left_total  Cumulative pulse count of left wheel.
 * @param[out] right_total Cumulative pulse count of right wheel.
 */
void encoder_driver_get_totals(int64_t *left_total, int64_t *right_total);

/**
 * @brief Reset cumulative total encoder counts to zero.
 */
void encoder_driver_reset_totals(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_DRIVER_H */
