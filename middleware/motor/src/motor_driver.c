/**
 * @file motor_driver.c
 * @brief Implementation of AT8236 motor driver for STM32F411.
 */

#include "motor_driver.h"
#include "motor_config.h"
#include "tim.h"

#include <stdlib.h>

static bool s_driver_inited = false;

static inline uint32_t clamp_duty(uint32_t duty) {
    if (duty > MOTOR_PWM_MAX_DUTY) {
        return MOTOR_PWM_MAX_DUTY;
    }
    return duty;
}

void motor_driver_init(void) {
    if (s_driver_inited) {
        return;
    }

    /* 确保比较值全部清零 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);

    /* 启动 TIM1 四通道 PWM 输出 */
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* 高级定时器必须开启主输出使能 (MOE) */
    __HAL_TIM_MOE_ENABLE(&htim1);

    s_driver_inited = true;
}

void motor_driver_set_output(int16_t left_pwm, int16_t right_pwm) {
    if (!s_driver_inited) {
        motor_driver_init();
    }

    /* --------------------------------------------------------------------- */
    /* 1. 左轮 (A 路, 底盘重装后修正物理基准):                                    */
    /*    前进 (left_pwm > 0): AIN1(PA8)=PWM, AIN2(PA9)=0                        */
    /*    后退 (left_pwm < 0): AIN1(PA8)=0,   AIN2(PA9)=PWM                      */
    /* --------------------------------------------------------------------- */
    if (left_pwm > 0) {
        uint32_t duty = clamp_duty((uint32_t)left_pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);  /* PA8  = PWM */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);    /* PA9  = 0 */
    } else if (left_pwm < 0) {
        uint32_t duty = clamp_duty((uint32_t)(-left_pwm));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);    /* PA8  = 0 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);  /* PA9  = PWM */
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    }

    /* --------------------------------------------------------------------- */
    /* 2. 右轮 (B 路, 底盘重装后修正物理基准):                                    */
    /*    前进 (right_pwm > 0): BIN1(PA10)=0,   BIN2(PA11)=PWM                   */
    /*    后退 (right_pwm < 0): BIN1(PA10)=PWM, BIN2(PA11)=0                     */
    /* --------------------------------------------------------------------- */
    if (right_pwm > 0) {
        uint32_t duty = clamp_duty((uint32_t)right_pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);    /* PA10 = 0 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty);  /* PA11 = PWM */
    } else if (right_pwm < 0) {
        uint32_t duty = clamp_duty((uint32_t)(-right_pwm));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty);  /* PA10 = PWM */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);    /* PA11 = 0 */
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
    }
}

void motor_driver_stop(void) {
    if (s_driver_inited) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
    }
}

void motor_driver_brake(void) {
    /* AT8236 低端短路刹车: 两端同时拉高或拉低 */
    motor_driver_stop();
}

/**
 * @brief 强符号实现覆盖 balance_task.c 的弱符号钩子函数，实现与平衡算法解耦直连。
 */
void balance_motor_set_output(int16_t left_pwm, int16_t right_pwm) {
    motor_driver_set_output(left_pwm, right_pwm);
}
