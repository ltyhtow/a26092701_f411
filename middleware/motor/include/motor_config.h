#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* 1: 启用电机极性与直驱测试任务; 0: 正常运行控制任务 */
#ifndef MOTOR_POLARITY_TEST_ENABLED
#define MOTOR_POLARITY_TEST_ENABLED 0U
#endif

/* 1: 直接 GPIO 纯直流拉高 (不用 PWM); 0: 使用 PWM */
#ifndef MOTOR_DIRECT_GPIO_DRIVE
#define MOTOR_DIRECT_GPIO_DRIVE     0U
#endif

#define MOTOR_PWM_PERIOD            4999U /* TIM1 20 kHz 周期 (ARR=4999, 100MHz APB2) */
#define MOTOR_PWM_MAX_DUTY          4500U /* 最大安全占空比限制 (90%)，保留保护余量 */
#define MOTOR_DEADBAND_DUTY         250U  /* 静态摩擦死区补偿值 (~5%) */

#endif /* MOTOR_CONFIG_H */
