#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* 1: 启用电机极性与直驱测试任务; 0: 正常运行 */
#ifndef MOTOR_POLARITY_TEST_ENABLED
#define MOTOR_POLARITY_TEST_ENABLED 1U
#endif

/* 1: 直接 GPIO 纯直流拉高 (不用 PWM, 方便万用表与方向测量); 0: 使用 PWM */
#ifndef MOTOR_DIRECT_GPIO_DRIVE
#define MOTOR_DIRECT_GPIO_DRIVE     1U
#endif

#define MOTOR_TEST_DUTY             2500U /* 50% PWM (仅在 MOTOR_DIRECT_GPIO_DRIVE=0 时生效) */

#endif /* MOTOR_CONFIG_H */
