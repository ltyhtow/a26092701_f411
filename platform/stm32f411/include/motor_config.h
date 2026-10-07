#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#define MOTOR_PWM_PERIOD            4999U /* UART 100 MHz: 20 kHz; USB 96 MHz: 19.2 kHz */
#define MOTOR_PWM_MAX_DUTY          4500U /* 最大安全占空比限制 (90%)，保留保护余量 */
#define MOTOR_DEADBAND_DUTY         200U  /* 保留当前控制默认值，须实测标定 (~4%) */
#define MOTOR_TEST_DUTY             2500U /* 测试占空比 (50% PWM) */

#endif /* MOTOR_CONFIG_H */
