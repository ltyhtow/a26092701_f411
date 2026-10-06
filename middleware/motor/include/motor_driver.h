/**
 * @file motor_driver.h
 * @brief AT8236 dual H-bridge motor driver for STM32F411 balance robot.
 * @details Drives 4-channel 20 kHz PWM via TIM1:
 *          - Left Motor (A-channel, polarity inverted):
 *              Forward:  AIN1(PA8)=0,   AIN2(PA9)=PWM
 *              Backward: AIN1(PA8)=PWM, AIN2(PA9)=0
 *          - Right Motor (B-channel, normal polarity):
 *              Forward:  BIN1(PA10)=PWM, BIN2(PA11)=0
 *              Backward: BIN1(PA10)=0,   BIN2(PA11)=PWM
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize TIM1 4-channel PWM channels and enable MOE main output.
 */
void motor_driver_init(void);

/**
 * @brief Set dual motor drive efforts with polarity alignment and saturation clamping.
 * @param left_pwm Left wheel PWM effort (-4500 to +4500, positive = forward).
 * @param right_pwm Right wheel PWM effort (-4500 to +4500, positive = forward).
 */
void motor_driver_set_output(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief Cut off all PWM outputs immediately (coast/freewheel, PWM=0).
 */
void motor_driver_stop(void);

/**
 * @brief Active braking (both low-side drivers on, shorting motor terminals).
 */
void motor_driver_brake(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
