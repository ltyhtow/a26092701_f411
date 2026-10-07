/**
 * @file motor_driver.h
 * @brief Platform-independent dual motor actuator contract.
 * @details Board-specific polarity and PWM realization belong to the backend.
 *          Physical forward direction still requires bench verification.
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the board motor outputs at zero effort.
 */
void motor_driver_init(void);

/**
 * @brief Set dual motor drive efforts with polarity alignment and saturation clamping.
 * @param left_pwm Left wheel PWM effort (-4500 to +4500, positive = forward).
 * @param right_pwm Right wheel PWM effort (-4500 to +4500, positive = forward).
 */
void motor_driver_set_output(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief Set all compare values to zero (takes effect at the next PWM update).
 */
void motor_driver_stop(void);

/**
 * @brief Compatibility alias for zero drive; no separate braking mode.
 */
void motor_driver_brake(void);

/**
 * @brief Latch a fatal stop until reset and immediately disable motor outputs.
 * @details Safe before driver/timer initialization and in fault/ISR context.
 *          The backend must not allocate, wait, or depend on a running scheduler.
 */
void motor_driver_emergency_stop(void);

/** @brief True after a fatal stop or a PWM startup failure, until MCU reset. */
bool motor_driver_fault_latched(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
