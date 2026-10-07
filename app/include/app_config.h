#ifndef APP_CONFIG_H
#define APP_CONFIG_H
#ifndef WHEEL_TEST_ENABLED
#define WHEEL_TEST_ENABLED 0
#endif
#ifndef ENCODER_PUSH_TEST_ENABLED
#define ENCODER_PUSH_TEST_ENABLED 0
#endif
#ifndef MOTOR_POLARITY_TEST_ENABLED
#define MOTOR_POLARITY_TEST_ENABLED 0
#endif
#ifndef MOTOR_DIRECT_GPIO_DRIVE
#define MOTOR_DIRECT_GPIO_DRIVE 0
#endif
#ifndef IMU_UART_TEST_ENABLED
#define IMU_UART_TEST_ENABLED 0
#endif
#if ENCODER_PUSH_TEST_ENABLED && (MOTOR_POLARITY_TEST_ENABLED || IMU_UART_TEST_ENABLED)
#error "Encoder push test is exclusive of other diagnostic modes"
#endif
#if WHEEL_TEST_ENABLED && (ENCODER_PUSH_TEST_ENABLED || MOTOR_POLARITY_TEST_ENABLED || IMU_UART_TEST_ENABLED)
#error "WheelTest is exclusive of other diagnostic modes"
#endif
#endif
