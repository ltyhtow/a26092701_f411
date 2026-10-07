#include "board_diagnostics.h"
#include "motor_driver.h"
#include "motor_config.h"
#include "main.h"

#if SERIAL_TRANSPORT_USB_CDC
#define MOTOR_TEST_INPUTS (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)
#else
#define MOTOR_TEST_INPUTS (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11)
#endif

void board_motor_test_stop(bool direct_gpio) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (direct_gpio) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        HAL_GPIO_WritePin(GPIOA, MOTOR_TEST_INPUTS, GPIO_PIN_RESET);
#if SERIAL_TRANSPORT_USB_CDC
        __HAL_RCC_GPIOB_CLK_ENABLE();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
#endif
    } else {
        motor_driver_stop();
    }
    __set_PRIMASK(primask);
}

bool board_motor_test_start(bool direct_gpio) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (motor_driver_fault_latched()) {
        __set_PRIMASK(primask);
        return false;
    }
    if (direct_gpio) {
        GPIO_InitTypeDef gpio = {0};
        __HAL_RCC_GPIOA_CLK_ENABLE();
        HAL_GPIO_WritePin(GPIOA, MOTOR_TEST_INPUTS, GPIO_PIN_RESET);
        gpio.Pin = MOTOR_TEST_INPUTS;
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &gpio);
        /* Match positive PWM; USB keeps PA11/PA12 exclusively on AF10. */
#if SERIAL_TRANSPORT_USB_CDC
        __HAL_RCC_GPIOB_CLK_ENABLE();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        gpio.Pin = GPIO_PIN_5;
        HAL_GPIO_Init(GPIOB, &gpio);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
#else
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_11, GPIO_PIN_SET);
#endif
    } else {
        motor_driver_init();
        motor_driver_set_output(MOTOR_TEST_DUTY, MOTOR_TEST_DUTY);
    }
    const bool started = !motor_driver_fault_latched();
    __set_PRIMASK(primask);
    return started;
}
