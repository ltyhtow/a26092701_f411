/**
 * @file motor_driver_stm32.c
 * @brief Implementation of AT8236 motor driver for STM32F411.
 */

#include "motor_driver.h"
#include "motor_config.h"
#include "tim.h"

static bool s_driver_inited = false;
static volatile bool s_fault_latched = false;

/* Keep USB D-/D+ (PA11/PA12) untouched when the bridge IN2 is on PB5. */
#if SERIAL_TRANSPORT_USB_CDC
#define MOTOR_INPUT_PINS (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)
#define MOTOR_INPUT_MODE_MASK (0x3FUL << 16U)
#define MOTOR_INPUT_OUTPUT_MODE (0x15UL << 16U)
#define RIGHT_IN2_TIMER (&htim3)
#define RIGHT_IN2_CHANNEL TIM_CHANNEL_2
#else
#define MOTOR_INPUT_PINS (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11)
#define MOTOR_INPUT_MODE_MASK (0xFFUL << 16U)
#define MOTOR_INPUT_OUTPUT_MODE (0x55UL << 16U)
#define RIGHT_IN2_TIMER (&htim1)
#define RIGHT_IN2_CHANNEL TIM_CHANNEL_4
#endif

void motor_driver_emergency_stop(void) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_fault_latched = true;

    /* Avoid dereferencing htim1: a fault can occur before MX_TIM1_Init. */
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM1->CCER = 0U;
        TIM1->CR1 &= ~TIM_CR1_CEN;
        TIM1->CCR1 = 0U;
        TIM1->CCR2 = 0U;
        TIM1->CCR3 = 0U;
#if !SERIAL_TRANSPORT_USB_CDC
        TIM1->CCR4 = 0U;
#endif
    }
#if SERIAL_TRANSPORT_USB_CDC
    if ((RCC->APB1ENR & RCC_APB1ENR_TIM3EN) != 0U) {
        TIM3->CCER = 0U;
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM3->CCR2 = 0U;
    }
#endif

    /* Disabling an AF output alone can leave the bridge input floating.
       Preload ODR low, then take over all four pins as push-pull outputs.
       This also cuts output in the optional direct-GPIO test mode. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIOA->BSRR = (uint32_t)MOTOR_INPUT_PINS << 16U;
    GPIOA->OTYPER &= ~(uint32_t)MOTOR_INPUT_PINS;
    GPIOA->PUPDR &= ~MOTOR_INPUT_MODE_MASK;
    GPIOA->MODER = (GPIOA->MODER & ~MOTOR_INPUT_MODE_MASK) | MOTOR_INPUT_OUTPUT_MODE;
#if SERIAL_TRANSPORT_USB_CDC
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIOB->BSRR = (uint32_t)GPIO_PIN_5 << 16U;
    GPIOB->OTYPER &= ~(uint32_t)GPIO_PIN_5;
    GPIOB->PUPDR &= ~(3UL << 10U);
    GPIOB->MODER = (GPIOB->MODER & ~(3UL << 10U)) | (1UL << 10U);
#endif
    __DSB();
    __set_PRIMASK(primask);
}

bool motor_driver_fault_latched(void) {
    return s_fault_latched;
}

static inline uint32_t clamp_duty(uint32_t duty) {
    if (duty > MOTOR_PWM_MAX_DUTY) {
        return MOTOR_PWM_MAX_DUTY;
    }
    return duty;
}

void motor_driver_init(void) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_driver_inited || s_fault_latched) {
        __set_PRIMASK(primask);
        return;
    }

    if (htim1.Instance != TIM1 || (RCC->APB2ENR & RCC_APB2ENR_TIM1EN) == 0U
#if SERIAL_TRANSPORT_USB_CDC
        || htim3.Instance != TIM3 || (RCC->APB1ENR & RCC_APB1ENR_TIM3EN) == 0U
#endif
    ) {
        motor_driver_emergency_stop();
        __set_PRIMASK(primask);
        return;
    }

    /* 确保比较值全部清零 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(RIGHT_IN2_TIMER, RIGHT_IN2_CHANNEL, 0U);

    /* TIM1 CH1-3 plus TIM1 CH4 (UART) or TIM3 CH2 (USB). */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(RIGHT_IN2_TIMER, RIGHT_IN2_CHANNEL) != HAL_OK) {
        motor_driver_emergency_stop();
        __set_PRIMASK(primask);
        return;
    }

    s_driver_inited = true;
    __set_PRIMASK(primask);
}

void motor_driver_set_output(int16_t left_pwm, int16_t right_pwm) {
    /* Serialize with an ISR stop; never resume PWM after a fatal stop. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_fault_latched) {
        __set_PRIMASK(primask);
        return;
    }
    if (!s_driver_inited) {
        motor_driver_init();
    }
    if (!s_driver_inited || s_fault_latched) {
        __set_PRIMASK(primask);
        return;
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
    /*    BIN2 is PA11 in UART builds, PB5 in USB builds.                      */
    /* --------------------------------------------------------------------- */
    if (right_pwm > 0) {
        uint32_t duty = clamp_duty((uint32_t)right_pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);    /* PA10 = 0 */
        __HAL_TIM_SET_COMPARE(RIGHT_IN2_TIMER, RIGHT_IN2_CHANNEL, duty);
    } else if (right_pwm < 0) {
        uint32_t duty = clamp_duty((uint32_t)(-right_pwm));
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty);  /* PA10 = PWM */
        __HAL_TIM_SET_COMPARE(RIGHT_IN2_TIMER, RIGHT_IN2_CHANNEL, 0U);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
        __HAL_TIM_SET_COMPARE(RIGHT_IN2_TIMER, RIGHT_IN2_CHANNEL, 0U);
    }
    __set_PRIMASK(primask);
}

void motor_driver_stop(void) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (htim1.Instance == TIM1 && (RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
#if !SERIAL_TRANSPORT_USB_CDC
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
#endif
    }
#if SERIAL_TRANSPORT_USB_CDC
    if (htim3.Instance == TIM3 && (RCC->APB1ENR & RCC_APB1ENR_TIM3EN) != 0U) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    }
#endif
    __set_PRIMASK(primask);
}

void motor_driver_brake(void) {
    /* Existing API uses the same zero-drive behavior as stop(). */
    motor_driver_stop();
}
