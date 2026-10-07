/* USB motor routing: compile the real timer setup, board diagnostics and driver.
 * The HAL model verifies requested channels, AF pins, clock/order and GPIO writes;
 * firmware build + on-board measurements remain necessary for electrical timing.
 */
#include "tim.h"
#include "motor_driver.h"
#include "motor_config.h"
#include "board_diagnostics.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

TIM_TypeDef test_tim1, test_tim2, test_tim3, test_tim5;
RCC_TypeDef test_rcc;
GPIO_TypeDef test_gpioa, test_gpiob;
uint32_t test_primask;
static unsigned pwm_starts, fail_start;
static uint32_t channels1, channels3;
#define USB_PINS (GPIO_PIN_11 | GPIO_PIN_12)
#define USB_MODES ((3UL << 22U) | (3UL << 24U))
#define USB_AFR_MASK ((15UL << 12U) | (15UL << 16U))

static void prepare_usb(void) {
    GPIOA->MODER = (2UL << 22U) | (2UL << 24U);
    GPIOA->AFR[1] = (10UL << 12U) | (10UL << 16U);
    GPIOA->OTYPER = USB_PINS;
    GPIOA->PUPDR = (1UL << 22U) | (2UL << 24U);
    GPIOA->OSPEEDR = USB_MODES;
    GPIOA->ODR = USB_PINS;
    TIM1->CCR4 = 1234U; /* Must remain unused by USB motor setup/stop. */
}

static void assert_usb_untouched(void) {
    assert((GPIOA->MODER & USB_MODES) == ((2UL << 22U) | (2UL << 24U)));
    assert((GPIOA->AFR[1] & USB_AFR_MASK) == ((10UL << 12U) | (10UL << 16U)));
    assert((GPIOA->OTYPER & USB_PINS) == USB_PINS);
    assert((GPIOA->PUPDR & USB_MODES) == ((1UL << 22U) | (2UL << 24U)));
    assert((GPIOA->OSPEEDR & USB_MODES) == USB_MODES);
    assert((GPIOA->ODR & USB_PINS) == USB_PINS);
    assert((GPIOA->BSRR & (USB_PINS | ((uint32_t)USB_PINS << 16U))) == 0U);
    assert(TIM1->CCR4 == 1234U);
}

void Error_Handler(void) { assert(!"Unexpected timer initialization failure"); abort(); }

HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer) {
    assert(timer->State == 0U); /* PWM owns the initial RESET -> READY transition. */
    assert(timer->Init.Prescaler == 0U && timer->Init.Period == 4999U);
    assert(timer->Instance == TIM1 || timer->Instance == TIM3);
    /* Model the vendor HAL invoking the board PWM MSP to enable its clock. */
    if (timer->Instance == TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    else RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    timer->Instance->PSC = timer->Init.Prescaler;
    timer->Instance->ARR = timer->Init.Period;
    timer->State = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *timer, TIM_ClockConfigTypeDef *config) {
    assert(timer->State == 1U && config->ClockSource == TIM_CLOCKSOURCE_INTERNAL);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *timer, TIM_MasterConfigTypeDef *config) {
    assert(timer->State == 1U && config->MasterSlaveMode == TIM_MASTERSLAVEMODE_DISABLE);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer, TIM_OC_InitTypeDef *config, uint32_t channel) {
    assert(config->Pulse == 0U && config->OCMode == TIM_OCMODE_PWM1);
    if (timer->Instance == TIM1) {
        assert(channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2 || channel == TIM_CHANNEL_3);
        channels1 |= 1UL << channel;
    } else {
        assert(timer->Instance == TIM3 && channel == TIM_CHANNEL_2);
        channels3 |= 1UL << channel;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *timer, TIM_BreakDeadTimeConfigTypeDef *config) {
    assert(timer->Instance == TIM1 && config->AutomaticOutput == TIM_AUTOMATICOUTPUT_DISABLE);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *timer, TIM_Encoder_InitTypeDef *config) {
    (void)timer; (void)config;
    assert(!"Motor test must not initialize encoder timers");
    return HAL_ERROR;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config) {
    if (port == GPIOA) {
        assert((config->Pin & ~0x0700U) == 0U);
        if (config->Mode == GPIO_MODE_AF_PP) assert(config->Alternate == 1U);
    } else {
        assert(port == GPIOB && config->Pin == GPIO_PIN_5);
        if (config->Mode == GPIO_MODE_AF_PP) assert(config->Alternate == 2U);
    }
    for (unsigned pin = 0; pin < 16U; ++pin) {
        if ((config->Pin & (1UL << pin)) == 0U) continue;
        port->MODER = (port->MODER & ~(3UL << (pin * 2U))) | (config->Mode << (pin * 2U));
        port->PUPDR = (port->PUPDR & ~(3UL << (pin * 2U))) | (config->Pull << (pin * 2U));
        port->OTYPER &= ~(1UL << pin);
        if (config->Mode == GPIO_MODE_AF_PP) {
            const unsigned shift = (pin % 8U) * 4U;
            port->AFR[pin / 8U] = (port->AFR[pin / 8U] & ~(15UL << shift)) | (config->Alternate << shift);
        }
    }
    assert_usb_untouched();
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, uint32_t value) {
    assert(test_primask == 1U);
    if (port == GPIOA) assert((pins & ~0x0700U) == 0U);
    else assert(port == GPIOB && pins == GPIO_PIN_5);
    if (value == GPIO_PIN_SET) {
        assert(!motor_driver_fault_latched());
        port->ODR |= pins;
    } else port->ODR &= ~(uint32_t)pins;
    port->BSRR = value == GPIO_PIN_SET ? pins : (uint32_t)pins << 16U;
    assert_usb_untouched();
}

void test_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value) {
    assert(test_primask == 1U);
    if (timer->Instance == TIM3) {
        assert(channel == TIM_CHANNEL_2);
        TIM3->CCR2 = value;
    } else {
        assert(timer->Instance == TIM1 && channel != TIM_CHANNEL_4);
        if (channel == TIM_CHANNEL_1) TIM1->CCR1 = value;
        else if (channel == TIM_CHANNEL_2) TIM1->CCR2 = value;
        else { assert(channel == TIM_CHANNEL_3); TIM1->CCR3 = value; }
    }
    assert_usb_untouched();
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    assert(test_primask == 1U && !motor_driver_fault_latched());
    assert(timer->State == 1U);
    ++pwm_starts;
    if (pwm_starts == fail_start) return HAL_ERROR;
    if (timer->Instance == TIM1) {
        assert(channel != TIM_CHANNEL_4 && (RCC->APB2ENR & RCC_APB2ENR_TIM1EN));
        TIM1->BDTR |= TIM_BDTR_MOE;
    } else assert(timer->Instance == TIM3 && channel == TIM_CHANNEL_2 &&
                  (RCC->APB1ENR & RCC_APB1ENR_TIM3EN));
    timer->Instance->CCER |= 1UL << channel;
    timer->Instance->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}

static void initialize_board(void) {
    MX_TIM1_Init();
    MX_TIM3_Init();
    assert(channels1 == 0x111U && channels3 == 0x10U);
    assert(96000000U / (TIM1->ARR + 1U) == 19200U);
    assert(TIM1->ARR == TIM3->ARR);
    assert((GPIOA->AFR[1] & 0xFFFU) == 0x111U); /* PA9/10 remain TIM1, not VBUS/ID. */
    assert(((GPIOB->AFR[0] >> 20U) & 15U) == 2U);
    assert_usb_untouched();
}

static void assert_zero_drive(void) {
    assert(TIM1->CCR1 == 0U && TIM1->CCR2 == 0U && TIM1->CCR3 == 0U && TIM3->CCR2 == 0U);
}
static void assert_fatal(void) {
    assert(motor_driver_fault_latched());
    assert((TIM1->BDTR & TIM_BDTR_MOE) == 0U && TIM1->CCER == 0U && TIM3->CCER == 0U);
    assert((TIM1->CR1 & TIM_CR1_CEN) == 0U && (TIM3->CR1 & TIM_CR1_CEN) == 0U);
    assert_zero_drive();
    assert((GPIOA->MODER & (0x3FUL << 16U)) == (0x15UL << 16U));
    assert((GPIOB->MODER & (3UL << 10U)) == (1UL << 10U));
    assert(GPIOA->BSRR == 0x07000000U && GPIOB->BSRR == 0x00200000U);
    assert_usb_untouched();
}

int main(int argc, char **argv) {
    assert(argc == 2);
    prepare_usb();
    if (strcmp(argv[1], "cold_fault") == 0) {
        motor_driver_emergency_stop();
        assert_fatal();
    } else {
        initialize_board();
        if (strcmp(argv[1], "normal") == 0) {
            motor_driver_set_output(1200, 2300);
            assert(pwm_starts == 4U);
            assert(TIM1->CCR1 == 1200U && TIM1->CCR2 == 0U && TIM1->CCR3 == 0U && TIM3->CCR2 == 2300U);
            motor_driver_set_output(INT16_MIN, INT16_MIN);
            assert(TIM1->CCR1 == 0U && TIM1->CCR2 == MOTOR_PWM_MAX_DUTY &&
                   TIM1->CCR3 == MOTOR_PWM_MAX_DUTY && TIM3->CCR2 == 0U);
            motor_driver_stop();
            assert_zero_drive();
            motor_driver_set_output(4501, 4501);
            assert(TIM1->CCR1 == MOTOR_PWM_MAX_DUTY && TIM3->CCR2 == MOTOR_PWM_MAX_DUTY);
            motor_driver_stop();
        } else if (strcmp(argv[1], "direct_gpio") == 0) {
            assert(board_motor_test_start(true));
            assert((GPIOA->ODR & 0x0700U) == GPIO_PIN_8 && (GPIOB->ODR & GPIO_PIN_5));
            board_motor_test_stop(true);
            assert((GPIOA->ODR & 0x0700U) == 0U && (GPIOB->ODR & GPIO_PIN_5) == 0U);
            motor_driver_emergency_stop();
            assert_fatal();
        } else {
            if (strncmp(argv[1], "start_failure_", 14U) == 0) {
                fail_start = (unsigned)atoi(argv[1] + 14U);
                assert(fail_start >= 1U && fail_start <= 4U);
            } else if (strcmp(argv[1], "missing_tim3_clock") == 0) RCC->APB1ENR = 0U;
            else assert(strcmp(argv[1], "running_fault") == 0);
            motor_driver_set_output(2500, 2700);
            if (fail_start == 0U && (RCC->APB1ENR & RCC_APB1ENR_TIM3EN)) {
                motor_driver_emergency_stop();
            }
            assert_fatal();
        }
    }
    if (motor_driver_fault_latched()) {
        const unsigned starts = pwm_starts;
        motor_driver_init();
        motor_driver_set_output(1500, 1500);
        assert(!board_motor_test_start(true) && !board_motor_test_start(false));
        assert(pwm_starts == starts);
        assert_fatal();
    }
    assert_usb_untouched();
    assert(test_primask == 0U);
    printf("PASS USB motor routing: %s\n", argv[1]);
    return 0;
}
