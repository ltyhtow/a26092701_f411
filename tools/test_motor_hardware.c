/* Host regression tests: link with the production motor_driver.c.
 * Each named scenario runs in a fresh process (the fatal latch has no reset API).
 * The register model checks software requests, not pin waveforms or mechanics.
 */
#include "tim.h"
#include "motor_driver.h"
#include "motor_config.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

TIM_TypeDef test_tim1;
RCC_TypeDef test_rcc;
GPIO_TypeDef test_gpioa;
TIM_HandleTypeDef htim1;
uint32_t test_primask;
static unsigned start_calls;
static unsigned fail_start_call;

void test_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value) {
    assert(test_primask == 1U);
    assert(timer->Instance == TIM1);
    switch (channel) {
        case TIM_CHANNEL_1: timer->Instance->CCR1 = value; break;
        case TIM_CHANNEL_2: timer->Instance->CCR2 = value; break;
        case TIM_CHANNEL_3: timer->Instance->CCR3 = value; break;
        case TIM_CHANNEL_4: timer->Instance->CCR4 = value; break;
        default: assert(0);
    }
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    assert(test_primask == 1U);
    assert(timer->Instance == TIM1);
    assert((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U);
    ++start_calls;
    if (start_calls == fail_start_call) {
        return HAL_ERROR;
    }
    timer->Instance->CCER |= 1UL << channel;
    timer->Instance->BDTR |= TIM_BDTR_MOE;
    timer->Instance->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}

static void prepare_timer(void) {
    htim1.Instance = TIM1;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    GPIOA->MODER = 0xA5AAAA5AU;
    GPIOA->PUPDR = 0xAAFFFFFFU;
    GPIOA->OTYPER = 0xFFFFU;
}

static void assert_zero_compare(void) {
    assert(TIM1->CCR1 == 0U && TIM1->CCR2 == 0U);
    assert(TIM1->CCR3 == 0U && TIM1->CCR4 == 0U);
}

static void assert_fatal_stop(void) {
    assert(motor_driver_fault_latched());
    assert((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    assert((TIM1->CR1 & TIM_CR1_CEN) == 0U);
    assert(TIM1->CCER == 0U);
    assert_zero_compare();
    assert((RCC->AHB1ENR & 1U) != 0U);
    assert(GPIOA->BSRR == 0x0F000000U);
    assert((GPIOA->MODER & 0x00FF0000U) == 0x00550000U);
    assert((GPIOA->OTYPER & 0x0F00U) == 0U);
    assert((GPIOA->PUPDR & 0x00FF0000U) == 0U);
}

static void assert_restart_refused(void) {
    const unsigned calls_before = start_calls;
    motor_driver_init();
    motor_driver_set_output(1000, -2000);
    motor_driver_stop();
    motor_driver_brake();
    assert(start_calls == calls_before);
    assert_fatal_stop();
}

int main(int argc, char **argv) {
    assert(argc == 2);
    assert(!motor_driver_fault_latched());
    if (strcmp(argv[1], "normal") == 0) {
        prepare_timer();
        motor_driver_set_output(1200, -2300);
        assert(start_calls == 4U && !motor_driver_fault_latched());
        assert(TIM1->CCR1 == 1200U && TIM1->CCR2 == 0U);
        assert(TIM1->CCR3 == 2300U && TIM1->CCR4 == 0U);
        motor_driver_set_output(INT16_MIN, INT16_MAX);
        assert(TIM1->CCR1 == 0U && TIM1->CCR2 == MOTOR_PWM_MAX_DUTY);
        assert(TIM1->CCR3 == 0U && TIM1->CCR4 == MOTOR_PWM_MAX_DUTY);
        test_primask = 1U;
        motor_driver_stop();
        assert(test_primask == 1U);
        assert_zero_compare();
        test_primask = 0U;
        motor_driver_set_output(500, 600);
        assert(TIM1->CCR1 == 500U && TIM1->CCR4 == 600U);
        assert(start_calls == 4U);
    } else if (strcmp(argv[1], "cold_stop") == 0) {
        /* No handle, peripheral clock, or driver initialization. */
        motor_driver_stop();
        assert(start_calls == 0U && !motor_driver_fault_latched());
        motor_driver_emergency_stop();
        assert_fatal_stop();
        prepare_timer();
        /* Simulate GPIO setup after a pre-initialization fault. */
        motor_driver_emergency_stop();
        assert_restart_refused();
    } else if (strcmp(argv[1], "running_stop") == 0) {
        prepare_timer();
        motor_driver_set_output(4000, 4200);
        const uint32_t other_modes = GPIOA->MODER & ~0x00FF0000U;
        const uint32_t other_pulls = GPIOA->PUPDR & ~0x00FF0000U;
        const uint32_t other_types = GPIOA->OTYPER & ~0x0F00U;
        test_primask = 1U;
        motor_driver_emergency_stop();
        assert(test_primask == 1U);
        assert(GPIOA->MODER == (other_modes | 0x00550000U));
        assert(GPIOA->PUPDR == other_pulls);
        assert(GPIOA->OTYPER == other_types);
        assert_fatal_stop();
        test_primask = 0U;
        assert_restart_refused();
    } else if (strcmp(argv[1], "stop_before_driver_init") == 0) {
        prepare_timer();
        TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 1500U;
        motor_driver_stop();
        assert_zero_compare();
        assert(start_calls == 0U && !motor_driver_fault_latched());
    } else if (strcmp(argv[1], "missing_clock") == 0) {
        htim1.Instance = TIM1;
        motor_driver_set_output(1500, 1600);
        assert(start_calls == 0U);
        assert_fatal_stop();
        assert_restart_refused();
    } else if (strncmp(argv[1], "start_failure_", 14U) == 0) {
        prepare_timer();
        fail_start_call = (unsigned)atoi(argv[1] + 14U);
        assert(fail_start_call >= 1U && fail_start_call <= 4U);
        motor_driver_set_output(1500, 1600);
        assert(start_calls == fail_start_call);
        assert_fatal_stop();
        assert_restart_refused();
    } else {
        fprintf(stderr, "Unknown scenario: %s\n", argv[1]);
        return 2;
    }
    assert(test_primask == 0U);
    printf("PASS motor hardware: %s\n", argv[1]);
    return 0;
}
