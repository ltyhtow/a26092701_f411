/* Link the real diagnostic task AND motor driver; delay mocks inject a fault.
 * Run each scenario in a fresh process, with MOTOR_DIRECT_GPIO_DRIVE=0 and =1.
 */
#include "motor_polarity_test_task.h"
#include "motor_driver.h"
#include "motor_config.h"
#include "board_diagnostics.h"
#include "app_clock.h"
#include "main.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

TIM_TypeDef test_tim1;
RCC_TypeDef test_rcc;
GPIO_TypeDef test_gpioa;
TIM_HandleTypeDef htim1;
uint32_t test_primask;
static TaskFunction_t task_entry;
static unsigned delays, deletes, positive_writes, inject_on_delay, pwm_starts;
static uint32_t gpio_levels;

TickType_t app_clock_period_ticks(uint32_t period_ms) { return period_ms; }

BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, unsigned stack,
                      void *argument, UBaseType_t priority, TaskHandle_t *handle) {
    (void)name; (void)stack; (void)argument; (void)priority;
    task_entry = entry;
    *handle = (void *)1;
    return pdPASS;
}

void vTaskDelete(TaskHandle_t handle) {
    assert(handle == NULL);
    ++deletes;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config) {
    assert(test_primask == 1U && !motor_driver_fault_latched());
    assert(port == GPIOA && config->Pin == 0x0F00U);
    assert(config->Mode == GPIO_MODE_OUTPUT_PP);
    port->MODER = (port->MODER & ~0x00FF0000U) | 0x00550000U;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, uint32_t value) {
    assert(port == GPIOA && test_primask == 1U);
    if (value == GPIO_PIN_SET) {
        assert(!motor_driver_fault_latched());
        assert(pins == (GPIO_PIN_8 | GPIO_PIN_11));
        gpio_levels |= pins;
        ++positive_writes;
    } else {
        gpio_levels &= ~(uint32_t)pins;
    }
    port->BSRR = value == GPIO_PIN_SET ? pins : (uint32_t)pins << 16U;
}

void test_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value) {
    assert(test_primask == 1U && timer->Instance == TIM1);
    if (value != 0U) {
        assert(!motor_driver_fault_latched());
        assert(value == MOTOR_TEST_DUTY);
        ++positive_writes;
    }
    switch (channel) {
        case TIM_CHANNEL_1: TIM1->CCR1 = value; break;
        case TIM_CHANNEL_2: TIM1->CCR2 = value; break;
        case TIM_CHANNEL_3: TIM1->CCR3 = value; break;
        case TIM_CHANNEL_4: TIM1->CCR4 = value; break;
        default: assert(0);
    }
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    assert(test_primask == 1U && !motor_driver_fault_latched());
    assert(timer->Instance == TIM1);
    ++pwm_starts;
    TIM1->CCER |= 1UL << channel;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}

void vTaskDelay(TickType_t ticks) {
    assert(test_primask == 0U);
    ++delays;
    assert(delays <= 2U); /* No repeating/unbounded diagnostic drive. */
    if (delays == 1U) {
        assert(ticks == pdMS_TO_TICKS(1000U));
        assert(positive_writes == 0U);
    } else {
        assert(ticks == pdMS_TO_TICKS(MOTOR_TEST_DURATION_MS));
#if MOTOR_DIRECT_GPIO_DRIVE
        assert(gpio_levels == (GPIO_PIN_8 | GPIO_PIN_11));
        assert(pwm_starts == 0U);
#else
        assert(TIM1->CCR1 == MOTOR_TEST_DUTY && TIM1->CCR2 == 0U);
        assert(TIM1->CCR3 == 0U && TIM1->CCR4 == MOTOR_TEST_DUTY);
        assert(pwm_starts == 4U);
#endif
    }
    if (delays == inject_on_delay) {
        motor_driver_emergency_stop();
        assert(motor_driver_fault_latched());
        assert(GPIOA->BSRR == 0x0F000000U);
        assert((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    }
}

int main(int argc, char **argv) {
    assert(argc == 2);
    if (strcmp(argv[1], "fault_wait") == 0) inject_on_delay = 1U;
    else if (strcmp(argv[1], "fault_running") == 0) inject_on_delay = 2U;
    else assert(strcmp(argv[1], "normal") == 0);
    htim1.Instance = TIM1;
    RCC->APB2ENR = RCC_APB2ENR_TIM1EN;
    TaskHandle_t task_handle;
    assert(motor_polarity_test_task_start(&task_handle) == pdPASS);
    task_entry(NULL);
    assert(deletes == 1U && test_primask == 0U);
    assert(TIM1->CCR1 == 0U && TIM1->CCR2 == 0U);
    assert(TIM1->CCR3 == 0U && TIM1->CCR4 == 0U);
    assert(gpio_levels == 0U);
    if (inject_on_delay == 1U) {
        assert(delays == 1U && positive_writes == 0U && pwm_starts == 0U);
    } else assert(delays == 2U && positive_writes > 0U);
    if (inject_on_delay != 0U) {
        assert(motor_polarity_test_task_start(&task_handle) == pdFAIL);
        const unsigned writes_before = positive_writes;
        assert(!board_motor_test_start(false));
        assert(!board_motor_test_start(true));
        assert(positive_writes == writes_before && test_primask == 0U);
    }
    printf("PASS motor diagnostic mode=%u scenario=%s\n", MOTOR_DIRECT_GPIO_DRIVE, argv[1]);
    return 0;
}
