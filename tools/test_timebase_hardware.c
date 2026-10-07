/* Links the production TIM11 timebase against a small clock/IRQ model. */
#include "stm32f4xx_hal.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

TIM_TypeDef test_tim11;
uint32_t uwTickPrio;
uint32_t test_clock_enabled, test_flags, test_interrupt_enabled;
static uint32_t pclk2, divider, tick_count, irq_priority;
static unsigned base_calls, start_calls, irq_enable_calls;
static bool priority_set;
static HAL_StatusTypeDef base_result, start_result;
extern TIM_HandleTypeDef htim11;

HAL_StatusTypeDef HAL_InitTick(uint32_t priority);
HAL_StatusTypeDef app_hal_timebase_status(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);

void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *config, uint32_t *latency) {
    config->APB2CLKDivider = divider;
    *latency = 3U;
}

uint32_t HAL_RCC_GetPCLK2Freq(void) { return pclk2; }

HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *timer) {
    assert(test_clock_enabled && timer->Instance == TIM11);
    ++base_calls;
    test_flags |= TIM_FLAG_UPDATE;
    return base_result;
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer) {
    assert(timer->Instance == TIM11);
    assert(priority_set && (test_flags & TIM_FLAG_UPDATE) == 0U);
    ++start_calls;
    if (start_result == HAL_OK) {
        test_interrupt_enabled |= TIM_IT_UPDATE;
    }
    return start_result;
}

void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority) {
    assert(irq == TIM1_TRG_COM_TIM11_IRQn && subpriority == 0U);
    irq_priority = priority;
    priority_set = true;
}

void HAL_NVIC_EnableIRQ(uint32_t irq) {
    assert(irq == TIM1_TRG_COM_TIM11_IRQn);
    assert(priority_set && start_result == HAL_OK);
    ++irq_enable_calls;
}

void HAL_IncTick(void) { ++tick_count; }

static void test_frequency(uint32_t peripheral_clock, uint32_t apb_divider) {
    pclk2 = peripheral_clock;
    divider = apb_divider;
    priority_set = false;
    assert(HAL_InitTick(15U) == HAL_OK);
    assert(app_hal_timebase_status() == HAL_OK);
    const uint32_t timer_clock = pclk2 * (divider == RCC_HCLK_DIV1 ? 1U : 2U);
    assert(timer_clock / ((htim11.Init.Prescaler + 1U) *
                         (htim11.Init.Period + 1U)) == 1000U);
    assert(irq_priority == 15U && uwTickPrio == 15U);
}

int main(void) {
    assert(app_hal_timebase_status() == HAL_ERROR);
    /* Invalid priority must not enable an IRQ at the reset priority. */
    assert(HAL_InitTick(16U) == HAL_ERROR);
    assert(app_hal_timebase_status() == HAL_ERROR);
    assert(base_calls == 0U && start_calls == 0U && irq_enable_calls == 0U);
    test_frequency(16000000U, RCC_HCLK_DIV1);  /* HAL_Init before PLL */
    test_frequency(100000000U, RCC_HCLK_DIV1); /* Current 100MHz configuration */
    test_frequency(50000000U, RCC_HCLK_DIV2);  /* APB prescaler regression */
    assert(base_calls == 3U && start_calls == 3U && irq_enable_calls == 3U);

    for (unsigned i = 0U; i < 1000U; ++i) {
        HAL_TIM_PeriodElapsedCallback(&htim11);
    }
    assert(tick_count == 1000U);
    TIM_TypeDef unrelated_timer;
    TIM_HandleTypeDef unrelated = {.Instance = &unrelated_timer};
    HAL_TIM_PeriodElapsedCallback(&unrelated);
    assert(tick_count == 1000U);

    HAL_SuspendTick();
    assert((test_interrupt_enabled & TIM_IT_UPDATE) == 0U);
    HAL_ResumeTick();
    assert((test_interrupt_enabled & TIM_IT_UPDATE) != 0U);

    base_result = HAL_ERROR;
    assert(HAL_InitTick(15U) == HAL_ERROR);
    assert(app_hal_timebase_status() == HAL_ERROR);
    assert(start_calls == 3U && irq_enable_calls == 3U);
    base_result = HAL_OK;
    start_result = HAL_ERROR;
    assert(HAL_InitTick(15U) == HAL_ERROR);
    assert(app_hal_timebase_status() == HAL_ERROR);
    assert(start_calls == 4U && irq_enable_calls == 3U);
    puts("PASS TIM11 hardware timebase: 1kHz clocks, isolated callback, IRQ ordering, failures");
    return 0;
}
