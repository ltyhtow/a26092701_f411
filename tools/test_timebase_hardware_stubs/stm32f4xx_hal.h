#ifndef TEST_TIMEBASE_HAL_H
#define TEST_TIMEBASE_HAL_H

#include <stdint.h>

typedef struct { uint32_t unused; } TIM_TypeDef;
typedef struct {
    uint32_t Period, Prescaler, ClockDivision, CounterMode, AutoReloadPreload;
} TIM_Base_InitTypeDef;
typedef struct {
    TIM_TypeDef *Instance;
    TIM_Base_InitTypeDef Init;
} TIM_HandleTypeDef;
typedef struct { uint32_t APB2CLKDivider; } RCC_ClkInitTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;

extern TIM_TypeDef test_tim11;
extern uint32_t uwTickPrio;
extern uint32_t test_clock_enabled, test_flags, test_interrupt_enabled;

#define TIM11 (&test_tim11)
#define __NVIC_PRIO_BITS 4U
#define RCC_HCLK_DIV1 1U
#define RCC_HCLK_DIV2 2U
#define TIM_COUNTERMODE_UP 0U
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0U
#define TIM1_TRG_COM_TIM11_IRQn 26U
#define TIM_FLAG_UPDATE 1U
#define TIM_IT_UPDATE 1U
#define __HAL_RCC_TIM11_CLK_ENABLE() (test_clock_enabled = 1U)
#define __HAL_TIM_CLEAR_FLAG(timer, flag) ((void)(timer), test_flags &= ~(flag))
#define __HAL_TIM_DISABLE_IT(timer, bit) ((void)(timer), test_interrupt_enabled &= ~(bit))
#define __HAL_TIM_ENABLE_IT(timer, bit) ((void)(timer), test_interrupt_enabled |= (bit))

void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *config, uint32_t *latency);
uint32_t HAL_RCC_GetPCLK2Freq(void);
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer);
void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(uint32_t irq);
void HAL_IncTick(void);

#endif
