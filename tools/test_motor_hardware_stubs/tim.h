#ifndef TEST_MOTOR_TIM_H
#define TEST_MOTOR_TIM_H

#include <stdint.h>

typedef struct {
    volatile uint32_t CR1, CCER, BDTR, CCR1, CCR2, CCR3, CCR4;
} TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { volatile uint32_t APB2ENR, AHB1ENR; } RCC_TypeDef;
typedef struct { volatile uint32_t MODER, OTYPER, PUPDR, BSRR; } GPIO_TypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;

extern TIM_TypeDef test_tim1;
extern RCC_TypeDef test_rcc;
extern GPIO_TypeDef test_gpioa;
extern TIM_HandleTypeDef htim1;
extern uint32_t test_primask;

#define TIM1 (&test_tim1)
#define RCC (&test_rcc)
#define GPIOA (&test_gpioa)
#define RCC_APB2ENR_TIM1EN (1UL << 0U)
#define TIM_BDTR_MOE (1UL << 15U)
#define TIM_CR1_CEN 1UL
#define GPIO_PIN_8 (1U << 8U)
#define GPIO_PIN_9 (1U << 9U)
#define GPIO_PIN_10 (1U << 10U)
#define GPIO_PIN_11 (1U << 11U)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define __HAL_RCC_GPIOA_CLK_ENABLE() (test_rcc.AHB1ENR |= 1U)
#define __get_PRIMASK() test_primask
#define __disable_irq() (test_primask = 1U)
#define __set_PRIMASK(value) (test_primask = (value))
#define __DSB() ((void)0)

void test_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value);
#define __HAL_TIM_SET_COMPARE(timer, channel, value) test_set_compare(timer, channel, value)
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);

#endif
