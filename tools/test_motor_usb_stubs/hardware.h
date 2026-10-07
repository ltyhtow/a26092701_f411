#ifndef TEST_MOTOR_USB_HARDWARE_H
#define TEST_MOTOR_USB_HARDWARE_H
#include <stdint.h>

typedef struct { volatile uint32_t CR1, CCER, BDTR, CCR1, CCR2, CCR3, CCR4, PSC, ARR; } TIM_TypeDef;
typedef struct { volatile uint32_t APB1ENR, APB2ENR, AHB1ENR; } RCC_TypeDef;
typedef struct { volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, BSRR, AFR[2], ODR; } GPIO_TypeDef;
typedef struct {
    uint32_t Prescaler, CounterMode, Period, ClockDivision, RepetitionCounter, AutoReloadPreload;
} TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init; uint32_t State; } TIM_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { uint32_t ClockSource; } TIM_ClockConfigTypeDef;
typedef struct { uint32_t MasterOutputTrigger, MasterSlaveMode; } TIM_MasterConfigTypeDef;
typedef struct {
    uint32_t OCMode, Pulse, OCPolarity, OCNPolarity, OCFastMode, OCIdleState, OCNIdleState;
} TIM_OC_InitTypeDef;
typedef struct {
    uint32_t OffStateRunMode, OffStateIDLEMode, LockLevel, DeadTime, BreakState, BreakPolarity, AutomaticOutput;
} TIM_BreakDeadTimeConfigTypeDef;
typedef struct {
    uint32_t EncoderMode, IC1Polarity, IC1Selection, IC1Prescaler, IC1Filter;
    uint32_t IC2Polarity, IC2Selection, IC2Prescaler, IC2Filter;
} TIM_Encoder_InitTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;

extern TIM_TypeDef test_tim1, test_tim2, test_tim3, test_tim5;
extern RCC_TypeDef test_rcc;
extern GPIO_TypeDef test_gpioa, test_gpiob;
extern uint32_t test_primask;
#define TIM1 (&test_tim1)
#define TIM2 (&test_tim2)
#define TIM3 (&test_tim3)
#define TIM5 (&test_tim5)
#define RCC (&test_rcc)
#define GPIOA (&test_gpioa)
#define GPIOB (&test_gpiob)
#define RCC_APB2ENR_TIM1EN 1UL
#define RCC_APB1ENR_TIM3EN (1UL << 1U)
#define TIM_BDTR_MOE (1UL << 15U)
#define TIM_CR1_CEN 1UL
#define GPIO_PIN_5 (1U << 5U)
#define GPIO_PIN_8 (1U << 8U)
#define GPIO_PIN_9 (1U << 9U)
#define GPIO_PIN_10 (1U << 10U)
#define GPIO_PIN_11 (1U << 11U)
#define GPIO_PIN_12 (1U << 12U)
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_AF1_TIM1 1U
#define GPIO_AF2_TIM3 2U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_AUTORELOAD_PRELOAD_ENABLE 1U
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0U
#define TIM_CLOCKSOURCE_INTERNAL 0U
#define TIM_TRGO_RESET 0U
#define TIM_MASTERSLAVEMODE_DISABLE 0U
#define TIM_OCMODE_PWM1 6U
#define TIM_OCPOLARITY_HIGH 0U
#define TIM_OCNPOLARITY_HIGH 0U
#define TIM_OCFAST_DISABLE 0U
#define TIM_OCIDLESTATE_RESET 0U
#define TIM_OCNIDLESTATE_RESET 0U
#define TIM_OSSR_DISABLE 0U
#define TIM_OSSI_DISABLE 0U
#define TIM_LOCKLEVEL_OFF 0U
#define TIM_BREAK_DISABLE 0U
#define TIM_BREAKPOLARITY_HIGH 1U
#define TIM_AUTOMATICOUTPUT_DISABLE 0U
#define TIM_ENCODERMODE_TI12 3U
#define TIM_ICPOLARITY_RISING 0U
#define TIM_ICSELECTION_DIRECTTI 1U
#define TIM_ICPSC_DIV1 0U
#define __HAL_RCC_GPIOA_CLK_ENABLE() (RCC->AHB1ENR |= 1U)
#define __HAL_RCC_GPIOB_CLK_ENABLE() (RCC->AHB1ENR |= 2U)
#define __get_PRIMASK() test_primask
#define __disable_irq() (test_primask = 1U)
#define __set_PRIMASK(value) (test_primask = (value))
#define __DSB() ((void)0)
#define __HAL_TIM_SET_COMPARE(timer, channel, value) test_set_compare(timer, channel, value)

void test_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value);
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *timer, TIM_ClockConfigTypeDef *config);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *timer, TIM_MasterConfigTypeDef *config);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer, TIM_OC_InitTypeDef *config, uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *timer, TIM_BreakDeadTimeConfigTypeDef *config);
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *timer, TIM_Encoder_InitTypeDef *config);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, uint32_t value);
#endif
