#ifndef TEST_ENCODER_TIM_H
#define TEST_ENCODER_TIM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct {
    uint32_t counter;
    bool running;
    HAL_StatusTypeDef start_result;
    unsigned start_calls;
    unsigned stop_calls;
} TIM_HandleTypeDef;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

#define TIM_CHANNEL_ALL 0xffffU
#define __HAL_TIM_SET_COUNTER(timer, value) ((timer)->counter = (value))
#define __HAL_TIM_GET_COUNTER(timer) ((timer)->counter)

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channels);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *timer, uint32_t channels);

#endif
