#ifndef TEST_MOTOR_USB_TIM_H
#define TEST_MOTOR_USB_TIM_H
#include "main.h"
extern TIM_HandleTypeDef htim1, htim2, htim3, htim5;
void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timer);
#endif
