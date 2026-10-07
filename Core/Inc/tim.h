#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim1;
#if SERIAL_TRANSPORT_USB_CDC
extern TIM_HandleTypeDef htim3;
#endif
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

void MX_TIM1_Init(void);
#if SERIAL_TRANSPORT_USB_CDC
void MX_TIM3_Init(void);
#endif
void MX_TIM2_Init(void);
void MX_TIM5_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
