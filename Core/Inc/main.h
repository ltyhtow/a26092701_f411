/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define SYS_LED_Pin GPIO_PIN_13
#define SYS_LED_GPIO_Port GPIOC

#define MOTOR1_IN1_Pin GPIO_PIN_8
#define MOTOR1_IN1_GPIO_Port GPIOA
#define MOTOR1_IN2_Pin GPIO_PIN_9
#define MOTOR1_IN2_GPIO_Port GPIOA
#define MOTOR2_IN1_Pin GPIO_PIN_10
#define MOTOR2_IN1_GPIO_Port GPIOA
#define MOTOR2_IN2_Pin GPIO_PIN_11
#define MOTOR2_IN2_GPIO_Port GPIOA

#define VBAT_ADC_Pin GPIO_PIN_0
#define VBAT_ADC_GPIO_Port GPIOB

#define MPU6050_INT_Pin GPIO_PIN_1
#define MPU6050_INT_GPIO_Port GPIOB
#define MPU6050_INT_EXTI_IRQn EXTI1_IRQn

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
