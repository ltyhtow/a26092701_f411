#include "motor_polarity_test_task.h"

#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

#define MOTOR_POLARITY_TEST_TASK_STACK_DEPTH_WORDS 256U

static void motor_outputs_set(uint32_t motor_a_in1,
                              uint32_t motor_a_in2,
                              uint32_t motor_b_in1,
                              uint32_t motor_b_in2) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_a_in1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_a_in2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor_b_in1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor_b_in2);
}

static BaseType_t motor_outputs_start(void) {
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
        return pdFAIL;
    }
    __HAL_TIM_MOE_ENABLE(&htim1);
    return pdPASS;
}

static void motor_polarity_test_task_entry(void *argument) {
    (void)argument;
    motor_outputs_set(MOTOR_TEST_DUTY, 0U, MOTOR_TEST_DUTY, 0U);
    if (motor_outputs_start() != pdPASS) {
        motor_outputs_set(0U, 0U, 0U, 0U);
        __HAL_TIM_MOE_DISABLE(&htim1);
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

BaseType_t motor_polarity_test_task_start(TaskHandle_t *task_handle) {
    if (task_handle == NULL) {
        return pdFAIL;
    }
    return xTaskCreate(motor_polarity_test_task_entry, "MotorTest",
                       MOTOR_POLARITY_TEST_TASK_STACK_DEPTH_WORDS, NULL, 1U,
                       task_handle);
}
