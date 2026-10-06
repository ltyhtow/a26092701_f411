#include "motor_polarity_test_task.h"

#include "motor_config.h"
#include "serial_transport.h"
#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

#define MOTOR_POLARITY_TEST_TASK_STACK_DEPTH_WORDS 256U

#if MOTOR_DIRECT_GPIO_DRIVE

static void motor_direct_gpio_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 先将所有输出引脚拉低，避免未初始化前的电平抖动 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void motor_direct_gpio_set_forward(void) {
    /* 电机 1 (A 路): AIN1(PA8)=HIGH (3.3V), AIN2(PA9)=LOW (0V) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

    /* 电机 2 (B 路): BIN1(PA10)=HIGH (3.3V), BIN2(PA11)=LOW (0V) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

static void motor_polarity_test_task_entry(void *argument) {
    (void)argument;

    /* 重新配置 PA8~PA11 为普通 GPIO 推挽输出，不经过 PWM 发生器 */
    motor_direct_gpio_init();

    /* 启动延时 1 秒，给操作者准备时间 */
    vTaskDelay(pdMS_TO_TICKS(1000U));

    /* A 路和 B 路同时直接拉高正转输出 (持续有效直到断电或复位) */
    motor_direct_gpio_set_forward();

    for (;;) {
        serial_transport_send_string("[MOTOR TEST] DIRECT GPIO: AIN1(PA8)=1 AIN2(PA9)=0 | BIN1(PA10)=1 BIN2(PA11)=0\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

#else

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

#endif

BaseType_t motor_polarity_test_task_start(TaskHandle_t *task_handle) {
    if (task_handle == NULL) {
        return pdFAIL;
    }
    return xTaskCreate(motor_polarity_test_task_entry, "MotorTest",
                       MOTOR_POLARITY_TEST_TASK_STACK_DEPTH_WORDS, NULL, 1U,
                       task_handle);
}
