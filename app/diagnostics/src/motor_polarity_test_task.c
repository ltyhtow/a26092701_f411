#include "motor_polarity_test_task.h"
#include "motor_driver.h"
#include "board_diagnostics.h"
#include "app_clock.h"

#define MOTOR_POLARITY_TEST_TASK_STACK_DEPTH_WORDS 256U

static void motor_polarity_test_task_entry(void *argument) {
    (void)argument;
    board_motor_test_stop(MOTOR_DIRECT_GPIO_DRIVE != 0U);
    vTaskDelay(app_clock_period_ticks(1000U));
    /* The board checks the fatal latch atomically with all output writes. */
    if (board_motor_test_start(MOTOR_DIRECT_GPIO_DRIVE != 0U)) {
        vTaskDelay(app_clock_period_ticks(MOTOR_TEST_DURATION_MS));
    }
    board_motor_test_stop(MOTOR_DIRECT_GPIO_DRIVE != 0U);
    vTaskDelete(NULL);
}

BaseType_t motor_polarity_test_task_start(TaskHandle_t *task_handle) {
    if (task_handle == NULL || motor_driver_fault_latched()) return pdFAIL;
    return xTaskCreate(motor_polarity_test_task_entry, "MotorTest",
                       MOTOR_POLARITY_TEST_TASK_STACK_DEPTH_WORDS, NULL, 1U,
                       task_handle);
}
