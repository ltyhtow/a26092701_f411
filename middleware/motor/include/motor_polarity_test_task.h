#ifndef MOTOR_POLARITY_TEST_TASK_H
#define MOTOR_POLARITY_TEST_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t motor_polarity_test_task_start(TaskHandle_t *task_handle);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_POLARITY_TEST_TASK_H */
