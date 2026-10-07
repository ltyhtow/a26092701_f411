#ifndef MOTOR_POLARITY_TEST_TASK_H
#define MOTOR_POLARITY_TEST_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"

/* Diagnostic drive ends after this interval and does not repeat. */
#ifndef MOTOR_TEST_DURATION_MS
#define MOTOR_TEST_DURATION_MS 1000U
#endif

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t motor_polarity_test_task_start(TaskHandle_t *task_handle);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_POLARITY_TEST_TASK_H */
