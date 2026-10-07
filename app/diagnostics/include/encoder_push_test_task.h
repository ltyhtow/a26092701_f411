#ifndef ENCODER_PUSH_TEST_TASK_H
#define ENCODER_PUSH_TEST_TASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#define ENCODER_PUSH_TEST_PERIOD_MS 50U
BaseType_t encoder_push_test_task_start(TaskHandle_t *task_handle);
#endif
