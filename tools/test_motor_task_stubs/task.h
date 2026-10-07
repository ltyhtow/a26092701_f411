#ifndef TEST_MOTOR_TASK_TASK_H
#define TEST_MOTOR_TASK_TASK_H
#include "FreeRTOS.h"
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, unsigned stack,
                      void *argument, UBaseType_t priority, TaskHandle_t *handle);
void vTaskDelay(TickType_t ticks);
void vTaskDelete(TaskHandle_t handle);
#endif
