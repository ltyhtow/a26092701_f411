#ifndef TEST_IMU_PORT_TASK_H
#define TEST_IMU_PORT_TASK_H
#include "FreeRTOS.h"
BaseType_t xTaskGetSchedulerState(void);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t delay);
void vTaskDelayUntil(TickType_t *wake, TickType_t delay);
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, unsigned stack,
                      void *argument, UBaseType_t priority, TaskHandle_t *handle);
#endif
