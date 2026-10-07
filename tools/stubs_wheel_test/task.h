#ifndef TEST_WHEEL_TASK_H
#define TEST_WHEEL_TASK_H
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
BaseType_t xTaskCreate(TaskFunction_t function, const char *name, unsigned stack,
                      void *argument, UBaseType_t priority, TaskHandle_t *handle);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *wake, TickType_t interval);
#endif
