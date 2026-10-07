#ifndef TEST_BALANCE_TASK_H
#define TEST_BALANCE_TASK_H
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *previous, TickType_t increment);
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, uint16_t stack,
                       void *argument, UBaseType_t priority, TaskHandle_t *handle);
#endif
