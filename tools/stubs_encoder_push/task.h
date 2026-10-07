#ifndef TEST_ENCODER_PUSH_TASK_H
#define TEST_ENCODER_PUSH_TASK_H
#include "FreeRTOS.h"
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, unsigned stack,
                      void *argument, UBaseType_t priority, TaskHandle_t *handle);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *wake, TickType_t interval);
#endif
