#ifndef TEST_PROTOCOL_TASK_H
#define TEST_PROTOCOL_TASK_H
#include "FreeRTOS.h"
#define taskSCHEDULER_NOT_STARTED 0
#define taskSCHEDULER_RUNNING 1
BaseType_t xTaskGetSchedulerState(void);
BaseType_t xTaskCreate(void (*task)(void *), const char *name, uint32_t stack,
                       void *argument, UBaseType_t priority, TaskHandle_t *handle);
void vTaskDelete(TaskHandle_t handle);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks);
#endif
