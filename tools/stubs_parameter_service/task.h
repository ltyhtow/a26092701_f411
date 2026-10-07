#ifndef TEST_PARAMETER_TASK_H
#define TEST_PARAMETER_TASK_H
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
#define taskSCHEDULER_NOT_STARTED 0
#define taskSCHEDULER_RUNNING 1
BaseType_t xTaskGetSchedulerState(void);
BaseType_t xTaskCreate(TaskFunction_t entry, const char *name, uint16_t stack,
    void *argument, UBaseType_t priority, TaskHandle_t *handle);
void vTaskDelete(TaskHandle_t handle);
#endif
