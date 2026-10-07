#ifndef TEST_SERIAL_TASK_H
#define TEST_SERIAL_TASK_H
#include "FreeRTOS.h"
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken);
#endif
