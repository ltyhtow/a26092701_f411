#ifndef TEST_PROTOCOL_SEMPHR_H
#define TEST_PROTOCOL_SEMPHR_H
#include "FreeRTOS.h"
SemaphoreHandle_t xSemaphoreCreateMutex(void);
void vSemaphoreDelete(SemaphoreHandle_t handle);
BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t timeout);
BaseType_t xSemaphoreGive(SemaphoreHandle_t handle);
#endif
