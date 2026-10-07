#ifndef TEST_IMU_PORT_FREERTOS_H
#define TEST_IMU_PORT_FREERTOS_H
#include <stddef.h>
#include <stdint.h>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
#define pdPASS 1
#define pdFAIL 0
#define taskSCHEDULER_NOT_STARTED 0
#endif
