#ifndef TEST_ENCODER_PUSH_FREERTOS_H
#define TEST_ENCODER_PUSH_FREERTOS_H
#include <stddef.h>
#include <stdint.h>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
#define pdFAIL 0
#define pdPASS 1
#define pdMS_TO_TICKS(value) (value)
#define portTICK_PERIOD_MS 1U
#endif
