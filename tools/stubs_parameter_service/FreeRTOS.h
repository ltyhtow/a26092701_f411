#ifndef TEST_PARAMETER_FREERTOS_H
#define TEST_PARAMETER_FREERTOS_H
#include <stdint.h>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdFAIL 0
#define portMAX_DELAY UINT32_MAX
void parameter_test_enter_critical(void);
void parameter_test_exit_critical(void);
#define taskENTER_CRITICAL() parameter_test_enter_critical()
#define taskEXIT_CRITICAL() parameter_test_exit_critical()
#endif
