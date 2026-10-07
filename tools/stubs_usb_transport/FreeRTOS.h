#ifndef TEST_USB_TRANSPORT_FREERTOS_H
#define TEST_USB_TRANSPORT_FREERTOS_H
#include <stdint.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#define pdFALSE 0
#define pdTRUE 1
#define pdPASS 1
#define pdFAIL 0
void test_enter_critical(void);
void test_exit_critical(void);
void test_yield_from_isr(BaseType_t value);
#define taskENTER_CRITICAL() test_enter_critical()
#define taskEXIT_CRITICAL() test_exit_critical()
#define portYIELD_FROM_ISR(value) test_yield_from_isr(value)
#endif
