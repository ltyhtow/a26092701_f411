#include "app_clock.h"
#include "monotonic_clock.h"
#include "task.h"
static monotonic_clock_t clock_state;
_Static_assert(sizeof(TickType_t) == sizeof(uint32_t), "app clock requires 32-bit FreeRTOS ticks");
uint32_t app_clock_now_ms(void) {
    taskENTER_CRITICAL();
    uint32_t ms = monotonic_clock_update(&clock_state, (uint32_t)xTaskGetTickCount(), configTICK_RATE_HZ);
    taskEXIT_CRITICAL();
    return ms;
}
TickType_t app_clock_period_ticks(uint32_t milliseconds) {
    return (TickType_t)monotonic_clock_period_ticks(milliseconds, configTICK_RATE_HZ);
}
