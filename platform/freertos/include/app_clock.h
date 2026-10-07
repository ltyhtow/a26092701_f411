#ifndef APP_CLOCK_H
#define APP_CLOCK_H
#include "FreeRTOS.h"
#include <stdint.h>
/* Task context, shared 32-bit monotonic milliseconds, including tick wrap. */
uint32_t app_clock_now_ms(void);
TickType_t app_clock_period_ticks(uint32_t milliseconds);
#endif
