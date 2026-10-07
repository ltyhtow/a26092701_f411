#ifndef MONOTONIC_CLOCK_H
#define MONOTONIC_CLOCK_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t last_ticks, milliseconds, fractional;
    bool initialized;
} monotonic_clock_t;
/* Observe at least once per full 32-bit tick wrap. Frequency is immutable. */
uint32_t monotonic_clock_update(monotonic_clock_t *clock, uint32_t ticks, uint32_t tick_hz);
uint32_t monotonic_clock_period_ticks(uint32_t milliseconds, uint32_t tick_hz);
#endif
