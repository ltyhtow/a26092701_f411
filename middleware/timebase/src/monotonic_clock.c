#include "monotonic_clock.h"
#include <stddef.h>
uint32_t monotonic_clock_update(monotonic_clock_t *clock, uint32_t ticks, uint32_t tick_hz) {
    if (clock == NULL || tick_hz == 0U) return 0U;
    uint64_t elapsed;
    if (!clock->initialized) {
        elapsed = (uint64_t)ticks * 1000U;
        clock->initialized = true;
    } else {
        elapsed = (uint64_t)(uint32_t)(ticks - clock->last_ticks) * 1000U + clock->fractional;
    }
    clock->milliseconds += (uint32_t)(elapsed / tick_hz);
    clock->fractional = (uint32_t)(elapsed % tick_hz);
    clock->last_ticks = ticks;
    return clock->milliseconds;
}
uint32_t monotonic_clock_period_ticks(uint32_t milliseconds, uint32_t tick_hz) {
    if (tick_hz == 0U) return 0U;
    uint64_t ticks = ((uint64_t)milliseconds * tick_hz + 999U) / 1000U;
    if (ticks == 0U) return 1U;
    return ticks >= UINT32_MAX ? UINT32_MAX - 1U : (uint32_t)ticks;
}
