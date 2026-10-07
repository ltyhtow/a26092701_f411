#include "monotonic_clock.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
static void check_frequency(uint32_t hz) {
    monotonic_clock_t clock = {0};
    assert(monotonic_clock_update(&clock, 0, hz) == 0);
    for (uint32_t i = 1; i <= hz * 3; ++i)
        assert(monotonic_clock_update(&clock, i, hz) == (uint64_t)i * 1000 / hz);
    assert(clock.milliseconds == 3000);
    /* Begin near the tick wrap, then accumulate exact time through it. */
    clock = (monotonic_clock_t){0};
    uint32_t first = UINT32_MAX - hz;
    uint32_t initial_ms = monotonic_clock_update(&clock, first, hz);
    uint32_t initial_fraction = clock.fractional;
    for (uint32_t i = 1; i <= hz * 2; ++i) {
        uint32_t actual = monotonic_clock_update(&clock, first + i, hz);
        uint32_t expected = initial_ms + (uint32_t)(((uint64_t)i * 1000 + initial_fraction) / hz);
        assert(actual == expected);
    }
    assert(monotonic_clock_period_ticks(0, hz) == 1);
    assert(monotonic_clock_period_ticks(5, hz) == (5 * hz + 999) / 1000);
}
int main(void) {
    check_frequency(100); check_frequency(1000); check_frequency(1024); check_frequency(2000);
    monotonic_clock_t clock = {.initialized = true, .last_ticks = 30, .milliseconds = UINT32_MAX - 2};
    assert(monotonic_clock_update(&clock, 35, 1000) == 2);
    assert(monotonic_clock_update(NULL, 5, 1000) == 0);
    assert(monotonic_clock_period_ticks(5, 0) == 0);
    assert(monotonic_clock_period_ticks(UINT32_MAX, UINT32_MAX) == UINT32_MAX - 1);
    puts("PASS: 100/1000/1024/2000 Hz, fractions, tick and millisecond wraps, interval rounding");
    return 0;
}
