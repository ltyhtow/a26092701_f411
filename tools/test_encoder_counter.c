/* No HAL, RTOS, board definitions, or stubs: portable counter behavior. */
#include "encoder_counter.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>

static void independent_contexts_and_polarity(void) {
    encoder_counter_t a, b;
    encoder_counter_init(&a, false, true, UINT32_MAX - 2U, 7U);
    encoder_counter_init(&b, true, false, 100U, 200U);
    assert(!a.snapshot.valid && !b.snapshot.valid);
    assert(encoder_counter_sample(&a, 5U, 2U));
    assert(a.snapshot.delta_left == 8 && a.snapshot.delta_right == 5);
    assert(a.snapshot.total_left == 8 && a.snapshot.total_right == 5);
    assert(encoder_counter_sample(&b, 97U, 195U));
    assert(b.snapshot.delta_left == 3 && b.snapshot.delta_right == -5);
    assert(a.snapshot.total_left == 8 && a.snapshot.total_right == 5);
    encoder_counter_snapshot_t copied;
    encoder_counter_get_snapshot(&a, &copied);
    assert(copied.raw_left == 5U && copied.raw_right == 2U && copied.valid);
    encoder_counter_reset_totals(&a);
    assert(a.snapshot.total_left == 0 && a.snapshot.total_right == 0);
    assert(encoder_counter_sample(&a, 6U, 1U));
    assert(a.snapshot.delta_left == 1 && a.snapshot.delta_right == 1);
}

static void invalid_sample_recovery(void) {
    encoder_counter_t counter;
    encoder_counter_init(&counter, true, false, 0U, 0U);
    assert(encoder_counter_sample(&counter, 10U, 20U));
    assert(!encoder_counter_sample(&counter, 10U + UINT32_C(0x80000000), 30U));
    assert(!counter.snapshot.valid);
    assert(counter.snapshot.delta_left == 0 && counter.snapshot.delta_right == 0);
    assert(counter.snapshot.total_left == -10 && counter.snapshot.total_right == 20);
    assert(encoder_counter_sample(&counter, 11U + UINT32_C(0x80000000), 31U));
    assert(counter.snapshot.delta_left == -1 && counter.snapshot.delta_right == 1);
    assert(!encoder_counter_sample(&counter, 11U + UINT32_C(0x80000000),
                                   31U + UINT32_C(0x80000000)));
    encoder_counter_invalidate(&counter);
    assert(counter.snapshot.delta_left == 0 && !counter.snapshot.valid);
}

static void saturation_and_max_delta(void) {
    encoder_counter_t counter;
    encoder_counter_init(&counter, true, false, 0U, 0U);
    counter.snapshot.total_left = INT64_MAX - 2;
    counter.snapshot.total_right = INT64_MIN + 2;
    assert(encoder_counter_sample(&counter, UINT32_MAX - 4U, UINT32_MAX - 4U));
    assert(counter.snapshot.total_left == INT64_MAX);
    assert(counter.snapshot.total_right == INT64_MIN);
    assert(encoder_counter_sample(&counter, UINT32_MAX - 3U, UINT32_MAX - 3U));
    assert(counter.snapshot.total_left == INT64_MAX - 1);
    assert(counter.snapshot.total_right == INT64_MIN + 1);
    encoder_counter_init(&counter, true, false, 0U, 0U);
    assert(encoder_counter_sample(&counter, INT32_MAX, INT32_MAX));
    assert(counter.snapshot.delta_left == -INT32_MAX);
    assert(counter.snapshot.delta_right == INT32_MAX);
    assert(encoder_counter_sample(&counter, 0U, 0U));
    assert(counter.snapshot.delta_left == INT32_MAX);
    assert(counter.snapshot.delta_right == -INT32_MAX);
}

int main(void) {
    independent_contexts_and_polarity();
    invalid_sample_recovery();
    saturation_and_max_delta();
    encoder_counter_init(NULL, false, false, 0U, 0U);
    encoder_counter_invalidate(NULL);
    encoder_counter_get_snapshot(NULL, NULL);
    encoder_counter_reset_totals(NULL);
    assert(!encoder_counter_sample(NULL, 0U, 0U));
    puts("PASS portable encoder counter: contexts, polarity, wrap, ambiguity, saturation");
    return 0;
}
