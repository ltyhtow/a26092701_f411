#include "encoder_counter.h"

#include <stddef.h>

static int64_t signed_delta(uint32_t current, uint32_t previous) {
    const uint32_t delta = current - previous;
    return delta <= INT32_MAX ? (int64_t)delta : (int64_t)delta - INT64_C(4294967296);
}

static int64_t add_total(int64_t total, int64_t delta) {
    if (delta > 0 && total > INT64_MAX - delta) return INT64_MAX;
    if (delta < 0 && total < INT64_MIN - delta) return INT64_MIN;
    return total + delta;
}

void encoder_counter_init(encoder_counter_t *counter, bool invert_left,
                          bool invert_right, uint32_t raw_left, uint32_t raw_right) {
    if (counter == NULL) return;
    *counter = (encoder_counter_t){0};
    counter->invert_left = invert_left;
    counter->invert_right = invert_right;
    counter->snapshot.raw_left = raw_left;
    counter->snapshot.raw_right = raw_right;
}

void encoder_counter_invalidate(encoder_counter_t *counter) {
    if (counter == NULL) return;
    counter->snapshot.valid = false;
    counter->snapshot.delta_left = 0;
    counter->snapshot.delta_right = 0;
}

bool encoder_counter_sample(encoder_counter_t *counter,
                            uint32_t raw_left, uint32_t raw_right) {
    if (counter == NULL) return false;
    encoder_counter_snapshot_t *sample = &counter->snapshot;
    int64_t left = signed_delta(raw_left, sample->raw_left);
    int64_t right = signed_delta(raw_right, sample->raw_right);
    encoder_counter_invalidate(counter);
    sample->raw_left = raw_left;
    sample->raw_right = raw_right;
    if (left == INT32_MIN || right == INT32_MIN) return false;
    if (counter->invert_left) left = -left;
    if (counter->invert_right) right = -right;
    sample->delta_left = (int32_t)left;
    sample->delta_right = (int32_t)right;
    sample->total_left = add_total(sample->total_left, left);
    sample->total_right = add_total(sample->total_right, right);
    sample->valid = true;
    return true;
}

void encoder_counter_get_snapshot(const encoder_counter_t *counter,
                                  encoder_counter_snapshot_t *snapshot) {
    if (counter != NULL && snapshot != NULL) *snapshot = counter->snapshot;
}

void encoder_counter_reset_totals(encoder_counter_t *counter) {
    if (counter == NULL) return;
    counter->snapshot.total_left = 0;
    counter->snapshot.total_right = 0;
}
