#ifndef ENCODER_COUNTER_H
#define ENCODER_COUNTER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t raw_left, raw_right;
    int32_t delta_left, delta_right;
    int64_t total_left, total_right;
    bool valid;
} encoder_counter_snapshot_t;

/* Caller-owned, single-writer state. No hardware, scheduler, or global state. */
typedef struct {
    encoder_counter_snapshot_t snapshot;
    bool invert_left, invert_right;
} encoder_counter_t;

void encoder_counter_init(encoder_counter_t *counter, bool invert_left,
                          bool invert_right, uint32_t raw_left, uint32_t raw_right);
/* Raw counters are modulo 2^32. Exactly half-range movement is ambiguous and
 * invalidates both deltas, advances the baseline, and preserves totals. */
bool encoder_counter_sample(encoder_counter_t *counter,
                            uint32_t raw_left, uint32_t raw_right);
void encoder_counter_invalidate(encoder_counter_t *counter);
void encoder_counter_get_snapshot(const encoder_counter_t *counter,
                                  encoder_counter_snapshot_t *snapshot);
/* Preserve raw baseline and last sample when resetting only odometry. */
void encoder_counter_reset_totals(encoder_counter_t *counter);

#ifdef __cplusplus
}
#endif
#endif
