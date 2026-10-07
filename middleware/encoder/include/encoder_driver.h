/**
 * @file encoder_driver.h
 * @brief Platform-independent dual quadrature encoder acquisition contract.
 */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "encoder_counter.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start both hardware encoder counters.
 * @return True only if both counters started; on failure both are stopped.
 * @note Single owner: initialize before task start; call sampling/totals APIs only
 *       from that task, or provide external locking (64-bit totals are not atomic).
 */
bool encoder_driver_init(void);

/** Whether both hardware counters were successfully started. */
bool encoder_driver_is_ready(void);

typedef encoder_counter_snapshot_t encoder_driver_snapshot_t;

/** Read board-configured direction; valid even before acquisition starts. */
void encoder_driver_get_polarity(bool *invert_left, bool *invert_right);

/* Read after read_speed in the same owner task; does not consume counts. */
void encoder_driver_get_snapshot(encoder_driver_snapshot_t *snapshot);

/**
 * @brief Read encoder speed delta feedback (called periodically by balance control task).
 * @param[out] left_speed  Left X4 quadrature counts since the previous read.
 * @param[out] right_speed Right X4 quadrature counts since the previous read.
 * @note Units are counts/sample (normally 5 ms), NOT counts/s, RPM or m/s.
 *       Caller must sample at its configured period and reject missed deadlines.
 *       Call init explicitly before reading. Startup failure or an ambiguous
 *       half-counter-range delta returns NAN for
 *       both outputs; consumers must treat this as invalid feedback, not zero speed.
 */
void encoder_driver_read_speed(float *left_speed, float *right_speed);

/**
 * @brief Get cumulative total encoder counts (for odometry / distance estimation).
 *        Totals saturate at INT64 limits rather than overflowing signed arithmetic.
 * @param[out] left_total  Cumulative pulse count of left wheel.
 * @param[out] right_total Cumulative pulse count of right wheel.
 */
void encoder_driver_get_totals(int64_t *left_total, int64_t *right_total);

/**
 * @brief Reset cumulative total encoder counts to zero.
 *        Does not change the speed sampling baseline.
 */
void encoder_driver_reset_totals(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_DRIVER_H */
