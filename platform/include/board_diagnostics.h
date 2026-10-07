#ifndef BOARD_DIAGNOSTICS_H
#define BOARD_DIAGNOSTICS_H

#include <stdbool.h>
#include <stdint.h>

/* Portable diagnostic IO contract, implemented by the selected board backend. */
/* Bits 0..3: left A, left B, right A, right B instantaneous input levels. */
uint32_t board_encoder_levels(void);

/* Start and stop are serialized against the fatal motor latch. A fault always
 * dominates a diagnostic start; direct mode uses the board's full-DC test. */
bool board_motor_test_start(bool direct_gpio);
void board_motor_test_stop(bool direct_gpio);

#endif
