#ifndef PARAMETER_PROFILE_H
#define PARAMETER_PROFILE_H

#include "balance_controller.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PARAMETER_KEY_COUNT 14U
#define PARAMETER_PROFILE_ENCODED_SIZE (PARAMETER_KEY_COUNT * 4U)

typedef enum {
    PARAMETER_KEY_BALANCE_KP = 1,
    PARAMETER_KEY_BALANCE_KD,
    PARAMETER_KEY_VELOCITY_KP,
    PARAMETER_KEY_VELOCITY_KI,
    PARAMETER_KEY_VELOCITY_INTEGRAL_LIMIT,
    PARAMETER_KEY_VELOCITY_LPF_ALPHA,
    PARAMETER_KEY_VELOCITY_MAX_OUTPUT,
    PARAMETER_KEY_TURN_KP,
    PARAMETER_KEY_TURN_KD,
    PARAMETER_KEY_TURN_MAX_OUTPUT,
    PARAMETER_KEY_MECHANICAL_ZERO,
    PARAMETER_KEY_DEADBAND_LEFT,
    PARAMETER_KEY_DEADBAND_RIGHT,
    PARAMETER_KEY_MAX_PWM
} parameter_key_t;

typedef enum {
    PARAMETER_OK = 0,
    PARAMETER_BAD_ARGUMENT,
    PARAMETER_UNKNOWN_KEY,
    PARAMETER_OUT_OF_RANGE
} parameter_result_t;

typedef struct {
    balance_controller_config_t controller;
    /* Immutable board policy inherited from defaults; not a tunable or stored
     * wire field. Lowering max_pwm does not lower this board ceiling. */
    float pwm_ceiling;
} parameter_profile_t;

/* Copies supplied robot defaults; NULL uses the controller examples.
 * Automatic recovery is always disabled. No runtime state,
 * enable flag or motor command is represented in a parameter profile. */
void parameter_profile_defaults(parameter_profile_t *profile,
                                const balance_controller_config_t *board_defaults);
bool parameter_profile_validate(const parameter_profile_t *profile);
parameter_result_t parameter_profile_get(const parameter_profile_t *profile,
                                         uint16_t key, float *value);
/* Transactional: rejection leaves the entire profile unchanged. */
parameter_result_t parameter_profile_set(parameter_profile_t *profile,
                                         uint16_t key, float value);
/* Stable signed Q16.16 little-endian fields, in key order. Never copies a C
 * structure to the medium. Decode inherits non-tunable safety/topology fields
 * from defaults and validates the entire candidate before publishing it. */
bool parameter_profile_encode(const parameter_profile_t *profile,
                               uint8_t *output, size_t length);
bool parameter_profile_decode(parameter_profile_t *profile,
                               const balance_controller_config_t *board_defaults,
                               const uint8_t *input, size_t length);

#ifdef __cplusplus
}
#endif
#endif
