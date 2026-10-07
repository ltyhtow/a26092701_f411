#include "parameter_profile.h"

#include <math.h>

#define PARAMETER_FIELDS(X) \
    X(PARAMETER_KEY_BALANCE_KP, balance_loop.kp) \
    X(PARAMETER_KEY_BALANCE_KD, balance_loop.kd) \
    X(PARAMETER_KEY_VELOCITY_KP, velocity_loop.kp) \
    X(PARAMETER_KEY_VELOCITY_KI, velocity_loop.ki) \
    X(PARAMETER_KEY_VELOCITY_INTEGRAL_LIMIT, velocity_loop.integral_limit) \
    X(PARAMETER_KEY_VELOCITY_LPF_ALPHA, velocity_loop.lpf_alpha) \
    X(PARAMETER_KEY_VELOCITY_MAX_OUTPUT, velocity_loop.max_output) \
    X(PARAMETER_KEY_TURN_KP, turn_loop.kp) \
    X(PARAMETER_KEY_TURN_KD, turn_loop.kd) \
    X(PARAMETER_KEY_TURN_MAX_OUTPUT, turn_loop.max_output) \
    X(PARAMETER_KEY_MECHANICAL_ZERO, mechanical_zero_pitch) \
    X(PARAMETER_KEY_DEADBAND_LEFT, deadband_left) \
    X(PARAMETER_KEY_DEADBAND_RIGHT, deadband_right) \
    X(PARAMETER_KEY_MAX_PWM, max_pwm)

static bool assign(parameter_profile_t *profile, uint16_t key, float value)
{
    switch (key) {
#define ASSIGN_CASE(id, member) case id: profile->controller.member = value; return true;
        PARAMETER_FIELDS(ASSIGN_CASE)
#undef ASSIGN_CASE
        default: return false;
    }
}

void parameter_profile_defaults(parameter_profile_t *profile,
                                const balance_controller_config_t *board_defaults)
{
    if (profile == NULL) {
        return;
    }
    if (board_defaults != NULL) {
        profile->controller = *board_defaults;
    } else {
        balance_controller_default_config(&profile->controller);
    }
    profile->controller.auto_recovery_enabled = false;
    profile->pwm_ceiling = profile->controller.max_pwm;
}

bool parameter_profile_validate(const parameter_profile_t *profile)
{
    if (profile == NULL || !balance_controller_config_is_valid(&profile->controller)) {
        return false;
    }
    const balance_controller_config_t *c = &profile->controller;
    return isfinite(profile->pwm_ceiling) && profile->pwm_ceiling >= 1.0f &&
        profile->pwm_ceiling <= INT16_MAX &&
        !c->auto_recovery_enabled && c->max_pwm <= profile->pwm_ceiling &&
        c->max_pwm >= 1.0f && fabsf(c->mechanical_zero_pitch) <= 15.0f &&
        fabsf(c->balance_loop.kp) <= 10000.0f && fabsf(c->balance_loop.kd) <= 10000.0f &&
        fabsf(c->velocity_loop.kp) <= 10000.0f && fabsf(c->velocity_loop.ki) <= 10000.0f &&
        c->velocity_loop.integral_limit <= 30000.0f &&
        c->velocity_loop.max_output <= profile->pwm_ceiling &&
        fabsf(c->turn_loop.kp) <= 10000.0f && fabsf(c->turn_loop.kd) <= 10000.0f &&
        c->turn_loop.max_output <= profile->pwm_ceiling;
}

parameter_result_t parameter_profile_get(const parameter_profile_t *profile,
                                         uint16_t key, float *value)
{
    if (profile == NULL || value == NULL) {
        return PARAMETER_BAD_ARGUMENT;
    }
    switch (key) {
#define GET_CASE(id, member) case id: *value = profile->controller.member; return PARAMETER_OK;
        PARAMETER_FIELDS(GET_CASE)
#undef GET_CASE
        default: return PARAMETER_UNKNOWN_KEY;
    }
}

parameter_result_t parameter_profile_set(parameter_profile_t *profile,
                                         uint16_t key, float value)
{
    if (profile == NULL) {
        return PARAMETER_BAD_ARGUMENT;
    }
    parameter_profile_t candidate = *profile;
    if (!assign(&candidate, key, value)) {
        return PARAMETER_UNKNOWN_KEY;
    }
    if (!parameter_profile_validate(&candidate)) {
        return PARAMETER_OUT_OF_RANGE;
    }
    *profile = candidate;
    return PARAMETER_OK;
}

bool parameter_profile_encode(const parameter_profile_t *profile,
                               uint8_t *output, size_t length)
{
    if (output == NULL || length != PARAMETER_PROFILE_ENCODED_SIZE ||
        !parameter_profile_validate(profile)) {
        return false;
    }
    for (uint16_t key = 1; key <= PARAMETER_KEY_COUNT; ++key) {
        float value = 0.0f;
        (void)parameter_profile_get(profile, key, &value);
        const uint32_t bits = (uint32_t)(int32_t)lroundf(value * 65536.0f);
        const size_t offset = (size_t)(key - 1U) * 4U;
        output[offset] = (uint8_t)bits;
        output[offset + 1U] = (uint8_t)(bits >> 8U);
        output[offset + 2U] = (uint8_t)(bits >> 16U);
        output[offset + 3U] = (uint8_t)(bits >> 24U);
    }
    return true;
}

bool parameter_profile_decode(parameter_profile_t *profile,
                               const balance_controller_config_t *board_defaults,
                               const uint8_t *input, size_t length)
{
    if (profile == NULL || input == NULL || length != PARAMETER_PROFILE_ENCODED_SIZE) {
        return false;
    }
    parameter_profile_t candidate;
    parameter_profile_defaults(&candidate, board_defaults);
    for (uint16_t key = 1; key <= PARAMETER_KEY_COUNT; ++key) {
        const size_t offset = (size_t)(key - 1U) * 4U;
        const uint32_t bits = (uint32_t)input[offset] |
            ((uint32_t)input[offset + 1U] << 8U) |
            ((uint32_t)input[offset + 2U] << 16U) |
            ((uint32_t)input[offset + 3U] << 24U);
        /* Avoid implementation-defined unsigned-to-signed conversion. */
        const int64_t signed_value = (bits & UINT32_C(0x80000000)) != 0U ?
            (int64_t)bits - INT64_C(4294967296) : (int64_t)bits;
        (void)assign(&candidate, key, (float)signed_value / 65536.0f);
    }
    if (!parameter_profile_validate(&candidate)) {
        return false;
    }
    *profile = candidate;
    return true;
}
