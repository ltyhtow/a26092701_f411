#include "parameter_profile.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void equal_tunable(const parameter_profile_t *left, const parameter_profile_t *right)
{
    for (uint16_t key = 1U; key <= PARAMETER_KEY_COUNT; ++key) {
        float a = 0, b = 0;
        assert(parameter_profile_get(left, key, &a) == PARAMETER_OK);
        assert(parameter_profile_get(right, key, &b) == PARAMETER_OK);
        assert(fabsf(a - b) <= 1.0f / 65536.0f);
    }
}

int main(void)
{
    balance_controller_config_t board_defaults;
    balance_controller_default_config(&board_defaults);
    board_defaults.max_pwm = 4500.0f;
    parameter_profile_t profile;
    parameter_profile_defaults(&profile, &board_defaults);
    assert(parameter_profile_validate(&profile));
    assert(profile.controller.velocity_loop.kp == 0.0f);
    assert(profile.controller.velocity_loop.ki == 0.0f);
    assert(profile.controller.max_pwm == 4500.0f);
    assert(!profile.controller.auto_recovery_enabled);
    assert(parameter_profile_set(&profile, PARAMETER_KEY_MECHANICAL_ZERO, -1.5f) == PARAMETER_OK);

    uint8_t bytes[PARAMETER_PROFILE_ENCODED_SIZE];
    assert(parameter_profile_encode(&profile, bytes, sizeof(bytes)));
    assert(bytes[0] == 0x00 && bytes[1] == 0x00 && bytes[2] == 0xB4 && bytes[3] == 0x00);
    assert(bytes[40] == 0x00 && bytes[41] == 0x80 && bytes[42] == 0xFE && bytes[43] == 0xFF);
    parameter_profile_t decoded;
    assert(parameter_profile_decode(&decoded, &board_defaults, bytes, sizeof(bytes)));
    equal_tunable(&profile, &decoded);

    const struct { uint16_t key; float value; } invalid[] = {
        {PARAMETER_KEY_BALANCE_KP, NAN}, {PARAMETER_KEY_BALANCE_KD, INFINITY},
        {PARAMETER_KEY_BALANCE_KP, 10001.0f}, {PARAMETER_KEY_VELOCITY_KI, -10001.0f},
        {PARAMETER_KEY_VELOCITY_INTEGRAL_LIMIT, -1.0f},
        {PARAMETER_KEY_VELOCITY_INTEGRAL_LIMIT, 30001.0f},
        {PARAMETER_KEY_VELOCITY_LPF_ALPHA, -0.1f}, {PARAMETER_KEY_VELOCITY_LPF_ALPHA, 1.1f},
        {PARAMETER_KEY_VELOCITY_MAX_OUTPUT, 4501.0f}, {PARAMETER_KEY_TURN_MAX_OUTPUT, -1.0f},
        {PARAMETER_KEY_MECHANICAL_ZERO, 15.01f}, {PARAMETER_KEY_MECHANICAL_ZERO, -15.01f},
        {PARAMETER_KEY_DEADBAND_LEFT, 4501.0f}, {PARAMETER_KEY_DEADBAND_RIGHT, -1.0f},
        {PARAMETER_KEY_MAX_PWM, 4501.0f}, {PARAMETER_KEY_MAX_PWM, 0.0f},
        {PARAMETER_KEY_MAX_PWM, 100.0f}
    };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        parameter_profile_t before = profile;
        assert(parameter_profile_set(&profile, invalid[i].key, invalid[i].value) == PARAMETER_OUT_OF_RANGE);
        assert(memcmp(&before, &profile, sizeof(profile)) == 0);
    }
    assert(parameter_profile_set(&profile, 0U, 1.0f) == PARAMETER_UNKNOWN_KEY);
    assert(parameter_profile_set(&profile, 15U, 1.0f) == PARAMETER_UNKNOWN_KEY);
    assert(parameter_profile_set(NULL, 1U, 0.0f) == PARAMETER_BAD_ARGUMENT);
    float unchanged = 42.0f;
    assert(parameter_profile_get(&profile, 99U, &unchanged) == PARAMETER_UNKNOWN_KEY);
    assert(unchanged == 42.0f);
    assert(parameter_profile_get(&profile, 1U, NULL) == PARAMETER_BAD_ARGUMENT);
    assert(!parameter_profile_validate(NULL));
    assert(!parameter_profile_encode(&profile, bytes, sizeof(bytes) - 1U));
    assert(!parameter_profile_decode(&decoded, &board_defaults, bytes, sizeof(bytes) - 1U));

    assert(parameter_profile_set(&profile, PARAMETER_KEY_BALANCE_KP, -10000.0f) == PARAMETER_OK);
    assert(parameter_profile_set(&profile, PARAMETER_KEY_VELOCITY_INTEGRAL_LIMIT, 30000.0f) == PARAMETER_OK);
    assert(parameter_profile_encode(&profile, bytes, sizeof(bytes)));
    assert(parameter_profile_decode(&decoded, &board_defaults, bytes, sizeof(bytes)));
    equal_tunable(&profile, &decoded);
    const parameter_profile_t before_bad_decode = decoded;
    /* Negative integral limit encoded as signed Q16.16 -1.0. */
    bytes[16] = 0; bytes[17] = 0; bytes[18] = 0xFF; bytes[19] = 0xFF;
    assert(!parameter_profile_decode(&decoded, &board_defaults, bytes, sizeof(bytes)));
    assert(memcmp(&decoded, &before_bad_decode, sizeof(decoded)) == 0);

    parameter_profile_defaults(&profile, &board_defaults);
    profile.controller.auto_recovery_enabled = true;
    assert(!parameter_profile_validate(&profile));
    parameter_profile_defaults(&decoded, &profile.controller);
    assert(!decoded.controller.auto_recovery_enabled);
    profile = decoded;
    profile.controller.velocity_coupling_mode = BALANCE_VEL_COUPLE_TILT_CASCADE;
    profile.controller.velocity_loop.max_output = 5.0f;
    assert(parameter_profile_validate(&profile));
    assert(parameter_profile_set(&profile, PARAMETER_KEY_VELOCITY_MAX_OUTPUT, 35.0f) == PARAMETER_OUT_OF_RANGE);
    /* A changed runtime max_pwm must not redefine the immutable board ceiling. */
    parameter_profile_defaults(&profile, &board_defaults);
    assert(parameter_profile_set(&profile, PARAMETER_KEY_MAX_PWM, 2000.0f) == PARAMETER_OK);
    assert(profile.pwm_ceiling == 4500.0f);
    assert(parameter_profile_encode(&profile, bytes, sizeof(bytes)));
    assert(parameter_profile_decode(&decoded, &board_defaults, bytes, sizeof(bytes)));
    assert(decoded.pwm_ceiling == 4500.0f && decoded.controller.max_pwm == 2000.0f);
    assert(parameter_profile_set(&decoded, PARAMETER_KEY_MAX_PWM, 4500.0f) == PARAMETER_OK);
    board_defaults.max_pwm = 3000.0f;
    parameter_profile_defaults(&profile, &board_defaults);
    assert(parameter_profile_set(&profile, PARAMETER_KEY_MAX_PWM, 3001.0f) == PARAMETER_OUT_OF_RANGE);
    puts("parameter profile: 14 keys, validation, ABI-independent Q16.16 and transactional rejection passed");
    return 0;
}
