#include "wheel_test.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

static void qualify(wheel_test_t *test, uint32_t now) {
    wheel_test_init(test, now);
    assert(!wheel_test_command(test, WHEEL_TEST_START, now));
    wheel_test_step(test, now, true, false, true);
    assert(wheel_test_command(test, WHEEL_TEST_START, now));
    assert(!wheel_test_command(test, WHEEL_TEST_START, now));
}
static void check_full_sequence(uint32_t base) {
    wheel_test_t test;
    qualify(&test, base);
    unsigned observed = 1U << WHEEL_TEST_COUNTDOWN;
    int16_t previous_left = 0, previous_right = 0;
    bool left_reached = false, right_reached = false, both_reached = false, reverse_reached = false;
    unsigned zero_before_reverse = 0;
    for (uint32_t elapsed = 10U; elapsed <= 21000U; elapsed += 10U) {
        const uint32_t now = base + elapsed;
        if (elapsed % 200U == 0U) assert(wheel_test_command(&test, WHEEL_TEST_HEARTBEAT, now));
        wheel_test_step(&test, now, true, false, true);
        const wheel_test_snapshot_t *s = &test.snapshot;
        observed |= 1U << s->phase;
        assert(s->test_elapsed_ms == elapsed && s->remaining_ms == 21000U - elapsed);
        if (elapsed < 21000U) {
            assert(s->left_pwm - previous_left <= 50 && s->left_pwm - previous_left >= -50);
            assert(s->right_pwm - previous_right <= 50 && s->right_pwm - previous_right >= -50);
        }
        if (elapsed < 5000U) assert(s->phase == WHEEL_TEST_COUNTDOWN && s->left_pwm == 0 && s->right_pwm == 0);
        else if (elapsed < 8000U) {
            assert(s->phase == WHEEL_TEST_LEFT_FORWARD && s->right_pwm == 0);
            if (s->left_pwm == 1200) left_reached = true;
        } else if (elapsed < 9000U) assert(s->phase == WHEEL_TEST_REST_LEFT);
        else if (elapsed < 12000U) {
            assert(s->phase == WHEEL_TEST_RIGHT_FORWARD && s->left_pwm == 0);
            if (s->right_pwm == 1200) right_reached = true;
        } else if (elapsed < 13000U) assert(s->phase == WHEEL_TEST_REST_RIGHT);
        else if (elapsed < 16000U) {
            assert(s->phase == WHEEL_TEST_BOTH_FORWARD);
            if (s->left_pwm == 1500 && s->right_pwm == 1500) both_reached = true;
        } else if (elapsed < 18000U) {
            assert(s->phase == WHEEL_TEST_REST_FORWARD);
            if (s->left_pwm == 0 && s->right_pwm == 0) ++zero_before_reverse;
        } else if (elapsed < 21000U) {
            assert(s->phase == WHEEL_TEST_BOTH_REVERSE);
            assert(zero_before_reverse >= 100U);
            if (s->left_pwm == -1500 && s->right_pwm == -1500) reverse_reached = true;
        } else assert(s->phase == WHEEL_TEST_DONE && s->left_pwm == 0 && s->right_pwm == 0);
        assert(s->left_pwm >= -1500 && s->left_pwm <= 1500);
        assert(s->right_pwm >= -1500 && s->right_pwm <= 1500);
        previous_left = s->left_pwm; previous_right = s->right_pwm;
    }
    assert(observed == 0x3FEU && left_reached && right_reached && both_reached && reverse_reached);
    assert(!wheel_test_command(&test, WHEEL_TEST_START, base + 22000U));
    wheel_test_step(&test, base + 23000U, false, true, false);
    assert(test.snapshot.phase == WHEEL_TEST_DONE && test.snapshot.reason == WHEEL_TEST_REASON_NONE);
}
static void check_faults(void) {
    wheel_test_t test;
    wheel_test_init(&test, 0U);
    wheel_test_step(&test, 0U, true, false, false);
    assert(!wheel_test_command(&test, WHEEL_TEST_START, 0U));
    assert(!wheel_test_command(&test, WHEEL_TEST_HEARTBEAT, 0U));
    assert(!wheel_test_command(&test, 99U, 0U));
    assert(test.snapshot.phase == WHEEL_TEST_WAIT && test.snapshot.remaining_ms == 21000U);
    assert(wheel_test_command(&test, WHEEL_TEST_STOP, 0U));
    assert(!wheel_test_command(&test, WHEEL_TEST_START, 0U));
    qualify(&test, 0U);
    for (uint32_t ms = 10; ms < 750; ms += 10) wheel_test_step(&test, ms, true, false, true);
    wheel_test_step(&test, 750U, true, false, true);
    assert(test.snapshot.reason == WHEEL_TEST_REASON_LINK_TIMEOUT);
    qualify(&test, 0U);
    assert(!wheel_test_command(&test, WHEEL_TEST_HEARTBEAT, 750U));
    assert(test.snapshot.reason == WHEEL_TEST_REASON_LINK_TIMEOUT);
    qualify(&test, 0U);
    wheel_test_step(&test, 101U, true, false, true);
    assert(test.snapshot.reason == WHEEL_TEST_REASON_DEADLINE);
    qualify(&test, 0U);
    assert(!wheel_test_command(&test, WHEEL_TEST_HEARTBEAT, 101U));
    assert(test.snapshot.reason == WHEEL_TEST_REASON_DEADLINE);
    qualify(&test, 0U);
    wheel_test_step(&test, 100U, true, false, true);
    assert(test.snapshot.phase == WHEEL_TEST_COUNTDOWN);
    wheel_test_step(&test, 110U, true, false, false);
    assert(test.snapshot.reason == WHEEL_TEST_REASON_SENSOR_ERROR);
    assert(!wheel_test_command(&test, WHEEL_TEST_START, 110U));
    qualify(&test, 0U);
    wheel_test_step(&test, 10U, true, true, true);
    assert(test.snapshot.reason == WHEEL_TEST_REASON_DRIVER_FAULT);
    wheel_test_init(&test, 0U);
    wheel_test_step(&test, 0U, false, false, false);
    assert(test.snapshot.reason == WHEEL_TEST_REASON_SENSOR_ERROR);
    qualify(&test, 0U);
    assert(wheel_test_command(&test, WHEEL_TEST_STOP, 10U));
    assert(test.snapshot.reason == WHEEL_TEST_REASON_USER_STOP);
    const uint32_t elapsed_when_stopped = test.snapshot.test_elapsed_ms;
    wheel_test_step(&test, 20U, false, true, false);
    assert(test.snapshot.reason == WHEEL_TEST_REASON_USER_STOP);
    assert(test.snapshot.test_elapsed_ms == elapsed_when_stopped);
    assert(test.snapshot.left_pwm == 0 && test.snapshot.right_pwm == 0);
}
int main(void) {
    check_full_sequence(0U);
    check_full_sequence(UINT32_MAX - 10000U);
    check_faults();
    puts("PASS wheel sequence, limits, dead time, latches and clock wrap");
    return 0;
}
