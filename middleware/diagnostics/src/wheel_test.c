#include "wheel_test.h"
#include <string.h>

static const uint32_t boundaries[] = {5000U, 8000U, 9000U, 12000U, 13000U, 16000U, 18000U, 21000U};

wheel_test_phase_t wheel_test_phase_at_elapsed(uint32_t elapsed_ms) {
    unsigned stage = 0U;
    while (stage < 8U && elapsed_ms >= boundaries[stage]) ++stage;
    return (wheel_test_phase_t)(stage + 1U);
}

static void update_time(wheel_test_t *test, uint32_t now) {
    wheel_test_snapshot_t *s = &test->snapshot;
    s->timestamp_ms = now;
    s->heartbeat_age_ms = test->heartbeat_seen ? now - test->heartbeat_ms : 0U;
    if (!wheel_test_terminal(test)) s->test_elapsed_ms = test->started ? now - test->start_ms : 0U;
    if (s->test_elapsed_ms > WHEEL_TEST_DURATION_MS) s->test_elapsed_ms = WHEEL_TEST_DURATION_MS;
    s->remaining_ms = wheel_test_terminal(test) ? 0U : WHEEL_TEST_DURATION_MS - s->test_elapsed_ms;
}

bool wheel_test_terminal(const wheel_test_t *test) {
    return test->snapshot.phase == WHEEL_TEST_DONE || test->snapshot.phase == WHEEL_TEST_ABORTED;
}

void wheel_test_init(wheel_test_t *test, uint32_t now) {
    memset(test, 0, sizeof(*test));
    test->previous_ms = now;
    test->snapshot.timestamp_ms = now;
    test->snapshot.remaining_ms = WHEEL_TEST_DURATION_MS;
}

void wheel_test_abort(wheel_test_t *test, wheel_test_reason_t reason, uint32_t now) {
    if (!wheel_test_terminal(test)) {
        update_time(test, now);
        test->snapshot.phase = WHEEL_TEST_ABORTED;
        test->snapshot.reason = reason;
        test->snapshot.phase_elapsed_ms = 0U;
    }
    test->snapshot.left_pwm = test->snapshot.right_pwm = 0;
    update_time(test, now);
}

bool wheel_test_command(wheel_test_t *test, uint8_t action, uint32_t now) {
    if (action == WHEEL_TEST_STOP) {
        wheel_test_abort(test, WHEEL_TEST_REASON_USER_STOP, now);
        return true;
    }
    if (wheel_test_terminal(test)) return false;
    /* Late heartbeats cannot erase an already expired output lease. */
    if (test->started && now - test->heartbeat_ms >= WHEEL_TEST_HEARTBEAT_MS) {
        wheel_test_abort(test, WHEEL_TEST_REASON_LINK_TIMEOUT, now);
        return false;
    }
    if (test->started && now - test->previous_ms > WHEEL_TEST_DEADLINE_MS) {
        wheel_test_abort(test, WHEEL_TEST_REASON_DEADLINE, now);
        return false;
    }
    if (action == WHEEL_TEST_START) {
        const uint16_t required = WHEEL_TEST_FLAG_ENCODER_READY | WHEEL_TEST_FLAG_SAMPLE_VALID;
        if (test->snapshot.phase != WHEEL_TEST_WAIT ||
            (test->snapshot.flags & required) != required ||
            (test->snapshot.flags & WHEEL_TEST_FLAG_MOTOR_LATCHED) != 0U ||
            now - test->previous_ms > WHEEL_TEST_DEADLINE_MS) return false;
        test->started = true;
        test->start_ms = now;
        test->snapshot.phase = WHEEL_TEST_COUNTDOWN;
        test->snapshot.phase_elapsed_ms = 0U;
    } else if (action != WHEEL_TEST_HEARTBEAT || !test->started) return false;
    test->heartbeat_seen = true;
    test->heartbeat_ms = now;
    test->snapshot.flags |= WHEEL_TEST_FLAG_HEARTBEAT_SEEN;
    update_time(test, now);
    return true;
}

static int16_t approach(int16_t output, int16_t target, uint32_t interval) {
    const int32_t maximum_step = (int32_t)(interval * 5U);
    int32_t delta = (int32_t)target - output;
    if (delta > maximum_step) delta = maximum_step;
    if (delta < -maximum_step) delta = -maximum_step;
    return (int16_t)(output + delta);
}

void wheel_test_step(wheel_test_t *test, uint32_t now, bool encoder_ready,
                     bool motor_fault, bool sample_valid) {
    wheel_test_snapshot_t *s = &test->snapshot;
    const uint32_t interval = now - test->previous_ms;
    test->previous_ms = now;
    s->flags = (encoder_ready ? WHEEL_TEST_FLAG_ENCODER_READY : 0U) |
               (motor_fault ? WHEEL_TEST_FLAG_MOTOR_LATCHED : 0U) |
               (sample_valid ? WHEEL_TEST_FLAG_SAMPLE_VALID : 0U) |
               (test->heartbeat_seen ? WHEEL_TEST_FLAG_HEARTBEAT_SEEN : 0U);
    /* Preserve the first terminal reason, even after the motor latch follows it. */
    if (wheel_test_terminal(test)) { update_time(test, now); return; }
    if (motor_fault) { wheel_test_abort(test, WHEEL_TEST_REASON_DRIVER_FAULT, now); return; }
    if (!encoder_ready || (test->started && !sample_valid)) {
        wheel_test_abort(test, WHEEL_TEST_REASON_SENSOR_ERROR, now); return;
    }
    if (interval > WHEEL_TEST_DEADLINE_MS) {
        wheel_test_abort(test, WHEEL_TEST_REASON_DEADLINE, now); return;
    }
    if (test->started && now - test->heartbeat_ms >= WHEEL_TEST_HEARTBEAT_MS) {
        wheel_test_abort(test, WHEEL_TEST_REASON_LINK_TIMEOUT, now); return;
    }
    update_time(test, now);
    if (!test->started) return;
    const uint32_t elapsed = now - test->start_ms;
    const unsigned stage = (unsigned)wheel_test_phase_at_elapsed(elapsed) - 1U;
    if (stage == 8U) {
        s->phase = WHEEL_TEST_DONE;
        s->phase_elapsed_ms = 0U;
        s->left_pwm = s->right_pwm = 0;
        s->remaining_ms = 0U;
        return;
    }
    s->phase = (wheel_test_phase_t)(stage + 1U);
    s->phase_elapsed_ms = elapsed - (stage == 0U ? 0U : boundaries[stage - 1U]);
    int16_t left = 0, right = 0;
    if (s->phase == WHEEL_TEST_LEFT_FORWARD) left = 1200;
    if (s->phase == WHEEL_TEST_RIGHT_FORWARD) right = 1200;
    if (s->phase == WHEEL_TEST_BOTH_FORWARD) left = right = 1500;
    if (s->phase == WHEEL_TEST_BOTH_REVERSE) left = right = -1500;
    s->left_pwm = approach(s->left_pwm, left, interval);
    s->right_pwm = approach(s->right_pwm, right, interval);
}
