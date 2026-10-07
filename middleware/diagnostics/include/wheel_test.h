#ifndef WHEEL_TEST_H
#define WHEEL_TEST_H
#include <stdbool.h>
#include <stdint.h>

#define WHEEL_TEST_PERIOD_MS 10U
#define WHEEL_TEST_SAMPLE_MS 50U
#define WHEEL_TEST_DEADLINE_MS 100U
#define WHEEL_TEST_HEARTBEAT_MS 750U
#define WHEEL_TEST_DURATION_MS 21000U
#define WHEEL_TEST_FLAG_ENCODER_READY 1U
#define WHEEL_TEST_FLAG_MOTOR_LATCHED 2U
#define WHEEL_TEST_FLAG_SAMPLE_VALID 4U
#define WHEEL_TEST_FLAG_HEARTBEAT_SEEN 8U

typedef enum {
    WHEEL_TEST_WAIT = 0, WHEEL_TEST_COUNTDOWN, WHEEL_TEST_LEFT_FORWARD,
    WHEEL_TEST_REST_LEFT, WHEEL_TEST_RIGHT_FORWARD, WHEEL_TEST_REST_RIGHT,
    WHEEL_TEST_BOTH_FORWARD, WHEEL_TEST_REST_FORWARD, WHEEL_TEST_BOTH_REVERSE,
    WHEEL_TEST_DONE, WHEEL_TEST_ABORTED
} wheel_test_phase_t;
typedef enum {
    WHEEL_TEST_REASON_NONE = 0, WHEEL_TEST_REASON_USER_STOP,
    WHEEL_TEST_REASON_LINK_TIMEOUT, WHEEL_TEST_REASON_SENSOR_ERROR,
    WHEEL_TEST_REASON_DRIVER_FAULT, WHEEL_TEST_REASON_DEADLINE
} wheel_test_reason_t;
typedef enum {
    WHEEL_TEST_START = 1, WHEEL_TEST_STOP = 2, WHEEL_TEST_HEARTBEAT = 3
} wheel_test_action_t;
typedef struct {
    wheel_test_phase_t phase;
    wheel_test_reason_t reason;
    uint32_t timestamp_ms, phase_elapsed_ms, heartbeat_age_ms;
    uint32_t remaining_ms, test_elapsed_ms;
    int16_t left_pwm, right_pwm;
    uint16_t flags;
} wheel_test_snapshot_t;
typedef struct {
    wheel_test_snapshot_t snapshot;
    uint32_t start_ms, previous_ms, heartbeat_ms;
    bool started, heartbeat_seen;
} wheel_test_t;

/* Single owner or externally serialized. Unsigned elapsed time supports one
 * 32-bit clock wrap; step must be serviced every 10 ms. No hardware or RTOS. */
void wheel_test_init(wheel_test_t *test, uint32_t now_ms);
void wheel_test_abort(wheel_test_t *test, wheel_test_reason_t reason, uint32_t now_ms);
void wheel_test_step(wheel_test_t *test, uint32_t now_ms, bool encoder_ready,
                     bool motor_fault, bool sample_valid);
bool wheel_test_command(wheel_test_t *test, uint8_t action, uint32_t now_ms);
bool wheel_test_terminal(const wheel_test_t *test);
/* Enables the acquisition owner to close the previous sampling interval before
 * applying a new stage, even when an asynchronous START is off its 50 ms grid. */
wheel_test_phase_t wheel_test_phase_at_elapsed(uint32_t elapsed_ms);
#endif
