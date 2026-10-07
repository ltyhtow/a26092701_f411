#ifndef BALANCE_TASK_H
#define BALANCE_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "balance_runtime.h"

#define BALANCE_TASK_PERIOD_MS BALANCE_RUNTIME_PERIOD_MS
#define BALANCE_TASK_PERIOD_S 0.005f
#define BALANCE_TASK_STACK_WORDS 512U
#define BALANCE_TASK_DEFAULT_PRIORITY 3U
#define BALANCE_TASK_ATTITUDE_MAX_AGE_MS BALANCE_RUNTIME_MAX_AGE_MS
#define BALANCE_TASK_CALIBRATION_TIMEOUT_MS BALANCE_RUNTIME_CALIBRATION_TIMEOUT_MS
#define BALANCE_TELEMETRY_DEFAULT_DIVISOR 4U

/* Output must be bounded/nonblocking: the platform calls it with interrupts
 * masked to serialize it with public immediate-stop requests. */
typedef void (*balance_motor_output_fn)(int16_t left_pwm, int16_t right_pwm);
typedef void (*balance_encoder_read_fn)(float *left_counts, float *right_counts);
/* Called outside the critical section; must be nonblocking and copy if retaining. */
typedef void (*balance_telemetry_fn)(const balance_runtime_snapshot_t *snapshot);

typedef enum {
    BALANCE_CONFIG_OK = 0, BALANCE_CONFIG_BUSY = 1, BALANCE_CONFIG_INVALID = 2,
    BALANCE_CONFIG_NOT_READY = 3, BALANCE_CONFIG_UNSAFE = 4
} balance_config_result_t;
/* Completion callbacks run outside the critical section in the control task.
 * They must be bounded/nonblocking, e.g. copying a result into an RTOS queue. */
typedef void (*balance_config_result_fn)(balance_config_result_t result, uint32_t cookie,
                                         uint32_t revision, void *context);
typedef void (*balance_pid_result_fn)(const pid_request_t *request,
                                      balance_config_result_t result, uint32_t revision);
typedef struct {
    QueueHandle_t attitude_queue;       /* attitude_sample_t, required */
    QueueHandle_t pid_config_queue;     /* pid_request_t */
    QueueHandle_t motion_command_queue; /* motion_request_t */
    QueueHandle_t system_command_queue; /* system_request_t */
    balance_motor_output_fn motor_output_hook; /* required, no weak fallback */
    balance_encoder_read_fn encoder_read_hook;
    bool (*calibration_request_hook)(void);
    balance_telemetry_fn telemetry_hook;
    balance_pid_result_fn pid_result_hook;
    const balance_controller_config_t *controller_config; /* copied on start */
    uint32_t priority;
    uint16_t stack_depth_words;
    bool enable_telemetry;
    uint8_t telemetry_divisor;
} balance_task_config_t;

void balance_task_default_config(balance_task_config_t *config);
BaseType_t balance_task_start(const balance_task_config_t *config, TaskHandle_t *task_handle);
BaseType_t balance_task_init_and_start(QueueHandle_t attitude_queue, QueueHandle_t pid_config_queue,
    QueueHandle_t motion_command_queue, balance_motor_output_fn motor_hook, TaskHandle_t *task_handle);
/* A bounded adapter iteration, owned exclusively by the control task. Useful for
 * an alternative scheduler or host adapter verification; never call concurrently. */
void balance_task_run_cycle(void);
void balance_task_set_motor_output_hook(balance_motor_output_fn hook);
void balance_task_set_encoder_read_hook(balance_encoder_read_fn hook);
bool balance_task_request_arm(void);
void balance_task_request_disarm(void);
bool balance_task_reset_fault(void);
void balance_task_emergency_stop(void);
balance_state_t balance_task_get_state(void);
void balance_task_get_controller(balance_controller_t *controller);
void balance_task_get_safety_fsm(safety_fsm_t *fsm);
void balance_task_get_latest_outputs(balance_controller_outputs_t *outputs);
/* Copies into a single pending slot; OK here means queued. Only the completion
 * callback reports application. A slot cannot be overwritten before completion. */
balance_config_result_t balance_task_request_config(const balance_controller_config_t *config,
    uint32_t cookie, balance_config_result_fn callback, void *context);
uint32_t balance_task_get_config_revision(void);
void balance_task_get_config_snapshot(balance_controller_config_t *config, uint32_t *revision);
void balance_task_get_snapshot(balance_runtime_snapshot_t *snapshot);
/* Worker-owned, nonrecursive maintenance gate. Entry requires DISARMED.
 * Full configuration requests remain allowed; arm/legacy PID remain blocked.
 * Exit discards queued system/motion commands and leaves a disarm latch.
 * Complete any pending configuration callback before calling end. */
bool balance_task_begin_maintenance(void);
void balance_task_end_maintenance(void);
bool balance_task_is_maintenance(void);
void balance_task_set_telemetry_enabled(bool enable, uint8_t divisor);

#endif
