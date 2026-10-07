#ifndef WHEEL_TEST_TASK_H
#define WHEEL_TEST_TASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "wheel_test.h"

BaseType_t wheel_test_task_start(TaskHandle_t *task_handle);
/* Task context only; STOP immediately latches the motor off until reset. */
bool wheel_test_task_command(uint8_t action);
void wheel_test_task_get_snapshot(wheel_test_snapshot_t *snapshot);
/* One owner task calls this every 10 ms; public for deterministic host checks. */
void wheel_test_task_run_cycle(void);
#endif
