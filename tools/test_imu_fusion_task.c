#include "imu_fusion_task.h"
#include "app_clock.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool fail_allocation, fail_creation;
static unsigned allocated, freed, created, deleted, critical_depth;
static BaseType_t scheduler;
static int task_token, input_token, output_token;
void test_enter_critical(void) { ++critical_depth; }
void test_exit_critical(void) { assert(critical_depth > 0); --critical_depth; }
BaseType_t xTaskGetSchedulerState(void) { return scheduler; }
void *pvPortMalloc(size_t size) {
    if (fail_allocation) return NULL;
    assert(size == imu_fusion_context_size());
    void *pointer = malloc(size); assert(pointer != NULL); ++allocated; return pointer;
}
void vPortFree(void *pointer) { assert(pointer != NULL); ++freed; free(pointer); }
BaseType_t xTaskCreate(void (*task)(void *), const char *name, uint32_t stack,
                       void *argument, UBaseType_t priority, TaskHandle_t *handle) {
    assert(task != NULL && strcmp(name, "IMUFusion") == 0 && stack >= 512U);
    assert(argument == NULL && priority == 3U);
    if (fail_creation) return pdFAIL;
    *handle = &task_token; ++created; return pdPASS;
}
void vTaskDelete(TaskHandle_t handle) { assert(handle == &task_token); ++deleted; }
uint32_t app_clock_now_ms(void) { return 1234U; }
TickType_t app_clock_period_ticks(uint32_t milliseconds) { return milliseconds; }
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t timeout) {
    (void)queue; (void)item; (void)timeout; assert(false); return pdFALSE;
}
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item) {
    (void)queue; (void)item; assert(false); return pdFALSE;
}

int main(void) {
    TaskHandle_t task = NULL;
    assert(!imu_fusion_task_request_calibration());
    assert(imu_fusion_task_start(NULL, &output_token, &task) == pdFAIL);
    assert(imu_fusion_task_start(&input_token, NULL, &task) == pdFAIL);
    assert(imu_fusion_task_start(&input_token, &output_token, NULL) == pdFAIL);
    fail_allocation = true;
    assert(imu_fusion_task_start(&input_token, &output_token, &task) == pdFAIL);
    assert(allocated == 0 && task == NULL && !imu_fusion_task_request_calibration());
    fail_allocation = false; fail_creation = true;
    assert(imu_fusion_task_start(&input_token, &output_token, &task) == pdFAIL);
    assert(allocated == 1 && freed == 1 && created == 0 && task == NULL);
    fail_creation = false;
    assert(imu_fusion_task_start(&input_token, &output_token, &task) == pdPASS);
    assert(task == &task_token && imu_fusion_task_request_calibration());
    assert(imu_fusion_task_start(&input_token, &output_token, &task) == pdFAIL);
    scheduler = taskSCHEDULER_RUNNING;
    imu_fusion_task_stop();
    assert(deleted == 0 && freed == 1 && imu_fusion_task_request_calibration());
    scheduler = taskSCHEDULER_NOT_STARTED;
    imu_fusion_task_stop(); imu_fusion_task_stop();
    assert(deleted == created && freed == allocated && !imu_fusion_task_request_calibration());
    scheduler = taskSCHEDULER_RUNNING;
    assert(imu_fusion_task_start(&input_token, &output_token, &task) == pdFAIL);
    scheduler = taskSCHEDULER_NOT_STARTED;
    assert(imu_fusion_task_start(&input_token, &output_token, &task) == pdPASS);
    imu_fusion_task_stop();
    assert(deleted == created && freed == allocated && critical_depth == 0);
    puts("PASS: Fusion task owns opaque context and task, rolls back failures and supports pre-scheduler retry");
    return 0;
}
