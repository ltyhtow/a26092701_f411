#include "imu_task.h"

#include "task.h"

#include <string.h>

#define IMU_TASK_STACK_DEPTH_WORDS 384U

static QueueHandle_t imu_sample_queue;
static QueueHandle_t imu_fusion_input_queue;

static void imu_task_entry(void *argument) {
    imu_sample_message_t message;
    imu_sample_t sample;
    uint8_t init_result;
    TickType_t last_wake_time;

    (void)argument;
    while ((init_result = imu_port_init()) != 0U) {
        memset(&message, 0, sizeof(message));
        message.timestamp_ms = (uint32_t)xTaskGetTickCount();
        message.error_code = (uint16_t)init_result;
        imu_port_get_diagnostics(&message.diagnostics);
        (void)xQueueOverwrite(imu_sample_queue, &message);
        (void)xQueueOverwrite(imu_fusion_input_queue, &message);
        vTaskDelay(pdMS_TO_TICKS(500U));
    }

    last_wake_time = xTaskGetTickCount();
    for (;;) {
        memset(&message, 0, sizeof(message));
        message.timestamp_ms = (uint32_t)xTaskGetTickCount();
        imu_port_get_diagnostics(&message.diagnostics);
        if (imu_port_read(&sample) == 0U) {
            message.status_flags = IMU_SAMPLE_STATUS_VALID;
            message.sample = sample;
        } else {
            message.error_code = 1U;
        }
        (void)xQueueOverwrite(imu_sample_queue, &message);
        (void)xQueueOverwrite(imu_fusion_input_queue, &message);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
    }
}

BaseType_t imu_task_start(QueueHandle_t sample_queue,
                          QueueHandle_t fusion_input_queue,
                          TaskHandle_t *task_handle) {
    if (sample_queue == NULL || fusion_input_queue == NULL || task_handle == NULL) {
        return pdFAIL;
    }
    imu_sample_queue = sample_queue;
    imu_fusion_input_queue = fusion_input_queue;
    return xTaskCreate(imu_task_entry, "IMU", IMU_TASK_STACK_DEPTH_WORDS,
                       NULL, 3U, task_handle);
}
