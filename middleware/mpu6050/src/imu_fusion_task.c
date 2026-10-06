#include "imu_fusion_task.h"

#include <string.h>

#define IMU_FUSION_TASK_STACK_DEPTH_WORDS 512U

static QueueHandle_t fusion_sample_queue;
static QueueHandle_t fusion_attitude_queue;

static void imu_fusion_task_entry(void *argument) {
    imu_sample_message_t sample_message;
    imu_fusion_output_t output;
    static imu_fusion_t fusion;

    (void)argument;
    imu_fusion_init(&fusion);

    for (;;) {
        if (xQueueReceive(fusion_sample_queue, &sample_message,
                          portMAX_DELAY) != pdTRUE) {
            continue;
        }

        memset(&output, 0, sizeof(output));
        output.timestamp_ms = sample_message.timestamp_ms;
        if ((sample_message.status_flags & IMU_SAMPLE_STATUS_VALID) == 0U) {
            output.status_flags = IMU_FUSION_STATUS_INPUT_INVALID;
            (void)xQueueOverwrite(fusion_attitude_queue, &output);
            continue;
        }

        (void)imu_fusion_update(&fusion, &sample_message.sample,
                                sample_message.timestamp_ms, &output);
        (void)xQueueOverwrite(fusion_attitude_queue, &output);
    }
}

BaseType_t imu_fusion_task_start(QueueHandle_t sample_queue,
                                 QueueHandle_t attitude_queue,
                                 TaskHandle_t *task_handle) {
    if (sample_queue == NULL || attitude_queue == NULL || task_handle == NULL) {
        return pdFAIL;
    }
    fusion_sample_queue = sample_queue;
    fusion_attitude_queue = attitude_queue;
    return xTaskCreate(imu_fusion_task_entry, "IMUFusion",
                       IMU_FUSION_TASK_STACK_DEPTH_WORDS, NULL, 3U,
                       task_handle);
}
