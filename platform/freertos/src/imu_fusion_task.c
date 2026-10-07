#include "imu_fusion_task.h"
#include "app_clock.h"

#include <string.h>

/* GCC 14 -O0: the AHRS update chain needs 2992 bytes before callees and
 * the Cortex-M4F exception/context frame.  A 2048-byte stack corrupts the
 * adjacent IMU task.  Keep this budget checked against compiler .su output.
 */
#define IMU_FUSION_TASK_STACK_DEPTH_WORDS 1024U

static QueueHandle_t fusion_sample_queue;
static QueueHandle_t fusion_attitude_queue;
static volatile bool calibration_requested;
static bool fusion_task_started;
static imu_fusion_t *fusion_context;
static TaskHandle_t fusion_task_handle;

bool imu_fusion_task_request_calibration(void) {
    taskENTER_CRITICAL();
    const bool ready = fusion_task_started;
    if (ready) {
        calibration_requested = true;
    }
    taskEXIT_CRITICAL();
    return ready;
}

static void imu_fusion_task_entry(void *argument) {
    imu_sample_message_t sample_message;
    imu_fusion_output_t output;
    imu_fusion_t *fusion = fusion_context;

    (void)argument;
    imu_fusion_init(fusion);

    for (;;) {
        taskENTER_CRITICAL();
        const bool reset_filter = calibration_requested;
        calibration_requested = false;
        taskEXIT_CRITICAL();
        if (reset_filter) {
            imu_fusion_init(fusion);
            memset(&output, 0, sizeof(output));
            output.timestamp_ms = app_clock_now_ms();
            output.status_flags = IMU_FUSION_STATUS_CALIBRATING;
            (void)xQueueOverwrite(fusion_attitude_queue, &output);
        }
        if (xQueueReceive(fusion_sample_queue, &sample_message,
                          app_clock_period_ticks(IMU_SAMPLE_PERIOD_MS)) != pdTRUE) {
            continue;
        }

        memset(&output, 0, sizeof(output));
        output.timestamp_ms = sample_message.timestamp_ms;
        if ((sample_message.status_flags & IMU_SAMPLE_STATUS_VALID) == 0U) {
            output.status_flags = IMU_FUSION_STATUS_INPUT_INVALID;
            (void)xQueueOverwrite(fusion_attitude_queue, &output);
            continue;
        }

        (void)imu_fusion_update(fusion, &sample_message.sample,
                                sample_message.timestamp_ms, &output);
        (void)xQueueOverwrite(fusion_attitude_queue, &output);
    }
}

BaseType_t imu_fusion_task_start(QueueHandle_t sample_queue,
                                 QueueHandle_t attitude_queue,
                                 TaskHandle_t *task_handle) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED ||
        sample_queue == NULL || attitude_queue == NULL || task_handle == NULL || fusion_task_started) {
        return pdFAIL;
    }
    *task_handle = NULL;
    fusion_sample_queue = sample_queue;
    fusion_attitude_queue = attitude_queue;
    fusion_context = pvPortMalloc(imu_fusion_context_size());
    if (fusion_context == NULL) {
        imu_fusion_task_stop();
        return pdFAIL;
    }
    calibration_requested = false;
    const BaseType_t result = xTaskCreate(imu_fusion_task_entry, "IMUFusion",
                                         IMU_FUSION_TASK_STACK_DEPTH_WORDS, NULL, 3U,
                                         &fusion_task_handle);
    fusion_task_started = (result == pdPASS);
    if (result != pdPASS) {
        imu_fusion_task_stop();
    } else {
        *task_handle = fusion_task_handle;
    }
    return result;
}

void imu_fusion_task_stop(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) return;
    fusion_task_started = false;
    calibration_requested = false;
    if (fusion_task_handle != NULL) {
        vTaskDelete(fusion_task_handle);
        fusion_task_handle = NULL;
    }
    if (fusion_context != NULL) {
        vPortFree(fusion_context);
        fusion_context = NULL;
    }
    fusion_sample_queue = NULL;
    fusion_attitude_queue = NULL;
}
