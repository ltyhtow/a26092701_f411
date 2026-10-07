#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "imu_config.h"
#include "imu_port.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_SAMPLE_STATUS_VALID 0x0001U

typedef struct {
    uint32_t timestamp_ms;
    uint16_t status_flags; /* bit0: sample valid */
    uint16_t error_code;
    imu_port_diagnostics_t diagnostics;
    imu_sample_t sample;
} imu_sample_message_t;

BaseType_t imu_task_start(QueueHandle_t sample_queue,
                          QueueHandle_t fusion_input_queue,
                          TaskHandle_t *task_handle);

#ifdef __cplusplus
}
#endif

#endif /* IMU_TASK_H */
