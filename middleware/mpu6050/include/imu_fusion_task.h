#ifndef IMU_FUSION_TASK_H
#define IMU_FUSION_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "imu_fusion.h"
#include "imu_task.h"

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t imu_fusion_task_start(QueueHandle_t sample_queue,
                                 QueueHandle_t attitude_queue,
                                 TaskHandle_t *task_handle);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FUSION_TASK_H */
