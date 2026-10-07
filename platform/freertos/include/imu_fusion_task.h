#ifndef IMU_FUSION_TASK_H
#define IMU_FUSION_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "imu_fusion.h"
#include "imu_task.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t imu_fusion_task_start(QueueHandle_t sample_queue,
                                 QueueHandle_t attitude_queue,
                                 TaskHandle_t *task_handle);

/* Start/stop are supported only before scheduler startup. Stop owns deletion
 * of the task and opaque filter storage; callers must not vTaskDelete it. Calls
 * during scheduler operation leave the running service unchanged. */
void imu_fusion_task_stop(void);

/* Task-context request. The fusion task owns and resets its filter state. */
bool imu_fusion_task_request_calibration(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FUSION_TASK_H */
