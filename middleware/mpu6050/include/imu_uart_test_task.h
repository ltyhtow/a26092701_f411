#ifndef IMU_UART_TEST_TASK_H
#define IMU_UART_TEST_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t imu_uart_test_task_start(QueueHandle_t sample_queue,
                                    QueueHandle_t attitude_queue,
                                    TaskHandle_t *task_handle);

#ifdef __cplusplus
}
#endif

#endif /* IMU_UART_TEST_TASK_H */
