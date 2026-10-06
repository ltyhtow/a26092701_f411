#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "serial_protocol.h"
#include "imu_task.h"
#include "imu_fusion_task.h"
#include "imu_uart_test_task.h"
#include "motor_polarity_test_task.h"
#include "balance_task.h"
#include "motor_driver.h"
#include "encoder_driver.h"

int32_t app_synctasks_init(void);

extern QueueHandle_t motion_command_queue;
extern QueueHandle_t pid_config_queue;
extern QueueHandle_t system_command_queue;
extern QueueHandle_t imu_sample_queue;
extern QueueHandle_t imu_fusion_input_queue;
extern QueueHandle_t imu_attitude_queue;

#ifdef __cplusplus
}
#endif

#endif /* APP_FREERTOS_H */
