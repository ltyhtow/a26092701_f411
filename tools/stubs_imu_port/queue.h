#ifndef TEST_IMU_PORT_QUEUE_H
#define TEST_IMU_PORT_QUEUE_H
#include "FreeRTOS.h"
typedef void *QueueHandle_t;
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item);
#endif
