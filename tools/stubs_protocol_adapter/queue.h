#ifndef TEST_PROTOCOL_QUEUE_H
#define TEST_PROTOCOL_QUEUE_H
#include "FreeRTOS.h"
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t timeout);
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item);
#endif
