#ifndef TEST_BALANCE_QUEUE_H
#define TEST_BALANCE_QUEUE_H
#include "FreeRTOS.h"
typedef struct test_queue *QueueHandle_t;
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait);
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue);
BaseType_t xQueueReset(QueueHandle_t queue);
#endif
