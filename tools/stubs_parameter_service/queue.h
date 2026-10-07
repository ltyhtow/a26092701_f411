#ifndef TEST_PARAMETER_QUEUE_H
#define TEST_PARAMETER_QUEUE_H
#include "FreeRTOS.h"
typedef struct parameter_test_queue *QueueHandle_t;
QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size);
void vQueueDelete(QueueHandle_t queue);
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t wait);
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item);
BaseType_t xQueueReset(QueueHandle_t queue);
#endif
