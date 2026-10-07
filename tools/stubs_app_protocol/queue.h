#ifndef TEST_APP_PROTOCOL_QUEUE_H
#define TEST_APP_PROTOCOL_QUEUE_H
#include "../stubs_balance_task/queue.h"
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait);
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item);
#endif
