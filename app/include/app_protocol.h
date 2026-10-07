#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H
#include "FreeRTOS.h"
#include "queue.h"
#include "balance_runtime.h"
#include "balance_task.h"
#include "serial_protocol_engine.h"
/* Private application routing context: no exported queue globals. */
typedef struct {
    QueueHandle_t motion, pid, system;
    bool (*parameter_handler)(const parameter_command_t *request, void *context);
    void *parameter_context;
} app_command_queues_t;
bool app_protocol_receive(uint32_t command, const serial_message_t *message, void *context);
void app_protocol_telemetry(const balance_runtime_snapshot_t *snapshot);
void app_protocol_pid_result(const pid_request_t *request, balance_config_result_t result, uint32_t revision);
#endif
