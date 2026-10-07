#ifndef PARAMETER_SERVICE_H
#define PARAMETER_SERVICE_H
#include "balance_controller.h"
#include "serial_protocol.h"
#include <stdbool.h>
/* Before scheduler, after balance_task_start; stops/locks outputs through boot load. */
bool parameter_service_start(const balance_controller_config_t *board_defaults);
void parameter_service_stop(void);
/* Protocol callback: GET/STATUS are immediate; modifying jobs are copied to a worker. */
bool parameter_service_submit(const parameter_command_t *command, void *context);
#endif
