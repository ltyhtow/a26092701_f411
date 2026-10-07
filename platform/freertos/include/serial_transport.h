#ifndef SERIAL_TRANSPORT_H
#define SERIAL_TRANSPORT_H

/* RTOS transport adapter contract; not part of the portable wire protocol. */

#include "FreeRTOS.h"
#include "lwrb/lwrb.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t serial_transport_init(TaskHandle_t notify_task);
void serial_transport_deinit(void);
lwrb_t* serial_transport_rx_buffer(void);
lwrb_t* serial_transport_tx_buffer(void);
void serial_transport_poll_tx(void);
/* Task/mutex context: discard parser state after a transport session change. */
bool serial_transport_take_rx_reset(void);
/* Changes on a USB session boundary; the continuously connected UART uses 0. */
uint32_t serial_transport_link_generation(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_TRANSPORT_H */
