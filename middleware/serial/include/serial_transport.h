#ifndef SERIAL_TRANSPORT_H
#define SERIAL_TRANSPORT_H

#include "FreeRTOS.h"
#include "lwrb/lwrb.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t serial_transport_init(TaskHandle_t notify_task);
void serial_transport_deinit(void);
lwrb_t* serial_transport_rx_buffer(void);
lwrb_t* serial_transport_tx_buffer(void);
void serial_transport_poll_tx(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_TRANSPORT_H */
