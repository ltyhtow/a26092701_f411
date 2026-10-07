#ifndef SERIAL_PROTOCOL_ADAPTER_H
#define SERIAL_PROTOCOL_ADAPTER_H

#include "serial_protocol_engine.h"

/* Lifecycle is deliberately restricted to before scheduler startup. Start
 * owns the protocol task/mutex and transport; failed start releases all three.
 * Application callback runs in a short protocol-task critical section and
 * must never block. Replies use the nonblocking protocol send path.
 * Calls while the scheduler runs are rejected: start returns false; stop leaves
 * the running service unchanged. Dynamic runtime teardown is not supported. */
bool serial_service_start(serial_packet_handler_t callback, void *context);
void serial_service_stop(void);

#endif
