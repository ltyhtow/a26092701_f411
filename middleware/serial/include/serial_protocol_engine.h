#ifndef SERIAL_PROTOCOL_ENGINE_H
#define SERIAL_PROTOCOL_ENGINE_H

#include "serial_codec.h"
#include "lwpkt/lwpkt.h"

/* Application callbacks must return promptly. False records a queue overrun. */
typedef bool (*serial_packet_handler_t)(uint32_t command,
                                        const serial_message_t *message, void *context);

#define SERIAL_ENGINE_RX_BUDGET 128U
#define SERIAL_ENGINE_PACKET_BUDGET 8U

typedef struct {
    lwpkt_t packet;
    lwrb_t staged_rx;
    uint8_t staged_storage[SERIAL_ENGINE_RX_BUDGET + 1U];
    lwrb_t *input_rx;
    serial_packet_handler_t on_packet;
    void *context;
    serial_protocol_stats_t stats;
    bool ready;
} serial_protocol_engine_t;

/* No HAL, RTOS or global singleton. Caller supplies rings, time and locking. */
bool serial_engine_init(serial_protocol_engine_t *engine, lwrb_t *tx, lwrb_t *rx,
                        serial_packet_handler_t on_packet, void *context);
void serial_engine_deinit(serial_protocol_engine_t *engine);
/* Drop partial/staged receive packets at a transport session boundary.
 * The caller owns synchronization and clearing its transport input ring.
 * Pending TX, handlers, configuration and diagnostics are preserved.
 */
void serial_engine_reset_rx(serial_protocol_engine_t *engine);
/* Each call consumes at most RX_BUDGET new bytes and processes at most
 * PACKET_BUDGET results. Returns true if buffered work remains. */
bool serial_engine_poll(serial_protocol_engine_t *engine, uint32_t now_ms);
serial_protocol_result_t serial_engine_send(serial_protocol_engine_t *engine,
                                            uint32_t command, const void *native,
                                            size_t native_size);

#endif
