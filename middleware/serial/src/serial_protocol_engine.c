#include "serial_protocol_engine.h"
#include <string.h>

static void on_event(lwpkt_t *packet, lwpkt_evt_type_t event) {
    serial_protocol_engine_t *engine = lwpkt_get_arg(packet);
    if (engine != NULL && event == LWPKT_EVT_TIMEOUT) ++engine->stats.rx_timeouts;
}

bool serial_engine_init(serial_protocol_engine_t *engine, lwrb_t *tx, lwrb_t *rx,
                        serial_packet_handler_t on_packet, void *context) {
    if (engine == NULL || tx == NULL || rx == NULL || tx == rx ||
        !lwrb_is_ready(tx) || !lwrb_is_ready(rx)) return false;
    memset(engine, 0, sizeof(*engine));
    if (!lwrb_init(&engine->staged_rx, engine->staged_storage, sizeof(engine->staged_storage)) ||
        lwpkt_init(&engine->packet, tx, &engine->staged_rx) != lwpktOK) return false;
    engine->input_rx = rx;
    engine->on_packet = on_packet;
    engine->context = context;
    lwpkt_set_arg(&engine->packet, engine);
    if (lwpkt_set_evt_fn(&engine->packet, on_event) != lwpktOK) return false;
    engine->ready = true;
    return true;
}

void serial_engine_deinit(serial_protocol_engine_t *engine) {
    if (engine != NULL) {
        engine->ready = false;
        engine->input_rx = NULL;
        engine->on_packet = NULL;
        engine->context = NULL;
    }
}

void serial_engine_reset_rx(serial_protocol_engine_t *engine) {
    if (engine == NULL || !engine->ready) return;
    lwrb_reset(&engine->staged_rx);
    (void)lwpkt_reset(&engine->packet);
}

bool serial_engine_poll(serial_protocol_engine_t *engine, uint32_t now_ms) {
    if (engine == NULL || !engine->ready) return false;
    /* Never parse the ISR-owned ring directly: a continuous noise stream could
     * otherwise keep lwpkt_read's byte loop running indefinitely. This private
     * bounded staging ring preserves incomplete packets between polls. */
    uint8_t incoming[SERIAL_ENGINE_RX_BUDGET];
    const lwrb_sz_t space = lwrb_get_free(&engine->staged_rx);
    const lwrb_sz_t received = lwrb_read(engine->input_rx, incoming, space);
    if (received != 0U) (void)lwrb_write(&engine->staged_rx, incoming, received);
    for (unsigned i = 0U; i < SERIAL_ENGINE_PACKET_BUDGET; ++i) {
        if (engine->packet.m.state == LWPKT_STATE_START && lwrb_get_full(&engine->staged_rx) != 0U)
            engine->packet.last_rx_time = now_ms;
        const lwpktr_t result = lwpkt_process(&engine->packet, now_ms);
        if (result == lwpktERRCRC) ++engine->stats.rx_crc_errors;
        else if (result == lwpktERRSTOP) ++engine->stats.rx_stop_errors;
        else if (result == lwpktERRMEM) ++engine->stats.rx_memory_errors;
        else if (result == lwpktVALID) {
            const uint32_t command = lwpkt_get_cmd(&engine->packet);
            serial_message_t message;
            if (serial_codec_wire_size(command) == 0U) ++engine->stats.unknown_commands;
            else if (!serial_codec_decode(command, lwpkt_get_data(&engine->packet),
                                           lwpkt_get_data_len(&engine->packet),
                                           &message, sizeof(message))) {
                ++engine->stats.rx_invalid_payloads;
            } else if (engine->on_packet != NULL &&
                       !engine->on_packet(command, &message, engine->context)) {
                ++engine->stats.queue_overruns;
            }
        }
        if (result == lwpktWAITDATA || result == lwpktINPROG || result == lwpktOK) break;
    }
    return lwrb_get_full(engine->input_rx) != 0U || lwrb_get_full(&engine->staged_rx) != 0U;
}

serial_protocol_result_t serial_engine_send(serial_protocol_engine_t *engine,
                                            uint32_t command, const void *native,
                                            size_t native_size) {
    if (engine == NULL || !engine->ready) return SERIAL_PROTOCOL_NOT_READY;
    uint8_t wire[SERIAL_CODEC_MAX_WIRE_SIZE];
    size_t length;
    if (!serial_codec_encode(command, native, native_size, wire, sizeof(wire), &length))
        return SERIAL_PROTOCOL_INVALID_ARGUMENT;
    const lwpktr_t result = lwpkt_write(&engine->packet, command, wire, length);
    if (result == lwpktOK) return SERIAL_PROTOCOL_OK;
    return result == lwpktERRMEM ? SERIAL_PROTOCOL_TX_FULL : SERIAL_PROTOCOL_ERROR;
}
