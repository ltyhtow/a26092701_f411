#include "serial_protocol_engine.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t rx_data[1024], tx_data[256], source_data[1024], unused_data[8];
static lwrb_t rx, tx, source, unused;
static lwpkt_t sender;
static serial_protocol_engine_t engine;
static unsigned callbacks;
static bool reject_callback;
static system_command_t last_system;
static parameter_command_t last_parameter;
static command_response_t last_response;

static bool packet_callback(uint32_t command, const serial_message_t *message, void *context) {
    assert(context == &callbacks);
    if (command == SERIAL_CMD_SYSTEM) last_system = message->system;
    else if (command == SERIAL_CMD_PARAMETER) last_parameter = message->parameter;
    else {
        assert(command == SERIAL_CMD_RESPONSE || command == SERIAL_CMD_PID_ACK);
        last_response = message->response;
    }
    ++callbacks;
    return !reject_callback;
}

static void initialize(void) {
    callbacks = 0; reject_callback = false;
    assert(lwrb_init(&rx, rx_data, sizeof(rx_data)));
    assert(lwrb_init(&tx, tx_data, sizeof(tx_data)));
    assert(lwrb_init(&source, source_data, sizeof(source_data)));
    assert(lwrb_init(&unused, unused_data, sizeof(unused_data)));
    assert(lwpkt_init(&sender, &source, &unused) == lwpktOK);
    assert(serial_engine_init(&engine, &tx, &rx, packet_callback, &callbacks));
}

static size_t create_frame(uint32_t command, const uint8_t *payload, size_t size, uint8_t *frame) {
    assert(lwpkt_write(&sender, command, payload, size) == lwpktOK);
    return lwrb_read(&source, frame, 128);
}

static void feed(const uint8_t *frame, size_t length) {
    assert(lwrb_write(&rx, frame, length) == length);
}

int main(void) {
    uint8_t frame[128], valid[128];
    const uint8_t payload[] = {1,7,2,0};
    initialize();
    assert(!serial_engine_init(NULL, &tx, &rx, NULL, NULL));
    assert(!serial_engine_init(&engine, &tx, &tx, NULL, NULL));
    const size_t length = create_frame(SERIAL_CMD_SYSTEM, payload, sizeof(payload), valid);
    assert(length == 9);
    /* Split every byte separately through the real parser. */
    for (size_t i = 0; i < length; ++i) {
        feed(valid + i, 1);
        assert(!serial_engine_poll(&engine, (uint32_t)i));
        assert(callbacks == (i + 1U == length ? 1U : 0U));
    }
    assert(last_system.action == 2 && last_system.sequence == 7);

    memcpy(frame, valid, length); frame[length - 2U] ^= 1U;
    feed(frame, length); feed(valid, length);
    assert(!serial_engine_poll(&engine, 10));
    assert(callbacks == 2 && engine.stats.rx_crc_errors == 1);
    memcpy(frame, valid, length); frame[length - 1U] = 0;
    feed(frame, length); feed(valid, length);
    assert(!serial_engine_poll(&engine, 11));
    assert(callbacks == 3 && engine.stats.rx_stop_errors == 1);
    const uint8_t oversized[] = {0xAA, 3, 65};
    feed(oversized, sizeof(oversized)); feed(valid, length);
    assert(!serial_engine_poll(&engine, 12));
    assert(callbacks == 4 && engine.stats.rx_memory_errors == 1);

    size_t n = create_frame(SERIAL_CMD_PID_ACK, payload, sizeof(payload), frame);
    feed(frame, n);
    n = create_frame(SERIAL_CMD_RESPONSE, payload, sizeof(payload), frame); feed(frame, n);
    n = create_frame(0xFE, payload, sizeof(payload), frame); feed(frame, n);
    uint8_t bad_payload[] = {1,7,2,1};
    n = create_frame(SERIAL_CMD_SYSTEM, bad_payload, sizeof(bad_payload), frame); feed(frame, n);
    n = create_frame(SERIAL_CMD_SYSTEM, payload, 3, frame); feed(frame, n);
    assert(!serial_engine_poll(&engine, 13));
    assert(callbacks == 4 && engine.stats.unknown_commands == 1 && engine.stats.rx_invalid_payloads == 4);
    reject_callback = true; feed(valid, length);
    assert(!serial_engine_poll(&engine, 14));
    assert(callbacks == 5 && engine.stats.queue_overruns == 1);

    /* Truncated frame times out and the next full frame resynchronizes. */
    feed(valid, 4); assert(!serial_engine_poll(&engine, 15));
    assert(!serial_engine_poll(&engine, 115));
    assert(engine.stats.rx_timeouts == 1);
    feed(valid, length); assert(!serial_engine_poll(&engine, 116));
    assert(callbacks == 6);

    initialize();
    /* Startup at a nonzero clock must not instantly expire the first fragment. */
    feed(valid, 4);
    assert(!serial_engine_poll(&engine, 100000));
    feed(valid + 4, length - 4);
    assert(!serial_engine_poll(&engine, 100005));
    assert(callbacks == 1 && engine.stats.rx_timeouts == 0);

    /* Session changes drop parser prefixes and staged packets, while keeping
     * pending TX, counters and the application callback intact. */
    initialize();
    feed(valid, 4);
    assert(!serial_engine_poll(&engine, 1));
    engine.stats.rx_crc_errors = 7;
    const system_command_t queued_reply = {1, 7, 2, 0};
    assert(serial_engine_send(&engine, SERIAL_CMD_SYSTEM, &queued_reply, sizeof(queued_reply)) == SERIAL_PROTOCOL_OK);
    assert(lwrb_write(&engine.staged_rx, valid, length) == length);
    serial_engine_reset_rx(&engine);
    serial_engine_reset_rx(NULL);
    assert(lwrb_get_full(&engine.staged_rx) == 0 && lwrb_get_full(&tx) == length);
    feed(valid + 4, length - 4);
    assert(!serial_engine_poll(&engine, 2));
    assert(callbacks == 0 && engine.stats.rx_crc_errors == 7);
    feed(valid, length);
    assert(!serial_engine_poll(&engine, 3));
    assert(callbacks == 1 && last_system.sequence == 7);

    initialize();
    for (unsigned i = 0; i < 20U; ++i) feed(valid, length);
    assert(serial_engine_poll(&engine, 1));
    assert(callbacks == SERIAL_ENGINE_PACKET_BUDGET);
    assert(lwrb_get_full(&rx) == 20U * length - SERIAL_ENGINE_RX_BUDGET);
    unsigned iterations = 0;
    while (serial_engine_poll(&engine, 2)) assert(++iterations < 8U);
    assert(callbacks == 20);
    /* Noise is bounded by bytes too, independently of packet boundaries. */
    uint8_t noise[500]; memset(noise, 0xFE, sizeof(noise)); feed(noise, sizeof(noise));
    assert(serial_engine_poll(&engine, 3));
    assert(lwrb_get_full(&rx) == sizeof(noise) - SERIAL_ENGINE_RX_BUDGET);

    initialize();
    const system_command_t command = {1,7,2,0xAA};
    assert(serial_engine_send(&engine, SERIAL_CMD_SYSTEM, &command, sizeof(command)) == SERIAL_PROTOCOL_OK);
    n = lwrb_read(&tx, frame, sizeof(frame));
    assert(n == length && memcmp(frame, valid, n) == 0);
    assert(serial_engine_send(&engine, SERIAL_CMD_RESPONSE, &command, sizeof(command)) == SERIAL_PROTOCOL_INVALID_ARGUMENT);
    assert(serial_engine_send(&engine, SERIAL_CMD_SYSTEM, NULL, sizeof(command)) == SERIAL_PROTOCOL_INVALID_ARGUMENT);
    unsigned successful = 0;
    while (serial_engine_send(&engine, SERIAL_CMD_SYSTEM, &command, sizeof(command)) == SERIAL_PROTOCOL_OK)
        assert(++successful < 30U);
    const lwrb_sz_t before = lwrb_get_full(&tx);
    assert(serial_engine_send(&engine, SERIAL_CMD_SYSTEM, &command, sizeof(command)) == SERIAL_PROTOCOL_TX_FULL);
    assert(lwrb_get_full(&tx) == before); /* no partial frame writes */
    serial_engine_deinit(&engine);
    assert(!serial_engine_poll(&engine, 1000));
    assert(serial_engine_send(&engine, SERIAL_CMD_SYSTEM, &command, sizeof(command)) == SERIAL_PROTOCOL_NOT_READY);
    initialize();
    const parameter_command_t parameter = {1, 23, PARAMETER_OP_SET, 1, -65536};
    assert(serial_engine_send(&engine, SERIAL_CMD_PARAMETER, &parameter, sizeof(parameter)) == SERIAL_PROTOCOL_OK);
    n = lwrb_read(&tx, frame, sizeof(frame));
    assert(n == 13U && frame[0] == 0xAA && frame[n - 1U] == 0x55);
    feed(frame, n); assert(!serial_engine_poll(&engine, 10U));
    assert(last_parameter.sequence == 23U && last_parameter.value_q16_16 == -65536);
    const command_response_t response = {.version = 1, .sequence = 23,
        .request_command = SERIAL_CMD_PARAMETER, .status = SERIAL_RESPONSE_UNSAFE,
        .key = 1, .controller_state = 3, .fault_flags = 0x20, .config_revision = 42, .timestamp_ms = 10};
    for (uint32_t cmd = SERIAL_CMD_PID_ACK; cmd <= SERIAL_CMD_RESPONSE; ++cmd) {
        assert(serial_engine_send(&engine, cmd, &response, sizeof(response)) == SERIAL_PROTOCOL_OK);
        n = lwrb_read(&tx, frame, sizeof(frame));
        assert(n == 29U && frame[0] == 0xAA && frame[n - 1U] == 0x55);
        feed(frame, n); assert(!serial_engine_poll(&engine, 11U));
        assert(last_response.sequence == 23U && last_response.status == SERIAL_RESPONSE_UNSAFE);
        assert(last_response.config_revision == 42U && last_response.fault_flags == 0x20U);
    }
    puts("PASS: actual LwPKT CRC/stop/timeout resync, typed dispatch, byte/packet budgets and atomic TX capacity");
    return 0;
}
