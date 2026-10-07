/* Real codec -> LwPKT -> LwRB interoperability fixture. No hardware access. */
#include "serial_codec.h"
#include "lwpkt/lwpkt.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t tx_bytes[256], rx_bytes[256], spare_bytes[256];
static lwrb_t tx, rx, spare;
static lwpkt_t sender, receiver;
static uint32_t now_ms;

static void roundtrip(FILE *file, uint32_t command, const void *native, size_t native_size,
                      const uint8_t *golden, size_t golden_size) {
    uint8_t payload[SERIAL_CODEC_MAX_WIRE_SIZE], encoded_again[SERIAL_CODEC_MAX_WIRE_SIZE], frame[128];
    size_t payload_size, second_size;
    assert(serial_codec_encode(command, native, native_size, payload, sizeof(payload), &payload_size));
    assert(payload_size == golden_size && memcmp(payload, golden, golden_size) == 0);
    assert(lwpkt_write(&sender, command, payload, payload_size) == lwpktOK);
    const size_t frame_size = lwrb_read(&tx, frame, sizeof(frame));
    assert(frame_size == golden_size + 5U);
    assert(frame[0] == 0xAA && frame[1] == command && frame[2] == golden_size);
    assert(frame[frame_size - 1U] == 0x55);
    for (size_t i = 0; i < frame_size; ++i) {
        assert(lwrb_write(&rx, frame + i, 1U) == 1U);
        const lwpktr_t result = lwpkt_process(&receiver, ++now_ms);
        assert((result == lwpktVALID) == (i + 1U == frame_size));
    }
    assert(lwpkt_get_cmd(&receiver) == command);
    assert(lwpkt_get_data_len(&receiver) == golden_size);
    serial_message_t decoded;
    assert(serial_codec_decode(command, lwpkt_get_data(&receiver), lwpkt_get_data_len(&receiver),
                               &decoded, sizeof(decoded)));
    assert(serial_codec_encode(command, &decoded, native_size, encoded_again,
                               sizeof(encoded_again), &second_size));
    assert(second_size == golden_size && memcmp(encoded_again, golden, golden_size) == 0);
    assert(fwrite(frame, 1U, frame_size, file) == frame_size);
}

int main(int argc, char **argv) {
    assert(argc == 2);
    assert(lwrb_init(&tx, tx_bytes, sizeof(tx_bytes)));
    assert(lwrb_init(&rx, rx_bytes, sizeof(rx_bytes)));
    assert(lwrb_init(&spare, spare_bytes, sizeof(spare_bytes)));
    assert(lwpkt_init(&sender, &tx, &spare) == lwpktOK);
    assert(lwpkt_init(&receiver, &spare, &rx) == lwpktOK);
    FILE *file = fopen(argv[1], "wb");
    assert(file != NULL);
    for (uint8_t action = 1; action <= 3; ++action) {
        const wheel_test_command_t command = {
            .version = 1, .sequence = (uint8_t)(action + 6U), .action = action, .reserved = 0xAA
        };
        const uint8_t golden[] = {1, (uint8_t)(action + 6U), action, 0};
        roundtrip(file, SERIAL_CMD_WHEEL_CONTROL, &command, sizeof(command), golden, sizeof(golden));
    }
    const wheel_test_telemetry_t status = {
        .version = 1, .sequence = 10, .phase = 8, .reason = 0,
        .timestamp_ms = UINT32_C(0x88776655), .phase_elapsed_ms = 2000,
        .left_pwm = -1500, .right_pwm = -1200, .heartbeat_age_ms = 200,
        .flags = 13, .reserved = 0xFFFF, .remaining_ms = 1000, .test_elapsed_ms = 20000
    };
    const uint8_t status_golden[] = {
        1,10,8,0, 0x55,0x66,0x77,0x88, 0xD0,0x07,0,0,
        0x24,0xFA,0x50,0xFB, 0xC8,0,0,0, 13,0,0,0,
        0xE8,0x03,0,0, 0x20,0x4E,0,0
    };
    roundtrip(file, SERIAL_CMD_WHEEL_TEST, &status, sizeof(status), status_golden, sizeof(status_golden));
    const wheel_test_telemetry_t done = {
        .version = 1, .sequence = 11, .phase = 9, .reason = 0,
        .timestamp_ms = UINT32_C(0x88776A3D), .phase_elapsed_ms = 0,
        .left_pwm = 0, .right_pwm = 0, .heartbeat_age_ms = 200,
        .flags = 15, .remaining_ms = 0, .test_elapsed_ms = 21000
    };
    const uint8_t done_golden[] = {
        1,11,9,0, 0x3D,0x6A,0x77,0x88, 0,0,0,0,
        0,0,0,0, 0xC8,0,0,0, 15,0,0,0,
        0,0,0,0, 0x08,0x52,0,0
    };
    roundtrip(file, SERIAL_CMD_WHEEL_TEST, &done, sizeof(done), done_golden, sizeof(done_golden));
    assert(fclose(file) == 0);

    serial_message_t decoded, original;
    memset(&decoded, 0xA5, sizeof(decoded)); original = decoded;
    uint8_t invalid_control[] = {1,7,1,1};
    assert(!serial_codec_decode(SERIAL_CMD_WHEEL_CONTROL, invalid_control, sizeof(invalid_control),
                                &decoded, sizeof(decoded)));
    assert(memcmp(&decoded, &original, sizeof(decoded)) == 0);
    invalid_control[3] = 0; invalid_control[2] = 0;
    assert(!serial_codec_decode(SERIAL_CMD_WHEEL_CONTROL, invalid_control, sizeof(invalid_control),
                                &decoded, sizeof(decoded)));
    invalid_control[2] = 4;
    assert(!serial_codec_decode(SERIAL_CMD_WHEEL_CONTROL, invalid_control, sizeof(invalid_control),
                                &decoded, sizeof(decoded)));
    uint8_t invalid_status[sizeof(status_golden)];
    memcpy(invalid_status, status_golden, sizeof(invalid_status));
    invalid_status[22] = 1;
    assert(!serial_codec_decode(SERIAL_CMD_WHEEL_TEST, invalid_status, sizeof(invalid_status),
                                &decoded, sizeof(decoded)));
    assert(memcmp(&decoded, &original, sizeof(decoded)) == 0);
    assert(!serial_codec_decode(SERIAL_CMD_WHEEL_TEST, status_golden, sizeof(status_golden) - 1U,
                                &decoded, sizeof(decoded)));
    puts("PASS: WheelTest LE golden payloads, real LwPKT/LwRB fragmented roundtrips and 101-byte fixture");
    return 0;
}
