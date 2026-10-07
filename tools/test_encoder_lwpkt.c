#include "serial_codec.h"
#include "lwpkt/lwpkt.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv) {
    assert(argc == 2);
    encoder_test_telemetry_t sample = {
        .version = 1, .sequence = 7, .flags = 0x17,
        .timestamp_ms = 123450, .sample_period_ms = 50,
        .raw_left = 0x55AA55AA, .raw_right = 123456,
        .delta_left = -12, .delta_right = 34, .gpio_levels = 9,
        .total_left = -INT64_C(1234567890123), .total_right = INT64_C(2345678901234),
        .tx_dropped = 2, .sample_errors = 3
    };
    uint8_t send_data[256], receive_data[256], spare_data[256], frame[128];
    lwrb_t send_ring, receive_ring, spare_ring;
    lwpkt_t sender, receiver;
    assert(lwrb_init(&send_ring, send_data, sizeof(send_data)));
    assert(lwrb_init(&receive_ring, receive_data, sizeof(receive_data)));
    assert(lwrb_init(&spare_ring, spare_data, sizeof(spare_data)));
    assert(lwpkt_init(&sender, &send_ring, &spare_ring) == lwpktOK);
    assert(lwpkt_init(&receiver, &spare_ring, &receive_ring) == lwpktOK);
    uint8_t payload[SERIAL_ENCODER_TEST_WIRE_SIZE];
    size_t payload_size;
    assert(serial_codec_encode(SERIAL_CMD_ENCODER_TEST, &sample, sizeof(sample),
                                payload, sizeof(payload), &payload_size));
    assert(lwpkt_write(&sender, SERIAL_CMD_ENCODER_TEST, payload, payload_size) == lwpktOK);
    const size_t length = lwrb_read(&send_ring, frame, sizeof(frame));
    assert(length == 61 && frame[0] == 0xAA && frame[1] == 0x87 && frame[2] == 56 && frame[60] == 0x55);
    /* Feed the actual LwPKT receiver in two chunks: no ad-hoc framing. */
    assert(lwrb_write(&receive_ring, frame, 17) == 17);
    assert(lwpkt_process(&receiver, 1) != lwpktVALID);
    assert(lwrb_write(&receive_ring, frame + 17, length - 17) == length - 17);
    assert(lwpkt_process(&receiver, 2) == lwpktVALID);
    assert(lwpkt_get_cmd(&receiver) == SERIAL_CMD_ENCODER_TEST);
    assert(lwpkt_get_data_len(&receiver) == payload_size);
    assert(memcmp(lwpkt_get_data(&receiver), payload, payload_size) == 0);
    encoder_test_telemetry_t decoded;
    assert(serial_codec_decode(SERIAL_CMD_ENCODER_TEST, lwpkt_get_data(&receiver),
                                lwpkt_get_data_len(&receiver), &decoded, sizeof(decoded)));
    assert(decoded.total_left == sample.total_left && decoded.total_right == sample.total_right);
    assert(decoded.delta_left == -12 && decoded.delta_right == 34);
    FILE *file = fopen(argv[1], "wb");
    assert(file != NULL);
    assert(fwrite(frame, 1, length, file) == length);
    assert(fclose(file) == 0);
    puts("PASS: real LwPKT/LwRB encoder frame round trip and binary fixture");
    return 0;
}
