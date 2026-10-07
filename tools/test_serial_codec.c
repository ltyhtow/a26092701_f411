#include "serial_codec.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static void golden(uint32_t command, const void *native, size_t size,
                   const uint8_t *expected, size_t length) {
    uint8_t wire[80], again[80];
    serial_message_t decoded;
    size_t written = 999U;
    memset(wire, 0xA5, sizeof(wire));
    assert(serial_codec_encode(command, native, size, wire, sizeof(wire), &written));
    assert(written == length && serial_codec_wire_size(command) == length);
    assert(memcmp(wire, expected, length) == 0 && wire[length] == 0xA5);
    memset(&decoded, 0xCC, sizeof(decoded));
    assert(serial_codec_decode(command, wire, length, &decoded, sizeof(decoded)));
    assert(serial_codec_encode(command, &decoded, size, again, sizeof(again), &written));
    assert(memcmp(again, expected, length) == 0);
    assert(!serial_codec_decode(command, wire, length - 1U, &decoded, sizeof(decoded)));
    assert(!serial_codec_decode(command, wire, length + 1U, &decoded, sizeof(decoded)));
    assert(!serial_codec_decode(command, wire, length, &decoded, size - 1U));
    assert(!serial_codec_encode(command, native, size, wire, length - 1U, &written));
    assert(written == 0U);
    assert(!serial_codec_encode(command, native, size - 1U, wire, sizeof(wire), &written));
    wire[0] = 2U;
    assert(!serial_codec_decode(command, wire, length, &decoded, sizeof(decoded)));
}

static void reserved(uint32_t command, const void *native, size_t size,
                     const unsigned *positions, size_t count) {
    uint8_t wire[64]; size_t length;
    serial_message_t decoded, untouched;
    assert(serial_codec_encode(command, native, size, wire, sizeof(wire), &length));
    memset(&decoded, 0xA5, sizeof(decoded)); untouched = decoded;
    for (size_t i = 0; i < count; ++i) {
        assert(wire[positions[i]] == 0U);
        wire[positions[i]] = 1U;
        assert(!serial_codec_decode(command, wire, length, &decoded, sizeof(decoded)));
        assert(memcmp(&decoded, &untouched, sizeof(decoded)) == 0);
        wire[positions[i]] = 0U;
    }
}

int main(void) {
    const motion_command_t motion = {1, 7, {0xAA, 0x55}, INT32_MIN, INT32_MAX, 0x1234, 3};
    const uint8_t motion_wire[] = {1,7,0,0, 0,0,0,0x80, 0xFF,0xFF,0xFF,0x7F, 0x34,0x12,3,0};
    golden(SERIAL_CMD_MOTION, &motion, sizeof(motion), motion_wire, sizeof(motion_wire));
    const unsigned motion_reserved[] = {2,3};
    reserved(SERIAL_CMD_MOTION, &motion, sizeof(motion), motion_reserved, 2);

    const pid_config_command_t pid = {1,7,2,0xAA, 0x04030201, -2, INT32_MIN, INT32_MAX};
    const uint8_t pid_wire[] = {1,7,2,0, 1,2,3,4, 0xFE,0xFF,0xFF,0xFF,
                                0,0,0,0x80, 0xFF,0xFF,0xFF,0x7F};
    golden(SERIAL_CMD_PID_CONFIG, &pid, sizeof(pid), pid_wire, sizeof(pid_wire));
    const unsigned single_reserved[] = {3};
    reserved(SERIAL_CMD_PID_CONFIG, &pid, sizeof(pid), single_reserved, 1);

    const system_command_t system = {1,7,4,0xAA};
    const uint8_t system_wire[] = {1,7,4,0};
    golden(SERIAL_CMD_SYSTEM, &system, sizeof(system), system_wire, sizeof(system_wire));
    reserved(SERIAL_CMD_SYSTEM, &system, sizeof(system), single_reserved, 1);

    const parameter_command_t parameter = {1, 7, PARAMETER_OP_SET, 14, INT32_MIN};
    const uint8_t parameter_wire[] = {1,7,2,14,0,0,0,0x80};
    golden(SERIAL_CMD_PARAMETER, &parameter, sizeof(parameter), parameter_wire, sizeof(parameter_wire));
    /* Unknown operation/key values remain decodable for an explicit response. */
    const parameter_command_t unknown = {1, 8, 255, 255, 0};
    const uint8_t unknown_wire[] = {1,8,255,255,0,0,0,0};
    golden(SERIAL_CMD_PARAMETER, &unknown, sizeof(unknown), unknown_wire, sizeof(unknown_wire));
    const command_response_t response = {1,7,4,4,14,3,0x1234,INT32_MIN,0x04030201,0x08070605,0x88776655};
    const uint8_t response_wire[] = {1,7,4,4,14,3,0x34,0x12,0,0,0,0x80,
        1,2,3,4,5,6,7,8,0x55,0x66,0x77,0x88};
    golden(SERIAL_CMD_RESPONSE, &response, sizeof(response), response_wire, sizeof(response_wire));
    golden(SERIAL_CMD_PID_ACK, &response, sizeof(response), response_wire, sizeof(response_wire));

    const motion_telemetry_t telemetry = {1,7,0x1234, 0x04030201,-2,INT32_MIN,INT32_MAX,
                                         INT16_MIN,INT16_MAX,0x5678,0x9ABC,0x88776655};
    const uint8_t telemetry_wire[] = {1,7,0x34,0x12, 1,2,3,4, 0xFE,0xFF,0xFF,0xFF,
        0,0,0,0x80, 0xFF,0xFF,0xFF,0x7F, 0,0x80,0xFF,0x7F, 0x78,0x56,0xBC,0x9A, 0x55,0x66,0x77,0x88};
    golden(SERIAL_CMD_TELEMETRY, &telemetry, sizeof(telemetry), telemetry_wire, sizeof(telemetry_wire));

    const imu_telemetry_t status = {1,7,0x1234,{INT16_MIN,-2,INT16_MAX},{1,0x1234,-1},0x5678,0xAAAA,0x88776655};
    const uint8_t status_wire[] = {1,7,0x34,0x12, 0,0x80,0xFE,0xFF,0xFF,0x7F,
                                   1,0,0x34,0x12,0xFF,0xFF,0x78,0x56,0,0,0x55,0x66,0x77,0x88};
    golden(SERIAL_CMD_IMU_STATUS, &status, sizeof(status), status_wire, sizeof(status_wire));
    const unsigned status_reserved[] = {18,19};
    reserved(SERIAL_CMD_IMU_STATUS, &status, sizeof(status), status_reserved, 2);

    imu_diagnostic_t diagnostic;
    memset(&diagnostic, 0xA5, sizeof(diagnostic)); /* Includes native ABI padding. */
    diagnostic.version = 1; diagnostic.sequence = 7; diagnostic.address_8bit = 0xD0;
    diagnostic.who_am_i = 0x68; diagnostic.init_result = 0x1234;
    diagnostic.hal_status = 0x04030201; diagnostic.hal_error_codes = 0x08070605;
    diagnostic.timestamp_ms = 0x88776655;
    const uint8_t diagnostic_wire[] = {1,7,0xD0,0x68,0x34,0x12,0,0,1,2,3,4,5,6,7,8,
                                      0,0,0,0,0x55,0x66,0x77,0x88};
    golden(SERIAL_CMD_IMU_DIAGNOSTIC, &diagnostic, sizeof(diagnostic), diagnostic_wire, sizeof(diagnostic_wire));
    const unsigned diagnostic_reserved[] = {6,7,16,17,18,19};
    reserved(SERIAL_CMD_IMU_DIAGNOSTIC, &diagnostic, sizeof(diagnostic), diagnostic_reserved, 6);

    const imu_attitude_telemetry_t attitude = {1,7,0x1234,0x5678,0xAAAA,
        1,-2,INT32_MIN,INT32_MAX,0x04030201,0,0x40000000,-1,0x08070605,0x0C0B0A09,0x88776655};
    const uint8_t attitude_wire[] = {1,7,0x34,0x12,0x78,0x56,0,0,
        1,0,0,0,0xFE,0xFF,0xFF,0xFF,0,0,0,0x80,0xFF,0xFF,0xFF,0x7F,
        1,2,3,4,0,0,0,0,0,0,0,0x40,0xFF,0xFF,0xFF,0xFF,
        5,6,7,8,9,10,11,12,0x55,0x66,0x77,0x88};
    golden(SERIAL_CMD_IMU_ATTITUDE, &attitude, sizeof(attitude), attitude_wire, sizeof(attitude_wire));
    const unsigned attitude_reserved[] = {6,7};
    reserved(SERIAL_CMD_IMU_ATTITUDE, &attitude, sizeof(attitude), attitude_reserved, 2);

    const encoder_test_telemetry_t encoder = {1,7,0x1234,0x88776655,50,0x55AA55AA,0x04030201,
        INT32_MIN,INT32_MAX,9,INT64_MIN,INT64_MAX,2,3};
    const uint8_t encoder_wire[] = {1,7,0x34,0x12,0x55,0x66,0x77,0x88,50,0,0,0,
        0xAA,0x55,0xAA,0x55,1,2,3,4,0,0,0,0x80,0xFF,0xFF,0xFF,0x7F,9,0,0,0,
        0,0,0,0,0,0,0,0x80,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,2,0,0,0,3,0,0,0};
    golden(SERIAL_CMD_ENCODER_TEST, &encoder, sizeof(encoder), encoder_wire, sizeof(encoder_wire));
    encoder_test_telemetry_t parsed;
    assert(serial_codec_decode(SERIAL_CMD_ENCODER_TEST, encoder_wire, sizeof(encoder_wire), &parsed, sizeof(parsed)));
    assert(parsed.total_left == INT64_MIN && parsed.total_right == INT64_MAX);
    assert(parsed.delta_left == INT32_MIN && parsed.delta_right == INT32_MAX);

    uint8_t scratch[64]; size_t length;
    serial_message_t output;
    for (unsigned command = 0; command < 256U; ++command) {
        if (serial_codec_wire_size(command) != 0U) continue;
        assert(serial_codec_native_size(command) == 0U);
        assert(!serial_codec_encode(command, &system, sizeof(system), scratch, sizeof(scratch), &length));
        assert(!serial_codec_decode(command, system_wire, sizeof(system_wire), &output, sizeof(output)));
    }
    assert(!serial_codec_encode(SERIAL_CMD_SYSTEM, NULL, sizeof(system), scratch, sizeof(scratch), &length));
    assert(!serial_codec_decode(SERIAL_CMD_SYSTEM, NULL, 4, &output, sizeof(output)));
    assert(!serial_codec_decode(SERIAL_CMD_SYSTEM, system_wire, 4, NULL, sizeof(output)));
    system_command_t invalid_system = system; invalid_system.action = 0;
    assert(!serial_codec_encode(SERIAL_CMD_SYSTEM, &invalid_system, sizeof(invalid_system), scratch, sizeof(scratch), &length));
    invalid_system.action = 5;
    assert(!serial_codec_encode(SERIAL_CMD_SYSTEM, &invalid_system, sizeof(invalid_system), scratch, sizeof(scratch), &length));
    memcpy(scratch, system_wire, 4); scratch[2] = 5;
    assert(!serial_codec_decode(SERIAL_CMD_SYSTEM, scratch, 4, &output, sizeof(output)));
    memcpy(scratch, pid_wire, 20); scratch[2] = 3;
    assert(!serial_codec_decode(SERIAL_CMD_PID_CONFIG, scratch, 20, &output, sizeof(output)));
    memcpy(scratch, motion_wire, 16); scratch[14] = 4;
    assert(!serial_codec_decode(SERIAL_CMD_MOTION, scratch, 16, &output, sizeof(output)));
    puts("PASS: fixed LE legacy/parameter/result golden layouts, reserved/padding zeroes, signed limits and invalid payloads");
    return 0;
}
