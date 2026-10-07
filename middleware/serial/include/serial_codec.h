#ifndef SERIAL_CODEC_H
#define SERIAL_CODEC_H

#include "serial_protocol.h"
#include <stdbool.h>

#define SERIAL_CODEC_MAX_WIRE_SIZE 56U
#define SERIAL_ENCODER_TEST_WIRE_SIZE 56U

typedef union {
    motion_command_t motion;
    pid_config_command_t pid;
    system_command_t system;
    parameter_command_t parameter;
    command_response_t response;
    motion_telemetry_t telemetry;
    imu_telemetry_t imu_status;
    imu_diagnostic_t imu_diagnostic;
    imu_attitude_telemetry_t imu_attitude;
    encoder_test_telemetry_t encoder;
    wheel_test_command_t wheel_control;
    wheel_test_telemetry_t wheel_test;
} serial_message_t;

/* Zero means unsupported. */
size_t serial_codec_wire_size(uint32_t command);
size_t serial_codec_native_size(uint32_t command);
/* Encode ignores native padding/reserved fields and always emits zeroes there.
 * Decode checks exact lengths, version and all reserved wire bytes. */
bool serial_codec_encode(uint32_t command, const void *native, size_t native_size,
                         uint8_t *wire, size_t capacity, size_t *written);
bool serial_codec_decode(uint32_t command, const uint8_t *wire, size_t length,
                         void *native, size_t native_capacity);

#endif
