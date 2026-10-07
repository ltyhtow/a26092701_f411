/* Application routing boundaries; link the real App/src/app_protocol.c. */
#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_protocol.h"
#include "app_config.h"
#if WHEEL_TEST_ENABLED
#include "wheel_test_task.h"
#endif

struct test_queue {
    size_t item_size;
    unsigned writes;
    bool full;
    unsigned char item[64];
};
static struct test_queue motion_queue = {.item_size = sizeof(motion_request_t)};
static struct test_queue pid_queue = {.item_size = sizeof(pid_request_t)};
static struct test_queue system_queue = {.item_size = sizeof(system_request_t)};
static app_command_queues_t queues = {.motion = &motion_queue, .pid = &pid_queue, .system = &system_queue};
static unsigned stop_count, queue_calls, tx_calls, event_count;
static unsigned events[16];
static motion_telemetry_t transmitted;
static command_response_t response;
static uint32_t response_command;
static unsigned response_calls;
static bool maintenance;
static serial_protocol_result_t tx_result = SERIAL_PROTOCOL_BUSY;

uint32_t balance_task_get_config_revision(void) { return 42U; }
bool balance_task_is_maintenance(void) { return maintenance; }
void balance_task_get_snapshot(balance_runtime_snapshot_t *snapshot) {
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->state = BALANCE_STATE_FALLEN;
    snapshot->fault_flags = SAFETY_FAULT_EMERGENCY_STOP;
    snapshot->timestamp_ms = 1234U;
}

void balance_task_request_disarm(void) {
    ++stop_count;
    assert(event_count < 16U);
    events[event_count++] = 1U;
}
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait) {
    assert(wait == 0U && queue != NULL && item != NULL);
    ++queue_calls;
    assert(event_count < 16U);
    events[event_count++] = 2U;
    if (queue->full) return pdFAIL;
    assert(queue->item_size <= sizeof(queue->item));
    memcpy(queue->item, item, queue->item_size);
    ++queue->writes;
    queue->full = true;
    return pdPASS;
}
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item) {
    assert(queue == &motion_queue && item != NULL);
    ++queue_calls;
    assert(event_count < 16U);
    events[event_count++] = 3U;
    memcpy(queue->item, item, queue->item_size);
    ++queue->writes;
    queue->full = true;
    return pdPASS;
}
serial_protocol_result_t serial_protocol_try_send(uint32_t command, const void *data, size_t length) {
    assert(data != NULL);
    if (command == SERIAL_CMD_PID_ACK || command == SERIAL_CMD_RESPONSE) {
        assert(length == sizeof(response));
        memcpy(&response, data, length);
        response_command = command;
        ++response_calls;
        return tx_result;
    }
    assert(command == SERIAL_CMD_TELEMETRY && length == sizeof(transmitted));
    memcpy(&transmitted, data, length);
    ++tx_calls;
    return tx_result;
}
serial_protocol_result_t serial_protocol_send(uint32_t command, const void *data, size_t length) {
    (void)command; (void)data; (void)length;
    assert(!"Telemetry must use nonblocking send");
    return SERIAL_PROTOCOL_ERROR;
}

#if WHEEL_TEST_ENABLED
static unsigned wheel_calls;
static uint8_t wheel_actions[8];
bool wheel_test_task_command(uint8_t action) {
    assert(wheel_calls < sizeof(wheel_actions));
    wheel_actions[wheel_calls++] = action;
    return false; /* Router accepts frame; dedicated status reports outcome. */
}
static void test_wheel_routes(void) {
    serial_message_t message = {0};
    for (uint8_t action = WHEEL_TEST_START; action <= WHEEL_TEST_HEARTBEAT; ++action) {
        message.wheel_control = (wheel_test_command_t){1,7,action,0};
        assert(app_protocol_receive(SERIAL_CMD_WHEEL_CONTROL, &message, &queues));
        assert(wheel_actions[action - 1U] == action);
    }
    message.system = (system_command_t){1,8,SYSTEM_ACTION_DISARM,0};
    assert(app_protocol_receive(SERIAL_CMD_SYSTEM, &message, NULL));
    assert(wheel_actions[3] == WHEEL_TEST_STOP);
    message.motion = (motion_command_t){.version = 1, .flags = 0};
    assert(app_protocol_receive(SERIAL_CMD_MOTION, &message, &queues));
    assert(wheel_actions[4] == WHEEL_TEST_STOP);
    message.motion.flags = 2U; /* A clear-fault flag cannot discard disable. */
    assert(app_protocol_receive(SERIAL_CMD_MOTION, &message, &queues));
    assert(wheel_actions[5] == WHEEL_TEST_STOP);
    message.motion.flags = 1U;
    assert(app_protocol_receive(SERIAL_CMD_MOTION, &message, &queues));
    message.system.action = SYSTEM_ACTION_ARM;
    assert(app_protocol_receive(SERIAL_CMD_SYSTEM, &message, &queues));
    message.system.action = SYSTEM_ACTION_RESET;
    assert(app_protocol_receive(SERIAL_CMD_SYSTEM, &message, &queues));
    message.system.action = SYSTEM_ACTION_CALIBRATE;
    assert(app_protocol_receive(SERIAL_CMD_SYSTEM, &message, &queues));
    assert(app_protocol_receive(SERIAL_CMD_PID_CONFIG, &message, &queues));
    assert(app_protocol_receive(SERIAL_CMD_PARAMETER, &message, &queues));
    assert(app_protocol_receive(SERIAL_CMD_WHEEL_TEST, &message, &queues));
    assert(wheel_calls == 6U && queue_calls == 0U && stop_count == 0U && tx_calls == 0U);
    assert(!app_protocol_receive(SERIAL_CMD_WHEEL_CONTROL, NULL, &queues));
}
#elif !ENCODER_PUSH_TEST_ENABLED && !MOTOR_POLARITY_TEST_ENABLED
static void test_motion_domain(void) {
    serial_message_t message = {0};
    message.motion = (motion_command_t){.version = SERIAL_PROTOCOL_VERSION,
        .linear_q16_16 = 3 * 65536 + 32768, .yaw_q16_16 = -2 * 65536 - 16384,
        .timeout_ms = 275, .flags = 1};
    assert(app_protocol_receive(SERIAL_CMD_MOTION, &message, &queues));
    motion_request_t request;
    memcpy(&request, motion_queue.item, sizeof(request));
    assert(request.linear_counts_per_5ms == 3.5f && request.yaw_rate_dps == -2.25f);
    assert(request.timeout_ms == 275U && request.enable && !request.clear_fault);
    assert(stop_count == 0U && motion_queue.writes == 1U);
    message.motion.flags = 2U;
    event_count = 0U;
    assert(app_protocol_receive(SERIAL_CMD_MOTION, &message, &queues));
    memcpy(&request, motion_queue.item, sizeof(request));
    assert(!request.enable && request.clear_fault);
    assert(stop_count == 1U && motion_queue.writes == 2U);
    assert(event_count == 2U && events[0] == 1U && events[1] == 3U);
}
static void test_pid_and_system_routes(void) {
    serial_message_t message = {0};
    message.pid = (pid_config_command_t){.version = SERIAL_PROTOCOL_VERSION,
        .sequence = 9, .loop_id = 1, .kp_q16_16 = 16384, .ki_q16_16 = 65536,
        .kd_q16_16 = -8192, .integral_limit_q16_16 = 42 * 65536 + 32768};
    event_count = 0U;
    assert(app_protocol_receive(SERIAL_CMD_PID_CONFIG, &message, &queues));
    pid_request_t pid;
    memcpy(&pid, pid_queue.item, sizeof(pid));
    assert(pid.loop_id == 1U && pid.kp == 0.25f && pid.ki == 1.0f);
    assert(pid.kd == -0.125f && pid.integral_limit == 42.5f);
    assert(pid.token == 9U && response_calls == 0U); /* No false queue-accept ACK. */
    assert(!app_protocol_receive(SERIAL_CMD_PID_CONFIG, &message, &queues));
    assert(pid_queue.writes == 1U);
    assert(response_calls == 1U && response_command == SERIAL_CMD_PID_ACK);
    assert(response.sequence == 9U && response.key == 1U && response.status == SERIAL_RESPONSE_BUSY);
    app_protocol_pid_result(&pid, BALANCE_CONFIG_OK, 43U);
    assert(response.status == SERIAL_RESPONSE_OK && response.config_revision == 43U);
    assert(response.controller_state == BALANCE_STATE_FALLEN && response.fault_flags == SAFETY_FAULT_EMERGENCY_STOP);
    message.system = (system_command_t){.version = SERIAL_PROTOCOL_VERSION, .action = SYSTEM_ACTION_ARM};
    assert(app_protocol_receive(SERIAL_CMD_SYSTEM, &message, &queues));
    system_request_t system;
    memcpy(&system, system_queue.item, sizeof(system));
    assert(system.action == SYSTEM_ACTION_ARM);
    const unsigned stops_before = stop_count;
    event_count = 0U;
    message.system.action = SYSTEM_ACTION_DISARM;
    assert(!app_protocol_receive(SERIAL_CMD_SYSTEM, &message, &queues));
    assert(stop_count == stops_before + 1U);
    assert(event_count == 2U && events[0] == 1U && events[1] == 2U);
    system_queue.full = false;
    message.system.action = SYSTEM_ACTION_RESET;
    assert(app_protocol_receive(SERIAL_CMD_SYSTEM, &message, &queues));
    memcpy(&system, system_queue.item, sizeof(system));
    assert(system.action == SYSTEM_ACTION_RESET);
}
static bool parameter_accept;
static parameter_command_t last_parameter;
static bool parameter_handler(const parameter_command_t *request, void *context) {
    assert(context == &parameter_accept);
    last_parameter = *request;
    return parameter_accept;
}
static void test_parameter_routes(void) {
    serial_message_t message = {0};
    message.parameter = (parameter_command_t){1, 99, PARAMETER_OP_SET, 1, 42 * 65536};
    assert(!app_protocol_receive(SERIAL_CMD_PARAMETER, &message, &queues));
    assert(response_command == SERIAL_CMD_RESPONSE && response.sequence == 99U);
    assert(response.status == SERIAL_RESPONSE_NOT_READY && response.request_command == SERIAL_CMD_PARAMETER);
    queues.parameter_handler = parameter_handler;
    queues.parameter_context = &parameter_accept;
    assert(!app_protocol_receive(SERIAL_CMD_PARAMETER, &message, &queues));
    assert(response.status == SERIAL_RESPONSE_BUSY);
    const unsigned count = response_calls;
    parameter_accept = true;
    assert(app_protocol_receive(SERIAL_CMD_PARAMETER, &message, &queues));
    assert(last_parameter.value_q16_16 == 42 * 65536 && last_parameter.sequence == 99);
    assert(response_calls == count); /* Worker, not router, confirms completion. */
}
static void test_tuning_rejections(void) {
    serial_message_t message = {0};
    message.pid = (pid_config_command_t){.version = 1, .sequence = 21, .loop_id = 0,
        .kp_q16_16 = 10001 * 65536};
    const unsigned calls = queue_calls;
    assert(app_protocol_receive(SERIAL_CMD_PID_CONFIG, &message, &queues));
    assert(response.status == SERIAL_RESPONSE_INVALID && response.sequence == 21U);
    assert(queue_calls == calls);
    maintenance = true;
    message.pid.kp_q16_16 = 40 * 65536;
    assert(app_protocol_receive(SERIAL_CMD_PID_CONFIG, &message, &queues));
    assert(response.status == SERIAL_RESPONSE_BUSY);
    message.system = (system_command_t){1,22,SYSTEM_ACTION_ARM,0};
    assert(app_protocol_receive(SERIAL_CMD_SYSTEM, &message, &queues));
    assert(response.status == SERIAL_RESPONSE_BUSY && response.request_command == SERIAL_CMD_SYSTEM);
    message.motion = (motion_command_t){.version = 1, .sequence = 23, .flags = 1};
    assert(app_protocol_receive(SERIAL_CMD_MOTION, &message, &queues));
    assert(response.status == SERIAL_RESPONSE_BUSY && response.request_command == SERIAL_CMD_MOTION);
    assert(queue_calls == calls);
    maintenance = false;
}
static void test_ignored_device_frames(void) {
    const uint32_t commands[] = {SERIAL_CMD_TELEMETRY, SERIAL_CMD_IMU_STATUS,
        SERIAL_CMD_IMU_DIAGNOSTIC, SERIAL_CMD_IMU_ATTITUDE, SERIAL_CMD_ENCODER_TEST,
        SERIAL_CMD_WHEEL_CONTROL, SERIAL_CMD_WHEEL_TEST};
    const unsigned calls_before = queue_calls, stops_before = stop_count;
    const serial_message_t message = {0};
    for (unsigned n = 0; n < sizeof(commands) / sizeof(commands[0]); ++n)
        assert(app_protocol_receive(commands[n], &message, &queues));
    assert(queue_calls == calls_before && stop_count == stops_before);
    assert(!app_protocol_receive(SERIAL_CMD_MOTION, NULL, &queues));
    assert(!app_protocol_receive(SERIAL_CMD_MOTION, &message, NULL));
}
#else
static void test_passive_mode(void) {
    serial_message_t message = {0};
    message.system.action = SYSTEM_ACTION_ARM;
    assert(app_protocol_receive(SERIAL_CMD_SYSTEM, &message, &queues));
    message.motion.flags = 1U;
    assert(app_protocol_receive(SERIAL_CMD_MOTION, &message, &queues));
    assert(app_protocol_receive(SERIAL_CMD_PID_CONFIG, &message, &queues));
    assert(app_protocol_receive(SERIAL_CMD_SYSTEM, NULL, NULL));
    assert(queue_calls == 0U && stop_count == 0U);
}
#endif

static void test_telemetry_boundaries(void) {
    balance_runtime_snapshot_t snapshot = {0};
    snapshot.timestamp_ms = UINT32_MAX - 2U;
    snapshot.state = BALANCE_STATE_FALLEN;
    snapshot.fault_flags = SAFETY_FAULT_EMERGENCY_STOP | SAFETY_FAULT_SENSOR_INVALID;
    snapshot.attitude.pitch_deg = 1.5f;
    snapshot.attitude.gyro_dps[1] = -2.25f;
    snapshot.left_counts_per_5ms = 8.0f;
    snapshot.right_counts_per_5ms = -8.0f;
    snapshot.outputs.left_pwm = 1234;
    snapshot.outputs.right_pwm = -2345;
    app_protocol_telemetry(&snapshot);
    assert(tx_calls == 1U && transmitted.sequence == 0U && transmitted.version == SERIAL_PROTOCOL_VERSION);
    assert(transmitted.status_flags == BALANCE_STATE_FALLEN && transmitted.fault_code == snapshot.fault_flags);
    assert(transmitted.angle_q16_16 == 98304 && transmitted.gyro_q16_16 == -147456);
    assert(transmitted.left_speed_q16_16 == 8 * 65536 && transmitted.right_speed_q16_16 == -8 * 65536);
    assert(transmitted.left_pwm == 1234 && transmitted.right_pwm == -2345);
    assert(transmitted.timestamp_ms == snapshot.timestamp_ms && transmitted.battery_mv == 0U);
    snapshot.attitude.pitch_deg = FLT_MAX;
    snapshot.attitude.gyro_dps[1] = -FLT_MAX;
    snapshot.left_counts_per_5ms = NAN;
    snapshot.right_counts_per_5ms = INFINITY;
    tx_result = SERIAL_PROTOCOL_TX_FULL;
    app_protocol_telemetry(&snapshot);
    assert(transmitted.angle_q16_16 == INT32_MAX && transmitted.gyro_q16_16 == INT32_MIN);
    assert(transmitted.left_speed_q16_16 == 0 && transmitted.right_speed_q16_16 == 0);
    snapshot.attitude.pitch_deg = 1.0f / 131072.0f;
    snapshot.attitude.gyro_dps[1] = -1.0f / 131072.0f;
    snapshot.left_counts_per_5ms = 32768.0f;
    snapshot.right_counts_per_5ms = -32768.0f;
    app_protocol_telemetry(&snapshot);
    assert(transmitted.angle_q16_16 == 1 && transmitted.gyro_q16_16 == -1);
    assert(transmitted.left_speed_q16_16 == INT32_MAX && transmitted.right_speed_q16_16 == INT32_MIN);
    for (unsigned n = tx_calls; n < 260U; ++n) app_protocol_telemetry(&snapshot);
    assert(tx_calls == 260U && transmitted.sequence == 3U);
    app_protocol_telemetry(NULL);
    assert(tx_calls == 260U);
}
int main(void) {
#if WHEEL_TEST_ENABLED
    test_wheel_routes();
#elif !ENCODER_PUSH_TEST_ENABLED && !MOTOR_POLARITY_TEST_ENABLED
    test_motion_domain(); test_pid_and_system_routes(); test_parameter_routes();
    test_tuning_rejections(); test_ignored_device_frames();
#else
    test_passive_mode();
#endif
    test_telemetry_boundaries();
    puts("PASS: application protocol routing and nonblocking telemetry");
    return 0;
}
