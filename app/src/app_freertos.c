/* Composition root: queue ownership, task startup and board port injection. */
#include "app_freertos.h"
#include "app_config.h"
#include "app_clock.h"
#include "balance_task.h"
#include "board_status.h"
#include "encoder_driver.h"
#include "encoder_push_test_task.h"
#include "wheel_test_task.h"
#include "imu_task.h"
#include "imu_fusion_task.h"
#include "imu_uart_test_task.h"
#include "motor_driver.h"
#include "motor_config.h"
#include "motor_polarity_test_task.h"
#include "serial_protocol_adapter.h"
#include "app_protocol.h"
#include "parameter_service.h"
#include "task.h"

static TaskHandle_t led_task, imu_task, imu_diagnostic_task;
static TaskHandle_t diagnostic_task, balance_task;
static bool start_attempted;
static QueueHandle_t imu_samples, fusion_input, attitude;
static app_command_queues_t commands;

static void status_task(void *argument) {
    (void)argument;
    for (;;) {
        uint32_t period = 500U;
#if ENCODER_PUSH_TEST_ENABLED || WHEEL_TEST_ENABLED
        period = encoder_driver_is_ready() ? 500U : 100U;
#else
        attitude_sample_t sample;
        if (!imu_port_is_ready()) period = 100U;
        else if (xQueuePeek(attitude, &sample, 0U) == pdTRUE &&
                 (sample.status_flags & ATTITUDE_STATUS_CALIBRATING) != 0U) period = 250U;
#if !MOTOR_POLARITY_TEST_ENABLED
        else if (balance_task_get_state() == BALANCE_STATE_ARMED) {
            board_status_set(true);
            vTaskDelay(app_clock_period_ticks(100U));
            continue;
        } else if (balance_task_get_state() == BALANCE_STATE_FALLEN) period = 150U;
#endif
#endif
        board_status_toggle();
        vTaskDelay(app_clock_period_ticks(period));
    }
}

static void cleanup(void) {
    serial_service_stop();
    parameter_service_stop();
    imu_fusion_task_stop();
    TaskHandle_t *tasks[] = {&led_task, &imu_task, &imu_diagnostic_task, &diagnostic_task, &balance_task};
    for (unsigned i = 0; i < sizeof(tasks)/sizeof(tasks[0]); ++i) {
        if (*tasks[i] != NULL) { vTaskDelete(*tasks[i]); *tasks[i] = NULL; }
    }
    QueueHandle_t *queues[] = {&commands.motion, &commands.pid, &commands.system,
                               &imu_samples, &fusion_input, &attitude};
    for (unsigned i = 0; i < sizeof(queues)/sizeof(queues[0]); ++i) {
        if (*queues[i] != NULL) { vQueueDelete(*queues[i]); *queues[i] = NULL; }
    }
}

int32_t app_synctasks_init(void) {
    if (start_attempted || xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) return -1;
    start_attempted = true;
    commands.motion = xQueueCreate(1U, sizeof(motion_request_t));
    commands.pid = xQueueCreate(2U, sizeof(pid_request_t));
    commands.system = xQueueCreate(4U, sizeof(system_request_t));
    imu_samples = xQueueCreate(1U, sizeof(imu_sample_message_t));
    fusion_input = xQueueCreate(1U, sizeof(imu_sample_message_t));
    attitude = xQueueCreate(1U, sizeof(attitude_sample_t));
    if (!commands.motion || !commands.pid || !commands.system || !imu_samples || !fusion_input || !attitude)
        goto failed;
    if (!serial_service_start(app_protocol_receive, &commands)) goto failed;
#if WHEEL_TEST_ENABLED
    if (wheel_test_task_start(&diagnostic_task) != pdPASS) goto failed;
#elif ENCODER_PUSH_TEST_ENABLED
    if (encoder_push_test_task_start(&diagnostic_task) != pdPASS) goto failed;
#else
    TaskHandle_t fusion_task;
    if (imu_task_start(imu_samples, fusion_input, &imu_task) != pdPASS) goto failed;
    if (imu_fusion_task_start(fusion_input, attitude, &fusion_task) != pdPASS) goto failed;
#if IMU_UART_TEST_ENABLED
    if (imu_uart_test_task_start(imu_samples, attitude, &imu_diagnostic_task) != pdPASS) goto failed;
#endif
#if MOTOR_POLARITY_TEST_ENABLED
    if (motor_polarity_test_task_start(&diagnostic_task) != pdPASS) goto failed;
#else
    motor_driver_init();
    if (motor_driver_fault_latched() || !encoder_driver_init()) goto failed;
    balance_controller_config_t controller;
    balance_controller_default_config(&controller);
    controller.max_pwm = (float)MOTOR_PWM_MAX_DUTY;
    controller.deadband_left = controller.deadband_right = (float)MOTOR_DEADBAND_DUTY;
    balance_task_config_t config;
    balance_task_default_config(&config);
    config.controller_config = &controller;
    config.attitude_queue = attitude;
    config.motion_command_queue = commands.motion;
    config.pid_config_queue = commands.pid;
    config.system_command_queue = commands.system;
    config.motor_output_hook = motor_driver_set_output;
    config.encoder_read_hook = encoder_driver_read_speed;
    config.calibration_request_hook = imu_fusion_task_request_calibration;
    config.telemetry_hook = app_protocol_telemetry;
    config.pid_result_hook = app_protocol_pid_result;
    config.enable_telemetry = true;
    config.telemetry_divisor = 4U;
    if (balance_task_start(&config, &balance_task) != pdPASS) goto failed;
    if (!parameter_service_start(&controller)) goto failed;
    commands.parameter_handler = parameter_service_submit;
    commands.parameter_context = NULL;
#endif
#endif
    if (xTaskCreate(status_task, "Status", 256U, NULL, 1U, &led_task) != pdPASS) goto failed;
    return 0;
failed:
    motor_driver_emergency_stop();
    cleanup();
    return -1;
}
