#include "app_freertos.h"
#include "main.h"
#include "projdefs.h"
#include "lwpkt/lwpkt.h"
#include "serial_transport.h"
#include "imu_config.h"
#include "motor_config.h"

#include <string.h>

#define Task1_stack_depth_words         128U
#define ProtocolTask_stack_depth_words   384U
#define PROTOCOL_TASK_PERIOD_MS         5U

static TaskHandle_t Task1_Handle;
static TaskHandle_t ProtocolTask_Handle;
static TaskHandle_t ImuTask_Handle;
static TaskHandle_t ImuFusionTask_Handle;
#if MOTOR_POLARITY_TEST_ENABLED
static TaskHandle_t MotorPolarityTestTask_Handle;
#endif
#if IMU_UART_TEST_ENABLED
static TaskHandle_t ImuUartTestTask_Handle;
#endif

static lwpkt_t protocol_packet;
static SemaphoreHandle_t protocol_tx_mutex;
static volatile uint8_t protocol_ready;
static serial_protocol_stats_t protocol_stats;

QueueHandle_t motion_command_queue;
QueueHandle_t pid_config_queue;
QueueHandle_t system_command_queue;
QueueHandle_t imu_sample_queue;
QueueHandle_t imu_fusion_input_queue;
QueueHandle_t imu_attitude_queue;

static void function1(void *pvParameters);
static void protocol_task(void *pvParameters);
static void protocol_dispatch_packet(const lwpkt_t *packet);
static void protocol_event_callback(lwpkt_t *packet, lwpkt_evt_type_t event);
static void app_cleanup_before_scheduler(void);

int32_t app_synctasks_init(void)
{
  BaseType_t ret;

  motion_command_queue = xQueueCreate(1U, sizeof(motion_command_t));
  pid_config_queue = xQueueCreate(2U, sizeof(pid_config_command_t));
  system_command_queue = xQueueCreate(4U, sizeof(system_command_t));
  imu_sample_queue = xQueueCreate(1U, sizeof(imu_sample_message_t));
  imu_fusion_input_queue = xQueueCreate(1U, sizeof(imu_sample_message_t));
  imu_attitude_queue = xQueueCreate(1U, sizeof(imu_fusion_output_t));
  protocol_tx_mutex = xSemaphoreCreateMutex();

  if (motion_command_queue == NULL || pid_config_queue == NULL || system_command_queue == NULL ||
      imu_sample_queue == NULL || imu_fusion_input_queue == NULL ||
      imu_attitude_queue == NULL || protocol_tx_mutex == NULL)
  {
      app_cleanup_before_scheduler();
      return -1;
  }

  ret = xTaskCreate(protocol_task, "Protocol", ProtocolTask_stack_depth_words,
                    (void*) NULL, 2U, &ProtocolTask_Handle);
  if (ret != pdPASS)
  {
      app_cleanup_before_scheduler();
      return -1;
  }

  if (serial_transport_init(ProtocolTask_Handle) != pdPASS)
  {
      app_cleanup_before_scheduler();
      return -1;
  }

  ret = imu_task_start(imu_sample_queue, imu_fusion_input_queue, &ImuTask_Handle);
  if (ret != pdPASS)
  {
      app_cleanup_before_scheduler();
      return -1;
  }

  ret = imu_fusion_task_start(imu_fusion_input_queue, imu_attitude_queue,
                              &ImuFusionTask_Handle);
  if (ret != pdPASS)
  {
      app_cleanup_before_scheduler();
      return -1;
  }

#if IMU_UART_TEST_ENABLED
  ret = imu_uart_test_task_start(imu_sample_queue, imu_attitude_queue,
                                &ImuUartTestTask_Handle);
  if (ret != pdPASS)
  {
      app_cleanup_before_scheduler();
      return -1;
  }
#endif

#if MOTOR_POLARITY_TEST_ENABLED
  ret = motor_polarity_test_task_start(&MotorPolarityTestTask_Handle);
  if (ret != pdPASS)
  {
      app_cleanup_before_scheduler();
      return -1;
  }
#endif

  /* Heartbeat Task (PC13 SYS_LED toggle) */
  ret = xTaskCreate(function1, "Task1", Task1_stack_depth_words,
                    (void*) NULL, 0, &Task1_Handle);
  if (ret != pdPASS)
  {
      app_cleanup_before_scheduler();
      return -1;
  }

  return 0;
}

static void function1(void *pvParameters)
{
  (void)pvParameters;
  imu_fusion_output_t attitude;
  uint32_t delay_ms = 500U;

  for(;;)
  {
    if (imu_port_is_ready() == 0U)
    {
      /* MPU6050 未连接或 I2C 通信失败：快闪 (100ms) */
      delay_ms = 100U;
    }
    else if (xQueuePeek(imu_attitude_queue, &attitude, 0U) == pdTRUE)
    {
      if ((attitude.status_flags & IMU_FUSION_STATUS_CALIBRATING) != 0U)
      {
        /* 启动静止校准中 (累计400个样本，约2秒)：中速闪 (250ms) */
        delay_ms = 250U;
      }
      else
      {
        /* 校准完毕，姿态解算正常：慢闪 (500ms) */
        delay_ms = 500U;
      }
    }
    else
    {
      delay_ms = 500U;
    }

    HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
  }
}

static void protocol_dispatch_packet(const lwpkt_t *packet)
{
  const uint8_t *data = (const uint8_t *)lwpkt_get_data(packet);
  const size_t length = lwpkt_get_data_len(packet);
  const uint32_t command = lwpkt_get_cmd(packet);

  if (data == NULL)
  {
      return;
  }

  switch (command)
  {
    case SERIAL_CMD_MOTION:
    {
      motion_command_t value;
      if (length == sizeof(value))
      {
          memcpy(&value, data, sizeof(value));
          if (value.version == SERIAL_PROTOCOL_VERSION && value.reserved[0] == 0U &&
              value.reserved[1] == 0U && (value.flags & ~0x0003U) == 0U)
          {
              (void)xQueueOverwrite(motion_command_queue, &value);
          }
      }
      break;
    }

    case SERIAL_CMD_PID_CONFIG:
    {
      pid_config_command_t value;
      if (length == sizeof(value))
      {
          memcpy(&value, data, sizeof(value));
          if (value.version == SERIAL_PROTOCOL_VERSION && value.reserved == 0U &&
              value.loop_id <= 2U)
          {
              if (xQueueSend(pid_config_queue, &value, 0U) != pdPASS)
              {
                  protocol_stats.queue_overruns++;
              }
          }
      }
      break;
    }

    case SERIAL_CMD_SYSTEM:
    {
      system_command_t value;
      if (length == sizeof(value))
      {
          memcpy(&value, data, sizeof(value));
          if (value.version == SERIAL_PROTOCOL_VERSION && value.reserved == 0U &&
              value.action >= 1U && value.action <= 4U)
          {
              if (xQueueSend(system_command_queue, &value, 0U) != pdPASS)
              {
                  protocol_stats.queue_overruns++;
              }
          }
      }
      break;
    }

    default:
      protocol_stats.unknown_commands++;
      break;
  }
}

static void protocol_event_callback(lwpkt_t *packet, lwpkt_evt_type_t event)
{
  (void)packet;
  if (event == LWPKT_EVT_TIMEOUT)
  {
      protocol_stats.rx_timeouts++;
  }
}

static void protocol_task(void *pvParameters)
{
  lwrb_t *rx_buffer = serial_transport_rx_buffer();
  lwrb_t *tx_buffer = serial_transport_tx_buffer();
  (void)pvParameters;

  if (lwpkt_init(&protocol_packet, tx_buffer, rx_buffer) != lwpktOK)
  {
      vTaskDelete(NULL);
      return;
  }
  (void)lwpkt_set_evt_fn(&protocol_packet, protocol_event_callback);
  protocol_ready = 1U;

  for (;;)
  {
    lwpktr_t result;
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();

    (void)xSemaphoreTake(protocol_tx_mutex, portMAX_DELAY);
    do
    {
      result = lwpkt_process(&protocol_packet, now_ms);
      if (result == lwpktERRCRC)
      {
          protocol_stats.rx_crc_errors++;
      }
      else if (result == lwpktERRSTOP)
      {
          protocol_stats.rx_stop_errors++;
      }
      else if (result == lwpktERRMEM)
      {
          protocol_stats.rx_memory_errors++;
      }
      if (result == lwpktVALID)
      {
          protocol_dispatch_packet(&protocol_packet);
      }
    } while (result == lwpktVALID || result == lwpktERR || result == lwpktERRCRC ||
             result == lwpktERRSTOP || result == lwpktERRMEM);

    serial_transport_poll_tx();
    (void)xSemaphoreGive(protocol_tx_mutex);
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PROTOCOL_TASK_PERIOD_MS));
  }
}

serial_protocol_result_t serial_protocol_send(uint32_t command, const void *data, size_t length)
{
  lwpktr_t result;
  const BaseType_t protocol_task_owns_mutex =
      (xTaskGetCurrentTaskHandle() == ProtocolTask_Handle) ? pdTRUE : pdFALSE;

  if (protocol_ready == 0U || protocol_tx_mutex == NULL ||
      (protocol_task_owns_mutex == pdFALSE &&
       xSemaphoreTake(protocol_tx_mutex, pdMS_TO_TICKS(10U)) != pdTRUE))
  {
      return (protocol_ready == 0U) ? SERIAL_PROTOCOL_NOT_READY : SERIAL_PROTOCOL_BUSY;
  }
  if ((data == NULL && length != 0U) || length > LWPKT_CFG_MAX_DATA_LEN)
  {
      if (protocol_task_owns_mutex == pdFALSE)
      {
          (void)xSemaphoreGive(protocol_tx_mutex);
      }
      return SERIAL_PROTOCOL_INVALID_ARGUMENT;
  }

  result = lwpkt_write(&protocol_packet, command, data, length);
  serial_transport_poll_tx();
  if (protocol_task_owns_mutex == pdFALSE)
  {
      (void)xSemaphoreGive(protocol_tx_mutex);
  }
  if (result == lwpktOK)
  {
      return SERIAL_PROTOCOL_OK;
  }
  return (result == lwpktERRMEM) ? SERIAL_PROTOCOL_TX_FULL : SERIAL_PROTOCOL_ERROR;
}

void serial_protocol_get_stats(serial_protocol_stats_t *stats)
{
  if (stats != NULL)
  {
      taskENTER_CRITICAL();
      *stats = protocol_stats;
      taskEXIT_CRITICAL();
  }
}

static void app_cleanup_before_scheduler(void)
{
  serial_transport_deinit();
  if (ProtocolTask_Handle != NULL)
  {
      vTaskDelete(ProtocolTask_Handle);
      ProtocolTask_Handle = NULL;
  }
  if (Task1_Handle != NULL)
  {
      vTaskDelete(Task1_Handle);
      Task1_Handle = NULL;
  }
  if (ImuTask_Handle != NULL)
  {
      vTaskDelete(ImuTask_Handle);
      ImuTask_Handle = NULL;
  }
  if (ImuFusionTask_Handle != NULL)
  {
      vTaskDelete(ImuFusionTask_Handle);
      ImuFusionTask_Handle = NULL;
  }
#if MOTOR_POLARITY_TEST_ENABLED
  if (MotorPolarityTestTask_Handle != NULL)
  {
      vTaskDelete(MotorPolarityTestTask_Handle);
      MotorPolarityTestTask_Handle = NULL;
  }
#endif
#if IMU_UART_TEST_ENABLED
  if (ImuUartTestTask_Handle != NULL)
  {
      vTaskDelete(ImuUartTestTask_Handle);
      ImuUartTestTask_Handle = NULL;
  }
#endif
  if (protocol_tx_mutex != NULL)
  {
      vSemaphoreDelete(protocol_tx_mutex);
      protocol_tx_mutex = NULL;
  }
  if (motion_command_queue != NULL)
  {
      vQueueDelete(motion_command_queue);
      motion_command_queue = NULL;
  }
  if (pid_config_queue != NULL)
  {
      vQueueDelete(pid_config_queue);
      pid_config_queue = NULL;
  }
  if (system_command_queue != NULL)
  {
      vQueueDelete(system_command_queue);
      system_command_queue = NULL;
  }
  if (imu_sample_queue != NULL)
  {
      vQueueDelete(imu_sample_queue);
      imu_sample_queue = NULL;
  }
  if (imu_fusion_input_queue != NULL)
  {
      vQueueDelete(imu_fusion_input_queue);
      imu_fusion_input_queue = NULL;
  }
  if (imu_attitude_queue != NULL)
  {
      vQueueDelete(imu_attitude_queue);
      imu_attitude_queue = NULL;
  }
}
