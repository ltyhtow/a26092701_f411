#include "serial_transport.h"

#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SERIAL_RX_RING_SIZE 256U
#define SERIAL_TX_RING_SIZE 256U
#define SERIAL_DMA_RX_SIZE  64U

static lwrb_t rx_rb;
static lwrb_t tx_rb;
static uint8_t rx_rb_data[SERIAL_RX_RING_SIZE];
static uint8_t tx_rb_data[SERIAL_TX_RING_SIZE];
static uint8_t dma_rx_data[SERIAL_DMA_RX_SIZE];
static TaskHandle_t notify_task;
static volatile uint32_t dma_rx_position;
static volatile uint32_t tx_dma_length;
static volatile uint32_t tx_dma_complete_length;
static volatile uint8_t tx_dma_active;
static volatile uint8_t tx_dma_complete_pending;
static volatile uint8_t tx_dma_error_pending;
static volatile uint8_t rx_restart_pending;

static void serial_notify_from_isr(void) {
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (notify_task != NULL) {
        (void)xTaskNotifyFromISR(notify_task, 0U, eNoAction, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static void serial_copy_dma_rx(uint32_t size_byte) {
    if (size_byte > SERIAL_DMA_RX_SIZE) {
        size_byte = SERIAL_DMA_RX_SIZE;
    }
    if (size_byte > dma_rx_position) {
        (void)lwrb_write(&rx_rb, &dma_rx_data[dma_rx_position],
                         (lwrb_sz_t)(size_byte - dma_rx_position));
        dma_rx_position = size_byte;
    }
}

static void serial_restart_rx(void) {
    dma_rx_position = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_data,
                                     SERIAL_DMA_RX_SIZE) != HAL_OK) {
        rx_restart_pending = 1U;
    }
}

static void serial_service_rx_restart(void) {
    if (rx_restart_pending != 0U && huart2.RxState == HAL_UART_STATE_READY) {
        rx_restart_pending = 0U;
        serial_restart_rx();
    }
}

BaseType_t serial_transport_init(TaskHandle_t task) {
    notify_task = task;
    dma_rx_position = 0U;
    tx_dma_length = 0U;
    tx_dma_complete_length = 0U;
    tx_dma_active = 0U;
    tx_dma_complete_pending = 0U;
    tx_dma_error_pending = 0U;
    rx_restart_pending = 0U;

    if (lwrb_init(&rx_rb, rx_rb_data, sizeof(rx_rb_data)) == 0U ||
        lwrb_init(&tx_rb, tx_rb_data, sizeof(tx_rb_data)) == 0U) {
        return pdFAIL;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_data,
                                     SERIAL_DMA_RX_SIZE) != HAL_OK) {
        return pdFAIL;
    }
    return pdPASS;
}

void serial_transport_deinit(void) {
    if (huart2.RxState != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortReceive(&huart2);
    }
    if (huart2.gState != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortTransmit(&huart2);
    }
    notify_task = NULL;
    dma_rx_position = 0U;
    tx_dma_length = 0U;
    tx_dma_complete_length = 0U;
    tx_dma_active = 0U;
    tx_dma_complete_pending = 0U;
    tx_dma_error_pending = 0U;
    rx_restart_pending = 0U;
}

lwrb_t* serial_transport_rx_buffer(void) {
    return &rx_rb;
}

lwrb_t* serial_transport_tx_buffer(void) {
    return &tx_rb;
}

void serial_transport_poll_tx(void) {
    void* address;
    lwrb_sz_t length;

    serial_service_rx_restart();

    taskENTER_CRITICAL();
    if (tx_dma_error_pending != 0U) {
        tx_dma_error_pending = 0U;
        tx_dma_complete_pending = 0U;
        tx_dma_complete_length = 0U;
        tx_dma_active = 0U;
        tx_dma_length = 0U;
    } else if (tx_dma_complete_pending != 0U) {
        (void)lwrb_skip(&tx_rb, tx_dma_complete_length);
        tx_dma_complete_pending = 0U;
        tx_dma_complete_length = 0U;
        tx_dma_length = 0U;
        tx_dma_active = 0U;
    }
    if (tx_dma_active != 0U) {
        taskEXIT_CRITICAL();
        return;
    }

    length = lwrb_get_linear_block_read_length(&tx_rb);
    address = lwrb_get_linear_block_read_address(&tx_rb);
    if (length == 0U || address == NULL) {
        taskEXIT_CRITICAL();
        return;
    }

    tx_dma_length = length;
    tx_dma_active = 1U;
    taskEXIT_CRITICAL();

    if (HAL_UART_Transmit_DMA(&huart2, address, (uint16_t)length) != HAL_OK) {
        taskENTER_CRITICAL();
        tx_dma_active = 0U;
        tx_dma_length = 0U;
        taskEXIT_CRITICAL();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart == &huart2) {
        serial_copy_dma_rx(Size);
        serial_restart_rx();
        serial_notify_from_isr();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart2) {
        tx_dma_complete_length = tx_dma_length;
        tx_dma_complete_pending = 1U;
        serial_notify_from_isr();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart2) {
        if (tx_dma_active != 0U && huart->gState == HAL_UART_STATE_READY) {
            tx_dma_error_pending = 1U;
        }
        if (huart->RxState == HAL_UART_STATE_READY) {
            rx_restart_pending = 1U;
        }
        serial_notify_from_isr();
    }
}

void serial_transport_send_string(const char *str) {
    if (str == NULL) {
        return;
    }
    size_t len = strlen(str);
    if (len == 0U) {
        return;
    }
    taskENTER_CRITICAL();
    (void)lwrb_write(&tx_rb, str, (lwrb_sz_t)len);
    taskEXIT_CRITICAL();
    serial_transport_poll_tx();
}
