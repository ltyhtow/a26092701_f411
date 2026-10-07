#include "serial_transport.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart2;
static uint8_t *rx_target;
static uint16_t rx_size;
static unsigned receive_starts, tx_starts;
static HAL_StatusTypeDef next_start_result;
static unsigned critical_depth, unlock_to_inject;

void test_enter_critical(void) { critical_depth++; }
void test_exit_critical(void) {
    assert(critical_depth > 0);
    if (--critical_depth == 0 && unlock_to_inject > 0 && --unlock_to_inject == 0) {
        HAL_UART_ErrorCallback(&huart2); /* Model a pending UART RX error IRQ. */
    }
}

void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken) {
    (void)task; *woken = pdFALSE;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size) {
    receive_starts++;
    if (uart->RxState != HAL_UART_STATE_READY) return HAL_BUSY;
    if (next_start_result != HAL_OK) {
        HAL_StatusTypeDef result = next_start_result;
        next_start_result = HAL_OK;
        return result;
    }
    rx_target = data; rx_size = size; uart->RxState = HAL_UART_STATE_BUSY_RX;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size) {
    assert(data && size); tx_starts++; uart->gState = HAL_UART_STATE_BUSY_TX; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart) { uart->RxState = HAL_UART_STATE_READY; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart) { uart->gState = HAL_UART_STATE_READY; return HAL_OK; }
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart) { return uart->event; }

static void reset_transport(void) {
    memset(&huart2, 0, sizeof(huart2));
    receive_starts = tx_starts = 0; next_start_result = HAL_OK;
    critical_depth = unlock_to_inject = 0;
    assert(serial_transport_init((void *)1) == pdPASS);
    assert(rx_size == 64);
}
static void receive_event(HAL_UART_RxEventTypeTypeDef event, uint16_t size) {
    huart2.event = event;
    if (event != HAL_UART_RXEVENT_HT) huart2.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_RxEventCallback(&huart2, size);
}
static void expect_bytes(const uint8_t *expected, unsigned count) {
    uint8_t actual[128];
    assert(count <= sizeof(actual));
    assert(lwrb_get_full(serial_transport_rx_buffer()) == count);
    assert(lwrb_read(serial_transport_rx_buffer(), actual, count) == count);
    assert(memcmp(actual, expected, count) == 0);
}

int main(void) {
    uint8_t first[64], second[64], combined[128];
    for (unsigned i = 0; i < 64; i++) { first[i] = (uint8_t)i; second[i] = (uint8_t)(128+i); }
    memcpy(combined, first, 64); memcpy(combined + 64, second, 64);

    reset_transport();
    memcpy(rx_target, first, 40);
    receive_event(HAL_UART_RXEVENT_HT, 32);
    assert(receive_starts == 1); /* A half transfer must never restart DMA. */
    receive_event(HAL_UART_RXEVENT_IDLE, 40);
    expect_bytes(first, 40);
    assert(receive_starts == 2);

    reset_transport();
    memcpy(rx_target, first, 64);
    receive_event(HAL_UART_RXEVENT_HT, 32);
    receive_event(HAL_UART_RXEVENT_TC, 64);
    memcpy(rx_target, second, 64);
    receive_event(HAL_UART_RXEVENT_HT, 32);
    receive_event(HAL_UART_RXEVENT_TC, 64);
    expect_bytes(combined, 128);

    reset_transport();
    memcpy(rx_target, first, 21);
    receive_event(HAL_UART_RXEVENT_IDLE, 21);
    expect_bytes(first, 21);

    reset_transport();
    memcpy(rx_target, first, 40);
    receive_event(HAL_UART_RXEVENT_HT, 32);
    next_start_result = HAL_ERROR;
    receive_event(HAL_UART_RXEVENT_IDLE, 40);
    expect_bytes(first, 40);
    serial_transport_poll_tx(); /* Deferred restart succeeds without recopy. */
    assert(receive_starts == 3);
    memcpy(rx_target, second, 7);
    receive_event(HAL_UART_RXEVENT_IDLE, 7);
    expect_bytes(second, 7);

    reset_transport();
    assert(lwrb_write(serial_transport_tx_buffer(), first, 20) == 20);
    serial_transport_poll_tx(); serial_transport_poll_tx();
    assert(tx_starts == 1);
    huart2.gState = HAL_UART_STATE_READY;
    HAL_UART_TxCpltCallback(&huart2);
    serial_transport_poll_tx();
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 0);

    reset_transport();
    assert(lwrb_write(serial_transport_tx_buffer(), first, 20) == 20);
    unlock_to_inject = 2; /* service_rx_restart unlock, then TX start unlock */
    serial_transport_poll_tx();
    assert(unlock_to_inject == 0);
    serial_transport_poll_tx();
    assert(tx_starts == 1);
    huart2.gState = HAL_UART_STATE_READY;
    HAL_UART_TxCpltCallback(&huart2);
    HAL_UART_ErrorCallback(&huart2); /* RX error must not discard completed TX. */
    serial_transport_poll_tx();
    assert(tx_starts == 1 && lwrb_get_full(serial_transport_tx_buffer()) == 0);
    serial_transport_deinit();
    puts("PASS serial transport: RX boundaries/retry, TX completion and RX-error races");
    return 0;
}
