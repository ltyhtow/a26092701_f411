#ifndef TEST_SERIAL_USART_H
#define TEST_SERIAL_USART_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef enum { HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_IDLE } HAL_UART_RxEventTypeTypeDef;
enum { HAL_UART_STATE_READY, HAL_UART_STATE_BUSY_RX, HAL_UART_STATE_BUSY_TX };
#define HAL_UART_ERROR_DMA (1U << 4)
typedef struct { uint32_t RxState, gState, ErrorCode; HAL_UART_RxEventTypeTypeDef event; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart2;
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
#endif
