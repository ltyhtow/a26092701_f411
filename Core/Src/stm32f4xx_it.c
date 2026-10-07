#include "main.h"
#include "stm32f4xx_it.h"
#include "motor_driver.h"
#include "fault_capture.h"
#if SERIAL_TRANSPORT_USB_CDC
#include "usb_cdc_device.h"
#endif

extern TIM_HandleTypeDef htim11;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
/* No C prologue may run before MSP/PSP and EXC_RETURN are preserved. The first
 * handler claims the snapshot; a nested NMI only cuts the bridge and halts on a
 * separate stack. The basic/extended frame is decoded by the checked C helper. */
#define FAULT_CAPTURE_STRINGIFY_INNER(value) #value
#define FAULT_CAPTURE_STRINGIFY(value) FAULT_CAPTURE_STRINGIFY_INNER(value)
#define CORTEX_FAULT_ENTRY() __asm volatile ( \
    "cpsid i\n" \
    "mrs r0, ipsr\n" \
    "ldr r12, =g_fault_capture_guard\n" \
    "ldr r1, [r12]\n" \
    "cmp r1, #0\n" \
    "bne 1f\n" \
    "str r0, [r12]\n" \
    "mrs r2, msp\n" \
    "mrs r3, psp\n" \
    "mov r1, lr\n" \
    "tst r1, #4\n" \
    "ite eq\n" \
    "moveq r0, r2\n" \
    "movne r0, r3\n" \
    "ldr r12, =g_fault_capture_stack + " FAULT_CAPTURE_STRINGIFY(FAULT_CAPTURE_STACK_BYTES) "\n" \
    "msr msp, r12\n" \
    "isb\n" \
    "b fault_capture_entry\n" \
    "1:\n" \
    "ldr r12, =g_fault_nested_stack + " FAULT_CAPTURE_STRINGIFY(FAULT_CAPTURE_STACK_BYTES) "\n" \
    "msr msp, r12\n" \
    "isb\n" \
    "b fault_capture_nested_halt\n")

__attribute__((naked, noreturn)) void NMI_Handler(void) { CORTEX_FAULT_ENTRY(); }
__attribute__((naked, noreturn)) void HardFault_Handler(void) { CORTEX_FAULT_ENTRY(); }
__attribute__((naked, noreturn)) void MemManage_Handler(void) { CORTEX_FAULT_ENTRY(); }
__attribute__((naked, noreturn)) void BusFault_Handler(void) { CORTEX_FAULT_ENTRY(); }
__attribute__((naked, noreturn)) void UsageFault_Handler(void) { CORTEX_FAULT_ENTRY(); }

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt (Timebase).
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim11);
}

/**
  * @brief This function handles EXTI line1 interrupt (MPU6050 INT).
  */
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(MPU6050_INT_Pin);
}

/**
  * @brief This function handles DMA1 stream5 global interrupt (USART2 RX).
  */
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
  * @brief This function handles DMA1 stream6 global interrupt (USART2 TX).
  */
void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

#if SERIAL_TRANSPORT_USB_CDC
void OTG_FS_IRQHandler(void)
{
  usb_cdc_device_irq_handler();
}
#endif
