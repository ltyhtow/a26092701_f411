#ifndef TEST_USB_DEVICE_HAL_H
#define TEST_USB_DEVICE_HAL_H
#include <stdint.h>
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t GINTSTS, GINTMSK; } test_usb_registers_t;
extern test_usb_registers_t test_usb_registers;
#define USB_OTG_FS (&test_usb_registers)
#define USB_OTG_GINTSTS_USBRST (1U << 12)
#define GPIOA ((void *)1)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_MODE_AF_PP 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIO_AF10_OTG_FS 10U
#define OTG_FS_IRQn 67U
void test_usb_clock(unsigned operation);
#define __HAL_RCC_GPIOA_CLK_ENABLE() test_usb_clock(1U)
#define __HAL_RCC_USB_OTG_FS_CLK_ENABLE() test_usb_clock(2U)
#define __HAL_RCC_USB_OTG_FS_FORCE_RESET() test_usb_clock(3U)
#define __HAL_RCC_USB_OTG_FS_RELEASE_RESET() test_usb_clock(4U)
#define __HAL_RCC_USB_OTG_FS_CLK_DISABLE() test_usb_clock(5U)
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
uint32_t HAL_GetTick(void);
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *pins);
void HAL_GPIO_DeInit(void *port, uint32_t pins);
void NVIC_DisableIRQ(unsigned irq);
void NVIC_ClearPendingIRQ(unsigned irq);
void HAL_NVIC_SetPriority(unsigned irq, unsigned preemption, unsigned sub);
#endif
