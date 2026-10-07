#ifndef TEST_USB_TRANSPORT_STM32_H
#define TEST_USB_TRANSPORT_STM32_H
#include <stdint.h>
uint32_t __get_IPSR(void);
uint32_t __get_PRIMASK(void);
uint32_t __get_BASEPRI(void);
#endif
