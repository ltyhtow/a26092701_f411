#ifndef TEST_USB_DEVICE_USBD_H
#define TEST_USB_DEVICE_USBD_H
#include <stdbool.h>
#include <stdint.h>
bool usbd_edpt_busy(uint8_t port, uint8_t endpoint);
#endif
