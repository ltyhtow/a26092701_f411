#ifndef TEST_USB_CDC_DEVICE_H
#define TEST_USB_CDC_DEVICE_H
#include <stdbool.h>
#include <stdint.h>
bool usb_cdc_device_init(void);
void usb_cdc_device_deinit(void);
void usb_cdc_device_poll(void);
bool usb_cdc_device_open(void);
bool usb_cdc_device_arm_receive(void);
bool usb_cdc_device_transmit(const uint8_t *data, uint16_t length, uint32_t token);
void serial_usb_rx_complete(const uint8_t *data, uint32_t length);
void serial_usb_tx_complete(uint32_t token);
void serial_usb_link_reset(void);
#endif
