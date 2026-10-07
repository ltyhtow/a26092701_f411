#ifndef USB_CDC_DEVICE_H
#define USB_CDC_DEVICE_H
#include <stdbool.h>
#include <stdint.h>

#define USB_CDC_PACKET_SIZE 64U

/* Only the protocol task calls these APIs, after the scheduler starts. The
 * device uses TinyUSB OS_NONE: poll drains ISR events without running an RTOS
 * task or printing text. open means configured, DTR asserted, not suspended. */
bool usb_cdc_device_init(void);
void usb_cdc_device_deinit(void);
void usb_cdc_device_poll(void);
bool usb_cdc_device_open(void);
bool usb_cdc_device_transmit(const uint8_t *data, uint16_t length, uint32_t token);
bool usb_cdc_device_arm_receive(void);
void usb_cdc_device_irq_handler(void);

/* Transport callbacks. RX/TX callbacks execute inside poll (task context).
 * link_reset may also execute at ISR event enqueue: it must only post flags /
 * change the transport generation, never clear a ring or issue motor control.
 * A successful TX owns its token until matching completion or link reset. */
void serial_usb_rx_complete(const uint8_t *data, uint32_t length);
void serial_usb_tx_complete(uint32_t token);
void serial_usb_link_reset(void);
#endif
