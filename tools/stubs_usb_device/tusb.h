#ifndef TEST_USB_DEVICE_TUSB_H
#define TEST_USB_DEVICE_TUSB_H
#include <stdbool.h>
#include <stdint.h>
#define TUSB_ROLE_DEVICE 1U
#define TUSB_SPEED_FULL 2U
#define TUD_CFGID_DWC2 1U
#define CFG_TUD_CDC_TX_BUFSIZE 64U
typedef struct { struct { unsigned bm_double_buffered; bool vbus_sensing; } dwc2; } tud_configure_param_t;
typedef struct { unsigned role, speed; } tusb_rhport_init_t;
bool tud_configure(uint8_t port, unsigned id, const tud_configure_param_t *configuration);
bool tusb_init(uint8_t port, const tusb_rhport_init_t *configuration);
bool tud_deinit(uint8_t port);
bool tusb_deinit(uint8_t port);
bool tud_cdc_connected(void);
bool tud_suspended(void);
uint32_t tud_cdc_write_available(void);
uint32_t tud_cdc_write(const void *data, uint32_t length);
uint32_t tud_cdc_write_flush(void);
bool tud_cdc_write_clear(void);
void tud_cdc_read_flush(void);
uint32_t tud_cdc_available(void);
uint32_t tud_cdc_read(void *data, uint32_t length);
void tud_task_ext(uint32_t timeout_ms, bool in_isr);
bool tud_task_event_ready(void);
void tud_int_handler(uint8_t port);
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts);
void tud_cdc_tx_complete_cb(uint8_t itf);
void tud_event_hook_cb(uint8_t port, uint32_t event, bool in_isr);
#endif
