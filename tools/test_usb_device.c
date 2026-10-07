/* Production TinyUSB wrapper; fake peripheral/class boundary, no USB device. */
#include "usb_cdc_device.h"
#include "usb_descriptors.h"
#include "stm32f4xx_hal.h"
#include "tusb.h"
#include "device/dcd.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t primask, now_ms;
static bool configure_ok = true, init_ok = true, configured, dtr, suspended, queued;
static bool endpoint_busy, partial_write;
static unsigned init_count, deinit_count, irq_count, poll_count, reset_count, rx_count, tx_count;
static uint32_t completed_token, received_length;
static uint8_t rx_fifo[64], tx_fifo[64], last_rx[64];
static uint32_t rx_size, tx_size, endpoint_length;
test_usb_registers_t test_usb_registers;

uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __set_PRIMASK(uint32_t mask) { primask = mask; }
uint32_t HAL_GetTick(void) { return now_ms; }
void test_usb_clock(unsigned operation) { assert(operation >= 1U && operation <= 5U); }
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *pins) {
    assert(port == GPIOA && pins->Pin == (GPIO_PIN_11 | GPIO_PIN_12));
    assert(pins->Mode == GPIO_MODE_AF_PP && pins->Pull == GPIO_NOPULL);
    assert(pins->Alternate == GPIO_AF10_OTG_FS);
}
void HAL_GPIO_DeInit(void *port, uint32_t pins) { assert(port == GPIOA && pins == (GPIO_PIN_11 | GPIO_PIN_12)); }
void NVIC_DisableIRQ(unsigned irq) { assert(irq == OTG_FS_IRQn); }
void NVIC_ClearPendingIRQ(unsigned irq) { assert(irq == OTG_FS_IRQn); }
void HAL_NVIC_SetPriority(unsigned irq, unsigned preemption, unsigned sub) {
    assert(irq == OTG_FS_IRQn && preemption >= 5U && sub == 0U);
}
bool tud_configure(uint8_t port, unsigned id, const tud_configure_param_t *configuration) {
    assert(port == 0U && id == TUD_CFGID_DWC2 && !configuration->dwc2.vbus_sensing);
    return configure_ok;
}
bool tusb_init(uint8_t port, const tusb_rhport_init_t *configuration) {
    assert(port == 0U && configuration->role == TUSB_ROLE_DEVICE && configuration->speed == TUSB_SPEED_FULL);
    ++init_count; return init_ok;
}
bool tud_deinit(uint8_t port) {
    assert(port == 0U); ++deinit_count;
    configured = dtr = suspended = queued = endpoint_busy = false;
    rx_size = tx_size = 0U;
    return true;
}
bool tusb_deinit(uint8_t port) { return tud_deinit(port); }
bool tud_cdc_connected(void) { return configured && dtr; }
bool tud_suspended(void) { return suspended; }
bool usbd_edpt_busy(uint8_t port, uint8_t endpoint) {
    assert(port == 0U && endpoint == USB_CDC_EP_IN); return endpoint_busy;
}
uint32_t tud_cdc_write_available(void) { return sizeof(tx_fifo) - tx_size; }
uint32_t tud_cdc_write_flush(void) {
    if (endpoint_busy) return 0U;
    endpoint_length = tx_size;
    endpoint_busy = tx_size != 0U;
    const uint32_t sent = tx_size; tx_size = 0U;
    return sent;
}
uint32_t tud_cdc_write(const void *data, uint32_t length) {
    assert(primask == 1U && length <= sizeof(tx_fifo));
    const uint32_t written = length - (partial_write ? 1U : 0U);
    memcpy(tx_fifo, data, written); tx_size = written;
    /* Real TinyUSB flushes automatically when a full-size packet is queued. */
    if (tx_size == sizeof(tx_fifo)) (void)tud_cdc_write_flush();
    return written;
}
bool tud_cdc_write_clear(void) { tx_size = 0U; return true; }
void tud_cdc_read_flush(void) { rx_size = 0U; }
uint32_t tud_cdc_available(void) { return rx_size; }
uint32_t tud_cdc_read(void *data, uint32_t length) {
    assert(primask == 1U);
    if (length > rx_size) length = rx_size;
    memcpy(data, rx_fifo, length); rx_size -= length;
    memmove(rx_fifo, rx_fifo + length, rx_size);
    return length;
}
void tud_task_ext(uint32_t timeout, bool in_isr) {
    assert(timeout == 0U && !in_isr && primask == 0U); ++poll_count;
}
bool tud_task_event_ready(void) { return queued; }
void tud_int_handler(uint8_t port) {
    assert(port == 0U); ++irq_count;
    test_usb_registers.GINTSTS &= ~USB_OTG_GINTSTS_USBRST;
}
void serial_usb_link_reset(void) { ++reset_count; }
void serial_usb_rx_complete(const uint8_t *data, uint32_t length) {
    assert(primask == 0U && length <= sizeof(last_rx));
    memcpy(last_rx, data, length); received_length = length; ++rx_count;
}
void serial_usb_tx_complete(uint32_t token) { completed_token = token; ++tx_count; }

static void setup(void) {
    assert(primask == 0U);
    usb_cdc_device_deinit();
    configured = dtr = suspended = queued = endpoint_busy = partial_write = false;
    rx_size = tx_size = 0U;
    init_count = deinit_count = irq_count = poll_count = reset_count = rx_count = tx_count = 0U;
    configure_ok = init_ok = true;
    now_ms = 1000U;
    memset(&test_usb_registers, 0, sizeof(test_usb_registers));
    assert(usb_cdc_device_init());
    assert(!usb_cdc_device_open());
    configured = dtr = true;
    tud_cdc_line_state_cb(0U, true, false);
    usb_cdc_device_poll();
    assert(usb_cdc_device_open());
}
static void complete(void) {
    const bool zlp_needed = endpoint_length == 64U;
    endpoint_busy = false;
    tud_cdc_tx_complete_cb(0U);
    /* Matches cdcd_xfer_cb: callback precedes automatic zero-length packet. */
    if (zlp_needed) { endpoint_length = 0U; endpoint_busy = true; }
}
static void test_rx_permission_and_repeated_dtr(void) {
    setup();
    memset(rx_fifo, 0xA5, sizeof(rx_fifo)); rx_size = sizeof(rx_fifo);
    usb_cdc_device_poll();
    assert(rx_count == 0U && rx_size == 64U);
    assert(usb_cdc_device_arm_receive());
    assert(!usb_cdc_device_arm_receive());
    usb_cdc_device_poll();
    assert(rx_count == 1U && received_length == 64U && rx_size == 0U);
    for (unsigned i = 0U; i < 64U; ++i) assert(last_rx[i] == 0xA5);
    assert(usb_cdc_device_arm_receive());
    tud_cdc_line_state_cb(0U, true, true); /* RTS-only change must retain permission. */
    memset(rx_fifo, 0x55, sizeof(rx_fifo)); rx_size = 64U;
    usb_cdc_device_poll();
    assert(rx_count == 2U && last_rx[0] == 0x55);
}
static void test_tx_token_and_zlp(void) {
    setup();
    uint8_t bytes[64]; memset(bytes, 0xAA, sizeof(bytes));
    assert(!usb_cdc_device_transmit(NULL, 1U, 1U));
    assert(!usb_cdc_device_transmit(bytes, 0U, 1U));
    assert(!usb_cdc_device_transmit(bytes, 65U, 1U));
    assert(usb_cdc_device_transmit(bytes, 64U, 10U));
    assert(!usb_cdc_device_transmit(bytes, 8U, 11U));
    complete();
    assert(tx_count == 1U && completed_token == 10U && endpoint_busy);
    assert(!usb_cdc_device_transmit(bytes, 8U, 11U)); /* ZLP has no new token. */
    complete();
    assert(tx_count == 1U && !endpoint_busy);
    assert(usb_cdc_device_transmit(bytes, 8U, 11U));
    complete();
    assert(tx_count == 2U && completed_token == 11U);
    partial_write = true;
    assert(!usb_cdc_device_transmit(bytes, 8U, 12U));
    assert(tx_size == 0U);
    tud_cdc_tx_complete_cb(0U);
    assert(tx_count == 2U);
}
static void test_reset_barrier_and_stale_events(void) {
    setup();
    uint8_t bytes[16] = {0};
    assert(usb_cdc_device_transmit(bytes, sizeof(bytes), 20U));
    assert(usb_cdc_device_arm_receive());
    memcpy(rx_fifo, bytes, sizeof(bytes)); rx_size = sizeof(bytes);
    queued = true;
    tud_event_hook_cb(0U, DCD_EVENT_BUS_RESET, true);
    assert(!usb_cdc_device_open());
    assert(!usb_cdc_device_arm_receive() && !usb_cdc_device_transmit(bytes, 8U, 21U));
    complete(); /* Old completion processed while the epoch barrier is raised. */
    assert(tx_count == 0U);
    usb_cdc_device_poll();
    assert(!usb_cdc_device_open() && rx_count == 0U);
    queued = false;
    usb_cdc_device_poll();
    assert(rx_size == 0U && tx_size == 0U);
    assert(usb_cdc_device_open());
    assert(usb_cdc_device_transmit(bytes, 8U, 21U));
    complete();
    assert(tx_count == 1U && completed_token == 21U);
}
static void test_masks_start_failure_and_deinit(void) {
    setup();
    primask = 1U;
    assert(usb_cdc_device_arm_receive());
    assert(primask == 1U);
    primask = 0U;
    usb_cdc_device_irq_handler(); assert(irq_count == 1U);
    usb_cdc_device_deinit();
    assert(!usb_cdc_device_open());
    usb_cdc_device_irq_handler(); assert(irq_count == 1U);
    configure_ok = false;
    assert(!usb_cdc_device_init());
    assert(!usb_cdc_device_open());
    configure_ok = true; init_ok = false;
    assert(!usb_cdc_device_init());
    assert(!usb_cdc_device_open());
}
static void reenumerate(void) {
    configured = dtr = suspended = false;
    tud_event_hook_cb(0U, DCD_EVENT_BUS_RESET, true);
    usb_cdc_device_poll();
    configured = dtr = true;
    tud_cdc_line_state_cb(0U, true, false);
    usb_cdc_device_poll();
    assert(usb_cdc_device_open());
}
static void test_dtr_close_cancels_pending_endpoints(void) {
    setup();
    uint8_t bytes[16] = {0};
    assert(usb_cdc_device_transmit(bytes, sizeof(bytes), 30U));
    assert(usb_cdc_device_arm_receive());
    memcpy(rx_fifo, bytes, sizeof(bytes)); rx_size = sizeof(bytes);
    dtr = false;
    tud_cdc_line_state_cb(0U, false, false);
    assert(!usb_cdc_device_open());
    usb_cdc_device_poll();
    assert(deinit_count == 1U && !endpoint_busy && rx_size == 0U && tx_size == 0U);
    assert(!usb_cdc_device_open() && tx_count == 0U);
    /* Any old event can no longer acknowledge data after endpoint shutdown. */
    tud_cdc_tx_complete_cb(0U);
    assert(tx_count == 0U);
    now_ms += 99U; usb_cdc_device_poll();
    assert(init_count == 1U);
    now_ms += 1U; usb_cdc_device_poll();
    assert(init_count == 2U && !usb_cdc_device_open());
    reenumerate();
    assert(usb_cdc_device_transmit(bytes, sizeof(bytes), 31U));
    complete(); assert(tx_count == 1U && completed_token == 31U);
}
static void test_usbrst_before_enumdone_does_not_reopen(void) {
    setup();
    test_usb_registers.GINTSTS = test_usb_registers.GINTMSK = USB_OTG_GINTSTS_USBRST;
    usb_cdc_device_irq_handler();
    assert(irq_count == 1U && reset_count > 0U && !usb_cdc_device_open());
    assert(!queued); /* DWC2 has not emitted its later ENUMDNE reset event. */
    for (unsigned i = 0; i < 3U; ++i) usb_cdc_device_poll();
    assert(!usb_cdc_device_open());
    reenumerate();
}
static void test_initial_dtr_low_and_active_suspend(void) {
    setup(); usb_cdc_device_deinit();
    deinit_count = 0U; init_count = 0U;
    assert(usb_cdc_device_init());
    configured = true; dtr = false;
    tud_cdc_line_state_cb(0U, false, false);
    usb_cdc_device_poll();
    assert(init_count == 1U && deinit_count == 0U && !usb_cdc_device_open());
    dtr = true; tud_cdc_line_state_cb(0U, true, false); usb_cdc_device_poll();
    assert(usb_cdc_device_open());
    suspended = true;
    tud_event_hook_cb(0U, DCD_EVENT_SUSPEND, true);
    assert(!usb_cdc_device_open());
    usb_cdc_device_poll();
    assert(deinit_count == 1U && !usb_cdc_device_open());
    now_ms += 100U; usb_cdc_device_poll();
    assert(init_count == 2U);
    reenumerate();
}
int main(void) {
    test_rx_permission_and_repeated_dtr();
    test_tx_token_and_zlp();
    test_reset_barrier_and_stale_events();
    test_masks_start_failure_and_deinit();
    test_dtr_close_cancels_pending_endpoints();
    test_usbrst_before_enumdone_does_not_reopen();
    test_initial_dtr_low_and_active_suspend();
    puts("PASS: USB device wrapper RX permit, DTR state, TX token/ZLP, reset barrier and masks");
    return 0;
}
