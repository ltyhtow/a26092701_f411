/* CDC byte transport. Framing remains serial_codec -> LwPKT -> LwRB. */
#include "serial_transport.h"
#include "usb_cdc_device.h"
#include "stm32f4xx.h"

#include <stdbool.h>
#include <stdint.h>

#define USB_SERIAL_RING_SIZE 256U
#define USB_OUT_PACKET_SIZE 64U

static lwrb_t rx_ring, tx_ring;
static uint8_t rx_storage[USB_SERIAL_RING_SIZE], tx_storage[USB_SERIAL_RING_SIZE];
static TaskHandle_t volatile notify_task;
static volatile bool initialized, device_ready;
static bool init_attempted, observed_open, device_poll_active;
static volatile bool reset_pending, rx_reset_pending, rx_armed;
static volatile bool tx_active, tx_complete_pending;
static uint16_t tx_length;
static uint32_t next_token;
static volatile uint32_t tx_token, generation;

static void notify_from_callback(void) {
    /* Device init/deinit and synchronous driver completion may call back from
     * task context. In that case the owner is already servicing the transport. */
    if (initialized && notify_task != NULL && __get_IPSR() != 0U &&
        xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(notify_task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

void serial_usb_link_reset(void) {
    if (!initialized) return;
    /* USB must quiesce the endpoints before this callback. Never reset ring
     * indices here: the ISR can interrupt an ongoing LwPKT read/write. */
    ++generation;
    reset_pending = true;
    rx_armed = false;
    tx_active = tx_complete_pending = false;
    notify_from_callback();
}

void serial_usb_rx_complete(const uint8_t *data, uint32_t length) {
    if (!initialized || !rx_armed) return;
    rx_armed = false;
    if (!reset_pending && device_ready && usb_cdc_device_open() &&
        length <= USB_OUT_PACKET_SIZE && (length == 0U || data != NULL) &&
        lwrb_get_free(&rx_ring) >= length) {
        if (length != 0U) (void)lwrb_write(&rx_ring, data, length);
    }
    /* Re-arming happens in the owner task only, with a whole packet reserved. */
    notify_from_callback();
}

void serial_usb_tx_complete(uint32_t token) {
    if (!initialized || reset_pending || !tx_active || token != tx_token) return;
    tx_complete_pending = true;
    notify_from_callback();
}

static void clear_buffers_locked(void) {
    lwrb_reset(&rx_ring);
    lwrb_reset(&tx_ring);
    rx_armed = false;
    tx_active = tx_complete_pending = false;
    tx_length = 0U;
}

static bool service_link_locked(void) {
    const bool open = device_ready && usb_cdc_device_open();
    if (reset_pending || open != observed_open) {
        /* A rise also discards packets queued while the port was closed. */
        if (open != observed_open) ++generation;
        clear_buffers_locked();
        reset_pending = false;
        rx_reset_pending = true;
        observed_open = open;
    } else if (!open) {
        /* Offline telemetry must not consume capacity or replay on reconnect. */
        clear_buffers_locked();
    }
    return open;
}

static void finish_tx_locked(void) {
    if (tx_active && tx_complete_pending) {
        (void)lwrb_skip(&tx_ring, tx_length);
        tx_active = tx_complete_pending = false;
        tx_length = 0U;
    }
}

BaseType_t serial_transport_init(TaskHandle_t task) {
    if (initialized || task == NULL) return pdFAIL;
    if (!lwrb_init(&rx_ring, rx_storage, sizeof(rx_storage)) ||
        !lwrb_init(&tx_ring, tx_storage, sizeof(tx_storage))) return pdFAIL;
    notify_task = task;
    init_attempted = device_ready = observed_open = false;
    device_poll_active = false;
    reset_pending = rx_armed = tx_active = tx_complete_pending = false;
    rx_reset_pending = true;
    tx_length = 0U;
    ++generation;
    initialized = true;
    /* No USB IRQ/peripheral starts before the scheduler is running. */
    return pdPASS;
}

void serial_transport_deinit(void) {
    taskENTER_CRITICAL();
    const bool attempted = init_attempted;
    initialized = false;
    notify_task = NULL;
    ++generation;
    taskEXIT_CRITICAL();
    if (attempted) usb_cdc_device_deinit();
    taskENTER_CRITICAL();
    if (lwrb_is_ready(&rx_ring) && lwrb_is_ready(&tx_ring)) clear_buffers_locked();
    init_attempted = device_ready = observed_open = reset_pending = false;
    rx_reset_pending = true;
    taskEXIT_CRITICAL();
}

lwrb_t *serial_transport_rx_buffer(void) { return &rx_ring; }
lwrb_t *serial_transport_tx_buffer(void) { return &tx_ring; }

uint32_t serial_transport_link_generation(void) { return generation; }

bool serial_transport_take_rx_reset(void) {
    if (!initialized || __get_IPSR() != 0U || xTaskGetCurrentTaskHandle() != notify_task) return false;
    taskENTER_CRITICAL();
    (void)service_link_locked();
    const bool pending = rx_reset_pending;
    rx_reset_pending = false;
    taskEXIT_CRITICAL();
    return pending;
}

void serial_transport_poll_tx(void) {
    if (!initialized || __get_IPSR() != 0U ||
        xTaskGetSchedulerState() != taskSCHEDULER_RUNNING ||
        xTaskGetCurrentTaskHandle() != notify_task) return;
    /* serial_protocol_send can also be called by control/diagnostic workers.
     * Their bounded stacks must never inherit TinyUSB initialization or its
     * event loop. Only the protocol task consumes queued bytes every 5 ms. */
    const bool unmasked = __get_PRIMASK() == 0U && __get_BASEPRI() == 0U;
    if (!init_attempted && unmasked) {
        /* Initialization may wait for the peripheral; never do it with
         * interrupts masked. A failed attempt stays failed until deinit/init. */
        init_attempted = true;
        device_ready = usb_cdc_device_init();
        if (!device_ready) usb_cdc_device_deinit();
    }
    /* TinyUSB defers device events to its task poll. A protocol receive
     * callback may submit a response from a short outer critical section;
     * it must not recursively run the USB event loop with IRQs masked. */
    if (device_ready && unmasked && !device_poll_active) {
        device_poll_active = true;
        usb_cdc_device_poll();
        device_poll_active = false;
    }
    taskENTER_CRITICAL();
    if (!service_link_locked()) { taskEXIT_CRITICAL(); return; }
    finish_tx_locked();
    if (!rx_armed && lwrb_get_free(&rx_ring) >= USB_OUT_PACKET_SIZE) {
        rx_armed = true; /* Publish first to permit synchronous completion. */
        if (!usb_cdc_device_arm_receive()) rx_armed = false;
    }
    if (!reset_pending && !tx_active && usb_cdc_device_open()) {
        const lwrb_sz_t length = lwrb_get_linear_block_read_length(&tx_ring);
        const uint8_t *data = lwrb_get_linear_block_read_address(&tx_ring);
        if (length != 0U && data != NULL) {
            /* The CDC wrapper associates one token with at most one 64-byte
             * USB data packet. LwPKT frames may span any number of packets. */
            tx_length = (uint16_t)(length > USB_OUT_PACKET_SIZE ? USB_OUT_PACKET_SIZE : length);
            tx_token = ++next_token;
            tx_complete_pending = false;
            tx_active = true;
            if (!usb_cdc_device_transmit(data, tx_length, tx_token)) {
                tx_active = tx_complete_pending = false;
                tx_length = 0U;
            } else {
                /* Only an accepted transfer with its matching completion can
                 * consume bytes, including completion during transmit(). */
                finish_tx_locked();
            }
        }
    }
    taskEXIT_CRITICAL();
}
