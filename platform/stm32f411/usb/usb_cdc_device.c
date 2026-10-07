#include "usb_cdc_device.h"
#include "usb_descriptors.h"
#include "stm32f4xx_hal.h"
#include "tusb.h"
#include "device/dcd.h"
#include "device/usbd_pvt.h"

static volatile bool initialized;
static volatile bool stack_running;
static volatile bool link_barrier;
static volatile bool hardware_cancelled;
static volatile bool await_reset_event;
static volatile bool dtr_asserted;
static volatile bool reconnect_requested;
static bool reconnect_waiting;
static uint32_t reconnect_started_ms;
static volatile bool rx_armed;
static volatile bool tx_cancelled;
static bool tx_pending;
static uint32_t tx_token;
static uint8_t rx_packet[USB_CDC_PACKET_SIZE];

/* All class and transfer state belongs to the protocol task. Only the minimal
 * invalidation flags below can be written while TinyUSB queues an ISR event. */
static void invalidate_link(bool cancel_hardware) {
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    link_barrier = true;
    hardware_cancelled |= cancel_hardware;
    rx_armed = false;
    tx_cancelled = true;
    __set_PRIMASK(mask);
    serial_usb_link_reset(); /* Event only: may run in ISR context. */
}

uint32_t tusb_time_millis_api(void) { return HAL_GetTick(); }

static bool start_stack(void) {
    const tud_configure_param_t configuration = {.dwc2 = {.bm_double_buffered = 0, .vbus_sensing = false}};
    const tusb_rhport_init_t port = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    /* Initialization creates its event queue before enabling the controller IRQ. */
    stack_running = true;
    if (!tud_configure(0U, TUD_CFGID_DWC2, &configuration) || !tusb_init(0U, &port)) {
        NVIC_DisableIRQ(OTG_FS_IRQn);
        (void)tusb_deinit(0U);
        stack_running = false;
        return false;
    }
    return true;
}

static void stop_stack(void) {
    NVIC_DisableIRQ(OTG_FS_IRQn);
    if (stack_running) (void)tusb_deinit(0U);
    stack_running = false;
    NVIC_ClearPendingIRQ(OTG_FS_IRQn);
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    tx_pending = false;
    rx_armed = false;
    dtr_asserted = false;
    await_reset_event = false;
}

bool usb_cdc_device_init(void) {
    if (initialized) return true;
    /* GPIOA 9/10 remain motor pins: no VBUS sensing and no OTG ID input. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    GPIO_InitTypeDef pins = {0};
    pins.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    pins.Mode = GPIO_MODE_AF_PP;
    pins.Pull = GPIO_NOPULL;
    pins.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    pins.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &pins);
    NVIC_DisableIRQ(OTG_FS_IRQn);
    NVIC_ClearPendingIRQ(OTG_FS_IRQn);
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 6U, 0U);

    rx_armed = false;
    tx_pending = false;
    tx_cancelled = false;
    hardware_cancelled = false;
    await_reset_event = false;
    dtr_asserted = false;
    reconnect_requested = reconnect_waiting = false;
    link_barrier = false;
    initialized = true; /* The IRQ can be enabled by initialization itself. */
    if (!start_stack()) {
        usb_cdc_device_deinit();
        return false;
    }
    return true;
}

void usb_cdc_device_deinit(void) {
    if (!initialized) return;
    NVIC_DisableIRQ(OTG_FS_IRQn);
    invalidate_link(true);
    stop_stack();
    initialized = false;
    reconnect_requested = reconnect_waiting = false;
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
}

bool usb_cdc_device_open(void) {
    return initialized && stack_running && !link_barrier && !await_reset_event &&
           !reconnect_requested && !reconnect_waiting && tud_cdc_connected() && !tud_suspended();
}

bool usb_cdc_device_transmit(const uint8_t *data, uint16_t length, uint32_t token) {
    if (data == NULL || length == 0U || length > USB_CDC_PACKET_SIZE) return false;
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool accepted = false;
    /* A full-size packet is followed by TinyUSB's ZLP. Do not accept the next
     * token until that endpoint is idle, or the ZLP completion could falsely
     * acknowledge data that is still waiting in the CDC FIFO. */
    if (usb_cdc_device_open() && !tx_pending && !usbd_edpt_busy(0U, USB_CDC_EP_IN) &&
        tud_cdc_write_available() == CFG_TUD_CDC_TX_BUFSIZE) {
        tx_pending = true;
        tx_cancelled = false;
        tx_token = token;
        if (tud_cdc_write(data, length) == length) {
            (void)tud_cdc_write_flush();
            accepted = true;
        } else {
            tx_pending = false;
            (void)tud_cdc_write_clear();
        }
    }
    __set_PRIMASK(mask);
    return accepted;
}

bool usb_cdc_device_arm_receive(void) {
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool accepted = usb_cdc_device_open() && !rx_armed;
    if (accepted) rx_armed = true;
    __set_PRIMASK(mask);
    return accepted;
}

void usb_cdc_device_poll(void) {
    if (!initialized) return;
    if (reconnect_waiting) {
        if ((uint32_t)(HAL_GetTick() - reconnect_started_ms) < 100U) return;
        reconnect_waiting = false;
        hardware_cancelled = false;
        await_reset_event = true;
        /* Failure stays offline until an explicit transport reinitialization. */
        if (!start_stack()) return;
    }
    if (!stack_running) return;
    /* Bounded to 16 TinyUSB events per call; no waits and no printf. */
    tud_task_ext(0U, false);

    if (reconnect_requested) {
        /* Cancel BOTH hardware endpoints and all queued TinyUSB events. A FIFO
         * clear alone cannot retract a packet already armed in the OTG core.
         * Deinit runs only after the callback/event loop has returned. */
        stop_stack();
        reconnect_requested = false;
        link_barrier = true;
        reconnect_started_ms = HAL_GetTick();
        reconnect_waiting = true;
        return;
    }

    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (link_barrier && !await_reset_event && !tud_task_event_ready()) {
        /* At this point earlier completions and reset/deconfigure events have
         * been processed. Never reopen a session while old events are queued. */
        (void)tud_cdc_write_clear();
        tud_cdc_read_flush();
        if (hardware_cancelled) tx_pending = false;
        hardware_cancelled = false;
        link_barrier = false;
    }
    uint32_t received = 0U;
    if (rx_armed && usb_cdc_device_open() && tud_cdc_available() != 0U) {
        rx_armed = false;
        received = tud_cdc_read(rx_packet, sizeof(rx_packet));
    }
    __set_PRIMASK(mask);
    if (received != 0U) serial_usb_rx_complete(rx_packet, received);
}

void usb_cdc_device_irq_handler(void) {
    if (!initialized || !stack_running) return;
    /* DWC2 posts BUS_RESET only at ENUMDNE. Invalidate immediately at USBRST;
     * a temporarily empty task queue in between must not reopen the old link. */
    if ((USB_OTG_FS->GINTSTS & USB_OTG_FS->GINTMSK & USB_OTG_GINTSTS_USBRST) != 0U) {
        await_reset_event = true;
        dtr_asserted = false;
        invalidate_link(true);
    }
    tud_int_handler(0U);
}

void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr) {
    (void)rhport;
    (void)in_isr;
    if (eventid == DCD_EVENT_BUS_RESET || eventid == DCD_EVENT_UNPLUGGED) {
        await_reset_event = false;
        dtr_asserted = false;
        invalidate_link(true);
    } else if (eventid == DCD_EVENT_SUSPEND) {
        /* A live session is cancelled completely, including in-flight OUT data,
         * so a host resume can never deliver commands from before suspension. */
        if (dtr_asserted) reconnect_requested = true;
        invalidate_link(false);
    }
}

void tud_mount_cb(void) { /* DTR still gates the link after enumeration. */ }
void tud_umount_cb(void) { dtr_asserted = false; invalidate_link(true); }
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    /* The enqueue hook already invalidated this event in ISR context. */
}
void tud_resume_cb(void) { /* Old TX token remains cancelled until completion. */ }

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)rts;
    if (itf != 0U) return;
    const bool was_asserted = dtr_asserted;
    dtr_asserted = dtr;
    if (!dtr) {
        if (was_asserted) reconnect_requested = true;
        invalidate_link(false);
    } else if (!was_asserted) {
        /* Bytes received before this DTR session cannot become commands in it. */
        rx_armed = false;
        tud_cdc_read_flush();
    }
}

void tud_cdc_tx_complete_cb(uint8_t itf) {
    if (itf != 0U || !tx_pending) return;
    const uint32_t token = tx_token;
    const bool report = !tx_cancelled && usb_cdc_device_open();
    tx_pending = false;
    if (report) serial_usb_tx_complete(token);
}

void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    /* Keep the 64-byte TinyUSB RX FIFO full/NAKing until transport explicitly
     * arms an application read. No ring write or parsing inside this callback. */
}
