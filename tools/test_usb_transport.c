/* Production USB transport + real LwRB, deterministic device/RTOS boundaries. */
#include "serial_transport.h"
#include "usb_cdc_device.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

extern bool serial_transport_take_rx_reset(void);
extern uint32_t serial_transport_link_generation(void);

static unsigned critical, isr, notifications, yields;
static BaseType_t scheduler;
static TaskHandle_t current_task;
static unsigned init_calls, deinit_calls, poll_calls, arm_calls, tx_calls;
static bool init_ok, arm_ok, tx_ok, connected, rx_device_armed, tx_device_busy;
static bool synchronous_tx, synchronous_rx, reset_in_init, reenter_poll;
static uint32_t token_sent;
static uint16_t length_sent;
static const uint8_t *pointer_sent;
static uint8_t synchronous_data[64];

void test_enter_critical(void) { assert(isr == 0U); ++critical; }
void test_exit_critical(void) { assert(critical > 0U); --critical; }
void test_yield_from_isr(BaseType_t value) { assert(isr != 0U && value == pdTRUE); ++yields; }
uint32_t __get_IPSR(void) { return isr; }
uint32_t __get_PRIMASK(void) { return 0U; }
uint32_t __get_BASEPRI(void) { return critical == 0U ? 0U : 0x50U; }
BaseType_t xTaskGetSchedulerState(void) { return scheduler; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return current_task; }
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken) {
    assert(isr != 0U && scheduler == taskSCHEDULER_RUNNING && task == (void *)1);
    ++notifications; *woken = pdTRUE;
}
bool usb_cdc_device_init(void) {
    assert(critical == 0U && isr == 0U && scheduler == taskSCHEDULER_RUNNING);
    ++init_calls;
    if (reset_in_init) serial_usb_link_reset();
    return init_ok;
}
void usb_cdc_device_deinit(void) {
    assert(critical == 0U);
    ++deinit_calls;
    connected = rx_device_armed = tx_device_busy = false;
    serial_usb_link_reset();
}
void usb_cdc_device_poll(void) {
    assert(critical == 0U && isr == 0U);
    ++poll_calls;
    if (reenter_poll) {
        reenter_poll = false;
        serial_transport_poll_tx();
    }
}
bool usb_cdc_device_open(void) { return connected; }
bool usb_cdc_device_arm_receive(void) {
    assert(critical > 0U && connected && !rx_device_armed);
    ++arm_calls;
    if (!arm_ok) return false;
    rx_device_armed = true;
    if (synchronous_rx) {
        synchronous_rx = false;
        rx_device_armed = false;
        serial_usb_rx_complete(synchronous_data, sizeof(synchronous_data));
    }
    return true;
}
bool usb_cdc_device_transmit(const uint8_t *data, uint16_t length, uint32_t token) {
    assert(critical > 0U && connected && data != NULL && length > 0U && length <= 64U);
    ++tx_calls;
    if (!tx_ok || tx_device_busy) return false;
    pointer_sent = data; length_sent = length; token_sent = token;
    tx_device_busy = true;
    if (synchronous_tx) {
        tx_device_busy = false;
        serial_usb_tx_complete(token);
    }
    return true;
}

static void initialize(void) {
    assert(critical == 0U && isr == 0U);
    serial_transport_deinit();
    scheduler = taskSCHEDULER_NOT_STARTED;
    current_task = (void *)1;
    init_calls = deinit_calls = poll_calls = arm_calls = tx_calls = 0U;
    notifications = yields = 0U;
    init_ok = arm_ok = tx_ok = true;
    connected = rx_device_armed = tx_device_busy = false;
    synchronous_tx = synchronous_rx = reset_in_init = reenter_poll = false;
    pointer_sent = NULL; length_sent = 0;
    assert(serial_transport_init(NULL) == pdFAIL);
    assert(serial_transport_init((void *)1) == pdPASS);
    assert(serial_transport_init((void *)1) == pdFAIL);
    assert(init_calls == 0U);
}
static void connect(void) {
    scheduler = taskSCHEDULER_RUNNING;
    connected = true;
    serial_transport_poll_tx();
    assert(init_calls == 1U && rx_device_armed);
    assert(serial_transport_take_rx_reset());
    assert(!serial_transport_take_rx_reset());
}
static void receive(const uint8_t *data, uint32_t length) {
    assert(rx_device_armed && critical == 0U);
    rx_device_armed = false;
    isr = 83U;
    serial_usb_rx_complete(data, length);
    isr = 0U;
}
static void complete(uint32_t token) {
    assert(critical == 0U);
    if (token == token_sent) tx_device_busy = false;
    isr = 83U;
    serial_usb_tx_complete(token);
    isr = 0U;
}
static void reset_link(void) {
    assert(critical == 0U);
    connected = false;
    /* Models low-level endpoint cancellation before the reset notification. */
    rx_device_armed = tx_device_busy = false;
    isr = 83U;
    serial_usb_link_reset();
    isr = 0U;
}

static void test_deferred_init_and_context(void) {
    initialize();
    serial_transport_poll_tx();
    scheduler = taskSCHEDULER_SUSPENDED;
    serial_transport_poll_tx();
    assert(init_calls == 0U);
    scheduler = taskSCHEDULER_NOT_STARTED;
    isr = 83U; serial_usb_link_reset(); isr = 0U;
    assert(notifications == 0U);
    reset_in_init = true;
    connect();
    assert(notifications == 0U); /* init callback was task context */
    const unsigned before = poll_calls;
    taskENTER_CRITICAL(); serial_transport_poll_tx(); taskEXIT_CRITICAL();
    assert(poll_calls == before); /* No tud_task under an outer protocol lock. */
    reenter_poll = true;
    serial_transport_poll_tx();
    assert(poll_calls == before + 1U);
    const uint8_t byte = 0xAA;
    receive(&byte, 1U);
    assert(notifications == 1U && yields == 1U);
}

static void test_failed_init_is_latched(void) {
    initialize(); init_ok = false; scheduler = taskSCHEDULER_RUNNING;
    for (unsigned i = 0; i < 10U; ++i) serial_transport_poll_tx();
    assert(init_calls == 1U && deinit_calls == 1U && arm_calls == 0U && poll_calls == 0U);
    init_ok = true; connected = true;
    serial_transport_poll_tx();
    assert(init_calls == 1U && arm_calls == 0U);
}

static void test_non_owner_never_runs_usb(void) {
    initialize(); scheduler = taskSCHEDULER_RUNNING; connected = true;
    current_task = (void *)2;
    serial_transport_poll_tx();
    assert(init_calls == 0U && poll_calls == 0U && arm_calls == 0U && tx_calls == 0U);
    assert(!serial_transport_take_rx_reset());
    current_task = (void *)1;
    connect();
    const unsigned polls = poll_calls;
    const uint8_t data[] = {0xAA, 6, 4, 1, 1, 2, 0};
    assert(lwrb_write(serial_transport_tx_buffer(), data, sizeof(data)) == sizeof(data));
    current_task = (void *)2;
    serial_transport_poll_tx();
    assert(poll_calls == polls && tx_calls == 0U && lwrb_get_full(serial_transport_tx_buffer()) == sizeof(data));
    current_task = (void *)1;
    serial_transport_poll_tx();
    assert(tx_calls == 1U && tx_device_busy);
}

static void test_rx_backpressure_and_copy(void) {
    initialize(); connect();
    uint8_t bytes[64], actual[255];
    for (unsigned i = 0; i < sizeof(bytes); ++i) bytes[i] = (uint8_t)i;
    for (unsigned i = 0; i < 3U; ++i) { receive(bytes, sizeof(bytes)); serial_transport_poll_tx(); }
    assert(lwrb_get_full(serial_transport_rx_buffer()) == 192U);
    assert(!rx_device_armed && arm_calls == 3U); /* 63 free is insufficient. */
    memset(bytes, 0xEE, sizeof(bytes)); /* Driver's reusable packet buffer is not retained. */
    assert(lwrb_read(serial_transport_rx_buffer(), actual, 64U) == 64U);
    for (unsigned i = 0; i < 64U; ++i) assert(actual[i] == i);
    serial_transport_poll_tx();
    assert(rx_device_armed && arm_calls == 4U);
    receive(bytes, sizeof(bytes));
    assert(lwrb_get_full(serial_transport_rx_buffer()) == 192U);
    assert(lwrb_read(serial_transport_rx_buffer(), actual, sizeof(actual)) == 192U);
    for (unsigned i = 0; i < 128U; ++i) assert(actual[i] == i % 64U);
    for (unsigned i = 128; i < 192U; ++i) assert(actual[i] == 0xEE);
}

static void test_rx_arm_busy_and_synchronous_callback(void) {
    initialize(); arm_ok = false; scheduler = taskSCHEDULER_RUNNING; connected = true;
    serial_transport_poll_tx();
    assert(arm_calls == 1U && !rx_device_armed);
    arm_ok = true; synchronous_rx = true;
    memset(synchronous_data, 0xA5, sizeof(synchronous_data));
    serial_transport_poll_tx();
    assert(lwrb_get_full(serial_transport_rx_buffer()) == 64U && !rx_device_armed);
    assert(notifications == 0U);
    serial_transport_poll_tx();
    assert(rx_device_armed);
    receive(NULL, 0U);
    assert(lwrb_get_full(serial_transport_rx_buffer()) == 64U);
}

static void test_tx_busy_completion_and_buffer_lifetime(void) {
    initialize(); connect();
    uint8_t first[64], second[32];
    memset(first, 0xAA, sizeof(first)); memset(second, 0x55, sizeof(second));
    assert(lwrb_write(serial_transport_tx_buffer(), first, sizeof(first)) == sizeof(first));
    tx_ok = false;
    serial_transport_poll_tx(); serial_transport_poll_tx();
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 64U && !tx_device_busy);
    tx_ok = true;
    serial_transport_poll_tx();
    assert(tx_device_busy && length_sent == 64U && memcmp(pointer_sent, first, 64U) == 0);
    const uint32_t old_token = token_sent;
    assert(lwrb_write(serial_transport_tx_buffer(), second, sizeof(second)) == sizeof(second));
    serial_transport_poll_tx();
    assert(memcmp(pointer_sent, first, 64U) == 0 && lwrb_get_full(serial_transport_tx_buffer()) == 96U);
    complete(old_token);
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 96U); /* IRQ does not skip. */
    serial_transport_poll_tx();
    assert(token_sent != old_token && length_sent == 32U && memcmp(pointer_sent, second, 32U) == 0);
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 32U);
    complete(old_token); serial_transport_poll_tx(); /* Delayed duplicate cannot consume transfer 2. */
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 32U);
    complete(token_sent); serial_transport_poll_tx();
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 0U);
}

static void test_synchronous_tx_and_wrap(void) {
    initialize(); connect(); synchronous_tx = true;
    uint8_t bytes[200]; memset(bytes, 0x7E, sizeof(bytes));
    assert(lwrb_write(serial_transport_tx_buffer(), bytes, 200U) == 200U);
    for (unsigned i = 0; i < 4U; ++i) serial_transport_poll_tx();
    assert(length_sent == 8U);
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 0U && notifications == 0U);
    assert(lwrb_write(serial_transport_tx_buffer(), bytes, 100U) == 100U);
    serial_transport_poll_tx();
    assert(length_sent == 56U && lwrb_get_full(serial_transport_tx_buffer()) == 44U);
    serial_transport_poll_tx();
    assert(length_sent == 44U && lwrb_get_full(serial_transport_tx_buffer()) == 0U);
}

static void test_reset_disconnect_and_reconnect(void) {
    initialize(); connect();
    uint8_t old_data[32], new_data[32];
    memset(old_data, 0xAA, sizeof(old_data)); memset(new_data, 0x55, sizeof(new_data));
    receive(old_data, sizeof(old_data));
    assert(lwrb_write(serial_transport_tx_buffer(), old_data, sizeof(old_data)) == sizeof(old_data));
    serial_transport_poll_tx();
    const uint32_t old_token = token_sent, old_generation = serial_transport_link_generation();
    reset_link();
    assert(serial_transport_link_generation() != old_generation);
    assert(lwrb_get_full(serial_transport_rx_buffer()) == 32U); /* ISR never resets LwRB. */
    assert(serial_transport_take_rx_reset());
    assert(lwrb_get_full(serial_transport_rx_buffer()) == 0U && lwrb_get_full(serial_transport_tx_buffer()) == 0U);
    complete(old_token);
    assert(lwrb_write(serial_transport_tx_buffer(), old_data, sizeof(old_data)) == sizeof(old_data));
    serial_transport_poll_tx();
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 0U); /* Closed-port discard. */
    assert(lwrb_write(serial_transport_tx_buffer(), old_data, sizeof(old_data)) == sizeof(old_data));
    connected = true;
    const unsigned starts = tx_calls;
    serial_transport_poll_tx();
    assert(serial_transport_take_rx_reset() && tx_calls == starts); /* Rising edge also flushes. */
    assert(lwrb_write(serial_transport_tx_buffer(), new_data, sizeof(new_data)) == sizeof(new_data));
    serial_transport_poll_tx();
    const uint32_t new_token = token_sent;
    assert(new_token != old_token && memcmp(pointer_sent, new_data, 32U) == 0);
    complete(old_token); serial_transport_poll_tx();
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 32U);
    complete(new_token); serial_transport_poll_tx();
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 0U);
    /* Suspend/DTR close/reset share exactly this session-invalidating contract. */
    reset_link(); serial_transport_poll_tx();
    assert(serial_transport_take_rx_reset() && !rx_device_armed);
}

static void test_deinit_blocks_late_callbacks(void) {
    initialize(); connect();
    uint8_t bytes[8] = {0};
    assert(lwrb_write(serial_transport_tx_buffer(), bytes, sizeof(bytes)) == sizeof(bytes));
    serial_transport_poll_tx();
    const uint32_t stale_token = token_sent;
    serial_transport_deinit();
    const unsigned count = notifications;
    isr = 83U;
    serial_usb_rx_complete(bytes, sizeof(bytes));
    serial_usb_tx_complete(stale_token);
    serial_usb_link_reset();
    isr = 0U;
    assert(notifications == count && lwrb_get_full(serial_transport_rx_buffer()) == 0U);
    assert(lwrb_get_full(serial_transport_tx_buffer()) == 0U);
    assert(!serial_transport_take_rx_reset());
}

int main(void) {
    test_deferred_init_and_context();
    test_failed_init_is_latched();
    test_non_owner_never_runs_usb();
    test_rx_backpressure_and_copy();
    test_rx_arm_busy_and_synchronous_callback();
    test_tx_busy_completion_and_buffer_lifetime();
    test_synchronous_tx_and_wrap();
    test_reset_disconnect_and_reconnect();
    test_deinit_blocks_late_callbacks();
    puts("PASS: USB transport deferred init, RX backpressure, token completion, reset epochs and buffer lifetime");
    return 0;
}
