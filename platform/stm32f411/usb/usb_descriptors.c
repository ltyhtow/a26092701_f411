#include "tusb.h"
#include "usb_descriptors.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* Development VID/PID follow the TinyUSB CDC example convention. An assigned
 * product VID/PID can be substituted here before distributing USB hardware. */
static const tusb_desc_device_t device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_CDC_VENDOR_ID, .idProduct = USB_CDC_PRODUCT_ID,
    .bcdDevice = 0x0100, .iManufacturer = 1, .iProduct = 2, .iSerialNumber = 3,
    .bNumConfigurations = 1
};

enum { INTERFACE_CDC = 0, INTERFACE_CDC_DATA, INTERFACE_COUNT };
static const uint8_t configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, INTERFACE_COUNT, 0, TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(INTERFACE_CDC, 4, USB_CDC_EP_NOTIFY, 8, USB_CDC_EP_OUT, USB_CDC_EP_IN, 64)
};

uint8_t const *tud_descriptor_device_cb(void) { return (const uint8_t *)&device_descriptor; }
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    return index == 0U ? configuration_descriptor : NULL;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t descriptor[33];
    static const char *const strings[] = {NULL, "Balance Car Project", "F411 Balance CDC", NULL, "LwPKT protocol"};
    uint8_t count = 0;
    if (index == 0U) {
        descriptor[1] = 0x0409;
        count = 1;
    } else if (index == 3U) {
        const uint32_t uid[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
        static const char hex[] = "0123456789ABCDEF";
        for (unsigned word = 0; word < 3U; ++word) {
            for (unsigned digit = 0; digit < 8U; ++digit) {
                descriptor[++count] = (uint16_t)hex[(uid[word] >> (28U - 4U * digit)) & 15U];
            }
        }
    } else {
        if (index >= sizeof(strings) / sizeof(strings[0]) || strings[index] == NULL) return NULL;
        const size_t length = strlen(strings[index]);
        count = (uint8_t)(length > 32U ? 32U : length);
        for (uint8_t i = 0; i < count; ++i) descriptor[i + 1U] = (uint8_t)strings[index][i];
    }
    descriptor[0] = (uint16_t)((TUSB_DESC_STRING << 8U) | (2U * count + 2U));
    return descriptor;
}
