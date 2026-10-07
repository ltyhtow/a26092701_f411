# Optional native CDC backend. TinyUSB is a pinned, unmodified MIT vendor subset.
set(TINYUSB_ROOT "${PROJECT_SOURCE_DIR}/middleware/usb_device/third_party/tinyusb")
set(USB_BOARD "${PROJECT_SOURCE_DIR}/platform/stm32f411/usb")
add_library(tinyusb STATIC
    "${TINYUSB_ROOT}/src/tusb.c"
    "${TINYUSB_ROOT}/src/common/tusb_fifo.c"
    "${TINYUSB_ROOT}/src/device/usbd.c"
    "${TINYUSB_ROOT}/src/class/cdc/cdc_device.c"
    "${TINYUSB_ROOT}/src/portable/synopsys/dwc2/dcd_dwc2.c"
    "${TINYUSB_ROOT}/src/portable/synopsys/dwc2/dwc2_common.c")
target_include_directories(tinyusb PUBLIC "${TINYUSB_ROOT}/src" "${USB_BOARD}")
target_link_libraries(tinyusb PRIVATE stm32_headers)
target_sources(stm32_board PRIVATE
    "${USB_BOARD}/usb_cdc_device.c"
    "${USB_BOARD}/usb_descriptors.c")
target_link_libraries(stm32_board PRIVATE tinyusb)
