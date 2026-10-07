host_test(test_usb_device test_usb_device.c "${BOARD}/usb/usb_cdc_device.c")
target_include_directories(test_usb_device PRIVATE stubs_usb_device "${BOARD}/usb")
add_test(NAME usb_device COMMAND test_usb_device)
