host_test(test_motor_usb test_motor_usb.c "${BOARD}/src/motor_driver_stm32.c"
    "${BOARD}/src/board_motor_diagnostics_stm32.c" "${PROJECT_ROOT}/Core/Src/tim.c")
target_include_directories(test_motor_usb PRIVATE test_motor_usb_stubs "${BOARD}/include")
target_link_libraries(test_motor_usb PRIVATE robot_port_api)
target_compile_definitions(test_motor_usb PRIVATE SERIAL_TRANSPORT_USB_CDC=1)
foreach(case normal cold_fault running_fault direct_gpio missing_tim3_clock start_failure_1 start_failure_2 start_failure_3 start_failure_4)
    add_test(NAME motor_usb_${case} COMMAND test_motor_usb ${case})
endforeach()
