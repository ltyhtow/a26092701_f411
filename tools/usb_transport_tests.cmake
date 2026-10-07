# Included after host_test(), BOARD and OS are defined by tools/CMakeLists.txt.
host_test(test_usb_transport test_usb_transport.c "${BOARD}/src/serial_transport_usb.c")
target_include_directories(test_usb_transport PRIVATE stubs_usb_transport "${OS}/include")
target_link_libraries(test_usb_transport PRIVATE lwrb)
add_test(NAME usb_transport COMMAND test_usb_transport)
