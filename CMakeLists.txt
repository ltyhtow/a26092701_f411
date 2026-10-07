cmake_minimum_required(VERSION 3.22)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
project(a26092701_f411 C ASM)
# Emit per-function stack sizes for the firmware regression budget check.
add_compile_options($<$<COMPILE_LANG_AND_ID:C,GNU>:-fstack-usage>)
option(ENCODER_PUSH_TEST "Passive encoder test with motors latched off" OFF)
option(EEPROM_ENABLED "AT24C08 EEPROM on I2C2 PB10/PB9 (requires wiring)" OFF)
option(WHEEL_TEST "Host-triggered suspended-wheel motor and encoder test" OFF)
option(SERIAL_TRANSPORT_USB_CDC "Native USB CDC on PA11/PA12; move motor PA11 wire to PB5" OFF)
include(cmake/modules.cmake)
add_subdirectory(cmake/stm32cubemx)
add_library(app_clock_port STATIC platform/freertos/src/app_clock.c)
target_include_directories(app_clock_port PUBLIC platform/freertos/include)
target_link_libraries(app_clock_port PUBLIC FreeRTOS PRIVATE monotonic_clock)
add_library(balance_freertos STATIC platform/freertos/src/balance_task.c)
target_include_directories(balance_freertos PUBLIC platform/freertos/include)
target_link_libraries(balance_freertos PUBLIC balance_core FreeRTOS PRIVATE app_clock_port)
add_library(imu_freertos STATIC platform/freertos/src/imu_task.c platform/freertos/src/imu_fusion_task.c)
target_include_directories(imu_freertos PUBLIC platform/freertos/include)
target_link_libraries(imu_freertos PUBLIC imu_fusion_core FreeRTOS PRIVATE app_clock_port)
add_library(serial_freertos STATIC platform/freertos/src/serial_protocol_adapter.c)
target_include_directories(serial_freertos PUBLIC platform/freertos/include)
target_link_libraries(serial_freertos PUBLIC protocol_engine FreeRTOS PRIVATE app_clock_port)
add_library(app_configuration INTERFACE)
target_include_directories(app_configuration INTERFACE app/include)
target_compile_definitions(app_configuration INTERFACE ENCODER_PUSH_TEST_ENABLED=$<BOOL:${ENCODER_PUSH_TEST}>)
target_compile_definitions(app_configuration INTERFACE WHEEL_TEST_ENABLED=$<BOOL:${WHEEL_TEST}>)
add_library(app_diagnostics STATIC app/diagnostics/src/encoder_push_test_task.c
    app/diagnostics/src/motor_polarity_test_task.c app/diagnostics/src/imu_uart_test_task.c)
target_sources(app_diagnostics PRIVATE app/diagnostics/src/wheel_test_task.c)
target_include_directories(app_diagnostics PUBLIC app/diagnostics/include)
target_link_libraries(app_diagnostics PUBLIC app_configuration FreeRTOS wheel_test_core
    PRIVATE app_clock_port robot_port_api imu_freertos serial_freertos)
add_library(robot_application OBJECT app/src/app_freertos.c app/src/app_protocol.c platform/freertos/src/failure_hooks.c)
target_sources(robot_application PRIVATE app/src/parameter_service.c)
target_include_directories(robot_application PRIVATE platform/stm32f411/include)
target_link_libraries(robot_application PRIVATE app_configuration app_diagnostics balance_freertos
    imu_freertos serial_freertos app_clock_port robot_port_api parameters_core)
add_executable(${PROJECT_NAME} Core/Src/main.c startup_stm32f411xe.s)
target_link_libraries(${PROJECT_NAME} PRIVATE robot_application stm32_board app_configuration app_diagnostics
    balance_freertos imu_freertos serial_freertos app_clock_port encoder_core parameters_core wheel_test_core STM32_Drivers FreeRTOS stm32_headers m)
target_include_directories(${PROJECT_NAME} PRIVATE middleware/motor/include)
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin)

# Check actual compiler stack reports on every build, including CLion builds.
# This bounds the known AHRS update path, not every possible task call path.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_custom_target(check_fusion_stack_budget ALL
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/check_fusion_stack_budget.py"
        --build-dir "${CMAKE_CURRENT_BINARY_DIR}"
        --task-source "${CMAKE_CURRENT_SOURCE_DIR}/platform/freertos/src/imu_fusion_task.c"
    DEPENDS imu_freertos imu_fusion_core fusion_vendor
    COMMENT "Checking IMUFusion stack budget from GCC reports"
    VERBATIM)
add_dependencies(${PROJECT_NAME} check_fusion_stack_budget)
if(SERIAL_TRANSPORT_USB_CDC)
    target_link_libraries(${PROJECT_NAME} PRIVATE tinyusb)
endif()
