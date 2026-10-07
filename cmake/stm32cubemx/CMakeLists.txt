cmake_minimum_required(VERSION 3.22)
# Board-generated sources only; no application or portable middleware.
set(ROOT "${PROJECT_SOURCE_DIR}")
set(HAL "${ROOT}/Drivers/STM32F4xx_HAL_Driver")
set(RTOS "${ROOT}/Middlewares/Third_Party/FreeRTOS/Source")
add_library(stm32_headers INTERFACE)
target_compile_definitions(stm32_headers INTERFACE USE_HAL_DRIVER STM32F411xE)
target_compile_definitions(stm32_headers INTERFACE SERIAL_TRANSPORT_USB_CDC=$<BOOL:${SERIAL_TRANSPORT_USB_CDC}>)
target_include_directories(stm32_headers INTERFACE "${ROOT}/Core/Inc" "${HAL}/Inc" "${HAL}/Inc/Legacy"
    "${ROOT}/Drivers/CMSIS/Device/ST/STM32F4xx/Include" "${ROOT}/Drivers/CMSIS/Include")
set(HAL_COMPONENTS tim tim_ex uart i2c i2c_ex adc adc_ex rcc rcc_ex flash flash_ex
    flash_ramfunc gpio dma dma_ex pwr pwr_ex cortex exti)
set(HAL_SOURCES "${HAL}/Src/stm32f4xx_hal.c")
foreach(component IN LISTS HAL_COMPONENTS)
    list(APPEND HAL_SOURCES "${HAL}/Src/stm32f4xx_hal_${component}.c")
endforeach()
add_library(STM32_Drivers STATIC ${HAL_SOURCES})
target_link_libraries(STM32_Drivers PRIVATE stm32_headers)
add_library(FreeRTOS STATIC "${RTOS}/croutine.c" "${RTOS}/event_groups.c" "${RTOS}/list.c" "${RTOS}/queue.c"
    "${RTOS}/stream_buffer.c" "${RTOS}/tasks.c" "${RTOS}/timers.c"
    "${RTOS}/CMSIS_RTOS_V2/cmsis_os2.c" "${RTOS}/portable/MemMang/heap_4.c" "${RTOS}/portable/GCC/ARM_CM4F/port.c")
target_include_directories(FreeRTOS PUBLIC "${RTOS}/include" "${RTOS}/portable/GCC/ARM_CM4F"
    "${ROOT}/platform/freertos/config" PRIVATE "${RTOS}/CMSIS_RTOS_V2")
target_link_libraries(FreeRTOS PRIVATE stm32_headers)
add_library(stm32_board OBJECT "${ROOT}/Core/Src/adc.c" "${ROOT}/Core/Src/dma.c" "${ROOT}/Core/Src/gpio.c"
    "${ROOT}/Core/Src/i2c.c" "${ROOT}/Core/Src/tim.c" "${ROOT}/Core/Src/usart.c"
    "${ROOT}/Core/Src/stm32f4xx_it.c" "${ROOT}/Core/Src/stm32f4xx_hal_msp.c"
    "${ROOT}/Core/Src/stm32f4xx_hal_timebase_tim.c" "${ROOT}/Core/Src/system_stm32f4xx.c"
    "${ROOT}/Core/Src/syscalls.c" "${ROOT}/Core/Src/sysmem.c"
    "${ROOT}/platform/stm32f411/src/motor_driver_stm32.c"
    "${ROOT}/platform/stm32f411/src/encoder_driver_stm32.c"
    "${ROOT}/platform/stm32f411/src/mpu6050_port_stm32.c"
    "${ROOT}/platform/stm32f411/src/board_encoder_diagnostics_stm32.c"
    "${ROOT}/platform/stm32f411/src/board_motor_diagnostics_stm32.c"
    "${ROOT}/platform/stm32f411/src/board_status_stm32.c"
    "${ROOT}/middleware/mpu6050/third_party/libdriver/src/driver_mpu6050.c")
target_include_directories(stm32_board PRIVATE "${ROOT}/platform/stm32f411/include"
    "${ROOT}/platform/freertos/include" "${ROOT}/middleware/mpu6050/third_party/libdriver/src")
target_link_libraries(stm32_board PRIVATE stm32_headers FreeRTOS robot_port_api encoder_core lwrb)
if(SERIAL_TRANSPORT_USB_CDC)
    include("${ROOT}/cmake/usb_device.cmake")
    target_sources(stm32_board PRIVATE "${ROOT}/platform/stm32f411/src/serial_transport_usb.c")
    target_include_directories(stm32_board PRIVATE "${ROOT}/platform/stm32f411/usb")
else()
    target_sources(stm32_board PRIVATE "${ROOT}/platform/stm32f411/src/serial_transport_stm32.c")
endif()
target_sources(stm32_board PRIVATE "${ROOT}/platform/stm32f411/src/fault_capture_stm32.c")
set_source_files_properties("${ROOT}/platform/stm32f411/src/fault_capture_stm32.c"
    "${ROOT}/Core/Src/stm32f4xx_it.c" PROPERTIES
    COMPILE_OPTIONS "-mgeneral-regs-only;-fno-stack-protector;-fno-instrument-functions")
target_sources(stm32_board PRIVATE "${ROOT}/platform/stm32f411/src/eeprom_at24c08_stm32.c")
target_compile_definitions(stm32_board PRIVATE EEPROM_ENABLED=$<BOOL:${EEPROM_ENABLED}>)
if(EEPROM_ENABLED)
    target_sources(stm32_board PRIVATE "${ROOT}/middleware/eeprom/third_party/libdriver/driver_at24cxx.c")
    target_include_directories(stm32_board PRIVATE "${ROOT}/middleware/eeprom/third_party/libdriver")
endif()
