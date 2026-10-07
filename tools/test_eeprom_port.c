/* Tests the unmodified upstream driver through the actual STM32 adapter. */
#include "eeprom_port.h"
#include "eeprom_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#if EEPROM_ENABLED
#include "stm32f4xx_hal.h"
#include "app_clock.h"
#include "task.h"

I2C_TypeDef mock_i2c2;
GPIO_TypeDef mock_gpiob;
static I2C_HandleTypeDef *owned_bus;
static uint8_t memory[EEPROM_PORT_CAPACITY_BYTES];
static unsigned clocks, gpio_calls, init_calls, deinit_calls, reads, writes;
static unsigned delays, hal_delays, fail_read, fail_write;
static uint32_t ipsr, primask;
static bool fail_init;
static BaseType_t scheduler = taskSCHEDULER_RUNNING;
static uint16_t last_address, last_register, last_length;

void mock_eeprom_clock(unsigned operation) {
    assert(operation == clocks % 4U);
    ++clocks;
}
uint32_t __get_IPSR(void) { return ipsr; }
uint32_t __get_PRIMASK(void) { return primask; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config) {
    assert(port == GPIOB && config->Mode == GPIO_MODE_AF_OD);
    assert(config->Pull == GPIO_NOPULL && config->Speed == GPIO_SPEED_FREQ_LOW);
    assert(config->Pin == (gpio_calls % 2U == 0U ? GPIO_PIN_10 : GPIO_PIN_9));
    assert(config->Alternate == (gpio_calls % 2U == 0U ? GPIO_AF4_I2C2 : GPIO_AF9_I2C2));
    ++gpio_calls;
}
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *bus) {
    assert(bus->Instance == I2C2 && bus->Init.ClockSpeed == 100000U);
    assert(bus->Init.DutyCycle == I2C_DUTYCYCLE_2 && bus->Init.OwnAddress1 == 0U);
    assert(bus->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT);
    assert(bus->Init.DualAddressMode == I2C_DUALADDRESS_DISABLE && bus->Init.OwnAddress2 == 0U);
    assert(bus->Init.GeneralCallMode == I2C_GENERALCALL_DISABLE);
    assert(bus->Init.NoStretchMode == I2C_NOSTRETCH_DISABLE);
    assert(clocks == (init_calls + 1U) * 4U && gpio_calls == (init_calls + 1U) * 2U);
    owned_bus = bus;
    ++init_calls;
    return fail_init ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *bus) {
    assert(bus == owned_bus && bus->Instance == I2C2);
    ++deinit_calls;
    return HAL_OK;
}
static size_t decode_address(I2C_HandleTypeDef *bus, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint16_t length, uint32_t timeout) {
    const unsigned base = EEPROM_AT24C08_A2 ? 0xA8U : 0xA0U;
    assert(bus == owned_bus && bus->Instance == I2C2);
    assert(address >= base && address <= base + 6U && address % 2U == 0U);
    assert(reg_size == I2C_MEMADD_SIZE_8BIT && reg < 256U && timeout == 20U);
    assert(length != 0U && (unsigned)reg + length <= 256U);
    last_address = address; last_register = reg; last_length = length;
    return ((address - base) >> 1U) * 256U + reg;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout) {
    const size_t offset = decode_address(bus, address, reg, reg_size, size, timeout);
    assert(size <= EEPROM_READ_CHUNK_BYTES);
    ++reads;
    if (reads == fail_read) return HAL_TIMEOUT;
    memcpy(data, memory + offset, size);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout) {
    const size_t offset = decode_address(bus, address, reg, reg_size, size, timeout);
    assert(size <= 16U && (reg % 16U) + size <= 16U);
    ++writes;
    if (writes == fail_write) return HAL_ERROR;
    memcpy(memory + offset, data, size);
    return HAL_OK;
}
BaseType_t xTaskGetSchedulerState(void) { return scheduler; }
TickType_t app_clock_period_ticks(uint32_t milliseconds) {
    return (milliseconds * configTICK_RATE_HZ + 999U) / 1000U;
}
void vTaskDelay(TickType_t ticks) {
    assert(scheduler == taskSCHEDULER_RUNNING);
    assert(ticks == app_clock_period_ticks(EEPROM_AT24C08_WRITE_CYCLE_MS) + 1U);
    ++delays;
}
void HAL_Delay(uint32_t milliseconds) {
    assert(scheduler == taskSCHEDULER_NOT_STARTED);
    assert(milliseconds == EEPROM_AT24C08_WRITE_CYCLE_MS);
    ++hal_delays;
}

int main(void) {
    uint8_t input[1024], output[1024];
    for (size_t i = 0U; i < sizeof(input); ++i) input[i] = (uint8_t)(i ^ (i >> 8U));
    memset(memory, 0xFF, sizeof(memory));
    assert(!eeprom_port_read(0U, output, 1U));
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_NOT_READY);
    fail_init = true;
    assert(!eeprom_port_init() && reads == 0U && writes == 0U);
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_INIT);
    fail_init = false;
    fail_read = 1U;
    assert(!eeprom_port_init() && writes == 0U);
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_READ && deinit_calls == 1U);
    assert(!eeprom_port_write(0U, input, 1U));
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_NOT_READY);
    fail_read = 0U;
    assert(eeprom_port_init() && writes == 0U);
    assert(last_address == (EEPROM_AT24C08_A2 ? 0xA8U : 0xA0U));
    assert(last_register == 0U && last_length == 1U);
    for (size_t i = 0U; i < sizeof(memory); ++i) assert(memory[i] == 0xFFU);

    /* Real upstream splitter must protect every 16-byte page and bank. */
    assert(eeprom_port_write(15U, input, 520U));
    assert(writes == 34U && delays == 34U);
    assert(memcmp(memory + 15U, input, 520U) == 0 && memory[14] == 0xFFU && memory[535] == 0xFFU);
    assert(eeprom_port_read(15U, output, 520U) && memcmp(input, output, 520U) == 0);
    assert(eeprom_port_write(0U, input, sizeof(input)));
    assert(eeprom_port_read(0U, output, sizeof(output)) && memcmp(input, output, sizeof(input)) == 0);
    assert(last_address == (EEPROM_AT24C08_A2 ? 0xAEU : 0xA6U));
    assert(last_register == 224U && last_length == 32U);

    const unsigned previous_reads = reads, previous_writes = writes;
    assert(!eeprom_port_write(1023U, input, 2U));
    assert(!eeprom_port_read(1024U, output, 1U));
    assert(!eeprom_port_read(65535U, output, SIZE_MAX));
    assert(!eeprom_port_read(0U, NULL, 1U));
    assert(!eeprom_port_write(0U, NULL, 1U));
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_ARGUMENT);
    assert(eeprom_port_write(1024U, NULL, 0U) && eeprom_port_read(1024U, NULL, 0U));
    assert(reads == previous_reads && writes == previous_writes);
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_NONE);

    memset(memory, 0xFF, sizeof(memory));
    const unsigned prior_delays = delays;
    fail_write = writes + 2U;
    assert(!eeprom_port_write(15U, input, 50U));
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_WRITE && writes == fail_write);
    assert(delays == prior_delays + 1U && memory[15] == input[0] && memory[16] == 0xFFU);
    assert(memory[32] == 0xFFU && memory[64] == 0xFFU);
    fail_write = 0U;
    fail_read = reads + 2U;
    assert(!eeprom_port_read(250U, output, 40U));
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_READ && reads == fail_read);
    fail_read = 0U;
    assert(eeprom_port_read(250U, output, 40U));
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_NONE);

    const unsigned previous_clocks = clocks;
    ipsr = 16U;
    assert(!eeprom_port_init() && !eeprom_port_write(0U, input, 1U));
    ipsr = 0U; primask = 1U;
    assert(!eeprom_port_read(0U, output, 1U));
    primask = 0U; scheduler = taskSCHEDULER_SUSPENDED;
    assert(!eeprom_port_write(0U, input, 1U));
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_CONTEXT && clocks == previous_clocks);
    scheduler = taskSCHEDULER_NOT_STARTED;
    assert(eeprom_port_write(0U, input, 1U) && hal_delays == 1U);
    scheduler = taskSCHEDULER_RUNNING;
    assert(eeprom_port_init() && eeprom_port_last_error() == EEPROM_PORT_ERROR_NONE);
    assert(memory[0] == input[0]);
    puts("PASS AT24C08 real LibDriver: I2C2 mapping, readonly init, 16-byte pages, banks, capacity, failure, context, RTOS delay");
    return 0;
}

#else

/* This target intentionally links without HAL, CMSIS, FreeRTOS or LibDriver:
 * disabled code cannot touch GPIO or clocks, even when APIs are invoked. */
int main(void) {
    uint8_t byte = 0U;
    assert(!eeprom_port_init());
    assert(!eeprom_port_read(0U, &byte, 1U));
    assert(!eeprom_port_write(0U, &byte, 1U));
    assert(eeprom_port_last_error() == EEPROM_PORT_ERROR_NOT_CONFIGURED);
    puts("PASS disabled EEPROM has no hardware dependency or side effects");
    return 0;
}

#endif
