#include "eeprom_port.h"
#include "eeprom_config.h"

#if EEPROM_ENABLED
#include "driver_at24cxx.h"
#include "stm32f4xx_hal.h"
#include "app_clock.h"
#include "task.h"

static I2C_HandleTypeDef eeprom_i2c;
static at24cxx_handle_t eeprom_driver;
static bool ready;
static uint32_t last_error = EEPROM_PORT_ERROR_NOT_READY;

static bool context_allowed(void) {
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U ||
        xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) {
        last_error = EEPROM_PORT_ERROR_CONTEXT;
        return false;
    }
    return true;
}

static uint8_t eeprom_iic_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_9;
    gpio.Alternate = GPIO_AF9_I2C2;
    HAL_GPIO_Init(GPIOB, &gpio);
    eeprom_i2c.Instance = I2C2;
    eeprom_i2c.Init.ClockSpeed = EEPROM_I2C_CLOCK_HZ;
    eeprom_i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    eeprom_i2c.Init.OwnAddress1 = 0U;
    eeprom_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    eeprom_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    eeprom_i2c.Init.OwnAddress2 = 0U;
    eeprom_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    eeprom_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    return HAL_I2C_Init(&eeprom_i2c) == HAL_OK ? 0U : 1U;
}

static uint8_t eeprom_iic_deinit(void) {
    return HAL_I2C_DeInit(&eeprom_i2c) == HAL_OK ? 0U : 1U;
}

static uint8_t eeprom_iic_read(uint8_t address, uint8_t reg,
                             uint8_t *data, uint16_t length) {
    return HAL_I2C_Mem_Read(&eeprom_i2c, address, reg, I2C_MEMADD_SIZE_8BIT,
                           data, length, EEPROM_I2C_TIMEOUT_MS) == HAL_OK ? 0U : 1U;
}

static uint8_t eeprom_iic_write(uint8_t address, uint8_t reg,
                              uint8_t *data, uint16_t length) {
    return HAL_I2C_Mem_Write(&eeprom_i2c, address, reg, I2C_MEMADD_SIZE_8BIT,
                            data, length, EEPROM_I2C_TIMEOUT_MS) == HAL_OK ? 0U : 1U;
}

/* LibDriver requires both address-width callbacks even for the 8-bit device.
 * Reject a wrong device type instead of emitting an incompatible bus cycle. */
static uint8_t unsupported_address16(uint8_t address, uint16_t reg,
                                    uint8_t *data, uint16_t length) {
    (void)address; (void)reg; (void)data; (void)length;
    return 1U;
}

static void eeprom_delay_ms(uint32_t milliseconds) {
    /* Upstream uses 6 ms. Leave its source intact while accommodating legacy
     * AT24C08 variants with a longer write cycle. Add one tick to account for
     * entry immediately before the next scheduler tick. */
    if (milliseconds < EEPROM_AT24C08_WRITE_CYCLE_MS) {
        milliseconds = EEPROM_AT24C08_WRITE_CYCLE_MS;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(app_clock_period_ticks(milliseconds) + 1U);
    } else {
        HAL_Delay(milliseconds);
    }
}

static void eeprom_debug_print(const char *const format, ...) {
    /* No unframed logging on the shared telemetry UART. */
    (void)format;
}

bool eeprom_port_init(void) {
    if (!context_allowed()) return false;
    ready = false;
    if (eeprom_driver.inited != 0U) (void)at24cxx_deinit(&eeprom_driver);
    eeprom_i2c = (I2C_HandleTypeDef){0};
    DRIVER_AT24CXX_LINK_INIT(&eeprom_driver, at24cxx_handle_t);
    DRIVER_AT24CXX_LINK_IIC_INIT(&eeprom_driver, eeprom_iic_init);
    DRIVER_AT24CXX_LINK_IIC_DEINIT(&eeprom_driver, eeprom_iic_deinit);
    DRIVER_AT24CXX_LINK_IIC_READ(&eeprom_driver, eeprom_iic_read);
    DRIVER_AT24CXX_LINK_IIC_WRITE(&eeprom_driver, eeprom_iic_write);
    DRIVER_AT24CXX_LINK_IIC_READ_ADDRESS16(&eeprom_driver, unsupported_address16);
    DRIVER_AT24CXX_LINK_IIC_WRITE_ADDRESS16(&eeprom_driver, unsupported_address16);
    DRIVER_AT24CXX_LINK_DELAY_MS(&eeprom_driver, eeprom_delay_ms);
    DRIVER_AT24CXX_LINK_DEBUG_PRINT(&eeprom_driver, eeprom_debug_print);
    if (at24cxx_set_type(&eeprom_driver, AT24C08) != 0U ||
        at24cxx_set_addr_pin(&eeprom_driver, EEPROM_AT24C08_A2 ?
                            AT24CXX_ADDRESS_A100 : AT24CXX_ADDRESS_A000) != 0U ||
        at24cxx_init(&eeprom_driver) != 0U) {
        last_error = EEPROM_PORT_ERROR_INIT;
        return false;
    }
    uint8_t probe;
    if (at24cxx_read(&eeprom_driver, 0U, &probe, 1U) != 0U) {
        (void)at24cxx_deinit(&eeprom_driver);
        last_error = EEPROM_PORT_ERROR_READ;
        return false;
    }
    ready = true;
    last_error = EEPROM_PORT_ERROR_NONE;
    return true;
}

static bool request_valid(uint16_t address, const void *data, size_t length) {
    if (!context_allowed()) return false;
    if (!ready) {
        last_error = EEPROM_PORT_ERROR_NOT_READY;
        return false;
    }
    if (address > EEPROM_PORT_CAPACITY_BYTES ||
        length > EEPROM_PORT_CAPACITY_BYTES - address ||
        (length != 0U && data == NULL)) {
        last_error = EEPROM_PORT_ERROR_ARGUMENT;
        return false;
    }
    last_error = EEPROM_PORT_ERROR_NONE;
    return true;
}

bool eeprom_port_read(uint16_t address, uint8_t *data, size_t length) {
    if (!request_valid(address, data, length)) return false;
    while (length != 0U) {
        size_t chunk = 256U - (address % 256U);
        if (chunk > EEPROM_READ_CHUNK_BYTES) chunk = EEPROM_READ_CHUNK_BYTES;
        if (chunk > length) chunk = length;
        if (at24cxx_read(&eeprom_driver, address, data, (uint16_t)chunk) != 0U) {
            last_error = EEPROM_PORT_ERROR_READ;
            return false;
        }
        address = (uint16_t)(address + chunk);
        data += chunk;
        length -= chunk;
    }
    return true;
}

bool eeprom_port_write(uint16_t address, const uint8_t *data, size_t length) {
    if (!request_valid(address, data, length)) return false;
    /* Upstream accepts a mutable pointer but only reads the payload. It owns
     * all page splitting and bank-address selection. Zero length is handled
     * here to avoid upstream issuing a zero-byte bus transaction. */
    if (length != 0U && at24cxx_write(&eeprom_driver, address,
                                    (uint8_t *)data, (uint16_t)length) != 0U) {
        last_error = EEPROM_PORT_ERROR_WRITE;
        return false;
    }
    return true;
}

#else

static uint32_t last_error = EEPROM_PORT_ERROR_NOT_CONFIGURED;
bool eeprom_port_init(void) { return false; }
bool eeprom_port_read(uint16_t address, uint8_t *data, size_t length) {
    (void)address; (void)data; (void)length;
    return false;
}
bool eeprom_port_write(uint16_t address, const uint8_t *data, size_t length) {
    (void)address; (void)data; (void)length;
    return false;
}

#endif

uint32_t eeprom_port_last_error(void) { return last_error; }
