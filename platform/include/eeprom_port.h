#ifndef EEPROM_PORT_H
#define EEPROM_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EEPROM_PORT_CAPACITY_BYTES 1024U

typedef enum {
    EEPROM_PORT_ERROR_NONE = 0,
    EEPROM_PORT_ERROR_NOT_CONFIGURED = 1,
    EEPROM_PORT_ERROR_NOT_READY = 2,
    EEPROM_PORT_ERROR_ARGUMENT = 3,
    EEPROM_PORT_ERROR_INIT = 4,
    EEPROM_PORT_ERROR_READ = 5,
    EEPROM_PORT_ERROR_WRITE = 6,
    EEPROM_PORT_ERROR_CONTEXT = 7
} eeprom_port_error_t;

/* Single storage-worker ownership: not thread safe. Never call from control
 * tasks, ISR, scheduler-suspended or interrupt-masked code. Init probes with
 * a read only; it never erases or formats EEPROM. Calls are bounded per I2C
 * transaction, but a whole-chip write can take approximately one second.
 * Zero-length requests are no-ops; address==capacity is valid only for these.
 * Write success reports transport completion, not persistence verification.
 * The storage layer must read back data before acknowledging a saved record. */
bool eeprom_port_init(void);
bool eeprom_port_read(uint16_t address, uint8_t *data, size_t length);
bool eeprom_port_write(uint16_t address, const uint8_t *data, size_t length);
uint32_t eeprom_port_last_error(void);

#endif
