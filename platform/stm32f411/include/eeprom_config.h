#ifndef EEPROM_CONFIG_H
#define EEPROM_CONFIG_H

/* Enable explicitly only after the optional EEPROM wiring is confirmed.
 * Dedicated I2C2: PB10 AF4 SCL, PB9 AF9 SDA. External pull-ups to 3.3 V.
 * MPU6050 stays on I2C1. AT24C08 uses A2; A1/A0 are not address selectors.
 * A2=0 occupies 7-bit 0x50..0x53; A2=1 occupies 0x54..0x57.
 * WP must be tied low to allow writes; never leave it floating. */
#ifndef EEPROM_ENABLED
#define EEPROM_ENABLED 0
#endif
#ifndef EEPROM_AT24C08_A2
#define EEPROM_AT24C08_A2 0
#endif
#ifndef EEPROM_AT24C08_WRITE_CYCLE_MS
#define EEPROM_AT24C08_WRITE_CYCLE_MS 12U
#endif
#define EEPROM_I2C_CLOCK_HZ 100000U
#define EEPROM_I2C_TIMEOUT_MS 20U
#define EEPROM_READ_CHUNK_BYTES 32U

#if EEPROM_ENABLED != 0 && EEPROM_ENABLED != 1
#error "EEPROM_ENABLED must be 0 or 1"
#endif
#if EEPROM_AT24C08_A2 != 0 && EEPROM_AT24C08_A2 != 1
#error "EEPROM_AT24C08_A2 must match the A2 hardware strap (0 or 1)"
#endif
#if EEPROM_AT24C08_WRITE_CYCLE_MS < 10U || EEPROM_AT24C08_WRITE_CYCLE_MS > 100U
#error "AT24C08 write-cycle wait must be between 10 and 100 ms"
#endif

#endif
