# Optional AT24C08 EEPROM port

The user identified the fitted chip as **AT24C08N**. No read-only device ID exists
in this interface, so a successful probe does not prove its maker, capacity, or
condition. A module-level read/write/restore test is still required on hardware.

This implementation selects LibDriver's `AT24C08`: 1024 bytes, 16-byte write
pages, an 8-bit word address and two bank bits in the device address. The
[Microchip/Atmel family reference](https://ww1.microchip.com/downloads/en/DeviceDoc/doc0180.pdf)
describes AT24C08A, including a 5 ms maximum write cycle; it is **not** proof that
the user's complete legacy marking is AT24C08A or that an unbranded module is
authentic. The adapter conservatively waits at least 12 ms after each page write,
instead of upstream's 6 ms, with one extra RTOS tick for timing phase margin.
This allowance does not replace identifying and testing the fitted part.

## Board connection

Use a dedicated **I2C2** bus. The existing MPU6050 I2C1 bus is untouched.

| Module signal | STM32F411 connection |
|---|---|
| VCC | 3.3 V |
| GND | Common ground |
| SCL | PB10, AF4 I2C2 |
| SDA | PB9, AF9 I2C2 |
| A2 | GND by default; `EEPROM_AT24C08_A2=1` for a high strap |
| WP | GND to permit saving; high protects against writes |

Use pull-ups to **3.3 V**, not 5 V. The backend runs at 100 kHz and uses no MCU
internal pull-up. Check existing module pull-ups rather than automatically
adding a second pair. The AT24C08 uses A2 to select the chip; A1/A0 are not
independent chip address selectors for this density. With A2 low it responds
at 7-bit addresses 0x50 through 0x53; A2 high selects 0x54 through 0x57. The
LibDriver and STM32 HAL callback pass those addresses left-shifted by one.

`EEPROM_ENABLED` defaults to 0. In the disabled build all port calls return
false/NOT_CONFIGURED and compile without any HAL, GPIO or RTOS dependency.
Enabling is an explicit firmware configuration choice after wiring is checked.

## Ownership and failure behavior

- Only the storage worker owns this port. It is not reentrant and must not be
  called from the balance task. ISR, interrupt-masked and scheduler-suspended
  calls are rejected before a bus transaction.
- Init configures only I2C2 and probes with a one-byte read. It never formats,
  erases or writes the chip and never calls `Error_Handler` on an absent device.
- Reads are bounded to at most 32 bytes and split at 256-byte bank boundaries.
  Each HAL transaction has a 20 ms timeout. The upstream driver splits writes
  into 16-byte pages and the adapter yields between pages.
- No I2C1/IMU mutex is held. EEPROM absence or a NACK must be reported to the
  caller; it must not prevent the vehicle from using validated RAM defaults.
- A false write result may follow a partially written prefix. Transactional
  record selection and CRC, plus read-back verification, belong to the storage
  layer. In particular, some chips ACK writes while WP prevents changes.
- A successful transport result is not an acknowledgement that a parameter
  record is durable. Only the storage layer may make that claim after verifying
  its complete record. No automatic retry or periodic rewrite consumes endurance.

## Regressions

`tools/eeprom_port_tests.cmake` defines three host targets: disabled, A2 low,
and A2 high. The enabled targets compile the actual STM32 adapter and the
unmodified vendor source, mocking only HAL/RTOS. They validate pins and bus
settings, read-only initialization, page/bank boundaries, full 1024-byte
transfers, overflow/null/zero-length arguments, partial write failures, read
failure, no-hardware disabled behavior, context rejection, and a non-1000 Hz
RTOS delay. Firmware compilation and on-board validation remain separate checks.

See [THIRD_PARTY.md](THIRD_PARTY.md) for immutable source provenance and notices.
