# TinyUSB source provenance

- Upstream: https://github.com/hathach/tinyusb
- Release: `0.21.0` (2026-06-30)
- Commit: `dae3f9a366bfcddbf9dcf1b48d7500286a849539`
- License: MIT; the original `LICENSE` and file copyright notices are retained.
- Imported subset: the device core, CDC device class, OS_NONE adapter, common utilities,
  and Synopsys DWC2 device backend for STM32. No host classes or other MCU backends.
- Upstream files are unmodified. Local configuration, descriptors and transport glue
  are in `platform/stm32f411/usb`.

The ST standalone USB Device middleware was evaluated first but not imported: its
current SLA0044 license contains restrictions incompatible with this project's
planned copyleft distribution. TinyUSB provides the USB stack and DWC2 driver;
the board continues to use its existing STM32 CMSIS/HAL for pins, clocks and time.

The native USB CDC path is binary-only: existing LwPKT/LwRB framing is unchanged.
OS_NONE class processing is called exclusively by the protocol task. USB ISR
callbacks enqueue device events and invalidate the transport epoch on link loss.
The stack and CDC class use static storage; no malloc-backed USB class allocation.
