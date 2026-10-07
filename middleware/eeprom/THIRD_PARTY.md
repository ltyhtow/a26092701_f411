# AT24Cxx dependency notice

This module vendors **LibDriver AT24CXX**, not a newly implemented EEPROM
transaction driver. The project-level license has not been selected or changed.

- Repository: https://github.com/libdriver/at24cxx
- Pinned commit: `aa2a2a39375b7e936bc7081526df00bbd66da28e`
- Commit date reported by GitHub: `2026-09-06T09:41:16Z`
- Commit: https://github.com/libdriver/at24cxx/commit/aa2a2a39375b7e936bc7081526df00bbd66da28e
- License: MIT, Copyright (c) 2015 - present LibDriver.
- License text: [third_party/libdriver/LICENSE](third_party/libdriver/LICENSE).
- Upstream README: [third_party/libdriver/README.md](third_party/libdriver/README.md).

The two upstream `src/` files are stored directly in `third_party/libdriver/`;
their contents and the upstream LICENSE/README are unmodified. All downloads
use the immutable commit above. SHA-256 checksums:

| Local file | SHA-256 |
|---|---|
| `driver_at24cxx.c` | `0F1B97508C2C23A1E3255A8537D69BE92D4EDE90D405CEF2009580D628E8CA75` |
| `driver_at24cxx.h` | `9F039D6E82768713B9AD0E1A1FC3B0BFCC3FB19FABF13523220431955B05E041` |
| `LICENSE` | `81753BF783B8E971FCBCAC6E4008D58F32C44818F3678DF9774414565CB7C465` |
| `README.md` | `6017DF77D205F8044C74B2764311652BFBD21C3F3E3AF455F0BDE3F8AC233015` |

Keep the MIT copyright and permission notice with copies of this dependency,
including when distributing the enclosing project under a chosen copyleft
license. This is a dependency inventory, not a whole-project license audit.

The STM32 adapter is project code outside the vendored directory. It configures
I2C2, validates arguments/context, bounds reads, and converts the driver's delay
callback to a yielding FreeRTOS wait. The real upstream implementation retains
responsibility for AT24C08 addressing and 16-byte write-page splitting.
