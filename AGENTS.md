# Project workflow

- Work in native Windows with PowerShell 7. Use `rg` to search and `apply_patch`
  for source/configuration edits. Preserve unrelated working-tree changes.
- The user authorized automatic **local Git commits** after each completed task.
  Run the checks appropriate to the change, review the diff, and commit the
  task's source, tests and documentation without asking again. Never create a
  commit that claims unperformed hardware validation. Remote pushes require a
  user request; they are not part of this local-commit convention.
- Do not commit credentials, Python caches, build artifacts or raw serial
  captures. Preserve third-party source licenses and provenance.
- If work is delegated, use at most four concurrent agents including the main
  agent, with separate file ownership.
- The firmware executable/CLion run target is always `a26092701_f411`.
  `UsbWheelTest` is its CMake preset, not another executable. Keep library and
  test build targets; they are dependencies, not separate firmware to flash.
- `UsbWheelTest` uses USB CDC and requires the motor input formerly on PA11
  to be wired to PB5. See `USB_CDC_GUIDE.md` for current setup and validation.
- Preserve binary framing: typed payload -> LwPKT -> LwRB -> UART DMA or USB CDC.
  Never put raw UTF-8 diagnostics into the transport buffers.
- Standard verification: `pwsh -NoProfile -File tools\run_static_checks.ps1 -IncludeDiagnostics`.
  This builds/tests without flashing or opening hardware.
