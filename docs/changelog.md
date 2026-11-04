# Changelog

What changed in the Vortex V1 HAL, newest first. Releases are tagged `vX.Y.Z` and listed on
[GitHub releases](https://github.com/hardfoc/hf-hal-vortex-v1/releases); no release has been tagged yet.

## Unreleased

### Breaking

- hf-core and hf-as5047u-driver are bumped to the AS5047U getters that return `as5047u::Result<T>`. Code that reads
  the sensor through `EncoderManager::VisitDriver()` or the handler reads `.value` after checking `.ok()`.

### Fixed

- `EncoderManager::ReadDiagnostics()` filled `As5047uDiagnostics` with `communication_ok = true` when its ERRFL read
  failed (the failed read came back as 0, and the read's transport flags were discarded). It now returns the bus
  error and leaves the struct alone. `ReadAngle()`, `ReadAngleDegrees()`, `ReadVelocityRPM()` and
  `ReadFieldStrength()` no longer overwrite the caller's value with the 0 of a failed read; their error now also
  comes from the read's own `Result`, not only from the sticky flags.

### Changed

- The documentation builds with the shared HardFOC theme (`hardfoc-mkdocs-theme`); the copied template, hooks and
  figure toolkit are gone. The Doxyfile moved to the repository root.
- The documentation follows the HAL page set: board card, quickstart, bring-up and troubleshooting, changelog; the
  roadmap links the CANopen node and drive observability designs.
- hf-core is bumped to the null-safe TMC9660 accessors and the NTC init fix. The board NTC config enables threshold
  monitoring with the 0–85 °C window again, so the handler applies it during init, and `ApplyBoardNtcThresholds()`
  is gone. `GpioManager` leaves a TMC9660 bridge pin unregistered when `Tmc9660Handler::gpio()` returns nullptr.

### Added

- `Vortex` facade with nine managers: communication channels, NVS, GPIO, motor controller (TMC9660 over UART or
  SPI), ADC, IMU (BNO08x), encoders (AS5047U), LED and temperature.
- Pin map, examples catalogue and figures generated from the code and checked in CI.
