# Changelog

What changed in the Vortex V1 HAL, newest first. Releases are tagged `vX.Y.Z` and listed on
[GitHub releases](https://github.com/hardfoc/hf-hal-vortex-v1/releases); no release has been tagged yet.

## Unreleased

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
