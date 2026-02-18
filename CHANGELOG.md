# Changelog — HardFOC Vortex V1 HAL

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Added

- **On-target test suite**: Five new test files under `examples/esp32/main/`:
  `gpio_stress_test.cpp`, `adc_calibration_test.cpp`,
  `motor_controller_test.cpp`, `encoder_imu_test.cpp`, `led_temp_test.cpp`
  using the shared `TestFramework.h` harness.
- **CI app entries**: `app_config.yml` updated with test app registrations.
- **CI workflows**: Seven GitHub Actions workflows for build, lint, docs, and
  release automation (`.github/workflows/`).
- **Lint configs**: `.clang-tidy`, `.clang-format`, `cppcheck.cfg`,
  `.markdownlint.json`, `.editorconfig`.
- **Doxygen / Jekyll doc pipeline**: `Doxyfile`, Jekyll site skeleton
  (`docs/_config.yml`, layouts, includes), and `ci-docs-publish.yml` workflow.
- **Documentation index**: Expanded `DOCUMENTATION_INDEX.md` with getting-started
  guide, cross-references, and quickstart code.

### Changed

- **`CommChannelsManager`**: `CommError` enum replaces raw `bool` for all bus
  accessor error reporting.
- **`EnsureInitialized()`**: Now returns `bool` across all managers (previously
  void on some).
- **`VortexApiExample.cpp`**: Updated to use the new `CommError`-based API.
- **`Logger`**: Consistent `GetInstance()` singleton usage across all managers.

### Removed

- `tests/` directory: Replaced by structured on-target test harness under
  `examples/esp32/main/`.

---

## [1.0.0] — 2025-06-01

Initial release of the HardFOC Vortex V1 HAL for ESP32-C6.

### Included

- Eight managers: ADC, CommChannels, Encoder, GPIO, IMU, LED, MotorController,
  Temperature.
- Vortex API façade (`Vortex.h` / `Vortex.cpp`).
- Board pin configuration (`pincfg`).
- ESP-IDF 5.4+ build integration.
