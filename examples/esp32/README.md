# HardFOC Vortex HAL — ESP32 firmware apps

## Overview

This directory contains buildable ESP-IDF **apps** under `main/`, each selected by
`APP_TYPE` via `scripts/build_app.sh`. They are meant for **hardware integration**:
build, flash, and read the serial log to confirm behaviour.

**Primary reference app:** `vortex_api_test` — exercises the full `Vortex` singleton
(init, diagnostics, comms, GPIO, motors, ADC, IMU, encoders, LEDs, temperature)
with patterns aligned to the Vortex V1 pin map (`main/common/vortex_board_pins.hpp`).

On a **complete bench** (nominal DC bus, sensors populated), turn on **strict hardware
checks** via `idf.py menuconfig` → *Component config* → *Main* → *Vortex API test* (see
[`docs/BENCH_MATRIX.md`](docs/BENCH_MATRIX.md)) so wiring or supply problems fail tests
instead of logging only.

**Board bring-up:** [`docs/BENCH_MATRIX.md`](docs/BENCH_MATRIX.md) (apps ↔ benches,
supply limits, timeouts). **Full app index (every `APP_TYPE` + source file):**
[`docs/README.md`](docs/README.md). Do not copy TMC9660 EVKIT GPIO numbers; use the Vortex
functional pin config and `vortex_board_pins.hpp`.

## Quick start

```bash
# Build the default app (full HAL integration test)
./scripts/build_app.sh vortex_api_test Release

# Flash and monitor (set ESPPORT if needed)
./scripts/flash_app.sh flash_monitor vortex_api_test Release

# List all apps
./scripts/build_app.sh list
```

### Native `idf.py` (flash + monitor)

Each app uses its own **`-B` build directory** under `builds/` (created by `build_app.sh`).
Use ESP-IDF’s combined target after exporting IDF:

```bash
source ~/esp/esp-idf/export.sh
export ESPPORT=/dev/ttyACM0
cd examples/esp32
./scripts/build_app.sh vortex_led_ws2812_smoke Release   # once
./scripts/idf_flash_monitor.sh vortex_led_ws2812_smoke Release
```

That runs: `idf.py -C <this dir> -B <per-app build> -p $ESPPORT flash monitor`.

If the serial console only shows `waiting for download` when automation reopens the port,
pass through monitor options (e.g. skip the extra reset):

```bash
./scripts/idf_flash_monitor.sh vortex_led_ws2812_smoke Release --no-reset
```

Non-interactive hosts (no TTY) need a pseudo-tty wrapper, e.g. `script -qec '…' /dev/null`.

## App families

| Family | Examples | Notes |
|--------|----------|--------|
| Full HAL | `vortex_api_test` | Default; broad API + board-oriented checks |
| Focused | `led_temp_test`, `encoder_imu_test`, `motor_controller_test`, `motor_controller_uart_test`, … | Deeper single-area coverage |
| Smoke / bench | `vortex_led_ws2812_smoke`, `vortex_motor_comms_smoke`, `vortex_bldc_*`, … | Hardware smoke or motion benches (see bench matrix) |

## Project layout

```
examples/esp32/
├── CMakeLists.txt
├── app_config.yml          # app type → source file, CI flags
├── sdkconfig / sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── vortex_api_test.cpp
│   ├── led_temp_test.cpp
│   └── …                    # other app entry .cpp files
├── docs/
└── scripts/                 # build / flash tooling (submodule)
```

## Build system

`build_app.sh` passes `APP_TYPE` and `BUILD_TYPE` into CMake so each app links a
single `main/` translation unit. See `app_config.yml` for the authoritative list.
