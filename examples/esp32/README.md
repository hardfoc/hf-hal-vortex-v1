# HardFOC Vortex HAL — ESP32 Examples

## Overview

This directory contains buildable ESP-IDF examples that exercise the **Vortex HAL API**
from `lib/api/Vortex.h`. Each `.cpp` file in `main/` is a standalone test/example
that can be built and flashed independently using the `build_app.sh` script.

## Available Examples

| App Type | Source File | Description |
|---|---|---|
| `vortex_api_test` | `vortex_api_test.cpp` | Full Vortex API initialization and diagnostics |
| `motor_test` | `motor_test.cpp` | Motor controller via `vortex.motors` |
| `encoder_test` | `encoder_test.cpp` | Encoder reading via `vortex.encoders` |
| `imu_test` | `imu_test.cpp` | IMU sensor data via `vortex.imu` |
| `led_test` | `led_test.cpp` | LED animations via `vortex.leds` |
| `temperature_test` | `temperature_test.cpp` | Temperature readings via `vortex.temp` |

## Quick Start

```bash
# Build the default example
./scripts/build_app.sh vortex_api_test Release

# Flash to device
./scripts/flash_app.sh

# List all available examples
./scripts/build_app.sh list
```

## Project Structure

```
examples/esp32/
├── CMakeLists.txt          # Project root (requires APP_TYPE from build_app.sh)
├── README.md               # This file
├── sdkconfig               # ESP-IDF SDK configuration
├── main/
│   ├── CMakeLists.txt      # Component registration
│   ├── vortex_api_test.cpp # Full API test
│   ├── motor_test.cpp      # Motor controller test
│   ├── encoder_test.cpp    # Encoder test
│   ├── imu_test.cpp        # IMU test
│   ├── led_test.cpp        # LED test
│   └── temperature_test.cpp# Temperature test
└── scripts/                # Build/flash tooling (submodule)
```

## Build System

This project uses the same build infrastructure as the HardFOC external driver
examples. The `scripts/` directory is a Git submodule pointing to the shared
build tooling repository.

Each example is built by passing `APP_TYPE` and `BUILD_TYPE` to CMake via
`build_app.sh`, which maps the app type to the correct source file.
