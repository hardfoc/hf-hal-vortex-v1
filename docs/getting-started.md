# Getting started

This page takes you from nothing to a Vortex V1 board printing its start-up log. Each step says how to tell that it
worked. Plan on half an hour, most of it the ESP-IDF download.

## What you need

| Item | Notes |
| --- | --- |
| A Vortex V1 board | The board starts and logs over USB alone |
| USB cable | Data-capable; the ESP32-C6 enumerates as a USB serial / JTAG device |
| A motor supply | Only to spin a motor. Without it the motor manager comes up degraded, which is expected |
| Linux, macOS or Windows (WSL2) | The build scripts are Bash |

## 1. Install ESP-IDF

The examples target `esp32c6` and build with ESP-IDF **release/v5.5** (both read from
[`examples/esp32/app_config.yml`](../examples/esp32/app_config.yml), which CI uses too).

```bash
mkdir -p ~/esp && cd ~/esp
git clone -b release/v5.5 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32c6
. ./export.sh
```

**It worked if** `idf.py --version` prints a v5.5 version. Run `. ~/esp/esp-idf/export.sh` in every new terminal.

## 2. Clone with submodules

The HAL pulls the shared core (`lib/core`: handlers, drivers, the pin map), the build scripts and CANopenNode as git
submodules.

```bash
git clone --recurse-submodules https://github.com/hardfoc/hf-hal-vortex-v1.git
cd hf-hal-vortex-v1
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

**It worked if** `ls lib/core/handlers` lists handler folders and `ls examples/esp32/scripts` shows `build_app.sh`.

!!! note "One submodule is private"
    `docs/hf-development-handbook` is a private repository and is marked `update = none`, so the recursive update
    skips it. You do not need it to build.

## 3. Build an example

Every example is an "app" selected by name. Start with `vortex_api_test`, the default app, which initialises the
whole HAL and exercises each manager.

```bash
cd examples/esp32
./scripts/build_app.sh list                        # every app name
./scripts/build_app.sh vortex_api_test Release     # build one
```

**It worked if** the script ends with a successful link and a build folder named after the app, the build type and
the target appears under `examples/esp32/`.

## 4. Flash and monitor

Connect the board over USB, then:

```bash
./scripts/flash_app.sh flash_monitor vortex_api_test Release
```

Set `ESPPORT` first if more than one serial port is present (for example `export ESPPORT=/dev/ttyACM0`). Leave the
monitor with ++ctrl+bracket-right++.

**It worked if** the log shows `Vortex API initialization completed successfully in ... ms` followed by a
`Component Status` line with `OK` or `FAIL` for each of NVS, Comms, GPIO, Motors, ADC, IMU, Encoders, LEDs and
Temp. `FAIL` on Motors, ADC or Temp without a motor supply is normal: those steps are best effort. See
[Initialisation](architecture/initialization.md).

## 5. Use the HAL in your own code

```cpp
#include "api/Vortex.h"
#include "managers/LedManager.h"
#include "managers/TemperatureManager.h"

extern "C" void app_main() {
    auto& vortex = Vortex::GetInstance();
    if (!vortex.EnsureInitialized()) {
        return;  // NVS, comms, GPIO or the LED failed: nothing below can work
    }
    (void)vortex.leds.IndicateReady();

    float chip_c = 0.0f;
    if (vortex.temp.ReadInternalTemperature(chip_c) == hf_temp_err_t::TEMP_SUCCESS) {
        // chip_c is the ESP32-C6 die temperature
    }
}
```

## Next steps

- [Learn](learn/index.md) walks through the rest of the site in order.
- [Examples](examples/index.md) lists every app and what it shows.
- [Troubleshooting](guides/troubleshooting.md) covers the common first-day problems.
