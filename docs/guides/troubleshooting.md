# Troubleshooting

The problems people hit on the first day, and what to do about each. If yours is not here, the start-up log usually
names the manager that failed; start from that manager's page.

## Build

| Symptom | Cause | Fix |
| --- | --- | --- |
| `fatal: not a git repository` or empty `lib/core` | Submodules not initialised | `git submodule update --init --recursive` |
| `APP_TYPE not defined` | `idf.py build` run directly | Build with `./scripts/build_app.sh <app> <Debug\|Release>` |
| `No supported board variant defined!` | The pin-map selector got no `HARDFOC_BOARD_*` define | Build through this repository's CMake; it defines `HARDFOC_BOARD_VORTEX_V1` |
| Errors in ESP-IDF headers | Wrong ESP-IDF version or target | Use release/v5.5 and `esp32c6` ([Get started](../getting-started.md)) |

## Start-up

| Symptom | Cause | Fix |
| --- | --- | --- |
| `Vortex API initialization failed` | A required step failed: NVS, comms, GPIO or the LED | Read the error just above it. NVS already erases and retries a full or outdated partition; if it still fails, check that the partition table has an `nvs` partition |
| `Initialized in degraded mode — Motors:OFF ...` | A best-effort manager did not start | Expected without a motor supply or with parts not fitted; otherwise check that manager's wiring |
| Motors, ADC and Temp all `OFF` together | The onboard TMC9660 did not come up; ADC and two temperature sensors depend on it | Check the motor supply and the TMC9660 control lines on the PCAL95555 |
| IMU `OFF` | No answer at 0x4A, or reset / boot lines not driven | Run `vortex_i2c_scan_pcal_bno_pca`; check `PCAL_IMU_RST` and `PCAL_IMU_BOOT` |
| Stack overflow in `main` during start-up | Main task stack too small | Keep `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` or more |

## Running

| Symptom | Cause | Fix |
| --- | --- | --- |
| Encoder reads return `NOT_INITIALIZED` | No device created | `vortex.encoders.CreateAs5047uDevice(0, SpiDeviceId::AS5047U_POSITION_ENCODER)` |
| LED blink or breath does not move | Nobody calls `UpdateAnimation()` | Call it every 50 ms from a task |
| `SetOnboardTmc9660Transport ignored` in the log | Called after `EnsureInitialized()` | Call it before |
| GPIO call returns `GPIO_ERR_PIN_NOT_FOUND` | Name typo, or a core / comm pin | Copy the name from the [pin map](../hardware/pin-map.md); only GPIO-category pins are registered |
| The monitor shows only `waiting for download` | The port was reopened in download mode | Pass `--no-reset` to `./scripts/idf_flash_monitor.sh`, or press reset |

## Documentation

| Symptom | Fix |
| --- | --- |
| CI says a generated region or figure is stale | `python3 tools/docs/gen.py && python3 tools/figkit/build.py`, then commit |
| `mkdocs build --strict` reports a link | Fix the link; links to code outside `docs/` work as relative paths |
