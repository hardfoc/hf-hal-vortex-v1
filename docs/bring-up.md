# Bring-up and troubleshooting

How to bring a new Vortex V1 board up one chip at a time, and what to do when a step fails. If your problem is not
here, the start-up log usually names the manager that failed; start from that manager's page.

## Bring up a new board

Work from the cheapest check to the most expensive, and do not connect a motor until the steps before pass. Each
app is built and flashed as in the [quickstart](quickstart.md)
(`./scripts/flash_app.sh flash_monitor <app> Release` from `examples/esp32`).

<div class="hf-steps" markdown>

1. **USB only, no motor supply.** Flash `vortex_api_test`. NVS, Comms, GPIO and LEDs must be `OK`: they are
   required, and `EnsureInitialized()` returns false without them. `FAIL` on Motors, ADC or Temp is expected here.
2. **I2C devices.** Flash `vortex_i2c_scan_pcal_bno_pca`: the PCAL95555 and the BNO08x must answer on the shared
   I2C bus. `vortex_pcal_gpio_imu_bringup` adds full I2C diagnostics when one does not.
3. **LED.** Flash `vortex_led_ws2812_smoke`: the WS2812 runs its colour demo.
4. **Motor supply, no motion.** Connect the motor supply and flash `vortex_motor_comms_smoke`: it reads back the
   TMC9660 control lines on the PCAL95555, brings the TMC9660 out of its bootloader and reads telemetry without
   moving anything. Then `vortex_bldc_telemetry_sweep` checks the power-stage configuration.
5. **First spin.** Follow the bench guides before you run any `vortex_bldc_*` motion app: the
   [BLDC bring-up guide](../examples/esp32/docs/BLDC_BRINGUP.md) (bootloader and parameter-mode order) and the
   [bench matrix](../examples/esp32/docs/BENCH_MATRIX.md) (per-app goals, supply and safety notes).
   `vortex_bldc_open_loop_uart` is the confirmed-working open-loop spin.

</div>

!!! warning "Bench limits live in one file"
    Supply, shunt, current caps, I²t and thermal limits for every bench app are in
    [`vortex_bench_safety.hpp`](../examples/esp32/main/common/vortex_bench_safety.hpp). Check them against your
    motor and supply before step 5.

## Build

| Symptom | Cause | Fix |
| --- | --- | --- |
| `fatal: not a git repository` or empty `lib/core` | Submodules not initialised | `git submodule update --init --recursive` |
| `APP_TYPE not defined` | `idf.py build` run directly | Build with `./scripts/build_app.sh <app> <Debug\|Release>` |
| `No supported board variant defined!` | The pin-map selector got no `HARDFOC_BOARD_*` define | Build through this repository's CMake; it defines `HARDFOC_BOARD_VORTEX_V1` |
| Errors in ESP-IDF headers | Wrong ESP-IDF version or target | Use release/v5.5 and `esp32c6` ([Get started](quickstart.md)) |

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
| GPIO call returns `GPIO_ERR_PIN_NOT_FOUND` | Name typo, or a core / comm pin | Copy the name from the [pin map](hardware/pin-map.md); only GPIO-category pins are registered |
| The monitor shows only `waiting for download` | The port was reopened in download mode | Pass `--no-reset` to `./scripts/idf_flash_monitor.sh`, or press reset |

## Documentation

| Symptom | Fix |
| --- | --- |
| CI says a generated region or figure is stale | `hardfoc-docs gen && hardfoc-docs figures`, then commit |
| `mkdocs build --strict` reports a link | Fix the link; links to code outside `docs/` work as relative paths |
