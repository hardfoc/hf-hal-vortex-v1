# Vortex V1 — bench matrix (ESP32-C6 HAL examples)

Nominal bench supply: **24 V DC**. Phase / torque current ceiling for bring-up examples: **1 A** (`vortex_bench_safety.hpp`). Motion profiles are **time-bounded** in firmware; use an e-stop and a secured shaft.

### `vortex_api_test` — strict hardware mode

For a **fully powered and wired** Vortex bench, enable Kconfig options (then rebuild):

`idf.py menuconfig` → **Component config** → **Main** → **Vortex API test (vortex_api_test)**

- **`CONFIG_VORTEX_API_TEST_STRICT_HW`**: require VM rail in 12–32 V, TMC `FAULT_n` high (not faulted), expected GPIO names, comms initialized, WS2812 diagnostics healthy, ESP32 die temp read OK.
- **`CONFIG_VORTEX_API_TEST_STRICT_SENSORS`** (depends on strict HW): require IMU + encoder enumeration, successful encoder angle read, and ≥1 motor device.

Defaults are **off** so CI and USB-debug builds stay permissive. Alternatively set the same symbols in `sdkconfig.defaults` for a team bench profile.

| App (`app_config.yml`) | Goal | CI | Notes |
| ---------------------- | ---- | -- | ----- |
| `vortex_led_ws2812_smoke` | WS2812 / `LedManager` | yes | GPIO3 data; demo runs boot pulse, RGB sweep, rainbow, breath, blink, status colours, ends dim green (~20 s). |
| `vortex_i2c_scan_pcal_bno_pca` | Shared I2C + PCAL + BNO + PCA9685 | yes | SDA 21 / SCL 22; runtime slot @ 0x40 for `Pca9685Handler`. |
| `vortex_motor_comms_smoke` | TMC9660 telemetry, no spin | yes | Uses `MotorController::visitDriver` after `Vortex` registers onboard SPI + PCAL control pins. |
| `vortex_bldc_open_loop` | Open-loop voltage segment | no | Short velocity command; **motion** — secure motor. |
| `vortex_bldc_velocity_foc_hall` | FOC + Hall velocity | no | Requires Hall wiring; **motion**. |
| `vortex_bldc_velocity_foc_abn` | FOC + ABN velocity | no | Set CPR to encoder; **motion**. |
| `vortex_bldc_position_foc_as5047` | Position PI + encoder angle log | no | Logs `EncoderManager` AS5047 sample; TMC path uses ABN CPR placeholder — see app header for SPI-enc upgrade. **Motion**. |
| `vortex_brake_dissipator_gpio` | TMC9660 GPIO17/18 toggles | no | Schematic-dependent loads — GPIO-level only unless qualified. |

**Do not** copy GPIO numbers from `hf-tmc9660-driver/examples/esp32/main/esp32_tmc9660_test_config.hpp` for Vortex; use `hf_functional_pin_config_vortex_v1.hpp` and `examples/esp32/main/common/vortex_board_pins.hpp`.

**Driver study index:** `examples/esp32/main/common/vortex_driver_reference.md`.
