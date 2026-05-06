# Vortex ESP32 examples — full index

This directory holds **example-specific** documentation: what each on-target app is for, which source file is the entry point, and how it relates to the HAL. **Board-level architecture** and manager theory live under **[`docs/hal/`](../../../docs/hal/README.md)** in the HAL repo root.

Also read:

- **[`BENCH_MATRIX.md`](BENCH_MATRIX.md)** — supply limits, motion warnings, CI vs manual benches.
- **[`../README.md`](../README.md)** — build/flash commands (`scripts/build_app.sh`, `APP_TYPE`).
- **[`../app_config.yml`](../app_config.yml)** — authoritative `APP_TYPE` → source mapping (below mirrors it).

## Build reminder

```bash
cd examples/esp32
./scripts/build_app.sh list
./scripts/build_app.sh vortex_api_test Release
```

## Apps (`app_config.yml`)

| `APP_TYPE` | Entry source | Category | CI | What it exercises |
|------------|----------------|----------|----|-------------------|
| `vortex_api_test` | [`main/vortex_api_test.cpp`](../main/vortex_api_test.cpp) | system | yes | Full `Vortex` bring-up, managers, diagnostics; primary integration reference |
| `gpio_stress_test` | [`main/gpio_stress_test.cpp`](../main/gpio_stress_test.cpp) | component | yes | GPIO stress / concurrency-oriented checks |
| `adc_calibration_test` | [`main/adc_calibration_test.cpp`](../main/adc_calibration_test.cpp) | component | yes | ADC path calibration / read patterns |
| `motor_controller_test` | [`main/motor_controller_test.cpp`](../main/motor_controller_test.cpp) | component | yes | `MotorController` / TMC9660 onboard **SPI** (default `Vortex` transport) |
| `motor_controller_uart_test` | [`main/motor_controller_test.cpp`](../main/motor_controller_test.cpp) | component | yes | Same suite with `Vortex::SetOnboardTmc9660Transport(Uart)` — onboard **UART**, SPI mux off |
| `encoder_imu_test` | [`main/encoder_imu_test.cpp`](../main/encoder_imu_test.cpp) | component | yes | `EncoderManager` + `ImuManager` |
| `led_temp_test` | [`main/led_temp_test.cpp`](../main/led_temp_test.cpp) | component | yes | `LedManager` + `TemperatureManager` |
| `vortex_led_ws2812_smoke` | [`main/vortex_led_ws2812_smoke.cpp`](../main/vortex_led_ws2812_smoke.cpp) | component | yes | WS2812 smoke; see bench matrix |
| `vortex_i2c_scan_pcal_bno_pca` | [`main/vortex_i2c_scan_pcal_bno_pca.cpp`](../main/vortex_i2c_scan_pcal_bno_pca.cpp) | component | yes | Shared I²C, PCAL, BNO08x, PCA9685 slot |
| `vortex_motor_comms_smoke` | [`main/vortex_motor_comms_smoke.cpp`](../main/vortex_motor_comms_smoke.cpp) | component | yes | TMC9660 telemetry / comms without motion |
| `vortex_bldc_open_loop` | [`main/vortex_bldc_open_loop.cpp`](../main/vortex_bldc_open_loop.cpp) | component | no | Open-loop segment — **motion**; see bench matrix |
| `vortex_bldc_velocity_foc_hall` | [`main/vortex_bldc_velocity_foc_hall.cpp`](../main/vortex_bldc_velocity_foc_hall.cpp) | component | no | FOC + Hall — **motion** |
| `vortex_bldc_velocity_foc_abn` | [`main/vortex_bldc_velocity_foc_abn.cpp`](../main/vortex_bldc_velocity_foc_abn.cpp) | component | no | FOC + incremental encoder — **motion** |
| `vortex_bldc_position_foc_as5047` | [`main/vortex_bldc_position_foc_as5047.cpp`](../main/vortex_bldc_position_foc_as5047.cpp) | component | no | Position loop + AS5047 angle — **motion** |
| `vortex_bldc_telemetry_sweep` | [`main/vortex_bldc_telemetry_sweep.cpp`](../main/vortex_bldc_telemetry_sweep.cpp) | component | yes | TMC9660 full bring-up + DRV_EN + telemetry/fault sweep — **no motion** |
| `vortex_bldc_torque_foc_hall` | [`main/vortex_bldc_torque_foc_hall.cpp`](../main/vortex_bldc_torque_foc_hall.cpp) | component | no | Hall FOC closed-loop torque (Iq command) — **motion** |
| `vortex_brake_dissipator_gpio` | [`main/vortex_brake_dissipator_gpio.cpp`](../main/vortex_brake_dissipator_gpio.cpp) | component | no | GPIO toggles for brake/dissipator path |

## Shared example code

| Path | Role |
|------|------|
| [`main/common/vortex_board_pins.hpp`](../main/common/vortex_board_pins.hpp) | Board pin constants for examples |
| [`main/common/vortex_bench_safety.hpp`](../main/common/vortex_bench_safety.hpp) | Current / supply guardrails for benches |
| [`main/common/vortex_driver_reference.md`](../main/common/vortex_driver_reference.md) | Driver study index for this board |

## Tie-back to HAL docs

After you know *which app* covers a behaviour, use **[`docs/hal/architecture.md`](../../../docs/hal/architecture.md)** for init order and degraded mode, and **[`docs/hal/managers-and-handlers.md`](../../../docs/hal/managers-and-handlers.md)** to jump to the manager and handler headers.
