# Vortex V1 — bench matrix (ESP32-C6 HAL examples)

Long-form TMC9660 BLDC context (bootloader vs runtime, suggested flash order): **[`BLDC_BRINGUP.md`](BLDC_BRINGUP.md)**.

Nominal bench supply: **24 V DC**. Phase / torque current cap for the supplied
24 V / 30 W reference motor: **1.5 A** (`vortex_bench_safety.hpp`). Motion
profiles are **time-bounded** in firmware; use an e-stop and a secured shaft.

## TMC9660 BLDC bring-up shape

All `vortex_bldc_*` apps share `main/common/vortex_motor_bench_common.hpp`,
which mirrors the EvKit `bldc_comprehensive_test::configureCompleteBLDCMotor()`
ordering for parameter mode:

1. `SYSTEM_OFF` → 2. motor (`MotorProfile`, **BLDC + pole pairs before gate**) → 3. gate driver
(`PowerStageProfile`) → 4. current sensing (`AutoConfig`) → 5. FOC PI
gains (torque/flux + velocity) → 6. protection (OV/UV, OT, OC, I²t) → 7. ramp
(direct velocity mode by default) → 8. brake (off by default) → 9. stop events →
10. heartbeat (defaults) → 11. power management (defaults) → 12. ADC offset
calibration → 13. **`DRV_EN` active** (ordering matches `configure_complete_bldc()`; motor
type is set **before** gate driver so 3-phase BLDC firmware accepts UVW-only programming).

For the historical EvKit `configureCompleteBLDCMotor()` sequence (gate before motor), see
`bldc_comprehensive_test.cpp` — Vortex uses motor-first to avoid `REPLY_WRONG_TYPE` on Y2 SAPs.

After step 13 the chip is **armed but in `SYSTEM_OFF`**. The app then sets the
commutation mode (`FOC_OPENLOOP_VOLTAGE_MODE`, `FOC_HALL_SENSOR`, `FOC_ABN`,
…) and writes `setTargetVelocity(...)` or `setTargetTorque(...)`.

Every electrical/mechanical/firmware constant lives in
`main/common/vortex_bench_safety.hpp` (motor specs, MOSFET specs, shunt R,
PWM, target velocities/torques, profile durations, encoder CPR).

> **Hall / ABN feedback** also needs the TMC9660 **bootloader** to mux the
> sensor pins. Vortex's stock `Tmc9660Handler::kDefaultBootConfig` leaves
> `cfg.hall.enable = false` and `cfg.abn1.enable = false`. To use the
> Hall/ABN apps, supply a custom `BootloaderConfig*` to
> `MotorController::CreateOnboardDevice(...)` with those enabled (Hall →
> GPIO2/3/4, ABN1 → GPIO8/13/14 in the EvKit reference).

### `vortex_api_test` — strict hardware mode

For a **fully powered and wired** Vortex bench, enable Kconfig options (then rebuild):

`idf.py menuconfig` → **Component config** → **Main** → **Vortex API test (vortex_api_test)**

- **`CONFIG_VORTEX_API_TEST_STRICT_HW`**: require VM rail in 12–32 V, TMC `FAULT_n` high (not faulted), expected GPIO names, comms initialized, WS2812 diagnostics healthy, ESP32 die temp read OK.
- **`CONFIG_VORTEX_API_TEST_STRICT_SENSORS`** (depends on strict HW): require IMU + encoder enumeration, successful encoder angle read, and ≥1 motor device.

Defaults are **off** so CI and USB-debug builds stay permissive. Alternatively set the same symbols in `sdkconfig.defaults` for a team bench profile.

| App (`app_config.yml`) | Goal | CI | Bench status (FW051V100, 2026-05) | Notes |
| ---------------------- | ---- | -- | ---- | ----- |
| `vortex_led_ws2812_smoke` | WS2812 / `LedManager` | yes | n/a | GPIO3 data; demo runs boot pulse, RGB sweep, rainbow, breath, blink, status colours, ends dim green (~20 s). |
| `vortex_i2c_scan_pcal_bno_pca` | Shared I2C + PCAL + BNO + PCA9685 | yes | n/a | SDA 21 / SCL 22; runtime slot @ 0x40 for `Pca9685Handler`. |
| `vortex_motor_comms_smoke` | TMC9660 telemetry, no spin | yes | n/a | Uses `MotorController::visitDriver` after `Vortex` registers onboard SPI + PCAL control pins. |
| `vortex_bldc_telemetry_sweep` | Full TMC9660 BLDC config + DRV_EN, **no commutation** — telemetry only | yes | ✅ verified | Safe with motor unplugged; exercises gate driver / current sense / protection auto-config and reads supply, chip temp, fault flags. Houses the **NR 245-248 INVALID_VALUE diag** (off by default; see `kBenchPostTmclGateCurrentDiag` and [`BLDC_UART_BRINGUP.md`](BLDC_UART_BRINGUP.md)). |
| `vortex_bldc_open_loop` | Open-loop FOC **current-mode** spin (no sensor) | no | ✅ **spins** (gd_err=0 thru ramp/cruise/down) | **SPI** (`SetOnboardTmc9660Transport(Spi)`). Full bring-up + UVW VGS-short workaround + ramper + `FOC_OPENLOOP_CURRENT_MODE` + `setTargetVelocity`. Uses chip-default gate currents (NR 245/246 = 4/4) which are **enough** for 58 nC FETs at 24 V; see [`BLDC_UART_BRINGUP.md`](BLDC_UART_BRINGUP.md) "Gate-current SAP rejection". **Motion** — secure shaft, 12–24 V. |
| `vortex_bldc_open_loop_uart` | Same open-loop profile over **UART** | no | ✅ **spins** (gd_err=0; `pos 475 → 41 417`) | Forces UART transport; PCAL line dump; ramp/cruise/ramp-down with `phi_e`/phase telemetry. **Motion** — [`BLDC_UART_BRINGUP.md`](BLDC_UART_BRINGUP.md). |
| `vortex_bldc_velocity_foc_hall` | Hall-FOC velocity loop | no | ✅ mode arms cleanly (no fault, no revert); ⚠️ shaft static if Hall sensors not wired on GPIO2/3/4 | Same bring-up + UVW VGS-short workaround + `configureAuto(HallConfig)` + `clear_fault_flags` + `FOC_HALL_SENSOR` + post-mode-change `COMMUTATION_MODE` read-back retry. Requires Hall enabled in bootloader **and physical Hall sensors on GPIO2/3/4**. **Motion**. |
| `vortex_bldc_torque_foc_hall` | Hall-FOC closed-loop torque (Iq command) | no | ⚠️ same Hall-wiring caveat | `setTargetTorque(kTorqueModeTargetMa)` instead of velocity. Shares the Hall-FOC bring-up; the same VGS workaround applies (apply via `disable_vortex_uvw_vgs_short_protection` if extending this app). **Motion** — load shaft, motor will speed up freely if unloaded. |
| `vortex_bldc_velocity_foc_abn` | ABN-FOC velocity loop | no | ✅ mode arms cleanly (no fault, no revert); ⚠️ shaft static if ABN encoder not wired on GPIO8/13/14 | Same bring-up + UVW VGS-short workaround + `configureAuto(AbnConfig{CPR})` + `clear_fault_flags` + `FOC_ABN` + post-init read-back retry; encoder runs forced phi_e zero swing on init. Requires ABN enabled in bootloader **and physical encoder on GPIO8/13/14**. **Motion**. |
| `vortex_bldc_position_foc_as5047` | Position PI + encoder angle log | no | ⚠️ host-side AS5047 read OK; TMC closed-loop will sit static unless AS5047 is wired through SPI-enc on the TMC | Logs `EncoderManager` AS5047 sample; TMC path uses ABN CPR placeholder — see app header for SPI-enc upgrade. **Motion**. |
| `vortex_brake_dissipator_gpio` | TMC9660 GPIO17/18 toggles | no | n/a | Schematic-dependent loads — GPIO-level only unless qualified. |

**Do not** copy GPIO numbers from `hf-tmc9660-driver/examples/esp32/main/esp32_tmc9660_test_config.hpp` for Vortex; use `hf_functional_pin_config_vortex_v1.hpp` and `examples/esp32/main/common/vortex_board_pins.hpp`.

**Driver study index:** `examples/esp32/main/common/vortex_driver_reference.md`.
