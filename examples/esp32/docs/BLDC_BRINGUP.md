# TMC9660 BLDC on Vortex V1 — bring-up guide

This guide ties together **bootloader configuration**, **parameter-mode** motor setup, and the **ESP32 example apps** used to validate the HardFOC Vortex V1 power stage before you run closed-loop motion tests.

Companion docs:

- [`BENCH_MATRIX.md`](BENCH_MATRIX.md) — per-app goals, CI vs manual, supply and safety notes.
- [`BLDC_UART_BRINGUP.md`](BLDC_UART_BRINGUP.md) — UART open-loop spin, VGS-short workaround, fault-mask rationale (same power stage applies on SPI).
- [`README.md`](README.md) — full `APP_TYPE` index and build commands.

## Two configuration layers

1. **Bootloader / IO configuration** (persistent or applied at boot)  
   Selects communication (SPI vs UART), clocks, GPIO mux for Hall sensors, incremental encoder (ABN), optional SPI flash, brake outputs, and similar **pin-level** behaviour. Analog Devices documents this flow in **[AN-2601: TMC9660 Configuration and Bootstrapping](https://www.analog.com/media/en/technical-documentation/app-notes/an-2601.pdf)** (UblTools, `ioconfig_tmc9660-3ph-eval.toml` for the **TMC9660-3PH-EVAL** class of hardware).

2. **Parameter mode** (runtime over TMCL after the chip has left the bootloader path your firmware uses)  
   Configures **gate driver**, **current sensing**, **motor profile** (BLDC, pole pairs, PWM), **FOC loops**, **protection**, **ramp**, **heartbeat**, **power management**, **ADC offset calibration**, then **`DRV_EN`**. The Vortex bench helpers mirror the proven sequence in the submodule example `hf-tmc9660-driver/examples/esp32/main/bldc_comprehensive_test.cpp::configureCompleteBLDCMotor()`.

Application code on Vortex typically gets a live driver through **`MotorController::visitDriver()`** after `Vortex::EnsureInitialized()` (see [`MotorController.h`](../../../lib/managers/MotorController.h)). To use a **non-default bootloader image** when creating the onboard device, pass a `const tmc9660::BootloaderConfig*` into **`MotorController::CreateOnboardDevice(..., bootCfg)`** (SPI or UART overload). The stock handler default leaves **Hall** and **ABN1** disabled in boot config; Hall/ABN FOC examples need a custom `BootloaderConfig` that matches your wiring (EvKit reference map: Hall GPIO2/3/4, ABN1 GPIO8/13/14 — **do not** assume those GPIO numbers on Vortex; align to your schematic and `hf_functional_pin_config_vortex_v1.hpp`).

## Source files (this repo)

| Path | Role |
|------|------|
| [`main/common/vortex_bench_safety.hpp`](../main/common/vortex_bench_safety.hpp) | Single source of truth for bench supply, shunt, FET model, PWM, current caps, I²t, thermal limits, motion profile timings, encoder CPR. **Tune here first** when BOM or motor changes. |
| [`main/common/vortex_motor_bench_common.hpp`](../main/common/vortex_motor_bench_common.hpp) | `configure_complete_bldc()` and step helpers: gate driver, current sense, motor, torque/flux + velocity PI, protection, ramp (direct velocity), brake/stop defaults, **heartbeat**, **power**, optional ADC offset cal, **`DRV_EN`**. Also **`disable_vortex_uvw_vgs_short_protection()`**, **`bench_open_loop_motion_fault_critical()`**, and **`kBenchOpenLoopIgnoredGeneralErrors`** for Vortex-class open-loop (see [`BLDC_UART_BRINGUP.md`](BLDC_UART_BRINGUP.md)). |
| [`lib/core/handlers/tmc9660/Tmc9660Handler.cpp`](../../../lib/core/handlers/tmc9660/Tmc9660Handler.cpp) | `kDefaultBootConfig` — EvKit-oriented parameter/SPI defaults; Hall/ABN off unless you override at `CreateOnboardDevice`. |
| [`lib/core/hf-core-drivers/external/hf-tmc9660-driver/`](../../../lib/core/hf-core-drivers/external/hf-tmc9660-driver/) | `tmc9660::TMC9660<Comm>`, `BootloaderConfig`, TMCL types. |

Driver study index for navigating the submodule: [`main/common/vortex_driver_reference.md`](../main/common/vortex_driver_reference.md).

## Thirteen-step parameter-mode order (bench)

Used by all `vortex_bldc_*` apps via `configure_complete_bldc()`:

1. `SYSTEM_OFF`  
2. Motor (`MotorProfile` — **BLDC type and pole pairs before gate driver** so Y2-phase SAPs are not rejected)  
3. Gate driver (`PowerStageProfile` — shunt, bus voltage, PWM frequency, FET Qg/Rds(on), UV level, …)  
4. Current sensing (`AutoConfig`)  
5. Torque/flux + velocity PI (`configureAuto`)  
6. Protection (OV/UV, overtemperature, overcurrent, I²t)  
7. Ramp (direct velocity mode on, ramp generator off by default)  
8. Brake (defaults off — enable only if your schematic uses chopper/mech brake like the eval board)  
9. Stop events (defaults)  
10. Heartbeat (defaults — watchdog off unless you enable it in config)  
11. Power management (defaults)  
12. ADC offset calibration (motor stationary, `SYSTEM_OFF`)  
13. **`DRV_EN` active** — outputs live; commutation mode still `SYSTEM_OFF` until the app sets it.

The app then selects **`FOC_OPENLOOP_CURRENT_MODE`** or **`FOC_OPENLOOP_VOLTAGE_MODE`**, **`FOC_HALL_SENSOR`**, **`FOC_ABN`**, etc., and issues **`setTargetVelocity`** / **`setTargetTorque`**.

**Vortex open-loop prerequisite:** before `FOC_OPENLOOP_CURRENT_MODE`, call
`disable_vortex_uvw_vgs_short_protection()` (or equivalent SAPs). The TMC9660
ROM enables UVW VGS-short protection by default; on the Vortex 3-phase stage
this trips spurious gate-driver faults and the fault handler reverts
`COMMUTATION_MODE` to `SYSTEM_OFF`, so **`TARGET_VELOCITY` SAPs return
`REPLY_INVALID_VALUE`**. See [`BLDC_UART_BRINGUP.md`](BLDC_UART_BRINGUP.md).

## Vortex vs TMC9660-3PH-EVKIT example code

- **Pin map:** Always use **`vortex_board_pins.hpp`** and the generated functional pin header — never copy GPIO numbers from `hf-tmc9660-driver/examples/esp32/main/esp32_tmc9660_test_config.hpp`.
- **Motor inductance in the gate profile:** The EvKit comprehensive example uses a small **50 µH** placeholder; Vortex **`vortex_bench_safety.hpp`** uses **~390 µH** per phase for the documented 24 V / 30 W reference motor (derived from line-line inductance). Revisit if you change motors.
- **Brake:** Bench helpers leave brake/chopper configuration at benign defaults; enable to match your PCB if you have a dissipative brake path wired like the eval kit.

## Suggested on-bench validation order

1. **`vortex_motor_comms_smoke`** — confirms SPI (or your transport) and basic telemetry without arming motion beyond what the app does.  
2. **`vortex_bldc_telemetry_sweep`** — full parameter-mode stack including **`DRV_EN`**, **no commutation**; safe with motor disconnected; read supply, die temperature, fault flags.  
3. **`vortex_bldc_open_loop_uart`** (recommended) or **`vortex_bldc_open_loop`** — open-loop current-mode motion with VGS workaround; secured shaft; see [`BLDC_UART_BRINGUP.md`](BLDC_UART_BRINGUP.md).  
4. **`vortex_bldc_velocity_foc_hall`** / **`vortex_bldc_velocity_foc_abn`** — only after bootloader mux matches the sensor wired; see app file headers.  
5. **`vortex_bldc_torque_foc_hall`** — torque command; **high risk** if unloaded (shaft can run away); use load or very conservative `kTorqueModeTargetMa` in `vortex_bench_safety.hpp`.

Always use a current-limited supply, secured coupling, and an appropriate emergency stop for any motion test.

## Builds and tooling

- Each `APP_TYPE` uses its own build directory under `examples/esp32/builds/` (see `build_app.sh`).  
- Running **several `build_app.sh` processes in parallel** can occasionally corrupt incremental artifacts (`*.obj: file truncated`). If a link step fails mysteriously, rebuild that app with **`--clean`**.  
- Commands: [`../README.md`](../README.md).

## Further reading

- [TMC9660 product page](https://www.analog.com/en/products/tmc9660.html) (documentation hub).  
- Submodule: **`bldc_comprehensive_test.cpp`** — scenario reference for open-loop, Hall FOC, and ABN FOC (transport and pin constants still Vortex-specific).

## SPI vs UART for BLDC benches

Both buses use the same **logical** TMCL fields (opcode, 12-bit type, motor bank,
32-bit value); the **byte packing** of type + motor differs between the 8-byte
SPI frame and the 9-byte UART frame — see `TMCLFrame::toSpi` / `toUart` in
`tmc9660_comm_interface.hpp`. For **open-loop spin on
Vortex hardware**, SPI and UART both need the **UVW VGS-short disable**
sequence from [`BLDC_UART_BRINGUP.md`](BLDC_UART_BRINGUP.md); the historical
SPI-only "enable failed" symptom is consistent with commutation reverting to
`SYSTEM_OFF` after spurious `GDRV_ERROR`, not with a broken SPI frame.

**Verified parity (2026-05):** with the bench helper defaults updated to
`with_oc_vgs_protection=false`, `with_gate_current_limits=false`,
`skip_y2_phase=true`, both transports complete `configure_complete_bldc()`
cleanly:

* `vortex_bldc_open_loop` (forces SPI) — full ramp + cruise + ramp-down,
  `phi_e` advancing, `Iuvw`/`Uuvw` rotating, `gd_err = 0x00000000`,
  `Open-loop profile finished OK`.
* `vortex_bldc_open_loop_uart` (UART) — identical motion profile,
  `gd_err = 0`, `pos 475 → 41 417` over the 5 s ramp/cruise/ramp-down.
* `vortex_bldc_telemetry_sweep` (UART) — `DRV_EN` armed, all 10 telemetry
  samples printed, `Telemetry sweep complete`.
* `vortex_bldc_velocity_foc_hall` (UART, FOC_HALL_SENSOR) — mode arms
  cleanly (`CM_rb=6`, no fault, no revert). Shaft only spins when Hall
  sensors are physically wired to TMC9660 GPIO2/3/4 (EvKit map).
* `vortex_bldc_velocity_foc_abn` (UART, FOC_ABN) — mode arms cleanly
  (`CM_rb=7`, no fault, no revert). Shaft only spins when ABN encoder is
  physically wired to TMC9660 GPIO8/13/14 (EvKit map).

The earlier `gateDriver.configurePowerStageProtection failed` /
`REPLY_INVALID_CMD` / `REPLY_INVALID_VALUE` regression was caused by the
helper sending Y2-phase + VGS-short-on/off SAPs that the Vortex 3-phase
firmware (FW051V100) rejects, not by a broken SPI nibble packing. The
follow-up `INVALID_VALUE` rejection on **NR 245/246** (UVW gate-current
sink/source) is a separate FW051V100 quirk — confirmed independent of
`MOTOR_TYPE` ordering — that the bench works around by **leaving the
chip's silicon defaults in place** (270 mA sink / 135 mA source); see
[`BLDC_UART_BRINGUP.md`](BLDC_UART_BRINGUP.md) "Gate-current SAP
rejection on FW051V100" for the diagnostic and the EvKit-ordering
verification.

**SPI-only follow-up:** if TMCL still returns `INVALID_CMD` with correct
12-bit TYPE packing and valid SAPs, verify pacing between the two SPI
transactions of one TMCL datagram (`SpiCommInterface::transferTMCL` delay
before the NO_OP phase) and the post-transfer delay in
`HalSpiTmc9660Comm::spiTransferTMCL` — fast SPI without enough gap can race
the chip’s TMCL parser even when bootloader `GetVersion` / `MST` checks pass.
