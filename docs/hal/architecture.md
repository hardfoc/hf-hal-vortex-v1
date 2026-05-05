# Vortex HAL — architecture (as implemented)

This page describes the **Vortex V1** HAL as the C++ sources lay it out today. When it disagrees with marketing-style README prose elsewhere, **trust the headers and `Vortex.cpp`**.

## Layers (bottom to top)

1. **MCU / ESP-IDF** — SPI, I²C, UART, TWAI, GPIO, RMT, NVS, etc.
2. **`hf-internal-interface-wrap` and related** — `BaseSpi`, `BaseI2c`, `BaseUart`, `BaseGpio`, `BaseAdc`, … (see handbook [base-interfaces](../hf-development-handbook/process/base-interfaces.md)).
3. **`hf-core` drivers** — device logic (e.g. TMC9660, AS5047U, BNO08x) templated or standalone under `lib/core/` (submodule).
4. **`hf-core` handlers** — one device façade per handler; owns driver + comm adapters; MCU-agnostic (e.g. `Tmc9660Handler` in `lib/core/handlers/`).
5. **HAL managers** (`lib/managers/*.h`) — **board-aware** singletons: fixed registries, pin/BOM knowledge, `EnsureInitialized` / `Shutdown`, mutexes, diagnostics.
6. **`Vortex` API** (`lib/api/Vortex.h`, `Vortex.cpp`) — single entry point: wires manager singletons in dependency order, exposes `vortex.nvs`, `vortex.comms`, `vortex.gpio`, …, aggregates diagnostics and `CollectManagerHealth()`.
7. **Application** — `app_main` in `examples/esp32/main/*.cpp` (or product firmware) calls `Vortex::GetInstance().EnsureInitialized()` and uses managers through **`vortex.*`** (recommended).

The handbook’s [layered architecture](../hf-development-handbook/process/layered-architecture.md) is the generic picture; the sections below are **Vortex-specific**.

## `Vortex` singleton: members and roles

Public references (after `EnsureInitialized()`):

| Member | Manager class | Role (short) |
|--------|---------------|--------------|
| `nvs` | `NvsManager` | NVS namespaces for cal/config early in boot |
| `comms` | `CommChannelsManager` | SPI / I²C / UART / CAN used by the rest of the board |
| `gpio` | `GpioManager` | ESP32 + PCAL95555 (+ views into TMC9660 GPIO where applicable) |
| `motors` | `MotorController` | TMC9660 instances (onboard + optional externals) |
| `adc` | `AdcManager` | Functional ADC channels; **Vortex routes these through the TMC9660 path** (see `AdcManager.h`) |
| `imu` | `ImuManager` | BNO08x on I²C |
| `encoders` | `EncoderManager` | AS5047U on SPI |
| `leds` | `LedManager` | WS2812 status |
| `temp` | `TemperatureManager` | NTC / chip / derived temps using ADC + motor data |

`Vortex` holds **references** to existing manager singletons (see `Vortex::Vortex()` in `lib/api/Vortex.cpp`); it does not replace `GetInstance()` on each manager.

## Initialization order and “degraded” bring-up

Authoritative sequence is documented in **`lib/api/Vortex.h`** and implemented in **`Vortex::InitializeAllComponents()`** in `lib/api/Vortex.cpp`:

1. **NvsManager** — required.
2. **CommChannelsManager** — required (buses).
3. **GpioManager** — required.
4. **LedManager** — required early so **LED-only** benches work without a working TMC9660.
5. **MotorController** — best-effort: failure logs a **warning**, not a hard fail of the whole API.
6. **AdcManager** — best-effort (depends on motors / TMC9660 bring-up for the Vortex ADC story).
7. **ImuManager**, **EncoderManager**, **TemperatureManager** — best-effort.

**Success rule:** `EnsureInitialized()` returns **true** when **NVS + comms + GPIO + LEDs** are all up (`core_ok` in `Vortex.cpp`). Optional subsystems may be off; warnings describe **degraded** mode (motors/ADC/IMU/encoders/temp).

Shutdown in `Vortex::Shutdown()` tears down in **reverse** order (temp → … → comms → nvs).

## Diagnostics and health

- **`VortexSystemDiagnostics`** — per-flag initialization status and counters (see `Vortex.h`).
- **`CollectManagerHealth(ManagerHealthSnapshot&)`** — unified view: each manager’s last error cast to `uint8_t` (interpret per manager enum as documented on `ManagerHealthSnapshot` in `Vortex.h`).
- **`PerformHealthCheck()`** — logs per-manager line items using the snapshot.

## Pin and board truth

Functional pin names and enums are generated/centralized under:

- `lib/core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp`

Example apps use **`examples/esp32/main/common/vortex_board_pins.hpp`** and **`vortex_driver_reference.md`** for bench-oriented notes. Do **not** assume EVKit or other boards’ GPIO numbers apply to Vortex.

## Where to go deeper

| Topic | Location |
|--------|----------|
| Manager behaviour and registries | `lib/managers/*.h` (Doxygen in headers) |
| Handler APIs | `lib/core/handlers/<device>/` |
| On-target apps and CI matrix | [`examples/esp32/docs/README.md`](../../examples/esp32/docs/README.md), [`examples/esp32/docs/BENCH_MATRIX.md`](../../examples/esp32/docs/BENCH_MATRIX.md) |
| Quick manager ↔ handler map | [managers-and-handlers.md](managers-and-handlers.md) |
