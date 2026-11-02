# MotorController

Owns the TMC9660 motor controllers: the onboard one, and up to two more on the external SPI chip selects. The
TMC9660 runs the motor loops itself (FOC current, velocity and position, ramps); this manager creates its handler,
manages the host link and gives you unit-aware velocity and position calls in the motor frame or, through a gear
ratio, the load frame. Source: [`MotorController.h`](../../lib/managers/MotorController.h).

## Devices

| Index | Device | Created by |
| ---: | --- | --- |
| 0 | Onboard TMC9660 | `Vortex::EnsureInitialized()` (start-up step 5), on UART by default or SPI if selected |
| 2, 3 | External TMC9660 on `EXTERNAL_DEVICE_1` / `EXTERNAL_DEVICE_2` | You: `CreateExternalDevice()` |

The onboard device needs the four control lines on the PCAL95555 (reset, driver enable, fault, wake), which the
facade takes from `GpioManager`. The transport is chosen with `Vortex::SetOnboardTmc9660Transport()` before
initialisation; see [Initialisation](../architecture/initialization.md#the-onboard-tmc9660-and-its-transport).

## API

| Call | Notes |
| --- | --- |
| `handler(index = 0)` | `Tmc9660Handler*`, or `nullptr` if there is no device at that index |
| `visitDriver(func, index = 0)` | Calls `func(auto& driver)` with the typed TMC9660 driver (SPI or UART variant) while holding the device lock and the handler lock; the way to reach every TMCL parameter |
| `setMotorVelocity` / `getMotorVelocity`, `setMotorPosition` / `getMotorPosition` | Motor (rotor) frame; take a value, a `tmc9660::units` unit and a `MotorContext` |
| `setMotorMechanics(MotorMechanics)`, `getMotorMechanics()` | Gear ratio (motor turns per output turn) and output inversion |
| `setLoadVelocity` / `getLoadVelocity`, `setLoadPosition` / `getLoadPosition` | Load (output shaft) frame, through the mechanics |
| `CreateExternalDevice(cs_index, ...)`, `DeleteExternalDevice(cs_index)` | External TMC9660 over SPI or UART |
| `IsDeviceValid`, `GetActiveDeviceIndices`, `InitializeAllDevices`, `GetInitializationStatus` | Device bookkeeping |
| `GetLastError()`, `GetSystemDiagnostics(MotorSystemDiagnostics&)`, `DumpStatistics()` | Health per device |

A `MotorContext` tells the unit conversions about the motor: motor type, pole pairs, velocity sensor and encoder
counts per revolution.

## Example: run the load shaft at 30 rpm

```cpp
#include "api/Vortex.h"
#include "managers/MotorController.h"

using ::tmc9660::units::VelocityUnit;

constexpr ::tmc9660::units::MotorContext kCtx{
    tmc9660::tmcl::MotorType::BLDC_MOTOR,
    7,                                                  // pole pairs of your motor
    tmc9660::tmcl::VelocitySensorSelection::DIGITAL_HALL,
    0u,                                                 // encoder counts per revolution (none)
};

auto& motors = Vortex::GetInstance().motors;
if (motors.IsInitialized() && motors.handler() != nullptr) {
    (void)motors.setMotorMechanics(MotorMechanics{10.0, false});   // 10:1 reducer
    (void)motors.setLoadVelocity(30.0, VelocityUnit::Rpm, kCtx);
}
```

Commanding motion assumes the TMC9660 has been configured for your motor (type, pole pairs, current limits,
feedback, commutation mode). The [BLDC bring-up notes](../../examples/esp32/docs/BLDC_BRINGUP.md) and the motor
bench examples (`vortex_bldc_*` in [Examples](../examples/index.md)) show the full sequence.

## Diagnostics

`GetSystemDiagnostics()` fills `MotorSystemDiagnostics`: whether the manager is up, how many device slots are active
and initialised, the last error, and per device whether it is active, initialised and its error count.

## Drive observability

The hooks the [drive observability design](https://github.com/hardfoc/hardfoc-vortex-v1/blob/main/docs/design/drive-observability.md)
needs from the HAL. Each call is one pipelined TMCL batch (on SPI, N accesses cost N + 1 frames instead of 2N), sent
through the handler's `visitDriver()` with the device lock held. A call never starts a bootloader bring-up: a device
that is not up returns `NOT_INITIALIZED` and sends nothing. The helpers live in
[`api_helpers/Tmc9660DriveObservability.h`](../../lib/managers/api_helpers/Tmc9660DriveObservability.h).

| Call | Does |
| --- | --- |
| `ReadParameters(span<MotorParameterAccess>)`, `WriteParameters(...)` | Batch GAP / SAP of any axis parameters; per-element value and TMCL status |
| `ReadTelemetry(set, MotorTelemetrySnapshot&)` | Refreshes a caller-owned snapshot. `Cyclic` = the design's fixed slots (position 144, velocity 125, packed Iq/Id 331, status 289, and the error words 299 / 300 on alternate calls: 5 accesses); `Status` = status, error, ADC, supply, temperatures, total current, I²t sums; `Full` = all 16 fields. Each field keeps the sequence of its last good read (`Age()`), a failed access keeps the old value, and the batch time is recorded |
| `ReadDriveDiagnostics(MotorDriveDiagnostics&)` | Reads 289/299/300/301 in one batch and decodes them: regulation stopped, chip reset seen (watchdog bit, so re-apply the image), heartbeat stopped, retries failed, thermal and I²t conditions, gate-driver faults per phase and side, ADC clipping, and a severity (`Ok`, `Warning`, `Fault`); on SPI also the TMCL link counters |
| `ClearLatchedFaults(general, gate_driver, adc)` | Write-1-to-clear of the latched bits |
| `ApplyParameterImage(image, options, result)` | Writes a parameter image with `MOTOR_TYPE` first (FW051V100 answers most parameters with `WRONG_TYPE` until it is set); `differential` writes only what differs on the chip, `verify` reads everything back. The result counts written, skipped, rejected and mismatched entries and names the first bad one. The TMC9660's own NVM is not used |
| `GetAccessStats()`, `ResetAccessStats()` | Per device: batches, accesses, failed accesses, last and longest batch time |

Values are raw chip units. The snapshot's helpers convert what the design marks as solid (supply 0.1 V, chip
temperature, total current in mA); the packed 331/332 values stay unscaled until calibrated. The CANopen object
dictionary, the CLI and the dashboard that use these hooks belong to the product firmware, not to this HAL.

```cpp
auto& motors = Vortex::GetInstance().motors;
MotorTelemetrySnapshot snap{};          // keep it: ages are counted across calls
if (motors.ReadTelemetry(MotorTelemetrySet::Cyclic, snap) == MotorError::SUCCESS) {
    const int32_t pos = snap.ActualPosition();
    const int16_t iq_raw = snap.TorqueCurrentRaw();
}
MotorDriveDiagnostics diag{};
if (motors.ReadDriveDiagnostics(diag) == MotorError::SUCCESS && diag.chip_reset_detected) {
    // the TMC9660 restarted: apply the parameter image again
}
```

Anything else on the chip (every axis and global parameter, RAMDebug) is reached through `visitDriver()`.

## Pitfalls

- **No motor manager, no TMC9660 features.** If the onboard TMC9660 does not come up (for example with its supply
  off), step 5 fails with a warning. Everything that depends on the TMC9660 (ADC channels, the NTC and chip temperatures,
  `TMC_GPIO17/18`) is then unavailable too.
- **Pick the transport before start-up.** `SetOnboardTmc9660Transport()` after `EnsureInitialized()` is ignored.
- **The SPI mux is shared.** On SPI, `MotorController` asserts `PCAL_TMC_SPI_COMM_EN` around its transfers; other
  SPI2 users must not assume the TMC9660 is off the bus.
- **Index 1 is not a device.** External devices use the chip-select indices 2 and 3.
- **Do not copy evaluation-kit pin numbers.** Hall and ABN wiring on Vortex need not match the TMC9660 kit; use the
  [pin map](../hardware/pin-map.md) and the bring-up notes.
