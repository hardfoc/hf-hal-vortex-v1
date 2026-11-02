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
| `visitDriver(func, index = 0)` | Calls `func(auto& driver)` with the typed TMC9660 driver (SPI or UART variant); the way to reach every TMCL parameter |
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
