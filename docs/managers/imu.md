# ImuManager

Owns the BNO08x IMUs: the onboard one on I2C at 0x4A, and up to three external ones on I2C or SPI. The BNO08x runs
its own sensor fusion; this manager creates its handler (which drives the reset and boot lines on the PCAL95555) and
optionally wires its data-ready interrupt. Source: [`ImuManager.h`](../../lib/managers/ImuManager.h).

## Devices

| Index | Device | Created by |
| ---: | --- | --- |
| 0 | Onboard BNO08x (I2C 0x4A) | `ImuManager::EnsureInitialized()` |
| 1 to 3 | External BNO08x | You: `CreateExternalBno08xDevice(index, i2c_address, speed_hz)` or `CreateExternalBno08xDevice(index, SpiDeviceId)` |

## API

| Call | Notes |
| --- | --- |
| `GetBno08xHandler(index = 0)` | `Bno08xHandler*`, or `nullptr` |
| `GetSensor(index = 0)` | `IBno08xDriverOps*`: the driver itself (enable reports, read the latest data) |
| `CreateExternalBno08xDevice(...)`, `DeleteExternalDevice(index)` | External IMUs |
| `ConfigureInterrupt(index, callback)`, `EnableInterrupt`, `DisableInterrupt`, `IsInterruptEnabled`, `WaitForInterrupt(index, timeout_ms)` | Data-ready interrupt; the onboard IMU uses `PCAL_IMU_INT` through `GpioManager` |
| `IsDeviceValid`, `GetActiveDeviceIndices`, `GetAvailableDevices`, `GetDeviceType` | Device bookkeeping |
| `GetLastError()`, `GetSystemDiagnostics(ImuSystemDiagnostics&)`, `DumpStatistics()` | Health per device |

On the handler, `Update()` services the SH-2 protocol and dispatches new reports to the callback set with
`SetSensorCallback()`; `visitDriver()` runs a function on the driver under the handler's lock.

## Example: poll the onboard IMU

```cpp
#include "api/Vortex.h"
#include "managers/ImuManager.h"

auto& imu = Vortex::GetInstance().imu;
if (Bno08xHandler* h = imu.GetBno08xHandler(); h != nullptr) {
    // Set a callback once with h->SetSensorCallback(...), then service the device periodically:
    (void)h->Update();
}
```

The bench app `vortex_pcal_gpio_imu_bringup` and the manager test `encoder_imu_test` in
[Examples](../examples/index.md) show reports being enabled and read.

## Diagnostics

`GetSystemDiagnostics()` fills `ImuSystemDiagnostics`: whether the manager is up, active and initialised device
counts, the last error, and a snapshot per device.

## Pitfalls

- **The IMU's control lines are on the expander.** Reset, boot and interrupt go through the PCAL95555 over I2C, so
  `GpioManager` must be up first (the facade orders this).
- **Call `Update()` often enough.** The BNO08x queues reports; if nothing services it, reports are dropped.
- **I2C is shared** with the PCAL95555. Long I2C transactions from your code delay IMU servicing.
