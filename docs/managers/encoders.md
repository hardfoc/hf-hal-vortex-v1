# EncoderManager

Owns up to four AS5047U magnetic angle encoders. Vortex V1 has a chip select wired for one encoder
(`AS5047U_POSITION_ENCODER`, GPIO20) and two external chip selects that can each take another, but **no encoder is
created automatically**: you create each device. Source: [`EncoderManager.h`](../../lib/managers/EncoderManager.h).

## API

| Call | Notes |
| --- | --- |
| `CreateAs5047uDevice(index, SpiDeviceId, config = default)` | Creates an encoder on one of the board's chip selects; also an overload taking a `BaseSpi&` |
| `DeleteDevice(index)` | Removes it |
| `ReadAngle(index, uint16_t& counts)`, `ReadAngleDegrees(index, double& deg)` | 14-bit angle, or degrees |
| `ReadVelocityRPM(index, double& rpm)` | Velocity from the sensor |
| `ReadDiagnostics(index, As5047uDiagnostics&)`, `SetZeroPosition(index, counts)` | Sensor diagnostics, zero offset |
| `ReadAllAngles(...)`, `ReadAllVelocities(...)`, `CheckAllDevicesHealth()` | Every active device, under the manager lock |
| `GetAs5047uHandler(index)`, `GetAs5047uDriver(index)` | The handler or the typed driver for chip features |
| `IsDeviceValid`, `GetActiveDeviceIndices`, `GetAvailableDevices`, `GetDeviceType` | Device bookkeeping |
| `GetLastError()`, `GetSystemDiagnostics(EncoderSystemDiagnostics&)`, `DumpStatistics()` | Counters and the last error |

## Example: create the onboard-slot encoder and read it

```cpp
#include "api/Vortex.h"
#include "managers/EncoderManager.h"

auto& enc = Vortex::GetInstance().encoders;
if (enc.CreateAs5047uDevice(0, SpiDeviceId::AS5047U_POSITION_ENCODER)) {
    double deg = 0.0;
    if (enc.ReadAngleDegrees(0, deg) == EncoderError::SUCCESS) {
        // deg is the shaft angle, 0 to 360
    }
}
```

## Pitfalls

- **Nothing to read until you create a device.** `EnsureInitialized()` succeeds with zero devices.
- **SPI2 is shared** with the TMC9660 (when on SPI) and the external slots.
- **Single-device reads take no manager lock.** Read one encoder from one task, or use `ReadAllAngles()`. See
  [Threading](../architecture/threading.md).
- **The TMC9660 can use the same encoder.** The position benches feed the AS5047U angle to the host while the
  TMC9660 uses its own ABN input; decide which side owns the feedback before you close a loop on it.
