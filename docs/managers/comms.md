# CommChannelsManager

Owns every bus on the board and hands out device handles. Other managers get their buses from it; your code
normally needs it only to talk to an external SPI or I2C device, or to the CAN bus. Source:
[`CommChannelsManager.h`](../../lib/managers/CommChannelsManager.h).

## The buses

| Bus | Settings at start-up | Devices |
| --- | --- | --- |
| SPI2 | 10 MHz per device by default | `TMC9660_MOTOR_CONTROLLER` (mode 3), `AS5047U_POSITION_ENCODER` (mode 1), `EXTERNAL_DEVICE_1` and `EXTERNAL_DEVICE_2` (mode 0) |
| I2C | 100 kHz for the built-in devices | BNO08x IMU at 0x4A, PCAL95555 at 0x20, plus a bounded number of devices you add at run time |
| UART | 115200 baud | TMCL link to the TMC9660 |
| TWAI (CAN) | 500 kbit/s | The CAN port; `GetCanOpenBus()` is an alias of `GetCanBus()` for CANopen code |

Pins are in the [pin map](../hardware/pin-map.md) (the `COMM_*` rows).

## API

| Call | Notes |
| --- | --- |
| `GetSpiDevice(SpiDeviceId)`, `GetSpiDevice(uint8_t index)` | `BaseSpi*` for one chip select, or `nullptr` |
| `GetI2cDevice(I2cDeviceId)`, `GetI2cDevice(bus_index, device_index)` | `BaseI2c*` for a built-in or runtime device |
| `CreateI2cDevice(address, speed_hz = 400000)` | Adds a runtime I2C device; returns its index, or -1 on failure |
| `RemoveI2cDevice(index)`, `I2cProbeAddress(address, timeout_ms)`, `I2cScanBus(out, max)`, `I2cResetBus()` | Runtime I2C helpers |
| `GetUartBus()` | `BaseUart*` to the TMC9660 link |
| `GetCanBus()`, `GetCanOpenBus()` | `BaseCan*` |
| `GetLastError()`, `GetSystemDiagnostics(CommSystemDiagnostics&)`, `DumpStatistics()` | Per-bus validity, device counts, last error |

## Example: scan the I2C bus

```cpp
#include "api/Vortex.h"
#include "managers/CommChannelsManager.h"

auto& comms = Vortex::GetInstance().comms;
uint8_t found[16] = {};
const size_t n = comms.I2cScanBus(found, sizeof(found));
for (size_t i = 0; i < n; ++i) {
    ESP_LOGI("app", "I2C device at 0x%02X", found[i]);   // expect 0x20 (PCAL95555) and 0x4A (BNO08x)
}
```

## Diagnostics

`GetSystemDiagnostics()` fills `CommSystemDiagnostics`: whether SPI, I2C, UART and CAN are valid, the number of SPI
devices and runtime I2C devices, and the last error. `DumpStatistics()` logs the same.

## Pitfalls

- **Borrowed pointers.** The returned bus pointers belong to the manager; do not delete them and do not keep them
  across `Deinitialize()`.
- **The accessors and the runtime I2C calls take no lock.** Add devices during start-up, from one task. See
  [Threading](../architecture/threading.md).
- **SPI2 and the TMC9660 mux.** The TMC9660 sees SPI2 only while `PCAL_TMC_SPI_COMM_EN` selects it.
  `MotorController` handles this for its own transfers; do not drive that pin yourself.
- **Runtime I2C devices are heap-allocated**, so create them at start-up, not in a control loop.
