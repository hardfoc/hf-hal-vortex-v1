# GpioManager

Registers every GPIO-category pin of the pin map, whichever chip it is on, and lets you drive or read it by name or
by enum. On Vortex V1 those pins live on three chips: the ESP32-C6, the PCAL95555 I/O expander and the TMC9660.
Source: [`GpioManager.h`](../../lib/managers/GpioManager.h).

## Which pins it owns

At start-up it walks the pin map and registers the pins whose category is GPIO (and the user, ADC and SPI
categories). Core pins (crystal, boot, USB) and comm pins (bus lines, the LED data line) are skipped; their owners
configure them. The names are the **Name** column of the [pin map](../hardware/pin-map.md), for example
`GPIO_PCAL_TMC_DRV_EN`, `GPIO_PCAL_IMU_RST`, `GPIO_EXT_GPIO_CS_1`.

| Chip | Pins | How they are reached |
| --- | --- | --- |
| ESP32-C6 | The two external chip selects | An `EspGpio` the manager creates |
| PCAL95555 | TMC9660 control and fault lines, IMU boot / interrupt / reset, CAN standby, power good, TMC9660 GPIO17/18 read-back | Pin objects from the `Pcal95555Handler`, over I2C |
| TMC9660 | `TMC_GPIO17`, `TMC_GPIO18` | The TMC9660 handler, registered once `MotorController` has created the onboard device |

Each pin takes its polarity from the pin map. "Active" means the asserted level, so for an active-low pin
`SetActive()` drives it low.

## API

| Call | Notes |
| --- | --- |
| `Get(std::string_view name)`, `Get(HfFunctionalGpioPin pin)` | `std::shared_ptr<BaseGpio>`, empty if the pin is not registered |
| `Set(name, bool active)`, `SetActive(name)`, `SetInactive(name)`, `Toggle(name)` | Return `hf_gpio_err_t` |
| `Read(name, bool& active)`, `IsActive(name, bool& active)` | Logical state: `true` means asserted (polarity applied) |
| `SetDirection`, `SetPullMode`, `SetOutputMode` | Reconfigure a pin at run time |
| `ConfigureInterrupt(...)`, `EnableInterrupt`, `DisableInterrupt`, `SupportsInterrupts` | Edge interrupts on pins that support them |
| `Contains(name)`, `Size()` | Lookup helpers |
| `GetLastError()`, `GetSystemDiagnostics(GpioSystemDiagnostics&)`, `DumpStatistics()` | Operation counters and the last error |

## Example: read the TMC9660 fault line

```cpp
#include "api/Vortex.h"
#include "managers/GpioManager.h"

auto& gpio = Vortex::GetInstance().gpio;
bool fault = false;
if (gpio.Read("GPIO_PCAL_TMC_FAULT_STATUS", fault) == hf_gpio_err_t::GPIO_SUCCESS && fault) {
    // the TMC9660 is signalling a fault (the line is active low; Read() reports "asserted")
}
```

In a loop, look the pin up once and keep it: `auto drv = gpio.Get(HfFunctionalGpioPin::PCAL_TMC_DRV_EN);`. The
string calls search the table on every call.

## Diagnostics

`GetSystemDiagnostics()` reports the number of registered pins and the total, successful and failed operation
counts. `DumpStatistics()` logs the same, with per-pin access and error counts.

## Pitfalls

- **`Read()` returns the logical state**, not the electrical level: the header comment says "true = high", but the
  code returns `IsActive()`. For an active-low pin, `true` means the line is low.
- **Expander pins are slow.** Every PCAL95555 access is an I2C transfer; do not toggle them in a fast loop.
- **`MotorController` owns the TMC9660 control lines.** Driving `PCAL_TMC_DRV_EN`, `PCAL_TMC_RST_CTRL` or
  `PCAL_TMC_SPI_COMM_EN` yourself disables or confuses the motor controller.
- **No lock on pin operations.** Give each pin one owner task. See [Threading](../architecture/threading.md).
- **A wrong name** returns `GPIO_ERR_PIN_NOT_FOUND`. Copy names from the pin map.
