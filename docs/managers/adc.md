# AdcManager

Gives you every ADC channel of the pin map by name or by enum. On Vortex V1 all of them are hosted by the TMC9660:
its analog inputs AIN0 to AIN3 and its internal measurements (phase currents, supply and driver voltages, chip and
external temperatures, motor current, velocity and position). The ESP32-C6's own ADC is not used. Source:
[`AdcManager.h`](../../lib/managers/AdcManager.h).

## Channels

The channels and their names are in the [pin map](../hardware/pin-map.md#adc-channels). Every one is read through a
single `Tmc9660AdcWrapper` that borrows the onboard TMC9660 handler from `MotorController`, so this manager is up
only if the motor controller is.

## API

| Call | Notes |
| --- | --- |
| `ReadVoltage(name, float& volts, samples = 1, time_between_samples = 0)` | Also takes an `HfFunctionalAdcChannel`; averages `samples` readings |
| `ReadRaw(name, uint32_t& raw, samples = 1, time_between_samples = 0)` | Raw counts; also takes the enum |
| `Get(name)`, `Get(HfFunctionalAdcChannel)` | `BaseAdc*` for the channel, or `nullptr` |
| `Contains(name)`, `Size()` | Lookup helpers |
| `GetLastError()`, `GetSystemDiagnostics(AdcSystemDiagnostics&)`, `DumpStatistics()` | Counters and the last error |

## Example: read the supply voltage

```cpp
#include "api/Vortex.h"
#include "managers/AdcManager.h"

auto& adc = Vortex::GetInstance().adc;
float vsupply = 0.0f;
if (adc.ReadVoltage(HfFunctionalAdcChannel::TMC9660_SUPPLY_VOLTAGE, vsupply, 4) == hf_adc_err_t::ADC_SUCCESS) {
    // vsupply holds the average of four readings
}
```

## Pitfalls

- **Depends on the TMC9660.** If `MotorController` did not start, every read fails; check `adc.IsInitialized()`.
- **Internal channels are TMC9660 parameters**, read over the motor controller's host link (UART or SPI), so a read
  costs a TMCL transaction, not a microsecond ADC conversion.
- **Use names from the pin map** (or the enum). A name that is not in the map returns an error, not a reading.
