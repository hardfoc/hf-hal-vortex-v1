# TemperatureManager

Reads the board's three temperature sources by name and gives simple statistics across them. It is up if at least
one sensor works. Source: [`TemperatureManager.h`](../../lib/managers/TemperatureManager.h).

## Sensors

| Name | Source | Needs |
| --- | --- | --- |
| `ESP32_INTERNAL` | The ESP32-C6's on-chip sensor (die temperature) | nothing |
| `NTC_THERMISTOR` | NTC thermistor on TMC9660 input AIN3, through `AdcManager` (`ADC_TMC9660_AIN3`) | `AdcManager`, so the TMC9660 |
| `MOTOR_TEMP` | The TMC9660's internal chip temperature | `MotorController` |

## API

| Call | Notes |
| --- | --- |
| `ReadTemperatureCelsius(name, float&)`, `ReadTemperatureFahrenheit(name, float&)` | By sensor name |
| `ReadInternalTemperature(float&)`, `ReadNtcTemperature(float&)`, `ReadMotorTemperature(float&)` | Shortcuts for the three names |
| `GetSystemTemperatureStats(min, max, avg)` | Across the registered sensors |
| `GetSensorCount()`, `GetSensorEntry(name)` | Registered sensors |
| `GetLastError()`, `GetSystemDiagnostics(TempSystemDiagnostics&)`, `DumpStatistics()` | Counters and the last error |

All reads return `hf_temp_err_t`; `TEMP_SUCCESS` is 0.

## Example: board and chip temperature

```cpp
#include "api/Vortex.h"
#include "managers/TemperatureManager.h"

auto& temp = Vortex::GetInstance().temp;
float board_c = 0.0f, chip_c = 0.0f;
const bool have_board = temp.ReadNtcTemperature(board_c) == hf_temp_err_t::TEMP_SUCCESS;
const bool have_chip = temp.ReadMotorTemperature(chip_c) == hf_temp_err_t::TEMP_SUCCESS;
```

## Pitfalls

- **Two of three sensors depend on the TMC9660.** Without the motor controller only `ESP32_INTERNAL` reads.
- **The die temperature is not the board temperature.** It runs above ambient with the CPU load.
- **`MOTOR_TEMP` is the controller chip**, not the motor winding.
