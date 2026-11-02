# LedManager

Drives the single WS2812 RGB status LED (data on GPIO3, through the ESP32-C6's RMT peripheral): solid colours,
brightness, simple animations and a set of system status patterns. It starts early (step 4) so an LED-only bench
works without the motor controller. Source: [`LedManager.h`](../../lib/managers/LedManager.h).

## API

| Call | Notes |
| --- | --- |
| `SetColor(LedColor)`, `SetColor(uint32_t rgb)`, `TurnOff()`, `GetCurrentColor(LedColor&)` | `LedColors::RED`, `GREEN`, `BLUE`, `YELLOW`, `CYAN`, `MAGENTA`, `WHITE`, `ORANGE`, `PURPLE`, `BLACK` |
| `SetBrightnessPercent(0..100)`, `SetBrightnessRaw(0..255)`, `SetMaxBrightness(...)` | Brightness, capped by the maximum |
| `StartAnimation(LedAnimation, colour)`, `StopAnimation()`, `UpdateAnimation()`, `IsAnimationActive()` | `SOLID`, `BLINK`, `BREATH`, `RAINBOW` and the status patterns |
| `IndicateBoot()`, `IndicateReady()`, `IndicateWarning()`, `IndicateError()`, `IndicateCalibration()` | Blue blink, solid green, yellow blink, red fast blink, cyan breathe |
| `GetLastError()`, `GetSystemDiagnostics(LedSystemDiagnostics&)`, `DumpStatistics()` | Counters and the last error |

## Example: show the start-up result

```cpp
#include "api/Vortex.h"
#include "managers/LedManager.h"

auto& vortex = Vortex::GetInstance();
const bool ok = vortex.EnsureInitialized();
if (vortex.leds.IsInitialized()) {
    (void)(ok ? vortex.leds.IndicateReady() : vortex.leds.IndicateError());
}
// in a periodic task, about every 50 ms:
//   (void)vortex.leds.UpdateAnimation();
```

## Pitfalls

- **Animations need a caller.** The manager has no task; blink, breath, rainbow and the blinking status patterns
  move only when you call `UpdateAnimation()` (the step is 50 ms).
- **It is a required start-up step.** If the RMT channel cannot be set up, `Vortex::EnsureInitialized()` fails.
- **Full white at full brightness is bright and draws current.** Use `SetMaxBrightness()` to cap it.
