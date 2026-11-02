# Managers

A manager is the one object that owns a board function: it knows which chips and pins implement it (from the pin
map), creates and owns the hf-core handlers for them, and gives you a small API. Every manager is a singleton
reached through the `Vortex` facade.

| Manager | Member | What it gives you | Start-up step |
| --- | --- | --- | --- |
| [CommChannelsManager](comms.md) | `vortex.comms` | SPI2 devices, the I2C bus, the UART to the TMC9660, the TWAI (CAN) bus | required |
| [NvsManager](nvs.md) | `vortex.nvs` | Three flash key-value namespaces: application, calibration, configuration | required |
| [GpioManager](gpio.md) | `vortex.gpio` | Every GPIO-category pin by name or enum, on the ESP32-C6, the PCAL95555 and the TMC9660 | required |
| [MotorController](motors.md) | `vortex.motors` | The onboard TMC9660 (and up to two external ones), motor and load-frame velocity and position | best effort |
| [AdcManager](adc.md) | `vortex.adc` | The 15 TMC9660-hosted ADC channels: currents, supply voltages, temperatures, AIN inputs | best effort |
| [ImuManager](imu.md) | `vortex.imu` | The onboard BNO08x IMU and up to three external ones, data-ready interrupts | best effort |
| [EncoderManager](encoders.md) | `vortex.encoders` | Up to four AS5047U angle encoders, created by you | best effort |
| [LedManager](leds.md) | `vortex.leds` | The WS2812 status LED: colours, brightness, animations, status patterns | required |
| [TemperatureManager](temperature.md) | `vortex.temp` | ESP32-C6 die, NTC thermistor and TMC9660 chip temperatures | best effort |

Every manager has the same lifecycle calls: `GetInstance()`, `EnsureInitialized()` (idempotent), `IsInitialized()`,
`Deinitialize()` (or `Shutdown()` for the LED manager), `GetLastError()`, `GetSystemDiagnostics(...)` and
`DumpStatistics()`. The start-up step column says whether a failure stops `Vortex::EnsureInitialized()`; see
[Initialisation](../architecture/initialization.md).
