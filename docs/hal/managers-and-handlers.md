# Vortex — managers and hf-core handlers

**Managers** live in this HAL under `lib/managers/`. **Handlers** (device facades) and **drivers** live in the **`hf-core`** submodule under `lib/core/` (e.g. `lib/core/handlers/tmc9660/Tmc9660Handler.h`).

Use **`Vortex::GetInstance()`** and then **`vortex.<subsystem>`** for application code; the table below is for navigation into sources.

| `Vortex` member | Manager (HAL) | Primary hf-core / driver touchpoints | Notes |
|-----------------|---------------|--------------------------------------|--------|
| `nvs` | [`NvsManager`](../../lib/managers/NvsManager.h) | NVS via board utilities | Initialized first |
| `comms` | [`CommChannelsManager`](../../lib/managers/CommChannelsManager.h) | `BaseSpi` / `BaseI2c` / `BaseUart` / CAN | Foundation for buses |
| `gpio` | [`GpioManager`](../../lib/managers/GpioManager.h) | `BaseGpio` implementations, PCAL95555 handler | Board pin map |
| `motors` | [`MotorController`](../../lib/managers/MotorController.h) | [`Tmc9660Handler`](../../lib/core/handlers/tmc9660/Tmc9660Handler.h), TMC9660 driver | Onboard index `0`; optional `BootloaderConfig*` at `CreateOnboardDevice()` for Hall/ABN mux. BLDC parameter-mode bring-up for examples: [`examples/esp32/docs/BLDC_BRINGUP.md`](../../examples/esp32/docs/BLDC_BRINGUP.md). |
| `adc` | [`AdcManager`](../../lib/managers/AdcManager.h) | TMC9660-hosted ADC wrapper (see manager header) | Vortex: channels tied to TMC9660 / functional ADC enum |
| `imu` | [`ImuManager`](../../lib/managers/ImuManager.h) | BNO08x handler under `lib/core/handlers/` | I²C via comms |
| `encoders` | [`EncoderManager`](../../lib/managers/EncoderManager.h) | AS5047U handler under `lib/core/handlers/` | SPI via comms |
| `leds` | [`LedManager`](../../lib/managers/LedManager.h) | WS2812 / RMT path | Brought up early for smoke tests |
| `temp` | [`TemperatureManager`](../../lib/managers/TemperatureManager.h) | NTC / sensor helpers + ADC | Depends on ADC + motors |

For **generic** definitions of “manager” vs “handler”, see the handbook: [manager pattern](../hf-development-handbook/process/manager-pattern.md), [handler pattern](../hf-development-handbook/process/handler-pattern.md).

## Logger

`Logger` is used across the tree (`handlers/logger/Logger.h` in `hf-core`). `Vortex`’s constructor initializes it before other subsystems log.
