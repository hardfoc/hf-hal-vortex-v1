# ADR 0001: Keep the MCU behind the interface wrap; only managers know the board

| | |
| --- | --- |
| Status | Accepted (records the current design) |
| Date | 2026-11-01 |

## Context

HardFOC boards share chips (the TMC9660, AS5047U, BNO08x, WS2812, CAN) and a common core (`hf-core`: handlers,
drivers, interface wrap). Vortex V1 runs on an ESP32-C6 today. A future Vortex is expected on an STM32 with CAN FD,
and the CANopen device work planned for the board is meant to move with it unchanged. The motor, sensor and
protocol code must not be tied to ESP-IDF.

## Options

1. **Write the HAL directly against ESP-IDF.** Fast to start; every manager and handler would need rewriting for
   another MCU.
2. **Put the MCU behind interface classes and keep board knowledge in one layer.** Chip drivers and handlers use
   only `BaseSpi`, `BaseI2c`, `BaseGpio`, `BaseAdc`, `BaseCan`, `BaseUart`, `BaseNvs`, `BaseTemperature`; the
   managers are the only code that knows which chip is on which pin of which board.

## Decision

Option 2. The boundary is:

- **Drivers and handlers** (in `hf-core`) depend only on the `Base*` interface classes.
- **The ESP32 implementations** (`EspSpiBus`, `EspI2cBus`, `EspUart`, `EspCan`, `EspGpio`, `EspNvs`,
  `EspTemperature`) are named only in the managers that create buses and peripherals: `CommChannelsManager`,
  `GpioManager`, `NvsManager` and `TemperatureManager`. The generated
  [manager map](../architecture/ownership.md#managers-handlers-and-drivers) lists every manager's interface classes,
  read from the `#include` lines.
- **Board wiring** lives in the per-board pin map in `hf-pincfg` ([ADR 0003](0003-pin-map-ownership.md)).
- **Application code** sees managers, handlers and `Base*` pointers (for example `BaseCan*` from
  `GetCanOpenBus()`), never `Esp*` types.

## Consequences

- A port to another MCU replaces the `Esp*` construction in those four managers with another implementation of the
  same interfaces (the `Stm*` classes in the interface wrap are the start of one) and adds a pin map; handlers,
  drivers and code above the managers stay.
- **CAN FD** fits the same seam: an FD-capable `BaseCan` implementation (frames up to 64 bytes, data-phase bit
  timing) is created by `CommChannelsManager` instead of `EspCan`. Code that only sees `BaseCan*` keeps working;
  code that wants FD frames checks the port's capabilities. `BaseCan`'s message type still carries 8 data bytes, so
  enlarging it is the first step of an FD port.
- Known leaks across the boundary today, to fix before a port: `LedManager` uses the ESP-IDF RMT WS2812 driver and
  returns a `gpio_num_t`; `CommChannelsManager.cpp` writes ESP32 port numbers (SPI host 1, `I2C_NUM_0`, UART port
  1); `examples/esp32` call ESP-IDF directly, which is fine for examples.
