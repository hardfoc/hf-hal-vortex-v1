# ADR 0001: Keep the MCU behind the interface wrap; only managers know the board

| | |
| --- | --- |
| Status | Accepted; enforced by the board binding and the boundary lint since the bsp_bind change |
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
  `EspTemperature`, and the WS2812 status LED through hf-core's `Ws2812Handler`) are named only in the board
  binding, [`lib/managers/bsp_bind/esp32c6/`](../../lib/managers/bsp_bind/esp32c6/VortexBoardBindEsp32c6.cpp).
  The managers call the MCU-free functions of
  [`VortexBoardBind.h`](../../lib/managers/bsp_bind/VortexBoardBind.h) and see only `Base*` objects. The generated
  [manager map](../architecture/ownership.md#managers-handlers-and-drivers) lists every manager's interface classes,
  read from the `#include` lines, and the [binding table](../architecture/index.md#the-board-binding) which binding
  functions each manager calls.
- **The rule is checked**: `tools/lint/lint_hal_boundary.sh` (CI job "HAL boundary lint") fails on an ESP-IDF,
  FreeRTOS, `mcu/` or RMT include in `lib/api` or `lib/managers` outside `bsp_bind/<platform>/`. Example apps that
  still include HAL internals are listed in `tools/lint/hal_boundary_allowlist.txt`, which may only shrink.
- **Board wiring** lives in the per-board pin map in `hf-pincfg` ([ADR 0003](0003-pin-map-ownership.md)).
- **Application code** sees managers, handlers and `Base*` pointers (for example `BaseCan*` from
  `GetCanOpenBus()`), never `Esp*` types.

## Consequences

- A port to another MCU adds one directory, `bsp_bind/<family>/`, implementing `VortexBoardBind.h` with that MCU's
  interface classes (the `Stm*` classes in the interface wrap are the start of one), plus a pin map; managers,
  handlers, drivers and code above them stay. The host build (`tests/host`) is a second implementation of the same
  header, so the seam is exercised on every pull request.
- **CAN FD** fits the same seam: an FD-capable `BaseCan` implementation (frames up to 64 bytes, data-phase bit
  timing) is created by the platform's `CanOpen()` instead of `EspCan`. Code that only sees `BaseCan*` keeps working;
  code that wants FD frames checks the port's capabilities. `BaseCan`'s message type still carries 8 data bytes, so
  enlarging it is the first step of an FD port.
- The leaks this ADR used to list are closed: `LedManager` no longer uses the RMT driver or `gpio_num_t`, and
  `CommChannelsManager` passes MCU-neutral specs (pins, mode, bit rate) to the binding instead of ESP32 config
  structs. `examples/esp32` still call ESP-IDF directly, which is fine for examples.
