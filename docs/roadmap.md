# Roadmap

Where the Vortex V1 HAL is going, and the known gaps in what is here today. Nothing on this page is a promise of a
date; it is the order of work as currently planned.

## CANopen device

The next large piece of work is making Vortex V1 a **CiA 402 servo drive node** on its CAN port: one object
dictionary as the single source for firmware, EDS file and tools; CANopenNode v4 behind a narrow stack interface;
the TMC9660's own loops doing the motion while the ESP32-C6 runs the CiA 402 state machine, limits and faults; and
host tests that run the same node against a simulated motor.

| What | Where | State |
| --- | --- | --- |
| CANopen node design: CiA 301 / 402 drive, object dictionary as the single source, threads, wireless access, CAN FD route | [canopen-node.md](https://github.com/hardfoc/hardfoc-vortex-v1/blob/main/docs/design/canopen-node.md) | Draft, under review |
| Drive observability: every TMC9660 parameter, telemetry and fault in the object dictionary, the CLI and a dashboard | [drive-observability.md](https://github.com/hardfoc/hardfoc-vortex-v1/blob/main/docs/design/drive-observability.md) | Draft, under review |
| Header-only contracts (CAN frame, CiA 402 rule table, fault policy, router policy, OD store) with host tests | [`contracts/`](https://github.com/hardfoc/hardfoc-vortex-v1/tree/main/contracts) in hardfoc-vortex-v1 | Merged |
| Notes on the current CAN path, `hf-utils-canopen` and the TMC9660 | [canopen.md](https://github.com/hardfoc/hardfoc-vortex-v1/blob/main/docs/canopen.md) | Published |
| CANopenNode sources | [`third_party/CANopenNode`](../third_party/README.md) (submodule), built with the HAL's *Build CANopenNode slave stack* option | Present |

The HAL's part is small by design: the CAN port stays a `BaseCan*` from
[`CommChannelsManager`](managers/comms.md), and the drive talks to the TMC9660 through
[`MotorController`](managers/motors.md).

## Another MCU and CAN FD

A future Vortex is expected on an STM32 with CAN FD. [ADR 0001](adr/0001-mcu-agnostic-boundary.md) records the
boundary that makes this a port of four managers rather than a rewrite, and lists what crosses it today.

## Known gaps

| Gap | Where | Page |
| --- | --- | --- |
| `GetSystemWarnings()` always returns no entries | `Vortex.cpp` | [Errors](architecture/errors.md) |
| `Vortex::EnsureInitialized()` and `Shutdown()` take no lock | `Vortex.cpp` | [Threading](architecture/threading.md) |
| Per-pin GPIO calls and single-encoder reads take no lock | `GpioManager.cpp`, `EncoderManager.cpp` | [Threading](architecture/threading.md) |
| `GpioManager::Read()` documents "true = high" but returns the logical (active) state | `GpioManager.h` | [GpioManager](managers/gpio.md) |
| Most managers allocate with throwing `make_unique`, so a failed allocation aborts | `lib/managers` | [ADR 0002](adr/0002-static-allocation.md) |
| `vortex_board_pins.hpp` repeats GPIO numbers without a check against the pin map | `examples/esp32/main/common` | [ADR 0003](adr/0003-pin-map-ownership.md) |
| ESP32 port numbers and `gpio_num_t` in managers | `CommChannelsManager.cpp`, `LedManager.h` | [ADR 0001](adr/0001-mcu-agnostic-boundary.md) |
| No host unit tests in this repository; the examples prove behaviour on a board | - | [Testing and CI](testing.md) |
