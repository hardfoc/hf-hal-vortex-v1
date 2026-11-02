# ADR 0002: Allocate once at start-up; fixed tables, no growing containers

| | |
| --- | --- |
| Status | Accepted; tightened by the static-allocation change (see "Decision" and "Consequences") |
| Date | 2026-11-01 |

## Context

The board drives a motor for long periods. Heap fragmentation or an allocation failure in the middle of a control
loop is hard to diagnose and can leave the drive in an unknown state. On the other hand, which chips answer is only
known at start-up, external devices are added by the application, and some objects (bus drivers, chip drivers) are
easiest to create on the heap. The examples build with C++ exceptions off.

## Options

1. **Everything static.** No heap at all; every driver object a member. Rigid, and some driver classes are not
   designed for it.
2. **Allocate during start-up and device creation, never in a read, write or control call.** Tables with a known
   size are fixed arrays; objects whose existence depends on start-up results are created on the heap and kept for
   the program's life.
3. **Allocate freely.** Simple, and the problems above.

## Decision

Option 2:

- Tables are `std::array` sized at compile time: the GPIO and ADC entries (from the pin map), the SPI device table
  (`SpiDeviceId::DEVICE_COUNT`), the handler slots (`MAX_TMC9660_DEVICES`, `MAX_IMU_DEVICES`,
  `MAX_ENCODER_DEVICES`, all 4), the bounded runtime I2C slots, and the facade's diagnostics arrays
  (`kVortexManagerCount`).
- Handlers live in place in fixed slots (`std::array<std::optional<Handler>, N>`, emplaced when a device is created,
  reset when it is deleted or the manager shuts down). Bus objects and MCU pins live in the board binding's static
  storage. GPIO entries are plain `BaseGpio*` into those objects.
- `lib/api` and `lib/managers` contain no `new`, `make_unique`, `make_shared` or `std::function`; callbacks are a
  function pointer plus a `void*`.
- No manager allocates in its read, write, motion, LED or diagnostics calls; a host test counts `operator new`
  across steady-state calls and requires zero.
- Initialisation fails closed: a manager whose bring-up fails releases what it opened, and the facade shuts down
  every manager again if a required one fails.

## Consequences

- After `Vortex::EnsureInitialized()`, and once your own devices are created, the motor, ADC, temperature, GPIO,
  encoder, IMU and LED paths do not touch the heap.

Where this is not strictly true today:

- Run-time device creation (`MotorController::CreateExternalDevice()`, `ImuManager::CreateExternalBno08xDevice()`,
  `EncoderManager::CreateAs5047uDevice()`) emplaces into a fixed slot; the manager itself no longer allocates. The
  handler it builds may (next point), so create devices during your own start-up.
- Allocations inside hf-core handlers and the ESP-IDF drivers are outside this decision: `Tmc9660Handler`,
  `As5047uHandler`, `Bno08xHandler`, `Pcal95555Handler`, `NtcTemperatureHandler` and `Ws2812Handler` create their
  drivers with `std::unique_ptr` when they initialise. Moving those in place is hf-core work.
