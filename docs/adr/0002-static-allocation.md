# ADR 0002: Allocate once at start-up; fixed tables, no growing containers

| | |
| --- | --- |
| Status | Accepted (records the current design, with the exceptions listed) |
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
- Bus objects, handlers and drivers are held in `std::unique_ptr` (GPIO pins in `std::shared_ptr`, because their
  owners differ), created when the owning manager initialises or when you create a device.
- No manager allocates in its read, write, motion or LED calls.

## Consequences

- After `Vortex::EnsureInitialized()`, and once your own devices are created, the motor, ADC, temperature, GPIO,
  encoder, IMU and LED paths do not touch the heap.

Where this is not strictly true today:

- Run-time device creation allocates: `CreateI2cDevice()`, `MotorController::CreateExternalDevice()`,
  `ImuManager::CreateExternalBno08xDevice()`, `EncoderManager::CreateAs5047uDevice()`. Create devices during your
  own start-up.
- Most managers use `std::make_unique` / `std::make_shared`; with exceptions off, an allocation failure aborts
  instead of failing initialisation. Only `TemperatureManager` uses `hf::utils::make_unique_nothrow`.
- `ImuManager::ConfigureInterrupt()` stores a `std::function`, which may allocate for a capturing callback.
- Allocations inside ESP-IDF and the chip drivers are not covered by this decision.
