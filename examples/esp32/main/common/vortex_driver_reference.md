# TMC9660 driver reference (Vortex HAL bring-up)

Source tree under `hf-hal-vortex-v1/lib/core/hf-core-drivers/external/hf-tmc9660-driver/`:

- **`inc/tmc9660.hpp`** — main `tmc9660::TMC9660<Comm>` API (`motorConfig`, `telemetry`, bootloader flow).
- **`inc/tmc9660_comm_interface.hpp`** — CRTP SPI/UART comm interfaces (matches `Tmc9660Handler` adapters).
- **`inc/register_mode/`**, **`inc/parameter_mode/`** — register / TMCL parameter access.
- **`inc/bootloader/`** — bootloader protocol and config structs.
- **`src/bootloader/tmc9660_bootloader.cpp`** — implementation.

Proven ESP32-C6 examples (`examples/esp32/main/`):

- **`bldc_comprehensive_test.cpp`** — open-loop, FOC HALL, FOC ABN, velocity/current limits (use as scenario template; **do not** copy GPIO constants — use `vortex_board_pins.hpp` + HAL).
- **`telemetry_comprehensive_test.cpp`** — supply / temperature reads without spinning the motor.
- **`esp32_tmc9660_bus.hpp`** — SPI mode 3, UART framing patterns.

HAL path: `MotorController::visitDriver()` → `Tmc9660Handler::visitDriver()` → typed `TMC9660<HalSpiTmc9660Comm>`.
