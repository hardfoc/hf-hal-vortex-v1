# TMC9660 driver reference (Vortex HAL bring-up)

Source tree under `hf-hal-vortex-v1/lib/core/hf-core-drivers/external/hf-tmc9660-driver/`:

- **`inc/tmc9660.hpp`** — main `tmc9660::TMC9660<Comm>` API: `motorConfig`, `gateDriver`, `currentSensing`, `feedbackSense`, `torqueFluxControl`, `velocityControl`, `protection`, `ramp`, `brake`, `stopEvents`, `heartbeat`, `power`, `telemetry`, bootloader helpers.
- **`inc/tmc9660_comm_interface.hpp`** — CRTP SPI/UART comm interfaces (matches `Tmc9660Handler` adapters).
- **`inc/register_mode/`**, **`inc/parameter_mode/`** — register / TMCL parameter access.
- **`inc/bootloader/`** — bootloader protocol and `BootloaderConfig` (Hall, ABN1, SPI flash, …).
- **`src/bootloader/tmc9660_bootloader.cpp`** — implementation.

Proven ESP32-C6 examples inside the driver submodule (`examples/esp32/main/`):

- **`bldc_comprehensive_test.cpp`** — `configureCompleteBLDCMotor()` (13-step BLDC parameter path), open-loop / Hall / ABN scenarios. Use as a **sequence template**; **do not** copy GPIO constants — use `vortex_board_pins.hpp` and the Vortex functional pin map.
- **`telemetry_comprehensive_test.cpp`** — supply / temperature reads without spinning the motor.
- **`esp32_tmc9660_bus.hpp`** — SPI mode 3, UART framing patterns.

Vortex HAL firmware examples (this repo, `examples/esp32/main/`):

- **`common/vortex_bench_safety.hpp`** — bench constants (supply, shunt, FET, PWM, current caps, motion timings).
- **`common/vortex_motor_bench_common.hpp`** — `configure_complete_bldc()` aligned with `configureCompleteBLDCMotor()`.
- Narrative guide: **`examples/esp32/docs/BLDC_BRINGUP.md`**.

HAL path: `MotorController::visitDriver()` → `Tmc9660Handler::visitDriver()` → typed `TMC9660<HalUartTmc9660Comm>` when Vortex default transport is UART (`SetOnboardTmc9660Transport(Spi)` for SPI). Optional **`CreateOnboardDevice(..., bootCfg)`** to replace stock `kDefaultBootConfig` when Hall/ABN mux must be enabled in the bootloader image.
