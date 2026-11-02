# API reference

The full C++ API of the HAL, from the Doxygen comments in `lib/api` and `lib/managers`.

[Open the API reference](api/index.html){ .md-button .md-button--primary }

## Start here

| Page | What it is |
| --- | --- |
| [`Vortex`](api/class_vortex.html) | The board facade: `GetInstance()`, `EnsureInitialized()`, the transport choice for the onboard TMC9660, the manager members, system diagnostics |
| [`MotorController`](api/class_motor_controller.html) | The TMC9660 devices: motor- and load-frame motion, `visitDriver()` for every TMCL parameter |
| [`EncoderManager`](api/class_encoder_manager.html) | AS5047U angle encoders |
| [`CommChannelsManager`](api/class_comm_channels_manager.html) | SPI2 devices, I2C, the TMC9660 UART, CAN and USB Serial/JTAG |
| [`GpioManager`](api/class_gpio_manager.html) | GPIO pins of the pin map by name, on the ESP32-C6, the PCAL95555 and the TMC9660 |

## Build it locally

Run Doxygen from the repository root and open the `index.html` it writes:

```bash
mkdir -p build/doxygen && doxygen Doxyfile
open build/doxygen/html/index.html    # xdg-open on Linux
```

The published reference uses the HardFOC stylesheet, which the docs workflow adds; a local build uses Doxygen's
default look.
