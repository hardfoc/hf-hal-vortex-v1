# Hardware

What is on the Vortex V1 board, as the software sees it: one ESP32-C6, the buses it drives and the chips on each
bus. Every pin number on this page comes from the board's functional pin map; the full tables are on the
[pin map](pin-map.md) page.

<!-- figure: board-overview -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="../diagrams/board-overview.dark.svg">
  <img src="../diagrams/board-overview.svg"
    alt="Block diagram of the Vortex V1 board: the ESP32-C6 connects over SPI2 to the TMC9660 and AS5047U,
    over UART to the TMC9660, over I2C to the PCAL95555 expander and BNO08x IMU, over TWAI to the CAN
    transceiver and to a WS2812 LED; the PCAL95555 drives the TMC9660 and IMU control lines.">
</picture>
<!-- /figure -->

## The chips

| Chip | What it does | Bus | Manager that owns it |
| --- | --- | --- | --- |
| ESP32-C6 | RISC-V MCU with Wi-Fi 6, BLE, 802.15.4 and TWAI (CAN); runs the HAL and your application | - | all of them |
| TMC9660 | Motor controller with integrated gate drivers and its own FOC current, velocity and position loops; also provides GPIO, ADC channels and a chip temperature | UART (default) or SPI2 | [`MotorController`](../managers/motors.md) |
| AS5047U | 14-bit magnetic angle encoder | SPI2 | [`EncoderManager`](../managers/encoders.md) |
| BNO08x | 9-axis IMU with on-chip sensor fusion | I2C | [`ImuManager`](../managers/imu.md) |
| PCAL95555 | 16-bit I2C I/O expander: TMC9660 control lines, IMU boot/reset, CAN standby, power good | I2C | [`GpioManager`](../managers/gpio.md) |
| NTC thermistor | Board temperature, read on TMC9660 input AIN3 | through the TMC9660 | [`TemperatureManager`](../managers/temperature.md) |
| CAN transceiver | Physical layer for the TWAI controller | TWAI | [`CommChannelsManager`](../managers/comms.md) |
| WS2812 | RGB status LED | RMT data line | [`LedManager`](../managers/leds.md) |

## Things to know before you wire anything

- **The TMC9660 has two host links.** The UART is always connected. SPI2 reaches the TMC9660 only while the
  expander pin `PCAL_TMC_SPI_COMM_EN` (active low) selects it, because the same SPI2 lines also serve the AS5047U
  and the external slots. The HAL uses the UART unless you call
  `Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Spi)` before `EnsureInitialized()`.
- **TMC9660 GPIO17 and GPIO18 appear twice in the pin map**: as expander inputs (`PCAL_TMC_GPIO17_EXP_IN`,
  `PCAL_TMC_GPIO18_EXP_IN`) and as TMC9660 GPIOs (`TMC_GPIO17`, `TMC_GPIO18`). Both views see the same nets.
- **There is no encoder fitted by default.** The AS5047U chip select is wired, but `EncoderManager` creates no
  device until you call `CreateAs5047uDevice()`.
- **Do not copy pin numbers from TMC9660 evaluation-kit examples.** Use the [pin map](pin-map.md).

## Bench documents

The bring-up notes written on the bench live next to the examples, not on this site:
[bench matrix](../../examples/esp32/docs/BENCH_MATRIX.md),
[BLDC bring-up](../../examples/esp32/docs/BLDC_BRINGUP.md) and the
[example docs index](../../examples/esp32/docs/README.md).
