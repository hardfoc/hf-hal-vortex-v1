---
hide:
  - navigation
  - toc
---

<div class="hf-hero" markdown>

<span class="hf-kicker">HardFOC</span>

# Vortex V1 HAL

The software layer for the HardFOC Vortex V1 motor controller board. One `Vortex` object gives your ESP32-C6
application the TMC9660 motor controller, encoders, the IMU, temperatures, ADC channels, GPIO, CAN, storage and a
status LED, through a small set of managers that know the board's wiring.

[Get started](getting-started.md){ .md-button .md-button--primary }
[See the architecture](architecture/index.md){ .md-button }

</div>

## What the board is

Vortex V1 drives one motor. An ESP32-C6 runs your application and talks to a **TMC9660**, a motor controller with
integrated gate drivers and its own current, velocity and position loops (BLDC, stepper or brushed DC), over UART
(the default) or SPI. Around it sit an **AS5047U** magnetic angle encoder slot on SPI, a **BNO08x** IMU and a
**PCAL95555** I/O expander on I2C, an NTC thermistor read through the TMC9660, a CAN port and a **WS2812** status
LED. The [hardware page](hardware/index.md) shows every bus and pin.

This repository (`hf-hal-vortex-v1`) is the hardware abstraction layer (HAL): it turns that board into C++
objects. Your application does not touch GPIO numbers or SPI frames; it calls `vortex.motors`, `vortex.encoders`,
`vortex.temp`.

<!-- figure: hal-layers -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="diagrams/hal-layers.dark.svg">
  <img src="diagrams/hal-layers.svg"
    alt="Layer stack of the Vortex V1 HAL: application, the Vortex facade, 9 managers, 5 hf-core handlers, 6
    chip drivers, and the interface wrap on ESP-IDF and the ESP32-C6.">
</picture>
<!-- /figure -->

## Where to go next

<div class="grid cards" markdown>

- :material-rocket-launch-outline: **[Get started](getting-started.md)**

    Install ESP-IDF, clone with submodules, build, flash and watch the first example.

- :material-school-outline: **[Learn](learn/index.md)**

    A guided path through these pages, in the order that makes sense.

- :material-chip: **[Hardware](hardware/index.md)**

    Buses, chips and the pin map, generated from the board's pin header.

- :material-layers-outline: **[Architecture](architecture/index.md)**

    Layers, start-up order, ownership, threading and errors.

- :material-engine-outline: **[Managers](managers/index.md)**

    One page per manager: what it does, its API, an example and its pitfalls.

- :material-play-box-outline: **[Examples](examples/index.md)**

    Every buildable app, generated from `app_config.yml`.

</div>
