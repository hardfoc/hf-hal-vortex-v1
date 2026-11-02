# ADR 0003: The hf-pincfg X-macro table is the single source of pin truth

| | |
| --- | --- |
| Status | Accepted (records the current design) |
| Date | 2026-11-01 |

## Context

A pin number written in more than one place drifts: the schematic changes, one copy is updated, another is not,
and the board misbehaves in a way no compiler catches. Vortex V1 spreads its pins over three chips (ESP32-C6,
PCAL95555, TMC9660), which makes copies even easier to get wrong. Pin numbers used to be repeated in this
repository's README and docs.

## Options

1. **Pins as constants in each manager.** Simple; every manager and every document repeats them.
2. **One table per board, used by the code and by the docs.** The board's functional pin map in `hf-pincfg`
   (`hf_functional_pin_config_vortex_v1.hpp`), written as X-macro rows; everything else reads it.

## Decision

Option 2:

- Each row of `HF_FUNCTIONAL_GPIO_PIN_LIST` / `HF_FUNCTIONAL_ADC_CHANNEL_LIST` names a function, its chip, unit,
  pin, polarity, pull, drive and current. The same rows generate the enums, name tables and mapping arrays.
- The managers register from the generated arrays: `GpioManager` creates a pin for each GPIO-category row on
  whichever chip it is, `AdcManager` a channel for each ADC row; `CommChannelsManager` and the facade look pins up by
  `HfFunctionalGpioPin` (`GetGpioMapping()`, `gpio.Get(...)`). No manager writes a GPIO number.
- The [pin map](../hardware/pin-map.md) page and the [board figure](../hardware/index.md) are generated from the same
  rows (`tools/docs/gen.py`, `tools/figkit/build.py`), and CI fails if they are stale.

## Consequences

- Changing a pin is one edit in `hf-pincfg`, a submodule bump here, and a docs rebuild.
- The pin map is shared by every HardFOC HAL through `hf-pincfg`; the board is chosen at build time with
  `HARDFOC_BOARD_VORTEX_V1` (set in this repository's `CMakeLists.txt`).
- One copy remains: [`vortex_board_pins.hpp`](../../examples/esp32/main/common/vortex_board_pins.hpp) repeats the
  bus and LED GPIO numbers for examples that call ESP-IDF directly. It says it must match the pin map but nothing
  checks it; a compile-time check against the pin map would close the gap.
