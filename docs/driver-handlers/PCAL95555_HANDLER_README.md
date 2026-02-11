# Pcal95555Handler - GPIO Expander Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-Pcal95555Handler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-PCA9555%20|%20PCAL9555A-orange.svg)
![Interface](https://img.shields.io/badge/interface-I2C-green.svg)

**Non-templated HAL handler for the PCA9555 / PCAL9555A 16-bit I2C GPIO expander**

</div>

## Overview

The `Pcal95555Handler` wraps the templated `pcal95555::PCAL95555<I2cType>` driver behind a non-templated facade, bridging it into the HardFOC HAL layer. It provides per-pin and batch GPIO operations, a `BaseGpio`-compatible pin factory, hardware interrupt support with edge-filtered callbacks, and PCAL9555A "Agile I/O" features (pull resistors, drive strength, input latch, output mode).

### Key Features

- **16-Bit GPIO Expansion**: Per-pin direction, read, write, toggle, pull mode
- **Batch Operations**: 16-bit mask operations for direction, output, and pull modes via driver API
- **BaseGpio Pin Factory**: `CreateGpioPin()` returns `shared_ptr<BaseGpio>` instances for manager integration
- **Chip Variant Awareness**: Auto-detects PCA9555 vs PCAL9555A; Agile I/O methods gracefully fail on PCA9555
- **Interrupt Management**: Hardware INT pin support with per-pin rising/falling edge callbacks
- **Lazy Initialization**: Driver and adapter created on first use
- **Thread-Safe**: Single `handler_mutex_` protects all state (I2C is serialized anyway)

## Architecture

```
GpioManager
    |
    v
Pcal95555Handler (non-templated facade)
  |-- Pin Registry: up to 16 Pcal95555GpioPin (BaseGpio) instances
  |-- Edge-filtered interrupt dispatch (prev_input_state_ tracking)
  |-- Pull mode cache (seeded from hardware on PCAL9555A init)
    |
    v
pcal95555::PCAL95555<HalI2cPcal95555Comm>  (typed driver)
    |
    v
HalI2cPcal95555Comm  (CRTP I2C adapter)
    |
    v
BaseI2c (platform I2C device)
```

## Construction

```cpp
// Without hardware interrupt support (polling mode)
Pcal95555Handler handler(i2c_device);

// With hardware interrupt support
Pcal95555Handler handler(i2c_device, &interrupt_gpio_pin);
```

**Parameters:**
- `i2c_device` -- `BaseI2c&` with 7-bit I2C address pre-configured (0x20-0x27). Must outlive handler.
- `interrupt_pin` -- Optional `BaseGpio*` connected to the expander's active-low INT output. Pass `nullptr` for polling mode.

## Initialization

```cpp
if (!handler.EnsureInitialized()) {
    // Failed to create driver or detect chip
}

// Check chip features
if (handler.HasAgileIO()) {
    // PCAL9555A detected -- pull resistors, drive strength, etc. available
}
```

`EnsureInitialized()` creates the I2C adapter and driver, auto-detects the chip variant, optionally configures the hardware interrupt pin, seeds `prev_input_state_` for edge detection, and (on PCAL9555A) seeds the internal pull mode cache from hardware registers.

## Per-Pin GPIO Operations

Pin numbers are 0-15 (Port 0 = pins 0-7, Port 1 = pins 8-15).

```cpp
// Direction
handler.SetDirection(0, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);

// Write output
handler.SetOutput(0, true);    // HIGH
handler.SetOutput(0, false);   // LOW
handler.Toggle(0);

// Read input
bool state;
handler.ReadInput(5, state);

// Pull mode (PCAL9555A only)
handler.SetPullMode(5, hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_PULL_UP);

// Get pull mode (from internal cache)
hf_gpio_pull_mode_t pull;
handler.GetPullMode(5, pull);
```

All per-pin methods return `hf_gpio_err_t` (`GPIO_SUCCESS` on success).

## Batch GPIO Operations

Operate on multiple pins simultaneously using 16-bit masks (bit N = pin N):

```cpp
// Set pins 0-3 as outputs
handler.SetDirections(0x000F, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);

// Set pins 0-3 HIGH (uses driver's SetMultipleOutputs for efficiency)
handler.SetOutputs(0x000F, true);

// Set pull-up on pins 8-11 (PCAL9555A only)
handler.SetPullModes(0x0F00, hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_PULL_UP);
```

## BaseGpio Pin Factory

Create `BaseGpio`-compatible pin wrappers for use with `GpioManager`:

```cpp
auto pin0 = handler.CreateGpioPin(
    0,                                                    // pin number
    hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT,        // initial direction
    hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,          // polarity
    hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL, // output mode
    hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING,      // pull mode
    true);                                                 // allow_existing

if (pin0) {
    // Standard BaseGpio interface
    pin0->SetActive(true);
    pin0->Toggle();
}

// Retrieve existing pin
auto existing = handler.GetGpioPin(0);  // returns shared_ptr or nullptr

// Check pin state
bool created = handler.IsPinCreated(0);
uint16_t mask = handler.GetCreatedPinMask();  // bit N set if pin N exists
```

Multiple calls to `CreateGpioPin()` for the same pin number return the same instance (when `allow_existing = true`).

### Pcal95555GpioPin Features

Each `Pcal95555GpioPin` implements `BaseGpio` and delegates to the parent handler:

- **Direction**: INPUT / OUTPUT via expander driver
- **Read/Write**: Level read/write via expander driver
- **Pull Mode**: FLOATING / PULL_UP / PULL_DOWN (PCAL9555A only)
- **Polarity Inversion**: `pin->SetPolarityInversion(true)` (PCAL9555A only)
- **Interrupt Masking**: `pin->SetInterruptMask(false)` to enable interrupts
- **Interrupt Configuration**: `pin->ConfigureInterrupt(trigger, callback)` for edge callbacks
- **Output Mode**: Returns `GPIO_ERR_UNSUPPORTED_OPERATION` (use handler's `SetOutputMode()` for per-port control)

## Interrupt Management

### Hardware Interrupt Setup

Pass a `BaseGpio*` interrupt pin at construction. The handler configures it as a falling-edge interrupt input (the PCAL9555A INT output is active-low, open-drain).

```cpp
BaseGpio* int_pin = /* ... */;
Pcal95555Handler handler(i2c_device, int_pin);
handler.EnsureInitialized();

// Check capabilities
handler.HasInterruptSupport();   // true if int_pin provided
handler.IsInterruptConfigured(); // true after successful init
```

### Per-Pin Callbacks with Edge Detection

Register callbacks on individual pins. The handler's `ProcessInterrupts()` reads the current input state, computes rising/falling edges against `prev_input_state_`, and only dispatches if the edge matches the configured trigger:

```cpp
// Via Pcal95555GpioPin (BaseGpio interface)
auto pin5 = handler.CreateGpioPin(5, hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
pin5->ConfigureInterrupt(
    hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE,
    [](BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* data) {
        // Called only on falling edges of pin 5
    });

// Unmask pin 5's interrupt on the PCAL9555A
handler.SetInterruptMask(5, false);  // false = unmask (enable)
```

### Interrupt Status

```cpp
// Read and clear all 16 pin interrupt status bits
uint16_t status;
handler.GetAllInterruptStatus(status);

// Per-pin status
bool pin5_pending;
handler.GetInterruptStatus(5, pin5_pending);

// Read current interrupt mask
uint16_t mask;
handler.GetAllInterruptMasks(mask);  // 0=enabled, 1=masked per PCAL convention
```

## PCAL9555A Agile I/O Features

These methods require PCAL9555A. On PCA9555 they return `false` or `GPIO_ERR_UNSUPPORTED_OPERATION`.

```cpp
// Check chip variant
handler.HasAgileIO();           // true if PCAL9555A
handler.GetChipVariant();       // pcal95555::ChipVariant::PCAL9555A

// Input polarity inversion
handler.SetPolarityInversion(0, true);   // invert pin 0 input

// Per-pin interrupt mask (PCAL register-level)
handler.SetInterruptMask(0, false);      // unmask pin 0

// Drive strength (25% / 50% / 75% / 100%)
handler.SetDriveStrength(0, DriveStrength::Level3);  // 100%

// Input latch
handler.EnableInputLatch(0, true);

// Output mode (per-port, not per-pin)
handler.SetOutputMode(false, true);  // Port 0 push-pull, Port 1 open-drain

// Reset all registers to power-on defaults
handler.ResetToDefault();
```

## Error Management

```cpp
// Get driver error flags (see pcal95555 Error enum for bit meanings)
uint16_t errors = handler.GetErrorFlags();

// Clear specific or all error flags
handler.ClearErrorFlags();           // clear all
handler.ClearErrorFlags(0x0001);     // clear specific bit
```

## Diagnostics

```cpp
handler.DumpDiagnostics();
```

Logs initialization status, I2C address, chip variant, pin registry contents, interrupt configuration, and error flags at INFO level.

## Utility

```cpp
constexpr uint8_t count = Pcal95555Handler::PinCount();  // 16
uint8_t addr = handler.GetI2cAddress();                   // 7-bit I2C address
```

## Deinitialization

```cpp
handler.EnsureDeinitialized();
// Disables hardware interrupt, clears pin registry, releases driver and adapter
```

## Typical Usage Example

```cpp
#include "Pcal95555Handler.h"

void example(BaseI2c& i2c, BaseGpio& int_pin) {
    // Create handler with interrupt support
    Pcal95555Handler handler(i2c, &int_pin);

    if (!handler.EnsureInitialized()) return;

    // Batch configure: pins 0-7 as outputs, 8-15 as inputs with pull-ups
    handler.SetDirections(0x00FF, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    handler.SetDirections(0xFF00, hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    handler.SetPullModes(0xFF00, hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_PULL_UP);

    // Set outputs HIGH
    handler.SetOutputs(0x00FF, true);

    // Create a BaseGpio pin for GpioManager integration
    auto pin8 = handler.CreateGpioPin(8, hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    if (pin8) {
        pin8->ConfigureInterrupt(
            hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_BOTH_EDGES,
            [](BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* data) {
                // Edge-filtered callback
            });
        handler.SetInterruptMask(8, false);  // unmask
    }

    // Read input
    bool state;
    handler.ReadInput(8, state);

    // Diagnostics
    handler.DumpDiagnostics();
}
```

## See Also

- **[GpioManager](../component-handlers/GPIO_MANAGER_README.md)** -- Creates and owns Pcal95555Handler instances
- **[TMC9660 Handler](TMC9660_HANDLER_README.md)** -- Motor controller handler
- **[hf-pcal95555-driver](../../lib/core/hf-core-drivers/external/hf-pcal95555-driver/)** -- Underlying templated driver

---

*Part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*
