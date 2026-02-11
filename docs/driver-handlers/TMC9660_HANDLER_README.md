# Tmc9660Handler - Motor Controller Driver Handler

<div align="center">

![Driver](https://img.shields.io/badge/driver-Tmc9660Handler-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-TMC9660-orange.svg)
![Interface](https://img.shields.io/badge/interface-SPI%20|%20UART-green.svg)

**Non-templated HAL handler for TMC9660 motor controller with BaseGpio, BaseAdc, and BaseTemperature integration**

</div>

## Overview

The `Tmc9660Handler` wraps the templated `tmc9660::TMC9660<CommType>` driver behind a non-templated facade, bridging it into the HardFOC HAL layer. It supports both SPI and UART communication, provides inner-class adapters for `BaseGpio`, `BaseAdc`, and `BaseTemperature`, and exposes convenience methods for common motor control operations.

### Key Features

- **Dual Communication**: SPI or UART, selected at construction time
- **Motor Control**: Convenience methods for motor type, commutation, feedback sensors, PID gains, DRV_EN control, and current sensing calibration
- **Typed Driver Access**: `spiDriver()` / `uartDriver()` for direct access to all 18+ driver subsystems
- **Visitor Pattern**: `visitDriver()` for generic code that works with either comm mode
- **BaseGpio Wrappers**: GPIO17/GPIO18 as `BaseGpio` instances (INPUT and OUTPUT supported)
- **BaseAdc Wrapper**: 15-channel ADC via non-contiguous channel ID scheme
- **BaseTemperature Wrapper**: Chip temperature sensor via `BaseTemperature`
- **Telemetry**: Supply voltage, chip temperature, motor current, velocity, position, error flags
- **Bootloader Integration**: Full bootloader init sequence with hardware reset

## Architecture

```
Application / Managers
        |
        v
 Tmc9660Handler (non-templated facade)
   |-- Gpio (BaseGpio)        -- GPIO17, GPIO18 digital I/O
   |-- Adc (BaseAdc)          -- 15 channels across 5 ranges
   |-- Temperature (BaseTemp) -- chip temperature sensor
   |-- spiDriver() / uartDriver()  -- direct typed access
   |-- visitDriver(lambda)         -- generic typed access
        |
        v
 tmc9660::TMC9660<CommType>  (18 subsystems)
        |
        v
 HalSpiTmc9660Comm / HalUartTmc9660Comm  (CRTP adapters)
        |
        v
 BaseSpi / BaseUart + BaseGpio (control pins)
```

## Construction

The handler requires a communication bus **and** four host-side GPIO control pins:

```cpp
// SPI mode
Tmc9660Handler handler(spi, rst_pin, drv_en_pin, faultn_pin, wake_pin,
                       address, &bootConfig);

// UART mode
Tmc9660Handler handler(uart, rst_pin, drv_en_pin, faultn_pin, wake_pin,
                       address, &bootConfig);
```

**Parameters:**
- `spi` / `uart` -- `BaseSpi&` or `BaseUart&` (must outlive handler)
- `rst` -- `BaseGpio&` for RST pin (output, hardware reset)
- `drv_en` -- `BaseGpio&` for DRV_EN pin (output, power stage enable)
- `faultn` -- `BaseGpio&` for FAULTN pin (input, fault status)
- `wake` -- `BaseGpio&` for WAKE pin (output, hibernate wake)
- `address` -- 7-bit TMCL device address (default: 0)
- `bootCfg` -- Bootloader configuration pointer (default: `kDefaultBootConfig`)

## Initialization

```cpp
if (!handler.Initialize()) {
    // Bootloader init failed
}

if (!handler.IsDriverReady()) {
    // Driver not yet initialized
}
```

`Initialize()` creates the typed driver, runs the bootloader sequence (hardware reset, config write, parameter mode entry), and creates GPIO/ADC/Temperature wrappers.

## Motor Control Convenience Methods

All require `IsDriverReady() == true`.

```cpp
// Motor configuration
handler.SetMotorType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
handler.SetPWMFrequency(25000);
handler.SetCommutationMode(tmc9660::tmcl::CommutationMode::FOC_HALL);

// Feedback sensor setup
handler.ConfigureHallSensor();           // or:
handler.ConfigureABNEncoder(4096);       // 4096 counts/rev

// DRV_EN hardware control (distinct from software enable)
handler.EnableDriverOutput();            // assert DRV_EN pin

// Current sensing calibration (motor must be stationary)
handler.CalibrateCurrentSensing(true, 1000);

// PID loop gains
handler.SetCurrentLoopGains(200, 100);
handler.SetVelocityLoopGains(500, 10);
handler.SetPositionLoopGains(50, 5);

// Software motor enable/disable
handler.EnableMotor();
handler.DisableMotor();

// Motion control
handler.SetTargetVelocity(1000);
handler.SetTargetPosition(50000);
handler.SetTargetTorque(1500);
```

### Typical Startup Sequence

```cpp
handler.Initialize();
handler.EnableDriverOutput();             // 1. Hardware gate on
handler.CalibrateCurrentSensing();        // 2. ADC offset calibration
handler.SetMotorType(MotorType::BLDC_MOTOR, 7);
handler.ConfigureHallSensor();
handler.SetCurrentLoopGains(200, 100);
handler.SetVelocityLoopGains(500, 10);
handler.SetCommutationMode(CommutationMode::FOC_HALL);
handler.EnableMotor();                    // 3. Software enable
handler.SetTargetVelocity(1000);          // 4. Begin control
```

## Telemetry

```cpp
float supply_v   = handler.GetSupplyVoltage();       // volts
float chip_temp   = handler.GetChipTemperature();     // celsius
int16_t current   = handler.GetMotorCurrent();        // milliamps
int32_t velocity  = handler.GetActualVelocity();      // internal units
int32_t position  = handler.GetActualPosition();      // encoder counts
uint16_t ext_temp = handler.GetExternalTemperature(); // raw ADC

uint32_t status_flags, error_flags, gate_errors;
handler.GetStatusFlags(status_flags);
handler.GetErrorFlags(error_flags);
handler.ClearErrorFlags();
handler.GetGateDriverErrorFlags(gate_errors);
handler.ClearGateDriverErrorFlags();
```

## Core Parameter Access

For parameters not covered by convenience methods:

```cpp
handler.WriteParameter(tmc9660::tmcl::Parameters::SOME_PARAM, value);
handler.ReadParameter(tmc9660::tmcl::Parameters::SOME_PARAM, value);
handler.SendCommand(tmc9660::tmcl::Op::SAP, type, motor, value, &reply);
```

## Typed Driver Access

For direct access to all 18+ driver subsystems when the comm mode is known:

```cpp
// SPI mode -- direct typed access (no lambda needed)
auto* drv = handler.spiDriver();
if (drv) {
    drv->feedbackSense.configureHall();
    drv->motorConfig.setType(tmcl::MotorType::BLDC_MOTOR, 7);
    drv->gateDriver.configureBreakBeforeMakeTiming_ns(500, 500);
    drv->protection.setOvercurrentLimit(5000);
    drv->velocityControl.setTargetVelocity(1000);
    float temp = drv->telemetry.getChipTemperature();
}

// UART mode
auto* drv = handler.uartDriver();
if (drv) { /* same API */ }
```

## Visitor Pattern

For generic code that must work with either SPI or UART:

```cpp
handler.visitDriver([](auto& driver) {
    driver.feedbackSense.configureHall();
    driver.protection.setOvertemperatureLimit(120);
});

auto vel = handler.visitDriver([](auto& driver) -> int32_t {
    return driver.telemetry.getActualVelocity();
});
```

## Peripheral Wrappers

### GPIO (BaseGpio)

Wraps TMC9660 internal GPIO pins 17 and 18 as `BaseGpio` instances.

```cpp
auto& gpio17 = handler.gpio(17);
auto& gpio18 = handler.gpio(18);

gpio17.Initialize();
gpio17.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
gpio17.SetActive(true);

gpio18.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
hf_gpio_level_t level;
gpio18.GetPinLevel(level);
```

- **Supported directions**: INPUT and OUTPUT (configures via `driver.gpio.setMode()`)
- **Output mode**: PUSH_PULL only
- **Pull mode**: FLOATING only (pull is configured via bootloader config)
- **Write guard**: `SetPinLevel()` returns `GPIO_ERR_INVALID_CONFIGURATION` if direction is INPUT

### ADC (BaseAdc)

15-channel ADC using a non-contiguous channel ID scheme:

| Channel ID | Type | Description |
|:----------:|:-----|:------------|
| 0-3 | AIN | External analog inputs (GPIO5) |
| 10-13 | Current sense | Phase current ADC (I0-I3) |
| 20-21 | Voltage | 20=supply, 21=driver voltage |
| 30-31 | Temperature | 30=chip (Celsius), 31=external NTC |
| 40-42 | Motor data | 40=current (mA), 41=velocity, 42=position |

```cpp
auto& adc = handler.adc();
float voltage;
adc.ReadChannelV(20, voltage);  // Supply voltage

// Use IsChannelAvailable() to validate sparse channel IDs
if (adc.IsChannelAvailable(30)) {
    adc.ReadChannelV(30, voltage);  // Chip temperature in Celsius
}
```

`GetMaxChannels()` returns 15 (total count). Since channel IDs are sparse, always use `IsChannelAvailable()` for validation.

The `Tmc9660AdcWrapper` class provides a thin delegation wrapper for `AdcManager` ownership without transferring handler ownership.

### Temperature (BaseTemperature)

```cpp
auto& temp = handler.temperature();
float celsius;
temp.ReadCelsius(celsius);
```

## Communication Info

```cpp
tmc9660::CommMode mode = handler.GetCommMode();
const auto& cfg = handler.bootConfig();
```

## Diagnostics

```cpp
handler.DumpDiagnostics();  // Logs to system logger at INFO level
```

## See Also

- **[MotorController](../component-handlers/MOTOR_CONTROLLER_README.md)** -- Singleton that owns `Tmc9660Handler` instances
- **[Tmc9660AdcWrapper](../../lib/handlers/Tmc9660AdcWrapper.h)** -- Delegation wrapper for `AdcManager` ownership
- **[TMC9660 Temperature README](TMC9660_TEMPERATURE_README.md)** -- Temperature wrapper documentation
- **[PCAL95555 Handler](PCAL95555_HANDLER_README.md)** -- GPIO expander handler

---

*Part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*
