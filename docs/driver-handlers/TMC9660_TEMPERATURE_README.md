# TMC9660 Internal Temperature Sensor

## Overview

The TMC9660 motor controller includes an internal chip temperature sensor, exposed through the `Tmc9660Handler::Temperature` inner class which implements the `BaseTemperature` interface. This enables the TMC9660's chip temperature to be monitored through the standard HardFOC temperature management system alongside other sensors.

## Architecture

```
TemperatureManager
    |
    v
Tmc9660TemperatureWrapper (delegation, manager ownership)
    |
    v
Tmc9660Handler::Temperature (BaseTemperature impl)
    |
    v
Tmc9660Handler::visitDriver() -> driver.telemetry.getChipTemperature()
```

The `Temperature` inner class is owned by the `Tmc9660Handler`. For `TemperatureManager` ownership, use `Tmc9660TemperatureWrapper` (defined in `TemperatureManager.h`), which is a thin delegation wrapper that simply forwards all `BaseTemperature` calls to `handler.temperature()`.

## Sensor Specifications

| Property | Value |
|:---------|:------|
| Range | -40 C to +150 C |
| Resolution | 0.1 C |
| Accuracy | +/-2 C (typical) |
| Response time | ~100ms |
| Capabilities | `HF_TEMP_CAP_HIGH_PRECISION`, `HF_TEMP_CAP_FAST_RESPONSE` |

## Usage

### Accessing the Temperature Sensor

```cpp
Tmc9660Handler handler(spi, rst, drv_en, faultn, wake);
handler.Initialize();

auto& temp = handler.temperature();
```

### Reading Temperature

```cpp
// Via BaseTemperature interface
float celsius;
hf_temp_err_t err = temp.ReadTemperatureCelsius(&celsius);
if (err == hf_temp_err_t::TEMP_SUCCESS) {
    printf("Chip temperature: %.1f C\n", celsius);
}
```

### Via Handler Telemetry (Alternative)

The handler also exposes chip temperature directly through the telemetry API:

```cpp
float temp_c = handler.GetChipTemperature();  // returns NaN if not ready
```

And through the ADC wrapper (channel 30):

```cpp
auto& adc = handler.adc();
float temp_c;
adc.ReadChannelV(30, temp_c);  // Channel 30 = chip temperature in Celsius
```

### Sensor Information

```cpp
hf_temp_sensor_info_t info;
if (temp.GetSensorInfo(&info) == hf_temp_err_t::TEMP_SUCCESS) {
    printf("Sensor: %s %s\n", info.manufacturer, info.model);
    printf("Range: %.0f C to %.0f C\n", info.min_temp_celsius, info.max_temp_celsius);
    printf("Resolution: %.2f C\n", info.resolution_celsius);
}
```

### Capabilities

```cpp
hf_u32_t caps = temp.GetCapabilities();
// HF_TEMP_CAP_HIGH_PRECISION | HF_TEMP_CAP_FAST_RESPONSE
```

## Integration with TemperatureManager

```cpp
// TemperatureManager creates a delegation wrapper internally:
auto wrapper = std::make_unique<Tmc9660TemperatureWrapper>(handler);
// The wrapper delegates all BaseTemperature calls to handler.temperature()
```

The `Tmc9660TemperatureWrapper` does not own the handler. The handler must remain alive for the wrapper's lifetime. This is naturally satisfied when `MotorController` owns the handler and `TemperatureManager` creates the wrapper, since both are singletons with matched lifetimes.

## Error Conditions

| Error | Cause |
|:------|:------|
| `TEMP_ERR_NOT_INITIALIZED` | Handler not initialized (`IsDriverReady() == false`) |
| `TEMP_ERR_READ_FAILED` | TMC9660 communication failure or returns -273.0 C |
| `TEMP_ERR_OUT_OF_RANGE` | Reading outside -40 C to +150 C range |

## Thread Safety

- All temperature read operations are protected by an internal `RtosMutex`
- Statistics and diagnostics are updated atomically under the same mutex
- Safe for concurrent access from multiple RTOS tasks

## Troubleshooting

| Symptom | Cause | Fix |
|:--------|:------|:----|
| Returns -273 C | Communication issue or sensor not ready | Check SPI/UART connection, ensure `Initialize()` succeeded |
| Out-of-range errors | Sensor reads outside expected range | Check TMC9660 hardware health |
| `TEMP_ERR_NOT_INITIALIZED` | Handler not initialized | Call `handler.Initialize()` first |
| High consecutive error count | Persistent communication issue | Check wiring, bus speed, interference |

## See Also

- **[TMC9660 Handler](TMC9660_HANDLER_README.md)** -- Parent handler documentation
- **[BaseTemperature Interface](../../development/BASE_TEMPERATURE_INTERFACE.md)** -- Base interface specification
