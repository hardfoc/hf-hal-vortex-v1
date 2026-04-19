# TemperatureManager - Temperature Monitoring System

<div align="center">

![Component](https://img.shields.io/badge/component-TemperatureManager-blue.svg)
![Hardware](https://img.shields.io/badge/hardware-NTC%20Thermistors-orange.svg)
![Interface](https://img.shields.io/badge/interface-ADC%20%7C%20I2C-green.svg)

**Unified temperature monitoring system with multi-source sensor support**

</div>

## 📋 Overview

The `TemperatureManager` is a comprehensive temperature monitoring system that provides unified access to temperature sensors across the HardFOC HAL platform. It supports multiple temperature sensor types including NTC thermistors, integrated temperature sensors, and external temperature monitoring devices.

### ✨ Key Features

- **🌡️ Multi-Sensor Support**: NTC thermistors, integrated sensors, external devices
- **📊 Unified API**: Single interface for all temperature measurements
- **🔧 Automatic Calibration**: Built-in calibration and compensation
- **📈 Trend Analysis**: Temperature history and trend monitoring
- **🛡️ Safety Monitoring**: Over-temperature protection and alerts
- **⚡ Real-time Updates**: High-speed temperature monitoring
- **🔍 Health Diagnostics**: Sensor health and accuracy monitoring

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   TemperatureManager                           │
├─────────────────────────────────────────────────────────────────┤
│  NTC Thermistors │ Multiple NTC sensors via ADC               │
├─────────────────────────────────────────────────────────────────┤
│  Integrated Sensors │ On-board temperature monitoring          │
├─────────────────────────────────────────────────────────────────┤
│  External Devices │ I2C temperature sensors                    │
├─────────────────────────────────────────────────────────────────┤
│  Calibration System │ Automatic compensation and scaling       │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Temperature Monitoring

```cpp
#include "component-handlers/TemperatureManager.h"
#include "utils-and-drivers/driver-handlers/Logger.h"

void temperature_basic_example() {
    auto& logger = Logger::GetInstance();
    
    // Get temperature manager instance
    auto& temp_manager = TemperatureManager::GetInstance();
    temp_manager.EnsureInitialized();
    
    // Read temperature from ESP32 internal sensor
    float temperature_celsius;
    if (temp_manager.ReadTemperature("ESP32_INTERNAL", temperature_celsius) == hf_temp_err_t::TEMP_SUCCESS) {
        logger.Info("TEMP", "ESP32 Internal Temperature: %.2f°C", temperature_celsius);
    }
    
    // Read from NTC temperature sensor
    float ntc_temp;
    if (temp_manager.ReadTemperature("NTC_TEMPERATURE", ntc_temp) == hf_temp_err_t::TEMP_SUCCESS) {
        logger.Info("TEMP", "NTC Temperature: %.2f°C", ntc_temp);
    }
    
    // Read from TMC9660 temperature sensor
    float tmc_temp;
    if (temp_manager.ReadTemperature("TMC9660_TEMPERATURE", tmc_temp) == hf_temp_err_t::TEMP_SUCCESS) {
        logger.Info("TEMP", "TMC9660 Temperature: %.2f°C", tmc_temp);
    }
}
```

## 📖 API Reference

### Core Operations

#### Construction and Initialization
```cpp
class TemperatureManager {
public:
    // Singleton access
    static TemperatureManager& GetInstance() noexcept;
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool Initialize() noexcept;
    void Deinitialize() noexcept;
    bool IsInitialized() const noexcept;
};
```

#### Temperature Measurements
```cpp
// Basic temperature reading
TemperatureError ReadTemperature(const std::string& sensor_id, float& temperature_celsius) noexcept;

// Multiple temperature readings
TemperatureError ReadAllTemperatures(std::vector<TemperatureReading>& readings) noexcept;

// Temperature with metadata
TemperatureError ReadTemperatureWithMetadata(const std::string& sensor_id, 
                                            TemperatureReading& reading) noexcept;
```

#### Sensor Management
```cpp
// Sensor registration and discovery
TemperatureError RegisterSensor(const std::string& sensor_id, 
                               const TemperatureSensorConfig& config) noexcept;

TemperatureError UnregisterSensor(const std::string& sensor_id) noexcept;

std::vector<std::string> GetAvailableSensors() const noexcept;

bool IsSensorAvailable(const std::string& sensor_id) const noexcept;
```

#### Calibration and Configuration
```cpp
// Calibration operations
TemperatureError CalibrateSensor(const std::string& sensor_id) noexcept;

TemperatureError SetCalibrationOffset(const std::string& sensor_id, float offset_celsius) noexcept;

TemperatureError GetCalibrationOffset(const std::string& sensor_id, float& offset_celsius) noexcept;

// Configuration management
TemperatureError UpdateSensorConfig(const std::string& sensor_id, 
                                   const TemperatureSensorConfig& config) noexcept;

TemperatureError GetSensorConfig(const std::string& sensor_id, 
                                TemperatureSensorConfig& config) noexcept;
```

#### Monitoring and Alerts
```cpp
// Temperature monitoring
TemperatureError StartMonitoring(const std::string& sensor_id, 
                                float update_rate_hz = 1.0f) noexcept;

TemperatureError StopMonitoring(const std::string& sensor_id) noexcept;

// Alert configuration
TemperatureError SetTemperatureAlert(const std::string& sensor_id, 
                                    float threshold_celsius, 
                                    TemperatureAlertType alert_type) noexcept;

TemperatureError ClearTemperatureAlert(const std::string& sensor_id) noexcept;
```

#### System Status and Diagnostics
```cpp
// System status
TemperatureSystemStatus GetSystemStatus() const noexcept;

bool IsSystemHealthy() const noexcept;

// Sensor diagnostics
TemperatureError GetSensorDiagnostics(const std::string& sensor_id, 
                                     TemperatureSensorDiagnostics& diagnostics) noexcept;

// Performance statistics
TemperatureStatistics GetStatistics() const noexcept;

void ResetStatistics() noexcept;
```

## 🎯 Hardware Support

### Supported Temperature Sensors

#### NTC Thermistors
- **Interface**: ADC channels via AdcManager
- **Temperature Range**: -40°C to +125°C
- **Accuracy**: ±0.5°C typical
- **Response Time**: <1 second
- **Calibration**: Automatic Steinhart-Hart compensation

#### Integrated Temperature Sensors
- **ESP32 Internal**: On-chip temperature monitoring
- **TMC9660 Internal**: Motor controller temperature
- **Range**: -40°C to +125°C
- **Accuracy**: ±2°C typical

#### External I2C Temperature Sensors
- **Interface**: I2C via CommChannelsManager
- **Supported Devices**: Various I2C temperature sensors
- **Configurable**: Address and resolution settings

### Sensor Configuration

```cpp
struct TemperatureSensorConfig {
    std::string sensor_id;
    TemperatureSensorType sensor_type;
    std::string hardware_source;  // "ADC_CH0", "I2C_DEVICE_1", etc.
    
    // NTC specific
    float r25_ohms;              // Resistance at 25°C
    float beta_value;            // Beta coefficient
    float series_resistance;     // Series resistor value
    
    // Calibration
    float calibration_offset;
    std::vector<CalibrationPoint> calibration_points;
    
    // Monitoring
    float min_temperature;
    float max_temperature;
    float alert_threshold_high;
    float alert_threshold_low;
    
    // Update settings
    float update_rate_hz;
    bool enable_monitoring;
};
```

## 📊 Examples

### Basic Temperature Reading

```cpp
void basic_temperature_example() {
    auto& temp_manager = TemperatureManager::GetInstance();
    temp_manager.EnsureInitialized();
    
    // Read from multiple sensors
    std::vector<std::string> sensors = {"NTC_1", "NTC_2", "TMC9660_TEMP"};
    
    for (const auto& sensor_id : sensors) {
        float temperature;
        if (temp_manager.ReadTemperature(sensor_id, temperature) == TemperatureError::SUCCESS) {
            logger.Info("TEMP", "%s: %.2f°C\n", sensor_id.c_str(), temperature);
        }
    }
}
```

### Advanced Temperature Monitoring

```cpp
void advanced_monitoring_example() {
    auto& temp_manager = TemperatureManager::GetInstance();
    temp_manager.EnsureInitialized();
    
    // Configure NTC sensor
    TemperatureSensorConfig config;
    config.sensor_id = "MOTOR_TEMP";
    config.sensor_type = TemperatureSensorType::NTC_THERMISTOR;
    config.hardware_source = "ADC_CH0";
    config.r25_ohms = 10000.0f;  // 10kΩ NTC
    config.beta_value = 3950.0f;
    config.series_resistance = 10000.0f;  // 10kΩ series resistor
    config.min_temperature = -40.0f;
    config.max_temperature = 125.0f;
    config.alert_threshold_high = 80.0f;
    config.update_rate_hz = 10.0f;
    config.enable_monitoring = true;
    
    // Register sensor
    if (temp_manager.RegisterSensor("MOTOR_TEMP", config) == TemperatureError::SUCCESS) {
        logger.Info("TEMP", "Motor temperature sensor registered\n");
    }
    
    // Set up temperature alert
    if (temp_manager.SetTemperatureAlert("MOTOR_TEMP", 85.0f, 
                                        TemperatureAlertType::HIGH_TEMPERATURE) == TemperatureError::SUCCESS) {
        logger.Info("TEMP", "High temperature alert configured at 85°C\n");
    }
    
    // Start monitoring
    if (temp_manager.StartMonitoring("MOTOR_TEMP", 5.0f) == TemperatureError::SUCCESS) {
        logger.Info("TEMP", "Temperature monitoring started at 5Hz\n");
    }
}
```

### Temperature Calibration

```cpp
void calibration_example() {
    auto& temp_manager = TemperatureManager::GetInstance();
    temp_manager.EnsureInitialized();
    
    // Perform sensor calibration
    if (temp_manager.CalibrateSensor("NTC_1") == TemperatureError::SUCCESS) {
        logger.Info("TEMP", "Sensor calibration completed\n");
    }
    
    // Set manual calibration offset
    if (temp_manager.SetCalibrationOffset("NTC_1", 1.5f) == TemperatureError::SUCCESS) {
        logger.Info("TEMP", "Calibration offset set to +1.5°C\n");
    }
    
    // Verify calibration
    float offset;
    if (temp_manager.GetCalibrationOffset("NTC_1", offset) == TemperatureError::SUCCESS) {
        logger.Info("TEMP", "Current calibration offset: %.2f°C\n", offset);
    }
}
```

### System Diagnostics

```cpp
void diagnostics_example() {
    auto& temp_manager = TemperatureManager::GetInstance();
    temp_manager.EnsureInitialized();
    
    // Get system status
    auto status = temp_manager.GetSystemStatus();
    logger.Info("TEMP", "Temperature system status:\n");
    logger.Info("TEMP", "  Overall healthy: %s\n", status.overall_healthy ? "Yes" : "No");
    logger.Info("TEMP", "  Active sensors: %u\n", status.active_sensors);
    logger.Info("TEMP", "  Total sensors: %u\n", status.total_sensors);
    logger.Info("TEMP", "  Monitoring active: %s\n", status.monitoring_active ? "Yes" : "No");
    
    // Get sensor diagnostics
    for (const auto& sensor_id : temp_manager.GetAvailableSensors()) {
        TemperatureSensorDiagnostics diagnostics;
        if (temp_manager.GetSensorDiagnostics(sensor_id, diagnostics) == TemperatureError::SUCCESS) {
            logger.Info("TEMP", "Sensor %s diagnostics:\n", sensor_id.c_str());
            logger.Info("TEMP", "  Healthy: %s\n", diagnostics.healthy ? "Yes" : "No");
            logger.Info("TEMP", "  Last reading: %.2f°C\n", diagnostics.last_temperature);
            logger.Info("TEMP", "  Min temperature: %.2f°C\n", diagnostics.min_temperature);
            logger.Info("TEMP", "  Max temperature: %.2f°C\n", diagnostics.max_temperature);
            logger.Info("TEMP", "  Average temperature: %.2f°C\n", diagnostics.average_temperature);
            logger.Info("TEMP", "  Reading count: %u\n", diagnostics.reading_count);
            logger.Info("TEMP", "  Error count: %u\n", diagnostics.error_count);
        }
    }
    
    // Get performance statistics
    auto stats = temp_manager.GetStatistics();
    logger.Info("TEMP", "Performance statistics:\n");
    logger.Info("TEMP", "  Total readings: %u\n", stats.total_readings);
    logger.Info("TEMP", "  Successful readings: %u\n", stats.successful_readings);
    logger.Info("TEMP", "  Failed readings: %u\n", stats.failed_readings);
    logger.Info("TEMP", "  Average response time: %.2f ms\n", stats.average_response_time_ms);
    logger.Info("TEMP", "  Peak response time: %.2f ms\n", stats.peak_response_time_ms);
}
```

### Continuous Monitoring Loop

```cpp
void continuous_monitoring_loop() {
    auto& temp_manager = TemperatureManager::GetInstance();
    temp_manager.EnsureInitialized();
    
    // Configure monitoring for all sensors
    for (const auto& sensor_id : temp_manager.GetAvailableSensors()) {
        temp_manager.StartMonitoring(sensor_id, 2.0f);  // 2Hz monitoring
    }
    
    // Continuous monitoring loop
    for (int i = 0; i < 1000; i++) {
        // Read all temperatures
        std::vector<TemperatureReading> readings;
        if (temp_manager.ReadAllTemperatures(readings) == TemperatureError::SUCCESS) {
            logger.Info("TEMP", "Temperature readings (sample %d):\n", i);
            for (const auto& reading : readings) {
                logger.Info("TEMP", "  %s: %.2f°C (valid: %s)\n", 
                       reading.sensor_id.c_str(), 
                       reading.temperature_celsius,
                       reading.valid ? "Yes" : "No");
                
                // Check for temperature alerts
                if (reading.temperature_celsius > 80.0f) {
                    logger.Info("TEMP", "WARNING: High temperature detected on %s: %.2f°C\n", 
                           reading.sensor_id.c_str(), reading.temperature_celsius);
                }
            }
        }
        
        // Check system health
        if (!temp_manager.IsSystemHealthy()) {
            logger.Info("TEMP", "WARNING: Temperature system health check failed\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));  // 2Hz monitoring
    }
}
```

## 🔍 Advanced Usage

### Custom Temperature Sensors

```cpp
void custom_sensor_example() {
    auto& temp_manager = TemperatureManager::GetInstance();
    temp_manager.EnsureInitialized();
    
    // Configure custom I2C temperature sensor
    TemperatureSensorConfig config;
    config.sensor_id = "CUSTOM_I2C_TEMP";
    config.sensor_type = TemperatureSensorType::I2C_SENSOR;
    config.hardware_source = "I2C_DEVICE_2";
    config.min_temperature = -55.0f;
    config.max_temperature = 150.0f;
    config.update_rate_hz = 1.0f;
    
    // Register custom sensor
    if (temp_manager.RegisterSensor("CUSTOM_I2C_TEMP", config) == TemperatureError::SUCCESS) {
        logger.Info("TEMP", "Custom I2C temperature sensor registered\n");
    }
}
```

### Temperature Trend Analysis

```cpp
void trend_analysis_example() {
    auto& temp_manager = TemperatureManager::GetInstance();
    temp_manager.EnsureInitialized();
    
    // Collect temperature data over time
    std::vector<float> temperature_history;
    const int samples = 100;
    
    for (int i = 0; i < samples; i++) {
        float temperature;
        if (temp_manager.ReadTemperature("NTC_1", temperature) == TemperatureError::SUCCESS) {
            temperature_history.push_back(temperature);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz sampling
    }
    
    // Analyze temperature trends
    if (temperature_history.size() >= 2) {
        float min_temp = *std::min_element(temperature_history.begin(), temperature_history.end());
        float max_temp = *std::max_element(temperature_history.begin(), temperature_history.end());
        float avg_temp = std::accumulate(temperature_history.begin(), temperature_history.end(), 0.0f) / temperature_history.size();
        
        logger.Info("TEMP", "Temperature analysis:\n");
        logger.Info("TEMP", "  Min: %.2f°C\n", min_temp);
        logger.Info("TEMP", "  Max: %.2f°C\n", max_temp);
        logger.Info("TEMP", "  Average: %.2f°C\n", avg_temp);
        logger.Info("TEMP", "  Range: %.2f°C\n", max_temp - min_temp);
        
        // Check for temperature stability
        if ((max_temp - min_temp) < 2.0f) {
            logger.Info("TEMP", "Temperature is stable\n");
        } else {
            logger.Info("TEMP", "Temperature is varying significantly\n");
        }
    }
}
```

## � Error Codes

### TemperatureError Reference

| Code | Value | Description |
|------|-------|-------------|
| `SUCCESS` | 0 | Operation completed successfully |
| `NOT_INITIALIZED` | 1 | Manager not yet initialised |
| `INITIALIZATION_FAILED` | 2 | Sensor initialisation failed |
| `SENSOR_NOT_FOUND` | 3 | Named sensor does not exist in registry |
| `SENSOR_NOT_AVAILABLE` | 4 | Sensor exists but is unavailable |
| `READ_FAILED` | 5 | Driver-level read returned an error |
| `INVALID_PARAMETER` | 6 | Null pointer or out-of-range argument |
| `NO_SENSORS_AVAILABLE` | 7 | No sensors were registered at init |
| `ALLOCATION_FAILED` | 8 | Heap allocation for a sensor driver failed |
| `DEPENDENCY_NOT_READY` | 9 | Required dependency not initialised |
| `MUTEX_LOCK_FAILED` | 10 | RTOS mutex acquire timed out |

## �📚 See Also

- **[AdcManager Documentation](ADC_MANAGER_README.md)** - ADC system for NTC thermistor readings
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - I2C communication for external sensors
- **[NTC Temperature Handler Documentation](../driver-handlers/NTC_TEMPERATURE_HANDLER_README.md)** - NTC thermistor driver
- **[TMC9660 Temperature Documentation](../driver-handlers/TMC9660_TEMPERATURE_README.md)** - Motor controller temperature monitoring

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*
