# AdcManager - Advanced ADC Management System

<div align="center">

![Component](https://img.shields.io/badge/component-AdcManager-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Hardware](https://img.shields.io/badge/hardware-ESP32--C6%20|%20TMC9660-orange.svg)

**Comprehensive ADC management system for the HardFOC platform**

</div>

## 📋 Overview

The `AdcManager` is a singleton component handler that provides unified, thread-safe access to ADC channels across multiple hardware sources. It integrates with the platform mapping system to automatically manage ADC channels from ESP32-C6 internal ADC and TMC9660 motor controller ADC through a single, consistent API using string-based channel identification.

### ✨ Key Features

- **🔗 Multi-Source ADC Management**: ESP32-C6 internal ADC, TMC9660 ADC
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **📍 String-Based Channel Identification**: Flexible, extensible channel naming
- **🛡️ Platform Mapping Integration**: Automatic hardware discovery
- **📊 Advanced Diagnostics**: Real-time health monitoring and calibration
- **⚡ Batch Operations**: Optimized multi-channel operations
- **🎯 Voltage Calibration**: Automatic voltage conversion with reference scaling
- **🏥 Health Monitoring**: Per-chip and per-channel statistics

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        AdcManager                              │
├─────────────────────────────────────────────────────────────────┤
│  String-Based API    │ channel_name → hardware mapping        │
├─────────────────────────────────────────────────────────────────┤
│  Platform Integration│ Automatic channel discovery & registration│
├─────────────────────────────────────────────────────────────────┤
│  Hardware Handlers   │ ESP32 ADC, TMC9660 ADC handlers        │
├─────────────────────────────────────────────────────────────────┤
│  BaseAdc Interface   │ Unified ADC operations                  │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Usage

```cpp
#include "component-handlers/AdcManager.h"

void adc_example() {
    // Get singleton instance
    auto& adc = AdcManager::GetInstance();
    
    // Initialize the manager
    if (!adc.Initialize()) {
        logger.Info("ADC", "Failed to initialize ADC manager\n");
        return;
    }
    
    // Read raw ADC value
    uint32_t raw_value;
    if (adc.ReadChannelCount("ESP32_ADC1_CH0", raw_value) == HF_ADC_SUCCESS) {
        logger.Info("ADC", "Raw ADC value: %u\n", raw_value);
    }
    
    // Read calibrated voltage
    float voltage;
    if (adc.ReadVoltage("ESP32_ADC1_CH0", voltage) == HF_ADC_SUCCESS) {
        logger.Info("ADC", "Voltage: %.3fV\n", voltage);
    }
}
```

## 📖 API Reference

### Critical Enum Values and Meanings

#### **ADC Error Code Enums**
```cpp
// ADC Success/Error Codes
hf_adc_err_t::ADC_SUCCESS = 0                      // Operation successful
hf_adc_err_t::ADC_ERR_INVALID_PARAMETER = 4        // Invalid parameter
hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND = 7        // Channel not found
hf_adc_err_t::ADC_ERR_INVALID_CHANNEL = 13         // Invalid channel
hf_adc_err_t::ADC_ERR_HARDWARE_FAULT = 19          // Hardware fault
hf_adc_err_t::ADC_ERR_COMMUNICATION_FAILURE = 20   // Communication failure
hf_adc_err_t::ADC_ERR_CALIBRATION_FAILURE = 22     // Calibration failure
hf_adc_err_t::ADC_ERR_VOLTAGE_OUT_OF_RANGE = 23    // Voltage out of range
```

#### **ADC Channel Types**
```cpp
// Channel ID type (platform-agnostic)
using hf_channel_id_t = hf_u32_t;  // 32-bit unsigned integer

// Invalid channel constant
constexpr hf_channel_id_t HF_INVALID_CHANNEL = std::numeric_limits<hf_channel_id_t>::max();

// Time types for sampling
using hf_time_t = hf_u32_t;        // Time in milliseconds
using hf_timeout_ms_t = hf_time_t; // Timeout in milliseconds
```

#### **ADC Sampling Parameters**
```cpp
// Sample count type
using hf_u8_t = uint8_t;  // 8-bit unsigned integer (0-255 samples)

// Common timeout values
constexpr hf_time_t HF_TIMEOUT_DEFAULT_MS = 1000;  // Default timeout
constexpr hf_time_t HF_TIMEOUT_NONE = 0;           // No timeout (wait indefinitely)
constexpr hf_time_t HF_TIMEOUT_MAX = std::numeric_limits<hf_time_t>::max();
```

### **🎯 How to Use ADC Enums Correctly**

#### **1. ADC Error Handling Best Practices**
```cpp
// Always check return codes for ADC operations
float voltage;
auto result = adc.ReadVoltage("ADC_TMC9660_AIN3", voltage);
if (result != hf_adc_err_t::ADC_SUCCESS) {
    logger.Error("ADC", "Failed to read voltage: %s", HfAdcErrToString(result));
    
    // Handle specific error types
    switch (result) {
        case hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND:
            logger.Error("ADC", "Channel not found - check channel name");
            break;
        case hf_adc_err_t::ADC_ERR_HARDWARE_FAULT:
            logger.Error("ADC", "Hardware fault - check connections");
            break;
        case hf_adc_err_t::ADC_ERR_VOLTAGE_OUT_OF_RANGE:
            logger.Error("ADC", "Voltage out of range - check input voltage");
            break;
        default:
            logger.Error("ADC", "Unknown error occurred");
            break;
    }
}
```

#### **2. ADC Sampling Configuration**
```cpp
// Single sample reading (fastest)
float voltage;
adc.ReadVoltage("ADC_TMC9660_AIN3", voltage, 1);

// Multiple sample averaging (more accurate)
float voltage_avg;
adc.ReadVoltage("ADC_TMC9660_AIN3", voltage_avg, 16);  // 16 samples

// Raw count reading
uint32_t raw_count;
adc.ReadChannelCount("ADC_TMC9660_AIN3", raw_count, 8, 2);  // 8 samples, 2ms between
```

#### **3. Batch ADC Operations**
```cpp
// Read multiple channels simultaneously
std::vector<std::string_view> channels = {
    "ADC_TMC9660_AIN0", "ADC_TMC9660_AIN1", 
    "ADC_TMC9660_AIN2", "ADC_TMC9660_AIN3"
};

auto batch_result = adc.BatchRead(channels, 4, 1);  // 4 samples per channel, 1ms interval
if (batch_result.overall_result == hf_adc_err_t::ADC_SUCCESS) {
    for (size_t i = 0; i < channels.size(); i++) {
        logger.Info("ADC", "Channel %s: %.3fV", channels[i].data(), batch_result.voltages[i]);
    }
}
```

#### **4. Timeout and Error Handling**
```cpp
// Use appropriate timeouts for different operations
float voltage;

// Fast operation (short timeout)
auto result = adc.ReadVoltage("ADC_TMC9660_AIN3", voltage, 1);
if (result == hf_adc_err_t::ADC_ERR_TIMEOUT) {
    logger.Warn("ADC", "Fast read timeout - consider longer timeout");
}

// Slow operation (longer timeout for averaging)
result = adc.ReadVoltage("ADC_TMC9660_AIN3", voltage, 64);  // 64 samples
if (result == hf_adc_err_t::ADC_ERR_TIMEOUT) {
    logger.Error("ADC", "Averaging read timeout - check hardware");
}
```

#### **5. Channel Validation**
```cpp
// Check if channel exists before using
if (adc.Contains("ADC_TMC9660_AIN3")) {
    float voltage;
    auto result = adc.ReadVoltage("ADC_TMC9660_AIN3", voltage);
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info("ADC", "Voltage reading: %.3fV", voltage);
    }
} else {
    logger.Error("ADC", "Channel ADC_TMC9660_AIN3 not available");
}
```

### Core Operations

#### Initialization
```cpp
class AdcManager {
public:
    // Singleton access
    static AdcManager& GetInstance() noexcept;
    
    // Initialization
    bool Initialize() noexcept;
    bool IsInitialized() const noexcept;
    void Deinitialize() noexcept;
};
```

#### ADC Registration and Management
```cpp
// Channel registration
hf_adc_err_t RegisterAdc(std::string_view name, std::unique_ptr<BaseAdc> adc) noexcept;
BaseAdc* Get(std::string_view name) noexcept;
bool Contains(std::string_view name) const noexcept;
size_t Size() const noexcept;
```

#### Single Channel Operations
```cpp
// Raw ADC readings
hf_adc_err_t ReadChannelCount(std::string_view name, uint32_t& count, 
                             uint8_t samples = 1, uint16_t interval_ms = 0) noexcept;

// Voltage readings
hf_adc_err_t ReadVoltage(std::string_view name, float& voltage,
                        uint8_t samples = 1) noexcept;

// Channel validation
bool IsChannelAvailable(std::string_view name) const noexcept;
```

#### Batch Operations
```cpp
// Multi-channel operations
AdcBatchResult BatchRead(const AdcBatchOperation& operation) noexcept;
AdcBatchResult BatchReadVoltages(const std::vector<std::string_view>& channels,
                                uint8_t samples = 1, uint16_t interval_ms = 0) noexcept;
AdcBatchResult BatchReadCounts(const std::vector<std::string_view>& channels,
                              uint8_t samples = 1, uint16_t interval_ms = 0) noexcept;
```

#### Statistics and Diagnostics
```cpp
// System diagnostics
hf_adc_err_t GetSystemDiagnostics(AdcSystemDiagnostics& diagnostics) const noexcept;
hf_adc_err_t GetStatistics(std::string_view name, BaseAdc::AdcStatistics& statistics) const noexcept;
hf_adc_err_t ResetStatistics(std::string_view name) noexcept;
void DumpStatistics() const noexcept;
```

### Data Structures

#### AdcChannelInfo
```cpp
struct AdcChannelInfo {
    std::string_view name;                      // Human-readable name
    std::unique_ptr<BaseAdc> adc_driver;        // ADC driver instance
    HfFunctionalAdcChannel functional_channel;  // Functional channel identifier
    HfAdcChipType hardware_chip;                // Hardware chip identifier
    uint8_t hardware_channel_id;                // Hardware channel ID within the chip
    bool is_registered;                         // Registration status
    
    // Hardware configuration
    float reference_voltage;                    // Reference voltage for conversion
    uint32_t resolution_bits;                   // ADC resolution in bits
    uint32_t max_voltage_mv;                    // Maximum voltage in millivolts
    float voltage_divider;                      // Voltage divider ratio
    
    // Statistics and monitoring
    uint32_t access_count;                      // Number of times accessed
    uint32_t error_count;                       // Number of errors encountered
    uint64_t last_access_time;                  // Timestamp of last access
};
```

#### AdcSystemDiagnostics
```cpp
struct AdcSystemDiagnostics {
    bool system_healthy;                           // Overall system health
    uint32_t total_channels_registered;            // Total channels registered
    uint32_t channels_by_chip[4];                  // Channels per chip
    uint32_t total_operations;                     // Total operations performed
    uint32_t successful_operations;                // Successful operations
    uint32_t failed_operations;                    // Failed operations
    uint32_t calibration_errors;                   // Calibration errors
    uint32_t communication_errors;                 // Communication errors
    uint32_t hardware_errors;                      // Hardware errors
    uint64_t system_uptime_ms;                     // System uptime
    float average_read_time_us;                    // Average read time in microseconds
    hf_adc_err_t last_error;                       // Last error encountered
};
```

#### AdcBatchOperation
```cpp
struct AdcBatchOperation {
    std::vector<std::string_view> channel_names;    // Channel names to operate on
    std::vector<uint8_t> samples_per_channel;       // Samples per channel
    std::vector<uint16_t> intervals_ms;             // Intervals between samples in ms
    bool use_individual_specs;                      // Use individual specs or common settings
    uint8_t common_samples;                         // Common number of samples
    uint16_t common_interval_ms;                    // Common sampling interval
};
```

#### AdcBatchResult
```cpp
struct AdcBatchResult {
    std::vector<std::string_view> channel_names;    // Channel names operated on
    std::vector<float> voltages;                    // Resulting voltage readings
    std::vector<uint32_t> raw_values;               // Raw ADC values
    std::vector<hf_adc_err_t> results;              // Individual operation results
    hf_adc_err_t overall_result;                    // Overall operation result
    uint32_t total_time_ms;                         // Total operation time
    
    bool AllSuccessful() const noexcept;            // Check if all operations were successful
};
```

## 🎯 Hardware Support

### Supported Hardware Sources

| Hardware | Channels Available | Resolution | Reference Voltage | Features |
|----------|-------------------|------------|------------------|----------|
| **ESP32-C6** | 6 channels (ADC1: 0-4, ADC2: 0) | 12-bit | 3.3V | Calibration, attenuation |
| **TMC9660** | 12 channels (4 current + 4 analog + 4 voltage/temp) | 12-bit | 3.3V | Motor feedback, diagnostics |

### Channel Naming Convention

```cpp
// ESP32-C6 ADC channels
"ESP32_ADC1_CH0" to "ESP32_ADC1_CH4"    // ADC1 channels 0-4
"ESP32_ADC2_CH0"                        // ADC2 channel 0

// TMC9660 ADC channels
"TMC9660_CURRENT_I0" to "TMC9660_CURRENT_I3"     // Current sense channels
"TMC9660_AIN0" to "TMC9660_AIN3"                 // External analog inputs
"TMC9660_VOLTAGE_SUPPLY"                         // Supply voltage monitoring
"TMC9660_VOLTAGE_DRIVER"                         // Driver voltage monitoring
"TMC9660_TEMP_CHIP"                             // Chip temperature
"TMC9660_TEMP_EXTERNAL"                         // External temperature

// Functional channel names (platform-mapped)
"MOTOR_CURRENT_SENSE"                   // Motor current feedback
"BATTERY_VOLTAGE"                       // Battery voltage monitoring
"TEMPERATURE_SENSOR"                    // Temperature monitoring
"USER_ANALOG_1"                         // User-defined analog input
```

## 🔧 Configuration

### Platform Integration

The AdcManager automatically integrates with the platform mapping system:

```cpp
// Platform mapping integration
#include "hf_functional_pin_config_vortex_v1.hpp"

// Automatic channel discovery based on platform configuration
// Channels are registered automatically during initialization
```

### Error Handling

The AdcManager uses comprehensive error handling with specific error codes:

```cpp
// Common error codes
hf_adc_err_t::HF_ADC_SUCCESS              // Operation successful
hf_adc_err_t::HF_ADC_ERR_INVALID_PARAMETER // Invalid channel name or parameters
hf_adc_err_t::HF_ADC_ERR_NOT_INITIALIZED   // ADC manager not initialized
hf_adc_err_t::HF_ADC_ERR_HARDWARE_FAULT    // Hardware communication error
hf_adc_err_t::HF_ADC_ERR_CALIBRATION       // Calibration error
hf_adc_err_t::HF_ADC_ERR_OUT_OF_MEMORY     // Memory allocation failed
```

## 📊 Examples

### Basic ADC Reading

```cpp
#include "component-handlers/AdcManager.h"
#include "utils-and-drivers/driver-handlers/Logger.h"

void basic_adc_example() {
    auto& logger = Logger::GetInstance();
    auto& adc = AdcManager::GetInstance();
    adc.EnsureInitialized();
    
    // Single channel readings using correct channel names from pin config
    uint32_t raw_value;
    float voltage;
    
    if (adc.ReadChannelCount("ADC_TMC9660_AIN3", raw_value) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info("ADC", "Channel ADC_TMC9660_AIN3 raw: %u", raw_value);
    }
    
    if (adc.ReadVoltage("ADC_TMC9660_AIN3", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info("ADC", "Channel ADC_TMC9660_AIN3 voltage: %.3fV", voltage);
    }
    
    // Multiple samples for stability
    if (adc.ReadVoltage("ADC_TMC9660_AIN3", voltage, 16) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info("ADC", "Channel ADC_TMC9660_AIN3 filtered: %.3fV", voltage);
    }
    
    // Read TMC9660 internal monitoring channels
    if (adc.ReadVoltage("TMC9660_CHIP_TEMPERATURE", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info("ADC", "TMC9660 Chip Temperature: %.3fV", voltage);
    }
    
    if (adc.ReadVoltage("TMC9660_SUPPLY_VOLTAGE", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info("ADC", "TMC9660 Supply Voltage: %.3fV", voltage);
    }
}
```

### Multi-Channel Reading

```cpp
void multi_channel_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Define channels to read
    std::vector<std::string_view> channels = {
        "ESP32_ADC1_CH0",
        "ESP32_ADC1_CH1",
        "TMC9660_AIN0",
        "TMC9660_AIN1"
    };
    
    // Read all channels simultaneously
    auto readings = adc.BatchReadVoltages(channels, 4);  // 4 samples each
    
    logger.Info("ADC", "Multi-channel readings (time: %u ms):\n", readings.total_time_ms);
    for (size_t i = 0; i < channels.size(); i++) {
        if (readings.results[i] == HF_ADC_SUCCESS) {
            logger.Info("ADC", "  %.*s: %.3fV (raw: %u)\n",
                   static_cast<int>(channels[i].size()), channels[i].data(),
                   readings.voltages[i], readings.raw_values[i]);
        } else {
            logger.Info("ADC", "  %.*s: ERROR\n",
                   static_cast<int>(channels[i].size()), channels[i].data());
        }
    }
    
    logger.Info("ADC", "All successful: %s\n", readings.AllSuccessful() ? "YES" : "NO");
}
```

### Motor Current Monitoring

```cpp
void motor_current_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Monitor motor current from TMC9660
    logger.Info("ADC", "Motor current monitoring:\n");
    for (int i = 0; i < 100; i++) {
        float voltage;
        if (adc.ReadVoltage("TMC9660_CURRENT_I0", voltage, 8) == HF_ADC_SUCCESS) {
            // Convert voltage to current (example: 0.1V/A current sensor)
            float current = voltage / 0.1f;
            logger.Info("ADC", "Current: %.2fA (%.3fV)\n", current, voltage);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Battery Voltage Monitoring

```cpp
void battery_voltage_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Monitor battery voltage with filtering
    while (true) {
        float battery_voltage;
        if (adc.ReadVoltage("ESP32_ADC1_CH2", battery_voltage, 32) == HF_ADC_SUCCESS) {
            logger.Info("ADC", "Battery: %.2fV", battery_voltage);
            
            // Battery status indication
            if (battery_voltage > 12.0f) {
                logger.Info("ADC", " [GOOD]\n");
            } else if (battery_voltage > 10.5f) {
                logger.Info("ADC", " [LOW]\n");
            } else {
                logger.Info("ADC", " [CRITICAL]\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### System Diagnostics

```cpp
void diagnostics_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Get system status
    AdcSystemDiagnostics diagnostics;
    if (adc.GetSystemDiagnostics(diagnostics) == HF_ADC_SUCCESS) {
        logger.Info("ADC", "ADC System Status:\n");
        logger.Info("ADC", "  Overall healthy: %s\n", diagnostics.system_healthy ? "YES" : "NO");
        logger.Info("ADC", "  Total channels: %u\n", diagnostics.total_channels_registered);
        logger.Info("ADC", "  Total operations: %u\n", diagnostics.total_operations);
        logger.Info("ADC", "  Successful operations: %u\n", diagnostics.successful_operations);
        logger.Info("ADC", "  Failed operations: %u\n", diagnostics.failed_operations);
        logger.Info("ADC", "  Calibration errors: %u\n", diagnostics.calibration_errors);
        logger.Info("ADC", "  Communication errors: %u\n", diagnostics.communication_errors);
        logger.Info("ADC", "  Hardware errors: %u\n", diagnostics.hardware_errors);
        logger.Info("ADC", "  Avg read time: %.2fµs\n", diagnostics.average_read_time_us);
        logger.Info("ADC", "  System uptime: %llu ms\n", diagnostics.system_uptime_ms);
    }
    
    // Get channel statistics
    BaseAdc::AdcStatistics stats;
    if (adc.GetStatistics("ESP32_ADC1_CH0", stats) == HF_ADC_SUCCESS) {
        logger.Info("ADC", "\nChannel ESP32_ADC1_CH0 statistics:\n");
        logger.Info("ADC", "  Access count: %u\n", stats.access_count);
        logger.Info("ADC", "  Error count: %u\n", stats.error_count);
        logger.Info("ADC", "  Last access time: %llu\n", stats.last_access_time);
    }
    
    // Dump all statistics
    adc.DumpStatistics();
}
```

## 🔍 Advanced Usage

### ⚡ Performance Optimization Guide

The ADC Manager provides two distinct access patterns optimized for different performance requirements:

#### 🔍 String-Based API (Convenience & Extensibility)
Best for configuration, initialization, debugging, and user interfaces:

```cpp
void configuration_example() {
    auto& adc = AdcManager::GetInstance();
    adc.EnsureInitialized();
    
    // String-based API - Great for convenience and readability
    float voltage = adc.ReadVoltage("ADC_TMC9660_AIN3");      // ~200-800ns per call
    float current = adc.ReadVoltage("TMC9660_CURRENT_I0");    // Includes hash map lookup
    
    logger.Info("ADC", "Temperature sensor: %.3fV\n", voltage);
    logger.Info("ADC", "Motor current: %.3fV\n", current);
}
```

#### ⚡ Cached Access (High Performance)
For real-time control loops and high-frequency sampling (>1kHz):

```cpp
void high_performance_example() {
    auto& adc = AdcManager::GetInstance();
    adc.EnsureInitialized();
    
    // STEP 1: Cache BaseAdc pointers for fast access
    auto* adc_temp = adc.Get("ADC_TMC9660_AIN3");           // Get raw pointer
    auto* adc_current = adc.Get("TMC9660_CURRENT_I0");
    auto* adc_velocity = adc.Get("TMC9660_MOTOR_VELOCITY");
    
    // STEP 2: Validate cached pointers (once, outside loops)
    if (!adc_temp || !adc_current || !adc_velocity) {
        logger.Info("ADC", "ERROR: Failed to cache ADC pointers\n");
        return;
    }
    
    // STEP 3: Use cached pointers for direct hardware access
    for (int i = 0; i < 50000; i++) {
        // Direct BaseAdc access - ~20-100ns per call
        float temperature, current, velocity;
        
        // No string lookup overhead - maximum performance
        if (adc_temp->ReadVoltage(temperature) == hf_adc_err_t::ADC_SUCCESS) {
            // Process temperature reading immediately
        }
        
        if (adc_current->ReadVoltage(current) == hf_adc_err_t::ADC_SUCCESS) {
            // Process current reading for motor control
        }
        
        if (adc_velocity->ReadVoltage(velocity) == hf_adc_err_t::ADC_SUCCESS) {
            // Process velocity feedback for control loop
        }
    }
}
```

#### 📊 Performance Comparison

| Operation | String Lookup | Cached Access | Speedup |
|-----------|---------------|---------------|---------|
| `ReadVoltage()` | ~200-800ns | ~20-100ns | **5-15x faster** |
| `ReadChannel()` | ~300-900ns | ~30-120ns | **5-15x faster** |
| `ReadChannelCount()` | ~250-850ns | ~25-110ns | **5-15x faster** |

#### 🎯 When to Use Each Approach

**Use String-Based API for:**
- ✅ System configuration and initialization
- ✅ Calibration and diagnostic tools
- ✅ One-time sensor readings
- ✅ User interfaces and data logging
- ✅ Dynamic channel registration from configuration

**Use Cached Access for:**
- ⚡ Real-time motor control feedback loops (>1kHz)
- ⚡ High-frequency sensor monitoring
- ⚡ Current sensing for protection systems
- ⚡ Temperature monitoring in tight loops
- ⚡ Position/velocity feedback for servo control

#### 🔄 Batch Operations for Maximum Efficiency

For reading multiple channels simultaneously:

```cpp
void batch_operations_example() {
    auto& adc = AdcManager::GetInstance();
    adc.EnsureInitialized();
    
    // Method 1: String-based batch operations
    std::vector<std::string_view> channels = {
        "ADC_TMC9660_AIN0", "ADC_TMC9660_AIN1", 
        "ADC_TMC9660_AIN2", "ADC_TMC9660_AIN3"
    };
    
    // Benchmark single vs batch reads
    auto start_time = esp_timer_get_time();
    
    // Single reads (slower due to individual string lookups)
    for (int i = 0; i < 1000; i++) {
        for (const auto& channel : channels) {
            float voltage;
            adc.ReadVoltage(channel, voltage);
        }
    }
    
    auto single_time = esp_timer_get_time() - start_time;
    
    start_time = esp_timer_get_time();
    
    // Batch reads (faster - single lookup, optimized hardware access)
    for (int i = 0; i < 1000; i++) {
        auto result = adc.BatchReadVoltages(channels, 1);
        // Process all readings together
        for (size_t j = 0; j < result.voltages.size(); j++) {
            // Handle result.voltages[j]
        }
    }
    
    auto batch_time = esp_timer_get_time() - start_time;
    
    logger.Info("ADC", "Performance comparison:\n");
    logger.Info("ADC", "  Single reads: %lld µs\n", single_time);
    logger.Info("ADC", "  Batch reads: %lld µs\n", batch_time);
    logger.Info("ADC", "  Speedup: %.1fx\n", static_cast<float>(single_time) / batch_time);
}
```

#### 🚀 Ultra-High Performance with Cached Batch Access

For maximum performance, combine caching with batch operations:

```cpp
void ultra_high_performance_example() {
    auto& adc = AdcManager::GetInstance();
    adc.EnsureInitialized();
    
    // Cache multiple ADC channel pointers
    std::vector<BaseAdc*> cached_channels;
    std::vector<std::string_view> channel_names = {
        "ADC_TMC9660_AIN0", "ADC_TMC9660_AIN1", 
        "TMC9660_CURRENT_I0", "TMC9660_MOTOR_VELOCITY"
    };
    
    // Build cache once
    for (const auto& name : channel_names) {
        auto* channel = adc.Get(name);
        if (channel) {
            cached_channels.push_back(channel);
        }
    }
    
    if (cached_channels.size() != channel_names.size()) {
        logger.Info("ADC", "ERROR: Failed to cache all ADC channels\n");
        return;
    }
    
    // Ultra-fast reading loop with cached pointers
    for (int i = 0; i < 100000; i++) {
        std::array<float, 4> readings;
        
        // Direct hardware access with no overhead
        for (size_t j = 0; j < cached_channels.size(); j++) {
            cached_channels[j]->ReadVoltage(readings[j]);
        }
        
        // Process all readings immediately
        // This can run at maximum ADC sampling rate
    }
}
```

#### ⚠️ Important Performance Notes

1. **Cache Validation**: Always validate cached pointers before use
2. **Memory Management**: ADC manager retains ownership of BaseAdc objects
3. **Thread Safety**: Both string and cached access are thread-safe
4. **Channel Registration**: String lookups enable dynamic channel registration
5. **Hardware Optimization**: TMC9660 ADC channels can be read efficiently in batch
6. **Sampling Rate**: Cached access enables maximum hardware sampling rates

### Advanced Batch Operations

```cpp
void advanced_batch_example() {
    auto& adc = AdcManager::GetInstance();
    adc.Initialize();
    
    // Create batch operation with individual specifications
    std::vector<std::string_view> channels = {
        "ESP32_ADC1_CH0", "ESP32_ADC1_CH1", "TMC9660_AIN0"
    };
    
    std::vector<uint8_t> samples = {16, 8, 32};  // Different sample counts
    std::vector<uint16_t> intervals = {0, 10, 5}; // Different intervals
    
    AdcBatchOperation operation(channels, samples, intervals);
    auto result = adc.BatchRead(operation);
    
    logger.Info("ADC", "Advanced batch operation results:\n");
    for (size_t i = 0; i < channels.size(); i++) {
        if (result.results[i] == HF_ADC_SUCCESS) {
            logger.Info("ADC", "  %.*s: %.3fV (raw: %u, samples: %u, interval: %u ms)\n",
                   static_cast<int>(channels[i].size()), channels[i].data(),
                   result.voltages[i], result.raw_values[i],
                   samples[i], intervals[i]);
        }
    }
    
    logger.Info("ADC", "Total time: %u ms\n", result.total_time_ms);
}
```

## 🚨 Error Handling

### Common Error Scenarios

```cpp
void error_handling_example() {
    auto& adc = AdcManager::GetInstance();
    
    // Check initialization
    if (!adc.Initialize()) {
        logger.Info("ADC", "ERROR: Failed to initialize ADC manager\n");
        return;
    }
    
    // Validate channel exists before use
    if (!adc.IsChannelAvailable("ESP32_ADC1_CH0")) {
        logger.Info("ADC", "ERROR: Channel ESP32_ADC1_CH0 not available\n");
        return;
    }
    
    // Safe ADC operations with error checking
    float voltage;
    auto result = adc.ReadVoltage("ESP32_ADC1_CH0", voltage);
    if (result != HF_ADC_SUCCESS) {
        logger.Info("ADC", "ERROR: Failed to read ADC channel: %d\n", static_cast<int>(result));
    }
    
    // Monitor system health
    AdcSystemDiagnostics diagnostics;
    if (adc.GetSystemDiagnostics(diagnostics) == HF_ADC_SUCCESS) {
        if (!diagnostics.system_healthy) {
            logger.Info("ADC", "WARNING: ADC system health check failed\n");
            logger.Info("ADC", "Failed operations: %u\n", diagnostics.failed_operations);
            logger.Info("ADC", "Calibration errors: %u\n", diagnostics.calibration_errors);
        }
    }
}
```

### `hf_adc_err_t` Error Code Reference

| Category | Codes | Description |
|----------|-------|-------------|
| **General** | `ADC_SUCCESS` (0) | Operation completed successfully |
| | `ADC_ERR_FAILURE` (1) | General failure |
| | `ADC_ERR_NOT_INITIALIZED` (2) | Not initialized |
| | `ADC_ERR_ALREADY_INITIALIZED` (3) | Already initialized |
| **Parameter** | `ADC_ERR_INVALID_PARAMETER` (4) | Invalid parameter |
| | `ADC_ERR_NULL_POINTER` (5) | Null pointer |
| **Channel** | `ADC_ERR_CHANNEL_NOT_FOUND` (7) | Channel not found |
| | `ADC_ERR_CHANNEL_NOT_ENABLED` (8) | Channel not enabled |
| | `ADC_ERR_CHANNEL_READ_ERR` (11) | Channel read error |
| | `ADC_ERR_INVALID_CHANNEL` (13) | Invalid channel |
| | `ADC_ERR_CHANNEL_BUSY` (14) | Channel busy |
| **Sampling** | `ADC_ERR_SAMPLE_TIMEOUT` (16) | Sample timeout |
| | `ADC_ERR_VOLTAGE_OUT_OF_RANGE` (23) | Voltage out of range |
| **Hardware** | `ADC_ERR_HARDWARE_FAULT` (19) | Hardware fault |
| | `ADC_ERR_COMMUNICATION_FAILURE` (20) | Communication failure |
| | `ADC_ERR_DEVICE_NOT_RESPONDING` (21) | Device not responding |
| **Calibration** | `ADC_ERR_CALIBRATION_FAILURE` (22) | Calibration failure |
| | `ADC_ERR_CALIBRATION_NOT_FOUND` (28) | Calibration data not found |
| **System** | `ADC_ERR_SYSTEM_ERROR` (40) | System error |
| | `ADC_ERR_TIMEOUT` (45) | Operation timeout |
| | `ADC_ERR_UNKNOWN` (57) | Unknown error |

> Full enum: see `hf_adc_err_t` in `BaseAdc.h` (58 values covering channel, calibration, DMA, and system errors).

## 🔗 Integration

### With Motor Controller

```cpp
#include "component-handlers/AdcManager.h"

void motor_adc_integration() {
    auto& adc = AdcManager::GetInstance();
    auto& motor = MotorController::GetInstance();
    
    adc.Initialize();
    motor.EnsureInitialized();
    
    // Monitor motor parameters via ADC
    while (true) {
        // Read motor current from TMC9660 ADC
        float motor_current;
        if (adc.ReadVoltage("TMC9660_CURRENT_I0", motor_current) == HF_ADC_SUCCESS) {
            // Check for overcurrent condition
            if (motor_current > 5.0f) {
                logger.Info("ADC", "Overcurrent detected: %.2fA\n", motor_current);
                // Disable motor or reduce current
            }
        }
        
        // Read supply voltage
        float supply_voltage;
        if (adc.ReadVoltage("ESP32_ADC1_CH0", supply_voltage) == HF_ADC_SUCCESS) {
            // Check for undervoltage condition
            if (supply_voltage < 10.0f) {
                logger.Info("ADC", "Undervoltage detected: %.2fV\n", supply_voltage);
                // Implement protection measures
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## 📚 See Also

- **[GpioManager Documentation](GPIO_MANAGER_README.md)** - GPIO management system
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[MotorController Documentation](MOTOR_CONTROLLER_README.md)** - Motor control system
- **[TMC9660 Handler Documentation](../driver-handlers/TMC9660_HANDLER_README.md)** - TMC9660 driver with ADC

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*