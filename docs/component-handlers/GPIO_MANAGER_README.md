# GpioManager - Advanced GPIO Management System

<div align="center">

![Component](https://img.shields.io/badge/component-GpioManager-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Hardware](https://img.shields.io/badge/hardware-ESP32--C6%20|%20PCAL95555%20|%20TMC9660-orange.svg)

**Comprehensive GPIO management system for the HardFOC platform**

</div>

## 📋 Overview

The `GpioManager` is a singleton component handler that provides unified, thread-safe access to GPIO pins across multiple hardware sources. It integrates with the platform mapping system to automatically manage GPIOs from ESP32-C6, PCAL95555 GPIO expanders, and TMC9660 motor controllers through a single, consistent API using string-based pin identification.

### ✨ Key Features

- **🔗 Multi-Source GPIO Management**: ESP32-C6, PCAL95555, TMC9660
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **📍 String-Based Pin Identification**: Flexible, extensible pin naming
- **🛡️ Platform Mapping Integration**: Automatic hardware discovery
- **📊 Advanced Diagnostics**: Real-time health monitoring
- **⚡ Batch Operations**: Optimized multi-pin operations
- **🔔 Complete Interrupt Support**: Edge-triggered callbacks
- **🏥 Health Monitoring**: Per-chip and per-pin statistics

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        GpioManager                             │
├─────────────────────────────────────────────────────────────────┤
│  String-Based API    │ pin_name → hardware mapping            │
├─────────────────────────────────────────────────────────────────┤
│  Platform Integration│ Automatic pin discovery & registration │
├─────────────────────────────────────────────────────────────────┤
│  Hardware Handlers   │ ESP32, PCAL95555, TMC9660 handlers     │
├─────────────────────────────────────────────────────────────────┤
│  BaseGpio Interface  │ Unified GPIO operations                 │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Usage

```cpp
#include "component-handlers/GpioManager.h"

void gpio_example() {
    // Get singleton instance
    auto& gpio = GpioManager::GetInstance();
    
    // Initialize the manager
    if (!gpio.EnsureInitialized()) {
        logger.Info("GPIO", "Failed to initialize GPIO manager\n");
        return;
    }
    
    // Configure pin as output
    gpio.SetDirection("ESP32_GPIO_2", HF_GPIO_DIRECTION_OUTPUT);
    
    // Set pin active
    gpio.SetActive("ESP32_GPIO_2");
    
    // Read pin state
    bool state;
    if (gpio.Read("ESP32_GPIO_2", state) == HF_GPIO_SUCCESS) {
        logger.Info("GPIO", "Pin state: %s\n", state ? "ACTIVE" : "INACTIVE");
    }
}
```

## 📖 API Reference

### Critical Enum Values and Meanings

#### **GPIO State Enums**
```cpp
// GPIO Logical States (independent of electrical polarity)
hf_gpio_state_t::HF_GPIO_STATE_INACTIVE = 0  // Logical inactive state
hf_gpio_state_t::HF_GPIO_STATE_ACTIVE = 1    // Logical active state

// GPIO Electrical Levels (actual voltage levels)
hf_gpio_level_t::HF_GPIO_LEVEL_LOW = 0   // Electrical low level (0V)
hf_gpio_level_t::HF_GPIO_LEVEL_HIGH = 1  // Electrical high level (VCC)

// GPIO Active State Polarity (which electrical level = active)
hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW = 0   // Active state is electrical low
hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH = 1  // Active state is electrical high
```

#### **GPIO Configuration Enums**
```cpp
// GPIO Direction/Mode
hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT = 0   // Pin configured as input
hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT = 1  // Pin configured as output

// GPIO Output Drive Mode
hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL = 0  // Push-pull (strong high/low)
hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_OPEN_DRAIN = 1 // Open-drain (strong low, high-Z high)

// GPIO Pull Resistor Configuration
hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING = 0  // No pull resistor (floating)
hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP = 1        // Internal pull-up resistor
hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN = 2      // Internal pull-down resistor
hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN = 3   // Both pull-up and pull-down
```

#### **GPIO Interrupt Trigger Enums**
```cpp
hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_NONE = 0         // No interrupt
hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE = 1  // Low to high edge
hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE = 2 // High to low edge
hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_BOTH_EDGES = 3   // Both edges
hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_LOW_LEVEL = 4    // Low level
hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_HIGH_LEVEL = 5   // High level
```

#### **Error Code Enums**
```cpp
// GPIO Success/Error Codes
hf_gpio_err_t::GPIO_SUCCESS = 0                    // Operation successful
hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER = 4      // Invalid parameter
hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT = 13        // Hardware fault
hf_gpio_err_t::GPIO_ERR_COMMUNICATION_FAILURE = 14 // Communication failure
```

### **🎯 How to Use These Enums Correctly**

#### **1. GPIO State vs Electrical Level Understanding**
```cpp
// IMPORTANT: Understand the difference between logical state and electrical level
// Logical state = what the application sees (ACTIVE/INACTIVE)
// Electrical level = actual voltage on the pin (HIGH/LOW)

// Example: Active-low LED (LED turns ON when pin is electrically LOW)
gpio.Set("LED_PIN", true);  // Sets logical state to ACTIVE (LED turns ON)
// If pin is configured as ACTIVE_LOW, this sets electrical level to LOW

// Example: Active-high relay (relay turns ON when pin is electrically HIGH)
gpio.Set("RELAY_PIN", true);  // Sets logical state to ACTIVE (relay turns ON)
// If pin is configured as ACTIVE_HIGH, this sets electrical level to HIGH
```

#### **2. Proper GPIO Configuration**
```cpp
// Configure pin as output with proper settings
gpio.SetDirection("GPIO_EXT_GPIO_CS_1", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
gpio.SetOutputMode("GPIO_EXT_GPIO_CS_1", hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);
gpio.SetPullMode("GPIO_EXT_GPIO_CS_1", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);

// Configure pin as input with pull-up
gpio.SetDirection("GPIO_PCAL_GPIO17", hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
gpio.SetPullMode("GPIO_PCAL_GPIO17", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
```

#### **3. Interrupt Configuration**
```cpp
// Configure interrupt on rising edge
gpio.ConfigureInterrupt("GPIO_PCAL_IMU_INT", 
                       hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE,
                       my_interrupt_callback);

// Enable the interrupt
gpio.EnableInterrupt("GPIO_PCAL_IMU_INT");
```

#### **4. Error Handling Best Practices**
```cpp
// Always check return codes
auto result = gpio.Set("GPIO_EXT_GPIO_CS_1", true);
if (result != hf_gpio_err_t::GPIO_SUCCESS) {
    logger.Error("GPIO", "Failed to set pin: %s", HfGpioErrToString(result));
    // Handle error appropriately
}
```

#### **5. Understanding Active State Polarity**
```cpp
// Active-Low Configuration (common for LEDs, buttons)
// Logical ACTIVE = Electrical LOW
// Logical INACTIVE = Electrical HIGH
// Use when: LED cathode connected to pin, button pulls pin low when pressed

// Active-High Configuration (common for relays, enable signals)
// Logical ACTIVE = Electrical HIGH  
// Logical INACTIVE = Electrical LOW
// Use when: LED anode connected to pin, relay needs high to activate
```

### Core Operations

#### Initialization
```cpp
class GpioManager {
public:
    // Singleton access
    static GpioManager& GetInstance() noexcept;
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool Shutdown() noexcept;
    bool IsInitialized() const noexcept;
};
```

#### GPIO Registration and Management
```cpp
// Pin registration
hf_gpio_err_t RegisterGpio(std::string_view name, std::shared_ptr<BaseGpio> gpio) noexcept;
std::shared_ptr<BaseGpio> Get(std::string_view name) noexcept;
bool Contains(std::string_view name) const noexcept;
size_t Size() const noexcept;
```

#### Basic GPIO Operations
```cpp
// Pin control
hf_gpio_err_t Set(std::string_view name, bool value) noexcept;
hf_gpio_err_t SetActive(std::string_view name) noexcept;
hf_gpio_err_t SetInactive(std::string_view name) noexcept;
hf_gpio_err_t Read(std::string_view name, bool& state) noexcept;
hf_gpio_err_t Toggle(std::string_view name) noexcept;
hf_gpio_err_t IsActive(std::string_view name, bool& active) noexcept;
```

#### Pin Configuration
```cpp
// Pin configuration
hf_gpio_err_t SetDirection(std::string_view name, hf_gpio_direction_t direction) noexcept;
hf_gpio_err_t SetPullMode(std::string_view name, hf_gpio_pull_mode_t pull_mode) noexcept;
hf_gpio_err_t SetOutputMode(std::string_view name, hf_gpio_output_mode_t output_mode) noexcept;

// Configuration queries
hf_gpio_err_t GetDirection(std::string_view name, hf_gpio_direction_t& direction) const noexcept;
hf_gpio_err_t GetPullMode(std::string_view name, hf_gpio_pull_mode_t& pull_mode) const noexcept;
hf_gpio_err_t GetOutputMode(std::string_view name, hf_gpio_output_mode_t& output_mode) const noexcept;
```

#### Interrupt Support
```cpp
// Interrupt configuration
hf_gpio_err_t ConfigureInterrupt(std::string_view name,
                                 hf_gpio_interrupt_trigger_t trigger,
                                 BaseGpio::InterruptCallback callback,
                                 void* user_data = nullptr) noexcept;
hf_gpio_err_t EnableInterrupt(std::string_view name) noexcept;
hf_gpio_err_t DisableInterrupt(std::string_view name) noexcept;
bool SupportsInterrupts(std::string_view name) const noexcept;
```

#### Batch Operations
```cpp
// Multi-pin operations
GpioBatchResult BatchWrite(const GpioBatchOperation& operation) noexcept;
GpioBatchResult BatchRead(const std::vector<std::string_view>& pin_names) noexcept;
GpioBatchResult SetMultipleActive(const std::vector<std::string_view>& pin_names) noexcept;
GpioBatchResult SetMultipleInactive(const std::vector<std::string_view>& pin_names) noexcept;
```

#### Statistics and Diagnostics
```cpp
// System diagnostics
hf_gpio_err_t GetSystemDiagnostics(GpioSystemDiagnostics& diagnostics) const noexcept;
hf_gpio_err_t GetStatistics(std::string_view name, BaseGpio::PinStatistics& statistics) const noexcept;
hf_gpio_err_t ResetStatistics(std::string_view name) noexcept;
void DumpStatistics() const noexcept;
```

### Data Structures

#### GpioInfo
```cpp
struct GpioInfo {
    std::string_view name;                      // Human-readable name
    std::shared_ptr<BaseGpio> gpio_driver;      // GPIO driver instance
    HfFunctionalGpioPin functional_pin;         // Functional pin identifier
    HfPinCategory category;                     // Pin category (CORE, COMM, GPIO, USER)
    HfGpioChipType hardware_chip;               // Hardware chip identifier
    uint8_t hardware_pin_id;                    // Hardware pin ID within the chip
    bool is_registered;                         // Registration status
    bool is_input;                              // Pin direction
    bool current_state;                         // Last known pin state
    uint32_t access_count;                      // Number of times accessed
    uint32_t error_count;                       // Number of errors encountered
    uint64_t last_access_time;                  // Timestamp of last access
};
```

#### GpioSystemDiagnostics
```cpp
struct GpioSystemDiagnostics {
    bool system_healthy;                           // Overall system health
    uint32_t total_pins_registered;                // Total pins registered
    uint32_t pins_by_chip[4];                      // Pins per chip
    uint32_t pins_by_category[4];                  // Pins per category
    uint32_t total_operations;                     // Total operations performed
    uint32_t successful_operations;                // Successful operations
    uint32_t failed_operations;                    // Failed operations
    uint32_t communication_errors;                 // Communication errors
    uint32_t hardware_errors;                      // Hardware errors
    uint64_t system_uptime_ms;                     // System uptime
    hf_gpio_err_t last_error;                      // Last error encountered
};
```

#### GpioBatchOperation
```cpp
struct GpioBatchOperation {
    std::vector<std::string_view> pin_names;    // Pin names to operate on
    std::vector<bool> states;                   // Desired states (for write operations)
    bool is_write_operation;                    // true for write, false for read
};
```

#### GpioBatchResult
```cpp
struct GpioBatchResult {
    std::vector<std::string_view> pin_names;    // Pin names operated on
    std::vector<bool> states;                   // Resulting states
    std::vector<hf_gpio_err_t> results;         // Individual operation results
    hf_gpio_err_t overall_result;               // Overall operation result
    
    bool AllSuccessful() const noexcept;        // Check if all operations were successful
};
```

## 🎯 Hardware Support

### Supported Hardware Sources

| Hardware | Pins Available | Features |
|----------|----------------|----------|
| **ESP32-C6** | 40+ GPIO pins | Native GPIO, interrupts, pull-up/down |
| **PCAL95555** | 32 GPIO pins (2×16) | I2C expander, interrupt support |
| **TMC9660** | 18 GPIO pins | Motor controller GPIOs, fault monitoring |

### Pin Naming Convention

```cpp
// ESP32-C6 pins
"ESP32_GPIO_0" to "ESP32_GPIO_21"    // Standard GPIO pins
"ESP32_GPIO_BOOT"                    // Boot button
"ESP32_GPIO_EN"                      // Enable pin

// PCAL95555 pins (up to 2 chips)
"PCAL95555_CHIP1_PIN_0" to "PCAL95555_CHIP1_PIN_15"  // First chip
"PCAL95555_CHIP2_PIN_0" to "PCAL95555_CHIP2_PIN_15"  // Second chip

// TMC9660 pins
"TMC9660_GPIO_0" to "TMC9660_GPIO_17"    // Motor controller GPIOs
"TMC9660_FAULT"                          // Fault indication pin
"TMC9660_ENABLE"                         // Motor enable pin
```

## 🔧 Configuration

### Platform Integration

The GpioManager automatically integrates with the platform mapping system:

```cpp
// Platform mapping integration
#include "hf_functional_pin_config_vortex_v1.hpp"

// Automatic pin discovery based on platform configuration
// Pins are registered automatically during initialization
```

### Error Handling

The GpioManager uses comprehensive error handling with specific error codes:

```cpp
// Common error codes
hf_gpio_err_t::HF_GPIO_SUCCESS              // Operation successful
hf_gpio_err_t::HF_GPIO_ERR_INVALID_PARAMETER // Invalid pin name or parameters
hf_gpio_err_t::HF_GPIO_ERR_NOT_INITIALIZED   // GPIO manager not initialized
hf_gpio_err_t::HF_GPIO_ERR_HARDWARE_FAULT    // Hardware communication error
hf_gpio_err_t::HF_GPIO_ERR_PERMISSION_DENIED // Pin access denied
hf_gpio_err_t::HF_GPIO_ERR_OUT_OF_MEMORY     // Memory allocation failed
```

## 📊 Examples

### Basic GPIO Control

```cpp
#include "component-handlers/GpioManager.h"
#include "utils-and-drivers/driver-handlers/Logger.h"

void basic_gpio_example() {
    auto& logger = Logger::GetInstance();
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Configure pins using correct pin names
    gpio.SetDirection("GPIO_EXT_GPIO_CS_1", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    gpio.SetDirection("GPIO_PCAL_IMU_INT", hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    gpio.SetPullMode("GPIO_PCAL_IMU_INT", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
    
    // Control LED based on button
    while (true) {
        bool button_state;
        if (gpio.Read("GPIO_PCAL_IMU_INT", button_state) == hf_gpio_err_t::GPIO_SUCCESS) {
            gpio.Set("GPIO_EXT_GPIO_CS_1", !button_state);  // Active low button
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

### Multi-Pin Operations

```cpp
void multi_pin_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Configure multiple GPIO pins using correct pin names from pin config
    std::vector<std::string_view> gpio_pins = {
        "GPIO_EXT_GPIO_CS_1", "GPIO_EXT_GPIO_CS_2", 
        "GPIO_PCAL_DRV_EN", "GPIO_PCAL_RST_CTRL"
    };
    
    for (const auto& pin : gpio_pins) {
        gpio.SetDirection(pin, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    }
    
    // Create patterns
    std::vector<bool> pattern1 = {true, false, true, false};
    std::vector<bool> pattern2 = {false, true, false, true};
    
    // Alternate patterns using batch operations
    for (int i = 0; i < 10; i++) {
        GpioBatchOperation op1(gpio_pins, pattern1);
        auto result1 = gpio.BatchWrite(op1);
        
        vTaskDelay(pdMS_TO_TICKS(500));
        
        GpioBatchOperation op2(gpio_pins, pattern2);
        auto result2 = gpio.BatchWrite(op2);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

### GPIO Interrupts

```cpp
void interrupt_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Configure interrupt pin using correct pin name
    gpio.SetDirection("GPIO_PCAL_IMU_INT", hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    gpio.SetPullMode("GPIO_PCAL_IMU_INT", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
    
    // Set interrupt callback
    auto callback = [](std::string_view pin_name, bool state, void* user_data) {
        auto& logger = Logger::GetInstance();
        logger.Info("GPIO_INT", "Interrupt on %.*s: %s", 
                   static_cast<int>(pin_name.size()), pin_name.data(),
                   state ? "ACTIVE" : "INACTIVE");
    };
    
    gpio.ConfigureInterrupt("GPIO_PCAL_IMU_INT", hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_BOTH_EDGES, callback);
    gpio.EnableInterrupt("GPIO_PCAL_IMU_INT");
    
    logger.Info("GPIO", "Interrupt configured. Press button to trigger...");
    
    // Main loop
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### System Diagnostics

```cpp
void diagnostics_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Get system status
    GpioSystemDiagnostics diagnostics;
    if (gpio.GetSystemDiagnostics(diagnostics) == hf_gpio_err_t::GPIO_SUCCESS) {
        logger.Info("GPIO", "GPIO System Status:");
        logger.Info("GPIO", "  Overall healthy: %s", diagnostics.system_healthy ? "YES" : "NO");
        logger.Info("GPIO", "  Total pins: %u", diagnostics.total_pins_registered);
        logger.Info("GPIO", "  Total operations: %u", diagnostics.total_operations);
        logger.Info("GPIO", "  Successful operations: %u", diagnostics.successful_operations);
        logger.Info("GPIO", "  Failed operations: %u", diagnostics.failed_operations);
        logger.Info("GPIO", "  Communication errors: %u", diagnostics.communication_errors);
        logger.Info("GPIO", "  Hardware errors: %u", diagnostics.hardware_errors);
        logger.Info("GPIO", "  System uptime: %llu ms", diagnostics.system_uptime_ms);
    }
    
    // Get pin statistics
    BaseGpio::PinStatistics stats;
    if (gpio.GetStatistics("GPIO_EXT_GPIO_CS_1", stats) == hf_gpio_err_t::GPIO_SUCCESS) {
        logger.Info("GPIO", "Pin GPIO_EXT_GPIO_CS_1 statistics:");
        logger.Info("GPIO", "  Access count: %u", stats.access_count);
        logger.Info("GPIO", "  Error count: %u", stats.error_count);
        logger.Info("GPIO", "  Last access time: %llu", stats.last_access_time);
    }
    
    // Dump all statistics
    gpio.DumpStatistics();
}
```

## 🔍 Advanced Usage

### ⚡ Performance Optimization Guide

The GPIO Manager provides two distinct access patterns optimized for different performance requirements:

#### 🔍 String-Based API (Convenience & Extensibility)
Best for configuration, initialization, debugging, and user interfaces:

```cpp
void configuration_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // String-based API - Great for convenience and readability
    gpio.SetPin("GPIO_EXT_GPIO_CS_1", true);     // ~100-500ns per call
    gpio.ConfigurePin("GPIO_PCAL_GPIO17", false); // Includes hash map lookup
    
    bool state = gpio.GetPin("GPIO_EXT_GPIO_CS_1");
    logger.Info("GPIO", "Pin state: %s\n", state ? "HIGH" : "LOW");
}
```

#### ⚡ Cached Access (High Performance)
For real-time control loops and high-frequency operations (>1kHz):

```cpp
void high_performance_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // STEP 1: Cache BaseGpio pointers for fast access
    auto gpio_cs1 = gpio.Get("GPIO_EXT_GPIO_CS_1");      // Get shared_ptr
    auto gpio_cs2 = gpio.Get("GPIO_EXT_GPIO_CS_2");
    auto gpio_pcal17 = gpio.Get("GPIO_PCAL_GPIO17");
    
    // STEP 2: Validate cached pointers (once, outside loops)
    if (!gpio_cs1 || !gpio_cs2 || !gpio_pcal17) {
        logger.Info("GPIO", "ERROR: Failed to cache GPIO pointers\n");
        return;
    }
    
    // STEP 3: Use cached pointers for direct hardware access
    for (int i = 0; i < 100000; i++) {
        // Direct BaseGpio access - ~10-50ns per call
        gpio_cs1->SetActive();        // No string lookup overhead
        gpio_cs2->SetInactive();      // Direct hardware access
        gpio_pcal17->Toggle();        // Maximum performance
        
        // Read operations are also much faster
        bool state;
        if (gpio_cs1->Read(state) == hf_gpio_err_t::GPIO_SUCCESS) {
            // Process state immediately
        }
    }
}
```

#### 📊 Performance Comparison

| Operation | String Lookup | Cached Access | Speedup |
|-----------|---------------|---------------|---------|
| `SetPin()` | ~100-500ns | ~10-50ns | **5-10x faster** |
| `GetPin()` | ~150-600ns | ~15-60ns | **5-10x faster** |
| `Toggle()` | ~200-700ns | ~20-70ns | **5-10x faster** |

#### 🎯 When to Use Each Approach

**Use String-Based API for:**
- ✅ Application configuration and initialization
- ✅ User interfaces and debugging tools
- ✅ One-time setup operations
- ✅ Error handling and diagnostics
- ✅ Dynamic pin registration from configuration files

**Use Cached Access for:**
- ⚡ Real-time control loops running >1kHz
- ⚡ Motor control PWM generation
- ⚡ High-frequency sensor polling
- ⚡ Time-critical interrupt handlers
- ⚡ Communication protocol bit-banging

#### 🔄 Batch Operations for Maximum Efficiency

For operating on multiple pins simultaneously:

```cpp
void batch_operations_example() {
    auto& gpio = GpioManager::GetInstance();
    gpio.EnsureInitialized();
    
    // Batch operations - Most efficient for multiple pins
    std::vector<std::string_view> pins = {
        "GPIO_EXT_GPIO_CS_1", "GPIO_EXT_GPIO_CS_2", 
        "GPIO_PCAL_GPIO17", "GPIO_PCAL_GPIO18"
    };
    
    std::vector<bool> states = {true, false, true, false};
    
    // Single batch call - More efficient than individual operations
    GpioBatchOperation op(pins, states);
    auto result = gpio.BatchWrite(op);
    
    if (result.AllSuccessful()) {
        logger.Info("GPIO", "All pins set successfully\n");
    }
    
    // Batch read is also more efficient
    auto read_result = gpio.BatchRead(pins);
    for (size_t i = 0; i < pins.size(); i++) {
        if (read_result.results[i] == hf_gpio_err_t::GPIO_SUCCESS) {
            logger.Info("GPIO", "Pin %.*s: %s\n", 
                   static_cast<int>(pins[i].size()), pins[i].data(),
                   read_result.states[i] ? "ACTIVE" : "INACTIVE");
        }
    }
}
```

#### ⚠️ Important Performance Notes

1. **Cache Validation**: Always validate cached pointers before use
2. **Memory Management**: Cached `shared_ptr` objects automatically manage lifetime
3. **Thread Safety**: Both string and cached access are thread-safe
4. **Pin Registration**: String lookups enable dynamic pin registration for extensibility
5. **Hardware Abstraction**: Cached access still provides full hardware abstraction

## 🚨 Error Handling

### Common Error Scenarios

```cpp
void error_handling_example() {
    auto& gpio = GpioManager::GetInstance();
    
    // Check initialization
    if (!gpio.EnsureInitialized()) {
        logger.Info("GPIO", "ERROR: Failed to initialize GPIO manager\n");
        return;
    }
    
    // Validate pin exists before use
    if (!gpio.Contains("ESP32_GPIO_2")) {
        logger.Info("GPIO", "ERROR: Pin ESP32_GPIO_2 not registered\n");
        return;
    }
    
    // Safe pin operations with error checking
    auto result = gpio.SetActive("ESP32_GPIO_2");
    if (result != HF_GPIO_SUCCESS) {
        logger.Info("GPIO", "ERROR: Failed to set pin ESP32_GPIO_2: %d\n", static_cast<int>(result));
    }
    
    // Monitor system health
    GpioSystemDiagnostics diagnostics;
    if (gpio.GetSystemDiagnostics(diagnostics) == HF_GPIO_SUCCESS) {
        if (!diagnostics.system_healthy) {
            logger.Info("GPIO", "WARNING: GPIO system health check failed\n");
            logger.Info("GPIO", "Total errors: %u\n", diagnostics.failed_operations);
        }
    }
}
```

### `hf_gpio_err_t` Error Code Reference

| Category | Codes | Description |
|----------|-------|-------------|
| **General** | `GPIO_SUCCESS` (0) | Operation completed successfully |
| | `GPIO_ERR_FAILURE` (1) | General failure |
| | `GPIO_ERR_NOT_INITIALIZED` (2) | Not initialized |
| **Parameter** | `GPIO_ERR_INVALID_PARAMETER` (4) | Invalid parameter |
| | `GPIO_ERR_NULL_POINTER` (5) | Null pointer |
| **Pin** | `GPIO_ERR_INVALID_PIN` (7) | Invalid pin |
| | `GPIO_ERR_PIN_NOT_FOUND` (8) | Pin not found |
| | `GPIO_ERR_PIN_NOT_CONFIGURED` (9) | Pin not configured |
| | `GPIO_ERR_PIN_BUSY` (12) | Pin busy |
| | `GPIO_ERR_DIRECTION_MISMATCH` (24) | Direction mismatch |
| **Hardware** | `GPIO_ERR_HARDWARE_FAULT` (13) | Hardware fault |
| | `GPIO_ERR_READ_FAILURE` (22) | Read failure |
| | `GPIO_ERR_WRITE_FAILURE` (23) | Write failure |
| **Interrupt** | `GPIO_ERR_INTERRUPT_NOT_SUPPORTED` (26) | Interrupt not supported |
| | `GPIO_ERR_INTERRUPT_HANDLER_FAILED` (29) | Interrupt handler failed |
| **System** | `GPIO_ERR_TIMEOUT` (16) | Timeout |
| | `GPIO_ERR_DRIVER_ERROR` (34) | Driver error |
| | `GPIO_ERR_UNKNOWN` (38) | Unknown error |

> Full enum: see `hf_gpio_err_t` in `BaseGpio.h` (39 values covering pin, interrupt, and system errors).

## 🔗 Integration

### With Other Managers

```cpp
#include "component-handlers/GpioManager.h"

void integrated_example() {
    // Initialize all managers
    auto& gpio = GpioManager::GetInstance();
    auto& adc = AdcManager::GetInstance();
    auto& motor = MotorController::GetInstance();
    
    gpio.EnsureInitialized();
    adc.Initialize();
    motor.EnsureInitialized();
    
    // Use GPIO to control motor enable
    gpio.SetDirection("TMC9660_ENABLE", HF_GPIO_DIRECTION_OUTPUT);
    gpio.SetActive("TMC9660_ENABLE");
    
    // Monitor motor fault via GPIO
    gpio.SetDirection("TMC9660_FAULT", HF_GPIO_DIRECTION_INPUT);
    auto fault_callback = [&motor](std::string_view pin, bool state, void* user_data) {
        if (!state) {  // Fault is active low
            logger.Info("GPIO", "Motor fault detected!\n");
            // Handle fault condition
        }
    };
    gpio.ConfigureInterrupt("TMC9660_FAULT", HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE, fault_callback);
    gpio.EnableInterrupt("TMC9660_FAULT");
}
```

## 📚 See Also

- **[AdcManager Documentation](ADC_MANAGER_README.md)** - ADC management system
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[MotorController Documentation](MOTOR_CONTROLLER_README.md)** - Motor control system
- **[TMC9660 Handler Documentation](../driver-handlers/TMC9660_HANDLER_README.md)** - TMC9660 driver
- **[PCAL95555 Handler Documentation](../driver-handlers/PCAL95555_HANDLER_README.md)** - GPIO expander driver

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*