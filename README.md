# HardFOC Hardware Abstraction Layer (HAL) - Vortex V1

<div align="center">

![Version](https://img.shields.io/badge/version-2.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32--C6-green.svg)
![Board](https://img.shields.io/badge/Board-HardFOC%20Vortex%20V1-orange.svg)
![License](https://img.shields.io/badge/license-GPL-yellow.svg)
![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)
![Vortex API](https://img.shields.io/badge/Vortex%20API-Unified%20Interface-brightgreen.svg)

**Advanced hardware abstraction layer with unified Vortex API for the HardFOC Vortex V1 motor controller board**

[🚀 Quick Start](#quick-start) • [🔌 Vortex API](#vortex-api) • [📚 Documentation](#documentation) • [🏗️ Architecture](#architecture) • [🧪 Examples](#examples)

</div>

## 🎯 Overview

The HardFOC HAL provides a comprehensive, thread-safe hardware abstraction layer specifically designed for the **HardFOC Vortex V1** motor controller board. The system features the **Vortex API** - a unified singleton interface that provides access to all component handlers including GPIO, ADC, communication interfaces, motor controllers, IMU sensors, encoders, LED management, and temperature monitoring.

The **Vortex API** is named after the **HardFOC Vortex V1** board, providing a unified interface that mirrors the board's integrated design philosophy where all components work together.

### ✨ Key Features

- **🔌 Vortex API**: Unified singleton interface to all system components
- **🏗️ HardFOC Vortex V1 Optimized**: Specifically designed for the Vortex V1 board
- **🔧 Unified Hardware Management**: Single API for GPIO, ADC, SPI, I2C, UART, and CAN
- **🎛️ Multi-Source Support**: ESP32-C6, PCAL95555, TMC9660, BNO08x, AS5047U, NTC thermistors, WS2812 LEDs integrated
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **📊 Advanced Diagnostics**: Real-time health monitoring and error tracking
- **⚡ High Performance**: Optimized batch operations and interrupt handling
- **🛡️ Safety Features**: Pin validation, conflict detection, and fault recovery
- **🔌 Extensible Design**: Easy integration of new hardware components
- **🚀 Lazy Initialization**: Components initialized only when needed with proper dependency management

## 🔌 Vortex API - Unified System Interface

The **Vortex API** is the heart of the HardFOC Vortex V1 system, providing a unified interface to all component handlers. Named after the **HardFOC Vortex V1** board, it implements lazy initialization with proper dependency management, ensuring all systems are properly initialized in the correct order.

The API design mirrors the **Vortex V1** board's philosophy of integrated operation where all components work together as a unified system rather than separate modules.

### 🏗️ Vortex API Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    HardFOC Vortex V1 Board                     │
├─────────────────────────────────────────────────────────────────┤
│                        Vortex API                              │
├─────────────────────────────────────────────────────────────────┤
│  Component Handlers  │ Managers: GPIO, ADC, Comm, IMU, Motor  │
├─────────────────────────────────────────────────────────────────┤
│  Driver Handlers     │ TMC9660, PCAL95555, AS5047U, BNO08x    │
├─────────────────────────────────────────────────────────────────┤
│  Hardware Drivers    │ ESP32 interfaces & external drivers    │
└─────────────────────────────────────────────────────────────────┘
```

### 🚀 Quick Start with Vortex API

```cpp
#include "api/Vortex.h"

extern "C" void app_main(void) {
    // Get the unified Vortex API instance for HardFOC Vortex V1
    auto& vortex = Vortex::GetInstance();
    
    // Initialize all systems with proper dependency management
    if (vortex.EnsureInitialized()) {
        // Access all component handlers through unified interface
        auto& comms = vortex.comms;      // Communication channels
        auto& gpio = vortex.gpio;        // GPIO management
        auto& motors = vortex.motors;    // Motor controllers
        auto& adc = vortex.adc;          // ADC management
        auto& imu = vortex.imu;          // IMU sensors
        auto& encoders = vortex.encoders; // Encoders
        auto& leds = vortex.leds;        // LED management
        auto& temp = vortex.temp;        // Temperature sensors
        
        // Use any component naturally
        gpio.Set("GPIO_EXT_GPIO_CS_1", true);
        float voltage;
        adc.ReadChannelV("ADC_TMC9660_AIN3", voltage);
        auto* motor_handler = motors.handler(0);
        leds.SetStatus(LedAnimation::STATUS_OK);
        
        // Get system diagnostics
        VortexSystemDiagnostics diagnostics;
        vortex.GetSystemDiagnostics(diagnostics);
        logger.Info("MAIN", "HardFOC Vortex V1 System Health: %s\n", 
               diagnostics.system_healthy ? "HEALTHY" : "UNHEALTHY");
    }
}
```

### 🔗 Component Access Through Vortex API

The Vortex API provides direct access to all component handlers on the HardFOC Vortex V1 board:

| Component | Access | Purpose | Hardware Sources |
|-----------|--------|---------|------------------|
| **Communication** | `vortex.comms` | SPI, I2C, UART, CAN interfaces | ESP32-C6 |
| **GPIO Management** | `vortex.gpio` | Pin control and configuration | ESP32-C6, PCAL95555, TMC9660 |
| **Motor Controllers** | `vortex.motors` | Motor control and management | TMC9660 devices |
| **ADC Management** | `vortex.adc` | Analog-to-digital conversion | TMC9660 |
| **IMU Sensors** | `vortex.imu` | Motion and orientation sensing | BNO08x via I2C |
| **Encoders** | `vortex.encoders` | Position and velocity sensing | AS5047U via SPI |
| **LED Management** | `vortex.leds` | Status indication and animation | WS2812 LED |
| **Temperature** | `vortex.temp` | Temperature monitoring | TMC9660, NTC |

### 📊 System Diagnostics and Health Monitoring

```cpp
// Get comprehensive system diagnostics for HardFOC Vortex V1
VortexSystemDiagnostics diagnostics;
if (vortex.GetSystemDiagnostics(diagnostics)) {
    logger.Info("MAIN", "HardFOC Vortex V1 Overall Health: %s\n", diagnostics.system_healthy ? "HEALTHY" : "UNHEALTHY");
    logger.Info("MAIN", "Initialized Components: %u/%u\n", 
           diagnostics.initialized_components, diagnostics.total_components);
    logger.Info("MAIN", "Initialization Time: %llu ms\n", diagnostics.initialization_time_ms);
    logger.Info("MAIN", "System Uptime: %llu ms\n", diagnostics.system_uptime_ms);
}

// Perform health check
if (vortex.PerformHealthCheck()) {
    logger.Info("MAIN", "All HardFOC Vortex V1 systems operational\n");
}

// Get failed components
auto failed_components = vortex.GetFailedComponents();
for (const auto& component : failed_components) {
    logger.Info("MAIN", "Failed: %s\n", component.c_str());
}

// Dump comprehensive statistics
vortex.DumpSystemStatistics();
```

### 🔄 Initialization Order and Dependencies

The Vortex API manages initialization in the correct dependency order for the HardFOC Vortex V1 board:

```
1. CommChannelsManager (foundation - SPI, I2C, UART, CAN)
   ↓
2. GpioManager (depends on CommChannelsManager)
   ↓
3. MotorController (depends on CommChannelsManager)
   ↓
4. AdcManager (depends on MotorController)
   ↓
5. ImuManager (depends on CommChannelsManager, GpioManager)
   ↓
6. EncoderManager (depends on CommChannelsManager, GpioManager)
   ↓
7. LedManager (independent)
   ↓
8. TemperatureManager (depends on AdcManager, MotorController)
```

## ⚡ Performance Considerations

### String Lookups vs Cached Access

The HardFOC Vortex V1 API provides two access patterns optimized for different use cases:

#### 🔍 String-Based API (Convenience & Extensibility)
String-based functions like `gpio.Set("GPIO_EXT_GPIO_CS_1", true)` and `adc.ReadChannelV("ADC_TMC9660_AIN3", voltage)` provide:
- **Convenience**: Easy to use and understand
- **Extensibility**: Dynamic pin registration and configuration
- **Higher-level abstraction**: Perfect for application logic and configuration

```cpp
// String-based API - Great for convenience and extensibility
if (gpio.Set("GPIO_EXT_GPIO_CS_1", true) == hf_gpio_err_t::GPIO_SUCCESS) {
    // GPIO set successfully
}
float voltage;
if (adc.ReadChannelV("ADC_TMC9660_AIN3", voltage) == hf_adc_err_t::ADC_SUCCESS) {
    // ADC reading successful
}
```

#### ⚡ Cached Access (High Performance)
For performance-critical applications, cache base component pointers for direct access:

```cpp
// Cache GPIO and ADC pointers for fast access
auto gpio_cs1 = vortex.gpio.Get("GPIO_EXT_GPIO_CS_1");
auto* adc_temp = vortex.adc.Get("ADC_TMC9660_AIN3");

// Direct access - Much faster for tight loops
if (gpio_cs1) {
    gpio_cs1->SetActive();  // Direct BaseGpio access
}
if (adc_temp) {
    float voltage;
    adc_temp->ReadChannelV(0, voltage);  // Direct BaseAdc access
}
```

#### 📊 Performance Impact
- **String lookups**: ~100-500ns overhead per call (hash map lookup)
- **Cached access**: ~10-50ns (direct pointer access)
- **Recommendation**: Use cached access for control loops >1kHz

#### 🛡️ Error Handling
All API methods return error codes that should be checked:
- **GPIO Operations**: Return `hf_gpio_err_t` enum values
- **ADC Operations**: Return `hf_adc_err_t` enum values
- **Success**: `GPIO_SUCCESS` / `ADC_SUCCESS`
- **Common Errors**: `GPIO_ERR_INVALID_PARAMETER`, `ADC_ERR_INVALID_CHANNEL`, etc.

#### 🎯 Best Practices
1. **Use string API** for: Configuration, initialization, debugging, user interfaces
2. **Use cached access** for: Real-time control loops, high-frequency operations (>1kHz)
3. **Batch operations** for: Multiple pins/channels accessed together
4. **Cache validation**: Always check pointer validity before use
5. **Error handling**: Always check return codes for all API calls
6. **Resource management**: Use RAII patterns and proper cleanup

## 🚀 Quick Start

### 1. Include the Vortex API

```cpp
#include "api/Vortex.h"  // Unified Vortex API for HardFOC Vortex V1
```

### 2. Initialize and Use the System

```cpp
#include "api/Vortex.h"

extern "C" void app_main(void) {
    // Get the unified Vortex API for HardFOC Vortex V1
    auto& vortex = Vortex::GetInstance();
    
    // Initialize all systems with proper dependency management
    if (vortex.EnsureInitialized()) {
        logger.Info("MAIN", "HardFOC Vortex V1 initialized successfully!\n");
        
        // Access any component through the unified interface
        auto& gpio = vortex.gpio;
        auto& adc = vortex.adc;
        auto& motors = vortex.motors;
        auto& leds = vortex.leds;
        
        // Use components naturally (string-based for convenience)
        if (gpio.Set("GPIO_EXT_GPIO_CS_1", true) == hf_gpio_err_t::GPIO_SUCCESS) {
            logger.Info("MAIN", "GPIO set successfully\n");
        }
        
        float voltage;
        if (adc.ReadChannelV("ADC_TMC9660_AIN3", voltage) == hf_adc_err_t::ADC_SUCCESS) {
            logger.Info("MAIN", "ADC reading: %.3fV\n", voltage);
        }
        
        auto* motor_handler = motors.handler(0);
        leds.SetStatus(LedAnimation::STATUS_OK);
        
        // For performance-critical code, cache component pointers
        auto gpio_cs1 = gpio.Get("GPIO_EXT_GPIO_CS_1");
        auto* adc_temp = adc.Get("ADC_TMC9660_AIN3");
        
        // Use cached pointers for high-frequency operations
        if (gpio_cs1 && adc_temp) {
            for (int i = 0; i < 10000; i++) {
                // Direct BaseGpio/BaseAdc access - much faster than string lookups
                gpio_cs1->Toggle();
                float reading;
                adc_temp->ReadChannelV(0, reading);  // Note: requires channel ID
            }
        }
        
        // Main loop with health monitoring
        while (true) {
            // Check system health
            if (!vortex.PerformHealthCheck()) {
                logger.Info("MAIN", "HardFOC Vortex V1 system health check failed\n");
            }
            
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } else {
        logger.Info("MAIN", "HardFOC Vortex V1 initialization failed!\n");
        
        // Get detailed failure information
        auto failed_components = vortex.GetFailedComponents();
        for (const auto& component : failed_components) {
            logger.Info("MAIN", "Failed component: %s\n", component.c_str());
        }
    }
}
```

### 3. Advanced Usage Examples

#### GPIO Operations
```cpp
auto& gpio = vortex.gpio;

// Configure and use GPIO pins on HardFOC Vortex V1
gpio.SetDirection("GPIO_EXT_GPIO_CS_1", hf_gpio_direction_t::GPIO_DIRECTION_OUTPUT);  // Configure as output
gpio.Set("GPIO_EXT_GPIO_CS_1", true);         // Set pin high

// Read input pins from PCAL95555 expander
bool state;
gpio.Read("GPIO_PCAL_GPIO17", state);

// Batch operations for performance
std::array<std::string_view, 4> pins = {"GPIO_EXT_GPIO_CS_1", "GPIO_EXT_GPIO_CS_2", 
                                        "GPIO_PCAL_GPIO17", "GPIO_PCAL_GPIO18"};
auto states = gpio.BatchRead(pins);
```

#### ADC Operations
```cpp
auto& adc = vortex.adc;

// Read TMC9660 ADC channels
float voltage;
adc.ReadChannelV("ADC_TMC9660_AIN3", voltage);  // Temperature sensor

// Read multiple TMC9660 channels simultaneously
std::array<std::string_view, 4> channels = {
    "ADC_TMC9660_AIN0", "ADC_TMC9660_AIN1", 
    "ADC_TMC9660_AIN2", "ADC_TMC9660_AIN3"
};
auto readings = adc.BatchRead(channels);

// Read TMC9660 internal monitoring channels
float current;
adc.ReadChannelV("TMC9660_CURRENT_I0", current);
float temperature;
adc.ReadChannelV("TMC9660_CHIP_TEMPERATURE", temperature);
float motor_velocity;
adc.ReadChannelV("TMC9660_MOTOR_VELOCITY", motor_velocity);
```

#### Motor Control
```cpp
auto& motors = vortex.motors;

// Get motor controller handler for HardFOC Vortex V1
auto* handler = motors.handler(0);  // Onboard TMC9660
if (handler && handler->Initialize()) {
    auto tmc = handler->GetTmc9660Driver();
    
    // Configure motor parameters
    tmc->SetTargetVelocity(1000);  // RPM
    tmc->SetMaxCurrent(1000);      // mA
    
    // Enable motor
    tmc->EnableMotor(true);
}
```

#### IMU and Encoder Integration
```cpp
auto& imu = vortex.imu;
auto& encoders = vortex.encoders;

// Get IMU handler
auto* imu_handler = imu.GetBno08xHandler(0);
if (imu_handler) {
    // Configure IMU sensors
    imu_handler->EnableSensor(Bno08xSensorType::ROTATION_VECTOR, 50);
}

// Get encoder handler
auto* encoder_handler = encoders.GetAs5047uHandler(0);
if (encoder_handler) {
    // Read encoder angle
    uint16_t angle;
    if (encoders.ReadAngle(0, angle) == As5047uError::SUCCESS) {
        logger.Info("MAIN", "Encoder Angle: %u LSB\n", angle);
    }
}
```

## 📚 Documentation

### 📖 Complete Documentation Index
- **[📋 Documentation Index](DOCUMENTATION_INDEX.md)** - Complete guide to all documentation

### 🔌 Vortex API Documentation
- **[🔌 Vortex API Guide](lib/api/README.md)** - Comprehensive Vortex API documentation
- **[🚀 Vortex API Example](examples/VortexApiExample.cpp)** - Complete usage example
- **[🏗️ Vortex API Architecture](lib/api/Vortex.h)** - API design and implementation

### 🎯 Core System Documentation
- **[🔧 GPIO Manager](docs/component-handlers/GPIO_MANAGER_README.md)** - GPIO system guide
- **[📊 ADC Manager](docs/component-handlers/ADC_MANAGER_README.md)** - ADC system guide
- **[🏗️ Architecture Guidelines](docs/development/ARCHITECTURE_GUIDELINES.md)** - Core architecture

### 📚 Component Handler Documentation
- **[🎛️ GPIO Manager](docs/component-handlers/GPIO_MANAGER_README.md)** - GPIO management system
- **[📊 ADC Manager](docs/component-handlers/ADC_MANAGER_README.md)** - ADC management system
- **[📡 Communication Manager](docs/component-handlers/COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[🧭 IMU Manager](docs/component-handlers/IMU_MANAGER_README.md)** - IMU sensor management
- **[🎛️ Motor Controller](docs/component-handlers/MOTOR_CONTROLLER_README.md)** - Motor control system
- **[📐 Encoder Manager](docs/component-handlers/ENCODER_MANAGER_README.md)** - Encoder management
- **[💡 LED Manager](docs/component-handlers/LED_MANAGER_README.md)** - LED management
- **[🌡️ Temperature Manager](docs/component-handlers/TEMPERATURE_MANAGER_README.md)** - Temperature monitoring

### 🔧 Driver Handler Documentation
- **[🎛️ TMC9660 Handler](docs/driver-handlers/TMC9660_HANDLER_README.md)** - Motor controller driver
- **[🔌 PCAL95555 Handler](docs/driver-handlers/PCAL95555_HANDLER_README.md)** - GPIO expander driver
- **[📐 AS5047U Handler](docs/driver-handlers/AS5047U_HANDLER_README.md)** - Position encoder driver
- **[🧭 BNO08x Handler](docs/driver-handlers/BNO08X_HANDLER_README.md)** - IMU sensor driver

## 🏗️ Architecture

### HardFOC Vortex V1 Board Integration
The Vortex API is specifically designed for the **HardFOC Vortex V1** board, providing a unified interface that mirrors the board's integrated design:

```
┌─────────────────────────────────────────────────────────────────┐
│                    HardFOC Vortex V1 Board                     │
├─────────────────────────────────────────────────────────────────┤
│                        Vortex API                              │
├─────────────────────────────────────────────────────────────────┤
│  🎯 Singleton Access    │ Vortex::GetInstance()                │
├─────────────────────────────────────────────────────────────────┤
│  🔗 Component Access    │ vortex.comms, vortex.gpio, etc.      │
├─────────────────────────────────────────────────────────────────┤
│  📊 System Diagnostics  │ Health monitoring & error tracking   │
├─────────────────────────────────────────────────────────────────┤
│  ⚡ Performance Metrics │ Initialization time & uptime         │
└─────────────────────────────────────────────────────────────────┘
```

### Component Handlers (Managers)
High-level singleton managers providing unified interfaces for the HardFOC Vortex V1 board:

| Manager | Vortex Access | Purpose | Hardware Sources |
|---------|---------------|---------|------------------|
| **CommChannelsManager** | `vortex.comms` | Communication interfaces | ESP32-C6 SPI/I2C/UART/CAN |
| **GpioManager** | `vortex.gpio` | GPIO pin management | ESP32-C6, PCAL95555, TMC9660 |
| **AdcManager** | `vortex.adc` | ADC channel management | TMC9660 |
| **MotorController** | `vortex.motors` | Motor controller management | TMC9660 devices |
| **ImuManager** | `vortex.imu` | IMU sensor management | BNO08x via I2C |
| **EncoderManager** | `vortex.encoders` | Encoder management | AS5047U via SPI |
| **LedManager** | `vortex.leds` | LED management | WS2812 LED |
| **TemperatureManager** | `vortex.temp` | Temperature monitoring | TMC9660, NTC |

### Driver Handlers
Hardware-specific drivers providing device interfaces for the HardFOC Vortex V1 board:

| Handler | Device | Interface | Purpose |
|---------|--------|-----------|---------|
| **Tmc9660Handler** | TMC9660 | SPI/UART | Motor controller with GPIO/ADC |
| **Pcal95555Handler** | PCAL95555 | I2C | 16-bit GPIO expander |
| **As5047uHandler** | AS5047U | SPI | Magnetic position encoder |
| **Bno08xHandler** | BNO08x | I2C | 9-axis IMU sensor |

### Hardware Support Matrix for HardFOC Vortex V1

| Feature | ESP32-C6 | PCAL95555 | TMC9660 | AS5047U | BNO08x |
|---------|-----------|-----------|---------|---------|--------|
| **GPIO** | ✅ 2 pins | ✅ 12 pins | ✅ 8 pins | ❌ | ❌ |
| **ADC** | ❌ (disabled) | ❌ | ✅ 4 channels | ❌ | ❌ |
| **SPI** | ✅ Master | ❌ | ✅ Slave | ✅ Slave | ❌ |
| **I2C** | ✅ Master | ✅ Slave | ❌ | ❌ | ✅ Slave |
| **UART** | ✅ 1 port | ❌ | ✅ TMCL | ❌ | ❌ |
| **PWM** | ✅ 6 channels | ❌ | ✅ Motor PWM | ❌ | ❌ |
| **Interrupts** | ✅ GPIO | ✅ GPIO | ✅ Fault | ❌ | ✅ Data ready |

### Available GPIO Pins on HardFOC Vortex V1

#### ESP32-C6 GPIO Pins
- `GPIO_EXT_GPIO_CS_1` - External GPIO chip select 1
- `GPIO_EXT_GPIO_CS_2` - External GPIO chip select 2

#### PCAL95555 GPIO Expander Pins
- `GPIO_PCAL_GPIO17` - General purpose GPIO 17
- `GPIO_PCAL_GPIO18` - General purpose GPIO 18
- `GPIO_PCAL_FAULT_STATUS` - Fault status indicator
- `GPIO_PCAL_DRV_EN` - Driver enable control
- `GPIO_PCAL_RST_CTRL` - Reset control
- `GPIO_PCAL_PWR_GOOD` - Power good indicator
- `GPIO_PCAL_CAN_HS_STB_OP` - CAN high-speed standby operation
- `GPIO_PCAL_IMU_BOOT` - IMU boot control
- `GPIO_PCAL_IMU_INT` - IMU interrupt
- `GPIO_PCAL_IMU_RST` - IMU reset
- `GPIO_PCAL_SPI_COMM_EN` - SPI communication enable
- `GPIO_PCAL_WAKE_CTRL` - Wake control

### Available ADC Channels on HardFOC Vortex V1

#### TMC9660 ADC Channels
- `ADC_TMC9660_AIN0` - ADC Input 0 (Reserved)
- `ADC_TMC9660_AIN1` - ADC Input 1 (Reserved)
- `ADC_TMC9660_AIN2` - ADC Input 2 (Reserved)
- `ADC_TMC9660_AIN3` - ADC Input 3 (Temperature Sensor)

#### TMC9660 Internal Monitoring Channels
- `TMC9660_CURRENT_I0` - Current Sense I0
- `TMC9660_CURRENT_I1` - Current Sense I1
- `TMC9660_CURRENT_I2` - Current Sense I2
- `TMC9660_CURRENT_I3` - Current Sense I3
- `TMC9660_SUPPLY_VOLTAGE` - Supply Voltage
- `TMC9660_DRIVER_VOLTAGE` - Driver Voltage
- `TMC9660_CHIP_TEMPERATURE` - Chip Temperature
- `TMC9660_EXTERNAL_TEMPERATURE` - External Temperature
- `TMC9660_MOTOR_CURRENT` - Motor Current
- `TMC9660_MOTOR_VELOCITY` - Motor Velocity
- `TMC9660_MOTOR_POSITION` - Motor Position

## 🔧 API Reference

### Vortex API Core
```cpp
// Singleton access for HardFOC Vortex V1
auto& vortex = Vortex::GetInstance();

// Initialization
bool EnsureInitialized();
bool IsInitialized() const;

// Component access
CommChannelsManager& comms;
GpioManager& gpio;
MotorController& motors;
AdcManager& adc;
ImuManager& imu;
EncoderManager& encoders;
LedManager& leds;
TemperatureManager& temp;

// Diagnostics
bool GetSystemDiagnostics(VortexSystemDiagnostics& diagnostics);
bool PerformHealthCheck();
void DumpSystemStatistics();

// Utility
uint64_t GetSystemUptimeMs();
uint64_t GetInitializationTimeMs();
std::string GetSystemVersion();
```

### Component Handler Access
```cpp
// Through Vortex API (recommended for HardFOC Vortex V1)
auto& gpio = vortex.gpio;
auto& adc = vortex.adc;
auto& motors = vortex.motors;

// Direct access (legacy)
auto& gpio = GpioManager::GetInstance();
auto& adc = AdcManager::GetInstance();
auto& motors = MotorController::GetInstance();

// GPIO Operations
gpio.Set("GPIO_EXT_GPIO_CS_1", true);                    // Set pin high
bool state;
gpio.Read("GPIO_PCAL_GPIO17", state);                    // Read pin state
gpio.SetDirection("GPIO_EXT_GPIO_CS_1", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);

// ADC Operations
float voltage;
adc.ReadChannelV("ADC_TMC9660_AIN3", voltage);           // Read voltage
uint32_t raw_value;
adc.ReadChannelCount("ADC_TMC9660_AIN3", raw_value);     // Read raw count
```

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

// ADC Success/Error Codes
hf_adc_err_t::ADC_SUCCESS = 0                      // Operation successful
hf_adc_err_t::ADC_ERR_INVALID_CHANNEL = 13         // Invalid channel
hf_adc_err_t::ADC_ERR_HARDWARE_FAULT = 19          // Hardware fault
hf_adc_err_t::ADC_ERR_COMMUNICATION_FAILURE = 20   // Communication failure
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

// For ADC operations
float voltage;
auto adc_result = adc.ReadChannelV("ADC_TMC9660_AIN3", voltage);
if (adc_result != hf_adc_err_t::ADC_SUCCESS) {
    logger.Error("ADC", "Failed to read voltage: %s", HfAdcErrToString(adc_result));
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

## 🧪 Examples

### Complete Vortex API Example
```cpp
#include "api/Vortex.h"

extern "C" void app_main(void) {
    logger.Info("MAIN", "=== HardFOC Vortex V1 API Example ===\n");
    
    // Get the unified Vortex API for HardFOC Vortex V1
    auto& vortex = Vortex::GetInstance();
    
    // Initialize all systems
    if (vortex.EnsureInitialized()) {
        logger.Info("MAIN", "✓ HardFOC Vortex V1 initialized successfully!\n");
        
        // Demonstrate all components
        DemonstrateComms(vortex);
        DemonstrateGpio(vortex);
        DemonstrateMotors(vortex);
        DemonstrateAdc(vortex);
        DemonstrateImu(vortex);
        DemonstrateEncoders(vortex);
        DemonstrateLeds(vortex);
        DemonstrateTemperature(vortex);
        
        // System health monitoring
        logger.Info("MAIN", "=== HardFOC Vortex V1 System Health Check ===\n");
        if (vortex.PerformHealthCheck()) {
            logger.Info("MAIN", "✓ All HardFOC Vortex V1 systems operational\n");
        } else {
            logger.Info("MAIN", "✗ HardFOC Vortex V1 system health check failed\n");
        }
        
        // Continuous operation demo
        logger.Info("MAIN", "=== Continuous Operation Demo ===\n");
        for (int i = 0; i < 10; i++) {
            logger.Info("MAIN", "Tick %d/10 - Uptime: %llu ms\n", i + 1, vortex.GetSystemUptimeMs());
            
            // Blink LED to show activity
            if (i % 2 == 0) {
                vortex.leds.SetStatus(LedAnimation::STATUS_OK);
            } else {
                vortex.leds.SetStatus(LedAnimation::STATUS_WARN);
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        // Final status
        vortex.leds.SetStatus(LedAnimation::STATUS_OK);
        logger.Info("MAIN", "=== HardFOC Vortex V1 Demo Complete ===\n");
        
    } else {
        logger.Info("MAIN", "✗ HardFOC Vortex V1 initialization failed!\n");
        
        // Show what failed
        auto failed_components = vortex.GetFailedComponents();
        for (const auto& component : failed_components) {
            logger.Info("MAIN", "Failed component: %s\n", component.c_str());
        }
    }
}
```

### GPIO Control Example
```cpp
void DemonstrateGpio(Vortex& vortex) {
    logger.Info("MAIN", "=== HardFOC Vortex V1 GPIO Management Demo ===\n");
    
    auto& gpio = vortex.gpio;
    
    // Configure ESP32 GPIO pin as output
    gpio.ConfigurePin("GPIO_EXT_GPIO_CS_1", false);  // false = output
    
            // Blink LED using ESP32 GPIO
        for (int i = 0; i < 5; i++) {
            gpio.Set("GPIO_EXT_GPIO_CS_1", true);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio.Set("GPIO_EXT_GPIO_CS_1", false);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    
    // Read PCAL95555 GPIO pins
    bool gpio17_state;
    gpio.Read("GPIO_PCAL_GPIO17", gpio17_state);
    bool gpio18_state;
    gpio.Read("GPIO_PCAL_GPIO18", gpio18_state);
    logger.Info("MAIN", "PCAL95555 GPIO17: %s, GPIO18: %s\n", 
           gpio17_state ? "HIGH" : "LOW", 
           gpio18_state ? "HIGH" : "LOW");
    
    logger.Info("MAIN", "✓ HardFOC Vortex V1 GPIO demo completed\n");
}
```

### Multi-Channel ADC Reading
```cpp
void DemonstrateAdc(Vortex& vortex) {
    logger.Info("MAIN", "=== HardFOC Vortex V1 ADC Management Demo ===\n");
    
    auto& adc = vortex.adc;
    
    // Read TMC9660 ADC channels simultaneously
    std::array<std::string_view, 4> channels = {
        "ADC_TMC9660_AIN0", "ADC_TMC9660_AIN1", 
        "ADC_TMC9660_AIN2", "ADC_TMC9660_AIN3"
    };
    
    auto readings = adc.BatchRead(channels);
    
    for (size_t i = 0; i < channels.size(); i++) {
        logger.Info("MAIN", "  %s: %.3fV\n", channels[i].data(), readings.voltages[i]);
    }
    
    // Read TMC9660 internal monitoring channels
    float chip_temp;
    adc.ReadChannelV("TMC9660_CHIP_TEMPERATURE", chip_temp);
    float motor_current;
    adc.ReadChannelV("TMC9660_MOTOR_CURRENT", motor_current);
    float supply_voltage;
    adc.ReadChannelV("TMC9660_SUPPLY_VOLTAGE", supply_voltage);
    
    logger.Info("MAIN", "  TMC9660 Chip Temperature: %.3fV\n", chip_temp);
    logger.Info("MAIN", "  TMC9660 Motor Current: %.3fV\n", motor_current);
    logger.Info("MAIN", "  TMC9660 Supply Voltage: %.3fV\n", supply_voltage);
    
    logger.Info("MAIN", "✓ HardFOC Vortex V1 ADC demo completed\n");
}
```

### Motor Control Example
```cpp
void DemonstrateMotors(Vortex& vortex) {
    logger.Info("MAIN", "=== HardFOC Vortex V1 Motor Controller Demo ===\n");
    
    auto& motors = vortex.motors;
    
    // Get the onboard motor controller handler
    auto* handler = motors.handler(0);
    if (handler) {
        logger.Info("MAIN", "✓ HardFOC Vortex V1 onboard TMC9660 handler available\n");
        
        // Get the underlying driver
        auto driver = motors.driver(0);
        if (driver) {
            logger.Info("MAIN", "✓ HardFOC Vortex V1 TMC9660 driver available\n");
            
            // Configure motor parameters
            driver->SetTargetVelocity(1000);  // RPM
            driver->SetMaxCurrent(1000);      // mA
            
            // Enable motor
            driver->EnableMotor(true);
            
            logger.Info("MAIN", "✓ HardFOC Vortex V1 motor controller configured and enabled\n");
        }
    } else {
        logger.Info("MAIN", "✗ HardFOC Vortex V1 motor controller not available\n");
    }
}
```

## 🔗 Hardware Integration

### HardFOC Vortex V1 Board Configuration
- **[🏗️ Hardware Architecture](docs/development/ARCHITECTURE_GUIDELINES.md)** - Board-specific configuration and features

### Device Configuration for HardFOC Vortex V1
- **[🔌 PCAL95555 Handler](docs/driver-handlers/PCAL95555_HANDLER_README.md)** - GPIO expander documentation
- **[🎛️ TMC9660 Handler](docs/driver-handlers/TMC9660_HANDLER_README.md)** - Motor controller documentation
- **[📐 AS5047U Handler](docs/driver-handlers/AS5047U_HANDLER_README.md)** - Position encoder documentation
- **[🧭 BNO08x Handler](docs/driver-handlers/BNO08X_HANDLER_README.md)** - IMU sensor documentation

### Communication Interface
- **[📡 Communication Manager](docs/component-handlers/COMM_CHANNELS_MANAGER_README.md)** - SPI, I2C, UART, CAN communication

## 🧪 Testing

### Test Suite
```bash
# Build and run tests for HardFOC Vortex V1
cd tests
idf.py build
idf.py flash monitor
```

### Test Coverage
- **[🧪 Test Documentation](tests/README.md)** - Complete test suite guide
- **Vortex API Tests** - Unified API validation for HardFOC Vortex V1
- **Component Handler Tests** - Manager class validation
- **Driver Handler Tests** - Hardware driver validation  
- **Integration Tests** - System-level testing
- **Hardware-in-Loop Tests** - Real hardware validation

## 🤝 Contributing

### Development Guidelines
- **[📝 Coding Standards](docs/development/CODING_STANDARDS.md)**
- **[🏗️ Architecture Guidelines](docs/development/ARCHITECTURE_GUIDELINES.md)**
- **[🧪 Testing Requirements](docs/development/TESTING_REQUIREMENTS.md)**
- **[📚 Documentation Standards](docs/development/DOCUMENTATION_STANDARDS.md)**

### Getting Started
1. Fork the repository
2. Create a feature branch
3. Follow coding standards
4. Add comprehensive tests
5. Update documentation
6. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

- **📚 Documentation**: Start with [DOCUMENTATION_INDEX.md](DOCUMENTATION_INDEX.md)
- **🔌 Vortex API**: See [lib/api/README.md](lib/api/README.md) for unified API guide
- **🏗️ HardFOC Vortex V1**: See [Architecture Guidelines](docs/development/ARCHITECTURE_GUIDELINES.md) for board details
- **🐛 Issues**: Report bugs via GitHub Issues
- **💬 Discussions**: Use GitHub Discussions for questions
- **📧 Contact**: HardFOC Team

---

<div align="center">

**Built with ❤️ by the HardFOC Team**

[⭐ Star us on GitHub](https://github.com/hardfoc/hf-hal-vortex-v1) • [🐛 Report Bug](https://github.com/hardfoc/hf-hal-vortex-v1/issues) • [💡 Request Feature](https://github.com/hardfoc/hf-hal-vortex-v1/issues)

**The Vortex API provides a unified interface to the HardFOC Vortex V1 board!**

</div>
