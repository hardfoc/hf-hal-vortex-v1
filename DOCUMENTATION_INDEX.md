# HardFOC HAL Documentation Index

<div align="center">

![Documentation](https://img.shields.io/badge/documentation-complete-green.svg)
![Version](https://img.shields.io/badge/version-2.0-blue.svg)
![Status](https://img.shields.io/badge/status-up--to--date-brightgreen.svg)

**Comprehensive documentation guide for the HardFOC Hardware Abstraction Layer**

[🚀 Quick Start](#quick-start) • [📚 Component Handlers](#component-handlers) • [🔧 Driver Handlers](#driver-handlers) • [🏗️ Architecture](#architecture) • [🔌 API Reference](#api-reference)

</div>

## 📋 Overview

This documentation index provides comprehensive access to all HardFOC HAL system documentation. The system is organized into logical sections covering component handlers (managers), driver handlers, architecture guides, and API references.

## 🚀 Quick Start

### Essential Documentation
- **[🏠 Main README](README.md)** - Project overview and quick start guide
- **[⚡ Quick Start Examples](#quick-start-examples)** - Get up and running in minutes
- **[🔧 System Integration](lib/api/README.md)** - Integration with existing projects
- **[⚙️ Architecture Guidelines](docs/development/ARCHITECTURE_GUIDELINES.md)** - Hardware configuration guide

### Quick Start Examples

```cpp
// Minimal HardFOC initialization
#include "api/Vortex.h"

int main() {
    // Get the Vortex API instance
    auto& vortex = Vortex::GetInstance();
    
    // Initialize the complete system
    if (!vortex.EnsureInitialized()) {
        printf("System initialization failed\n");
        return -1;
    }
    
    // Use GPIO
    auto& gpio = GpioManager::GetInstance();
    gpio.SetPin("ESP32_GPIO_2", true);
    
    // Use ADC  
    auto& adc = AdcManager::GetInstance();
    float voltage = adc.ReadVoltage("ESP32_ADC1_CH0");
    
    // Main loop with health monitoring
    while (true) {
        // System runs continuously - Vortex API handles maintenance
        auto diagnostics = vortex.GetSystemDiagnostics();
        if (!diagnostics.system_healthy) {
            printf("System health check failed\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## 📚 Component Handlers

Component handlers provide high-level management interfaces for system resources. Each handler is a singleton that manages multiple hardware sources through a unified API.

### Core System Managers

| Component | Purpose | Hardware Sources | Documentation |
|-----------|---------|------------------|---------------|
| **🎛️ GpioManager** | GPIO pin management | ESP32, PCAL95555, TMC9660 | **[📖 GPIO Manager Guide](docs/component-handlers/GPIO_MANAGER_README.md)** |
| **📊 AdcManager** | ADC channel management | ESP32, TMC9660 | **[📖 ADC Manager Guide](docs/component-handlers/ADC_MANAGER_README.md)** |
| **📡 CommChannelsManager** | Communication interfaces | ESP32 SPI/I2C/UART/CAN | **[📖 Comm Manager Guide](docs/component-handlers/COMM_CHANNELS_MANAGER_README.md)** |
| **🎛️ MotorController** | Motor controller management | TMC9660 devices | **[📖 Motor Controller Guide](docs/component-handlers/MOTOR_CONTROLLER_README.md)** |
| **🧭 ImuManager** | IMU sensor management | BNO08x via I2C | **[📖 IMU Manager Guide](docs/component-handlers/IMU_MANAGER_README.md)** |
| **🌡️ TemperatureManager** | Temperature monitoring | NTC thermistors, integrated sensors | **[📖 Temperature Manager Guide](docs/component-handlers/TEMPERATURE_MANAGER_README.md)** |
| **💡 LedManager** | LED control system | WS2812 strips, individual LEDs | **[📖 LED Manager Guide](docs/component-handlers/LED_MANAGER_README.md)** |
| **🎯 EncoderManager** | Position encoder system | AS5047U, incremental encoders | **[📖 Encoder Manager Guide](docs/component-handlers/ENCODER_MANAGER_README.md)** |

### Component Handler Features

#### GpioManager - Advanced GPIO Management
- **Multi-Source Support**: ESP32 (40+ pins), PCAL95555 (32 pins), TMC9660 (8 pins)
- **String-Based API**: Flexible pin identification (`"ESP32_GPIO_2"`, `"PCAL95555_CHIP1_PIN_0"`)
- **Thread-Safe Operations**: Concurrent access from multiple tasks
- **Batch Operations**: Optimized multi-pin read/write operations
- **Interrupt Support**: Edge-triggered callbacks with safety features
- **Health Monitoring**: Per-chip and per-pin statistics

#### AdcManager - Unified ADC System
- **Multi-Source Support**: ESP32 (6 channels), TMC9660 (3 channels)
- **Calibration System**: Automatic voltage conversion with reference scaling
- **Batch Operations**: Simultaneous multi-channel readings
- **Filtering Support**: Hardware and software filtering options
- **Platform Integration**: Automatic channel discovery and registration

#### MotorController - Motor Management System  
- **Multi-Device Support**: Up to 4 TMC9660 controllers (1 onboard + 3 external)
- **Dynamic Device Management**: Runtime creation/deletion of external devices
- **Unified Interface**: Single API for all motor controllers
- **Handler Access**: Direct access to individual Tmc9660Handler instances
- **Communication Flexibility**: SPI and UART interface support

#### TemperatureManager - Temperature Monitoring System
- **Multi-Sensor Support**: NTC thermistors, integrated sensors, external devices
- **Unified API**: Single interface for all temperature measurements
- **Automatic Calibration**: Built-in calibration and compensation
- **Trend Analysis**: Temperature history and trend monitoring
- **Safety Monitoring**: Over-temperature protection and alerts

#### LedManager - LED Control System
- **WS2812 Support**: Full RGB LED strip control with RMT interface
- **Individual LED Control**: GPIO-based LED control
- **Color Management**: RGB, HSV, and temperature-based color control
- **Animation System**: Built-in animations and custom pattern support
- **Power Management**: Current limiting and thermal protection

#### EncoderManager - Position Encoder System
- **Multi-Encoder Support**: AS5047U, incremental, and custom encoders
- **Unified API**: Single interface for all position measurements
- **Real-time Tracking**: High-speed position and velocity monitoring
- **Automatic Calibration**: Built-in calibration and compensation
- **Position History**: Position tracking and trend analysis

## 🔧 Driver Handlers

Driver handlers provide hardware-specific interfaces for individual devices. Each handler encapsulates device communication, configuration, and operation.

### Hardware Device Drivers

| Driver | Device | Interface | Features | Documentation |
|--------|--------|-----------|----------|---------------|
| **🎛️ Tmc9660Handler** | TMC9660 Motor Controller | SPI/UART | Motor control, GPIO, ADC | **[📖 TMC9660 Handler Guide](docs/driver-handlers/TMC9660_HANDLER_README.md)** |
| **🔌 Pcal95555Handler** | PCAL95555 GPIO Expander | I2C | 16-bit GPIO expansion | **[📖 PCAL95555 Handler Guide](docs/driver-handlers/PCAL95555_HANDLER_README.md)** |
| **📐 As5047uHandler** | AS5047U Position Encoder | SPI | Magnetic angle sensing | **[📖 AS5047U Handler Guide](docs/driver-handlers/AS5047U_HANDLER_README.md)** |
| **🧭 Bno08xHandler** | BNO08x IMU Sensor | I2C | 9-axis motion sensing | **[📖 BNO08x Handler Guide](docs/driver-handlers/BNO08X_HANDLER_README.md)** |
| **📝 Logger** | Unified Logging System | UART/File/Network | Multi-destination logging | **[📖 Logger Handler Guide](docs/driver-handlers/LOGGER_HANDLER_README.md)** |

### Driver Handler Features

#### Tmc9660Handler - Complete Motor Control
- **Motor Types**: Stepper and BLDC motor support
- **Communication**: SPI and UART interfaces with automatic detection
- **GPIO Integration**: 8 configurable pins with BaseGpio compatibility
- **ADC Support**: 3 analog inputs with BaseAdc compatibility
- **Safety Features**: Fault detection, thermal protection, stallguard
- **Performance**: High-speed operation up to 10,000 RPM

#### Hardware Support Matrix

| Feature | ESP32 | PCAL95555 | TMC9660 | AS5047U | BNO08x | NTC Sensors | WS2812 LEDs | Logger |
|---------|-----------|-----------|---------|---------|--------|-------------|-------------|---------|
| **GPIO** | ✅ 40+ pins | ✅ 32 pins | ✅ 8 pins | ❌ | ❌ | ❌ | ✅ Individual LEDs | ❌ |
| **ADC** | ✅ 6 channels | ❌ | ✅ 3 channels | ❌ | ❌ | ✅ Thermistors | ❌ | ❌ |
| **SPI** | ✅ Master | ❌ | ✅ Slave | ✅ Slave | ❌ | ❌ | ❌ | ❌ |
| **I2C** | ✅ Master | ✅ Slave | ❌ | ❌ | ✅ Slave | ✅ External sensors | ❌ | ❌ |
| **UART** | ✅ 3 ports | ❌ | ✅ TMCL | ❌ | ❌ | ❌ | ❌ | ✅ Console output |
| **PWM** | ✅ 6 channels | ❌ | ✅ Motor PWM | ❌ | ❌ | ❌ | ❌ | ❌ |
| **RMT** | ✅ 8 channels | ❌ | ❌ | ❌ | ❌ | ❌ | ✅ LED strips | ❌ |
| **File System** | ✅ SPIFFS/LittleFS | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ✅ File logging |
| **Network** | ✅ WiFi/Ethernet | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ✅ Network logging |
| **Interrupts** | ✅ GPIO | ✅ GPIO | ✅ Fault | ❌ | ✅ Data ready | ❌ | ❌ | ❌ |

## 🏗️ Architecture Documentation

### Core System Architecture
- **[🏗️ Hardware Abstraction Architecture](docs/development/ARCHITECTURE_GUIDELINES.md)** - Complete HAL architecture
- **[⚡ GPIO System Architecture](docs/component-handlers/GPIO_MANAGER_README.md)** - GPIO system design and implementation

### System Integration Guides
- **[🔧 GPIO Manager Guide](docs/component-handlers/GPIO_MANAGER_README.md)** - GPIO system guide
- **[📊 ADC Manager Guide](docs/component-handlers/ADC_MANAGER_README.md)** - ADC system guide
- **[📡 Communication Manager](docs/component-handlers/COMM_CHANNELS_MANAGER_README.md)** - Communication system documentation
- **[🌡️ Temperature Manager Guide](docs/component-handlers/TEMPERATURE_MANAGER_README.md)** - Temperature monitoring system
- **[💡 LED Manager Guide](docs/component-handlers/LED_MANAGER_README.md)** - LED control system
- **[🎯 Encoder Manager Guide](docs/component-handlers/ENCODER_MANAGER_README.md)** - Position encoder system

### Architecture Principles

```
┌─────────────────────────────────────────────────────────────────┐
│                        HardFOC HAL                             │
├─────────────────────────────────────────────────────────────────┤
│  📌 API Layer           │ Public interfaces & integration        │
├─────────────────────────────────────────────────────────────────┤
│  🎛️ Component Handlers  │ Managers: GPIO, ADC, Comm, IMU, Motor  │
├─────────────────────────────────────────────────────────────────┤
│  🔧 Driver Handlers     │ TMC9660, PCAL95555, AS5047U, BNO08x    │
├─────────────────────────────────────────────────────────────────┤
│  ⚙️ Hardware Drivers    │ ESP32 interfaces & external drivers    │
└─────────────────────────────────────────────────────────────────┘
```

## 🔌 API Reference

### Public API Documentation
- **[🔌 Complete API Reference](lib/api/README.md)** - Full public API documentation
- **[🚀 Integration Guide](lib/api/README.md)** - System integration examples  
- **[⚙️ System Initialization](lib/api/README.md)** - Initialization procedures

### API Organization

#### Core API Includes
```cpp
#include "api/Vortex.h"                 // Unified Vortex API
```

#### Manager Access Patterns
```cpp
// Singleton access pattern
auto& gpio = GpioManager::GetInstance();
auto& adc = AdcManager::GetInstance();
auto& motor = MotorController::GetInstance();
auto& temp = TemperatureManager::GetInstance();
auto& led = LedManager::GetInstance();
auto& encoder = EncoderManager::GetInstance();

// Initialization pattern
gpio.EnsureInitialized();
adc.Initialize();
motor.EnsureInitialized();
temp.EnsureInitialized();
led.EnsureInitialized();
encoder.EnsureInitialized();

// Usage patterns
gpio.SetActive("GPIO_EXT_GPIO_CS_1");
float voltage = adc.ReadVoltage("ADC_TMC9660_AIN3");
auto* handler = motor.handler(0);
float temperature = temp.ReadTemperature("ESP32_INTERNAL");
led.SetColor(LedColor(255, 0, 0));
uint16_t angle = encoder.ReadAngle(0);
```

## 🔗 Hardware Integration

### Board Configuration
- **[🏗️ Hardware Architecture](docs/development/ARCHITECTURE_GUIDELINES.md)** - Board-specific configuration

### Device Documentation
- **[🔌 PCAL95555 Handler](docs/driver-handlers/PCAL95555_HANDLER_README.md)** - GPIO expander documentation
- **[🎛️ TMC9660 Handler](docs/driver-handlers/TMC9660_HANDLER_README.md)** - Motor controller documentation
- **[📐 AS5047U Handler](docs/driver-handlers/AS5047U_HANDLER_README.md)** - Position encoder documentation
- **[🧭 BNO08x Handler](docs/driver-handlers/BNO08X_HANDLER_README.md)** - IMU sensor documentation

### Communication Interface
- **[📡 Communication Manager](docs/component-handlers/COMM_CHANNELS_MANAGER_README.md)** - SPI, I2C, UART, CAN communication

## 🧪 Testing and Validation

### ESP32 On-Target Test Suite

All testing has been consolidated into the `examples/esp32/` on-target test framework that runs directly on ESP32 hardware. This replaces the previous host-based `tests/` directory.

- **[📦 ESP32 Test Suite](examples/esp32/README.md)** - On-target test framework and build instructions
- **[🔧 Test Framework Header](examples/esp32/main/TestFramework.h)** - Lightweight `TEST_ASSERT` macros for embedded
- **[🚀 API Integration Tests](examples/esp32/main/vortex_api_test.cpp)** - Full Vortex API smoke tests

### Running Tests

```bash
# Build and flash the on-target test suite
cd examples/esp32
./scripts/build_app.sh --app-type example --build-type debug \
    --app-source-file main/vortex_api_test.cpp --target esp32c6
```

### Test Coverage
- **Manager Initialization**: All 8 managers verify `EnsureInitialized()` → `true`
- **GPIO Operations**: Pin set/get, batch read/write validation
- **ADC Readings**: Channel voltage reads, diagnostics dump
- **Motor Controller**: Handler enumeration, basic register access
- **Encoder/IMU/LED/Temp**: Instance and health checks

### Performance Benchmarks
- **GPIO Performance**: Pin toggle rates, batch operation timing
- **ADC Performance**: Sample rates, multi-channel read timing
- **Communication Performance**: SPI/I2C/UART throughput and latency
- **Motor Control Performance**: Acceleration rates, positioning accuracy

## � Continuous Integration

### CI Pipelines (`.github/workflows/`)

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| **🔨 ESP32 Build CI** | push/PR → main | Builds `examples/esp32` on-target test suite |
| **🧹 C++ Lint** | push/PR → main | clang-format checks against `_config/.clang-format` |
| **🔍 C++ Analysis** | push/PR → main | clang-tidy static analysis via `_config/.clang-tidy` |
| **📝 Markdown Lint** | push/PR → main | markdownlint via `_config/.markdownlint.json` |
| **📋 YAML Lint** | push/PR → main | yamllint via `_config/.yamllint` |
| **🔗 Docs Link Check** | push/PR → main | lychee link validation via `_config/lychee.toml` |
| **📖 Docs Publish** | push → main/release/tags | Doxygen + Jekyll → GitHub Pages |
| **📦 Release** | tag `v*` | Automated release packaging |

### Documentation Generation

Doxygen API docs and Jekyll site are configured under `_config/`:
- **[Doxyfile](_config/Doxyfile)** — Generates C++ API docs for `lib/api/` and `lib/managers/`
- **[Jekyll _config.yml](_config/_config.yml)** — Just-the-docs theme with dark mode
- **[Layouts](_config/_layouts/)** — Custom layouts: minimal, print, workflow
- **[Includes](_config/_includes/)** — Custom head/footer HTML fragments
- **[doxygen-awesome-css](_config/doxygen-extensions/doxygen-awesome-css/)** — Doxygen theme submodule

## �🛠️ Development and Contributing

### Development Guidelines
- **[📝 Coding Standards](docs/development/CODING_STANDARDS.md)** - Code style and conventions
- **[🏗️ Architecture Guidelines](docs/development/ARCHITECTURE_GUIDELINES.md)** - System design principles
- **[⚡ Performance Optimization Guide](docs/development/PERFORMANCE_OPTIMIZATION_GUIDE.md)** - String lookups vs cached access
- **[🔧 Base Interface Reference](docs/development/BASEINTERFACE_REFERENCE.md)** - BaseGpio and BaseAdc detailed API
- **[🧪 Testing Requirements](docs/development/TESTING_REQUIREMENTS.md)** - Test coverage and quality
- **[📚 Documentation Standards](docs/development/DOCUMENTATION_STANDARDS.md)** - Documentation requirements

### Documentation Corrections
- **[📋 Documentation Corrections Summary](DOCUMENTATION_CORRECTIONS.md)** - Summary of base interface corrections made
- **[🚗 Motor Controller Corrections](MOTOR_CONTROLLER_CORRECTIONS.md)** - Comprehensive TMC9660 driver interface corrections

### Contribution Workflow
1. **Fork Repository** - Create your own fork for development
2. **Create Feature Branch** - Isolate your changes in a feature branch  
3. **Follow Standards** - Adhere to coding and documentation standards
4. **Add Tests** - Ensure comprehensive test coverage for new features
5. **Update Documentation** - Keep documentation current and complete
6. **Submit Pull Request** - Submit for review with clear description

## 📊 System Status and Monitoring

### Health Monitoring
```cpp
#include "handlers/Logger.h"

// System health check
auto& logger = Logger::GetInstance();
auto& vortex = Vortex::GetInstance();
auto diagnostics = vortex.GetSystemDiagnostics();
if (!diagnostics.system_healthy) {
    // Get detailed status from individual managers
    auto gpio_status = vortex.gpio.GetSystemStatus();
    auto adc_status = vortex.adc.GetSystemStatus();
    auto temp_status = vortex.temperature.GetSystemStatus();
    auto led_status = vortex.led.GetSystemStatus();
    auto encoder_status = vortex.encoder.GetSystemStatus();
    
    logger.Info("SYSTEM", "GPIO Health: %s", gpio_status.overall_healthy ? "OK" : "FAIL");
    logger.Info("SYSTEM", "ADC Health: %s", adc_status.overall_healthy ? "OK" : "FAIL");
    logger.Info("SYSTEM", "Temperature Health: %s", temp_status.system_healthy ? "OK" : "FAIL");
    logger.Info("SYSTEM", "LED Health: %s", led_status.system_healthy ? "OK" : "FAIL");
    logger.Info("SYSTEM", "Encoder Health: %s", encoder_status.system_healthy ? "OK" : "FAIL");
}
```

### Diagnostic Information
- **Real-time Statistics**: Access counts, error rates, performance metrics
- **Hardware Status**: Chip health, communication status, fault detection
- **System Metrics**: Memory usage, task performance, resource utilization

## 🔍 Advanced Topics

### Performance Optimization
- **Batch Operations**: Multi-pin GPIO, multi-channel ADC for better performance
- **Interrupt Usage**: Efficient event handling with minimal latency
- **Memory Management**: Optimized memory usage patterns
- **Real-time Considerations**: Task priorities and timing guarantees

### Custom Integration
- **Custom Hardware**: Adding new devices and handlers
- **Platform Porting**: Adapting to different microcontroller platforms
- **Protocol Extensions**: Adding new communication protocols
- **Application Integration**: Integrating with existing applications

### Troubleshooting Guides
- **[🚨 Common Issues](docs/troubleshooting/COMMON_ISSUES.md)** - Frequently encountered problems
- **[🔧 Debug Procedures](docs/troubleshooting/DEBUG_PROCEDURES.md)** - Step-by-step debugging
- **[📋 Error Codes](docs/troubleshooting/ERROR_CODES.md)** - Complete error code reference
- **[🏥 Health Monitoring](docs/troubleshooting/HEALTH_MONITORING.md)** - System health diagnostics

## 📚 Reference Materials

### External Documentation
- **[ESP32 Technical Reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/)** - ESP32 official documentation
- **[TMC9660 Datasheet](https://www.trinamic.com/products/integrated-circuits/details/tmc9660/)** - TMC9660 motor controller
- **[PCAL95555 Datasheet](https://www.nxp.com/docs/en/data-sheet/PCAL95555.pdf)** - PCAL95555 GPIO expander
- **[BNO08x Reference](https://www.ceva-dsp.com/product/bno080-085/)** - BNO08x IMU sensor

### Standards and Protocols
- **SPI Protocol**: Serial Peripheral Interface specifications
- **I2C Protocol**: Inter-Integrated Circuit specifications  
- **UART Protocol**: Universal Asynchronous Receiver-Transmitter
- **TMCL Protocol**: Trinamic Motion Control Language (ON SPI OR UART PORT)

## 🆘 Getting Help

### Support Channels
- **📚 Documentation**: Start with this comprehensive documentation
- **🐛 GitHub Issues**: Report bugs and request features
- **💬 GitHub Discussions**: Ask questions and share experiences
- **📧 Direct Contact**: Reach out to the HardFOC development team

### FAQ and Common Solutions
- **Initialization Issues**: Check power supply and connections
- **Communication Problems**: Verify interface configuration and wiring
- **Performance Issues**: Review system load and optimization guidelines
- **Integration Challenges**: Consult integration guides and examples

---

<div align="center">

**📖 Complete Documentation Coverage**

This documentation index covers 100% of the HardFOC HAL system. Each component, feature, and integration point is thoroughly documented with examples, troubleshooting guides, and best practices.

**[⭐ Star the Project](https://github.com/hardfoc/hf-hal)** • **[🐛 Report Issues](https://github.com/hardfoc/hf-hal/issues)** • **[💡 Request Features](https://github.com/hardfoc/hf-hal/issues)**

*Last updated: June 2025 | Version 3.0 | Status: Complete*

</div>
