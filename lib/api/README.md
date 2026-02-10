# Vortex API - Unified HardFOC Platform Interface

## Overview

The Vortex API provides a unified singleton interface to all component handlers in the HardFOC Vortex platform. It implements lazy initialization with proper dependency management, ensuring all systems are properly initialized in the correct order before exposing them to the user.

## 🏗️ Architecture

### Design Principles

- **True Singleton Pattern**: Uses Meyer's singleton for thread-safe initialization
- **Lazy Initialization**: Components are initialized only when needed
- **Dependency Management**: Proper initialization order based on component dependencies
- **Exception-Free Design**: All methods are `noexcept` with proper error handling
- **Reference-Based Access**: Direct access to component handlers by reference
- **Comprehensive Diagnostics**: Full system health monitoring and diagnostics

### Component Dependencies

The initialization order is carefully managed to handle dependencies:

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

## 🚀 Quick Start

### Basic Usage

```cpp
#include "api/Vortex.h"

// Get the Vortex API instance
auto& vortex = Vortex::GetInstance();

// Ensure all systems are initialized
if (vortex.EnsureInitialized()) {
    // Access communication channels
    auto& comms = vortex.comms;
    
    // Access GPIO management
    auto& gpio = vortex.gpio;
    
    // Access motor controllers
    auto& motors = vortex.motors;
    
    // Access IMU sensors
    auto& imu = vortex.imu;
    
    // Access encoders
    auto& encoders = vortex.encoders;
    
    // Access LED management
    auto& leds = vortex.leds;
    
    // Access temperature sensors
    auto& temp = vortex.temp;
    
    // Access ADC channels
    auto& adc = vortex.adc;
}
```

### ⚡ Performance-Optimized Usage

The Vortex API provides two access patterns for optimal performance across all component handlers:

#### 🔍 String-Based Access (Convenience & Extensibility)
```cpp
// String-based API - Great for configuration and setup
auto& vortex = Vortex::GetInstance();
if (vortex.EnsureInitialized()) {
    // GPIO operations with string lookup
    vortex.gpio.Set("GPIO_EXT_GPIO_CS_1", true);
    bool state;
    vortex.gpio.Read("GPIO_PCAL_GPIO17", state);
    
    // ADC operations with string lookup
    float voltage, current;
    vortex.adc.ReadChannelV("ADC_TMC9660_AIN3", voltage);
    vortex.adc.ReadChannelV("TMC9660_CURRENT_I0", current);
    
    // Motor control with string-based identification
    auto* motor = vortex.motors.handler(0);
    
    // Other components follow the same pattern
    vortex.leds.SetStatus(LedAnimation::STATUS_OK);
}
```

#### ⚡ Cached Access (High Performance)
```cpp
// Cache component pointers for maximum performance
auto& vortex = Vortex::GetInstance();
if (vortex.EnsureInitialized()) {
    // STEP 1: Cache pointers for fast access (do once, outside loops)
    auto gpio_cs1 = vortex.gpio.Get("GPIO_EXT_GPIO_CS_1");
    auto* adc_temp = vortex.adc.Get("ADC_TMC9660_AIN3");
    auto* adc_current = vortex.adc.Get("TMC9660_CURRENT_I0");
    auto* motor_handler = vortex.motors.handler(0);
    
    // STEP 2: Validate cached pointers
    if (!gpio_cs1 || !adc_temp || !adc_current || !motor_handler) {
        printf("ERROR: Failed to cache component pointers\n");
        return;
    }
    
    // STEP 3: Use cached pointers for high-frequency operations
    for (int i = 0; i < 50000; i++) {
        // Direct component access - 5-15x faster than string lookups
        gpio_cs1->Toggle();                    // ~10-50ns vs ~100-500ns
        
        float temp, current;
        adc_temp->ReadChannelV(0, temp);           // ~20-100ns vs ~200-800ns
        adc_current->ReadChannelV(0, current);     // Direct BaseAdc access
        
        // Process readings for motor control
        if (auto driver = motor_handler->GetTmc9660Driver()) {
            // Use motor driver directly for real-time control
        }
    }
}
```

#### 📊 Performance Guidelines by Component

| Component | String Lookup | Cached Access | Use Cached For |
|-----------|---------------|---------------|----------------|
| **GPIO** | ~100-500ns | ~10-50ns | Control loops >1kHz |
| **ADC** | ~200-800ns | ~20-100ns | Sensor feedback >1kHz |
| **Motor** | ~300-1000ns | ~30-150ns | Real-time motor control |
| **IMU** | ~400-1200ns | ~40-200ns | Motion control loops |
| **Encoder** | ~300-900ns | ~30-120ns | Position feedback |

### Advanced Usage

```cpp
// Get system diagnostics
VortexSystemDiagnostics diagnostics;
if (vortex.GetSystemDiagnostics(diagnostics)) {
    printf("System Health: %s\n", 
           diagnostics.system_healthy ? "HEALTHY" : "UNHEALTHY");
    printf("Initialized Components: %u/%u\n", 
           diagnostics.initialized_components, diagnostics.total_components);
}

// Perform health check
if (vortex.PerformHealthCheck()) {
    printf("All systems operational\n");
}

// Dump comprehensive statistics
vortex.DumpSystemStatistics();

// Get system information
printf("Version: %s\n", vortex.GetSystemVersion().c_str());
printf("Uptime: %llu ms\n", vortex.GetSystemUptimeMs());
printf("Init Time: %llu ms\n", vortex.GetInitializationTimeMs());
```

## 📋 API Reference

### Singleton Access

```cpp
static Vortex& GetInstance() noexcept;
```

Returns the singleton instance of the Vortex API.

### Initialization

```cpp
bool EnsureInitialized() noexcept;
bool IsInitialized() const noexcept;
```

Ensures all systems are properly initialized. Returns `true` if successful.

### Component Access

All components are accessed by reference:

```cpp
CommChannelsManager& comms;      // Communication channels
GpioManager& gpio;              // GPIO management
MotorController& motors;        // Motor controllers
AdcManager& adc;                // ADC management
ImuManager& imu;                // IMU sensors
EncoderManager& encoders;       // Encoders
LedManager& leds;               // LED management
TemperatureManager& temp;       // Temperature sensors
```

### Diagnostics

```cpp
bool GetSystemDiagnostics(VortexSystemDiagnostics& diagnostics) const noexcept;
std::vector<bool> GetComponentInitializationStatus() const noexcept;
std::vector<std::string> GetFailedComponents() const noexcept;
std::vector<std::string> GetSystemWarnings() const noexcept;
```

### Utility Methods

```cpp
uint64_t GetSystemUptimeMs() const noexcept;
uint64_t GetInitializationTimeMs() const noexcept;
void DumpSystemStatistics() const noexcept;
bool PerformHealthCheck() noexcept;
std::string GetSystemVersion() const noexcept;
```

## 🔧 Configuration

### Build Configuration

The Vortex API is automatically included when building the HardFOC HAL. It's registered as a component in the main `CMakeLists.txt`:

```cmake
# Include the Vortex API component
list(APPEND EXTRA_COMPONENT_DIRS "${HF_HAL_ROOT}/API")

# Add to requirements
REQUIRES driver freertos hf-ws2812-rmt-driver nvs_flash vortex-api
```

### Version Information

The API version is defined in the build configuration:

```cmake
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    VORTEX_API_VERSION_MAJOR=1
    VORTEX_API_VERSION_MINOR=0
    VORTEX_API_VERSION_PATCH=0
    VORTEX_API_VERSION_STRING="1.0.0"
)
```

## 📊 System Diagnostics

### VortexSystemDiagnostics Structure

```cpp
struct VortexSystemDiagnostics {
    bool system_healthy;                    // Overall system health
    bool comms_initialized;                 // Communication channels status
    bool gpio_initialized;                  // GPIO management status
    bool adc_initialized;                   // ADC management status
    bool motors_initialized;                // Motor controller status
    bool imu_initialized;                   // IMU management status
    bool encoders_initialized;              // Encoder management status
    bool leds_initialized;                  // LED management status
    bool temp_initialized;                  // Temperature management status
    
    uint32_t total_components;              // Total number of components
    uint32_t initialized_components;        // Number of initialized components
    uint32_t failed_components;             // Number of failed components
    
    uint64_t initialization_time_ms;        // Total initialization time
    uint64_t system_uptime_ms;              // System uptime
    
    std::vector<std::string> failed_components_list;  // List of failed components
    std::vector<std::string> warnings;                // System warnings
};
```

## 🧪 Examples

### Complete Example

See `examples/VortexApiExample.cpp` for a comprehensive demonstration of all Vortex API features.

### Minimal Example

```cpp
#include "api/Vortex.h"

extern "C" void app_main(void) {
    // Get Vortex API instance
    auto& vortex = Vortex::GetInstance();
    
    // Initialize all systems
    if (vortex.EnsureInitialized()) {
        // Use motor controller
        auto& motors = vortex.motors;
        auto* handler = motors.handler(0);
        if (handler) {
            // Motor controller operations
        }
        
        // Use IMU
        auto& imu = vortex.imu;
        auto* imu_handler = imu.GetBno08xHandler(0);
        if (imu_handler) {
            // IMU operations
        }
        
        // Use LED
        auto& leds = vortex.leds;
        leds.SetStatus(LedAnimation::STATUS_OK);
    }
}
```

## 🔍 Troubleshooting

### Common Issues

1. **Initialization Fails**
   - Check that all required hardware is connected
   - Verify component dependencies are satisfied
   - Check system logs for specific error messages

2. **Component Not Available**
   - Ensure the component is properly initialized
   - Check hardware connections
   - Verify configuration settings

3. **Performance Issues**
   - Monitor initialization time
   - Check for failed components
   - Review system diagnostics

### Debug Information

Enable debug logging to see detailed initialization information:

```cpp
// The API automatically logs initialization progress
// Check serial output for detailed information
```

## 📈 Performance

### Initialization Time

Typical initialization times:
- **Fast Path**: ~50-100ms (all components healthy)
- **Slow Path**: ~200-500ms (with retries and error handling)

### Memory Usage

- **Static Memory**: ~2KB (singleton instance)
- **Runtime Memory**: ~1KB (diagnostics and state tracking)
- **Component Memory**: Varies by component (not owned by API)

### Thread Safety

- **Singleton Access**: Thread-safe (Meyer's singleton)
- **Initialization**: Thread-safe with atomic operations
- **Component Access**: Depends on individual component implementations

## 🔮 Future Enhancements

### Planned Features

1. **Dynamic Component Loading**: Load components on-demand
2. **Configuration Management**: Runtime configuration changes
3. **Event System**: Component state change notifications
4. **Performance Monitoring**: Real-time performance metrics
5. **Plugin Architecture**: Extensible component system

### Extension Points

The API is designed for extensibility:
- Easy addition of new components
- Custom initialization sequences
- Specialized diagnostic information
- Platform-specific optimizations

## 📚 Related Documentation

- [Component Handlers Documentation](../component-handlers/)
- [Driver Handlers Documentation](../utils-and-drivers/driver-handlers/)
- [Core Utils Documentation](../utils-and-drivers/hf-core-utils/)
- [Core Drivers Documentation](../utils-and-drivers/hf-core-drivers/)

## 🤝 Contributing

When contributing to the Vortex API:

1. **Follow the Architecture**: Maintain the singleton pattern and dependency management
2. **Add Diagnostics**: Include proper error reporting and health monitoring
3. **Update Documentation**: Keep this README and code comments current
4. **Test Thoroughly**: Ensure all components work together properly
5. **Performance**: Monitor initialization time and memory usage

## 📄 License

This API is part of the HardFOC HAL and follows the same licensing terms.

---

**The Vortex API provides a unified interface to the entire HardFOC platform, making it easy to access all components through a single, well-designed interface.** 