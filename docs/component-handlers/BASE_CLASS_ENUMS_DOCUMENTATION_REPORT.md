# Base Class Enums Documentation Report

## 📋 Overview

This report documents all critical enums and their meanings from the base classes in the HardFOC HAL system. These enums are fundamental to understanding how to properly use the GPIO and ADC managers, and they define the core behavior and error handling patterns used throughout the system.

## 🔍 Base Classes Analyzed

### **1. BaseGpio.h** - GPIO Base Class
- **Location**: `utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseGpio.h`
- **Purpose**: Defines GPIO state, configuration, and error enums

### **2. BaseAdc.h** - ADC Base Class  
- **Location**: `utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseAdc.h`
- **Purpose**: Defines ADC error codes and operation types

### **3. HardwareTypes.h** - Platform-Agnostic Types
- **Location**: `utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/HardwareTypes.h`
- **Purpose**: Defines fundamental types used across the system

## 📊 Critical Enum Definitions

### **1. GPIO State Enums**

#### **Logical States (Application Level)**
```cpp
enum class hf_gpio_state_t : hf_u8_t {
    HF_GPIO_STATE_INACTIVE = 0,  ///< Logical inactive state
    HF_GPIO_STATE_ACTIVE = 1     ///< Logical active state
};
```

**Usage**: These represent what the application sees, independent of electrical polarity.

#### **Electrical Levels (Hardware Level)**
```cpp
enum class hf_gpio_level_t : hf_u8_t {
    HF_GPIO_LEVEL_LOW = 0,   ///< Electrical low level (0V)
    HF_GPIO_LEVEL_HIGH = 1   ///< Electrical high level (VCC)
};
```

**Usage**: These represent the actual voltage levels on the physical pin.

#### **Active State Polarity (Configuration)**
```cpp
enum class hf_gpio_active_state_t : hf_u8_t {
    HF_GPIO_ACTIVE_LOW = 0,  ///< Active state is electrical low
    HF_GPIO_ACTIVE_HIGH = 1  ///< Active state is electrical high
};
```

**Usage**: Defines which electrical level corresponds to the logical "active" state.

### **2. GPIO Configuration Enums**

#### **Pin Direction**
```cpp
enum class hf_gpio_direction_t : hf_u8_t {
    HF_GPIO_DIRECTION_INPUT = 0,   ///< Pin configured as input
    HF_GPIO_DIRECTION_OUTPUT = 1   ///< Pin configured as output
};
```

#### **Output Drive Mode**
```cpp
enum class hf_gpio_output_mode_t : hf_u8_t {
    HF_GPIO_OUTPUT_MODE_PUSH_PULL = 0,  ///< Push-pull output (strong high and low)
    HF_GPIO_OUTPUT_MODE_OPEN_DRAIN = 1  ///< Open-drain output (strong low, high-impedance high)
};
```

#### **Pull Resistor Configuration**
```cpp
enum class hf_gpio_pull_mode_t : hf_u8_t {
    HF_GPIO_PULL_MODE_FLOATING = 0,  ///< No pull resistor (floating/high-impedance)
    HF_GPIO_PULL_MODE_UP = 1,        ///< Internal pull-up resistor enabled
    HF_GPIO_PULL_MODE_DOWN = 2,      ///< Internal pull-down resistor enabled
    HF_GPIO_PULL_MODE_UP_DOWN = 3    ///< Both pull-up and pull-down resistors enabled
};
```

### **3. GPIO Interrupt Enums**

#### **Interrupt Trigger Types**
```cpp
enum class hf_gpio_interrupt_trigger_t : hf_u8_t {
    HF_GPIO_INTERRUPT_TRIGGER_NONE = 0,         ///< No interrupt (disabled)
    HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE = 1,  ///< Trigger on rising edge (low to high)
    HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE = 2, ///< Trigger on falling edge (high to low)
    HF_GPIO_INTERRUPT_TRIGGER_BOTH_EDGES = 3,   ///< Trigger on both rising and falling edges
    HF_GPIO_INTERRUPT_TRIGGER_LOW_LEVEL = 4,    ///< Trigger on low level
    HF_GPIO_INTERRUPT_TRIGGER_HIGH_LEVEL = 5    ///< Trigger on high level
};
```

### **4. Error Code Enums**

#### **GPIO Error Codes**
```cpp
enum class hf_gpio_err_t : hf_u8_t {
    GPIO_SUCCESS = 0,                    // Operation successful
    GPIO_ERR_FAILURE = 1,                // General failure
    GPIO_ERR_NOT_INITIALIZED = 2,        // Not initialized
    GPIO_ERR_ALREADY_INITIALIZED = 3,    // Already initialized
    GPIO_ERR_INVALID_PARAMETER = 4,      // Invalid parameter
    GPIO_ERR_NULL_POINTER = 5,           // Null pointer
    GPIO_ERR_OUT_OF_MEMORY = 6,          // Out of memory
    GPIO_ERR_INVALID_PIN = 7,            // Invalid pin
    GPIO_ERR_PIN_NOT_FOUND = 8,          // Pin not found
    GPIO_ERR_PIN_NOT_CONFIGURED = 9,     // Pin not configured
    GPIO_ERR_PIN_ALREADY_REGISTERED = 10, // Pin already registered
    GPIO_ERR_PIN_ACCESS_DENIED = 11,     // Pin access denied
    GPIO_ERR_PIN_BUSY = 12,              // Pin busy
    GPIO_ERR_HARDWARE_FAULT = 13,        // Hardware fault
    GPIO_ERR_COMMUNICATION_FAILURE = 14, // Communication failure
    GPIO_ERR_DEVICE_NOT_RESPONDING = 15, // Device not responding
    GPIO_ERR_TIMEOUT = 16,               // Timeout
    GPIO_ERR_VOLTAGE_OUT_OF_RANGE = 17,  // Voltage out of range
    GPIO_ERR_INVALID_CONFIGURATION = 18, // Invalid configuration
    GPIO_ERR_UNSUPPORTED_OPERATION = 19, // Unsupported operation
    GPIO_ERR_RESOURCE_BUSY = 20,         // Resource busy
    GPIO_ERR_RESOURCE_UNAVAILABLE = 21,  // Resource unavailable
    GPIO_ERR_READ_FAILURE = 22,          // Read failure
    GPIO_ERR_WRITE_FAILURE = 23,         // Write failure
    GPIO_ERR_DIRECTION_MISMATCH = 24,    // Direction mismatch
    GPIO_ERR_PULL_RESISTOR_FAILURE = 25, // Pull resistor failure
    GPIO_ERR_INTERRUPT_NOT_SUPPORTED = 26,     // Interrupt not supported
    GPIO_ERR_INTERRUPT_ALREADY_ENABLED = 27,   // Interrupt already enabled
    GPIO_ERR_INTERRUPT_NOT_ENABLED = 28,       // Interrupt not enabled
    GPIO_ERR_INTERRUPT_HANDLER_FAILED = 29,    // Interrupt handler failed
    GPIO_ERR_SYSTEM_ERROR = 30,          // System error
    GPIO_ERR_PERMISSION_DENIED = 31,     // Permission denied
    GPIO_ERR_OPERATION_ABORTED = 32,     // Operation aborted
    GPIO_ERR_NOT_SUPPORTED = 33,         // Operation not supported
    GPIO_ERR_DRIVER_ERROR = 34,          // Driver error
    GPIO_ERR_INVALID_STATE = 35,         // Invalid state
    GPIO_ERR_INVALID_ARG = 36,           // Invalid argument
    GPIO_ERR_CALIBRATION_FAILURE = 37    // Calibration failure
};
```

#### **ADC Error Codes**
```cpp
enum class hf_adc_err_t : hf_u8_t {
    ADC_SUCCESS = 0,                      // Operation successful
    ADC_ERR_FAILURE = 1,                  // General failure
    ADC_ERR_NOT_INITIALIZED = 2,          // Not initialized
    ADC_ERR_ALREADY_INITIALIZED = 3,      // Already initialized
    ADC_ERR_INVALID_PARAMETER = 4,        // Invalid parameter
    ADC_ERR_NULL_POINTER = 5,             // Null pointer
    ADC_ERR_OUT_OF_MEMORY = 6,            // Out of memory
    ADC_ERR_CHANNEL_NOT_FOUND = 7,        // Channel not found
    ADC_ERR_CHANNEL_NOT_ENABLED = 8,      // Channel not enabled
    ADC_ERR_CHANNEL_NOT_CONFIGURED = 9,   // Channel not configured
    ADC_ERR_CHANNEL_ALREADY_REGISTERED = 10, // Channel already registered
    ADC_ERR_CHANNEL_READ_ERR = 11,        // Channel read error
    ADC_ERR_CHANNEL_WRITE_ERR = 12,       // Channel write error
    ADC_ERR_INVALID_CHANNEL = 13,         // Invalid channel
    ADC_ERR_CHANNEL_BUSY = 14,            // Channel busy
    ADC_ERR_INVALID_SAMPLE_COUNT = 15,    // Invalid sample count
    ADC_ERR_SAMPLE_TIMEOUT = 16,          // Sample timeout
    ADC_ERR_SAMPLE_OVERFLOW = 17,         // Sample overflow
    ADC_ERR_SAMPLE_UNDERFLOW = 18,        // Sample underflow
    ADC_ERR_HARDWARE_FAULT = 19,          // Hardware fault
    ADC_ERR_COMMUNICATION_FAILURE = 20,   // Communication failure
    ADC_ERR_DEVICE_NOT_RESPONDING = 21,   // Device not responding
    ADC_ERR_CALIBRATION_FAILURE = 22,     // Calibration failure
    ADC_ERR_VOLTAGE_OUT_OF_RANGE = 23,    // Voltage out of range
    ADC_ERR_INVALID_CONFIGURATION = 24,   // Invalid configuration
    ADC_ERR_UNSUPPORTED_OPERATION = 25,   // Unsupported operation
    ADC_ERR_RESOURCE_BUSY = 26,           // Resource busy
    ADC_ERR_RESOURCE_UNAVAILABLE = 27,    // Resource unavailable
    ADC_ERR_CALIBRATION_NOT_FOUND = 28,   // Calibration data not found
    ADC_ERR_CALIBRATION_INVALID = 29,     // Invalid calibration data
    ADC_ERR_CALIBRATION_EXPIRED = 30,     // Calibration has expired
    ADC_ERR_CALIBRATION_DRIFT = 31,       // Calibration drift detected
    ADC_ERR_CALIBRATION_POINTS_INSUFFICIENT = 32, // Insufficient calibration points
    ADC_ERR_CALIBRATION_POINTS_INVALID = 33,      // Invalid calibration points
    ADC_ERR_CALIBRATION_LINEARITY_ERROR = 34,     // Calibration linearity error
    ADC_ERR_CALIBRATION_STORAGE_FAILURE = 35,     // Calibration storage failure
    ADC_ERR_CALIBRATION_LOAD_FAILURE = 36,        // Calibration load failure
    ADC_ERR_CALIBRATION_VERIFICATION_FAILED = 37, // Calibration verification failed
    ADC_ERR_CALIBRATION_TEMPERATURE_ERROR = 38,   // Temperature compensation error
    ADC_ERR_CALIBRATION_POLYNOMIAL_ERROR = 39,    // Polynomial calibration error
    ADC_ERR_SYSTEM_ERROR = 40,            // System error
    ADC_ERR_PERMISSION_DENIED = 41,       // Permission denied
    ADC_ERR_OPERATION_ABORTED = 42,       // Operation aborted
    ADC_ERR_INITIALIZATION_FAILED = 43,   // Initialization failed
    ADC_ERR_INVALID_PARAM = 44,           // Invalid parameter
    ADC_ERR_TIMEOUT = 45,                 // Operation timeout
    ADC_ERR_NOT_SUPPORTED = 46,           // Not supported
    ADC_ERR_INVALID_STATE = 47,           // Invalid state
    ADC_ERR_DRIVER_ERROR = 48,            // Driver error
    ADC_ERR_DMA_ERROR = 49,               // DMA error
    ADC_ERR_FILTER_ERROR = 50,            // Filter configuration error
    ADC_ERR_NO_CALLBACK = 51,             // No callback provided
    ADC_ERR_NOT_STARTED = 52,             // Operation not started
    ADC_ERR_CALIBRATION = 53,             // Calibration error
    ADC_ERR_BUSY = 54,                    // Resource busy
    ADC_ERR_HARDWARE_FAILURE = 55,        // Hardware failure
    ADC_ERR_CHANNEL_DISABLED = 56         // Channel disabled
};
```

### **5. Platform-Agnostic Types**

#### **Fundamental Types**
```cpp
// Integer types
using hf_u8_t = uint8_t;   // 8-bit unsigned
using hf_u16_t = uint16_t; // 16-bit unsigned
using hf_u32_t = uint32_t; // 32-bit unsigned
using hf_u64_t = uint64_t; // 64-bit unsigned
using hf_i8_t = int8_t;    // 8-bit signed
using hf_i16_t = int16_t;  // 16-bit signed
using hf_i32_t = int32_t;  // 32-bit signed
using hf_i64_t = int64_t;  // 64-bit signed
using hf_bool_t = bool;    // Boolean type

// Hardware-specific types
using hf_pin_num_t = hf_i32_t;      // GPIO pin number
using hf_port_num_t = hf_u32_t;     // Communication port number
using hf_host_id_t = hf_u32_t;      // Host/controller identifier
using hf_channel_id_t = hf_u32_t;   // ADC/PWM channel identifier
using hf_time_t = hf_u32_t;         // Time in milliseconds
using hf_frequency_hz_t = hf_u32_t; // Frequency in Hz
using hf_baud_rate_t = hf_u32_t;    // Baud rate
```

#### **Constants**
```cpp
// Invalid values
constexpr hf_pin_num_t HF_INVALID_PIN = -1;
constexpr hf_port_num_t HF_INVALID_PORT = std::numeric_limits<hf_port_num_t>::max();
constexpr hf_host_id_t HF_INVALID_HOST = std::numeric_limits<hf_host_id_t>::max();
constexpr hf_channel_id_t HF_INVALID_CHANNEL = std::numeric_limits<hf_channel_id_t>::max();

// Timeout values
constexpr hf_time_t HF_TIMEOUT_DEFAULT_MS = 1000;
constexpr hf_time_t HF_TIMEOUT_NONE = 0;
constexpr hf_time_t HF_TIMEOUT_MAX = std::numeric_limits<hf_time_t>::max();

// Limits
constexpr hf_pin_num_t HF_MAX_PIN_NUMBER = 255;
```

## 🎯 How to Use These Enums Correctly

### **1. GPIO State vs Electrical Level Understanding**

#### **Critical Concept**
The distinction between logical state and electrical level is fundamental to proper GPIO usage:

- **Logical State**: What the application sees (`ACTIVE`/`INACTIVE`)
- **Electrical Level**: Actual voltage on the pin (`HIGH`/`LOW`)
- **Active State Polarity**: Configuration that maps logical to electrical

#### **Examples**

**Active-Low LED (Common Configuration)**:
```cpp
// LED cathode connected to pin, anode to VCC
// When pin is electrically LOW, LED turns ON
// When pin is electrically HIGH, LED turns OFF

gpio.Set("LED_PIN", true);   // Sets logical state to ACTIVE
// If pin configured as ACTIVE_LOW: sets electrical level to LOW (LED ON)
// If pin configured as ACTIVE_HIGH: sets electrical level to HIGH (LED ON)
```

**Active-High Relay (Common Configuration)**:
```cpp
// Relay control pin needs HIGH to activate
// When pin is electrically HIGH, relay turns ON
// When pin is electrically LOW, relay turns OFF

gpio.Set("RELAY_PIN", true);  // Sets logical state to ACTIVE
// If pin configured as ACTIVE_LOW: sets electrical level to LOW (relay OFF)
// If pin configured as ACTIVE_HIGH: sets electrical level to HIGH (relay ON)
```

### **2. Proper GPIO Configuration**

#### **Output Pin Configuration**
```cpp
// Configure pin as output with proper settings
gpio.SetDirection("GPIO_EXT_GPIO_CS_1", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
gpio.SetOutputMode("GPIO_EXT_GPIO_CS_1", hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);
gpio.SetPullMode("GPIO_EXT_GPIO_CS_1", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
```

#### **Input Pin Configuration**
```cpp
// Configure pin as input with pull-up (common for buttons)
gpio.SetDirection("GPIO_PCAL_GPIO17", hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
gpio.SetPullMode("GPIO_PCAL_GPIO17", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
```

### **3. Interrupt Configuration**

#### **Rising Edge Interrupt**
```cpp
// Configure interrupt on rising edge (common for buttons)
gpio.ConfigureInterrupt("GPIO_PCAL_IMU_INT", 
                       hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE,
                       my_interrupt_callback);
gpio.EnableInterrupt("GPIO_PCAL_IMU_INT");
```

#### **Both Edges Interrupt**
```cpp
// Configure interrupt on both edges (for quadrature encoders)
gpio.ConfigureInterrupt("ENCODER_PIN", 
                       hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_BOTH_EDGES,
                       encoder_interrupt_callback);
gpio.EnableInterrupt("ENCODER_PIN");
```

### **4. Error Handling Best Practices**

#### **GPIO Error Handling**
```cpp
// Always check return codes
auto result = gpio.Set("GPIO_EXT_GPIO_CS_1", true);
if (result != hf_gpio_err_t::GPIO_SUCCESS) {
    logger.Error("GPIO", "Failed to set pin: %s", HfGpioErrToString(result));
    
    // Handle specific error types
    switch (result) {
        case hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND:
            logger.Error("GPIO", "Pin not found - check pin name");
            break;
        case hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT:
            logger.Error("GPIO", "Hardware fault - check connections");
            break;
        case hf_gpio_err_t::GPIO_ERR_COMMUNICATION_FAILURE:
            logger.Error("GPIO", "Communication failure - check I2C/SPI");
            break;
        default:
            logger.Error("GPIO", "Unknown error occurred");
            break;
    }
}
```

#### **ADC Error Handling**
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
        case hf_adc_err_t::ADC_ERR_CALIBRATION_FAILURE:
            logger.Error("ADC", "Calibration failure - recalibrate ADC");
            break;
        default:
            logger.Error("ADC", "Unknown error occurred");
            break;
    }
}
```

### **5. ADC Sampling Configuration**

#### **Single Sample (Fastest)**
```cpp
float voltage;
adc.ReadVoltage("ADC_TMC9660_AIN3", voltage, 1);  // 1 sample
```

#### **Multiple Sample Averaging (More Accurate)**
```cpp
float voltage_avg;
adc.ReadVoltage("ADC_TMC9660_AIN3", voltage_avg, 16);  // 16 samples
```

#### **Raw Count Reading**
```cpp
uint32_t raw_count;
adc.ReadChannelCount("ADC_TMC9660_AIN3", raw_count, 8, 2);  // 8 samples, 2ms between
```

### **6. Batch Operations**

#### **GPIO Batch Operations**
```cpp
// Read multiple pins simultaneously
std::vector<std::string_view> pins = {"GPIO_EXT_GPIO_CS_1", "GPIO_EXT_GPIO_CS_2", 
                                      "GPIO_PCAL_GPIO17", "GPIO_PCAL_GPIO18"};
auto states = gpio.BatchRead(pins);
if (states.overall_result == hf_gpio_err_t::GPIO_SUCCESS) {
    for (size_t i = 0; i < pins.size(); i++) {
        logger.Info("GPIO", "Pin %s: %s", pins[i].data(), 
                   states.states[i] ? "HIGH" : "LOW");
    }
}
```

#### **ADC Batch Operations**
```cpp
// Read multiple channels simultaneously
std::vector<std::string_view> channels = {
    "ADC_TMC9660_AIN0", "ADC_TMC9660_AIN1", 
    "ADC_TMC9660_AIN2", "ADC_TMC9660_AIN3"
};
auto readings = adc.BatchRead(channels, 4, 1);  // 4 samples per channel, 1ms interval
if (readings.overall_result == hf_adc_err_t::ADC_SUCCESS) {
    for (size_t i = 0; i < channels.size(); i++) {
        logger.Info("ADC", "Channel %s: %.3fV", channels[i].data(), readings.voltages[i]);
    }
}
```

## 📈 Common Usage Patterns

### **1. LED Control Pattern**
```cpp
// Configure LED pin (typically active-low)
gpio.SetDirection("LED_STATUS", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
gpio.SetOutputMode("LED_STATUS", hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);

// Turn LED ON
gpio.Set("LED_STATUS", true);   // Logical ACTIVE

// Turn LED OFF  
gpio.Set("LED_STATUS", false);  // Logical INACTIVE

// Blink LED
for (int i = 0; i < 5; i++) {
    gpio.Set("LED_STATUS", true);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio.Set("LED_STATUS", false);
    vTaskDelay(pdMS_TO_TICKS(200));
}
```

### **2. Button Input Pattern**
```cpp
// Configure button pin (typically active-low with pull-up)
gpio.SetDirection("BUTTON_INPUT", hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
gpio.SetPullMode("BUTTON_INPUT", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);

// Read button state
bool button_pressed;
gpio.Read("BUTTON_INPUT", button_pressed);
if (button_pressed) {
    logger.Info("BUTTON", "Button pressed");
}
```

### **3. Interrupt-Driven Input Pattern**
```cpp
// Configure interrupt pin
gpio.SetDirection("SENSOR_INT", hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
gpio.SetPullMode("SENSOR_INT", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);

// Configure interrupt callback
void sensor_interrupt_callback(BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) {
    logger.Info("SENSOR", "Sensor interrupt triggered");
    // Handle sensor data
}

// Configure and enable interrupt
gpio.ConfigureInterrupt("SENSOR_INT", 
                       hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE,
                       sensor_interrupt_callback);
gpio.EnableInterrupt("SENSOR_INT");
```

### **4. ADC Monitoring Pattern**
```cpp
// Read temperature sensor
float temperature_voltage;
auto result = adc.ReadVoltage("ADC_TMC9660_AIN3", temperature_voltage, 8);
if (result == hf_adc_err_t::ADC_SUCCESS) {
    float temperature_celsius = (temperature_voltage - 0.5) * 100.0;  // Convert to °C
    logger.Info("TEMP", "Temperature: %.1f°C", temperature_celsius);
}

// Monitor multiple channels
std::vector<std::string_view> monitor_channels = {
    "TMC9660_CHIP_TEMPERATURE", "TMC9660_SUPPLY_VOLTAGE", "TMC9660_MOTOR_CURRENT"
};
auto monitor_readings = adc.BatchRead(monitor_channels, 4, 2);
if (monitor_readings.overall_result == hf_adc_err_t::ADC_SUCCESS) {
    logger.Info("MONITOR", "Chip Temp: %.3fV, Supply: %.3fV, Current: %.3fV",
               monitor_readings.voltages[0], monitor_readings.voltages[1], monitor_readings.voltages[2]);
}
```

## 🛡️ Error Handling Guidelines

### **1. Always Check Return Codes**
```cpp
// Good practice
auto result = gpio.Set("GPIO_PIN", true);
if (result != hf_gpio_err_t::GPIO_SUCCESS) {
    // Handle error
}

// Bad practice
gpio.Set("GPIO_PIN", true);  // Ignoring return code
```

### **2. Use Specific Error Handling**
```cpp
auto result = adc.ReadVoltage("ADC_CHANNEL", voltage);
switch (result) {
    case hf_adc_err_t::ADC_SUCCESS:
        // Success - continue processing
        break;
    case hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND:
        // Handle missing channel
        break;
    case hf_adc_err_t::ADC_ERR_HARDWARE_FAULT:
        // Handle hardware issues
        break;
    case hf_adc_err_t::ADC_ERR_VOLTAGE_OUT_OF_RANGE:
        // Handle voltage range issues
        break;
    default:
        // Handle unknown errors
        break;
}
```

### **3. Log Errors Appropriately**
```cpp
auto result = gpio.Set("GPIO_PIN", true);
if (result != hf_gpio_err_t::GPIO_SUCCESS) {
    logger.Error("GPIO", "Failed to set pin: %s", HfGpioErrToString(result));
    // Additional error handling
}
```

## 🎯 Best Practices Summary

### **1. GPIO Best Practices**
- ✅ Always understand the difference between logical state and electrical level
- ✅ Configure pins properly (direction, pull mode, output mode)
- ✅ Use appropriate interrupt triggers for your application
- ✅ Always check return codes and handle errors
- ✅ Use batch operations for multiple pins when possible

### **2. ADC Best Practices**
- ✅ Use appropriate sample counts for your accuracy requirements
- ✅ Handle calibration errors appropriately
- ✅ Check for voltage out-of-range conditions
- ✅ Use batch operations for multiple channels
- ✅ Always validate channel existence before use

### **3. General Best Practices**
- ✅ Use the provided error string functions for logging
- ✅ Implement proper error recovery mechanisms
- ✅ Use appropriate timeouts for operations
- ✅ Validate all parameters before use
- ✅ Follow the platform-agnostic type system

## 📚 Documentation Updates Made

### **Files Updated**:
1. **README.md** - Added comprehensive enum documentation and usage examples
2. **docs/component-handlers/GPIO_MANAGER_README.md** - Added enum definitions and usage patterns
3. **docs/component-handlers/ADC_MANAGER_README.md** - Added enum definitions and usage patterns

### **Sections Added**:
- Critical Enum Values and Meanings
- How to Use These Enums Correctly
- GPIO State vs Electrical Level Understanding
- Proper GPIO Configuration
- Interrupt Configuration
- Error Handling Best Practices
- Understanding Active State Polarity
- ADC Sampling Configuration
- Batch Operations
- Timeout and Error Handling
- Channel Validation

## 🎯 Conclusion

The base class enums provide the foundation for all GPIO and ADC operations in the HardFOC HAL system. Understanding these enums and their proper usage is crucial for:

1. **Correct Hardware Control**: Proper mapping between logical and electrical states
2. **Robust Error Handling**: Comprehensive error detection and recovery
3. **Optimal Performance**: Efficient configuration and operation patterns
4. **Maintainable Code**: Consistent patterns across the entire system

The documentation updates ensure that developers have clear guidance on:
- What each enum value means
- How to use them correctly
- Common usage patterns
- Error handling best practices
- Performance optimization techniques

This comprehensive understanding of the base class enums will enable developers to write robust, efficient, and maintainable code that properly utilizes the HardFOC HAL system's capabilities.

---

*This report documents the comprehensive enum definitions and usage patterns from the base classes in the HardFOC HAL system.* 