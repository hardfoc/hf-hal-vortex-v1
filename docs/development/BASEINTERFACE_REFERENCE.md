# HardFOC Vortex V1 Base Interface Reference

## 📋 Overview

This reference guide provides detailed information about the BaseGpio and BaseAdc interfaces that are used throughout the HardFOC Vortex V1 system. Understanding these interfaces is crucial for optimal cached access performance, as they represent the actual hardware abstraction layer functions available when you cache component pointers.

**⚠️ Important**: This document covers the **actual base interfaces**, not the convenience wrapper functions provided by the component managers.

## 🔧 BaseGpio Interface Reference

The BaseGpio interface provides standardized access to GPIO functionality across different hardware sources (ESP32-C6, PCAL95555, TMC9660). This interface is implemented by:

- **PCAL95555Handler::Pcal95555GpioPin** - For PCAL95555 GPIO expander pins
- **Tmc9660Handler::Gpio** - For TMC9660 motor controller GPIO pins
- **ESP32 GPIO drivers** - For native ESP32-C6 GPIO pins

### Core Interface Methods

#### Initialization and Information
```cpp
// Lifecycle management
bool Initialize() noexcept;
bool Deinitialize() noexcept;
bool EnsureInitialized() noexcept;         // Lazy initialization
bool EnsureDeinitialized() noexcept;       // Lazy deinitialization
bool IsInitialized() const noexcept;

// Hardware information
bool IsPinAvailable() const noexcept;
hf_u8_t GetMaxPins() const noexcept;
const char* GetDescription() const noexcept;
hf_pin_num_t GetPin() const noexcept;      // Get pin number/identifier
```

#### Pin State Control (Core Performance Functions)
```cpp
// High-level state management (Active/Inactive based on polarity)
hf_gpio_err_t SetActive() noexcept;                             // ~15-70ns
hf_gpio_err_t SetInactive() noexcept;                           // ~15-70ns
hf_gpio_err_t Toggle() noexcept;                                // ~30-120ns
hf_gpio_err_t IsActive(bool& is_active) noexcept;               // ~20-80ns
hf_gpio_err_t SetState(hf_gpio_state_t state) noexcept;         // Set Active/Inactive
hf_gpio_state_t GetCurrentState() const noexcept;              // Get cached state

// Low-level electrical level control (PROTECTED - not directly accessible)
// virtual hf_gpio_err_t SetPinLevelImpl(hf_gpio_level_t level) noexcept = 0;
// virtual hf_gpio_err_t GetPinLevelImpl(hf_gpio_level_t& level) noexcept = 0;
```

#### Pin Configuration
```cpp
// Direction control
hf_gpio_err_t SetDirection(hf_gpio_direction_t direction) noexcept;
hf_gpio_direction_t GetDirection() const noexcept;
bool IsInput() const noexcept;
bool IsOutput() const noexcept;

// Pull resistor control
hf_gpio_err_t SetPullMode(hf_gpio_pull_mode_t mode) noexcept;
hf_gpio_pull_mode_t GetPullMode() const noexcept;

// Output mode control
hf_gpio_err_t SetOutputMode(hf_gpio_output_mode_t mode) noexcept;
hf_gpio_output_mode_t GetOutputMode() const noexcept;

// Active state polarity
void SetActiveState(hf_gpio_active_state_t active_state) noexcept;
hf_gpio_active_state_t GetActiveState() const noexcept;
```

#### Interrupt Support (Optional - Not All Implementations)
```cpp
// Check interrupt support (returns GPIO_ERR_NOT_SUPPORTED if not available)
hf_gpio_err_t SupportsInterrupts() const noexcept;

// Interrupt configuration (virtual functions - may return GPIO_ERR_NOT_SUPPORTED)
virtual hf_gpio_err_t ConfigureInterrupt(hf_gpio_interrupt_trigger_t trigger,
                                         InterruptCallback callback = nullptr,
                                         void* user_data = nullptr) noexcept;
virtual hf_gpio_err_t EnableInterrupt() noexcept;
virtual hf_gpio_err_t DisableInterrupt() noexcept;
virtual hf_gpio_err_t WaitForInterrupt(hf_u32_t timeout_ms = 0) noexcept;
virtual hf_gpio_err_t GetInterruptStatus(InterruptStatus& status) noexcept;
```

#### Hardware Verification and Diagnostics
```cpp
// Hardware verification (reads actual hardware registers)
hf_gpio_err_t VerifyDirection(hf_gpio_direction_t& direction) const noexcept;
hf_gpio_err_t VerifyOutputMode(hf_gpio_output_mode_t& mode) const noexcept;
hf_gpio_err_t VerifyHardwareConfiguration() const noexcept;

// Statistics and diagnostics (virtual - may return GPIO_ERR_UNSUPPORTED_OPERATION)
virtual hf_gpio_err_t GetStatistics(hf_gpio_statistics_t& statistics) const noexcept;
virtual hf_gpio_err_t GetDiagnostics(hf_gpio_diagnostics_t& diagnostics) const noexcept;
virtual hf_gpio_err_t ResetStatistics() noexcept;
virtual hf_gpio_err_t ResetDiagnostics() noexcept;
```

### Hardware-Specific Extensions

#### PCAL95555 Advanced Features
The PCAL95555 GPIO implementation provides additional features beyond the base interface:

```cpp
// Polarity inversion control (PCAL95555-specific)
hf_gpio_err_t SetPolarityInversion(hf_bool_t invert) noexcept;
hf_gpio_err_t GetPolarityInversion(hf_bool_t& invert) noexcept;

// Interrupt masking (PCAL95555-specific)
hf_gpio_err_t SetInterruptMask(hf_bool_t mask) noexcept;
hf_gpio_err_t GetInterruptMask(hf_bool_t& mask) noexcept;
hf_gpio_err_t GetInterruptStatus(hf_bool_t& status) noexcept;
```

### BaseGpio Performance Characteristics

| Function | PCAL95555 | TMC9660 | ESP32 | Use Case |
|----------|-----------|---------|-------|----------|
| **SetActive()** | ~35-90ns | ~25-70ns | ~15-50ns | State control |
| **SetInactive()** | ~35-90ns | ~25-70ns | ~15-50ns | State control |
| **IsActive()** | ~40-100ns | ~30-80ns | ~20-60ns | State reading |
| **Toggle()** | ~70-180ns | ~50-140ns | ~30-100ns | Pulse generation |
| **SetDirection()** | ~100-250ns | ~80-200ns | ~50-150ns | Configuration |

## 📊 BaseAdc Interface Reference

The BaseAdc interface provides standardized access to ADC functionality. This interface is implemented by:

- **Tmc9660Handler::Adc** - For TMC9660 motor controller ADC channels
- **Tmc9660AdcWrapper** - Wrapper for TMC9660 ADC access
- **ESP32 ADC drivers** - For native ESP32-C6 ADC channels

### Core Interface Methods

#### Initialization and Information
```cpp
// Lifecycle management
bool Initialize() noexcept;
bool Deinitialize() noexcept;
bool EnsureInitialized() noexcept;         // Lazy initialization
bool EnsureDeinitialized() noexcept;       // Lazy deinitialization
bool IsInitialized() const noexcept;

// Hardware information
hf_u8_t GetMaxChannels() const noexcept;
bool IsChannelAvailable(hf_channel_id_t channel_id) const noexcept;
```

#### ADC Reading Functions (Core Performance Functions)
```cpp
// Voltage reading (most common - optimized)
hf_adc_err_t ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                         hf_u8_t numOfSamplesToAvg = 1,
                         hf_time_t timeBetweenSamples = 0) noexcept;        // ~20-100ns

// Raw count reading (fastest - no conversion)
hf_adc_err_t ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                             hf_u8_t numOfSamplesToAvg = 1,
                             hf_time_t timeBetweenSamples = 0) noexcept;    // ~15-80ns

// Combined reading (voltage + raw in one call)
hf_adc_err_t ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                        float& channel_reading_v, hf_u8_t numOfSamplesToAvg = 1,
                        hf_time_t timeBetweenSamples = 0) noexcept;         // ~25-120ns

// Multi-channel reading (batch operation - optional implementation)
virtual hf_adc_err_t ReadMultipleChannels(const hf_channel_id_t* channel_ids,
                                          hf_u8_t num_channels,
                                          hf_u32_t* raw_values,
                                          float* voltages) noexcept;
```

#### Statistics and Diagnostics (Optional)
```cpp
// Performance monitoring (virtual - may return ADC_ERR_UNSUPPORTED_OPERATION)
virtual hf_adc_err_t GetStatistics(hf_adc_statistics_t& statistics) const noexcept;
virtual hf_adc_err_t GetDiagnostics(hf_adc_diagnostics_t& diagnostics) const noexcept;
virtual hf_adc_err_t ResetStatistics() noexcept;
virtual hf_adc_err_t ResetDiagnostics() noexcept;
```

### TMC9660 ADC Specialized Functions

The TMC9660 ADC implementation provides hardware-specific channel reading methods:

```cpp
// External analog input channels (AIN0-AIN3)
hf_adc_err_t ReadAinChannel(uint8_t ain_channel,                // ~20-100ns
                           hf_u32_t& raw_value, float& voltage) noexcept;

// Current sensing channels (I0-I3)
hf_adc_err_t ReadCurrentSenseChannel(uint8_t current_channel,   // ~25-120ns
                                    hf_u32_t& raw_value, float& voltage) noexcept;

// Voltage monitoring channels (supply, driver)
hf_adc_err_t ReadVoltageChannel(uint8_t voltage_channel,        // ~30-140ns
                               hf_u32_t& raw_value, float& voltage) noexcept;

// Temperature channels (chip, external)
hf_adc_err_t ReadTemperatureChannel(uint8_t temp_channel,       // ~35-160ns
                                   hf_u32_t& raw_value, float& voltage) noexcept;

// Motor data channels (current, velocity, position)
hf_adc_err_t ReadMotorDataChannel(uint8_t motor_channel,        // ~40-180ns
                                 hf_u32_t& raw_value, float& voltage) noexcept;
```

### BaseAdc Performance Characteristics

| Function | TMC9660 | ESP32 | Use Case |
|----------|---------|-------|----------|
| **ReadChannelV()** | ~20-100ns | ~30-120ns | Real-time feedback |
| **ReadChannelCount()** | ~15-80ns | ~25-100ns | Fastest reading |
| **ReadChannel()** | ~25-120ns | ~35-140ns | Combined data |
| **ReadAinChannel()** | ~20-100ns | N/A | External inputs |
| **ReadCurrentSenseChannel()** | ~25-120ns | N/A | Motor current |
| **ReadMotorDataChannel()** | ~40-180ns | N/A | Motor feedback |

## 🔗 Component Manager Convenience Functions

The component managers (GpioManager, AdcManager) provide higher-level convenience functions that wrap the base interfaces:

### GpioManager Convenience API
```cpp
// Manager convenience functions (with string lookup overhead)
hf_gpio_err_t Set(std::string_view name, bool value) noexcept;     // Calls gpio->SetActive()/SetInactive()
hf_gpio_err_t SetActive(std::string_view name) noexcept;            // Calls gpio->SetActive()
hf_gpio_err_t SetInactive(std::string_view name) noexcept;          // Calls gpio->SetInactive()
hf_gpio_err_t Read(std::string_view name, bool& state) noexcept;    // Calls gpio->IsActive()
hf_gpio_err_t Toggle(std::string_view name) noexcept;               // Calls gpio->Toggle()

// Direct access to BaseGpio (for caching)
std::shared_ptr<BaseGpio> Get(std::string_view name) noexcept;
```

### AdcManager Convenience API
```cpp
// Manager convenience functions (with string lookup overhead)
hf_adc_err_t ReadVoltage(std::string_view name, float& voltage,
                        hf_u8_t numOfSamplesToAvg = 1) noexcept;

hf_adc_err_t ReadChannelCount(std::string_view name, hf_u32_t& value,
                             hf_u8_t numOfSamplesToAvg = 1,
                             hf_time_t timeBetweenSamples = 0) noexcept;

// Direct access to BaseAdc (for caching)
BaseAdc* Get(std::string_view name) noexcept;
```

## 🚀 Cached Access Implementation Patterns

### High-Performance GPIO Control
```cpp
class HighPerformanceGpioControl {
private:
    std::shared_ptr<BaseGpio> enable_pin_;
    std::shared_ptr<BaseGpio> direction_pin_;
    std::shared_ptr<BaseGpio> step_pin_;
    
public:
    bool Initialize() {
        // Cache GPIO pointers once
        enable_pin_ = vortex.gpio.Get("MOTOR_ENABLE");
        direction_pin_ = vortex.gpio.Get("MOTOR_DIRECTION");
        step_pin_ = vortex.gpio.Get("MOTOR_STEP");
        
        return enable_pin_ && direction_pin_ && step_pin_;
    }
    
    void HighFrequencyControl() {
        // Ultra-fast pin operations using BaseGpio interface
        while (running_) {
            // Direct BaseGpio calls - maximum performance
            enable_pin_->SetActive();                           // ~15-50ns
            direction_pin_->SetActive();                        // ~15-50ns
            
            // Generate step pulse
            step_pin_->SetActive();                             // ~15-50ns
            DelayNanoseconds(1000);  // 1µs pulse
            step_pin_->SetInactive();                           // ~15-50ns
            
            DelayMicroseconds(100);  // 10kHz step rate
        }
    }
};
```

### High-Performance ADC Sampling
```cpp
class HighPerformanceAdcSampling {
private:
    BaseAdc* current_sensor_;
    BaseAdc* velocity_sensor_;
    BaseAdc* position_sensor_;
    
public:
    bool Initialize() {
        // Cache ADC pointers once
        current_sensor_ = vortex.adc.Get("TMC9660_CURRENT_I0");
        velocity_sensor_ = vortex.adc.Get("TMC9660_MOTOR_VELOCITY");
        position_sensor_ = vortex.adc.Get("TMC9660_MOTOR_POSITION");
        
        return current_sensor_ && velocity_sensor_ && position_sensor_;
    }
    
    void HighFrequencySampling() {
        // Ultra-fast ADC sampling using BaseAdc interface
        std::array<float, 3> readings;
        
        while (sampling_) {
            // Direct BaseAdc calls - maximum performance
            current_sensor_->ReadChannelV(0, readings[0]);      // ~20-100ns
            velocity_sensor_->ReadChannelV(0, readings[1]);     // ~20-100ns
            position_sensor_->ReadChannelV(0, readings[2]);     // ~20-100ns
            
            // Process readings immediately
            ProcessControlLoop(readings);
            
            DelayMicroseconds(100);  // 10kHz sampling rate
        }
    }
};
```

## ⚠️ Key Differences: Base Interfaces vs Component Managers

### GPIO Access Patterns
1. **Component Manager**: `gpio_manager.SetPin("PIN_NAME", true)` - String lookup + BaseGpio call
2. **Cached BaseGpio**: `cached_pin->SetActive()` - Direct BaseGpio call
3. **No Direct Low-Level Access**: `SetPinLevelImpl()/GetPinLevelImpl()` are protected virtual functions

### ADC Access Patterns
1. **Component Manager**: `adc_manager.ReadVoltage("CHANNEL_NAME", voltage)` - String lookup + BaseAdc call  
2. **Cached BaseAdc**: `cached_adc->ReadChannelV(channel_id, voltage)` - Direct BaseAdc call
3. **Channel ID Required**: BaseAdc functions need `hf_channel_id_t`, not just voltage reference

### Important Limitations
- **No Public Pin Level Access**: Cannot directly call `SetPinLevel()`/`GetPinLevel()` - these are protected implementation details
- **Channel ID Management**: BaseAdc requires proper `hf_channel_id_t` values for each implementation
- **Virtual Function Overhead**: Some functions may return `NOT_SUPPORTED` depending on implementation
- **Error Handling**: BaseAdc/BaseGpio return error codes, manager convenience functions may use bool returns

## 📚 Interface Design Philosophy

### BaseGpio Design
- **Hardware Agnostic**: Same interface across ESP32, PCAL95555, and TMC9660
- **State-Based**: Works with logical Active/Inactive states with configurable polarity
- **Protected Low-Level**: Electrical level operations are implementation details
- **Feature Optional**: Interrupts and diagnostics may not be supported on all hardware

### BaseAdc Design
- **Channel-Based**: Operations require explicit channel IDs
- **Multi-Sample Support**: Built-in averaging and timing control
- **Error-Safe**: Comprehensive error code returns
- **Hardware-Specific Extensions**: TMC9660 provides specialized channel reading functions

### Performance Optimization Strategy
1. **Cache Interface Pointers**: Store BaseGpio*/BaseAdc* for direct access
2. **Use High-Level Functions**: Prefer SetActive()/IsActive() and ReadChannelV()
3. **Validate Once**: Check cached pointers during initialization, not in loops
4. **Handle Channel IDs**: Understand the channel mapping for your hardware
5. **Error Handling**: Always check return codes for BaseAdc/BaseGpio operations

This base interface reference provides accurate information about the actual hardware abstraction layer available for cached access, enabling developers to write high-performance code while understanding the real limitations and capabilities of the system.