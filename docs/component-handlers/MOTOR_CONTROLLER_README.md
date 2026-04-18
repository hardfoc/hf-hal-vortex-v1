# MotorController - TMC9660 Motor Control Management System

<div align="center">

![Component](https://img.shields.io/badge/component-MotorController-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Hardware](https://img.shields.io/badge/hardware-TMC9660-orange.svg)

**Advanced TMC9660 motor controller management system for the HardFOC platform**

</div>

## 📋 Overview

The `MotorController` is a singleton component handler that provides unified management of multiple TMC9660 motor controllers. It supports the onboard TMC9660 device and up to 2 external TMC9660 devices, providing array-based access to individual motor controllers and their associated GPIO/ADC resources.

### ✨ Key Features

- **🎛️ Multi-Device Management**: Support for up to 4 TMC9660 controllers
- **🔗 Unified Interface**: Single API for all motor controllers
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **🏗️ Dynamic Device Management**: Runtime creation/deletion of external devices
- **📊 Advanced Diagnostics**: Per-device health monitoring and statistics
- **⚡ High-Performance Control**: Optimized motor control operations
- **🛡️ Safety Features**: Parameter-based fault monitoring and protection
- **🔌 Flexible Communication**: SPI and UART interfaces supported

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     MotorController                            │
├─────────────────────────────────────────────────────────────────┤
│  Device Management   │ Array-based TMC9660 device access      │
├─────────────────────────────────────────────────────────────────┤
│  Handler Access      │ Individual Tmc9660Handler instances    │
├─────────────────────────────────────────────────────────────────┤
│  Communication       │ SPI/UART interface management          │
├─────────────────────────────────────────────────────────────────┤
│  TMC9660 Drivers     │ Parameter-based TMCL motor control     │
└─────────────────────────────────────────────────────────────────┘
```

### Device Index Mapping

| Index | Device Type | CS Line | Description |
|-------|-------------|---------|-------------|
| **0** | Onboard | SPI2_CS_TMC9660 | Primary motor controller (always present) |
| **1** | Reserved | - | Reserved for future use |
| **2** | External | EXT_GPIO_CS_1 | External device 1 (optional) |
| **3** | External | EXT_GPIO_CS_2 | External device 2 (optional) |

## 🚀 Quick Start

### Basic Usage

```cpp
#include "component-handlers/MotorController.h"

void motor_example() {
    // Get singleton instance
    auto& motor = MotorController::GetInstance();
    
    // Initialize the manager (automatically creates onboard TMC9660)
    if (!motor.EnsureInitialized()) {
        logger.Info("MOTOR", "Failed to initialize motor controller\n");
        return;
    }
    
    // Get onboard TMC9660 handler
    auto* handler = motor.handler(0);  // Index 0 = onboard device
    if (!handler) {
        logger.Info("MOTOR", "Handler not available\n");
        return;
    }
    
    // Initialize the handler (creates TMC9660 driver instance)
    if (!handler->Initialize()) {
        logger.Info("MOTOR", "Failed to initialize TMC9660 handler\n");
        return;
    }
    
    // Get the TMC9660 driver instance
    auto tmc = motor.driver(0);
    if (!tmc) {
        logger.Info("MOTOR", "TMC9660 driver not available\n");
        return;
    }
    
    // CRITICAL: Bootloader initialization (REQUIRED before any motor operations)
    tmc9660::BootloaderConfig cfg{};
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;  // Essential!
    cfg.boot.start_motor_control = true;
    
    auto boot_result = tmc->bootloaderInit(&cfg);
    if (boot_result != TMC9660::BootloaderInitResult::Success) {
        logger.Info("MOTOR", "Failed to initialize TMC9660 bootloader\n");
        return;
    }
    
    // Now can use TMC9660 parameter-based API
    if (tmc->setTargetVelocity(1000)) {  // Set 1000 internal velocity units
        logger.Info("MOTOR", "Target velocity set\n");
    }
    
    // Set torque using FOC control structure
    if (tmc->focControl.setTargetTorque(500)) {  // 500 mA
        logger.Info("MOTOR", "Target torque set\n");
    }
    
    logger.Info("MOTOR", "Motor controller initialized and ready\n");
}
```

## 📖 API Reference

### Core Operations

#### Initialization
```cpp
class MotorController {
public:
    // Singleton access
    static MotorController& GetInstance();
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool IsInitialized() const noexcept;
    
    // Device access
    Tmc9660Handler* handler(uint8_t deviceIndex = 0) noexcept;
    std::shared_ptr<TMC9660> driver(uint8_t deviceIndex = 0) noexcept;
    
    // Device management (returns typed MotorError)
    MotorError CreateExternalDevice(uint8_t csDeviceIndex, SpiDeviceId spiDeviceId, 
                            uint8_t address, const tmc9660::BootloaderConfig* bootCfg = nullptr);
    MotorError DeleteExternalDevice(uint8_t csDeviceIndex);
    
    // Error & diagnostics
    MotorError GetLastError() const noexcept;
    MotorError GetSystemDiagnostics(MotorSystemDiagnostics& diagnostics) const noexcept;
    
    // Status and diagnostics
    uint8_t GetDeviceCount() const noexcept;
    bool IsDeviceValid(uint8_t deviceIndex) const noexcept;
    std::vector<uint8_t> GetActiveDeviceIndices() const noexcept;
    std::vector<bool> InitializeAllDevices();
    void DumpStatistics() const noexcept;
};
```

### TMC9660 Driver Interface

#### Core Motor Control Functions
```cpp
// Velocity control (parameter-based TMCL API)
bool setTargetVelocity(int32_t velocity) noexcept;         // Set target velocity
bool getActualVelocity(int32_t& velocity) noexcept;        // Read actual velocity

// Position control
bool getActualPosition(int32_t& position) noexcept;        // Read actual position

// FOC Control structure
struct FOCControl {
    bool stop() noexcept;                                   // Stop motor (SYSTEM_OFF)
    bool setTargetTorque(int16_t milliamps) noexcept;      // Set torque in mA
    bool getActualTorque(int16_t& milliamps) noexcept;     // Read actual torque
    bool setTargetFlux(int16_t milliamps) noexcept;        // Set flux current
    bool getActualFlux(int16_t& milliamps) noexcept;       // Read actual flux
} focControl;

// Bootloader initialization (MANDATORY before any motor operations)
enum class BootloaderInitResult { Success, NoConfig, Failure };
BootloaderInitResult bootloaderInit(const tmc9660::BootloaderConfig* cfg) noexcept;
```

#### Communication Interface
```cpp
// Communication mode management
CommMode GetCommMode() const noexcept;          // Get active communication mode
bool SwitchCommInterface(CommMode mode);        // Switch between SPI/UART
TMC9660CommInterface& comm() noexcept;          // Get communication interface
```

## 💡 Usage Examples

### Single Motor Control

```cpp
void single_motor_control() {
    auto& motor = MotorController::GetInstance();
    
    if (!motor.EnsureInitialized()) {
        logger.Info("MOTOR", "Failed to initialize motor controller\n");
        return;
    }
    
    auto* handler = motor.handler(0);
    if (!handler || !handler->Initialize()) {
        logger.Info("MOTOR", "Failed to initialize TMC9660 handler\n");
        return;
    }
    
    auto tmc = motor.driver(0);
    if (!tmc) {
        logger.Info("MOTOR", "TMC9660 driver not available\n");
        return;
    }
    
    // CRITICAL: Bootloader initialization
    tmc9660::BootloaderConfig cfg{};
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    cfg.boot.start_motor_control = true;
    
    if (tmc->bootloaderInit(&cfg) != TMC9660::BootloaderInitResult::Success) {
        logger.Info("MOTOR", "Bootloader initialization failed\n");
        return;
    }
    
    // Start motor control
    if (tmc->setTargetVelocity(500)) {  // 500 internal velocity units
        logger.Info("MOTOR", "Velocity set to 500\n");
    }
    
    // Set torque limit
    if (tmc->focControl.setTargetTorque(1000)) {  // 1000 mA
        logger.Info("MOTOR", "Torque limit set to 1000 mA\n");
    }
    
    // Read feedback
    int32_t actual_velocity;
    if (tmc->getActualVelocity(actual_velocity)) {
        logger.Info("MOTOR", "Actual velocity: %ld\n", actual_velocity);
    }
    
    int32_t actual_position;
    if (tmc->getActualPosition(actual_position)) {
        logger.Info("MOTOR", "Actual position: %ld\n", actual_position);
    }
    
    // Stop motor
    if (tmc->focControl.stop()) {
        logger.Info("MOTOR", "Motor stopped\n");
    }
}
```

### Multi-Device Management

```cpp
void multi_device_example() {
    auto& motor = MotorController::GetInstance();
    
    if (!motor.EnsureInitialized()) {
        logger.Info("MOTOR", "Failed to initialize motor controller\n");
        return;
    }
    
    // Create external TMC9660 device
    MotorError err = motor.CreateExternalDevice(2, SpiDeviceId::EXTERNAL_DEVICE_1, 0x01);
    if (err == MotorError::kSuccess) {
        logger.Info("MOTOR", "External device created on CS_1\n");
        
        // Initialize external device
        auto* ext_handler = motor.handler(2);
        if (ext_handler && ext_handler->Initialize()) {
            auto ext_tmc = motor.driver(2);
            if (ext_tmc) {
                // Initialize external device bootloader
                tmc9660::BootloaderConfig cfg{};
                cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
                cfg.boot.start_motor_control = true;
                
                if (ext_tmc->bootloaderInit(&cfg) == TMC9660::BootloaderInitResult::Success) {
                    // Control external motor
                    ext_tmc->setTargetVelocity(1000);  // 1000 internal units
                    logger.Info("MOTOR", "External motor configured\n");
                }
            }
        }
    }
    
    // Control multiple motors
    auto onboard_tmc = motor.driver(0);
    auto external_tmc = motor.driver(2);
    
    if (onboard_tmc && external_tmc) {
        // Synchronized control
        onboard_tmc->setTargetVelocity(750);
        external_tmc->setTargetVelocity(750);
        logger.Info("MOTOR", "Both motors set to 750 velocity units\n");
    }
}
```

### TMC9660 GPIO and ADC Access

```cpp
void tmc9660_io_example() {
    auto& motor = MotorController::GetInstance();
    
    if (!motor.EnsureInitialized()) {
        return;
    }
    
    auto* handler = motor.handler(0);
    if (!handler || !handler->Initialize()) {
        return;
    }
    
    // Access TMC9660 internal GPIO (GPIO17, GPIO18)
    auto& gpio17 = handler->gpio(17);  // BaseGpio interface
    auto& gpio18 = handler->gpio(18);  // BaseGpio interface
    
    // Configure GPIO as outputs
    gpio17.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    gpio18.SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    
    // Control GPIO states
    gpio17.SetActive();     // Set GPIO17 active
    gpio18.SetInactive();   // Set GPIO18 inactive
    
    // Read GPIO states
    bool gpio17_state;
    if (gpio17.IsActive(gpio17_state) == hf_gpio_err_t::GPIO_SUCCESS) {
        logger.Info("MOTOR", "GPIO17 state: %s\n", gpio17_state ? "Active" : "Inactive");
    }
    
    // Access TMC9660 ADC
    auto& adc = handler->adc();  // BaseAdc interface
    
    // Read ADC channels (requires channel IDs)
    float voltage;
    if (adc.ReadVoltage(0, voltage) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info("MOTOR", "ADC Channel 0: %.3f V\n", voltage);
    }
    
    // Read multiple ADC channels
    std::array<hf_channel_id_t, 3> channels = {0, 1, 2};
    std::array<hf_u32_t, 3> raw_values;
    std::array<float, 3> voltages;
    
    if (adc.ReadMultipleChannels(channels.data(), 3, raw_values.data(), voltages.data()) == hf_adc_err_t::ADC_SUCCESS) {
        for (int i = 0; i < 3; i++) {
            logger.Info("MOTOR", "ADC Channel %d: %lu counts, %.3f V\n", i, raw_values[i], voltages[i]);
        }
    }
}
```

## ⚡ Performance Optimization for Motor Control

The motor control system provides two access patterns optimized for different use cases:

### 🔍 String-Based Access (Configuration & Setup)
```cpp
// Component manager convenience functions (with string lookup overhead)
auto& vortex = Vortex::GetInstance();
auto& motor = vortex.motors;

// String-based access for configuration (~300-1000ns)
auto* handler = motor.handler(0);    // Device index access
auto tmc = motor.driver(0);          // Driver access
```

### ⚡ Cached Access (Real-Time Control)
```cpp
class HighPerformanceMotorControl {
private:
    std::shared_ptr<TMC9660> tmc_driver_;
    BaseAdc* current_sensor_;
    BaseAdc* velocity_sensor_;
    
public:
    bool Initialize() {
        auto& motor = MotorController::GetInstance();
        
        // STEP 1: Cache TMC9660 driver pointer for fast access
        tmc_driver_ = motor.driver(0);
        if (!tmc_driver_) {
            logger.Info("MOTOR", "ERROR: TMC9660 driver not available\n");
            return false;
        }
        
        // STEP 2: Cache ADC channels for feedback
        auto& vortex = Vortex::GetInstance();
        current_sensor_ = vortex.adc.Get("TMC9660_CURRENT_I0");
        velocity_sensor_ = vortex.adc.Get("TMC9660_MOTOR_VELOCITY");
        
        if (!current_sensor_ || !velocity_sensor_) {
            logger.Info("MOTOR", "ERROR: Failed to cache motor feedback sensors\n");
            return false;
        }
        
        return true;
    }
    
    void HighFrequencyControlLoop() {
        // High-frequency motor control loop (5kHz example)
        TickType_t last_wake_time = xTaskGetTickCount();
        
        while (motor_control_active) {
            // Read motor feedback with correct BaseAdc interface (~20-100ns per reading)
            float current, velocity;
            current_sensor_->ReadChannelV(0, current);    // Channel ID required
            velocity_sensor_->ReadChannelV(0, velocity);  // BaseAdc interface
            
            // Process control algorithm
            float control_output = MotorPIDController(target_position, velocity, current);
            
            // Apply control output using cached TMC9660 driver (~20-100ns)
            tmc_driver_->setTargetVelocity(static_cast<int32_t>(control_output));
            
            // Read actual velocity for feedback
            int32_t actual_velocity;
            if (tmc_driver_->getActualVelocity(actual_velocity)) {
                // Process actual velocity feedback
                ProcessVelocityFeedback(actual_velocity);
            }
            
            // Precise timing for motor control (5kHz = 0.2ms)
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(0.2));
        }
    }
};
```

#### 📊 Performance Guidelines for Motor Control

| Operation | String Lookup | Cached Access | Use Cached For |
|-----------|---------------|---------------|----------------|
| **Driver Access** | ~50-200ns | ~20-100ns | **Real-time control** |
| **Motor Commands** | ~300-1000ns | ~20-100ns | **>1kHz control** |
| **Feedback Read** | ~400-1200ns | ~20-200ns | **>1kHz feedback** |
| **Parameter Set** | ~200-800ns | ~20-100ns | **Parameter tuning** |

#### 🎯 When to Use Each Approach

**Use MotorController String-Based API for:**
- ✅ Motor configuration and parameter setup
- ✅ Diagnostics and fault reporting
- ✅ User interface control
- ✅ One-time operations and initialization
- ✅ Control loops <100Hz

**Use Cached TMC9660 Driver Access for:**
- ⚡ Real-time motor control loops >1kHz
- ⚡ Servo control and position feedback
- ⚡ High-precision motion control
- ⚡ Parameter-based fault monitoring
- ⚡ Current limiting and torque control

### Error Handling

```cpp
void error_handling_example() {
    auto& motor = MotorController::GetInstance();
    
    // Check initialization
    if (!motor.EnsureInitialized()) {
        logger.Info("MOTOR", "ERROR: Failed to initialize motor controller\n");
        return;
    }
    
    // Validate device before use
    if (!motor.IsDeviceValid(0)) {
        logger.Info("MOTOR", "ERROR: Onboard device not valid\n");
        return;
    }
    
    // Safe device access
    auto* handler = motor.handler(0);
    if (!handler) {
        logger.Info("MOTOR", "ERROR: Handler not available\n");
        return;
    }
    
    if (!handler->Initialize()) {
        logger.Info("MOTOR", "ERROR: Failed to initialize handler\n");
        return;
    }
    
    auto tmc = motor.driver(0);
    if (!tmc) {
        logger.Info("MOTOR", "ERROR: TMC9660 driver not available\n");
        return;
    }
    
    // CRITICAL: Bootloader initialization
    tmc9660::BootloaderConfig cfg{};
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    cfg.boot.start_motor_control = true;
    
    auto boot_result = tmc->bootloaderInit(&cfg);
    if (boot_result != TMC9660::BootloaderInitResult::Success) {
        logger.Info("MOTOR", "ERROR: Bootloader initialization failed\n");
        return;
    }
    
    // Safe motor control with error checking
    if (!tmc->setTargetVelocity(500)) {
        logger.Info("MOTOR", "ERROR: Failed to set target velocity\n");
        return;
    }
    
    if (!tmc->focControl.setTargetTorque(1000)) {
        logger.Info("MOTOR", "ERROR: Failed to set target torque\n");
        return;
    }
    
    logger.Info("MOTOR", "Motor control configured successfully\n");
}
```

### MotorError Reference

| Code | Value | Description |
|------|-------|-------------|
| `SUCCESS` | 0 | Operation completed successfully |
| `NOT_INITIALIZED` | 1 | Manager not yet initialised |
| `INITIALIZATION_FAILED` | 2 | Device initialisation failed |
| `DEVICE_ALREADY_EXISTS` | 3 | A device already occupies this slot |
| `DEVICE_NOT_FOUND` | 4 | No active device at the given index |
| `INVALID_DEVICE_INDEX` | 5 | Device index out of valid range |
| `CANNOT_DELETE_ONBOARD` | 6 | Onboard device (index 0) cannot be deleted |
| `DEPENDENCY_NOT_READY` | 7 | Required dependency not initialised |
| `COMMUNICATION_FAILED` | 8 | A bus-level transfer failed |
| `INVALID_PARAMETER` | 9 | Null pointer or out-of-range argument |
| `HANDLER_CREATION_FAILED` | 10 | Heap allocation or handler init failed |
| `MUTEX_LOCK_FAILED` | 11 | RTOS mutex acquire timed out |

## 🛡️ Safety Considerations

### Required Bootloader Initialization
⚠️ **CRITICAL**: The TMC9660 requires bootloader initialization before any motor operations:

```cpp
// MANDATORY - Without this, motor control will NOT work
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;  // Essential!
cfg.boot.start_motor_control = true;
auto result = tmc->bootloaderInit(&cfg);
```

### Parameter-Based Safety
The TMC9660 uses a parameter-based TMCL API for all operations:
- Driver-level functions return `bool` (success/failure)
- Device management functions (`CreateExternalDevice`, `DeleteExternalDevice`) return typed `MotorError`
- Use `GetLastError()` to query the most recent error
- Use `GetSystemDiagnostics()` for per-device health snapshots
- Always check return values for safety-critical operations

### Communication Interface Management
- SPI interface takes precedence over UART when both are available
- Communication interfaces are bridged through TMC9660CommInterface
- Ensure proper BaseSpi/BaseUart initialization before TMC9660 use

## 📊 Diagnostics and Statistics

```cpp
void diagnostic_example() {
    auto& motor = MotorController::GetInstance();
    
    // Query last error
    MotorError last = motor.GetLastError();
    if (last != MotorError::kSuccess) {
        logger.Warn("MOTOR", "Motor controller last error: %u\n", static_cast<unsigned>(last));
    }
    
    // Get full system diagnostics
    MotorSystemDiagnostics diag{};
    if (motor.GetSystemDiagnostics(diag) == MotorError::kSuccess) {
        logger.Info("MOTOR", "System healthy: %s, active: %u, initialized: %u\n",
            diag.system_healthy ? "yes" : "no",
            diag.active_device_count, diag.initialized_device_count);
        
        // Per-device status
        for (size_t i = 0; i < 4; ++i) {
            if (diag.devices[i].active) {
                logger.Info("MOTOR", "  Device %u: init=%s, errors=%u\n",
                    static_cast<unsigned>(i),
                    diag.devices[i].initialized ? "yes" : "no",
                    diag.devices[i].error_count);
            }
        }
    }
    
    // Get active device list
    logger.Info("MOTOR", "Active devices: %d\n", motor.GetDeviceCount());
    
    auto active_indices = motor.GetActiveDeviceIndices();
    for (uint8_t index : active_indices) {
        logger.Info("MOTOR", "Device %d is active\n", index);
        
        if (motor.IsDeviceValid(index)) {
            auto* handler = motor.handler(index);
            if (handler) {
                // Dump comprehensive handler diagnostics
                handler->DumpDiagnostics();
            }
        }
    }
    
    // Dump comprehensive motor controller statistics
    motor.DumpStatistics();
}
```

## 🔗 Integration with Other Components

The MotorController integrates seamlessly with other HardFOC components:

```cpp
void integrated_example() {
    auto& vortex = Vortex::GetInstance();
    
    // Access all components
    auto& motor = vortex.motors;
    auto& gpio = vortex.gpio;
    auto& adc = vortex.adc;
    
    // Initialize motor controller
    if (!motor.EnsureInitialized()) {
        return;
    }
    
    // Use motor GPIO through GpioManager (string-based convenience)
    gpio.Set("TMC9660_GPIO17", true);
    
    // Use motor ADC through AdcManager (string-based convenience)
    float current;
    adc.ReadVoltage("TMC9660_CURRENT_I0", current);
    
    // Direct TMC9660 control (cached access)
    auto tmc = motor.driver(0);
    if (tmc) {
        tmc->setTargetVelocity(1000);
    }
}
```

---

**Note**: This documentation reflects the actual TMC9660 parameter-based TMCL API and bootloader requirements. The TMC9660 driver uses lowercase function names (`setTargetVelocity`, not `SetTargetVelocity`) and requires bootloader initialization for Parameter Mode operation before any motor control functions will work.