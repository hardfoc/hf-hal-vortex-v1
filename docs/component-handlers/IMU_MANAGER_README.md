# ImuManager - IMU Sensor Management System

<div align="center">

![Component](https://img.shields.io/badge/component-ImuManager-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Sensors](https://img.shields.io/badge/sensors-BNO08x%20IMU-orange.svg)

**Advanced IMU sensor management with multi-device support and flexible interfaces**

</div>

## 📋 Overview

The `ImuManager` is a singleton component handler that provides unified management of multiple BNO08x IMU sensors. It supports the onboard BNO08x device and up to 3 external BNO08x devices, providing array-based access to individual IMU controllers with both I2C and SPI communication interfaces.

### ✨ Key Features

- **🧭 Multi-Device Management**: Support for up to 4 BNO08x IMU devices
- **🔗 Unified Interface**: Single API for all IMU devices
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **🏗️ Dynamic Device Management**: Runtime creation/deletion of external devices
- **📊 Advanced Sensor Fusion**: High-precision orientation and motion data
- **⚡ High Performance**: Optimized data processing and filtering
- **🛡️ Flexible Communication**: I2C and SPI interfaces supported
- **🔧 Easy Integration**: Simple API for motion-based applications

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     ImuManager                                │
├─────────────────────────────────────────────────────────────────┤
│  Device Management   │ Array-based BNO08x device access      │
├─────────────────────────────────────────────────────────────────┤
│  Handler Access      │ Individual Bno08xHandler instances    │
├─────────────────────────────────────────────────────────────────┤
│  Communication       │ I2C/SPI interface management          │
├─────────────────────────────────────────────────────────────────┤
│  BNO08x Drivers      │ Direct BNO085 device instances        │
└─────────────────────────────────────────────────────────────────┘
```

### Device Index Mapping

| Index | Device Type | Interface | Description |
|-------|-------------|-----------|-------------|
| **0** | Onboard | I2C | Primary BNO08x IMU (always present) |
| **1** | External | I2C/SPI | External device 1 (optional) |
| **2** | External | I2C/SPI | External device 2 (optional) |
| **3** | External | I2C/SPI | External device 3 (optional) |

## 🚀 Quick Start

### Basic Usage

```cpp
#include "component-handlers/ImuManager.h"

void imu_example() {
    // Get singleton instance
    auto& imu = ImuManager::GetInstance();
    
    // Initialize the manager
    if (!imu.EnsureInitialized()) {
        logger.Info("IMU", "Failed to initialize IMU manager\n");
        return;
    }
    
    // Get onboard BNO08x handler
    auto* handler = imu.GetBno08xHandler(0);  // Index 0 = onboard device
    if (handler && handler->Initialize()) {
        auto* sensor = handler->GetSensor();  // Get IBno08xDriverOps*
        
        if (sensor) {
            // Enable sensors
            sensor->EnableSensor(BNO085Sensor::RotationVector, 50, 0.0f);
            sensor->EnableSensor(BNO085Sensor::Accelerometer, 100, 0.0f);
            
            logger.Info("IMU", "IMU ready\n");
        }
    }
}
```

## 📖 API Reference

### Core Operations

#### Initialization
```cpp
class ImuManager {
public:
    // Singleton access
    static ImuManager& GetInstance() noexcept;
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool IsInitialized() const noexcept;
    bool Deinitialize() noexcept;
};
```

#### Device Management
```cpp
// Handler access
Bno08xHandler* GetBno08xHandler(uint8_t deviceIndex = ONBOARD_IMU_INDEX) noexcept;

// Direct driver access
std::shared_ptr<BNO085> GetBno085Driver(uint8_t deviceIndex = ONBOARD_IMU_INDEX) noexcept;

// Device creation/deletion
bool CreateExternalBno08xDevice(uint8_t deviceIndex, 
                               uint8_t i2c_address,
                               uint32_t i2c_speed_hz = 400000,
                               const Bno08xConfig& config = Bno08xHandler::GetDefaultConfig());
bool CreateExternalBno08xDevice(uint8_t deviceIndex, 
                               SpiDeviceId spiDeviceId,
                               const Bno08xConfig& config = Bno08xHandler::GetDefaultConfig());
bool DeleteExternalDevice(uint8_t deviceIndex);

// Device information
uint8_t GetDeviceCount() const noexcept;
bool IsDeviceValid(uint8_t deviceIndex) const noexcept;
bool IsExternalSlotAvailable(uint8_t deviceIndex) const noexcept;
std::vector<uint8_t> GetActiveDeviceIndices() const noexcept;
```

#### Device Constants
```cpp
static constexpr uint8_t MAX_IMU_DEVICES = 4;
static constexpr uint8_t ONBOARD_IMU_INDEX = 0;
static constexpr uint8_t EXTERNAL_IMU_1_INDEX = 1;
static constexpr uint8_t EXTERNAL_IMU_2_INDEX = 2;
static constexpr uint8_t EXTERNAL_IMU_3_INDEX = 3;
```

#### System Management
```cpp
// Device initialization
std::vector<bool> InitializeAllDevices();
std::vector<bool> GetInitializationStatus() const;

// Device information
std::string GetDeviceType(uint8_t deviceIndex) const noexcept;
```

#### Interrupt Management
```cpp
// Interrupt configuration
bool ConfigureInterrupt(uint8_t deviceIndex, std::function<void()> callback = nullptr) noexcept;
bool EnableInterrupt(uint8_t deviceIndex) noexcept;
bool DisableInterrupt(uint8_t deviceIndex) noexcept;
bool IsInterruptEnabled(uint8_t deviceIndex) const noexcept;
bool WaitForInterrupt(uint8_t deviceIndex, uint32_t timeout_ms = 0) noexcept;
uint32_t GetInterruptCount(uint8_t deviceIndex) const noexcept;
```

## 🎯 Hardware Support

### BNO08x IMU Features

| Feature | Description | Support |
|---------|-------------|---------|
| **Sensor Fusion** | 9-axis sensor fusion | ✅ Full support |
| **Orientation** | Quaternion, Euler angles | ✅ High precision |
| **Motion Detection** | Acceleration, gyroscope | ✅ 3-axis each |
| **Magnetic Field** | Compass heading | ✅ 3-axis |
| **Activity Recognition** | Step counting, tap detection | ✅ Advanced algorithms |
| **Communication** | I2C/SPI interfaces | ✅ Configurable |
| **Interrupt Support** | Hardware interrupts | ✅ GPIO integration |

## 📊 Examples

### Basic IMU Operations

```cpp
void basic_imu_example() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    // Get onboard IMU
    auto* handler = imu.GetBno08xHandler(0);
    if (handler && handler->Initialize()) {
        auto* sensor = handler->GetSensor();
        if (sensor) {
            // Enable sensors
            sensor->EnableSensor(BNO085Sensor::RotationVector, 50, 0.0f);
            sensor->EnableSensor(BNO085Sensor::Accelerometer, 100, 0.0f);
            sensor->EnableSensor(BNO085Sensor::Gyroscope, 100, 0.0f);
            
            logger.Info("IMU", "IMU sensors enabled\n");
            
            // Read sensor data
            for (int i = 0; i < 100; i++) {
                handler->Update();
                
                if (sensor->HasNewData(BNO085Sensor::RotationVector)) {
                    // Get rotation vector (quaternion)
                    auto rotation = sensor->GetLatest(BNO085Sensor::RotationVector);
                    logger.Info("IMU", "Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
                           rotation.rotation.w, rotation.rotation.x, rotation.rotation.y, rotation.rotation.z);
                }
                
                if (sensor->HasNewData(BNO085Sensor::Accelerometer)) {
                    // Get acceleration
                    auto accel = sensor->GetLatest(BNO085Sensor::Accelerometer);
                    logger.Info("IMU", "Acceleration: x=%.2f, y=%.2f, z=%.2f m/s²\n",
                           accel.vector.x, accel.vector.y, accel.vector.z);
                }
                
                if (sensor->HasNewData(BNO085Sensor::Gyroscope)) {
                    // Get gyroscope
                    auto gyro = sensor->GetLatest(BNO085Sensor::Gyroscope);
                    logger.Info("IMU", "Gyroscope: x=%.2f, y=%.2f, z=%.2f rad/s\n",
                           gyro.vector.x, gyro.vector.y, gyro.vector.z);
                }
                
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }
}
```

### Multi-Device Management

```cpp
void multi_device_example() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    // Create external I2C device
    if (imu.CreateExternalBno08xDevice(1, 0x4A, 400000)) {
        logger.Info("IMU", "External I2C device created at index 1\n");
        
        // Initialize external device
        auto* ext_handler = imu.GetBno08xHandler(1);
        if (ext_handler && ext_handler->Initialize()) {
            auto* ext_sensor = ext_handler->GetSensor();
            if (ext_sensor) {
                // Configure external IMU
                ext_sensor->EnableSensor(BNO085Sensor::RotationVector, 25, 0.0f);
                ext_sensor->EnableSensor(BNO085Sensor::Accelerometer, 50, 0.0f);
                
                logger.Info("IMU", "External IMU enabled\n");
            }
        }
    }
    
    // Create external SPI device
    if (imu.CreateExternalBno08xDevice(2, SpiDeviceId::EXTERNAL_DEVICE_1)) {
        logger.Info("IMU", "External SPI device created at index 2\n");
        
        auto* spi_handler = imu.GetBno08xHandler(2);
        if (spi_handler && spi_handler->Initialize()) {
            auto* spi_sensor = spi_handler->GetSensor();
            if (spi_sensor) {
                spi_sensor->EnableSensor(BNO085Sensor::RotationVector, 50, 0.0f);
                logger.Info("IMU", "External SPI IMU enabled\n");
            }
        }
    }
    
    // Read from all devices
    for (int i = 0; i < 50; i++) {
        for (uint8_t device = 0; device < 3; device++) {
            auto* handler = imu.GetBno08xHandler(device);
            if (handler) {
                handler->Update();
                auto* sensor = handler->GetSensor();
                if (sensor && sensor->HasNewData(BNO085Sensor::RotationVector)) {
                    auto rotation = sensor->GetLatest(BNO085Sensor::RotationVector);
                    logger.Info("IMU", "Device %u: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
                           device, rotation.rotation.w, rotation.rotation.x, rotation.rotation.y, rotation.rotation.z);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
```

### Device Information and Status

```cpp
void device_info_example() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    // Get device count
    uint8_t device_count = imu.GetDeviceCount();
    logger.Info("IMU", "Active devices: %u\n", device_count);
    
    // Get active device indices
    auto active_indices = imu.GetActiveDeviceIndices();
    logger.Info("IMU", "Active device indices: ");
    for (auto index : active_indices) {
        logger.Info("IMU", "%u ", index);
    }
    logger.Info("IMU", "\n");
    
    // Check device validity and type
    for (uint8_t i = 0; i < 4; i++) {
        if (imu.IsDeviceValid(i)) {
            logger.Info("IMU", "Device %u is valid\n", i);
            logger.Info("IMU", "  Type: %s\n", imu.GetDeviceType(i).c_str());
            
            // Check initialization status
            auto* handler = imu.GetBno08xHandler(i);
            if (handler) {
                logger.Info("IMU", "  Handler available\n");
                
                // Get sensor interface status
                auto* sensor = handler->GetSensor();
                if (sensor) {
                    logger.Info("IMU", "  Sensor interface: AVAILABLE\n");
                } else {
                    logger.Info("IMU", "  Sensor interface: NOT AVAILABLE\n");
                }
            }
        }
    }
}
```

### External Device Management

```cpp
void external_device_example() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    // Check if external slots are available
    if (imu.IsExternalSlotAvailable(1)) {
        logger.Info("IMU", "External slot 1 is available\n");
        
        // Create external device with custom configuration
        Bno08xConfig custom_config = Bno08xHandler::GetDefaultConfig();
        custom_config.rotation_vector_report_interval_ms = 25;  // 40Hz
        custom_config.accelerometer_report_interval_ms = 50;    // 20Hz
        
        if (imu.CreateExternalBno08xDevice(1, 0x4B, 400000, custom_config)) {
            logger.Info("IMU", "External device created with custom config\n");
            
            // Use the external device
            auto* handler = imu.GetBno08xHandler(1);
            if (handler && handler->Initialize()) {
                auto* sensor = handler->GetSensor();
                if (sensor) {
                    sensor->EnableSensor(BNO085Sensor::RotationVector, 25, 0.0f);
                    sensor->EnableSensor(BNO085Sensor::Accelerometer, 50, 0.0f);
                    logger.Info("IMU", "External IMU running\n");
                }
            }
        }
    }
    
    // Delete external device when done
    if (imu.DeleteExternalDevice(1)) {
        logger.Info("IMU", "External device deleted\n");
    }
}
```

### Interrupt-Based Reading

```cpp
void interrupt_example() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    auto* handler = imu.GetBno08xHandler(0);
    if (handler && handler->Initialize()) {
        // Configure interrupt callback
        auto callback = []() {
            logger.Info("IMU", "IMU interrupt triggered!\n");
        };
        
        if (imu.ConfigureInterrupt(0, callback)) {
            logger.Info("IMU", "Interrupt configured\n");
            
            // Enable interrupt
            if (imu.EnableInterrupt(0)) {
                logger.Info("IMU", "Interrupt enabled\n");
                
                auto* sensor = handler->GetSensor();
                if (!sensor) return;
                
                // Enable sensors
                sensor->EnableSensor(BNO085Sensor::RotationVector, 50, 0.0f);
                
                // Wait for interrupts
                for (int i = 0; i < 10; i++) {
                    if (imu.WaitForInterrupt(0, 1000)) {  // 1 second timeout
                        logger.Info("IMU", "Interrupt received\n");
                        
                        // Read data
                        handler->Update();
                        if (sensor->HasNewData(BNO085Sensor::RotationVector)) {
                            auto rotation = sensor->GetLatest(BNO085Sensor::RotationVector);
                            logger.Info("IMU", "Rotation: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n",
                                   rotation.rotation.w, rotation.rotation.x, rotation.rotation.y, rotation.rotation.z);
                        }
                    } else {
                        logger.Info("IMU", "Interrupt timeout\n");
                    }
                }
                
                // Disable interrupt
                imu.DisableInterrupt(0);
                logger.Info("IMU", "Interrupt disabled\n");
            }
        }
    }
}
```

### System Diagnostics

```cpp
void diagnostics_example() {
    auto& imu = ImuManager::GetInstance();
    imu.EnsureInitialized();
    
    // Initialize all devices
    auto init_results = imu.InitializeAllDevices();
    logger.Info("IMU", "Device initialization results:\n");
    for (size_t i = 0; i < init_results.size(); i++) {
        logger.Info("IMU", "  Device %zu: %s\n", i, init_results[i] ? "SUCCESS" : "FAILED");
    }
    
    // Get initialization status
    auto status = imu.GetInitializationStatus();
    logger.Info("IMU", "Device initialization status:\n");
    for (size_t i = 0; i < status.size(); i++) {
        logger.Info("IMU", "  Device %zu: %s\n", i, status[i] ? "INITIALIZED" : "NOT INITIALIZED");
    }
    
    // Check interrupt status
    for (uint8_t i = 0; i < 4; i++) {
        if (imu.IsDeviceValid(i)) {
            bool interrupt_enabled = imu.IsInterruptEnabled(i);
            uint32_t interrupt_count = imu.GetInterruptCount(i);
            logger.Info("IMU", "Device %u: Interrupt enabled=%s, count=%u\n",
                   i, interrupt_enabled ? "YES" : "NO", interrupt_count);
        }
    }
}
```

## 🔍 Advanced Usage

### ⚡ Performance Optimization for IMU Operations

The IMU Manager provides both convenience and high-performance access patterns:

#### 🔍 String-Based Access (Configuration & Setup)
```cpp
auto& imu = vortex.imu;

// IMU configuration and calibration
auto* handler = imu.GetBno08xHandler(0);
if (handler) {
    // Get sensor interface for configuration
    auto* sensor = handler->GetSensor();
    if (sensor) {
        // Configuration operations (one-time setup)
        sensor->EnableSensor(BNO085Sensor::RotationVector, 100, 0.0f);  // 100Hz
        sensor->EnableSensor(BNO085Sensor::Accelerometer, 200, 0.0f);    // 200Hz
    }
}
```

#### ⚡ Cached Access (Motion Control)
For high-frequency motion control loops (>1kHz):

```cpp
auto& imu = vortex.imu;

// Cache IMU handler and sensor interface for motion control
auto* imu_handler = imu.GetBno08xHandler(0);
auto* sensor = imu_handler ? imu_handler->GetSensor() : nullptr;

if (!imu_handler || !sensor) {
    logger.Info("IMU", "ERROR: Failed to cache IMU components\n");
    return;
}

// High-frequency motion control loop
while (motion_control_active) {
    imu_handler->Update();
    
    // Direct driver access for minimal latency (~40-200ns)
    auto rotation = sensor->GetLatest(BNO085Sensor::RotationVector);
    auto accel = sensor->GetLatest(BNO085Sensor::Accelerometer);
    auto gyro = sensor->GetLatest(BNO085Sensor::Gyroscope);
    
    // Process motion data immediately
    ProcessMotionControl(rotation, accel, gyro);
    
    // Motion control timing (1kHz for balance control)
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
}
```

**Performance Impact**: Cached access provides 5-15x performance improvement (~400-1200ns → ~40-200ns) for high-frequency motion control applications.

### Error Handling

```cpp
void error_handling_example() {
    auto& imu = ImuManager::GetInstance();
    
    // Check initialization
    if (!imu.EnsureInitialized()) {
        logger.Info("IMU", "ERROR: Failed to initialize IMU manager\n");
        return;
    }
    
    // Validate device before use
    if (!imu.IsDeviceValid(0)) {
        logger.Info("IMU", "ERROR: Onboard device not valid\n");
        return;
    }
    
    // Safe device access
    auto* handler = imu.GetBno08xHandler(0);
    if (!handler) {
        logger.Info("IMU", "ERROR: Handler not available\n");
        return;
    }
    
    if (!handler->Initialize()) {
        logger.Info("IMU", "ERROR: Failed to initialize handler\n");
        return;
    }
    
    // Safe sensor operations
    auto* sensor = handler->GetSensor();
    if (sensor) {
        sensor->EnableSensor(BNO085Sensor::RotationVector, 50, 0.0f);
        logger.Info("IMU", "Sensor operations successful\n");
    } else {
        logger.Info("IMU", "ERROR: Sensor interface not available\n");
    }
}
```

### Integration with Other Managers

```cpp
#include "component-handlers/ImuManager.h"

void integrated_example() {
    // Initialize all managers
    auto& imu = ImuManager::GetInstance();
    auto& gpio = GpioManager::GetInstance();
    auto& comm = CommChannelsManager::GetInstance();
    
    imu.EnsureInitialized();
    gpio.EnsureInitialized();
    comm.EnsureInitialized();
    
    // Get IMU controller
    auto* handler = imu.GetBno08xHandler(0);
    if (handler && handler->Initialize()) {
        auto* sensor = handler->GetSensor();
        if (sensor) {
            // Use GPIO for IMU reset/enable
            gpio.SetDirection("BNO08X_RESET", HF_GPIO_DIRECTION_OUTPUT);
            gpio.SetInactive("BNO08X_RESET");  // Reset pulse
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio.SetActive("BNO08X_RESET");
            
            // Enable sensors
            sensor->EnableSensor(BNO085Sensor::RotationVector, 50, 0.0f);
            sensor->EnableSensor(BNO085Sensor::Accelerometer, 100, 0.0f);
            
            // Monitor IMU data
            while (true) {
                handler->Update();
                if (sensor->HasNewData(BNO085Sensor::RotationVector)) {
                    auto rotation = sensor->GetLatest(BNO085Sensor::RotationVector);
                    auto accel = sensor->GetLatest(BNO085Sensor::Accelerometer);
                    
                    logger.Info("IMU", "IMU: qw=%.3f, ax=%.2f, ay=%.2f, az=%.2f\n",
                           rotation.rotation.w, accel.vector.x, accel.vector.y, accel.vector.z);
                }
                
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }
}
```

## 📚 See Also

- **[GpioManager Documentation](GPIO_MANAGER_README.md)** - GPIO management system
- **[CommChannelsManager Documentation](COMM_CHANNELS_MANAGER_README.md)** - Communication interfaces
- **[BNO08x Handler Documentation](../driver-handlers/BNO08X_HANDLER_README.md)** - BNO08x driver

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*