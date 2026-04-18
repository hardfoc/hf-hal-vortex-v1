# CommChannelsManager - Communication Interfaces Management System

<div align="center">

![Component](https://img.shields.io/badge/component-CommChannelsManager-blue.svg)
![Thread Safe](https://img.shields.io/badge/thread--safe-yes-green.svg)
![Interfaces](https://img.shields.io/badge/interfaces-SPI%20|%20I2C%20|%20UART%20|%20CAN-orange.svg)

**Comprehensive communication interface management for the HardFOC platform**

</div>

## 📋 Overview

The `CommChannelsManager` is a singleton component handler that provides unified management of all communication interfaces on the HardFOC platform. It handles SPI, I2C, UART, and CAN interfaces with device-specific configuration and board-aware pin assignments.

### ✨ Key Features

- **📡 Multi-Protocol Support**: SPI, I2C, UART, CAN interfaces
- **🔗 Board-Aware Configuration**: Automatic pin mapping and device routing
- **🎯 Device-Specific Access**: Type-safe device enumeration and access
- **🔒 Thread-Safe Operations**: Concurrent access from multiple tasks
- **⚙️ Flexible Configuration**: Customizable interface parameters
- **🏥 Health Monitoring**: Interface status and diagnostics
- **🔧 Easy Integration**: Simple API for device access
- **🛡️ Error Handling**: Comprehensive error detection and recovery

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                  CommChannelsManager                           │
├─────────────────────────────────────────────────────────────────┤
│  Device Enumeration │ Type-safe device identification         │
├─────────────────────────────────────────────────────────────────┤
│  Interface Access   │ SPI, I2C, UART, CAN interface access    │
├─────────────────────────────────────────────────────────────────┤
│  Pin Configuration  │ Board-specific pin and bus mapping      │
├─────────────────────────────────────────────────────────────────┤
│  ESP32 Drivers      │ ESP32-C6 hardware interface drivers     │
└─────────────────────────────────────────────────────────────────┘
```

## 🚀 Quick Start

### Basic Interface Access

```cpp
#include "component-handlers/CommChannelsManager.h"

void comm_basic_example() {
    // Get singleton instance
    auto& comm = CommChannelsManager::GetInstance();
    
    // Initialize the manager
    if (!comm.EnsureInitialized()) {
        logger.Info("COMM", "Failed to initialize communication manager\n");
        return;
    }
    
    // Access SPI device
    auto* spi_device = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (spi_device) {
        logger.Info("COMM", "TMC9660 SPI device available\n");
    }
    
    // Access I2C device
    auto* i2c_device = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (i2c_device) {
        logger.Info("COMM", "BNO08x I2C device available\n");
    }
    
    // Access UART
    BaseUart* uart;
    if (comm.GetUart(0, uart)) {
        logger.Info("COMM", "UART0 interface available\n");
    }
    
    // Access CAN
    BaseCan* can;
    if (comm.GetCan(0, can)) {
        logger.Info("COMM", "CAN interface available\n");
    }
}
```

## 📖 API Reference

### Core Operations

#### Initialization and Management
```cpp
class CommChannelsManager {
public:
    // Singleton access
    static CommChannelsManager& GetInstance() noexcept;
    
    // Initialization
    bool EnsureInitialized() noexcept;
    bool EnsureDeinitialized() noexcept;
    bool IsInitialized() const noexcept;
};
```

#### Device Enumeration
```cpp
// SPI device identification
enum class SpiDeviceId : uint8_t {
    TMC9660_MOTOR_CONTROLLER = 0,  // TMC9660 motor controller (SPI Mode 3)
    AS5047U_POSITION_ENCODER = 1,  // AS5047U position encoder (SPI Mode 1)
    EXTERNAL_DEVICE_1 = 2,         // External device 1 (SPI Mode 0)
    EXTERNAL_DEVICE_2 = 3,         // External device 2 (SPI Mode 0)
    
    // Aliases
    MOTOR_CONTROLLER = TMC9660_MOTOR_CONTROLLER,
    POSITION_ENCODER = AS5047U_POSITION_ENCODER,
    TMC9660_SPI = TMC9660_MOTOR_CONTROLLER,
    
    SPI_DEVICE_COUNT
};

// I2C device identification
enum class I2cDeviceId : uint8_t {
    BNO08X_IMU = 0,              // BNO08x IMU sensor (address 0x4A or 0x4B)
    PCAL9555_GPIO_EXPANDER = 1,  // PCAL9555 GPIO expander (address 0x20-0x27)
    
    // Aliases
    IMU = BNO08X_IMU,
    GPIO_EXPANDER = PCAL9555_GPIO_EXPANDER,
    
    I2C_DEVICE_COUNT
};
```

#### SPI Interface Access
```cpp
// SPI device access
BaseSpi* GetSpiDevice(uint8_t bus_index, int device_index) noexcept;
BaseSpi* GetSpiDevice(SpiDeviceId device_id) noexcept;

// Custom SPI device registration
int RegisterCustomSpiDevice(std::shared_ptr<BaseSpi> custom_device, 
                           int device_index = -1, uint8_t bus_index = 0xFF) noexcept;
```

#### I2C Interface Access
```cpp
// I2C device access
BaseI2c* GetI2cDevice(uint8_t bus_index, int device_index) noexcept;
BaseI2c* GetI2cDevice(I2cDeviceId device_id) noexcept;

// Runtime I2C device creation
int CreateI2cDevice(uint8_t device_address, uint32_t speed_hz = 400000) noexcept;
int CreateI2cDevice(uint8_t bus_index, uint8_t device_address, uint32_t speed_hz = 400000) noexcept;

// Custom I2C device registration
int RegisterCustomI2cDevice(uint8_t bus_index, std::shared_ptr<BaseI2c> custom_device, 
                           uint8_t device_address = 0xFF) noexcept;
int RegisterCustomI2cDevice(std::shared_ptr<BaseI2c> custom_device, 
                           uint8_t device_address = 0xFF, uint8_t bus_index = 0xFF) noexcept;

// I2C device management
bool HasI2cDeviceAtAddress(uint8_t bus_index, uint8_t device_address) const noexcept;
```

#### UART Interface Access
```cpp
// UART access
bool GetUart(std::size_t bus_index, BaseUart*& bus) noexcept;
```

#### CAN Interface Access
```cpp
// CAN access
bool GetCan(std::size_t bus_index, BaseCan*& bus) noexcept;
```

#### System Information
```cpp
// Bus availability and device counts
bool IsBusAvailable(uint8_t bus_index) const noexcept;
uint8_t GetDeviceCountOnBus(uint8_t bus_index) const noexcept;

// MCU-specific access
bool GetMcuI2cBus(uint8_t bus_index, EspI2cBus*& bus) noexcept;
bool GetMcuSpiBus(uint8_t bus_index, EspSpiBus*& bus) noexcept;

// Statistics
void DumpStatistics() const noexcept;
```

## 🎯 Hardware Configuration

### Board Pin Mapping

The CommChannelsManager automatically configures pins based on the HardFOC board design:

```cpp
// SPI pin configuration (SPI2)
constexpr gpio_num_t SPI2_MOSI = GPIO_NUM_11;  // Master Out Slave In
constexpr gpio_num_t SPI2_MISO = GPIO_NUM_13;  // Master In Slave Out
constexpr gpio_num_t SPI2_SCLK = GPIO_NUM_12;  // Serial Clock

// SPI Chip Select pins
constexpr gpio_num_t SPI2_CS_TMC9660 = GPIO_NUM_10;      // TMC9660 motor controller
constexpr gpio_num_t SPI2_CS_AS5047U = GPIO_NUM_9;       // AS5047U position encoder
constexpr gpio_num_t EXT_GPIO_CS_1 = GPIO_NUM_8;         // External device 1
constexpr gpio_num_t EXT_GPIO_CS_2 = GPIO_NUM_7;         // External device 2

// I2C pin configuration (I2C0)
constexpr gpio_num_t I2C0_SDA = GPIO_NUM_6;   // Serial Data
constexpr gpio_num_t I2C0_SCL = GPIO_NUM_5;   // Serial Clock

// UART pin configuration
constexpr gpio_num_t UART0_TX = GPIO_NUM_21;  // UART0 Transmit
constexpr gpio_num_t UART0_RX = GPIO_NUM_20;  // UART0 Receive

// CAN pin configuration
constexpr gpio_num_t CAN_TX = GPIO_NUM_4;     // CAN Transmit
constexpr gpio_num_t CAN_RX = GPIO_NUM_3;     // CAN Receive
```

### Device Configuration

```cpp
// SPI device configurations
struct SpiDeviceConfig {
    uint8_t bus_index;           // SPI bus index
    int device_index;            // Device index on bus
    uint32_t clock_speed_hz;     // Clock speed in Hz
    uint8_t mode;                // SPI mode (0-3)
    uint8_t cs_pin;              // Chip select pin
    bool cs_active_low;          // CS active level
};

// I2C device configurations
struct I2cDeviceConfig {
    uint8_t bus_index;           // I2C bus index
    uint8_t device_address;      // 7-bit device address
    uint32_t clock_speed_hz;     // Clock speed in Hz
    bool pullup_enabled;         // Internal pullup enable
};
```

## 📊 Examples

### SPI Device Access

```cpp
void spi_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Access TMC9660 motor controller
    auto* tmc9660_spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (tmc9660_spi) {
        logger.Info("COMM", "TMC9660 SPI device ready\n");
        
        // Example SPI transaction
        uint8_t tx_data[4] = {0x01, 0x02, 0x03, 0x04};
        uint8_t rx_data[4];
        
        if (tmc9660_spi->TransmitReceive(tx_data, rx_data, 4) == HF_SPI_SUCCESS) {
            logger.Info("COMM", "SPI transaction successful\n");
        }
    }
    
    // Access AS5047U position encoder
    auto* encoder_spi = comm.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    if (encoder_spi) {
        logger.Info("COMM", "AS5047U encoder SPI device ready\n");
    }
}
```

### I2C Device Access

```cpp
void i2c_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Access BNO08x IMU
    auto* imu_i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (imu_i2c) {
        logger.Info("COMM", "BNO08x IMU I2C device ready\n");
        
        // Example I2C read
        uint8_t reg_addr = 0x00;
        uint8_t data[4];
        
        if (imu_i2c->ReadRegister(reg_addr, data, 4) == HF_I2C_SUCCESS) {
            logger.Info("COMM", "I2C read successful\n");
        }
    }
    
    // Access PCAL9555 GPIO expander
    auto* gpio_i2c = comm.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (gpio_i2c) {
        logger.Info("COMM", "PCAL9555 GPIO expander I2C device ready\n");
    }
}
```

### Runtime I2C Device Creation

```cpp
void runtime_i2c_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Create a new I2C device at runtime
    int device_index = comm.CreateI2cDevice(0x48, 100000);  // 100kHz speed
    if (device_index >= 0) {
        logger.Info("COMM", "Created I2C device at index %d\n", device_index);
        
        // Access the created device
        auto* device = comm.GetI2cDevice(0, device_index);
        if (device) {
            logger.Info("COMM", "Runtime I2C device ready\n");
        }
    }
    
    // Check if device exists at specific address
    if (comm.HasI2cDeviceAtAddress(0, 0x48)) {
        logger.Info("COMM", "Device found at address 0x48\n");
    }
}
```

### UART Communication

```cpp
void uart_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Access UART0
    BaseUart* uart;
    if (comm.GetUart(0, uart)) {
        logger.Info("COMM", "UART0 ready\n");
        
        // Configure UART
        uart->Configure(115200, 8, HF_UART_PARITY_NONE, HF_UART_STOP_BITS_1);
        
        // Send data
        const char* message = "Hello UART!\n";
        uart->Transmit(reinterpret_cast<const uint8_t*>(message), strlen(message));
        
        // Receive data
        uint8_t buffer[64];
        size_t received = uart->Receive(buffer, sizeof(buffer));
        if (received > 0) {
            logger.Info("COMM", "Received %zu bytes\n", received);
        }
    }
}
```

### CAN Communication

```cpp
void can_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Access CAN interface
    BaseCan* can;
    if (comm.GetCan(0, can)) {
        logger.Info("COMM", "CAN interface ready\n");
        
        // Configure CAN
        can->Configure(500000, HF_CAN_MODE_NORMAL);  // 500kbps
        
        // Send CAN message
        CanFrame tx_frame;
        tx_frame.id = 0x123;
        tx_frame.dlc = 4;
        tx_frame.data[0] = 0x01;
        tx_frame.data[1] = 0x02;
        tx_frame.data[2] = 0x03;
        tx_frame.data[3] = 0x04;
        
        if (can->Transmit(tx_frame) == HF_CAN_SUCCESS) {
            logger.Info("COMM", "CAN message sent\n");
        }
        
        // Receive CAN message
        CanFrame rx_frame;
        if (can->Receive(rx_frame) == HF_CAN_SUCCESS) {
            logger.Info("COMM", "CAN message received: ID=0x%03X, DLC=%d\n", 
                   rx_frame.id, rx_frame.dlc);
        }
    }
}
```

### Custom Device Registration

```cpp
void custom_device_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Create custom I2C device
    auto custom_i2c = std::make_shared<CustomI2cDevice>();
    
    // Register custom device
    int device_index = comm.RegisterCustomI2cDevice(0, custom_i2c, 0x50);
    if (device_index >= 0) {
        logger.Info("COMM", "Custom I2C device registered at index %d\n", device_index);
        
        // Access the custom device
        auto* device = comm.GetI2cDevice(0, device_index);
        if (device) {
            logger.Info("COMM", "Custom I2C device ready\n");
        }
    }
    
    // Create custom SPI device
    auto custom_spi = std::make_shared<CustomSpiDevice>();
    
    // Register custom SPI device
    int spi_index = comm.RegisterCustomSpiDevice(custom_spi, 4);  // Use index 4
    if (spi_index >= 0) {
        logger.Info("COMM", "Custom SPI device registered at index %d\n", spi_index);
    }
}
```

### System Diagnostics

```cpp
void diagnostics_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Check bus availability
    for (uint8_t i = 0; i < 4; i++) {
        if (comm.IsBusAvailable(i)) {
            logger.Info("COMM", "Bus %d is available\n", i);
            logger.Info("COMM", "  Device count: %u\n", comm.GetDeviceCountOnBus(i));
        }
    }
    
    // Check specific device availability
    if (comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER)) {
        logger.Info("COMM", "TMC9660 SPI device is available\n");
    }
    
    if (comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU)) {
        logger.Info("COMM", "BNO08x I2C device is available\n");
    }
    
    // Dump system statistics
    comm.DumpStatistics();
}
```

## 🔍 Advanced Usage

### Multi-Interface Integration

```cpp
void multi_interface_example() {
    auto& comm = CommChannelsManager::GetInstance();
    comm.EnsureInitialized();
    
    // Access multiple interfaces simultaneously
    auto* tmc9660_spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    auto* imu_i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    BaseUart* uart;
    comm.GetUart(0, uart);
    BaseCan* can;
    comm.GetCan(0, can);
    
    // Perform operations on all interfaces
    if (tmc9660_spi && imu_i2c && uart && can) {
        logger.Info("COMM", "All communication interfaces ready\n");
        
        // Example: Send motor command via SPI, read IMU via I2C, 
        // log via UART, and broadcast status via CAN
        // ... implementation details ...
    }
}
```

### Error Handling

```cpp
void error_handling_example() {
    auto& comm = CommChannelsManager::GetInstance();
    
    // Check initialization
    if (!comm.EnsureInitialized()) {
        logger.Info("COMM", "ERROR: Failed to initialize communication manager\n");
        return;
    }
    
    // Validate device availability before use
    auto* spi_device = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spi_device) {
        logger.Info("COMM", "ERROR: TMC9660 SPI device not available\n");
        return;
    }
    
    // Safe device operations with error checking
    uint8_t tx_data[4] = {0x01, 0x02, 0x03, 0x04};
    uint8_t rx_data[4];
    
    auto result = spi_device->TransmitReceive(tx_data, rx_data, 4);
    if (result != HF_SPI_SUCCESS) {
        logger.Info("COMM", "ERROR: SPI transaction failed: %d\n", static_cast<int>(result));
    }
    
    // Check bus availability
    if (!comm.IsBusAvailable(0)) {
        logger.Info("COMM", "WARNING: SPI bus 0 not available\n");
    }
}
```

### CommError Reference

| Code | Value | Description |
|------|-------|-------------|
| `SUCCESS` | 0 | Operation completed successfully |
| `NOT_INITIALIZED` | 1 | Manager not yet initialised |
| `INITIALIZATION_FAILED` | 2 | Bus or device initialisation failed |
| `BUS_NOT_AVAILABLE` | 3 | Requested bus is not present or not ready |
| `DEVICE_NOT_FOUND` | 4 | Device index or id not in registry |
| `INVALID_BUS_INDEX` | 5 | Bus index out of range |
| `INVALID_DEVICE_INDEX` | 6 | Device index out of range |
| `DEVICE_ALREADY_EXISTS` | 7 | A device at that address already exists |
| `DEVICE_CREATION_FAILED` | 8 | Heap allocation or driver init for device failed |
| `COMMUNICATION_FAILED` | 9 | A bus-level transfer failed |
| `INVALID_PARAMETER` | 10 | Null pointer or out-of-range argument |
| `MUTEX_LOCK_FAILED` | 11 | RTOS mutex acquire timed out |

## 🔗 Integration

### With Other Managers

```cpp
#include "component-handlers/CommChannelsManager.h"

void integrated_example() {
    // Initialize all managers
    auto& comm = CommChannelsManager::GetInstance();
    auto& gpio = GpioManager::GetInstance();
    auto& adc = AdcManager::GetInstance();
    auto& motor = MotorController::GetInstance();
    
    comm.EnsureInitialized();
    gpio.EnsureInitialized();
    adc.Initialize();
    motor.EnsureInitialized();
    
    // Use communication interfaces with other managers
    auto* tmc9660_spi = comm.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    auto* imu_i2c = comm.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    
    if (tmc9660_spi && imu_i2c) {
        // Motor control via SPI
        // IMU reading via I2C
        // GPIO control for enable/disable
        // ADC monitoring for current/voltage
        logger.Info("COMM", "All systems integrated and ready\n");
    }
}
```

## 📚 See Also

- **[GpioManager Documentation](GPIO_MANAGER_README.md)** - GPIO management system
- **[AdcManager Documentation](ADC_MANAGER_README.md)** - ADC management system
- **[MotorController Documentation](MOTOR_CONTROLLER_README.md)** - Motor control system
- **[TMC9660 Handler Documentation](../driver-handlers/TMC9660_HANDLER_README.md)** - TMC9660 driver
- **[BNO08x Handler Documentation](../driver-handlers/BNO08X_HANDLER_README.md)** - IMU driver

---

*This documentation is part of the HardFOC HAL system. For complete system documentation, see [Documentation Index](../../DOCUMENTATION_INDEX.md).*