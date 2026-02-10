/**
 * @file ImuManagerExample.cpp
 * @brief Example demonstrating the new ImuManager with multiple device support.
 * 
 * This example shows how to use the redesigned ImuManager that follows the
 * MotorController pattern with:
 * - Multiple IMU device support with indexed access
 * - Onboard BNO08x device (index 0) - always available
 * - External BNO08x devices (indices 1-3) - user-created
 * - Thread-safe device management
 * - Flexible I2C/SPI transport support
 * - Interrupt and polling modes
 * 
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 */

#include <vector>

// IMU Manager
#include "ImuManager.h"

// BNO08x handler and types
#include "handlers/Bno08xHandler.h"

// Communication manager for device IDs
#include "CommChannelsManager.h"

// RTOS utilities
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsAbstraction.h"

// Unified Logger
#include "handlers/Logger.h"

static const char* TAG = "ImuManagerExample";

//==============================================================================
// EXAMPLE FUNCTIONS
//==============================================================================

// Dummy device implementations for demonstration
class DummyI2cDevice : public BaseI2c {
public:
    hf_i2c_err_t Write(const uint8_t* data, size_t length) override {
        return hf_i2c_err_t::I2C_SUCCESS;
    }
    
    hf_i2c_err_t Read(uint8_t* data, size_t length) override {
        for (size_t i = 0; i < length; ++i) {
            data[i] = static_cast<uint8_t>(i + 0x30);
        }
        return hf_i2c_err_t::I2C_SUCCESS;
    }
    
    hf_i2c_err_t WriteRead(const uint8_t* write_data, size_t write_length,
                          uint8_t* read_data, size_t read_length) override {
        return hf_i2c_err_t::I2C_SUCCESS;
    }
};

class DummySpiDevice : public BaseSpi {
public:
    hf_spi_err_t Write(const uint8_t* data, size_t length) override {
        return hf_spi_err_t::SPI_SUCCESS;
    }
    
    hf_spi_err_t Read(uint8_t* data, size_t length) override {
        for (size_t i = 0; i < length; ++i) {
            data[i] = static_cast<uint8_t>(i + 0x40);
        }
        return hf_spi_err_t::SPI_SUCCESS;
    }
    
    hf_spi_err_t WriteRead(const uint8_t* write_data, size_t write_length,
                          uint8_t* read_data, size_t read_length) override {
        return hf_spi_err_t::SPI_SUCCESS;
    }
};

/**
 * @brief Demonstrate basic onboard IMU usage
 */
void DemonstrateOnboardImu() {
    Logger::GetInstance().Info(TAG, "=== Onboard IMU Example ===");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Ensure IMU manager is initialized
    if (!imu_mgr.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize IMU Manager");
        return;
    }
    
    // Get onboard BNO08x handler (index 0)
    Bno08xHandler* onboard_handler = imu_mgr.GetBno08xHandler(0);
    if (!onboard_handler) {
        Logger::GetInstance().Error(TAG, "Onboard BNO08x handler not available");
        return;
    }
    
    // Enable sensors
    Bno08xError result = onboard_handler->EnableSensor(BNO085Sensor::Accelerometer, 50);
    if (result == Bno08xError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "Accelerometer enabled");
    } else {
        Logger::GetInstance().Error(TAG, "Failed to enable accelerometer: %s", Bno08xErrorToString(result));
    }
    
    result = onboard_handler->EnableSensor(BNO085Sensor::Gyroscope, 50);
    if (result == Bno08xError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "Gyroscope enabled");
    } else {
        Logger::GetInstance().Error(TAG, "Failed to enable gyroscope: %s", Bno08xErrorToString(result));
    }
    
    // Read sensor data for 3 seconds
    Logger::GetInstance().Info(TAG, "Reading sensor data for 3 seconds...");
    uint32_t start_time_ms = os_get_elapsed_time_msec();
    uint32_t end_time_ms = start_time_ms + 3000; // 3 seconds
    
    while (os_get_elapsed_time_msec() < end_time_ms) {
        onboard_handler->Update();
        
        Bno08xVector3 accelerometer;
        result = onboard_handler->ReadAcceleration(accelerometer);
        if (result == Bno08xError::SUCCESS && accelerometer.valid) {
            Logger::GetInstance().Info(TAG, "Accel: x=%.3f, y=%.3f, z=%.3f", 
                     accelerometer.x, accelerometer.y, accelerometer.z);
        }
        
        Bno08xVector3 gyroscope;
        result = onboard_handler->ReadGyroscope(gyroscope);
        if (result == Bno08xError::SUCCESS && gyroscope.valid) {
            Logger::GetInstance().Info(TAG, "Gyro: x=%.3f, y=%.3f, z=%.3f", 
                     gyroscope.x, gyroscope.y, gyroscope.z);
        }
        
        os_delay_msec(50); // 50ms delay
    }
    
    Logger::GetInstance().Info(TAG, "Onboard IMU example completed");
}

/**
 * @brief Demonstrate external IMU device creation and usage with simplified API
 */
void DemonstrateExternalImu() {
    Logger::GetInstance().Info(TAG, "=== External IMU Example ===");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Ensure IMU manager is initialized
    if (!imu_mgr.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize IMU Manager");
        return;
    }
    
    // Check if external slot 1 is available
    if (!imu_mgr.IsExternalSlotAvailable(1)) {
        Logger::GetInstance().Warn(TAG, "External slot 1 not available");
        return;
    }
    
    // Create external BNO08x device with simplified API
    // ImuManager handles I2C device creation internally
    Logger::GetInstance().Info(TAG, "Creating external BNO08x device with simplified API...");
    
    bool created = imu_mgr.CreateExternalBno08xDevice(1, 0x4B, 400000); // BNO08x at address 0x4B
    if (!created) {
        Logger::GetInstance().Error(TAG, "Failed to create external BNO08x device");
        return;
    }
    
    Logger::GetInstance().Info(TAG, "External BNO08x device created successfully");
    Logger::GetInstance().Info(TAG, "Total devices: %u", imu_mgr.GetDeviceCount());
    
    // Get external BNO08x handler
    Bno08xHandler* external_handler = imu_mgr.GetBno08xHandler(1);
    if (!external_handler) {
        Logger::GetInstance().Error(TAG, "External BNO08x handler not available");
        return;
    }
    
    // Configure external device
    Bno08xError result = external_handler->EnableSensor(BNO085Sensor::Gyroscope, 50);
    if (result == Bno08xError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "External gyroscope enabled");
    } else {
        Logger::GetInstance().Error(TAG, "Failed to enable external gyroscope: %s", Bno08xErrorToString(result));
    }
    
    // Read data from external device
    Logger::GetInstance().Info(TAG, "Reading external sensor data for 3 seconds...");
    uint32_t start_time_ms = os_get_elapsed_time_msec();
    uint32_t end_time_ms = start_time_ms + 3000; // 3 seconds
    
    while (os_get_elapsed_time_msec() < end_time_ms) {
        external_handler->Update();
        
        Bno08xVector3 gyroscope;
        result = external_handler->ReadGyroscope(gyroscope);
        if (result == Bno08xError::SUCCESS && gyroscope.valid) {
            Logger::GetInstance().Info(TAG, "External Gyro: x=%.3f, y=%.3f, z=%.3f", 
                     gyroscope.x, gyroscope.y, gyroscope.z);
        }
        
        os_delay_msec(50); // 50ms delay
    }
    
    // Clean up: Delete external device (ImuManager handles I2C device cleanup)
    bool deleted = imu_mgr.DeleteExternalDevice(1);
    if (deleted) {
        Logger::GetInstance().Info(TAG, "External device deleted successfully");
    } else {
        Logger::GetInstance().Error(TAG, "Failed to delete external device");
    }
    
    Logger::GetInstance().Info(TAG, "External IMU example completed");
}

/**
 * @brief Demonstrate interrupt-based IMU usage
 */
void DemonstrateInterruptMode() {
    Logger::GetInstance().Info(TAG, "=== Interrupt Mode Example ===");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Ensure IMU manager is initialized
    if (!imu_mgr.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize IMU Manager");
        return;
    }
    
    // Configure interrupt for onboard device
    bool interrupt_configured = imu_mgr.ConfigureInterrupt(0, []() {
        Logger::GetInstance().Info(TAG, "BNO08x interrupt triggered!");
    });
    
    if (!interrupt_configured) {
        Logger::GetInstance().Warn(TAG, "Interrupt configuration failed - using polling mode");
        return;
    }
    
    // Enable interrupt
    bool interrupt_enabled = imu_mgr.EnableInterrupt(0);
    if (!interrupt_enabled) {
        Logger::GetInstance().Error(TAG, "Failed to enable interrupt");
        return;
    }
    
    Logger::GetInstance().Info(TAG, "Interrupt mode enabled - waiting for interrupts...");
    
    // Wait for interrupts for 5 seconds
    uint32_t start_time_ms = os_get_elapsed_time_msec();
    uint32_t end_time_ms = start_time_ms + 5000; // 5 seconds
    
    while (os_get_elapsed_time_msec() < end_time_ms) {
        // Wait for interrupt with timeout
        bool interrupt_received = imu_mgr.WaitForInterrupt(0, 1000); // 1 second timeout
        
        if (interrupt_received) {
            uint32_t interrupt_count = imu_mgr.GetInterruptCount(0);
            Logger::GetInstance().Info(TAG, "Interrupt received! Total count: %u", interrupt_count);
            
            // Process sensor data
            Bno08xHandler* handler = imu_mgr.GetBno08xHandler(0);
            if (handler) {
                handler->Update();
                
                Bno08xVector3 accelerometer;
                Bno08xError result = handler->ReadAcceleration(accelerometer);
                if (result == Bno08xError::SUCCESS && accelerometer.valid) {
                    Logger::GetInstance().Info(TAG, "Interrupt data - Accel: x=%.3f, y=%.3f, z=%.3f", 
                             accelerometer.x, accelerometer.y, accelerometer.z);
                }
            }
        } else {
            Logger::GetInstance().Info(TAG, "No interrupt received within timeout");
        }
    }
    
    // Disable interrupt
    imu_mgr.DisableInterrupt(0);
    Logger::GetInstance().Info(TAG, "Interrupt mode disabled");
    
    Logger::GetInstance().Info(TAG, "Interrupt mode example completed");
}

/**
 * @brief Demonstrate device management and status reporting
 */
void DemonstrateDeviceManagement() {
    Logger::GetInstance().Info(TAG, "=== Device Management Example ===");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Ensure IMU manager is initialized
    if (!imu_mgr.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize IMU Manager");
        return;
    }
    
    Logger::GetInstance().Info(TAG, "Device count: %u", imu_mgr.GetDeviceCount());
    
    // Check device status for all slots
    for (uint8_t i = 0; i < 4; ++i) {
        bool valid = imu_mgr.IsDeviceValid(i);
        bool available = imu_mgr.IsExternalSlotAvailable(i);
        std::string device_type = imu_mgr.GetDeviceType(i);
        
        Logger::GetInstance().Info(TAG, "Slot %u: valid=%s, available=%s, type=%s", 
                 i, valid ? "true" : "false", available ? "true" : "false", device_type.c_str());
    }
    
    // Get active device indices
    std::vector<uint8_t> active_devices = imu_mgr.GetActiveDeviceIndices();
    Logger::GetInstance().Info(TAG, "Active devices: %zu", active_devices.size());
    for (uint8_t device_index : active_devices) {
        Logger::GetInstance().Info(TAG, "  - Device %u: %s", device_index, imu_mgr.GetDeviceType(device_index).c_str());
    }
    
    // Initialize all devices
    std::vector<bool> init_results = imu_mgr.InitializeAllDevices();
    Logger::GetInstance().Info(TAG, "Initialization results:");
    for (size_t i = 0; i < init_results.size(); ++i) {
        Logger::GetInstance().Info(TAG, "  - Device %zu: %s", i, init_results[i] ? "SUCCESS" : "FAILED");
    }
    
    // Get initialization status
    std::vector<bool> init_status = imu_mgr.GetInitializationStatus();
    Logger::GetInstance().Info(TAG, "Initialization status:");
    for (size_t i = 0; i < init_status.size(); ++i) {
        Logger::GetInstance().Info(TAG, "  - Device %zu: %s", i, init_status[i] ? "INITIALIZED" : "NOT_INITIALIZED");
    }
    
    Logger::GetInstance().Info(TAG, "Device management example completed");
}

/**
 * @brief Demonstrate external SPI IMU device creation and usage
 */
void DemonstrateSpiImu() {
    Logger::GetInstance().Info(TAG, "=== SPI IMU Example ===");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Ensure IMU manager is initialized
    if (!imu_mgr.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize IMU Manager");
        return;
    }
    
    // Check if external slot 2 is available
    if (!imu_mgr.IsExternalSlotAvailable(2)) {
        Logger::GetInstance().Warn(TAG, "External slot 2 not available");
        return;
    }
    
    // Create external BNO08x device on SPI interface
    bool created = imu_mgr.CreateExternalBno08xDevice(2, SpiDeviceId::SPI2_CS_TMC9660);
    if (!created) {
        Logger::GetInstance().Error(TAG, "Failed to create external SPI BNO08x device");
        return;
    }
    
    Logger::GetInstance().Info(TAG, "External SPI BNO08x device created successfully");
    
    // Get external BNO08x handler
    Bno08xHandler* external_handler = imu_mgr.GetBno08xHandler(2);
    if (!external_handler) {
        Logger::GetInstance().Error(TAG, "External SPI BNO08x handler not available");
        return;
    }
    
    // Configure external device
    Bno08xError result = external_handler->EnableSensor(BNO085Sensor::Accelerometer, 100);
    if (result == Bno08xError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "External SPI accelerometer enabled");
    } else {
        Logger::GetInstance().Error(TAG, "Failed to enable external SPI accelerometer: %s", Bno08xErrorToString(result));
    }
    
    // Read data from external device
    Logger::GetInstance().Info(TAG, "Reading external SPI sensor data for 2 seconds...");
    uint32_t start_time = os_get_elapsed_time_msec();
    uint32_t end_time = start_time + (2000 / 1); // 2 seconds
    
    while (os_get_elapsed_time_msec() < end_time) {
        external_handler->Update();
        
        Bno08xVector3 accelerometer;
        result = external_handler->ReadAcceleration(accelerometer);
        if (result == Bno08xError::SUCCESS && accelerometer.valid) {
            Logger::GetInstance().Info(TAG, "External SPI Accel: x=%.3f, y=%.3f, z=%.3f", 
                     accelerometer.x, accelerometer.y, accelerometer.z);
        }
        
        os_delay_msec(100)); // 100ms delay
    }
    
    // Delete external device
    bool deleted = imu_mgr.DeleteExternalDevice(2);
    if (deleted) {
        Logger::GetInstance().Info(TAG, "External SPI device deleted successfully");
    } else {
        Logger::GetInstance().Error(TAG, "Failed to delete external SPI device");
    }
    
    Logger::GetInstance().Info(TAG, "SPI IMU example completed");
}

/**
 * @brief Demonstrate flexible device creation with custom implementations
 */
void DemonstrateFlexibleDeviceCreation() {
    Logger::GetInstance().Info("IMU_EXAMPLE", "=== Demonstrating Flexible Device Creation ===");
    
    auto& comm_manager = CommChannelsManager::GetInstance();
    comm_manager.EnsureInitialized();
    
    // Create runtime I2C devices
    Logger::GetInstance().Info("IMU_EXAMPLE", "Creating runtime I2C devices...");
    
    int device1 = comm_manager.CreateI2cDevice(0x48, 100000);  // Bus 0, Address 0x48, 100kHz
    int device2 = comm_manager.CreateI2cDevice(0x49, 400000);  // Bus 0, Address 0x49, 400kHz
    
    if (device1 >= 0) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Created I2C device 1: index=%d, address=0x48, speed=100kHz", device1);
    }
    
    if (device2 >= 0) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Created I2C device 2: index=%d, address=0x49, speed=400kHz", device2);
    }
    
    // Register custom I2C device
    Logger::GetInstance().Info("IMU_EXAMPLE", "Registering custom I2C device...");
    
    auto custom_i2c = std::make_shared<DummyI2cDevice>();
    int custom_device = comm_manager.RegisterCustomI2cDevice(custom_i2c, 0x50, 0xFF);  // Auto-assign bus
    
    if (custom_device >= 0) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Registered custom I2C device: index=%d, address=0x50", custom_device);
    }
    
    // Register custom SPI device
    Logger::GetInstance().Info("IMU_EXAMPLE", "Registering custom SPI device...");
    
    auto custom_spi = std::make_shared<DummySpiDevice>();
    int custom_spi_device = comm_manager.RegisterCustomSpiDevice(custom_spi, -1, 0xFF);  // Auto-assign index and bus
    
    if (custom_spi_device >= 0) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Registered custom SPI device: index=%d", custom_spi_device);
    }
    
    // Access devices using bus-aware methods
    Logger::GetInstance().Info("IMU_EXAMPLE", "Accessing devices using bus-aware methods...");
    
    if (device1 >= 0) {
        BaseI2c* i2c1 = comm_manager.GetI2cDevice(0, device1);  // Bus 0, Device 1
        if (i2c1) {
            Logger::GetInstance().Info("IMU_EXAMPLE", "Successfully accessed I2C device 1 on Bus 0");
        }
    }
    
    if (custom_device >= 0) {
        // Find which bus the custom device is on
        for (uint8_t bus = 0; bus < 20; bus++) {
            if (comm_manager.IsBusAvailable(bus)) {
                uint8_t device_count = comm_manager.GetDeviceCountOnBus(bus);
                Logger::GetInstance().Info("IMU_EXAMPLE", "Bus %d has %d devices", bus, device_count);
            }
        }
    }
    
    // Cleanup
    if (device1 >= 0) comm_manager.RemoveI2cDevice(device1);
    if (device2 >= 0) comm_manager.RemoveI2cDevice(device2);
    if (custom_device >= 0) comm_manager.RemoveI2cDevice(custom_device);
    if (custom_spi_device >= 0) comm_manager.RemoveSpiDevice(custom_spi_device);
    
    Logger::GetInstance().Info("IMU_EXAMPLE", "Flexible device creation demonstration completed");
}

/**
 * @brief Demonstrate flexible device creation with external buses and direct interfaces
 */
void DemonstrateFlexibleExternalBuses() {
    Logger::GetInstance().Info(TAG, "=== Flexible External Bus Example ===");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Ensure IMU manager is initialized
    if (!imu_mgr.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize IMU Manager");
        return;
    }
    
    Logger::GetInstance().Info(TAG, "Demonstrating flexible external bus capabilities...");
    
    // Example 1: External I2C bus with custom implementation
    Logger::GetInstance().Info(TAG, "--- External I2C Bus Example ---");
    
    // Create a custom I2C bus implementation for external hardware
    class ExternalI2cBus : public BaseI2c {
    public:
        ExternalI2cBus(uint8_t bus_id) : bus_id_(bus_id) {}
        
        hf_i2c_err_t Write(const uint8_t* data, size_t length) override {
            Logger::GetInstance().Info(TAG, "External I2C Bus %u Write: length=%zu", bus_id_, length);
            // Custom implementation for external I2C bus
            return hf_i2c_err_t::I2C_SUCCESS;
        }
        
        hf_i2c_err_t Read(uint8_t* data, size_t length) override {
            Logger::GetInstance().Info(TAG, "External I2C Bus %u Read: length=%zu", bus_id_, length);
            // Custom implementation for external I2C bus
            for (size_t i = 0; i < length; ++i) {
                data[i] = static_cast<uint8_t>(i + 0x30);
            }
            return hf_i2c_err_t::I2C_SUCCESS;
        }
        
        hf_i2c_err_t WriteRead(const uint8_t* write_data, size_t write_length,
                              uint8_t* read_data, size_t read_length) override {
            Logger::GetInstance().Info(TAG, "External I2C Bus %u WriteRead: write=%zu, read=%zu", 
                     bus_id_, write_length, read_length);
            return hf_i2c_err_t::I2C_SUCCESS;
        }
        
    private:
        uint8_t bus_id_;
    };
    
    // Create external I2C bus instances
    auto external_i2c_bus1 = std::make_shared<ExternalI2cBus>(1);
    auto external_i2c_bus2 = std::make_shared<ExternalI2cBus>(2);
    
    // Example 2: External SPI bus with custom implementation
    Logger::GetInstance().Info(TAG, "--- External SPI Bus Example ---");
    
    // Create a custom SPI bus implementation for external hardware
    class ExternalSpiBus : public BaseSpi {
    public:
        ExternalSpiBus(uint8_t bus_id) : bus_id_(bus_id) {}
        
        hf_spi_err_t Write(const uint8_t* data, size_t length) override {
            Logger::GetInstance().Info(TAG, "External SPI Bus %u Write: length=%zu", bus_id_, length);
            // Custom implementation for external SPI bus
            return hf_spi_err_t::SPI_SUCCESS;
        }
        
        hf_spi_err_t Read(uint8_t* data, size_t length) override {
            Logger::GetInstance().Info(TAG, "External SPI Bus %u Read: length=%zu", bus_id_, length);
            // Custom implementation for external SPI bus
            for (size_t i = 0; i < length; ++i) {
                data[i] = static_cast<uint8_t>(i + 0x40);
            }
            return hf_spi_err_t::SPI_SUCCESS;
        }
        
        hf_spi_err_t WriteRead(const uint8_t* write_data, size_t write_length,
                              uint8_t* read_data, size_t read_length) override {
            Logger::GetInstance().Info(TAG, "External SPI Bus %u WriteRead: write=%zu, read=%zu", 
                     bus_id_, write_length, read_length);
            return hf_spi_err_t::SPI_SUCCESS;
        }
        
    private:
        uint8_t bus_id_;
    };
    
    // Create external SPI bus instances
    auto external_spi_bus1 = std::make_shared<ExternalSpiBus>(1);
    auto external_spi_bus2 = std::make_shared<ExternalSpiBus>(2);
    
    // Example 3: Create BNO08x devices on external buses using direct interfaces
    Logger::GetInstance().Info(TAG, "--- Creating BNO08x on External Buses ---");
    
    // Create BNO08x on external I2C bus 1
    bool created_i2c1 = imu_mgr.CreateExternalBno08xDevice(1, *external_i2c_bus1);
    if (created_i2c1) {
        Logger::GetInstance().Info(TAG, "BNO08x created on external I2C bus 1");
    }
    
    // Create BNO08x on external I2C bus 2
    bool created_i2c2 = imu_mgr.CreateExternalBno08xDevice(2, *external_i2c_bus2);
    if (created_i2c2) {
        Logger::GetInstance().Info(TAG, "BNO08x created on external I2C bus 2");
    }
    
    // Create BNO08x on external SPI bus 1
    bool created_spi1 = imu_mgr.CreateExternalBno08xDevice(3, *external_spi_bus1);
    if (created_spi1) {
        Logger::GetInstance().Info(TAG, "BNO08x created on external SPI bus 1");
    }
    
    // Example 4: Use the external devices
    Logger::GetInstance().Info(TAG, "--- Using External Devices ---");
    
    // Get and use external I2C device 1
    Bno08xHandler* external_i2c_handler1 = imu_mgr.GetBno08xHandler(1);
    if (external_i2c_handler1) {
        Bno08xError result = external_i2c_handler1->EnableSensor(BNO085Sensor::Accelerometer, 50);
        if (result == Bno08xError::SUCCESS) {
            Logger::GetInstance().Info(TAG, "External I2C BNO08x 1 accelerometer enabled");
        }
    }
    
    // Get and use external I2C device 2
    Bno08xHandler* external_i2c_handler2 = imu_mgr.GetBno08xHandler(2);
    if (external_i2c_handler2) {
        Bno08xError result = external_i2c_handler2->EnableSensor(BNO085Sensor::Gyroscope, 50);
        if (result == Bno08xError::SUCCESS) {
            Logger::GetInstance().Info(TAG, "External I2C BNO08x 2 gyroscope enabled");
        }
    }
    
    // Get and use external SPI device
    Bno08xHandler* external_spi_handler = imu_mgr.GetBno08xHandler(3);
    if (external_spi_handler) {
        Bno08xError result = external_spi_handler->EnableSensor(BNO085Sensor::Magnetometer, 100);
        if (result == Bno08xError::SUCCESS) {
            Logger::GetInstance().Info(TAG, "External SPI BNO08x magnetometer enabled");
        }
    }
    
    // Example 5: Demonstrate device management with external buses
    Logger::GetInstance().Info(TAG, "--- Device Management with External Buses ---");
    
    Logger::GetInstance().Info(TAG, "Total devices: %u", imu_mgr.GetDeviceCount());
    
    // Check device status for all slots
    for (uint8_t i = 0; i < 4; ++i) {
        bool valid = imu_mgr.IsDeviceValid(i);
        std::string device_type = imu_mgr.GetDeviceType(i);
        
        Logger::GetInstance().Info(TAG, "Slot %u: valid=%s, type=%s", 
                 i, valid ? "true" : "false", device_type.c_str());
    }
    
    // Get active device indices
    std::vector<uint8_t> active_devices = imu_mgr.GetActiveDeviceIndices();
    Logger::GetInstance().Info(TAG, "Active devices: %zu", active_devices.size());
    for (uint8_t device_index : active_devices) {
        Logger::GetInstance().Info(TAG, "  - Device %u: %s", device_index, imu_mgr.GetDeviceType(device_index).c_str());
    }
    
    // Example 6: Clean up external devices
    Logger::GetInstance().Info(TAG, "--- Cleanup External Devices ---");
    
    if (imu_mgr.DeleteExternalDevice(1)) {
        Logger::GetInstance().Info(TAG, "External I2C device 1 deleted successfully");
    }
    
    if (imu_mgr.DeleteExternalDevice(2)) {
        Logger::GetInstance().Info(TAG, "External I2C device 2 deleted successfully");
    }
    
    if (imu_mgr.DeleteExternalDevice(3)) {
        Logger::GetInstance().Info(TAG, "External SPI device deleted successfully");
    }
    
    Logger::GetInstance().Info(TAG, "Flexible external bus example completed");
}

/**
 * @brief Demonstrate correct bus indexing architecture with device organization
 */
void DemonstrateBusIndexingArchitecture() {
    Logger::GetInstance().Info("IMU_EXAMPLE", "=== Demonstrating Multi-Bus Architecture ===");
    
    auto& comm_manager = CommChannelsManager::GetInstance();
    comm_manager.EnsureInitialized();
    
    // Show current bus configuration
    uint8_t total_buses = comm_manager.GetBusCount();
    Logger::GetInstance().Info("IMU_EXAMPLE", "Total buses available: %d", total_buses);
    
    // Check built-in bus (Bus 0)
    if (comm_manager.IsBusAvailable(0)) {
        uint8_t device_count = comm_manager.GetDeviceCountOnBus(0);
        Logger::GetInstance().Info("IMU_EXAMPLE", "Bus 0 (Built-in): %d devices", device_count);
    }
    
    // Demonstrate device access using device IDs (known locations)
    Logger::GetInstance().Info("IMU_EXAMPLE", "--- Device Access via Device IDs ---");
    
    // Access known devices using device IDs
    BaseI2c* bno08x_device = comm_manager.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (bno08x_device) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Successfully accessed BNO08x via device ID");
    }
    
    BaseI2c* pcal_device = comm_manager.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (pcal_device) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Successfully accessed PCAL9555 via device ID");
    }
    
    BaseSpi* tmc9660_device = comm_manager.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (tmc9660_device) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Successfully accessed TMC9660 via device ID");
    }
    
    BaseSpi* as5047u_device = comm_manager.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    if (as5047u_device) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Successfully accessed AS5047U via device ID");
    }
    
    // Demonstrate device access using bus/device indices
    Logger::GetInstance().Info("IMU_EXAMPLE", "--- Device Access via Bus/Device Indices ---");
    
    // Access devices using bus/device indices (same devices as above)
    BaseI2c* bno08x_via_index = comm_manager.GetI2cDevice(0, static_cast<int>(I2cDeviceId::BNO08X_IMU));
    if (bno08x_via_index) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Successfully accessed BNO08x via bus/device index");
    }
    
    BaseSpi* tmc9660_via_index = comm_manager.GetSpiDevice(0, static_cast<int>(SpiDeviceId::TMC9660_MOTOR_CONTROLLER));
    if (tmc9660_via_index) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Successfully accessed TMC9660 via bus/device index");
    }
    
    // Create external I2C device on Bus 0 (built-in bus)
    int device_index_0 = comm_manager.CreateI2cDevice(0, 0x28, 400000);
    if (device_index_0 >= 0) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Created I2C device on Bus 0: index=%d, address=0x28", device_index_0);
        
        // Access device using bus-aware lookup
        BaseI2c* device_0 = comm_manager.GetI2cDevice(0, device_index_0);
        if (device_0) {
            Logger::GetInstance().Info("IMU_EXAMPLE", "Successfully accessed device on Bus 0, index %d", device_index_0);
        }
    }
    
    // Create custom external bus (Bus 10)
    auto custom_i2c = std::make_shared<DummyI2cDevice>();
    int device_index_10 = comm_manager.RegisterCustomI2cDevice(10, custom_i2c, 0x29);
    if (device_index_10 >= 0) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Created custom I2C device on Bus 10: index=%d, address=0x29", device_index_10);
        
        // Access device using bus-aware lookup
        BaseI2c* device_10 = comm_manager.GetI2cDevice(10, device_index_10);
        if (device_10) {
            Logger::GetInstance().Info("IMU_EXAMPLE", "Successfully accessed device on Bus 10, index %d", device_index_10);
        }
    }
    
    // Create another custom external bus (Bus 11)
    auto custom_spi = std::make_shared<DummySpiDevice>();
    int device_index_11 = comm_manager.RegisterCustomSpiDevice(11, custom_spi, -1);
    if (device_index_11 >= 0) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "Created custom SPI device on Bus 11: index=%d", device_index_11);
        
        // Access device using bus-aware lookup
        BaseSpi* device_11 = comm_manager.GetSpiDevice(11, device_index_11);
        if (device_11) {
            Logger::GetInstance().Info("IMU_EXAMPLE", "Successfully accessed device on Bus 11, index %d", device_index_11);
        }
    }
    
    // Show updated bus configuration
    total_buses = comm_manager.GetBusCount();
    Logger::GetInstance().Info("IMU_EXAMPLE", "Total buses after creation: %d", total_buses);
    
    // Check each bus
    for (uint8_t bus = 0; bus < 12; bus++) {
        if (comm_manager.IsBusAvailable(bus)) {
            uint8_t device_count = comm_manager.GetDeviceCountOnBus(bus);
            Logger::GetInstance().Info("IMU_EXAMPLE", "Bus %d: %d devices", bus, device_count);
        }
    }
    
    // Cleanup
    if (device_index_0 >= 0) {
        comm_manager.RemoveI2cDevice(device_index_0);
    }
    if (device_index_10 >= 0) {
        comm_manager.RemoveI2cDevice(device_index_10);
    }
    if (device_index_11 >= 0) {
        comm_manager.RemoveSpiDevice(device_index_11);
    }
    
    Logger::GetInstance().Info("IMU_EXAMPLE", "Multi-bus architecture demonstration completed");
}

void DemonstrateDeviceIndexOverlapIssue() {
    Logger::GetInstance().Info("IMU_EXAMPLE", "=== Demonstrating Device Index Overlap Issue ===");
    
    auto& comm_manager = CommChannelsManager::GetInstance();
    comm_manager.EnsureInitialized();
    
    // Create devices with overlapping indices on different buses
    Logger::GetInstance().Info("IMU_EXAMPLE", "Creating devices with overlapping indices on different buses...");
    
    // Bus 0: Device index 5 (built-in)
    // Bus 10: Device index 5 (custom)
    // Bus 11: Device index 5 (custom)
    
    // Create custom I2C device on Bus 10 with index 5
    auto custom_i2c_10 = std::make_shared<DummyI2cDevice>();
    int device_10 = comm_manager.RegisterCustomI2cDevice(10, custom_i2c_10, 0x28);
    
    // Create custom I2C device on Bus 11 with index 5
    auto custom_i2c_11 = std::make_shared<DummyI2cDevice>();
    int device_11 = comm_manager.RegisterCustomI2cDevice(11, custom_i2c_11, 0x29);
    
    Logger::GetInstance().Info("IMU_EXAMPLE", "Created devices:");
    Logger::GetInstance().Info("IMU_EXAMPLE", "  Bus 0:  Device index 5 (built-in)");
    Logger::GetInstance().Info("IMU_EXAMPLE", "  Bus 10: Device index %d (custom)", device_10);
    Logger::GetInstance().Info("IMU_EXAMPLE", "  Bus 11: Device index %d (custom)", device_11);
    
    // Demonstrate the correct bus-aware access
    Logger::GetInstance().Info("IMU_EXAMPLE", "--- SOLUTION: Bus-aware device access ---");
    
    // Correct way: Specify both bus and device index
    BaseI2c* device_bus0 = comm_manager.GetI2cDevice(0, 5);   // Bus 0, Device 5
    if (device_bus0) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "CORRECT: GetI2cDevice(0, 5) = device on Bus 0");
    }
    
    BaseI2c* device_bus10 = comm_manager.GetI2cDevice(10, device_10); // Bus 10, Device 5
    if (device_bus10) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "CORRECT: GetI2cDevice(10, %d) = device on Bus 10", device_10);
    }
    
    BaseI2c* device_bus11 = comm_manager.GetI2cDevice(11, device_11); // Bus 11, Device 5
    if (device_bus11) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "CORRECT: GetI2cDevice(11, %d) = device on Bus 11", device_11);
    }
    
    // Cleanup
    if (device_10 >= 0) comm_manager.RemoveI2cDevice(device_10);
    if (device_11 >= 0) comm_manager.RemoveI2cDevice(device_11);
    
    Logger::GetInstance().Info("IMU_EXAMPLE", "Device index overlap demonstration completed");
}

void DemonstrateDeviceAddressAmbiguity() {
    Logger::GetInstance().Info("IMU_EXAMPLE", "=== Demonstrating Device Address Ambiguity Issue ===");
    
    auto& comm_manager = CommChannelsManager::GetInstance();
    comm_manager.EnsureInitialized();
    
    // Create devices with the same address on different buses
    Logger::GetInstance().Info("IMU_EXAMPLE", "Creating devices with same address on different buses...");
    
    // Bus 0: Device at address 0x28 (BNO08x - built-in)
    // Bus 10: Device at address 0x28 (Custom sensor)
    // Bus 11: Device at address 0x28 (Another custom sensor)
    
    // Create custom I2C device on Bus 10 with address 0x28
    auto custom_i2c_10 = std::make_shared<DummyI2cDevice>();
    int device_10 = comm_manager.RegisterCustomI2cDevice(10, custom_i2c_10, 0x28);
    
    // Create custom I2C device on Bus 11 with address 0x28
    auto custom_i2c_11 = std::make_shared<DummyI2cDevice>();
    int device_11 = comm_manager.RegisterCustomI2cDevice(11, custom_i2c_11, 0x28);
    
    Logger::GetInstance().Info("IMU_EXAMPLE", "Created devices:");
    Logger::GetInstance().Info("IMU_EXAMPLE", "  Bus 0:  Device at address 0x28 (BNO08x - built-in)");
    Logger::GetInstance().Info("IMU_EXAMPLE", "  Bus 10: Device at address 0x28 (Custom sensor)");
    Logger::GetInstance().Info("IMU_EXAMPLE", "  Bus 11: Device at address 0x28 (Another custom sensor)");
    
    // Demonstrate the correct bus-aware access
    Logger::GetInstance().Info("IMU_EXAMPLE", "--- SOLUTION: Bus-aware address checking ---");
    
    // Correct way: Specify both bus and address
    bool bus0_check = comm_manager.HasI2cDeviceAtAddress(0, 0x28);   // Bus 0, Address 0x28
    if (bus0_check) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "CORRECT: HasI2cDeviceAtAddress(0, 0x28) = true (BNO08x on Bus 0)");
    }
    
    bool bus10_check = comm_manager.HasI2cDeviceAtAddress(10, 0x28); // Bus 10, Address 0x28
    if (bus10_check) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "CORRECT: HasI2cDeviceAtAddress(10, 0x28) = true (Custom sensor on Bus 10)");
    }
    
    bool bus11_check = comm_manager.HasI2cDeviceAtAddress(11, 0x28); // Bus 11, Address 0x28
    if (bus11_check) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "CORRECT: HasI2cDeviceAtAddress(11, 0x28) = true (Another custom sensor on Bus 11)");
    }
    
    // Check non-existent address
    bool non_existent = comm_manager.HasI2cDeviceAtAddress(0, 0x99);  // Bus 0, Address 0x99
    if (!non_existent) {
        Logger::GetInstance().Info("IMU_EXAMPLE", "CORRECT: HasI2cDeviceAtAddress(0, 0x99) = false (No device at this address)");
    }
    
    // Cleanup
    if (device_10 >= 0) comm_manager.RemoveI2cDevice(device_10);
    if (device_11 >= 0) comm_manager.RemoveI2cDevice(device_11);
    
    Logger::GetInstance().Info("IMU_EXAMPLE", "Device address ambiguity demonstration completed");
}

//==============================================================================
// MAIN EXAMPLE FUNCTION
//==============================================================================

/**
 * @brief Main example function demonstrating all ImuManager features
 */
void RunImuManagerExample() {
    Logger::GetInstance().Info(TAG, "Starting ImuManager Example with Multiple Device Support");
    Logger::GetInstance().Info(TAG, "=====================================================");
    
    // Example 1: Basic onboard IMU usage
    DemonstrateOnboardImu();
    os_delay_msec(1000)); // 1 second delay
    
    // Example 2: Device management and status
    DemonstrateDeviceManagement();
    os_delay_msec(1000)); // 1 second delay
    
    // Example 3: External I2C IMU device with simplified API
    DemonstrateExternalImu();
    os_delay_msec(1000)); // 1 second delay
    
    // Example 4: External SPI IMU device
    DemonstrateSpiImu();
    os_delay_msec(1000)); // 1 second delay
    
    // Example 5: Interrupt mode (if supported)
    DemonstrateInterruptMode();

    // Example 6: Flexible device creation
    DemonstrateFlexibleDeviceCreation();

    // Example 7: Flexible external buses
    DemonstrateFlexibleExternalBuses();
    
    // Example 8: Bus indexing architecture
    DemonstrateBusIndexingArchitecture();

    // Example 9: Device index overlap issue
    DemonstrateDeviceIndexOverlapIssue();

    // Example 10: Device address ambiguity issue
    DemonstrateDeviceAddressAmbiguity();
    
    Logger::GetInstance().Info(TAG, "=====================================================");
    Logger::GetInstance().Info(TAG, "ImuManager Example completed successfully!");
}

//==============================================================================
// USAGE NOTES
//==============================================================================

/**
 * @brief Key features demonstrated in this example:
 * 
 * 1. **Multiple Device Support**: 
 *    - Onboard BNO08x (index 0) - always available
 *    - External BNO08x devices (indices 1-3) - user-created
 * 
 * 2. **Flexible Transport**:
 *    - I2C transport for onboard and external devices
 *    - SPI transport for external devices
 * 
 * 3. **Device Management**:
 *    - Dynamic creation/deletion of external devices
 *    - Device status queries and validation
 *    - Initialization status tracking
 * 
 * 4. **Indexed Access**:
 *    - GetBno08xHandler(deviceIndex) for handler access
 *    - GetBno085Driver(deviceIndex) for direct driver access
 * 
 * 5. **Interrupt Support**:
 *    - ConfigureInterrupt(deviceIndex, callback)
 *    - EnableInterrupt(deviceIndex) / DisableInterrupt(deviceIndex)
 *    - WaitForInterrupt(deviceIndex, timeout)
 * 
 * 6. **Thread Safety**:
 *    - All operations are thread-safe
 *    - Proper mutex protection
 *    - Atomic operations for status tracking
 * 
 * 7. **Error Handling**:
 *    - Comprehensive error codes
 *    - Graceful failure handling
 *    - Detailed logging
 * 
 * This design follows the same architectural excellence as MotorController
 * for consistent device management across the HardFOC system.
 */ 