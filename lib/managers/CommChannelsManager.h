#ifndef COMPONENT_HANDLER_COMM_CHANNELS_MANAGER_H_
#define COMPONENT_HANDLER_COMM_CHANNELS_MANAGER_H_

#include <memory>
#include <vector>
#include <cstdint>
#include <atomic> // Added for std::atomic
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/RtosMutex.h" // Added for RtosMutex

// Forward declarations for ESP32 comm interface classes
class EspSpiBus;
class EspSpiDevice;
class EspI2cBus;
class EspI2cDevice;
class EspUart;
class EspCan;

// Base interface includes
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseSpi.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseI2c.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseUart.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseCan.h"

// Board mapping includes (for pin/bus config)
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config.hpp"

/**
 * @enum SpiDeviceId
 * @brief Enumeration for SPI device identification on the board.
 * 
 * This enum provides a type-safe way to access specific SPI devices by their
 * functional purpose rather than numeric indices. The order matches the CS pin
 * configuration in kSpiCsPins array.
 */
enum class SpiDeviceId : uint8_t {
    TMC9660_MOTOR_CONTROLLER = 0,  ///< TMC9660 motor controller (SPI Mode 3)
    AS5047U_POSITION_ENCODER = 1,  ///< AS5047U position encoder (SPI Mode 1)
    EXTERNAL_DEVICE_1 = 2,         ///< External device 1 (SPI Mode 0)
    EXTERNAL_DEVICE_2 = 3,         ///< External device 2 (SPI Mode 0)
    
    // Aliases for common usage
    MOTOR_CONTROLLER = TMC9660_MOTOR_CONTROLLER,
    POSITION_ENCODER = AS5047U_POSITION_ENCODER,
    TMC9660_SPI = TMC9660_MOTOR_CONTROLLER,  ///< Alias for backward compatibility
    
    SPI_DEVICE_COUNT  ///< Total number of SPI devices
};

/**
 * @enum I2cDeviceId
 * @brief Enumeration for I2C device identification on the board.
 * 
 * This enum provides a type-safe way to access specific I2C devices by their
 * functional purpose rather than numeric indices or addresses.
 */
enum class I2cDeviceId : uint8_t {
    BNO08X_IMU = 0,              ///< BNO08x IMU sensor (address 0x4A or 0x4B)
    PCAL9555_GPIO_EXPANDER = 1,  ///< PCAL9555 GPIO expander (address 0x20-0x27)
    
    // Aliases for common usage
    IMU = BNO08X_IMU,
    GPIO_EXPANDER = PCAL9555_GPIO_EXPANDER,
    
    I2C_DEVICE_COUNT  ///< Total number of I2C devices
};

/**
 * @class CommChannelsManager
 * @brief Singleton for managing all board comm channels (SPI, I2C, UART, CAN).
 *
 * - Instantiates and owns all available comm buses for the board.
 * - Provides indexed accessors to the base interface for each bus.
 * - Board-agnostic: does not know about device purposes, only hardware buses.
 * - Uses board mapping for pin/bus configuration.
 * - Follows the singleton pattern (see GpioManager/AdcManager).
 */
class CommChannelsManager {
public:
    /**
     * @brief Get the singleton instance.
     */
    static CommChannelsManager& GetInstance() noexcept;

    // Non-copyable, non-movable
    CommChannelsManager(const CommChannelsManager&) = delete;
    CommChannelsManager& operator=(const CommChannelsManager&) = delete;
    CommChannelsManager(CommChannelsManager&&) = delete;
    CommChannelsManager& operator=(CommChannelsManager&&) = delete;

    //==================== INITIALZATION ====================//
    
    // Lazy Initialization
    bool EnsureInitialized() noexcept {
        if (!initialized_) {
            initialized_ = Initialize();
        }
        return initialized_;
    }
    bool EnsureDeinitialized() noexcept {
        if (initialized_) {
            initialized_ = !Deinitialize();
        }
        return !initialized_;
    }

    [[nodiscard]] bool IsInitialized() const noexcept {
        return initialized_;
    }

    //==================== SPI Accessors ====================//
    
    /**
     * @brief Get reference to a SPI device by bus index and device index.
     * @param bus_index Bus index where device is located
     * @param device_index Index of the device on that bus
     * @return Pointer to BaseSpi device, or nullptr if invalid
     */
    BaseSpi* GetSpiDevice(uint8_t bus_index, int device_index) noexcept;
    
    /**
     * @brief Get reference to a SPI device by device ID (enumeration-based access).
     * @param device_id Device identifier from SpiDeviceId enum
     * @return Pointer to BaseSpi device, or nullptr if invalid
     */
    BaseSpi* GetSpiDevice(SpiDeviceId device_id) noexcept;

    //==================== I2C Accessors ====================//
    
    /**
     * @brief Get reference to an I2C device by bus index and device index.
     * @param bus_index Bus index where device is located
     * @param device_index Index of the device on that bus
     * @return Pointer to BaseI2c device, or nullptr if invalid
     */
    BaseI2c* GetI2cDevice(uint8_t bus_index, int device_index) noexcept;
    
    /**
     * @brief Get reference to an I2C device by device ID (enumeration-based access).
     * @param device_id Device identifier from I2cDeviceId enum
     * @return Pointer to BaseI2c device, or nullptr if invalid
     */
    BaseI2c* GetI2cDevice(I2cDeviceId device_id) noexcept;
    
    //==================== RUNTIME I2C DEVICE MANAGEMENT ====================//
    
    /**
     * @brief Create an I2C device on the primary bus (Bus 0).
     * @param device_address 7-bit I2C device address
     * @param speed_hz I2C bus speed in Hz (default: 400kHz)
     * @return Device index if successful, -1 if failed
     * @note Device will be created on Bus 0 (primary ESP32 I2C bus)
     */
    int CreateI2cDevice(uint8_t device_address, uint32_t speed_hz = 400000) noexcept;
    
    /**
     * @brief Create an I2C device on a specific bus.
     * @param bus_index Bus index where to create the device (0-255)
     * @param device_address 7-bit I2C device address
     * @param speed_hz I2C bus speed in Hz (default: 400kHz)
     * @return Device index if successful, -1 if failed
     * @note The manager takes ownership of the created device
     */
    int CreateI2cDevice(uint8_t bus_index, uint8_t device_address, uint32_t speed_hz = 400000) noexcept;
    
    /**
     * @brief Register a custom BaseI2c device with the manager on a specific bus.
     * @param bus_index Bus index where to register the device (0-255)
     * @param custom_device Shared pointer to custom BaseI2c implementation
     * @param device_address I2C address for tracking (optional, 0xFF for unknown)
     * @return Device index if successful, -1 if failed
     * @note The manager takes ownership of the device
     */
    int RegisterCustomI2cDevice(uint8_t bus_index, std::shared_ptr<BaseI2c> custom_device, uint8_t device_address = 0xFF) noexcept;
    
    /**
     * @brief Register a custom BaseI2c device with the manager using direct interface.
     * @param custom_device Shared pointer to custom BaseI2c implementation
     * @param device_address I2C address for tracking (optional, 0xFF for unknown)
     * @param bus_index Bus index for association (optional, 0xFF for auto-assign)
     * @return Device index if successful, -1 if failed
     * @note This is the most flexible method - allows any external I2C interface
     * @note The manager takes ownership of the device
     * @note If bus_index is 0xFF, device will be assigned to next available bus
     */
    int RegisterCustomI2cDevice(std::shared_ptr<BaseI2c> custom_device, uint8_t device_address = 0xFF, uint8_t bus_index = 0xFF) noexcept;
    
    /**
     * @brief Check if an I2C device exists at the specified address on a specific bus.
     * @param bus_index Bus index to check
     * @param device_address 7-bit I2C device address to check
     * @return true if device exists on the specified bus, false otherwise
     */
    bool HasI2cDeviceAtAddress(uint8_t bus_index, uint8_t device_address) const noexcept;
    
    //==================== RUNTIME SPI DEVICE MANAGEMENT ====================//
    
    /**
     * @brief Register a custom BaseSpi device with the manager using direct interface.
     * @param custom_device Shared pointer to custom BaseSpi implementation
     * @param device_index Optional device index for tracking (-1 for auto-assign)
     * @param bus_index Bus index for association (optional, 0xFF for auto-assign)
     * @return Device index if successful, -1 if failed
     * @note This is the most flexible method - allows any external SPI interface
     * @note The manager takes ownership of the device
     * @note If bus_index is 0xFF, device will be assigned to next available bus
     */
    int RegisterCustomSpiDevice(std::shared_ptr<BaseSpi> custom_device, int device_index = -1, uint8_t bus_index = 0xFF) noexcept;

    //==================== UART Accessors ====================//
    
    /**
     * @brief Get reference to a UART bus by index.
     * @param bus_index Index of the UART bus
     * @param bus Reference to store the UART bus if successful
     * @return true if bus is available and reference is valid, false otherwise
     */
    bool GetUart(std::size_t bus_index, BaseUart*& bus) noexcept;
    
    /**
     * @brief Get count of available UART buses.
     * @return Number of UART buses
     */
    std::size_t GetUartCount() const noexcept;

    //==================== CAN Accessors ====================//
    
    /**
     * @brief Get reference to a CAN bus by index.
     * @param bus_index Index of the CAN bus
     * @param bus Reference to store the CAN bus if successful
     * @return true if bus is available and reference is valid, false otherwise
     */
    bool GetCan(std::size_t bus_index, BaseCan*& bus) noexcept;
    
    /**
     * @brief Get count of available CAN buses.
     * @return Number of CAN buses
     */
    std::size_t GetCanCount() const noexcept;

    //==================== BUS MANAGEMENT ====================//
    
    /**
     * @brief Get total number of buses (built-in + external).
     * @return Total bus count
     */
    uint8_t GetBusCount() const noexcept;
    
    /**
     * @brief Check if a bus index is available.
     * @param bus_index Bus index to check
     * @return true if bus is available, false otherwise
     */
    bool IsBusAvailable(uint8_t bus_index) const noexcept;
    
    /**
     * @brief Get number of devices on a specific bus.
     * @param bus_index Bus index
     * @return Number of devices on the bus
     */
    uint8_t GetDeviceCountOnBus(uint8_t bus_index) const noexcept;
    
    /**
     * @brief Get MCU-specific I2C bus reference (ESP32 buses only).
     * @param bus_index Bus index (0-2 for built-in ESP32 buses)
     * @param bus Reference to store the ESP32 I2C bus if successful
     * @return true if bus is available and reference is valid, false otherwise
     * @note Use only for advanced ESP32-specific operations
     */
    bool GetMcuI2cBus(uint8_t bus_index, EspI2cBus*& bus) noexcept;
    
    /**
     * @brief Get MCU-specific SPI bus reference (ESP32 buses only).
     * @param bus_index Bus index (0-2 for built-in ESP32 buses)
     * @param bus Reference to store the ESP32 SPI bus if successful
     * @return true if bus is available and reference is valid, false otherwise
     * @note Use only for advanced ESP32-specific operations
     */
    bool GetMcuSpiBus(uint8_t bus_index, EspSpiBus*& bus) noexcept;
    
    /**
     * @brief Dump comprehensive system statistics to log as INFO level.
     * Logs all bus statistics, device counts, and system health information.
     */
    void DumpStatistics() const noexcept;

private:
    CommChannelsManager();
    ~CommChannelsManager() = default;
    CommChannelsManager(const CommChannelsManager&) = delete;
    CommChannelsManager& operator=(const CommChannelsManager&) = delete;

    //==================== PRIVATE METHODS ====================//
    
    /**
     * @brief Initialize all comm channels.
     * @return true if successful, false otherwise
     */
    bool Initialize() noexcept;
    
    /**
     * @brief Deinitialize all comm channels.
     * @return true if successful, false otherwise
     */
    bool Deinitialize() noexcept;
    
    /**
     * @brief Register built-in devices with proper bus associations.
     */
    void RegisterBuiltinDevices() noexcept;
    
    /**
     * @brief Get next available device index for a given bus.
     * @param bus_index Bus index
     * @return Next available device index, or -1 if no slots available
     */
    int GetNextAvailableDeviceIndex(uint8_t bus_index) const noexcept;

    //==================== BUS MANAGEMENT ====================//
    
    // Built-in ESP32 buses (currently 1 each, but extensible)
    std::unique_ptr<EspSpiBus> spi_bus_;      // ESP32 SPI bus (Bus Index 0)
    std::unique_ptr<EspI2cBus> i2c_bus_;      // ESP32 I2C bus (Bus Index 0)
    
    // External buses (user-managed, we only track device associations)
    std::set<uint8_t> external_spi_buses_;    // Track external SPI bus indices (10+)
    std::set<uint8_t> external_i2c_buses_;    // Track external I2C bus indices (10+)
    
    //==================== DEVICE TRACKING ====================//
    
    // Built-in device indices (all on Bus 0)
    std::vector<int> spi_device_indices_;     // Built-in SPI device indices (Bus 0)
    std::vector<int> i2c_device_indices_;     // Built-in I2C device indices (Bus 0)
    
    // External device tracking (user-registered BaseI2c/BaseSpi)
    std::map<int, std::shared_ptr<BaseSpi>> custom_spi_devices_;    // device_index -> BaseSpi
    std::map<int, std::shared_ptr<BaseI2c>> custom_i2c_devices_;    // device_index -> BaseI2c
    
    // Device-to-bus mapping (for all devices)
    std::map<int, uint8_t> spi_device_to_bus_mapping_;    // device_index -> bus_index
    std::map<int, uint8_t> i2c_device_to_bus_mapping_;    // device_index -> bus_index
    
    // I2C-specific tracking
    std::vector<uint8_t> i2c_device_addresses_;    // Built-in I2C device addresses

    // Initialization state
    std::atomic<bool> initialized_{false};
    mutable RtosMutex mutex_;

    // Static configuration for SPI CS pins
    static constexpr HfFunctionalGpioPin kSpiCsPins[] = {
        HfFunctionalGpioPin::SPI2_CS_TMC9660,
        HfFunctionalGpioPin::SPI2_CS_AS5047,
        HfFunctionalGpioPin::EXT_GPIO_CS_1,
        HfFunctionalGpioPin::EXT_GPIO_CS_2,
    };
    static constexpr std::size_t kSpiCsPinCount = sizeof(kSpiCsPins) / sizeof(kSpiCsPins[0]);

    // Internal vectors for managing multiple buses
    std::vector<std::unique_ptr<EspUart>> uart_buses_;
    std::vector<std::unique_ptr<EspCan>> can_buses_;
};

#endif // COMPONENT_HANDLER_COMM_CHANNELS_MANAGER_H_ 