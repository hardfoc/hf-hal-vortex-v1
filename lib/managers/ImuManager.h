#pragma once

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <array>
#include "RtosMutex.h"
#include "handlers/bno08x/Bno08xHandler.h"

// Forward declarations (types defined in other managers)
class CommChannelsManager;
enum class SpiDeviceId : uint8_t;
class GpioManager;
class BaseGpio;
class BaseI2c;
class BaseSpi;

/**
 * @class ImuManager
 * @brief Singleton for managing multiple IMU devices with indexed access and flexible device management.
 *
 * This manager provides a global singleton for managing IMU devices, following the same pattern as MotorController:
 * - Always creates and manages the onboard BNO08x IMU device (device index 0)
 * - Allows dynamic creation/deletion of external IMU devices (indices 1-3)
 * - Array-based access to IMU handler instances by device index
 * - Thread-safe device registration and access
 * - Board-aware device management with predefined I2C/SPI assignments
 * - Currently supports BNO08x IMU family (extensible for other IMU types)
 *
 * **Key Features:**
 * - Multiple IMU device support with indexed access
 * - Lazy initialization of onboard BNO08x device
 * - Dynamic external device creation/deletion
 * - Thread-safe singleton pattern with RtosMutex
 * - Exception-free operation with pointer-based returns
 * - Proper device initialization and lifecycle management
 * - ESP-IDF v5.5+ I2C/SPI integration
 *
 * **Device Indexing:**
 * - Index 0: Onboard BNO08x IMU (always available, auto-created)
 * - Index 1: External IMU device 1 (optional, user-created)
 * - Index 2: External IMU device 2 (optional, user-created)
 * - Index 3: External IMU device 3 (optional, user-created)
 *
 * **Usage Example:**
 * @code
 * auto& imu_mgr = ImuManager::GetInstance();
 * if (imu_mgr.EnsureInitialized()) {
 *     // Access onboard BNO08x sensor driver (index 0)
 *     IBno08xDriverOps* sensor = imu_mgr.GetSensor(0);
 *     if (sensor) {
 *         // Configure sensors via driver interface
 *         sensor->EnableSensor(BNO085Sensor::RotationVector, 50, 0.0f);
 *     }
 *     
 *     // Create external BNO08x device (index 1) with runtime I2C device creation
 *     if (imu_mgr.CreateExternalBno08xDevice(1, 0x48, 400000)) {
 *         IBno08xDriverOps* ext_sensor = imu_mgr.GetSensor(1);
 *         if (ext_sensor) {
 *             ext_sensor->EnableSensor(BNO085Sensor::Accelerometer, 100, 0.0f);
 *         }
 *     }
 *     
 *     // Create external BNO08x device (index 2) with SPI interface
 *     if (imu_mgr.CreateExternalBno08xDevice(2, SpiDeviceId::EXTERNAL_DEVICE_1)) {
 *         IBno08xDriverOps* ext_sensor = imu_mgr.GetSensor(2);
 *         if (ext_sensor) {
 *             ext_sensor->EnableSensor(BNO085Sensor::RotationVector, 50, 0.0f);
 *         }
 *     }
 * }
 * @endcode
 *
 * @note This manager follows the same architectural excellence as MotorController
 *       for consistent device management across the HardFOC system.
 */

//==============================================================================
// IMU ERROR CODES
//==============================================================================

/**
 * @brief IMU manager error codes for consistent error reporting.
 *
 * Manager-layer errors for BNO08x device lifecycle and data operations.
 * Driver-level BNO08x errors are handled internally by the handler.
 */
enum class ImuError : uint8_t {
    SUCCESS = 0,
    NOT_INITIALIZED,
    INITIALIZATION_FAILED,
    DEVICE_ALREADY_EXISTS,
    DEVICE_NOT_FOUND,
    INVALID_DEVICE_INDEX,
    CANNOT_DELETE_ONBOARD,
    INVALID_I2C_ADDRESS,
    DEPENDENCY_NOT_READY,
    COMMUNICATION_FAILED,
    I2C_DEVICE_CREATION_FAILED,
    HANDLER_CREATION_FAILED,
    INTERRUPT_ERROR,
    ALREADY_DEINITIALIZED,
    MUTEX_LOCK_FAILED
};

/** @brief Convert ImuError to string for debugging. */
constexpr const char* ImuErrorToString(ImuError error) noexcept {
    switch (error) {
        case ImuError::SUCCESS:                    return "Success";
        case ImuError::NOT_INITIALIZED:            return "Not initialized";
        case ImuError::INITIALIZATION_FAILED:      return "Initialization failed";
        case ImuError::DEVICE_ALREADY_EXISTS:      return "Device already exists";
        case ImuError::DEVICE_NOT_FOUND:           return "Device not found";
        case ImuError::INVALID_DEVICE_INDEX:       return "Invalid device index";
        case ImuError::CANNOT_DELETE_ONBOARD:      return "Cannot delete onboard device";
        case ImuError::INVALID_I2C_ADDRESS:        return "Invalid I2C address";
        case ImuError::DEPENDENCY_NOT_READY:       return "Dependency not ready";
        case ImuError::COMMUNICATION_FAILED:       return "Communication failed";
        case ImuError::I2C_DEVICE_CREATION_FAILED: return "I2C device creation failed";
        case ImuError::HANDLER_CREATION_FAILED:    return "Handler creation failed";
        case ImuError::INTERRUPT_ERROR:            return "Interrupt configuration error";
        case ImuError::ALREADY_DEINITIALIZED:      return "Already deinitialized";
        case ImuError::MUTEX_LOCK_FAILED:          return "Mutex lock failed";
        default:                                   return "Unknown error";
    }
}

class ImuManager {
public:
    static constexpr uint8_t MAX_IMU_DEVICES = 4;           ///< Maximum supported IMU devices
    static constexpr uint8_t ONBOARD_IMU_INDEX = 0;         ///< Onboard IMU device index (BNO08x)
    static constexpr uint8_t EXTERNAL_IMU_1_INDEX = 1;      ///< External IMU device 1 index
    static constexpr uint8_t EXTERNAL_IMU_2_INDEX = 2;      ///< External IMU device 2 index
    static constexpr uint8_t EXTERNAL_IMU_3_INDEX = 3;      ///< External IMU device 3 index

public:
    /**
     * @brief Get the singleton instance of ImuManager.
     * @return Reference to the singleton ImuManager.
     */
    static ImuManager& GetInstance() noexcept;

    //**************************************************************************//
    //**                  DEVICE MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Ensure the IMU manager system is initialized.
     * @note This automatically creates the onboard BNO08x IMU device using CommChannelsManager
     * @return true if initialization successful, false otherwise
     */
    bool EnsureInitialized() noexcept;

    /**
     * @brief Check if the IMU manager is initialized.
     * @return true if initialized, false otherwise
     */
    bool IsInitialized() const noexcept;

    /**
     * @brief Deinitialize all IMUs and release resources.
     * @return true if deinitialized successfully, false otherwise
     */
    bool Deinitialize() noexcept;

    //**************************************************************************//
    //**                  HANDLER AND DRIVER MANAGEMENT                       **//
    //**************************************************************************//

    /**
     * @brief Get access to BNO08x IMU handler by device index.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return Pointer to Bno08xHandler if valid and active, nullptr otherwise
     * @note Returns nullptr if deviceIndex is invalid, device not active, or not initialized
     */
    Bno08xHandler* GetBno08xHandler(uint8_t deviceIndex = ONBOARD_IMU_INDEX) noexcept;

    /**
     * @brief Get direct access to the sensor driver for a device (type-erased BNO085 API).
     *
     * Use this to call driver methods directly: Update(), EnableSensor(), DisableSensor(),
     * SetCallback(), HasNewData(), GetLatest(), etc. The pointer is valid while the
     * handler for that device exists. Include Bno08xHandler.h for IBno08xDriverOps and
     * BNO085Sensor types.
     *
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return Non-owning pointer to the driver interface, or nullptr if invalid/not ready
     */
    IBno08xDriverOps* GetSensor(uint8_t deviceIndex = ONBOARD_IMU_INDEX) noexcept;

    //**************************************************************************//
    //**                  DEVICES MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Create an external BNO08x IMU device on I2C interface with specified address.
     * @param deviceIndex External device index (1, 2, or 3 only)
     * @param i2c_address 7-bit I2C device address (0x08-0x77)
     * @param i2c_speed_hz I2C speed in Hz (default: 400000 for 400kHz)
     * @param config Optional BNO08x configuration (defaults to default config)
     * @return true if device created successfully, false otherwise
     * @note This method handles both I2C device creation and BNO08x handler creation internally
     */
    bool CreateExternalBno08xDevice(uint8_t deviceIndex, 
                                   uint8_t i2c_address,
                                   uint32_t i2c_speed_hz = 400000,
                                   const Bno08xConfig& config = Bno08xHandler::GetDefaultConfig());



    /**
     * @brief Create an external BNO08x IMU device on SPI interface.
     * @param deviceIndex External device index (1, 2, or 3 only)
     * @param spiDeviceId SPI device ID for communication
     * @param config Optional BNO08x configuration (defaults to default config)
     * @return true if device created successfully, false otherwise
     */
    bool CreateExternalBno08xDevice(uint8_t deviceIndex, 
                                   SpiDeviceId spiDeviceId,
                                   const Bno08xConfig& config = Bno08xHandler::GetDefaultConfig());

    /**
     * @brief Create an external BNO08x IMU device using direct BaseI2c interface.
     * @param deviceIndex External device index (1, 2, or 3 only)
     * @param i2c_interface Direct reference to BaseI2c interface
     * @param config Optional BNO08x configuration (defaults to default config)
     * @return true if device created successfully, false otherwise
     * @note This is the most flexible method - allows any external I2C interface
     */
    bool CreateExternalBno08xDevice(uint8_t deviceIndex, 
                                   BaseI2c& i2c_interface,
                                   const Bno08xConfig& config = Bno08xHandler::GetDefaultConfig());

    /**
     * @brief Create an external BNO08x IMU device using direct BaseSpi interface.
     * @param deviceIndex External device index (1, 2, or 3 only)
     * @param spi_interface Direct reference to BaseSpi interface
     * @param config Optional BNO08x configuration (defaults to default config)
     * @return true if device created successfully, false otherwise
     * @note This is the most flexible method - allows any external SPI interface
     */
    bool CreateExternalBno08xDevice(uint8_t deviceIndex, 
                                   BaseSpi& spi_interface,
                                   const Bno08xConfig& config = Bno08xHandler::GetDefaultConfig());

    /**
     * @brief Delete an external IMU device.
     * @param deviceIndex External device index (1, 2, or 3 only)
     * @return true if device deleted successfully, false otherwise
     * @note Cannot delete onboard device (index 0). Only external devices can be deleted.
     */
    bool DeleteExternalDevice(uint8_t deviceIndex);

    /**
     * @brief Get the number of active IMU devices.
     * @return Number of active devices (1 to MAX_IMU_DEVICES)
     * @note Always includes onboard device, plus any active external devices
     */
    uint8_t GetDeviceCount() const noexcept;

    /**
     * @brief Check if a device index is valid and has an active device.
     * @param deviceIndex Device index to check
     * @return true if device exists and is active, false otherwise
     */
    bool IsDeviceValid(uint8_t deviceIndex) const noexcept;

    /**
     * @brief Check if an external device slot is available for creation.
     * @param deviceIndex External device index (1, 2, or 3 only)
     * @return true if slot is available, false if occupied or invalid index
     */
    bool IsExternalSlotAvailable(uint8_t deviceIndex) const noexcept;

    /**
     * @brief Get list of active device indices.
     * @return Vector of active device indices
     */
    std::vector<uint8_t> GetActiveDeviceIndices() const noexcept;

    /**
     * @brief Initialize all devices and report status.
     * @return Vector of initialization results (true/false) for each active device
     */
    std::vector<bool> InitializeAllDevices();

    /**
     * @brief Get initialization status for all devices.
     * @return Vector of initialization status for each active device
     */
    std::vector<bool> GetInitializationStatus() const;

    //**************************************************************************//
    //**                  DEVICE INFORMATION METHODS                          **//
    //**************************************************************************//

    /**
     * @brief Get information about available IMU devices.
     * @return Vector of device names/identifiers
     */
    std::vector<std::string> GetAvailableDevices() const noexcept;

    /**
     * @brief Get device type by index.
     * @param deviceIndex Device index
     * @return Device type string or "Unknown" if invalid
     */
    std::string GetDeviceType(uint8_t deviceIndex) const noexcept;

    //**************************************************************************//
    //**                  INTERRUPT SUPPORT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Configure GPIO interrupt for BNO08x INT pin on specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @param callback Optional callback function executed in interrupt context (keep minimal)
     * @return true if interrupt configuration successful, false otherwise
     * @note Uses PCAL_IMU_INT functional pin through GpioManager for onboard device
     * @note External devices may use different interrupt pins
     */
    bool ConfigureInterrupt(uint8_t deviceIndex, std::function<void()> callback = nullptr) noexcept;

    /**
     * @brief Enable BNO08x GPIO interrupt for specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return true if interrupt enabled successfully, false otherwise
     * @note Must call ConfigureInterrupt() first
     */
    bool EnableInterrupt(uint8_t deviceIndex) noexcept;

    /**
     * @brief Disable BNO08x GPIO interrupt for specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return true if interrupt disabled successfully, false otherwise
     */
    bool DisableInterrupt(uint8_t deviceIndex) noexcept;

    /**
     * @brief Check if interrupt is configured and enabled for specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return true if interrupt is active, false otherwise
     */
    bool IsInterruptEnabled(uint8_t deviceIndex) const noexcept;

    /**
     * @brief Wait for interrupt signal with timeout for specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @param timeout_ms Timeout in milliseconds (0 = wait indefinitely)
     * @return true if interrupt occurred, false on timeout
     * @note Useful for interrupt-driven processing in tasks
     */
    bool WaitForInterrupt(uint8_t deviceIndex, uint32_t timeout_ms = 0) noexcept;

    /**
     * @brief Get interrupt statistics for monitoring for specific device.
     * @param deviceIndex Device index (0=onboard, 1-3=external)
     * @return Number of interrupts processed since initialization
     */
    uint32_t GetInterruptCount(uint8_t deviceIndex) const noexcept;
    
    /**
     * @brief Dump comprehensive system statistics to log as INFO level.
     * Logs all device statistics, interrupt counts, and system health information.
     */
    void DumpStatistics() const noexcept;

    // Delete copy/move constructors and assignment operators
    ImuManager(const ImuManager&) = delete;
    ImuManager& operator=(const ImuManager&) = delete;
    ImuManager(ImuManager&&) = delete;
    ImuManager& operator=(ImuManager&&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern.
     */
    ImuManager() noexcept;

    /**
     * @brief Private destructor.
     */
    ~ImuManager() noexcept;

    /**
     * @brief Initialize the IMU manager system.
     * @note This automatically creates the onboard BNO08x IMU device using CommChannelsManager
     * @return true if initialization successful, false otherwise
     */
    bool Initialize() noexcept;

    /**
     * @brief Initialize the onboard BNO08x IMU device with I2C transport.
     * @return true if initialization successful, false otherwise
     */
    bool InitializeOnboardBno08xDevice() noexcept;

    /**
     * @brief Initialize GPIO interrupt for BNO08x INT pin.
     * @return true if GPIO setup successful, false otherwise
     */
    bool InitializeInterruptGpio() noexcept;

    /**
     * @brief Validate if device index is for external device.
     * @param deviceIndex Device index to validate
     * @return true if index is for external device (1, 2, or 3), false otherwise
     */
    bool IsExternalDeviceIndex(uint8_t deviceIndex) const noexcept;

    /**
     * @brief GPIO interrupt handler for BNO08x INT pin.
     * @param gpio Pointer to the GPIO that triggered interrupt
     * @param trigger Interrupt trigger type
     * @param user_data Pointer to user data (ImuManager instance)
     */
    static void GpioInterruptHandler(BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) noexcept;

    // ===============================
    // SYSTEM STATE
    // ===============================

    /**
     * @brief System initialization state (atomic for thread safety).
     */
    std::atomic<bool> initialized_{false};

    /**
     * @brief Main system mutex for thread-safe operations.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex manager_mutex_;

    // ===============================
    // DEVICE STORAGE
    // ===============================

    /**
     * @brief Array of BNO08x IMU handlers (one per device slot).
     * Uses unique_ptr for exclusive ownership of each handler.
     * Protected by RtosMutex for thread-safe access.
     */
    std::array<std::unique_ptr<Bno08xHandler>, MAX_IMU_DEVICES> bno08x_handlers_;

    /**
     * @brief Device initialization status tracking.
     * Tracks which devices have been successfully initialized.
     */
    std::array<bool, MAX_IMU_DEVICES> device_initialized_;

    /**
     * @brief Device active status tracking.
     * Tracks which device slots have active devices.
     */
    std::array<bool, MAX_IMU_DEVICES> device_active_;

    /**
     * @brief Onboard device creation tracking.
     * Tracks if onboard device has been created.
     */
    bool onboard_device_created_ = false;

    /**
     * @brief Track I2C device indices created by ImuManager for proper cleanup.
     * This is needed because external I2C devices are managed by ImuManager,
     * but their actual I2C handles are managed by CommChannelsManager.
     * We need to keep track of which external I2C devices were created by ImuManager
     * to ensure they are deleted when the ImuManager is deinitialized.
     */
    std::array<int, MAX_IMU_DEVICES> i2c_device_indices_;

    // ===============================
    // DEPENDENCIES
    // ===============================

    /**
     * @brief Reference to communication manager for I2C/SPI access.
     * Used for creating IMU devices with proper transport interfaces.
     */
    CommChannelsManager* comm_manager_ = nullptr;

    /**
     * @brief Reference to GPIO manager for interrupt pin access.
     * Used for configuring interrupt pins for IMU devices.
     */
    GpioManager* gpio_manager_ = nullptr;

    // ===============================
    // INTERRUPT SUPPORT
    // ===============================

    /**
     * @brief GPIO interrupt support for onboard device.
     * External devices may have their own interrupt configurations.
     */
    BaseGpio* interrupt_gpio_ = nullptr;                    ///< Raw pointer for compatibility
    std::shared_ptr<BaseGpio> interrupt_gpio_shared_;       ///< Shared pointer for safe ownership
    std::function<void()> interrupt_callback_;
    bool interrupt_configured_ = false;
    bool interrupt_enabled_ = false;
    std::atomic<uint32_t> interrupt_count_{0};
    void* interrupt_semaphore_ = nullptr;  ///< FreeRTOS semaphore for WaitForInterrupt()
}; 