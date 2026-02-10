#ifndef COMPONENT_HANDLER_MOTOR_CONTROLLER_H_
#define COMPONENT_HANDLER_MOTOR_CONTROLLER_H_

#include "Tmc9660Handler.h"
#include "CommChannelsManager.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"
#include <memory>
#include <array>
#include <vector>

/**
 * @file MotorController.h
 * @brief Singleton class for managing multiple TMC9660 motor controllers and their interfaces.
 *
 * This class provides a global singleton for managing TMC9660 motor controllers, including:
 * - Always creates and manages the onboard TMC9660 device (device index 0)
 * - Allows dynamic creation/deletion of external TMC9660 devices (indices 2-3)
 * - Array-based access to Tmc9660Handler instances by device index
 * - Access to individual TMC9660 drivers, GPIO, and ADC by device index
 * - Thread-safe device registration and access
 * - Board-aware device management with predefined CS pin assignments
 */
class MotorController {
public:
    static constexpr uint8_t MAX_TMC9660_DEVICES = 4;   ///< Maximum supported TMC9660 devices (board + external)
    static constexpr uint8_t ONBOARD_TMC9660_INDEX = 0; ///< Onboard TMC9660 device index (SPI2_CS_TMC9660)
    static constexpr uint8_t EXTERNAL_DEVICE_1_INDEX = 2; ///< External device 1 index (EXT_GPIO_CS_1)  
    static constexpr uint8_t EXTERNAL_DEVICE_2_INDEX = 3; ///< External device 2 index (EXT_GPIO_CS_2)
public:
    /**
     * @brief Get the singleton instance.
     */
    static MotorController& GetInstance();

    //**************************************************************************//
    //**                  DEVICE MANAGEMENT METHODS                           **//
    //**************************************************************************//

    bool EnsureInitialized() noexcept {
        if (!initialized_) {
            initialized_ = Initialize();
        }
        return initialized_;
    }

    inline bool IsInitialized() const noexcept { return initialized_; }

    //**************************************************************************//
    //**                  HANDLER AND DRIVER MANAGEMENT                       **//
    //**************************************************************************//

    /**
     * @brief Get access to Tmc9660Handler by device index.
     * @param deviceIndex Device index (0=onboard, 2-3=external)
     * @return Pointer to Tmc9660Handler if valid and active, nullptr otherwise
     * @note Returns nullptr if deviceIndex is invalid, device not active, or not initialized
     */
    Tmc9660Handler* handler(uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /**
     * @brief Get access to the underlying TMC9660 driver by device index.
     * @param deviceIndex Device index (0=onboard, 2-3=external) 
     * @return Shared pointer to TMC9660 driver, nullptr if invalid/not ready
     * @note Returns nullptr if deviceIndex is invalid, device not active/initialized, or driver unavailable
     */
    std::shared_ptr<TMC9660> driver(uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    //**************************************************************************//
    //**                  DEVICES MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Create an external TMC9660 device on specified CS line.
     * @param csDeviceIndex External device CS index (2 or 3 only)
     * @param spiDeviceId SPI device ID for communication (EXTERNAL_DEVICE_1 or EXTERNAL_DEVICE_2)
     * @param address TMC9660 device address
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @return true if device created successfully, false otherwise
     * @note Only EXTERNAL_DEVICE_1_INDEX (2) and EXTERNAL_DEVICE_2_INDEX (3) are allowed
     */
    bool CreateExternalDevice(uint8_t csDeviceIndex, 
                            SpiDeviceId spiDeviceId, 
                            uint8_t address,
                            const tmc9660::BootloaderConfig* bootCfg = nullptr);

    /**
     * @brief Delete an external TMC9660 device.
     * @param csDeviceIndex External device CS index (2 or 3 only)
     * @return true if device deleted successfully, false otherwise
     * @note Cannot delete onboard device (index 0). Only external devices can be deleted.
     */
    bool DeleteExternalDevice(uint8_t csDeviceIndex);

    /**
     * @brief Get the number of active TMC9660 devices.
     * @return Number of active devices (1 to MAX_TMC9660_DEVICES)
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
     * @param csDeviceIndex External device CS index (2 or 3 only)
     * @return true if slot is available, false if occupied or invalid index
     */
    bool IsExternalSlotAvailable(uint8_t csDeviceIndex) const noexcept;

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
    
    /**
     * @brief Dump comprehensive system statistics to log as INFO level.
     * Logs all device statistics, handler status, and system health information.
     */
    void DumpStatistics() const noexcept;

private:
    MotorController();
    ~MotorController() = default;
    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;

    /**
     * @brief Initialize the motor controller system.
     * @note This automatically creates the onboard TMC9660 device using CommChannelsManager
     * @return true if initialization successful, false otherwise
     */
    bool Initialize();

    /**
     * @brief Validate if CS device index is for external device.
     * @param csDeviceIndex Device index to validate
     * @return true if index is for external device (2 or 3), false otherwise
     */
    bool IsExternalDeviceIndex(uint8_t csDeviceIndex) const noexcept;

    std::array<std::unique_ptr<Tmc9660Handler>, MAX_TMC9660_DEVICES> tmcHandlers_;
    std::array<bool, MAX_TMC9660_DEVICES> deviceInitialized_;
    std::array<bool, MAX_TMC9660_DEVICES> deviceActive_;    ///< Track which devices are active
    
    bool onboardDeviceCreated_;         ///< Track if onboard device has been created
    bool initialized_;                  ///< Track if system has been initialized
    mutable RtosMutex deviceMutex_;     ///< RTOS mutex for thread-safe device access
};

#endif // COMPONENT_HANDLER_MOTOR_CONTROLLER_H_ 