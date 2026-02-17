#ifndef COMPONENT_HANDLER_MOTOR_CONTROLLER_H_
#define COMPONENT_HANDLER_MOTOR_CONTROLLER_H_

#include "handlers/tmc9660/Tmc9660Handler.h"
#include "CommChannelsManager.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"
#include <memory>
#include <array>
#include <atomic>
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
 *
 * @note Since the TMC9660 driver update, handler construction requires host-side GPIO
 *       control pins (RST, DRV_EN, FAULTN, WAKE). These must be provided when creating
 *       TMC9660 devices via the Tmc9660ControlPins struct.
 */

/**
 * @brief Bundle of host-side GPIO references for TMC9660 control pins.
 *
 * These are the GPIO pins on the host MCU (e.g., ESP32) that connect to the
 * TMC9660's control signals. They are required by the TMC9660 bootloader
 * initialization sequence for hardware reset and fault monitoring.
 *
 * @note The BaseGpio instances must be pre-configured (pin number, direction)
 *       before being passed to the handler.
 */
struct Tmc9660ControlPins {
    BaseGpio& rst;      ///< RST pin (output) - hardware reset
    BaseGpio& drv_en;   ///< DRV_EN pin (output) - driver enable
    BaseGpio& faultn;   ///< FAULTN pin (input) - fault status
    BaseGpio& wake;     ///< WAKE pin (output) - wake from hibernation
};

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
        if (!initialized_.load(std::memory_order_acquire)) {
            RtosUniqueLock<RtosMutex> lock(deviceMutex_);
            if (!initialized_.load(std::memory_order_relaxed)) {
                initialized_.store(Initialize(), std::memory_order_release);
            }
        }
        return initialized_.load(std::memory_order_acquire);
    }

    inline bool IsInitialized() const noexcept { return initialized_.load(std::memory_order_acquire); }

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
     * @brief Visit the underlying typed TMC9660 driver by device index.
     *
     * Since the TMC9660 driver is now template-based, direct pointer access is
     * replaced by a visitor pattern. The provided function receives a reference
     * to the typed driver (either TMC9660<HalSpiTmc9660Comm> or TMC9660<HalUartTmc9660Comm>).
     *
     * @tparam Func Callable type accepting auto& (the typed driver reference)
     * @param func The visitor function to execute on the driver
     * @param deviceIndex Device index (0=onboard, 2-3=external)
     * @return The return value of func, or default-constructed if driver not available
     *
     * @code
     * motorController.visitDriver([](auto& driver) {
     *     driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC, 7);
     * });
     * @endcode
     */
    template <typename Func>
    auto visitDriver(Func&& func, uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept {
        auto* h = handler(deviceIndex);
        if (h) {
            return h->visitDriver(std::forward<Func>(func));
        }
        // Driver not available - return default
        using ReturnType = decltype(h->visitDriver(std::forward<Func>(func)));
        if constexpr (std::is_void_v<ReturnType>) {
            return;
        } else {
            return ReturnType{};
        }
    }

    //**************************************************************************//
    //**                  DEVICES MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Create the onboard TMC9660 device using an SPI interface.
     * @param spiInterface Reference to BaseSpi implementation
     * @param address TMC9660 device address
     * @param pins Host-side GPIO control pin references for the TMC9660
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @return true if device created successfully, false if already exists
     * @note Must be called before Initialize() so the onboard device is registered.
     *       The GPIO pins must already be configured (pin number, direction).
     */
    bool CreateOnboardDevice(BaseSpi& spiInterface, 
                            uint8_t address,
                            const Tmc9660ControlPins& pins,
                            const tmc9660::BootloaderConfig* bootCfg = nullptr);

    /**
     * @brief Create the onboard TMC9660 device using a UART interface.
     * @param uartInterface Reference to BaseUart implementation
     * @param address TMC9660 device address
     * @param pins Host-side GPIO control pin references for the TMC9660
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @return true if device created successfully, false if already exists
     * @note Must be called before Initialize() so the onboard device is registered.
     *       The GPIO pins must already be configured (pin number, direction).
     */
    bool CreateOnboardDevice(BaseUart& uartInterface,
                            uint8_t address,
                            const Tmc9660ControlPins& pins,
                            const tmc9660::BootloaderConfig* bootCfg = nullptr);

    /**
     * @brief Create an external TMC9660 device on specified CS line (SPI).
     * @param csDeviceIndex External device CS index (2 or 3 only)
     * @param spiDeviceId SPI device ID for communication (EXTERNAL_DEVICE_1 or EXTERNAL_DEVICE_2)
     * @param address TMC9660 device address
     * @param pins Host-side GPIO control pin references for the TMC9660
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @return true if device created successfully, false otherwise
     * @note Only EXTERNAL_DEVICE_1_INDEX (2) and EXTERNAL_DEVICE_2_INDEX (3) are allowed
     */
    bool CreateExternalDevice(uint8_t csDeviceIndex, 
                            SpiDeviceId spiDeviceId, 
                            uint8_t address,
                            const Tmc9660ControlPins& pins,
                            const tmc9660::BootloaderConfig* bootCfg = nullptr);

    /**
     * @brief Create an external TMC9660 device on a UART interface.
     * @param csDeviceIndex External device slot index (2 or 3 only)
     * @param uartInterface Reference to BaseUart implementation
     * @param address TMC9660 device address
     * @param pins Host-side GPIO control pin references for the TMC9660
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @return true if device created successfully, false otherwise
     * @note Only EXTERNAL_DEVICE_1_INDEX (2) and EXTERNAL_DEVICE_2_INDEX (3) are allowed
     */
    bool CreateExternalDevice(uint8_t csDeviceIndex,
                            BaseUart& uartInterface,
                            uint8_t address,
                            const Tmc9660ControlPins& pins,
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
     * @brief Initialize all registered motor controller devices.
     *
     * Calls Tmc9660Handler::Initialize() on every active device. Devices must
     * be created first via CreateOnboardDevice() / CreateExternalDevice() which
     * supply the required host-side GPIO control pins.
     *
     * @return true if all active devices initialized successfully, false otherwise
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
    std::atomic<bool> initialized_;      ///< Track if system has been initialized
    mutable RtosMutex deviceMutex_;     ///< RTOS mutex for thread-safe device access
};

#endif // COMPONENT_HANDLER_MOTOR_CONTROLLER_H_ 