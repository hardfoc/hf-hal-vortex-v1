/**
 * @file EncoderManager.h
 * @brief Encoder device management for the HardFOC Vortex V1 platform.
 *
 * @details Manages up to four AS5047U position encoder devices.  The Vortex V1
 *          board has no onboard encoder — all devices are user-provisioned via
 *          CreateAs5047uDevice().  The board provides a dedicated SPI CS pin
 *          (SpiDeviceId::AS5047U_POSITION_ENCODER, GPIO 20) plus two external
 *          CS slots that can each host an AS5047U or other SPI sensor.
 *
 * @version 2.1
 */

#pragma once

#include <memory>
#include <functional>
#include <array>
#include "RtosMutex.h"
#include "handlers/as5047u/As5047uHandler.h"

// Forward declarations
class CommChannelsManager;
class GpioManager;
class BaseSpi;
enum class SpiDeviceId : uint8_t;
enum class AS5047U_Error : uint16_t;

/**
 * @brief Generic encoder manager error codes.
 *
 * These are manager-layer errors and are intentionally encoder-family agnostic.
 * Handler/driver internals keep native driver error flags and are translated
 * into this common manager error domain.
 */
enum class EncoderError : uint8_t {
    SUCCESS = 0,                ///< Operation completed successfully
    NOT_INITIALIZED,            ///< Manager not yet initialised
    INITIALIZATION_FAILED,      ///< Device initialisation failed
    INVALID_PARAMETER,          ///< Null pointer or out-of-range argument
    SPI_COMMUNICATION_FAILED,   ///< SPI bus transfer failed
    CRC_ERROR,                  ///< CRC mismatch on SPI frame
    FRAMING_ERROR,              ///< Malformed SPI frame received
    SENSOR_ERROR,               ///< Encoder sensor self-test failure
    TIMEOUT,                    ///< Operation timed out
    MUTEX_LOCK_FAILED           ///< RTOS mutex acquire timed out
};

/** @brief Backward-compatible alias for EncoderError. */
using As5047uError = EncoderError;

/**
 * @brief Convert EncoderError to a human-readable string.
 * @param error Error code to convert.
 * @return Null-terminated string representation.
 */
constexpr const char* EncoderErrorToString(EncoderError error) noexcept {
    switch (error) {
        case EncoderError::SUCCESS: return "Success";
        case EncoderError::NOT_INITIALIZED: return "Not initialized";
        case EncoderError::INITIALIZATION_FAILED: return "Initialization failed";
        case EncoderError::INVALID_PARAMETER: return "Invalid parameter";
        case EncoderError::SPI_COMMUNICATION_FAILED: return "SPI communication failed";
        case EncoderError::CRC_ERROR: return "CRC error";
        case EncoderError::FRAMING_ERROR: return "Framing error";
        case EncoderError::SENSOR_ERROR: return "Sensor error";
        case EncoderError::TIMEOUT: return "Timeout";
        case EncoderError::MUTEX_LOCK_FAILED: return "Mutex lock failed";
        default: return "Unknown error";
    }
}

/**
 * @brief Backward-compatible helper — calls EncoderErrorToString().
 * @param error Error code to convert.
 * @return Null-terminated string representation.
 */
constexpr const char* As5047uErrorToString(As5047uError error) noexcept {
    return EncoderErrorToString(error);
}

//==============================================================================
// ENCODER DIAGNOSTICS
//==============================================================================

/**
 * @brief System-level diagnostics snapshot for encoder health monitoring.
 */
struct EncoderSystemDiagnostics {
    bool system_healthy;               ///< True when all active devices are healthy
    bool system_initialized;           ///< EncoderManager::IsInitialized()
    uint8_t active_device_count;       ///< Number of active device slots
    uint8_t initialized_device_count;  ///< Number of successfully initialised devices
    EncoderError last_error;           ///< Most recent error code

    /** @brief Per-device health snapshot. */
    struct DeviceSnapshot {
        bool active;                   ///< Slot has a device
        bool initialized;              ///< Device initialised successfully
        uint32_t measurement_count;    ///< Cumulative successful readings
        uint32_t communication_error_count; ///< Cumulative SPI errors
    };
    static constexpr uint8_t kMaxDevices = 4;
    DeviceSnapshot devices[kMaxDevices]{}; ///< Per-slot snapshots (0–kMaxDevices-1)
};

/**
 * @class EncoderManager
 * @brief Singleton for managing multiple AS5047U encoder devices with indexed access.
 *
 * The Vortex V1 board has **no onboard encoder**.  All AS5047U devices are
 * user-provisioned at runtime via CreateAs5047uDevice().  The board exposes a
 * dedicated SPI CS line for the encoder (SpiDeviceId::AS5047U_POSITION_ENCODER,
 * GPIO 20) plus two external CS slots — any of them can host an AS5047U.
 *
 * **Key Features:**
 * - Up to 4 AS5047U devices (indices 0–3), all user-created
 * - Dynamic device creation/deletion on any slot
 * - Thread-safe singleton pattern with RtosMutex
 * - Exception-free operation with pointer-based returns
 * - ESP-IDF v5.5+ SPI integration
 *
 * **Usage Example:**
 * @code
 * auto& enc = EncoderManager::GetInstance();
 * enc.EnsureInitialized();
 *
 * // Create an AS5047U on the dedicated CS pin (slot 0)
 * enc.CreateAs5047uDevice(0, SpiDeviceId::AS5047U_POSITION_ENCODER);
 *
 * uint16_t angle;
 * if (enc.ReadAngle(0, angle) == EncoderError::SUCCESS) {
 *     printf("Angle: %u LSB (%.2f deg)\n",
 *            angle, static_cast<double>(angle) * (360.0 / 16384.0));
 * }
 * @endcode
 */
class EncoderManager {
public:
    static constexpr uint8_t MAX_ENCODER_DEVICES = 4;           ///< Maximum supported AS5047U devices

    // Slot indices — all user-provisioned, no onboard encoder.
    static constexpr uint8_t ENCODER_SLOT_0 = 0;
    static constexpr uint8_t ENCODER_SLOT_1 = 1;
    static constexpr uint8_t ENCODER_SLOT_2 = 2;
    static constexpr uint8_t ENCODER_SLOT_3 = 3;

    /// @deprecated Use ENCODER_SLOT_0. Kept for backward compatibility.
    static constexpr uint8_t ONBOARD_ENCODER_INDEX = ENCODER_SLOT_0;

public:
    /**
     * @brief Get the singleton instance of EncoderManager.
     * @return Reference to the singleton EncoderManager.
     */
    static EncoderManager& GetInstance() noexcept;

    //**************************************************************************//
    //**                  DEVICE MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Ensure the encoder manager system is initialized.
     * @note No devices are created automatically — call CreateAs5047uDevice() after init.
     * @return true if initialization successful, false otherwise
     */
    bool EnsureInitialized() noexcept;

    /**
     * @brief Check if the encoder manager is initialized.
     * @return true if initialized, false otherwise
     */
    bool IsInitialized() const noexcept;

    /**
     * @brief Get the most recent error code.
     * @return Last EncoderError set by any API call
     */
    [[nodiscard]] EncoderError GetLastError() const noexcept { return last_error_.load(std::memory_order_acquire); }

    /**
     * @brief Fill a diagnostics snapshot with current system state.
     * @param diagnostics Output structure to populate
     * @return EncoderError::SUCCESS on success, EncoderError::NOT_INITIALIZED if not init
     */
    [[nodiscard]] EncoderError GetSystemDiagnostics(EncoderSystemDiagnostics& diagnostics) const noexcept;

    /**
     * @brief Deinitialize all encoders and release resources.
     * @return true if deinitialized successfully, false otherwise
     */
    bool Deinitialize() noexcept;

    //**************************************************************************//
    //**                  HANDLER AND DRIVER MANAGEMENT                       **//
    //**************************************************************************//

    /**
     * @brief Get access to AS5047U encoder handler by device index.
     * @param deviceIndex Device slot (0–3)
     * @return Pointer to As5047uHandler if valid and active, nullptr otherwise
     */
    As5047uHandler* GetAs5047uHandler(uint8_t deviceIndex = ENCODER_SLOT_0) noexcept;

    /**
     * @brief Get access to the underlying AS5047U driver by device index.
     * @param deviceIndex Device slot (0–3)
     * @return Pointer to AS5047U driver, nullptr if invalid/not ready
     * @note Caller must not delete the returned pointer; lifetime is owned by the handler.
     */
    as5047u::AS5047U<As5047uSpiAdapter>* GetAs5047uDriver(uint8_t deviceIndex = ENCODER_SLOT_0) noexcept;

    //**************************************************************************//
    //**                  DEVICES MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Create an AS5047U encoder device on a given SPI device slot.
     * @param deviceIndex Device slot (0–3)
     * @param spiDeviceId SPI device ID for communication (e.g. AS5047U_POSITION_ENCODER)
     * @param config Optional AS5047U configuration (defaults to default config)
     * @return true if device created successfully, false otherwise
     */
    bool CreateAs5047uDevice(uint8_t deviceIndex,
                             SpiDeviceId spiDeviceId,
                             const As5047uConfig& config = As5047uHandler::GetDefaultConfig()) noexcept;

    /**
     * @brief Create an AS5047U encoder device using a direct BaseSpi interface.
     * @param deviceIndex Device slot (0–3)
     * @param spi_interface Direct reference to BaseSpi interface
     * @param config Optional AS5047U configuration (defaults to default config)
     * @return true if device created successfully, false otherwise
     */
    bool CreateAs5047uDevice(uint8_t deviceIndex,
                             BaseSpi& spi_interface,
                             const As5047uConfig& config = As5047uHandler::GetDefaultConfig()) noexcept;

    /// @deprecated Use CreateAs5047uDevice(). Kept for backward compatibility.
    bool CreateExternalAs5047uDevice(uint8_t deviceIndex,
                                     SpiDeviceId spiDeviceId,
                                     const As5047uConfig& config = As5047uHandler::GetDefaultConfig()) noexcept {
        return CreateAs5047uDevice(deviceIndex, spiDeviceId, config);
    }
    /// @deprecated Use CreateAs5047uDevice(). Kept for backward compatibility.
    bool CreateExternalAs5047uDevice(uint8_t deviceIndex,
                                     BaseSpi& spi_interface,
                                     const As5047uConfig& config = As5047uHandler::GetDefaultConfig()) noexcept {
        return CreateAs5047uDevice(deviceIndex, spi_interface, config);
    }

    /**
     * @brief Delete an encoder device and free its slot.
     * @param deviceIndex Device slot (0–3)
     * @return true if device deleted successfully, false otherwise
     */
    bool DeleteDevice(uint8_t deviceIndex) noexcept;

    /// @deprecated Use DeleteDevice(). Kept for backward compatibility.
    bool DeleteExternalDevice(uint8_t deviceIndex) noexcept { return DeleteDevice(deviceIndex); }

    /**
     * @brief Get the number of active encoder devices.
     * @return Number of active devices (0 to MAX_ENCODER_DEVICES)
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
     * @brief Get indices of all active encoder devices.
     * @param out    Fixed-size output array
     * @param count  Output: number of entries written
     */
    void GetActiveDeviceIndices(std::array<uint8_t, MAX_ENCODER_DEVICES>& out, size_t& count) const noexcept;

    /**
     * @brief Initialize all active encoder devices.
     * @param results  Output: per-slot initialization result
     * @return Number of devices successfully initialized
     */
    size_t InitializeAllDevices(std::array<bool, MAX_ENCODER_DEVICES>& results) noexcept;

    /**
     * @brief Get initialization status of all device slots.
     * @param status  Output: per-slot status
     */
    void GetInitializationStatus(std::array<bool, MAX_ENCODER_DEVICES>& status) const noexcept;

    //**************************************************************************//
    //**                  DEVICE INFORMATION METHODS                          **//
    //**************************************************************************//

    /**
     * @brief Get human-readable names of all available devices.
     * @param out    Fixed-size output array of const char* string literals
     * @param count  Output: number of entries written
     */
    void GetAvailableDevices(std::array<const char*, MAX_ENCODER_DEVICES>& out, size_t& count) const noexcept;

    /**
     * @brief Get device type by index.
     * @param deviceIndex Device index
     * @return Device type string literal or nullptr if invalid
     */
    [[nodiscard]] const char* GetDeviceType(uint8_t deviceIndex) const noexcept;

    //**************************************************************************//
    //**                  HIGH-LEVEL ENCODER OPERATIONS                       **//
    //**************************************************************************//

    /**
     * @brief Read angle from specific encoder device.
     * @param deviceIndex Device slot (0–3)
     * @param angle Output angle value (0-16383 LSB, 14-bit)
    * @return EncoderError::SUCCESS if successful
     */
    EncoderError ReadAngle(uint8_t deviceIndex, uint16_t& angle) noexcept;

    /**
     * @brief Read angle in degrees from specific encoder device.
     * @param deviceIndex Device slot (0–3)
     * @param angle_degrees Output angle in degrees (0.0-359.978°)
    * @return EncoderError::SUCCESS if successful
     */
    EncoderError ReadAngleDegrees(uint8_t deviceIndex, double& angle_degrees) noexcept;

    /**
     * @brief Read velocity from specific encoder device.
     * @param deviceIndex Device slot (0–3)
     * @param velocity_rpm Output velocity in RPM
    * @return EncoderError::SUCCESS if successful
     */
    EncoderError ReadVelocityRPM(uint8_t deviceIndex, double& velocity_rpm) noexcept;

    /**
     * @brief Read diagnostics from specific encoder device.
     * @param deviceIndex Device slot (0–3)
     * @param diagnostics Output diagnostic information
    * @return EncoderError::SUCCESS if successful
     */
    EncoderError ReadDiagnostics(uint8_t deviceIndex, As5047uDiagnostics& diagnostics) noexcept;

    /**
     * @brief Set zero position for specific encoder device.
     * @param deviceIndex Device slot (0–3)
     * @param zero_position Zero position in LSB (0-16383)
    * @return EncoderError::SUCCESS if successful
     */
    EncoderError SetZeroPosition(uint8_t deviceIndex, uint16_t zero_position) noexcept;

    /**
     * @brief Read angles from all active encoder devices.
     * @param angles          Output: per-slot angle readings (only valid where errors[i] == SUCCESS)
     * @param device_indices  Output: indices of active devices
     * @param errors          Output: per-slot error codes
     * @param count           Output: number of entries written
     */
    void ReadAllAngles(std::array<uint16_t, MAX_ENCODER_DEVICES>& angles,
                       std::array<uint8_t, MAX_ENCODER_DEVICES>& device_indices,
                       std::array<EncoderError, MAX_ENCODER_DEVICES>& errors,
                       size_t& count) noexcept;

    /**
     * @brief Read velocities from all active encoder devices.
     * @param velocities_rpm  Output: per-slot velocity readings
     * @param device_indices  Output: indices of active devices
     * @param errors          Output: per-slot error codes
     * @param count           Output: number of entries written
     */
    void ReadAllVelocities(std::array<double, MAX_ENCODER_DEVICES>& velocities_rpm,
                           std::array<uint8_t, MAX_ENCODER_DEVICES>& device_indices,
                           std::array<EncoderError, MAX_ENCODER_DEVICES>& errors,
                           size_t& count) noexcept;

    /**
     * @brief Check health status of all active encoder devices.
     * @return true if all active devices are healthy, false if any have errors
     */
    bool CheckAllDevicesHealth() noexcept;

    //**************************************************************************//
    //**                  SENSOR MONITORING METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Get measurement statistics for monitoring specific device.
     * @param deviceIndex Device slot (0–3)
     * @return Number of measurements taken since initialization
     */
    uint32_t GetMeasurementCount(uint8_t deviceIndex) const noexcept;

    /**
     * @brief Get communication error count for specific device.
     * @param deviceIndex Device slot (0–3)
     * @return Number of communication errors since initialization
     */
    uint32_t GetCommunicationErrorCount(uint8_t deviceIndex) const noexcept;
    
    /**
     * @brief Dump comprehensive system statistics to log as INFO level.
     * Logs all device statistics, interrupt counts, and system health information.
     */
    void DumpStatistics() const noexcept;

    // Delete copy/move constructors and assignment operators
    EncoderManager(const EncoderManager&) = delete;
    EncoderManager& operator=(const EncoderManager&) = delete;
    EncoderManager(EncoderManager&&) = delete;
    EncoderManager& operator=(EncoderManager&&) = delete;

private:
    /**
     * @brief Private constructor for singleton pattern.
     */
    EncoderManager() noexcept;

    /**
     * @brief Private destructor.
     */
    ~EncoderManager() noexcept;

    /**
     * @brief Initialize the encoder manager system (no devices auto-created).
     * @return true if initialization successful, false otherwise
     */
    bool Initialize() noexcept;

    /**
     * @brief Validate that deviceIndex is within [0, MAX_ENCODER_DEVICES).
     */
    static bool IsValidDeviceIndex(uint8_t deviceIndex) noexcept {
        return deviceIndex < MAX_ENCODER_DEVICES;
    }

    /**
    * @brief Map AS5047U driver sticky error flags to generic EncoderError.
     * @param sticky_flags Sticky error flags from AS5047U driver
    * @return Corresponding EncoderError code
     */
    static EncoderError mapDriverError(AS5047U_Error sticky_flags) noexcept;

    // ===============================
    // SYSTEM STATE
    // ===============================

    /**
     * @brief System initialization state (atomic for thread safety).
     */
    std::atomic<bool> initialized_{false};

    /**
     * @brief Most recent error code for system-level tracking.
     */
    std::atomic<EncoderError> last_error_{EncoderError::SUCCESS};

    /**
     * @brief Main system mutex for thread-safe operations.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex manager_mutex_;

    // ===============================
    // DEVICE STORAGE
    // ===============================

    /**
     * @brief Array of AS5047U encoder handlers (one per device slot).
     * Uses unique_ptr for exclusive ownership of each handler.
     * Protected by RtosMutex for thread-safe access.
     */
    std::array<std::unique_ptr<As5047uHandler>, MAX_ENCODER_DEVICES> as5047u_handlers_;

    /**
     * @brief Device initialization status tracking.
     * Tracks which devices have been successfully initialized.
     */
    std::array<bool, MAX_ENCODER_DEVICES> device_initialized_;

    /**
     * @brief Device active status tracking.
     * Tracks which device slots have active devices.
     */
    std::array<bool, MAX_ENCODER_DEVICES> device_active_;

    // (no onboard device — all encoder slots are user-provisioned)

    // ===============================
    // DEPENDENCIES
    // ===============================

    /**
     * @brief Reference to communication manager for SPI access.
     * Used for creating encoder devices with proper transport interfaces.
     */
    CommChannelsManager* comm_manager_ = nullptr;

    /**
     * @brief Reference to GPIO manager for interrupt pin access.
     * Used for configuring interrupt pins for encoder devices.
     */
    GpioManager* gpio_manager_ = nullptr;

    // ===============================
    // SENSOR MONITORING
    // ===============================

    /**
     * @brief Measurement and error tracking for all devices.
     */
    std::array<std::atomic<uint32_t>, MAX_ENCODER_DEVICES> measurement_counts_{0};  ///< Measurement counts per device
    std::array<std::atomic<uint32_t>, MAX_ENCODER_DEVICES> communication_error_counts_{0};  ///< Communication error counts per device
};

//==============================================================================
// CONVENIENCE
//==============================================================================

/**
 * @brief Convenience accessor — equivalent to EncoderManager::GetInstance().
 * @return Reference to the singleton EncoderManager.
 */
[[nodiscard]] inline EncoderManager& GetEncoderManager() noexcept {
    return EncoderManager::GetInstance();
}