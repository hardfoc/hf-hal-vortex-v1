/**
 * @file CommChannelsManager.h
 * @brief Communication channels manager for the HardFOC Vortex V1 platform.
 *
 * @details Manages all board communication buses (SPI, I2C, UART, CAN) for the
 *          Vortex motor-control board. The Vortex board uses:
 *            - SPI  : TMC9660 motor controller, AS5047U position encoder, 2 ext slots
 *            - I2C  : BNO08x IMU, PCAL95555 GPIO expander (+ runtime devices)
 *            - UART : TMC9660 TMCL protocol communication
 *            - CAN  : TWAI 500 kbps network bus
 *
 *          Follows hf HAL's ownership pattern: fixed-size arrays for known
 *          compile-time device counts, unique_ptr for bus objects, single
 *          RtosMutex for thread safety.
 *
 *          Extends with runtime I2C device creation for ImuManager
 *          (external BNO08x devices).
 *
 * @version 2.0
 */

#ifndef VORTEX_COMM_CHANNELS_MANAGER_H_
#define VORTEX_COMM_CHANNELS_MANAGER_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>

// Forward declarations — defined in .cpp where types are complete
class EspSpiBus;
class EspSpiDevice;
class EspI2cBus;
class EspI2cDevice;
class EspUart;
class EspCan;

#include "base/BaseSpi.h"
#include "base/BaseI2c.h"
#include "base/BaseUart.h"
#include "base/BaseCan.h"
#include "RtosMutex.h"

//==============================================================================
// COMM CHANNELS ERROR CODES
//==============================================================================

/**
 * @brief Communication channels manager error codes for consistent error reporting.
 *
 * Manager-layer errors for communication bus lifecycle and device operations.
 * Covers SPI, I2C, UART, and CAN bus management failures.
 */
enum class CommError : uint8_t {
    SUCCESS = 0,
    NOT_INITIALIZED,
    INITIALIZATION_FAILED,
    BUS_NOT_AVAILABLE,
    DEVICE_NOT_FOUND,
    INVALID_BUS_INDEX,
    INVALID_DEVICE_INDEX,
    DEVICE_ALREADY_EXISTS,
    DEVICE_CREATION_FAILED,
    COMMUNICATION_FAILED,
    INVALID_PARAMETER,
    MUTEX_LOCK_FAILED
};

/** @brief Convert CommError to string for debugging. */
constexpr const char* CommErrorToString(CommError error) noexcept {
    switch (error) {
        case CommError::SUCCESS:                return "Success";
        case CommError::NOT_INITIALIZED:        return "Not initialized";
        case CommError::INITIALIZATION_FAILED:  return "Initialization failed";
        case CommError::BUS_NOT_AVAILABLE:      return "Bus not available";
        case CommError::DEVICE_NOT_FOUND:       return "Device not found";
        case CommError::INVALID_BUS_INDEX:      return "Invalid bus index";
        case CommError::INVALID_DEVICE_INDEX:   return "Invalid device index";
        case CommError::DEVICE_ALREADY_EXISTS:  return "Device already exists";
        case CommError::DEVICE_CREATION_FAILED: return "Device creation failed";
        case CommError::COMMUNICATION_FAILED:   return "Communication failed";
        case CommError::INVALID_PARAMETER:      return "Invalid parameter";
        case CommError::MUTEX_LOCK_FAILED:      return "Mutex lock failed";
        default:                                return "Unknown error";
    }
}

//==============================================================================
// SPI DEVICE IDENTIFIERS
//==============================================================================

/**
 * @brief SPI device identifiers for the Vortex V1 board.
 * @details The order matches the CS pin configuration in the CommChannelsManager.
 */
enum class SpiDeviceId : uint8_t {
    TMC9660_MOTOR_CONTROLLER = 0, ///< TMC9660 motor controller (SPI Mode 3)
    AS5047U_POSITION_ENCODER = 1, ///< AS5047U position encoder (SPI Mode 1)
    EXTERNAL_DEVICE_1 = 2,        ///< External device slot 1 (SPI Mode 0)
    EXTERNAL_DEVICE_2 = 3,        ///< External device slot 2 (SPI Mode 0)
    DEVICE_COUNT                   ///< Sentinel — total number of SPI devices
};

//==============================================================================
// I2C DEVICE IDENTIFIERS
//==============================================================================

/**
 * @brief I2C device identifiers for the Vortex V1 board (built-in devices).
 */
enum class I2cDeviceId : uint8_t {
    BNO08X_IMU = 0,             ///< BNO08x IMU sensor (address 0x4A)
    PCAL9555_GPIO_EXPANDER = 1, ///< PCAL95555 GPIO expander (address 0x20)
    DEVICE_COUNT                 ///< Sentinel — total number of built-in I2C devices
};

//==============================================================================
// COMM CHANNELS MANAGER
//==============================================================================

/**
 * @class CommChannelsManager
 * @brief Singleton managing all board communication channels for Vortex V1.
 *
 * Design decisions :
 *  - Fixed compile-time device counts (no heap vectors for device tracking).
 *  - SPI bus + four devices known at link time.
 *  - I2C bus with two built-in devices + bounded runtime slots for ImuManager.
 *  - UART bus for TMC9660 TMCL protocol.
 *  - CAN bus for network communication.
 */
class CommChannelsManager {
public:
    static CommChannelsManager& GetInstance() noexcept;

    // Non-copyable, non-movable
    CommChannelsManager(const CommChannelsManager&) = delete;
    CommChannelsManager& operator=(const CommChannelsManager&) = delete;
    CommChannelsManager(CommChannelsManager&&) = delete;
    CommChannelsManager& operator=(CommChannelsManager&&) = delete;

    //==========================================================================
    // INITIALIZATION
    //==========================================================================

    bool EnsureInitialized() noexcept;
    [[nodiscard]] bool IsInitialized() const noexcept { return initialized_.load(std::memory_order_acquire); }

    /**
     * @brief Release all bus resources and reset to uninitialized state.
     * @return true on success
     */
    bool Deinitialize() noexcept;

    //==========================================================================
    // SPI ACCESSORS
    //==========================================================================

    /** @brief Get SPI device by enum id. Returns nullptr on bad id or uninit. */
    BaseSpi* GetSpiDevice(SpiDeviceId device_id) noexcept;

    /** @brief Get SPI device by raw index (0-based). */
    BaseSpi* GetSpiDevice(uint8_t device_index) noexcept;

    //==========================================================================
    // I2C ACCESSORS
    //==========================================================================

    /** @brief Get built-in I2C device by enum id. Returns nullptr if not initialized. */
    BaseI2c* GetI2cDevice(I2cDeviceId device_id) noexcept;

    /**
     * @brief Get I2C device by bus index and device index.
     * @param bus_index Bus index (only 0 supported)
     * @param device_index Device index returned by CreateI2cDevice or bus init
     * @return Pointer to BaseI2c device, or nullptr if invalid
     */
    BaseI2c* GetI2cDevice(uint8_t bus_index, int device_index) noexcept;

    //==========================================================================
    // RUNTIME I2C DEVICE MANAGEMENT (for ImuManager)
    //==========================================================================

    /**
     * @brief Create an I2C device on the primary bus at runtime.
     * @param device_address 7-bit I2C address
     * @param speed_hz I2C clock speed (default 400 kHz)
     * @return Device index on success, -1 on failure
     */
    int CreateI2cDevice(uint8_t device_address, uint32_t speed_hz = 400000) noexcept;

    /**
     * @brief Check if an I2C device exists at the specified address.
     * @param bus_index Bus index (only 0 supported)
     * @param device_address 7-bit I2C address
     * @return true if device exists
     */
    bool HasI2cDeviceAtAddress(uint8_t bus_index, uint8_t device_address) const noexcept;

    /**
     * @brief Remove a runtime-created I2C device.
     * @param device_index Device index returned by CreateI2cDevice
     * @return true if removed successfully
     */
    bool RemoveI2cDevice(int device_index) noexcept;

    //==========================================================================
    // UART / CAN ACCESSORS
    //==========================================================================

    /** @brief Get the UART bus (TMC9660 TMCL). Returns nullptr if not initialized. */
    BaseUart* GetUartBus() noexcept;

    /** @brief Get the TWAI/CAN bus. Returns nullptr if not initialized. */
    BaseCan* GetCanBus() noexcept;

    //==========================================================================
    // DIAGNOSTICS
    //==========================================================================

    void DumpStatistics() const noexcept;

private:
    CommChannelsManager() noexcept;
    ~CommChannelsManager();

    bool Initialize() noexcept;

    //==========================================================================
    // HARDWARE RESOURCES — heap-allocated via unique_ptr (singleton lifetime)
    //==========================================================================

    // SPI bus + device indices
    static constexpr uint8_t kSpiDeviceCount =
        static_cast<uint8_t>(SpiDeviceId::DEVICE_COUNT);

    std::unique_ptr<EspSpiBus> spi_bus_;
    std::array<int, kSpiDeviceCount> spi_device_indices_{};
    bool spi_bus_valid_{false};

    // I2C bus — built-in devices + bounded runtime slots
    static constexpr uint8_t kI2cBuiltinCount =
        static_cast<uint8_t>(I2cDeviceId::DEVICE_COUNT);
    static constexpr uint8_t kMaxI2cRuntimeDevices = 4;

    std::unique_ptr<EspI2cBus> i2c_bus_;
    std::array<int, kI2cBuiltinCount> i2c_builtin_indices_{};
    bool i2c_bus_valid_{false};

    /// Slot for tracking runtime-created I2C devices (external IMUs, etc.)
    struct RuntimeI2cSlot {
        int device_index{-1};
        uint8_t address{0xFF};
        bool active{false};
    };
    std::array<RuntimeI2cSlot, kMaxI2cRuntimeDevices> i2c_runtime_slots_{};

    // UART bus (TMC9660 TMCL)
    std::unique_ptr<EspUart> uart_bus_;
    bool uart_bus_valid_{false};

    // CAN bus
    std::unique_ptr<EspCan> can_bus_;
    bool can_bus_valid_{false};

    std::atomic<bool> initialized_{false};
    mutable RtosMutex mutex_;
};

#endif // VORTEX_COMM_CHANNELS_MANAGER_H_
