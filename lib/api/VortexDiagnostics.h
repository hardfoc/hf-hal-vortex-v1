/**
 * @file VortexDiagnostics.h
 * @brief One error catalog and one aggregated health snapshot for the whole Vortex HAL.
 *
 * @details Each manager keeps its own error enum (CommError, MotorError, hf_gpio_err_t, ...). The facade maps
 *          them onto ManagerError, the shared catalog, so a caller can judge the board without knowing nine
 *          enums; the raw code travels alongside for the detail. Vortex::CollectDiagnostics() fills
 *          VortexBoardDiagnostics from counters the managers already hold: it makes no bus transfer and does not
 *          allocate, so it is safe from a periodic health task.
 *
 *          Ownership: value types. Thread safety: filled under the facade's lifecycle lock. MCU-free.
 */

#ifndef VORTEX_DIAGNOSTICS_H_
#define VORTEX_DIAGNOSTICS_H_

#include <array>
#include <cstddef>
#include <cstdint>

/** @brief Number of managers the facade owns (NVS, comms, GPIO, LEDs, motors, ADC, IMU, encoders, temperature). */
inline constexpr size_t kVortexManagerCount = 9;

/** @brief The shared error catalog (manager lane). */
enum class ManagerError : uint8_t {
    None = 0,             ///< No error
    NotInitialized,       ///< The manager is not up (never initialised, shut down, or failed)
    DependencyNotReady,   ///< A manager or bus it needs is not up
    BusUnavailable,       ///< No usable bus (SPI / I2C / UART / CAN) or the binding refused it
    DeviceNotFound,       ///< No device at the slot, address or name
    InitFailed,           ///< Bring-up of a device failed (bootloader, sensor reset, ...)
    CommunicationFailed,  ///< A transfer failed (NACK, CRC, timeout)
    InvalidParameter,     ///< Out-of-range index or argument
    HardwareFault,        ///< The device reported a fault, or a read returned an error code
    Unknown,              ///< A code the facade does not map
};

/** @brief Short text for a ManagerError. */
[[nodiscard]] constexpr const char* ManagerErrorToString(ManagerError e) noexcept {
    switch (e) {
        case ManagerError::None:                return "None";
        case ManagerError::NotInitialized:      return "Not initialized";
        case ManagerError::DependencyNotReady:  return "Dependency not ready";
        case ManagerError::BusUnavailable:      return "Bus unavailable";
        case ManagerError::DeviceNotFound:      return "Device not found";
        case ManagerError::InitFailed:          return "Init failed";
        case ManagerError::CommunicationFailed: return "Communication failed";
        case ManagerError::InvalidParameter:    return "Invalid parameter";
        case ManagerError::HardwareFault:       return "Hardware fault";
        default:                                return "Unknown";
    }
}

/** @brief One manager's state in the aggregated snapshot. */
struct ManagerDiagnostics {
    const char* name{nullptr};            ///< Manager name (string literal)
    bool required{false};                 ///< true if EnsureInitialized() fails without it
    bool initialized{false};              ///< The manager reports IsInitialized()
    ManagerError error{ManagerError::None};
    uint8_t raw_error{0};                 ///< The manager's own last-error code
};

/** @brief Aggregated board health (Vortex::CollectDiagnostics()). Index order of `managers` = init order. */
struct VortexBoardDiagnostics {
    bool initialized{false};                          ///< Vortex::IsInitialized()
    ManagerError last_error{ManagerError::None};      ///< Why the last EnsureInitialized() failed, if it did
    uint8_t managers_up{0};                           ///< Managers that report initialised
    std::array<ManagerDiagnostics, kVortexManagerCount> managers{};

    // Buses (CommChannelsManager)
    bool spi_ok{false};
    bool i2c_ok{false};
    bool uart_ok{false};
    bool can_ok{false};

    // Devices
    uint16_t gpio_pins{0};               ///< Pins registered in GpioManager
    uint8_t motor_devices_active{0};     ///< TMC9660 slots with a handler
    uint8_t motor_devices_ready{0};      ///< ... whose bring-up succeeded
    uint8_t imu_devices_ready{0};
    uint8_t encoder_devices_ready{0};
    uint8_t adc_channels{0};
    uint8_t temperature_sensors{0};

    // Boot report
    uint8_t failed_components{0};
    uint8_t warnings{0};
    uint32_t init_time_ms{0};
};

#endif  // VORTEX_DIAGNOSTICS_H_
