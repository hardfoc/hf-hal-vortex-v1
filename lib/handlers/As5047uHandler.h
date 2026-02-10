/**
 * @file As5047uHandler.h
 * @brief Unified handler for AS5047U magnetic rotary position sensor with SPI integration.
 *
 * This class provides a modern, unified interface for a single AS5047U device following
 * the same architectural excellence as TMC9660Handler and PCAL95555Handler:
 * - Bridge pattern for BaseSpi integration
 * - Lazy initialization with lightweight construction
 * - Shared pointer management for safe memory handling
 * - Complete exception-free design with noexcept methods
 * - Thread-safe operations with RtosMutex protection
 * - Comprehensive error handling and validation
 * - High-level sensor abstraction with position/velocity/diagnostics
 * - Advanced AS5047U features (DAEC, OTP programming, interface configuration)
 * - Factory method for creating multiple sensor instances
 *
 * Features:
 * - 14-bit absolute angle measurement (0-16383 counts per revolution)
 * - Velocity measurement with multiple unit conversions
 * - Dynamic Angle Error Compensation (DAEC)
 * - ABI/UVW/PWM interface configuration
 * - OTP programming for permanent settings
 * - Comprehensive diagnostics and error handling
 * - Multiple SPI frame formats (16/24/32-bit)
 * - Thread-safe concurrent access
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 * @copyright HardFOC
 */

#ifndef COMPONENT_HANDLER_AS5047U_HANDLER_H_
#define COMPONENT_HANDLER_AS5047U_HANDLER_H_

#include <cstdint>
#include <memory>
#include <string>
#include "utils-and-drivers/hf-core-drivers/external/hf-as5047u-driver/src/AS5047U.hpp"
#include "base/BaseSpi.h"
#include "utils/RtosMutex.h"

//======================================================//
// AS5047U ERROR CODES
//======================================================//

/**
 * @brief AS5047U handler error codes for consistent error reporting
 */
enum class As5047uError : uint8_t {
    SUCCESS = 0,
    NOT_INITIALIZED,
    INITIALIZATION_FAILED,
    INVALID_PARAMETER,
    SPI_COMMUNICATION_FAILED,
    CRC_ERROR,
    FRAMING_ERROR,
    SENSOR_ERROR,
    OTP_PROGRAMMING_FAILED,
    CALIBRATION_FAILED,
    TIMEOUT,
    MUTEX_LOCK_FAILED
};

/**
 * @brief Convert As5047uError to string for debugging
 */
constexpr const char* As5047uErrorToString(As5047uError error) noexcept {
    switch (error) {
        case As5047uError::SUCCESS: return "Success";
        case As5047uError::NOT_INITIALIZED: return "Not initialized";
        case As5047uError::INITIALIZATION_FAILED: return "Initialization failed";
        case As5047uError::INVALID_PARAMETER: return "Invalid parameter";
        case As5047uError::SPI_COMMUNICATION_FAILED: return "SPI communication failed";
        case As5047uError::CRC_ERROR: return "CRC error";
        case As5047uError::FRAMING_ERROR: return "Framing error";
        case As5047uError::SENSOR_ERROR: return "Sensor error";
        case As5047uError::OTP_PROGRAMMING_FAILED: return "OTP programming failed";
        case As5047uError::CALIBRATION_FAILED: return "Calibration failed";
        case As5047uError::TIMEOUT: return "Timeout";
        case As5047uError::MUTEX_LOCK_FAILED: return "Mutex lock failed";
        default: return "Unknown error";
    }
}

//======================================================//
// AS5047U SPI BRIDGE ADAPTER
//======================================================//

/**
 * @brief Bridge adapter connecting BaseSpi interface to AS5047U spiBus interface.
 * 
 * This adapter implements the AS5047U::spiBus virtual interface using a HardFOC
 * BaseSpi implementation, enabling the AS5047U driver to work with any SPI
 * controller that inherits from BaseSpi.
 * 
 * Thread Safety: This adapter is thread-safe when the underlying BaseSpi
 * implementation is thread-safe.
 */
class As5047uSpiAdapter : public AS5047U::spiBus {
public:
    /**
     * @brief Construct SPI adapter with BaseSpi interface
     * @param spi_interface Reference to BaseSpi implementation
     */
    explicit As5047uSpiAdapter(BaseSpi& spi_interface) noexcept;

    /**
     * @brief Perform full-duplex SPI transfer
     * @param tx Transmit buffer (can be nullptr to send zeros)
     * @param rx Receive buffer (can be nullptr to discard received data)
     * @param len Number of bytes to transfer
     */
    void transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) noexcept override;

private:
    BaseSpi& spi_interface_;
};

//======================================================//
// AS5047U SENSOR DATA STRUCTURES
//======================================================//

/**
 * @brief Complete sensor measurement data structure
 */
struct As5047uMeasurement {
    uint16_t angle_raw;              ///< Raw angle (0-16383 LSB)
    uint16_t angle_compensated;      ///< DAEC compensated angle (0-16383 LSB)
    int16_t velocity_raw;            ///< Raw velocity (signed 14-bit)
    double velocity_deg_per_sec;     ///< Velocity in degrees per second
    double velocity_rad_per_sec;     ///< Velocity in radians per second
    double velocity_rpm;             ///< Velocity in revolutions per minute
    uint8_t agc_value;               ///< Automatic Gain Control value (0-255)
    uint16_t magnitude;              ///< Magnetic field magnitude (0-16383)
    uint16_t error_flags;            ///< Current error flags
    bool valid;                      ///< True if measurement is valid
};

/**
 * @brief Sensor diagnostic information
 */
struct As5047uDiagnostics {
    bool magnetic_field_ok;          ///< Magnetic field strength is adequate
    bool agc_warning;                ///< AGC at minimum or maximum
    bool cordic_overflow;            ///< CORDIC algorithm overflow
    bool offset_compensation_ok;     ///< Offset compensation completed
    bool communication_ok;           ///< SPI communication working
    uint16_t last_error_flags;       ///< Last error flags read
    uint32_t communication_errors;   ///< Count of communication errors
    uint32_t total_measurements;     ///< Total measurements taken
};

/**
 * @brief AS5047U configuration structure
 */
struct As5047uConfig {
    FrameFormat frame_format;        ///< SPI frame format (16/24/32-bit)
    uint8_t crc_retries;             ///< Number of CRC error retries
    bool enable_daec;                ///< Enable Dynamic Angle Error Compensation
    bool enable_adaptive_filter;     ///< Enable adaptive filtering
    uint16_t zero_position;          ///< Zero position offset (0-16383)
    bool enable_abi_output;          ///< Enable ABI incremental output
    bool enable_uvw_output;          ///< Enable UVW commutation output
    bool enable_pwm_output;          ///< Enable PWM output
    uint8_t abi_resolution_bits;     ///< ABI resolution in bits (10-14)
    uint8_t uvw_pole_pairs;          ///< UVW pole pairs (1-7)
    bool high_temperature_mode;      ///< Enable 150°C operation mode
};

//======================================================//
// AS5047U HANDLER CLASS
//======================================================//

/**
 * @brief Unified handler for AS5047U magnetic rotary position sensor.
 * 
 * This class provides a comprehensive interface for the AS5047U sensor with:
 * - Lazy initialization pattern for optimal memory usage
 * - Shared pointer management for safe cross-component sharing
 * - Complete exception-free design for embedded reliability
 * - Thread-safe operations with mutex protection
 * - Bridge pattern integration with BaseSpi
 * - High-level sensor abstraction
 * - Advanced AS5047U features and diagnostics
 * 
 * Architecture follows the same excellence as TMC9660Handler and PCAL95555Handler.
 */
class As5047uHandler {
public:
    //======================================================//
    // CONSTRUCTION AND LIFECYCLE
    //======================================================//

    /**
     * @brief Construct AS5047U handler with SPI interface
     * @param spi_interface Reference to BaseSpi implementation
     * @param config Initial sensor configuration
     * 
     * Note: Lightweight constructor following lazy initialization pattern.
     * Objects are created during Initialize() call.
     */
    explicit As5047uHandler(BaseSpi& spi_interface, 
                           const As5047uConfig& config = GetDefaultConfig()) noexcept;

    /**
     * @brief Destructor - automatically handles cleanup
     */
    ~As5047uHandler() noexcept = default;

    // Disable copy construction and assignment
    As5047uHandler(const As5047uHandler&) = delete;
    As5047uHandler& operator=(const As5047uHandler&) = delete;

    // Enable move construction and assignment
    As5047uHandler(As5047uHandler&&) noexcept = default;
    As5047uHandler& operator=(As5047uHandler&&) noexcept = default;

    //======================================================//
    // INITIALIZATION AND STATUS
    //======================================================//

    /**
     * @brief Initialize the AS5047U sensor (lazy initialization)
     * @return As5047uError::SUCCESS if successful, error code otherwise
     * 
     * Creates AS5047U driver instance and performs sensor initialization.
     * Safe to call multiple times - subsequent calls return current status.
     */
    As5047uError Initialize() noexcept;

    /**
     * @brief Deinitialize the sensor and free resources
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError Deinitialize() noexcept;

    /**
     * @brief Check if sensor is initialized and ready
     * @return True if sensor is ready for operations
     */
    bool IsInitialized() const noexcept;

    /**
     * @brief Check if sensor driver is ready (lazy initialization helper)
     * @return True if driver instance exists and is ready
     */
    bool IsSensorReady() const noexcept;

    /**
     * @brief Get shared pointer to AS5047U driver for advanced operations
     * @return Shared pointer to AS5047U driver or nullptr if not initialized
     * 
     * Note: Returns shared_ptr for safe cross-component sharing.
     */
    std::shared_ptr<AS5047U> GetSensor() noexcept;

    //======================================================//
    // SENSOR MEASUREMENTS
    //======================================================//

    /**
     * @brief Read complete sensor measurement data
     * @param measurement Output structure for measurement data
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadMeasurement(As5047uMeasurement& measurement) noexcept;

    /**
     * @brief Read current absolute angle with DAEC compensation
     * @param angle Output angle value (0-16383 LSB, 14-bit)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadAngle(uint16_t& angle) noexcept;

    /**
     * @brief Read raw angle without DAEC compensation
     * @param raw_angle Output raw angle value (0-16383 LSB, 14-bit)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadRawAngle(uint16_t& raw_angle) noexcept;

    /**
     * @brief Read rotational velocity
     * @param velocity_lsb Output velocity in LSB units (signed 14-bit)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadVelocity(int16_t& velocity_lsb) noexcept;

    /**
     * @brief Read velocity in degrees per second
     * @param velocity_deg_per_sec Output velocity in degrees/second
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadVelocityDegPerSec(double& velocity_deg_per_sec) noexcept;

    /**
     * @brief Read velocity in radians per second
     * @param velocity_rad_per_sec Output velocity in radians/second
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadVelocityRadPerSec(double& velocity_rad_per_sec) noexcept;

    /**
     * @brief Read velocity in revolutions per minute
     * @param velocity_rpm Output velocity in RPM
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadVelocityRPM(double& velocity_rpm) noexcept;

    //======================================================//
    // SENSOR DIAGNOSTICS
    //======================================================//

    /**
     * @brief Read comprehensive sensor diagnostics
     * @param diagnostics Output diagnostic information
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadDiagnostics(As5047uDiagnostics& diagnostics) noexcept;

    /**
     * @brief Read Automatic Gain Control value
     * @param agc_value Output AGC value (0-255)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadAGC(uint8_t& agc_value) noexcept;

    /**
     * @brief Read magnetic field magnitude
     * @param magnitude Output magnitude value (0-16383)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadMagnitude(uint16_t& magnitude) noexcept;

    /**
     * @brief Read and clear error flags
     * @param error_flags Output error flags
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ReadErrorFlags(uint16_t& error_flags) noexcept;

    /**
     * @brief Check if magnetic field is within acceptable range
     * @param field_ok Output field status
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError IsMagneticFieldOK(bool& field_ok) noexcept;

    //======================================================//
    // SENSOR CONFIGURATION
    //======================================================//

    /**
     * @brief Set zero position reference
     * @param zero_position Zero position in LSB (0-16383)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError SetZeroPosition(uint16_t zero_position) noexcept;

    /**
     * @brief Get current zero position setting
     * @param zero_position Output zero position value
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError GetZeroPosition(uint16_t& zero_position) noexcept;

    /**
     * @brief Set rotation direction
     * @param clockwise True for clockwise positive rotation
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError SetRotationDirection(bool clockwise) noexcept;

    /**
     * @brief Enable/disable Dynamic Angle Error Compensation
     * @param enable True to enable DAEC
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError SetDAEC(bool enable) noexcept;

    /**
     * @brief Enable/disable adaptive filtering
     * @param enable True to enable adaptive filter
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError SetAdaptiveFilter(bool enable) noexcept;

    /**
     * @brief Configure interface outputs (ABI, UVW, PWM)
     * @param enable_abi Enable ABI incremental output
     * @param enable_uvw Enable UVW commutation output
     * @param enable_pwm Enable PWM output
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ConfigureInterface(bool enable_abi, bool enable_uvw, bool enable_pwm) noexcept;

    /**
     * @brief Set ABI resolution
     * @param resolution_bits Resolution in bits (10-14)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError SetABIResolution(uint8_t resolution_bits) noexcept;

    /**
     * @brief Set UVW pole pairs
     * @param pole_pairs Number of pole pairs (1-7)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError SetUVWPolePairs(uint8_t pole_pairs) noexcept;

    /**
     * @brief Enable/disable 150°C temperature mode
     * @param enable True for 150°C mode (higher noise), false for 125°C (lower noise)
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError SetHighTemperatureMode(bool enable) noexcept;

    //======================================================//
    // ADVANCED FEATURES
    //======================================================//

    /**
     * @brief Program current settings to OTP memory (One-Time Programmable)
     * @return As5047uError::SUCCESS if successful
     * 
     * WARNING: OTP can only be programmed once. Ensure correct configuration
     * and proper supply voltage before calling this method.
     */
    As5047uError ProgramOTP() noexcept;

    /**
     * @brief Perform sensor calibration
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError PerformCalibration() noexcept;

    /**
     * @brief Reset sensor to default configuration
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError ResetToDefaults() noexcept;

    /**
     * @brief Update sensor configuration
     * @param config New configuration to apply
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError UpdateConfiguration(const As5047uConfig& config) noexcept;

    /**
     * @brief Get current sensor configuration
     * @param config Output current configuration
     * @return As5047uError::SUCCESS if successful
     */
    As5047uError GetConfiguration(As5047uConfig& config) noexcept;

    //======================================================//
    // UTILITY METHODS
    //======================================================//

    /**
     * @brief Convert angle LSB to degrees
     * @param angle_lsb Angle in LSB units (0-16383)
     * @return Angle in degrees (0.0-359.978°)
     */
    static constexpr double LSBToDegrees(uint16_t angle_lsb) noexcept {
        return (angle_lsb * 360.0) / 16384.0;
    }

    /**
     * @brief Convert angle LSB to radians
     * @param angle_lsb Angle in LSB units (0-16383)
     * @return Angle in radians (0.0-2π)
     */
    static constexpr double LSBToRadians(uint16_t angle_lsb) noexcept {
        return (angle_lsb * 2.0 * M_PI) / 16384.0;
    }

    /**
     * @brief Convert degrees to angle LSB
     * @param degrees Angle in degrees
     * @return Angle in LSB units (0-16383)
     */
    static constexpr uint16_t DegreesToLSB(double degrees) noexcept {
        return static_cast<uint16_t>((degrees * 16384.0) / 360.0) & 0x3FFF;
    }

    /**
     * @brief Convert radians to angle LSB
     * @param radians Angle in radians
     * @return Angle in LSB units (0-16383)
     */
    static constexpr uint16_t RadiansToLSB(double radians) noexcept {
        return static_cast<uint16_t>((radians * 16384.0) / (2.0 * M_PI)) & 0x3FFF;
    }

    /**
     * @brief Get default sensor configuration
     * @return Default As5047uConfig structure
     */
    static As5047uConfig GetDefaultConfig() noexcept;

    /**
     * @brief Get sensor description string
     * @return Description of the sensor
     */
    const char* GetDescription() const noexcept;

    /**
     * @brief Get last error code
     * @return Last error that occurred
     */
    As5047uError GetLastError() const noexcept;
    
    /**
     * @brief Dump comprehensive diagnostics and statistics to log as INFO level.
     * Logs AS5047U sensor status, communication health, and measurement statistics.
     */
    void DumpDiagnostics() const noexcept;

private:
    //======================================================//
    // PRIVATE MEMBERS
    //======================================================//

    BaseSpi& spi_ref_;                               ///< Reference to SPI interface
    std::unique_ptr<As5047uSpiAdapter> spi_adapter_; ///< SPI bridge adapter
    std::shared_ptr<AS5047U> as5047u_sensor_;        ///< AS5047U driver instance
    As5047uConfig config_;                           ///< Sensor configuration
    mutable RtosMutex handler_mutex_;                ///< Thread safety mutex
    bool initialized_;                               ///< Initialization state
    mutable As5047uError last_error_;                ///< Last error code
    mutable As5047uDiagnostics diagnostics_;         ///< Cached diagnostics
    char description_[64];                           ///< Sensor description

    //======================================================//
    // PRIVATE HELPER METHODS
    //======================================================//

    /**
     * @brief Validate sensor parameters
     * @return True if sensor is in valid state
     */
    bool ValidateSensor() const noexcept;

    /**
     * @brief Handle sensor errors and update diagnostics
     * @param sensor_errors Error flags from sensor
     */
    void HandleSensorErrors(uint16_t sensor_errors) noexcept;

    /**
     * @brief Update cached diagnostics
     */
    void UpdateDiagnostics() noexcept;

    /**
     * @brief Apply configuration to sensor
     * @param config Configuration to apply
     * @return True if successful
     */
    bool ApplyConfiguration(const As5047uConfig& config) noexcept;
};

//======================================================//
// FACTORY METHODS
//======================================================//

/**
 * @brief Create AS5047U handler instance
 * @param spi_interface Reference to SPI interface
 * @param config Sensor configuration
 * @return Unique pointer to AS5047U handler
 */
std::unique_ptr<As5047uHandler> CreateAs5047uHandler(
    BaseSpi& spi_interface,
    const As5047uConfig& config = As5047uHandler::GetDefaultConfig()) noexcept;

#endif // COMPONENT_HANDLER_AS5047U_HANDLER_H_
