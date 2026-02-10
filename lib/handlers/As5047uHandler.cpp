/**
 * @file As5047uHandler.cpp
 * @brief Implementation of unified AS5047U magnetic rotary position sensor handler.
 *
 * This file implements the comprehensive AS5047U handler with bridge pattern SPI integration,
 * lazy initialization, shared pointer management, and complete exception-free design.
 * 
 * Key implementation features:
 * - As5047uSpiAdapter bridge connecting BaseSpi to AS5047U::spiBus interface
 * - Lazy initialization pattern with deferred object creation
 * - Shared pointer management for safe cross-component access
 * - Thread-safe operations with RtosMutex protection
 * - Comprehensive error handling and validation
 * - High-level sensor abstraction methods
 * - Advanced AS5047U feature implementations
 * - Diagnostic and health monitoring
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 * @copyright HardFOC
 */

#include "As5047uHandler.h"
#include <cstring>
#include <algorithm>
#include <cmath>
#include "utils-and-drivers/driver-handlers/Logger.h"

//======================================================//
// AS5047U SPI ADAPTER IMPLEMENTATION
//======================================================//

As5047uSpiAdapter::As5047uSpiAdapter(BaseSpi& spi_interface) noexcept
    : spi_interface_(spi_interface) {}

void As5047uSpiAdapter::transfer(const uint8_t* tx, uint8_t* rx, std::size_t len) noexcept {
    // Handle null pointer cases gracefully
    if (len == 0) return;
    
    // Perform SPI transfer through BaseSpi interface
    // Note: BaseSpi implementations should handle CS assertion/deassertion
    hf_spi_err_t result = spi_interface_.Transfer(
        const_cast<uint8_t*>(tx),  // BaseSpi expects non-const tx buffer
        rx,
        static_cast<uint16_t>(len),
        1000  // 1 second timeout
    );
    
    // AS5047U::spiBus::transfer doesn't return error codes, so we handle errors internally
    // The AS5047U driver will detect communication issues through CRC and frame validation
    (void)result; // Suppress unused variable warning
}

//======================================================//
// AS5047U HANDLER IMPLEMENTATION
//======================================================//

As5047uHandler::As5047uHandler(BaseSpi& spi_interface, const As5047uConfig& config) noexcept
    : spi_ref_(spi_interface),
      spi_adapter_(nullptr),
      as5047u_sensor_(nullptr),
      config_(config),
      initialized_(false),
      last_error_(As5047uError::SUCCESS),
      diagnostics_{} {
    
    // Generate description string
    snprintf(description_, sizeof(description_), "AS5047U_Handler_SPI");
    
    // Initialize diagnostics structure
    memset(&diagnostics_, 0, sizeof(diagnostics_));
}

As5047uError As5047uHandler::Initialize() noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    // Already initialized - return success
    if (initialized_ && as5047u_sensor_) {
        last_error_ = As5047uError::SUCCESS;
        return last_error_;
    }
    
    // Create SPI adapter (bridge pattern)
    spi_adapter_ = std::make_unique<As5047uSpiAdapter>(spi_ref_);
    if (!spi_adapter_) {
        last_error_ = As5047uError::INITIALIZATION_FAILED;
        return last_error_;
    }
    
    // Create AS5047U sensor instance (lazy initialization)
    as5047u_sensor_ = std::make_shared<AS5047U>(*spi_adapter_, config_.frame_format);
    if (!as5047u_sensor_) {
        spi_adapter_.reset();
        last_error_ = As5047uError::INITIALIZATION_FAILED;
        return last_error_;
    }
    
    // Apply initial configuration
    if (!ApplyConfiguration(config_)) {
        as5047u_sensor_.reset();
        spi_adapter_.reset();
        last_error_ = As5047uError::INITIALIZATION_FAILED;
        return last_error_;
    }
    
    // Test basic sensor communication
    uint16_t test_angle = 0;
    As5047uError comm_test = ReadAngle(test_angle);
    if (comm_test != As5047uError::SUCCESS) {
        as5047u_sensor_.reset();
        spi_adapter_.reset();
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
        return last_error_;
    }
    
    // Initialize diagnostics
    UpdateDiagnostics();
    
    initialized_ = true;
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::Deinitialize() noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    // Reset shared pointer (safe automatic cleanup)
    as5047u_sensor_.reset();
    spi_adapter_.reset();
    
    initialized_ = false;
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

bool As5047uHandler::IsInitialized() const noexcept {
    MutexLockGuard lock(handler_mutex_);
    return initialized_ && as5047u_sensor_ && spi_adapter_;
}

bool As5047uHandler::IsSensorReady() const noexcept {
    MutexLockGuard lock(handler_mutex_);
    return as5047u_sensor_ != nullptr;
}

std::shared_ptr<AS5047U> As5047uHandler::GetSensor() noexcept {
    MutexLockGuard lock(handler_mutex_);
    return as5047u_sensor_;
}

//======================================================//
// SENSOR MEASUREMENTS
//======================================================//

As5047uError As5047uHandler::ReadMeasurement(As5047uMeasurement& measurement) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // Read all measurement data (driver handles retries internally)
    measurement.angle_compensated = as5047u_sensor_->getAngle(config_.crc_retries);
    measurement.angle_raw = as5047u_sensor_->getRawAngle(config_.crc_retries);
    measurement.velocity_raw = as5047u_sensor_->getVelocity(config_.crc_retries);
    measurement.velocity_deg_per_sec = as5047u_sensor_->getVelocityDegPerSec(config_.crc_retries);
    measurement.velocity_rad_per_sec = as5047u_sensor_->getVelocityRadPerSec(config_.crc_retries);
    measurement.velocity_rpm = as5047u_sensor_->getVelocityRPM(config_.crc_retries);
    measurement.agc_value = as5047u_sensor_->getAGC(config_.crc_retries);
    measurement.magnitude = as5047u_sensor_->getMagnitude(config_.crc_retries);
    measurement.error_flags = as5047u_sensor_->getErrorFlags(config_.crc_retries);
    
    // Handle sensor errors
    HandleSensorErrors(measurement.error_flags);
    
    // Check if measurement is valid (no CRC/framing/command errors)
    measurement.valid = (measurement.error_flags == 0) || 
                       !(measurement.error_flags & (static_cast<uint16_t>(AS5047U_Error::CrcError) |
                                                   static_cast<uint16_t>(AS5047U_Error::FramingError) |
                                                   static_cast<uint16_t>(AS5047U_Error::CommandError)));
    
    // Update diagnostics
    diagnostics_.total_measurements++;
    if (!measurement.valid) {
        diagnostics_.communication_errors++;
    }
    
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::ReadAngle(uint16_t& angle) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver handles retries internally and returns the actual value
    angle = as5047u_sensor_->getAngle(config_.crc_retries);
    
    // Check for communication errors using sticky flags
    auto error_flags = as5047u_sensor_->getStickyErrorFlags();
    if (static_cast<uint16_t>(error_flags) & (static_cast<uint16_t>(AS5047U_Error::CrcError) |
                                             static_cast<uint16_t>(AS5047U_Error::FramingError))) {
        last_error_ = As5047uError::CRC_ERROR;
        return last_error_;
    }
    
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::ReadRawAngle(uint16_t& raw_angle) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver handles retries internally and returns the actual value
    raw_angle = as5047u_sensor_->getRawAngle(config_.crc_retries);
    
    // Check for communication errors using sticky flags
    auto error_flags = as5047u_sensor_->getStickyErrorFlags();
    if (static_cast<uint16_t>(error_flags) & (static_cast<uint16_t>(AS5047U_Error::CrcError) |
                                             static_cast<uint16_t>(AS5047U_Error::FramingError))) {
        last_error_ = As5047uError::CRC_ERROR;
        return last_error_;
    }
    
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::ReadVelocity(int16_t& velocity_lsb) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver handles retries internally and returns the actual value
    velocity_lsb = as5047u_sensor_->getVelocity(config_.crc_retries);
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::ReadVelocityDegPerSec(double& velocity_deg_per_sec) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver handles retries internally and returns the actual value
    velocity_deg_per_sec = as5047u_sensor_->getVelocityDegPerSec(config_.crc_retries);
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::ReadVelocityRadPerSec(double& velocity_rad_per_sec) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver handles retries internally and returns the actual value
    velocity_rad_per_sec = as5047u_sensor_->getVelocityRadPerSec(config_.crc_retries);
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::ReadVelocityRPM(double& velocity_rpm) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver handles retries internally and returns the actual value
    velocity_rpm = as5047u_sensor_->getVelocityRPM(config_.crc_retries);
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

//======================================================//
// SENSOR DIAGNOSTICS
//======================================================//

As5047uError As5047uHandler::ReadDiagnostics(As5047uDiagnostics& diagnostics) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    UpdateDiagnostics();
    diagnostics = diagnostics_;
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::ReadAGC(uint8_t& agc_value) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver handles retries internally and returns the actual value
    agc_value = as5047u_sensor_->getAGC(config_.crc_retries);
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::ReadMagnitude(uint16_t& magnitude) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver handles retries internally and returns the actual value
    magnitude = as5047u_sensor_->getMagnitude(config_.crc_retries);
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::ReadErrorFlags(uint16_t& error_flags) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver handles retries internally and returns the actual value
    error_flags = as5047u_sensor_->getErrorFlags(config_.crc_retries);
    HandleSensorErrors(error_flags);
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::IsMagneticFieldOK(bool& field_ok) noexcept {
    uint8_t agc_value = 0;
    As5047uError result = ReadAGC(agc_value);
    
    if (result == As5047uError::SUCCESS) {
        // AGC values between 30 and 225 typically indicate good magnetic field
        field_ok = (agc_value >= 30 && agc_value <= 225);
    } else {
        field_ok = false;
    }
    
    return result;
}

//======================================================//
// SENSOR CONFIGURATION
//======================================================//

As5047uError As5047uHandler::SetZeroPosition(uint16_t zero_position) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    if (zero_position > 16383) {
        last_error_ = As5047uError::INVALID_PARAMETER;
        return last_error_;
    }
    
    // The driver returns true on success, false on communication failure
    bool success = as5047u_sensor_->setZeroPosition(zero_position, config_.crc_retries);
    if (success) {
        config_.zero_position = zero_position;
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::GetZeroPosition(uint16_t& zero_position) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver handles retries internally and returns the actual value
    zero_position = as5047u_sensor_->getZeroPosition(config_.crc_retries);
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

As5047uError As5047uHandler::SetRotationDirection(bool clockwise) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver returns true on success, false on communication failure
    bool success = as5047u_sensor_->setDirection(clockwise, config_.crc_retries);
    if (success) {
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::SetDAEC(bool enable) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver returns true on success, false on communication failure
    bool success = as5047u_sensor_->setDynamicAngleCompensation(enable, config_.crc_retries);
    if (success) {
        config_.enable_daec = enable;
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::SetAdaptiveFilter(bool enable) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver returns true on success, false on communication failure
    bool success = as5047u_sensor_->setAdaptiveFilter(enable, config_.crc_retries);
    if (success) {
        config_.enable_adaptive_filter = enable;
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::ConfigureInterface(bool enable_abi, bool enable_uvw, bool enable_pwm) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver returns true on success, false on communication failure
    bool success = as5047u_sensor_->configureInterface(enable_abi, enable_uvw, enable_pwm, config_.crc_retries);
    if (success) {
        config_.enable_abi_output = enable_abi;
        config_.enable_uvw_output = enable_uvw;
        config_.enable_pwm_output = enable_pwm;
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::SetABIResolution(uint8_t resolution_bits) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    if (resolution_bits < 10 || resolution_bits > 14) {
        last_error_ = As5047uError::INVALID_PARAMETER;
        return last_error_;
    }
    
    // The driver returns true on success, false on communication failure
    bool success = as5047u_sensor_->setABIResolution(resolution_bits, config_.crc_retries);
    if (success) {
        config_.abi_resolution_bits = resolution_bits;
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::SetUVWPolePairs(uint8_t pole_pairs) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    if (pole_pairs < 1 || pole_pairs > 7) {
        last_error_ = As5047uError::INVALID_PARAMETER;
        return last_error_;
    }
    
    // The driver returns true on success, false on communication failure
    bool success = as5047u_sensor_->setUVWPolePairs(pole_pairs, config_.crc_retries);
    if (success) {
        config_.uvw_pole_pairs = pole_pairs;
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::SetHighTemperatureMode(bool enable) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver returns true on success, false on communication failure
    bool success = as5047u_sensor_->set150CTemperatureMode(enable, config_.crc_retries);
    if (success) {
        config_.high_temperature_mode = enable;
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
    }
    return last_error_;
}

//======================================================//
// ADVANCED FEATURES
//======================================================//

As5047uError As5047uHandler::ProgramOTP() noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // The driver returns true on success, false on programming failure
    bool success = as5047u_sensor_->programOTP();
    if (success) {
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::OTP_PROGRAMMING_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::PerformCalibration() noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // Read current angle to use as zero reference
    uint16_t current_angle = as5047u_sensor_->getAngle(config_.crc_retries);
    
    // Set current position as zero (driver returns true on success, false on failure)
    bool success = as5047u_sensor_->setZeroPosition(current_angle, config_.crc_retries);
    if (success) {
        config_.zero_position = current_angle;
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::CALIBRATION_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::ResetToDefaults() noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // Apply default configuration
    As5047uConfig default_config = GetDefaultConfig();
    bool success = ApplyConfiguration(default_config);
    if (success) {
        config_ = default_config;
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::UpdateConfiguration(const As5047uConfig& config) noexcept {
    MutexLockGuard lock(handler_mutex_);
    
    if (!ValidateSensor()) {
        last_error_ = As5047uError::NOT_INITIALIZED;
        return last_error_;
    }
    
    bool success = ApplyConfiguration(config);
    if (success) {
        config_ = config;
        last_error_ = As5047uError::SUCCESS;
    } else {
        last_error_ = As5047uError::SPI_COMMUNICATION_FAILED;
    }
    return last_error_;
}

As5047uError As5047uHandler::GetConfiguration(As5047uConfig& config) noexcept {
    MutexLockGuard lock(handler_mutex_);
    config = config_;
    last_error_ = As5047uError::SUCCESS;
    return last_error_;
}

//======================================================//
// UTILITY METHODS
//======================================================//

As5047uConfig As5047uHandler::GetDefaultConfig() noexcept {
    As5047uConfig config;
    config.frame_format = FrameFormat::SPI_16;
    config.crc_retries = 2;
    config.enable_daec = true;
    config.enable_adaptive_filter = true;
    config.zero_position = 0;
    config.enable_abi_output = false;
    config.enable_uvw_output = false;
    config.enable_pwm_output = false;
    config.abi_resolution_bits = 14;
    config.uvw_pole_pairs = 1;
    config.high_temperature_mode = false;
    return config;
}

const char* As5047uHandler::GetDescription() const noexcept {
    return description_;
}

As5047uError As5047uHandler::GetLastError() const noexcept {
    MutexLockGuard lock(handler_mutex_);
    return last_error_;
}

//======================================================//
// PRIVATE HELPER METHODS
//======================================================//

bool As5047uHandler::ValidateSensor() const noexcept {
    return initialized_ && as5047u_sensor_ && spi_adapter_;
}

void As5047uHandler::HandleSensorErrors(uint16_t sensor_errors) noexcept {
    diagnostics_.last_error_flags = sensor_errors;
    
    // Update specific diagnostic flags
    diagnostics_.agc_warning = (sensor_errors & static_cast<uint16_t>(AS5047U_Error::AgcWarning)) != 0;
    diagnostics_.cordic_overflow = (sensor_errors & static_cast<uint16_t>(AS5047U_Error::CordicOverflow)) != 0;
    diagnostics_.offset_compensation_ok = (sensor_errors & static_cast<uint16_t>(AS5047U_Error::OffCompError)) == 0;
    diagnostics_.communication_ok = (sensor_errors & (static_cast<uint16_t>(AS5047U_Error::CrcError) |
                                                     static_cast<uint16_t>(AS5047U_Error::FramingError) |
                                                     static_cast<uint16_t>(AS5047U_Error::CommandError))) == 0;
    
    // Check magnetic field status
    diagnostics_.magnetic_field_ok = (sensor_errors & static_cast<uint16_t>(AS5047U_Error::MagHalf)) == 0;
}

void As5047uHandler::UpdateDiagnostics() noexcept {
    if (!as5047u_sensor_) return;
    
    // Read current error flags (driver handles retries internally)
    uint16_t error_flags = as5047u_sensor_->getErrorFlags(config_.crc_retries);
    HandleSensorErrors(error_flags);
}

bool As5047uHandler::ApplyConfiguration(const As5047uConfig& config) noexcept {
    if (!as5047u_sensor_) return false;
    
    bool success = true;
    
    // Set frame format (no return value, always succeeds)
    as5047u_sensor_->setFrameFormat(config.frame_format);
    
    // Configure DAEC (driver returns true on success, false on failure)
    success &= as5047u_sensor_->setDynamicAngleCompensation(config.enable_daec, config.crc_retries);
    
    // Configure adaptive filter (driver returns true on success, false on failure)
    success &= as5047u_sensor_->setAdaptiveFilter(config.enable_adaptive_filter, config.crc_retries);
    
    // Set zero position (driver returns true on success, false on failure)
    success &= as5047u_sensor_->setZeroPosition(config.zero_position, config.crc_retries);
    
    // Configure interfaces (driver returns true on success, false on failure)
    success &= as5047u_sensor_->configureInterface(config.enable_abi_output, 
                                                   config.enable_uvw_output, 
                                                   config.enable_pwm_output, 
                                                   config.crc_retries);
    
    // Set ABI resolution (driver returns true on success, false on failure)
    if (config.enable_abi_output) {
        success &= as5047u_sensor_->setABIResolution(config.abi_resolution_bits, config.crc_retries);
    }
    
    // Set UVW pole pairs (driver returns true on success, false on failure)
    if (config.enable_uvw_output) {
        success &= as5047u_sensor_->setUVWPolePairs(config.uvw_pole_pairs, config.crc_retries);
    }
    
    // Set temperature mode (driver returns true on success, false on failure)
    success &= as5047u_sensor_->set150CTemperatureMode(config.high_temperature_mode, config.crc_retries);
    
    return success;
}

//======================================================//
// FACTORY METHODS
//======================================================//

std::unique_ptr<As5047uHandler> CreateAs5047uHandler(BaseSpi& spi_interface, const As5047uConfig& config) noexcept {
    return std::make_unique<As5047uHandler>(spi_interface, config);
}

void As5047uHandler::DumpDiagnostics() const noexcept {
    static constexpr const char* TAG = "As5047uHandler";
    
    Logger::GetInstance().Info(TAG, "=== AS5047U HANDLER DIAGNOSTICS ===");
    
    RtosMutex::LockGuard lock(handler_mutex_);
    
    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", initialized_ ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Last Error: %s", 
        last_error_ == As5047uError::SUCCESS ? "SUCCESS" :
        last_error_ == As5047uError::NOT_INITIALIZED ? "NOT_INITIALIZED" :
        last_error_ == As5047uError::COMMUNICATION_ERROR ? "COMMUNICATION_ERROR" :
        last_error_ == As5047uError::SENSOR_ERROR ? "SENSOR_ERROR" :
        last_error_ == As5047uError::INVALID_PARAMETER ? "INVALID_PARAMETER" :
        last_error_ == As5047uError::TIMEOUT ? "TIMEOUT" : "UNKNOWN");
    
    // Sensor Status
    Logger::GetInstance().Info(TAG, "Sensor Status:");
    if (as5047u_sensor_) {
        Logger::GetInstance().Info(TAG, "  Driver Instance: ACTIVE");
        Logger::GetInstance().Info(TAG, "  Description: %s", description_);
    } else {
        Logger::GetInstance().Info(TAG, "  Driver Instance: NOT_INITIALIZED");
    }
    
    // Configuration
    Logger::GetInstance().Info(TAG, "Configuration:");
    Logger::GetInstance().Info(TAG, "  Frame Format: %s",
        config_.frame_format == FrameFormat::FRAME_16BIT ? "16-bit" :
        config_.frame_format == FrameFormat::FRAME_24BIT ? "24-bit" :
        config_.frame_format == FrameFormat::FRAME_32BIT ? "32-bit" : "Unknown");
    Logger::GetInstance().Info(TAG, "  CRC Retries: %d", config_.crc_retries);
    Logger::GetInstance().Info(TAG, "  DAEC Enabled: %s", config_.enable_daec ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Adaptive Filter: %s", config_.enable_adaptive_filter ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Zero Position: %d", config_.zero_position);
    Logger::GetInstance().Info(TAG, "  ABI Output: %s", config_.enable_abi_output ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  UVW Output: %s", config_.enable_uvw_output ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  PWM Output: %s", config_.enable_pwm_output ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  ABI Resolution: %d bits", config_.abi_resolution_bits);
    Logger::GetInstance().Info(TAG, "  UVW Pole Pairs: %d", config_.uvw_pole_pairs);
    Logger::GetInstance().Info(TAG, "  High Temp Mode: %s", config_.high_temperature_mode ? "YES" : "NO");
    
    // Diagnostics Information
    Logger::GetInstance().Info(TAG, "Sensor Diagnostics:");
    Logger::GetInstance().Info(TAG, "  Magnetic Field OK: %s", diagnostics_.magnetic_field_ok ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  AGC Warning: %s", diagnostics_.agc_warning ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  CORDIC Overflow: %s", diagnostics_.cordic_overflow ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Offset Compensation OK: %s", diagnostics_.offset_compensation_ok ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Communication OK: %s", diagnostics_.communication_ok ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Last Error Flags: 0x%04X", diagnostics_.last_error_flags);
    Logger::GetInstance().Info(TAG, "  Communication Errors: %d", diagnostics_.communication_errors);
    Logger::GetInstance().Info(TAG, "  Total Measurements: %d", diagnostics_.total_measurements);
    
    // SPI Interface Status
    Logger::GetInstance().Info(TAG, "SPI Interface:");
    if (spi_adapter_) {
        Logger::GetInstance().Info(TAG, "  SPI Adapter: ACTIVE");
    } else {
        Logger::GetInstance().Info(TAG, "  SPI Adapter: NOT_INITIALIZED");
    }
    
    // Performance Metrics
    if (diagnostics_.total_measurements > 0) {
        float error_rate = (float)diagnostics_.communication_errors / diagnostics_.total_measurements * 100.0f;
        Logger::GetInstance().Info(TAG, "Performance Metrics:");
        Logger::GetInstance().Info(TAG, "  Error Rate: %.2f%%", error_rate);
        Logger::GetInstance().Info(TAG, "  Success Rate: %.2f%%", 100.0f - error_rate);
    }
    
    // Memory Usage
    Logger::GetInstance().Info(TAG, "Memory Usage:");
    size_t estimated_memory = sizeof(*this);
    if (as5047u_sensor_) estimated_memory += sizeof(AS5047U);
    if (spi_adapter_) estimated_memory += sizeof(As5047uSpiAdapter);
    Logger::GetInstance().Info(TAG, "  Estimated Total: %d bytes", static_cast<int>(estimated_memory));
    
    // System Status Summary
    bool system_healthy = initialized_ && 
                         (last_error_ == As5047uError::SUCCESS) &&
                         diagnostics_.magnetic_field_ok &&
                         diagnostics_.communication_ok &&
                         !diagnostics_.cordic_overflow;
    
    Logger::GetInstance().Info(TAG, "System Status: %s", system_healthy ? "HEALTHY" : "DEGRADED");
    
    Logger::GetInstance().Info(TAG, "=== END AS5047U HANDLER DIAGNOSTICS ===");
}
