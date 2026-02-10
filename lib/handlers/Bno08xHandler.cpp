/**
 * @file Bno08xHandler.cpp
 * @brief Implementation of unified BNO08x IMU sensor handler.
 *
 * This implementation provides complete BNO08x sensor management with:
 * - Bridge pattern adapters for I2C/SPI integration
 * - Lazy initialization with shared pointer management
 * - Exception-free design for embedded reliability
 * - Thread-safe operations with RtosMutex protection
 * - Comprehensive IMU data processing and calibration
 * - Advanced gesture and activity detection
 * - Hardware control and interface management
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 * @copyright HardFOC
 */

#include "Bno08xHandler.h"
#include <cmath>
#include <algorithm>
#include "utils/RtosTask.h"
#include "utils-and-drivers/driver-handlers/Logger.h"

// Include SH2 error codes for proper error handling
extern "C" {
#include "sh2_err.h"
}

//======================================================//
// BNO08X I2C ADAPTER IMPLEMENTATION
//======================================================//

Bno08xI2cAdapter::Bno08xI2cAdapter(BaseI2c& i2c_interface, 
                                   BaseGpio* reset_gpio,
                                   BaseGpio* int_gpio) noexcept
    : i2c_interface_(i2c_interface)
    , reset_gpio_(reset_gpio)
    , int_gpio_(int_gpio)
    , is_open_(false) {
}

bool Bno08xI2cAdapter::open() {
    if (is_open_) return true;
    
    // Initialize GPIO pins if provided
    if (reset_gpio_) {
        reset_gpio_->SetDirection(BaseGpio::Direction::OUTPUT);
        reset_gpio_->SetValue(BaseGpio::Value::HIGH); // Keep out of reset
    }
    
    if (int_gpio_) {
        int_gpio_->SetDirection(BaseGpio::Direction::INPUT);
        int_gpio_->SetPullMode(BaseGpio::PullMode::PULL_UP);
    }
    
    // Initialize I2C interface
    if (i2c_interface_.Initialize() == BaseI2c::Result::SUCCESS) {
        is_open_ = true;
        return true;
    }
    
    return false;
}

void Bno08xI2cAdapter::close() {
    if (!is_open_) return;
    
    i2c_interface_.Deinitialize();
    is_open_ = false;
}

int Bno08xI2cAdapter::write(const uint8_t* data, uint32_t length) {
    if (!is_open_ || !data || length == 0) return -1;
    
    BaseI2c::TransferData transfer_data;
    transfer_data.buffer = const_cast<uint8_t*>(data);
    transfer_data.length = static_cast<uint16_t>(length);
    transfer_data.slave_address = 0x4A; // Default BNO08x I2C address
    
    if (i2c_interface_.Write(transfer_data) == BaseI2c::Result::SUCCESS) {
        return static_cast<int>(length);
    }
    
    return -1;
}

int Bno08xI2cAdapter::read(uint8_t* data, uint32_t length) {
    if (!is_open_ || !data || length == 0) return -1;
    
    BaseI2c::TransferData transfer_data;
    transfer_data.buffer = data;
    transfer_data.length = static_cast<uint16_t>(length);
    transfer_data.slave_address = 0x4A; // Default BNO08x I2C address
    
    if (i2c_interface_.Read(transfer_data) == BaseI2c::Result::SUCCESS) {
        return static_cast<int>(length);
    }
    
    return -1;
}

bool Bno08xI2cAdapter::dataAvailable() {
    if (!int_gpio_) return true; // Always assume data available if no interrupt pin
    
    // Check interrupt pin - active low for BNO08x
    return int_gpio_->GetValue() == BaseGpio::Value::LOW;
}

void Bno08xI2cAdapter::delay(uint32_t ms) {
    RtosTask::Delay(ms);
}

uint32_t Bno08xI2cAdapter::getTimeUs() {
    return static_cast<uint32_t>(RtosTask::GetTickCount() * 1000); // Convert ticks to microseconds
}

void Bno08xI2cAdapter::setReset(bool state) {
    if (reset_gpio_) {
        reset_gpio_->SetValue(state ? BaseGpio::Value::LOW : BaseGpio::Value::HIGH);
    }
}

void Bno08xI2cAdapter::setBoot(bool state) {
    // Boot mode control not typically used with I2C interface
    (void)state;
}

//======================================================//
// BNO08X SPI ADAPTER IMPLEMENTATION
//======================================================//

Bno08xSpiAdapter::Bno08xSpiAdapter(BaseSpi& spi_interface,
                                   BaseGpio* wake_gpio,
                                   BaseGpio* reset_gpio,
                                   BaseGpio* int_gpio) noexcept
    : spi_interface_(spi_interface)
    , wake_gpio_(wake_gpio)
    , reset_gpio_(reset_gpio)
    , int_gpio_(int_gpio)
    , is_open_(false) {
}

bool Bno08xSpiAdapter::open() {
    if (is_open_) return true;
    
    // Initialize GPIO pins
    if (wake_gpio_) {
        wake_gpio_->SetDirection(BaseGpio::Direction::OUTPUT);
        wake_gpio_->SetValue(BaseGpio::Value::HIGH); // Wake pin high for SPI
    }
    
    if (reset_gpio_) {
        reset_gpio_->SetDirection(BaseGpio::Direction::OUTPUT);
        reset_gpio_->SetValue(BaseGpio::Value::HIGH); // Keep out of reset
    }
    
    if (int_gpio_) {
        int_gpio_->SetDirection(BaseGpio::Direction::INPUT);
        int_gpio_->SetPullMode(BaseGpio::PullMode::PULL_UP);
    }
    
    // Configure SPI settings for BNO08x
    BaseSpi::Config spi_config;
    spi_config.clock_frequency = 3000000; // 3 MHz max for BNO08x
    spi_config.data_width = BaseSpi::DataWidth::BITS_8;
    spi_config.mode = BaseSpi::Mode::MODE_3; // CPOL=1, CPHA=1
    spi_config.bit_order = BaseSpi::BitOrder::MSB_FIRST;
    
    if (spi_interface_.Configure(spi_config) == BaseSpi::Result::SUCCESS &&
        spi_interface_.Initialize() == BaseSpi::Result::SUCCESS) {
        is_open_ = true;
        return true;
    }
    
    return false;
}

void Bno08xSpiAdapter::close() {
    if (!is_open_) return;
    
    spi_interface_.Deinitialize();
    is_open_ = false;
}

int Bno08xSpiAdapter::write(const uint8_t* data, uint32_t length) {
    if (!is_open_ || !data || length == 0) return -1;
    
    BaseSpi::TransferData transfer_data;
    transfer_data.tx_buffer = const_cast<uint8_t*>(data);
    transfer_data.tx_length = static_cast<uint16_t>(length);
    transfer_data.rx_buffer = nullptr;
    transfer_data.rx_length = 0;
    
    if (spi_interface_.Transfer(transfer_data) == BaseSpi::Result::SUCCESS) {
        return static_cast<int>(length);
    }
    
    return -1;
}

int Bno08xSpiAdapter::read(uint8_t* data, uint32_t length) {
    if (!is_open_ || !data || length == 0) return -1;
    
    BaseSpi::TransferData transfer_data;
    transfer_data.tx_buffer = nullptr;
    transfer_data.tx_length = 0;
    transfer_data.rx_buffer = data;
    transfer_data.rx_length = static_cast<uint16_t>(length);
    
    if (spi_interface_.Transfer(transfer_data) == BaseSpi::Result::SUCCESS) {
        return static_cast<int>(length);
    }
    
    return -1;
}

bool Bno08xSpiAdapter::dataAvailable() {
    if (!int_gpio_) return true; // Always assume data available if no interrupt pin
    
    // Check interrupt pin - active low for BNO08x
    return int_gpio_->GetValue() == BaseGpio::Value::LOW;
}

void Bno08xSpiAdapter::delay(uint32_t ms) {
    RtosTask::Delay(ms);
}

uint32_t Bno08xSpiAdapter::getTimeUs() {
    return static_cast<uint32_t>(RtosTask::GetTickCount() * 1000); // Convert ticks to microseconds
}

void Bno08xSpiAdapter::setReset(bool state) {
    if (reset_gpio_) {
        reset_gpio_->SetValue(state ? BaseGpio::Value::LOW : BaseGpio::Value::HIGH);
    }
}

void Bno08xSpiAdapter::setBoot(bool state) {
    // Boot mode control - implementation depends on hardware design
    (void)state;
}

void Bno08xSpiAdapter::setWake(bool state) {
    if (wake_gpio_) {
        wake_gpio_->SetValue(state ? BaseGpio::Value::HIGH : BaseGpio::Value::LOW);
    }
}

//======================================================//
// BNO08X HANDLER IMPLEMENTATION
//======================================================//

Bno08xHandler::Bno08xHandler(BaseI2c& i2c_interface,
                            const Bno08xConfig& config,
                            BaseGpio* reset_gpio,
                            BaseGpio* int_gpio) noexcept
    : transport_adapter_(std::make_unique<Bno08xI2cAdapter>(i2c_interface, reset_gpio, int_gpio))
    , bno08x_sensor_(nullptr)
    , config_(config)
    , handler_mutex_()
    , initialized_(false)
    , last_error_(Bno08xError::SUCCESS)
    , sensor_callback_(nullptr)
    , interface_type_(BNO085Interface::I2C)
    , reset_gpio_(reset_gpio)
    , wake_gpio_(nullptr)
    , int_gpio_(int_gpio) {
    
    snprintf(description_, sizeof(description_), 
             "BNO08x IMU Handler (I2C) - 9-DOF Sensor Fusion");
}

Bno08xHandler::Bno08xHandler(BaseSpi& spi_interface,
                            const Bno08xConfig& config,
                            BaseGpio* wake_gpio,
                            BaseGpio* reset_gpio,
                            BaseGpio* int_gpio) noexcept
    : transport_adapter_(std::make_unique<Bno08xSpiAdapter>(spi_interface, wake_gpio, reset_gpio, int_gpio))
    , bno08x_sensor_(nullptr)
    , config_(config)
    , handler_mutex_()
    , initialized_(false)
    , last_error_(Bno08xError::SUCCESS)
    , sensor_callback_(nullptr)
    , interface_type_(BNO085Interface::SPI)
    , reset_gpio_(reset_gpio)
    , wake_gpio_(wake_gpio)
    , int_gpio_(int_gpio) {
    
    snprintf(description_, sizeof(description_), 
             "BNO08x IMU Handler (SPI) - 9-DOF Sensor Fusion");
}

Bno08xError Bno08xHandler::Initialize() noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        last_error_ = Bno08xError::MUTEX_LOCK_FAILED;
        return last_error_;
    }
    
    if (initialized_) {
        last_error_ = Bno08xError::SUCCESS;
        return last_error_;
    }
    
    // Open transport interface
    if (!transport_adapter_->open()) {
        last_error_ = Bno08xError::COMMUNICATION_FAILED;
        return last_error_;
    }
    
    // Create BNO08x sensor instance with transport
    bno08x_sensor_ = std::make_shared<BNO085>(*transport_adapter_);
    
    if (!bno08x_sensor_) {
        transport_adapter_->close();
        last_error_ = Bno08xError::INITIALIZATION_FAILED;
        return last_error_;
    }
    
    // Perform hardware reset if GPIO available
    if (reset_gpio_) {
        HardwareReset(10);
        RtosTask::Delay(100); // Wait for reset to complete
    }
    
    // Initialize sensor (driver returns true on success, false on failure)
    if (!bno08x_sensor_->begin()) {
        bno08x_sensor_.reset();
        transport_adapter_->close();
        last_error_ = Bno08xError::SENSOR_NOT_RESPONDING;
        return last_error_;
    }
    
    // Apply initial configuration
    if (!ApplyConfiguration(config_)) {
        bno08x_sensor_.reset();
        transport_adapter_->close();
        last_error_ = Bno08xError::INITIALIZATION_FAILED;
        return last_error_;
    }
    
    // Set internal callback for sensor events
    bno08x_sensor_->setCallback([this](const SensorEvent& event) {
        this->HandleSensorEvent(event);
    });
    
    initialized_ = true;
    last_error_ = Bno08xError::SUCCESS;
    
    return last_error_;
}

Bno08xError Bno08xHandler::Deinitialize() noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        last_error_ = Bno08xError::MUTEX_LOCK_FAILED;
        return last_error_;
    }
    
    if (!initialized_) {
        last_error_ = Bno08xError::SUCCESS;
        return last_error_;
    }
    
    // Clear callback
    if (bno08x_sensor_) {
        bno08x_sensor_->setCallback(nullptr);
    }
    
    // Reset sensor instance
    bno08x_sensor_.reset();
    
    // Close transport
    if (transport_adapter_) {
        transport_adapter_->close();
    }
    
    initialized_ = false;
    last_error_ = Bno08xError::SUCCESS;
    
    return last_error_;
}

bool Bno08xHandler::IsInitialized() const noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    return lock.IsLocked() && initialized_;
}

bool Bno08xHandler::IsSensorReady() const noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    return lock.IsLocked() && initialized_ && bno08x_sensor_ && ValidateSensor();
}

std::shared_ptr<BNO085> Bno08xHandler::GetSensor() noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (!lock.IsLocked() || !initialized_) {
        return nullptr;
    }
    return bno08x_sensor_;
}

Bno08xError Bno08xHandler::Update() noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        last_error_ = Bno08xError::MUTEX_LOCK_FAILED;
        return last_error_;
    }
    
    if (!initialized_ || !ValidateSensor()) {
        last_error_ = Bno08xError::NOT_INITIALIZED;
        return last_error_;
    }
    
    // Process any available sensor data (driver handles errors internally)
    bno08x_sensor_->update();
    
    // Check for driver errors after update
    int driver_error = bno08x_sensor_->getLastError();
    if (driver_error != SH2_OK) {
        last_error_ = Bno08xError::COMMUNICATION_FAILED;
    } else {
        last_error_ = Bno08xError::SUCCESS;
    }
    
    return last_error_;
}

bool Bno08xHandler::HasNewData(BNO085Sensor sensor) const noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return false;
    }
    
    return bno08x_sensor_->hasNewData(sensor);
}

Bno08xError Bno08xHandler::ReadImuData(Bno08xImuData& imu_data) noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        return Bno08xError::MUTEX_LOCK_FAILED;
    }
    
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    // Read acceleration
    ReadAcceleration(imu_data.acceleration);
    
    // Read gyroscope
    ReadGyroscope(imu_data.gyroscope);
    
    // Read magnetometer
    ReadMagnetometer(imu_data.magnetometer);
    
    // Read linear acceleration
    ReadLinearAcceleration(imu_data.linear_acceleration);
    
    // Read gravity
    ReadGravity(imu_data.gravity);
    
    // Read rotation quaternion
    ReadQuaternion(imu_data.rotation);
    
    // Convert to Euler angles
    QuaternionToEuler(imu_data.rotation, imu_data.euler);
    
    // Set overall timestamp
    imu_data.timestamp_us = this->getTimeUs();
    imu_data.valid = true;
    
    return Bno08xError::SUCCESS;
}

Bno08xError Bno08xHandler::ReadAcceleration(Bno08xVector3& acceleration) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    // Check if new data is available before reading
    if (!bno08x_sensor_->hasNewData(BNO085Sensor::Accelerometer)) {
        acceleration.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
    
    // The driver uses getLatest() to retrieve sensor data
    SensorEvent event = bno08x_sensor_->getLatest(BNO085Sensor::Accelerometer);
    acceleration.x = event.vector.x;
    acceleration.y = event.vector.y;
    acceleration.z = event.vector.z;
    acceleration.accuracy = event.vector.accuracy;
    acceleration.timestamp_us = event.timestamp;
    acceleration.valid = true;  // Data is valid if we got here
    
    return Bno08xError::SUCCESS;
}

Bno08xError Bno08xHandler::ReadGyroscope(Bno08xVector3& gyroscope) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    // Check if new data is available before reading
    if (!bno08x_sensor_->hasNewData(BNO085Sensor::Gyroscope)) {
        gyroscope.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
    
    // The driver uses getLatest() to retrieve sensor data
    SensorEvent event = bno08x_sensor_->getLatest(BNO085Sensor::Gyroscope);
    gyroscope.x = event.vector.x;
    gyroscope.y = event.vector.y;
    gyroscope.z = event.vector.z;
    gyroscope.accuracy = event.vector.accuracy;
    gyroscope.timestamp_us = event.timestamp;
    gyroscope.valid = true;  // Data is valid if we got here
    
    return Bno08xError::SUCCESS;
}

Bno08xError Bno08xHandler::ReadMagnetometer(Bno08xVector3& magnetometer) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    // Check if new data is available before reading
    if (!bno08x_sensor_->hasNewData(BNO085Sensor::Magnetometer)) {
        magnetometer.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
    
    // The driver uses getLatest() to retrieve sensor data
    SensorEvent event = bno08x_sensor_->getLatest(BNO085Sensor::Magnetometer);
    magnetometer.x = event.vector.x;
    magnetometer.y = event.vector.y;
    magnetometer.z = event.vector.z;
    magnetometer.accuracy = event.vector.accuracy;
    magnetometer.timestamp_us = event.timestamp;
    magnetometer.valid = true;  // Data is valid if we got here
    
    return Bno08xError::SUCCESS;
}

Bno08xError Bno08xHandler::ReadQuaternion(Bno08xQuaternion& quaternion) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    // Check if new data is available before reading
    if (!bno08x_sensor_->hasNewData(BNO085Sensor::RotationVector)) {
        quaternion.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
    
    // The driver uses getLatest() to retrieve sensor data
    SensorEvent event = bno08x_sensor_->getLatest(BNO085Sensor::RotationVector);
    quaternion.w = event.rotation.w;
    quaternion.x = event.rotation.x;
    quaternion.y = event.rotation.y;
    quaternion.z = event.rotation.z;
    quaternion.accuracy = event.rotation.accuracy;
    quaternion.timestamp_us = event.timestamp;
    quaternion.valid = true;  // Data is valid if we got here
    
    return Bno08xError::SUCCESS;
}

Bno08xError Bno08xHandler::ReadEulerAngles(Bno08xEulerAngles& euler_angles) noexcept {
    Bno08xQuaternion quat;
    Bno08xError result = ReadQuaternion(quat);
    
    if (result == Bno08xError::SUCCESS && quat.valid) {
        QuaternionToEuler(quat, euler_angles);
        return Bno08xError::SUCCESS;
    }
    
    euler_angles.valid = false;
    return result;
}

Bno08xError Bno08xHandler::ReadLinearAcceleration(Bno08xVector3& linear_accel) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    // Check if new data is available before reading
    if (!bno08x_sensor_->hasNewData(BNO085Sensor::LinearAcceleration)) {
        linear_accel.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
    
    // The driver uses getLatest() to retrieve sensor data
    SensorEvent event = bno08x_sensor_->getLatest(BNO085Sensor::LinearAcceleration);
    linear_accel.x = event.vector.x;
    linear_accel.y = event.vector.y;
    linear_accel.z = event.vector.z;
    linear_accel.accuracy = event.vector.accuracy;
    linear_accel.timestamp_us = event.timestamp;
    linear_accel.valid = true;  // Data is valid if we got here
    
    return Bno08xError::SUCCESS;
}

Bno08xError Bno08xHandler::ReadGravity(Bno08xVector3& gravity) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    // Check if new data is available before reading
    if (!bno08x_sensor_->hasNewData(BNO085Sensor::Gravity)) {
        gravity.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
    
    // The driver uses getLatest() to retrieve sensor data
    SensorEvent event = bno08x_sensor_->getLatest(BNO085Sensor::Gravity);
    gravity.x = event.vector.x;
    gravity.y = event.vector.y;
    gravity.z = event.vector.z;
    gravity.accuracy = event.vector.accuracy;
    gravity.timestamp_us = event.timestamp;
    gravity.valid = true;  // Data is valid if we got here
    
    return Bno08xError::SUCCESS;
}

Bno08xError Bno08xHandler::EnableSensor(BNO085Sensor sensor, uint32_t interval_ms, float sensitivity) noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        return Bno08xError::MUTEX_LOCK_FAILED;
    }
    
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    // The driver returns true on success, false on failure
    if (bno08x_sensor_->enableSensor(sensor, interval_ms, sensitivity)) {
        return Bno08xError::SUCCESS;
    } else {
        // Check driver's last error for more specific error information
        int driver_error = bno08x_sensor_->getLastError();
        if (driver_error == SH2_ERR_BAD_PARAM) {
            return Bno08xError::INVALID_PARAMETER;
        } else if (driver_error == SH2_ERR_HUB) {
            return Bno08xError::SENSOR_NOT_RESPONDING;
        } else {
            return Bno08xError::COMMUNICATION_FAILED;
        }
    }
}

Bno08xError Bno08xHandler::DisableSensor(BNO085Sensor sensor) noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        return Bno08xError::MUTEX_LOCK_FAILED;
    }
    
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    // The driver returns true on success, false on failure
    if (bno08x_sensor_->disableSensor(sensor)) {
        return Bno08xError::SUCCESS;
    } else {
        // Check driver's last error for more specific error information
        int driver_error = bno08x_sensor_->getLastError();
        if (driver_error == SH2_ERR_BAD_PARAM) {
            return Bno08xError::INVALID_PARAMETER;
        } else if (driver_error == SH2_ERR_HUB) {
            return Bno08xError::SENSOR_NOT_RESPONDING;
        } else {
            return Bno08xError::COMMUNICATION_FAILED;
        }
    }
}

Bno08xError Bno08xHandler::HardwareReset(uint32_t reset_duration_ms) noexcept {
    if (!reset_gpio_) {
        return Bno08xError::HARDWARE_ERROR;
    }
    
    // Hold reset low
    reset_gpio_->SetValue(BaseGpio::Value::LOW);
    RtosTask::Delay(reset_duration_ms);
    
    // Release reset
    reset_gpio_->SetValue(BaseGpio::Value::HIGH);
    RtosTask::Delay(100); // Allow time for reset to complete
    
    return Bno08xError::SUCCESS;
}

void Bno08xHandler::SetSensorCallback(SensorCallback callback) noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (lock.IsLocked()) {
        sensor_callback_ = callback;
    }
}

void Bno08xHandler::ClearSensorCallback() noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (lock.IsLocked()) {
        sensor_callback_ = nullptr;
    }
}

void Bno08xHandler::QuaternionToEuler(const Bno08xQuaternion& quaternion, Bno08xEulerAngles& euler_angles) noexcept {
    if (!quaternion.valid) {
        euler_angles.valid = false;
        return;
    }
    
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    float w = quaternion.w;
    float x = quaternion.x;
    float y = quaternion.y;
    float z = quaternion.z;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    euler_angles.roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1) {
        euler_angles.pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    } else {
        euler_angles.pitch = std::asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    euler_angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    
    euler_angles.accuracy = quaternion.accuracy;
    euler_angles.timestamp_us = quaternion.timestamp_us;
    euler_angles.valid = true;
}

Bno08xConfig Bno08xHandler::GetDefaultConfig() noexcept {
    Bno08xConfig config;
    
    // Enable basic sensors
    config.enable_accelerometer = true;
    config.enable_gyroscope = true;
    config.enable_magnetometer = true;
    config.enable_rotation_vector = true;
    config.enable_linear_acceleration = false;
    config.enable_gravity = false;
    config.enable_game_rotation = false;
    
    // Disable activity detection by default
    config.enable_tap_detector = false;
    config.enable_step_counter = false;
    config.enable_shake_detector = false;
    config.enable_pickup_detector = false;
    config.enable_significant_motion = false;
    config.enable_activity_classifier = false;
    
    // Set reasonable intervals
    config.accelerometer_interval_ms = 50;   // 20 Hz
    config.gyroscope_interval_ms = 50;       // 20 Hz
    config.magnetometer_interval_ms = 100;   // 10 Hz
    config.rotation_interval_ms = 50;        // 20 Hz
    config.linear_accel_interval_ms = 50;    // 20 Hz
    config.gravity_interval_ms = 100;        // 10 Hz
    
    // Default interface
    config.interface_type = BNO085Interface::I2C;
    
    // Calibration settings
    config.auto_calibration = true;
    config.calibration_timeout_s = 60.0f;
    
    return config;
}

const char* Bno08xHandler::GetDescription() const noexcept {
    return description_;
}

Bno08xError Bno08xHandler::GetLastError() const noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    return lock.IsLocked() ? last_error_ : Bno08xError::MUTEX_LOCK_FAILED;
}

BNO085Interface Bno08xHandler::GetInterfaceType() const noexcept {
    return interface_type_;
}

//======================================================//
// PRIVATE HELPER METHODS
//======================================================//

bool Bno08xHandler::ValidateSensor() const noexcept {
    return bno08x_sensor_ != nullptr && transport_adapter_ != nullptr;
}

bool Bno08xHandler::ApplyConfiguration(const Bno08xConfig& config) noexcept {
    if (!ValidateSensor()) return false;
    
    // Enable sensors based on configuration
    if (config.enable_accelerometer) {
        EnableSensor(BNO085Sensor::Accelerometer, config.accelerometer_interval_ms);
    }
    
    if (config.enable_gyroscope) {
        EnableSensor(BNO085Sensor::Gyroscope, config.gyroscope_interval_ms);
    }
    
    if (config.enable_magnetometer) {
        EnableSensor(BNO085Sensor::Magnetometer, config.magnetometer_interval_ms);
    }
    
    if (config.enable_rotation_vector) {
        EnableSensor(BNO085Sensor::RotationVector, config.rotation_interval_ms);
    }
    
    if (config.enable_linear_acceleration) {
        EnableSensor(BNO085Sensor::LinearAcceleration, config.linear_accel_interval_ms);
    }
    
    if (config.enable_gravity) {
        EnableSensor(BNO085Sensor::Gravity, config.gravity_interval_ms);
    }
    
    // Enable activity detection
    if (config.enable_tap_detector) {
        EnableSensor(BNO085Sensor::TapDetector, 0);
    }
    
    if (config.enable_step_counter) {
        EnableSensor(BNO085Sensor::StepCounter, 0);
    }
    
    if (config.enable_shake_detector) {
        EnableSensor(BNO085Sensor::ShakeDetector, 0);
    }
    
    return true;
}

void Bno08xHandler::HandleSensorEvent(const SensorEvent& event) noexcept {
    // Forward to user callback if set
    if (sensor_callback_) {
        sensor_callback_(event);
    }
    
    // Internal event processing can be added here
}

uint32_t Bno08xHandler::getTimeUs() const noexcept {
    return transport_adapter_ ? transport_adapter_->getTimeUs() : 0;
}

//======================================================//
// FACTORY METHODS
//======================================================//

std::unique_ptr<Bno08xHandler> CreateBno08xHandlerI2c(
    BaseI2c& i2c_interface,
    const Bno08xConfig& config,
    BaseGpio* reset_gpio,
    BaseGpio* int_gpio) noexcept {
    
    return std::make_unique<Bno08xHandler>(i2c_interface, config, reset_gpio, int_gpio);
}

std::unique_ptr<Bno08xHandler> CreateBno08xHandlerSpi(
    BaseSpi& spi_interface,
    const Bno08xConfig& config,
    BaseGpio* wake_gpio,
    BaseGpio* reset_gpio,
    BaseGpio* int_gpio) noexcept {
    
    return std::make_unique<Bno08xHandler>(spi_interface, config, wake_gpio, reset_gpio, int_gpio);
}

void Bno08xHandler::DumpDiagnostics() const noexcept {
    static constexpr const char* TAG = "Bno08xHandler";
    
    Logger::GetInstance().Info(TAG, "=== BNO08X HANDLER DIAGNOSTICS ===");
    
    RtosMutex::LockGuard lock(handler_mutex_);
    
    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", initialized_ ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Last Error: %s", 
        last_error_ == Bno08xError::SUCCESS ? "SUCCESS" :
        last_error_ == Bno08xError::NOT_INITIALIZED ? "NOT_INITIALIZED" :
        last_error_ == Bno08xError::COMMUNICATION_ERROR ? "COMMUNICATION_ERROR" :
        last_error_ == Bno08xError::SENSOR_ERROR ? "SENSOR_ERROR" :
        last_error_ == Bno08xError::INVALID_PARAMETER ? "INVALID_PARAMETER" :
        last_error_ == Bno08xError::TIMEOUT ? "TIMEOUT" : "UNKNOWN");
    
    // Interface Type
    Logger::GetInstance().Info(TAG, "Communication Interface:");
    if (i2c_adapter_) {
        Logger::GetInstance().Info(TAG, "  Type: I2C");
        Logger::GetInstance().Info(TAG, "  I2C Adapter: ACTIVE");
    } else if (spi_adapter_) {
        Logger::GetInstance().Info(TAG, "  Type: SPI");
        Logger::GetInstance().Info(TAG, "  SPI Adapter: ACTIVE");
    } else {
        Logger::GetInstance().Info(TAG, "  Type: NONE");
    }
    
    // GPIO Status
    Logger::GetInstance().Info(TAG, "GPIO Configuration:");
    Logger::GetInstance().Info(TAG, "  Reset GPIO: %s", reset_gpio_ ? "CONFIGURED" : "NOT_CONFIGURED");
    Logger::GetInstance().Info(TAG, "  Interrupt GPIO: %s", int_gpio_ ? "CONFIGURED" : "NOT_CONFIGURED");
    Logger::GetInstance().Info(TAG, "  Wake GPIO: %s", wake_gpio_ ? "CONFIGURED" : "NOT_CONFIGURED");
    
    // Sensor Configuration
    Logger::GetInstance().Info(TAG, "Sensor Configuration:");
    Logger::GetInstance().Info(TAG, "  Description: %s", description_);
    Logger::GetInstance().Info(TAG, "  Sample Rate: %d Hz", config_.sample_rate_hz);
    Logger::GetInstance().Info(TAG, "  Enable Game Rotation: %s", config_.enable_game_rotation ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Accelerometer: %s", config_.enable_accelerometer ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Gyroscope: %s", config_.enable_gyroscope ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Magnetometer: %s", config_.enable_magnetometer ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Linear Accel: %s", config_.enable_linear_acceleration ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Gravity: %s", config_.enable_gravity ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Rotation Vector: %s", config_.enable_rotation_vector ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Step Counter: %s", config_.enable_step_counter ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Tap Detection: %s", config_.enable_tap_detection ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Shake Detection: %s", config_.enable_shake_detection ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Stability: %s", config_.enable_stability_detection ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Enable Activity: %s", config_.enable_activity_classification ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Auto Calibrate: %s", config_.auto_calibrate ? "YES" : "NO");
    
    // Driver Status
    Logger::GetInstance().Info(TAG, "BNO08x Driver:");
    if (bno08x_sensor_) {
        Logger::GetInstance().Info(TAG, "  Driver Instance: ACTIVE");
        
        // Try to get sensor information if available
        // Note: Add more BNO08x-specific diagnostics here based on driver capabilities
    } else {
        Logger::GetInstance().Info(TAG, "  Driver Instance: NOT_INITIALIZED");
    }
    
    // Diagnostics Information
    Logger::GetInstance().Info(TAG, "Sensor Diagnostics:");
    Logger::GetInstance().Info(TAG, "  Sensor Healthy: %s", diagnostics_.sensor_healthy ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Communication OK: %s", diagnostics_.communication_ok ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Calibration OK: %s", diagnostics_.calibration_ok ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Data Available: %s", diagnostics_.data_available ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Communication Errors: %d", diagnostics_.communication_errors);
    Logger::GetInstance().Info(TAG, "  Sensor Errors: %d", diagnostics_.sensor_errors);
    Logger::GetInstance().Info(TAG, "  Total Measurements: %d", diagnostics_.total_measurements);
    Logger::GetInstance().Info(TAG, "  Last Error Code: 0x%04X", diagnostics_.last_error_code);
    
    // Calibration Status
    Logger::GetInstance().Info(TAG, "Calibration Status:");
    Logger::GetInstance().Info(TAG, "  Accel Calibrated: %s", calibration_status_.accelerometer_calibrated ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Gyro Calibrated: %s", calibration_status_.gyroscope_calibrated ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Mag Calibrated: %s", calibration_status_.magnetometer_calibrated ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  System Calibrated: %s", calibration_status_.system_calibrated ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Accel Accuracy: %d", calibration_status_.accelerometer_accuracy);
    Logger::GetInstance().Info(TAG, "  Gyro Accuracy: %d", calibration_status_.gyroscope_accuracy);
    Logger::GetInstance().Info(TAG, "  Mag Accuracy: %d", calibration_status_.magnetometer_accuracy);
    Logger::GetInstance().Info(TAG, "  System Accuracy: %d", calibration_status_.system_accuracy);
    
    // Performance Metrics
    if (diagnostics_.total_measurements > 0) {
        float error_rate = (float)(diagnostics_.communication_errors + diagnostics_.sensor_errors) / 
                          diagnostics_.total_measurements * 100.0f;
        Logger::GetInstance().Info(TAG, "Performance Metrics:");
        Logger::GetInstance().Info(TAG, "  Error Rate: %.2f%%", error_rate);
        Logger::GetInstance().Info(TAG, "  Success Rate: %.2f%%", 100.0f - error_rate);
    }
    
    // Memory Usage
    Logger::GetInstance().Info(TAG, "Memory Usage:");
    size_t estimated_memory = sizeof(*this);
    if (bno08x_sensor_) estimated_memory += sizeof(BNO085);
    if (i2c_adapter_) estimated_memory += sizeof(Bno08xI2cAdapter);
    if (spi_adapter_) estimated_memory += sizeof(Bno08xSpiAdapter);
    Logger::GetInstance().Info(TAG, "  Estimated Total: %d bytes", static_cast<int>(estimated_memory));
    
    // System Status Summary
    bool system_healthy = initialized_ && 
                         (last_error_ == Bno08xError::SUCCESS) &&
                         diagnostics_.sensor_healthy &&
                         diagnostics_.communication_ok;
    
    Logger::GetInstance().Info(TAG, "System Status: %s", system_healthy ? "HEALTHY" : "DEGRADED");
    
    Logger::GetInstance().Info(TAG, "=== END BNO08X HANDLER DIAGNOSTICS ===");
}
