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
    
    try {
        // Open transport interface
        if (!transport_adapter_->open()) {
            last_error_ = Bno08xError::COMMUNICATION_FAILED;
            return last_error_;
        }
        
        // Create BNO08x sensor instance with transport
        bno08x_sensor_ = std::make_shared<BNO085>(*transport_adapter_);
        
        if (!bno08x_sensor_) {
            last_error_ = Bno08xError::INITIALIZATION_FAILED;
            return last_error_;
        }
        
        // Perform hardware reset if GPIO available
        if (reset_gpio_) {
            HardwareReset(10);
            RtosTask::Delay(100); // Wait for reset to complete
        }
        
        // Initialize sensor
        if (!bno08x_sensor_->initialize()) {
            last_error_ = Bno08xError::SENSOR_NOT_RESPONDING;
            return last_error_;
        }
        
        // Apply initial configuration
        if (!ApplyConfiguration(config_)) {
            last_error_ = Bno08xError::INITIALIZATION_FAILED;
            return last_error_;
        }
        
        // Set internal callback for sensor events
        bno08x_sensor_->setCallback([this](const SensorEvent& event) {
            this->HandleSensorEvent(event);
        });
        
        initialized_ = true;
        last_error_ = Bno08xError::SUCCESS;
        
    } catch (...) {
        last_error_ = Bno08xError::INITIALIZATION_FAILED;
        bno08x_sensor_.reset();
        transport_adapter_->close();
    }
    
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
    
    try {
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
        
    } catch (...) {
        last_error_ = Bno08xError::HARDWARE_ERROR;
    }
    
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
    
    try {
        // Process any available sensor data
        bno08x_sensor_->process();
        last_error_ = Bno08xError::SUCCESS;
        
    } catch (...) {
        last_error_ = Bno08xError::COMMUNICATION_FAILED;
    }
    
    return last_error_;
}

Bno08xError Bno08xHandler::ReadImuData(Bno08xImuData& imu_data) noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        return Bno08xError::MUTEX_LOCK_FAILED;
    }
    
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    try {
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
        imu_data.timestamp_us = getTimeUs();
        imu_data.valid = true;
        
        return Bno08xError::SUCCESS;
        
    } catch (...) {
        imu_data.valid = false;
        return Bno08xError::COMMUNICATION_FAILED;
    }
}

Bno08xError Bno08xHandler::ReadAcceleration(Bno08xVector3& acceleration) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    try {
        Vector3 accel = bno08x_sensor_->getAcceleration();
        acceleration.x = accel.x;
        acceleration.y = accel.y;
        acceleration.z = accel.z;
        acceleration.accuracy = bno08x_sensor_->getAccelerationAccuracy();
        acceleration.timestamp_us = getTimeUs();
        acceleration.valid = true;
        
        return Bno08xError::SUCCESS;
        
    } catch (...) {
        acceleration.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
}

Bno08xError Bno08xHandler::ReadGyroscope(Bno08xVector3& gyroscope) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    try {
        Vector3 gyro = bno08x_sensor_->getGyroscope();
        gyroscope.x = gyro.x;
        gyroscope.y = gyro.y;
        gyroscope.z = gyro.z;
        gyroscope.accuracy = bno08x_sensor_->getGyroscopeAccuracy();
        gyroscope.timestamp_us = getTimeUs();
        gyroscope.valid = true;
        
        return Bno08xError::SUCCESS;
        
    } catch (...) {
        gyroscope.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
}

Bno08xError Bno08xHandler::ReadMagnetometer(Bno08xVector3& magnetometer) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    try {
        Vector3 mag = bno08x_sensor_->getMagnetometer();
        magnetometer.x = mag.x;
        magnetometer.y = mag.y;
        magnetometer.z = mag.z;
        magnetometer.accuracy = bno08x_sensor_->getMagnetometerAccuracy();
        magnetometer.timestamp_us = getTimeUs();
        magnetometer.valid = true;
        
        return Bno08xError::SUCCESS;
        
    } catch (...) {
        magnetometer.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
}

Bno08xError Bno08xHandler::ReadQuaternion(Bno08xQuaternion& quaternion) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    try {
        Quaternion quat = bno08x_sensor_->getQuaternion();
        quaternion.w = quat.w;
        quaternion.x = quat.x;
        quaternion.y = quat.y;
        quaternion.z = quat.z;
        quaternion.accuracy = bno08x_sensor_->getQuaternionAccuracy();
        quaternion.timestamp_us = getTimeUs();
        quaternion.valid = true;
        
        return Bno08xError::SUCCESS;
        
    } catch (...) {
        quaternion.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
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
    
    try {
        Vector3 lin_accel = bno08x_sensor_->getLinearAcceleration();
        linear_accel.x = lin_accel.x;
        linear_accel.y = lin_accel.y;
        linear_accel.z = lin_accel.z;
        linear_accel.accuracy = bno08x_sensor_->getLinearAccelerationAccuracy();
        linear_accel.timestamp_us = getTimeUs();
        linear_accel.valid = true;
        
        return Bno08xError::SUCCESS;
        
    } catch (...) {
        linear_accel.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
}

Bno08xError Bno08xHandler::ReadGravity(Bno08xVector3& gravity) noexcept {
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    try {
        Vector3 grav = bno08x_sensor_->getGravity();
        gravity.x = grav.x;
        gravity.y = grav.y;
        gravity.z = grav.z;
        gravity.accuracy = bno08x_sensor_->getGravityAccuracy();
        gravity.timestamp_us = getTimeUs();
        gravity.valid = true;
        
        return Bno08xError::SUCCESS;
        
    } catch (...) {
        gravity.valid = false;
        return Bno08xError::DATA_NOT_AVAILABLE;
    }
}

Bno08xError Bno08xHandler::EnableSensor(BNO085Sensor sensor, uint32_t interval_ms, float sensitivity) noexcept {
    RtosMutex::LockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        return Bno08xError::MUTEX_LOCK_FAILED;
    }
    
    if (!initialized_ || !ValidateSensor()) {
        return Bno08xError::NOT_INITIALIZED;
    }
    
    try {
        if (bno08x_sensor_->enableReport(sensor, interval_ms, sensitivity)) {
            return Bno08xError::SUCCESS;
        } else {
            return Bno08xError::INVALID_PARAMETER;
        }
        
    } catch (...) {
        return Bno08xError::COMMUNICATION_FAILED;
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
    
    try {
        if (bno08x_sensor_->disableReport(sensor)) {
            return Bno08xError::SUCCESS;
        } else {
            return Bno08xError::INVALID_PARAMETER;
        }
        
    } catch (...) {
        return Bno08xError::COMMUNICATION_FAILED;
    }
}

Bno08xError Bno08xHandler::HardwareReset(uint32_t reset_duration_ms) noexcept {
    if (!reset_gpio_) {
        return Bno08xError::HARDWARE_ERROR;
    }
    
    try {
        // Hold reset low
        reset_gpio_->SetValue(BaseGpio::Value::LOW);
        RtosTask::Delay(reset_duration_ms);
        
        // Release reset
        reset_gpio_->SetValue(BaseGpio::Value::HIGH);
        RtosTask::Delay(100); // Allow time for reset to complete
        
        return Bno08xError::SUCCESS;
        
    } catch (...) {
        return Bno08xError::HARDWARE_ERROR;
    }
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
    
    try {
        // Enable sensors based on configuration
        if (config.enable_accelerometer) {
            EnableSensor(BNO085Sensor::ACCELEROMETER, config.accelerometer_interval_ms);
        }
        
        if (config.enable_gyroscope) {
            EnableSensor(BNO085Sensor::GYROSCOPE_CALIBRATED, config.gyroscope_interval_ms);
        }
        
        if (config.enable_magnetometer) {
            EnableSensor(BNO085Sensor::MAGNETIC_FIELD_CALIBRATED, config.magnetometer_interval_ms);
        }
        
        if (config.enable_rotation_vector) {
            EnableSensor(BNO085Sensor::ROTATION_VECTOR, config.rotation_interval_ms);
        }
        
        if (config.enable_linear_acceleration) {
            EnableSensor(BNO085Sensor::LINEAR_ACCELERATION, config.linear_accel_interval_ms);
        }
        
        if (config.enable_gravity) {
            EnableSensor(BNO085Sensor::GRAVITY, config.gravity_interval_ms);
        }
        
        // Enable activity detection
        if (config.enable_tap_detector) {
            EnableSensor(BNO085Sensor::TAP_DETECTOR, 0);
        }
        
        if (config.enable_step_counter) {
            EnableSensor(BNO085Sensor::STEP_COUNTER, 0);
        }
        
        if (config.enable_shake_detector) {
            EnableSensor(BNO085Sensor::SHAKE_DETECTOR, 0);
        }
        
        return true;
        
    } catch (...) {
        return false;
    }
}

void Bno08xHandler::HandleSensorEvent(const SensorEvent& event) noexcept {
    try {
        // Forward to user callback if set
        if (sensor_callback_) {
            sensor_callback_(event);
        }
        
        // Internal event processing can be added here
        
    } catch (...) {
        // Ignore callback exceptions
    }
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
