/**
 * @file Bno08xHandler.cpp
 * @brief Implementation of unified BNO08x IMU sensor handler.
 *
 * This implementation provides:
 * - CRTP communication adapters bridging BaseI2c/BaseSpi to bno08x::CommInterface
 * - Complete handler lifecycle (Initialize/Deinitialize)
 * - SH-2 sensor data reading for all IMU axes, quaternions, Euler angles
 * - Activity/gesture detection (tap, step, shake, pickup, stability)
 * - Calibration status monitoring
 * - Sensor enable/disable with configurable intervals
 * - Hardware control (reset, boot, wake pins)
 * - Thread-safe operations with recursive mutex
 * - Comprehensive diagnostics
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "Bno08xHandler.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include "handlers/Logger.h"

// SH-2 error codes for mapping
extern "C" {
#include "core/hf-core-drivers/external/hf-bno08x-driver/src/sh2/sh2_err.h"
}

// ============================================================================
//  I2C CRTP ADAPTER IMPLEMENTATION
// ============================================================================

bool HalI2cBno08xComm::Open() noexcept {
    // Initialize GPIO pins if provided
    if (reset_gpio_) {
        reset_gpio_->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
        // RSTN is active-low: ACTIVE = assert reset (drive LOW)
        reset_gpio_->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW);
        reset_gpio_->EnsureInitialized();
        // Start with reset released (INACTIVE = HIGH)
        reset_gpio_->SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
    }

    if (int_gpio_) {
        int_gpio_->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
        int_gpio_->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
        // INT is active-low: ACTIVE = data available (pin LOW)
        int_gpio_->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW);
        int_gpio_->EnsureInitialized();
    }

    // Ensure the I2C bus is initialized
    return i2c_.EnsureInitialized();
}

void HalI2cBno08xComm::Close() noexcept {
    // Don't deinitialize the I2C bus - it may be shared with other devices
}

int HalI2cBno08xComm::Write(const uint8_t* data, uint32_t length) noexcept {
    if (!data || length == 0) return -1;

    hf_i2c_err_t result = i2c_.Write(data, static_cast<hf_u16_t>(length));
    if (result == hf_i2c_err_t::I2C_SUCCESS) {
        return static_cast<int>(length);
    }
    return -1;
}

int HalI2cBno08xComm::Read(uint8_t* data, uint32_t length) noexcept {
    if (!data || length == 0) return -1;

    // Check INT pin first if available (configured active-low: IsActive() = data ready)
    if (int_gpio_) {
        bool is_active = false;
        if (int_gpio_->IsActive(is_active) == hf_gpio_err_t::GPIO_SUCCESS) {
            if (!is_active) {
                return 0;  // No data ready
            }
        }
    }

    hf_i2c_err_t result = i2c_.Read(data, static_cast<hf_u16_t>(length));
    if (result == hf_i2c_err_t::I2C_SUCCESS) {
        return static_cast<int>(length);
    }
    return -1;
}

bool HalI2cBno08xComm::DataAvailable() noexcept {
    if (!int_gpio_) return true;  // Assume data available if no INT pin

    // INT is active-low, configured as such: IsActive() = data available
    bool is_active = false;
    if (int_gpio_->IsActive(is_active) == hf_gpio_err_t::GPIO_SUCCESS) {
        return is_active;
    }
    return true;  // Assume available on read error
}

void HalI2cBno08xComm::Delay(uint32_t ms) noexcept {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t HalI2cBno08xComm::GetTimeUs() noexcept {
    return static_cast<uint32_t>(RtosTime::GetCurrentTimeUs());
}

void HalI2cBno08xComm::SetReset(bool state) noexcept {
    if (!reset_gpio_) return;
    // RSTN is active-low: state=true means assert reset (drive LOW)
    // Using active-low GPIO: ACTIVE = LOW, INACTIVE = HIGH
    if (state) {
        reset_gpio_->SetState(hf_gpio_state_t::HF_GPIO_STATE_ACTIVE);
    } else {
        reset_gpio_->SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
    }
}

void HalI2cBno08xComm::SetBoot(bool /*state*/) noexcept {
    // Boot pin control not typically wired for I2C
}

void HalI2cBno08xComm::SetWake(bool /*state*/) noexcept {
    // Wake pin not used for I2C
}

void HalI2cBno08xComm::SetPS0(bool /*state*/) noexcept {
    // PS pins are typically hard-wired
}

void HalI2cBno08xComm::SetPS1(bool /*state*/) noexcept {
    // PS pins are typically hard-wired
}

// ============================================================================
//  SPI CRTP ADAPTER IMPLEMENTATION
// ============================================================================

bool HalSpiBno08xComm::Open() noexcept {
    // Initialize GPIO pins if provided
    if (reset_gpio_) {
        reset_gpio_->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
        reset_gpio_->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW);
        reset_gpio_->EnsureInitialized();
        reset_gpio_->SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
    }

    if (int_gpio_) {
        int_gpio_->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
        int_gpio_->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP);
        int_gpio_->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW);
        int_gpio_->EnsureInitialized();
    }

    if (wake_gpio_) {
        wake_gpio_->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
        // WAKE is active-low: ACTIVE = assert wake (drive LOW)
        wake_gpio_->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW);
        wake_gpio_->EnsureInitialized();
        // Start with wake released (INACTIVE = HIGH)
        wake_gpio_->SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
    }

    // Ensure the SPI bus is initialized
    return spi_.EnsureInitialized();
}

void HalSpiBno08xComm::Close() noexcept {
    // Don't deinitialize the SPI bus - it may be shared
}

int HalSpiBno08xComm::Write(const uint8_t* data, uint32_t length) noexcept {
    if (!data || length == 0) return -1;

    hf_spi_err_t result = spi_.Write(data, static_cast<hf_u16_t>(length), 100);
    if (result == hf_spi_err_t::SPI_SUCCESS) {
        return static_cast<int>(length);
    }
    return -1;
}

int HalSpiBno08xComm::Read(uint8_t* data, uint32_t length) noexcept {
    if (!data || length == 0) return -1;

    // Check INT pin first if available
    if (int_gpio_) {
        bool is_active = false;
        if (int_gpio_->IsActive(is_active) == hf_gpio_err_t::GPIO_SUCCESS) {
            if (!is_active) {
                return 0;
            }
        }
    }

    hf_spi_err_t result = spi_.Read(data, static_cast<hf_u16_t>(length), 100);
    if (result == hf_spi_err_t::SPI_SUCCESS) {
        return static_cast<int>(length);
    }
    return -1;
}

bool HalSpiBno08xComm::DataAvailable() noexcept {
    if (!int_gpio_) return true;

    bool is_active = false;
    if (int_gpio_->IsActive(is_active) == hf_gpio_err_t::GPIO_SUCCESS) {
        return is_active;
    }
    return true;
}

void HalSpiBno08xComm::Delay(uint32_t ms) noexcept {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t HalSpiBno08xComm::GetTimeUs() noexcept {
    return static_cast<uint32_t>(RtosTime::GetCurrentTimeUs());
}

void HalSpiBno08xComm::SetReset(bool state) noexcept {
    if (!reset_gpio_) return;
    if (state) {
        reset_gpio_->SetState(hf_gpio_state_t::HF_GPIO_STATE_ACTIVE);
    } else {
        reset_gpio_->SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
    }
}

void HalSpiBno08xComm::SetBoot(bool /*state*/) noexcept {
    // Boot pin not typically wired
}

void HalSpiBno08xComm::SetWake(bool state) noexcept {
    if (!wake_gpio_) return;
    // WAKE is active-low: state=true means assert (drive LOW)
    if (state) {
        wake_gpio_->SetState(hf_gpio_state_t::HF_GPIO_STATE_ACTIVE);
    } else {
        wake_gpio_->SetState(hf_gpio_state_t::HF_GPIO_STATE_INACTIVE);
    }
}

void HalSpiBno08xComm::SetPS0(bool /*state*/) noexcept {
    // PS pins typically hard-wired
}

void HalSpiBno08xComm::SetPS1(bool /*state*/) noexcept {
    // PS pins typically hard-wired
}

// ============================================================================
//  BNO08X HANDLER IMPLEMENTATION
// ============================================================================

// --- Constructors ---

Bno08xHandler::Bno08xHandler(BaseI2c& i2c_device,
                             const Bno08xConfig& config,
                             BaseGpio* reset_gpio,
                             BaseGpio* int_gpio) noexcept
    : driver_ops_(std::make_unique<Bno08xDriverImpl<HalI2cBno08xComm>>(
          HalI2cBno08xComm(i2c_device, reset_gpio, int_gpio)))
    , config_(config)
    , interface_type_(BNO085Interface::I2C) {
    std::snprintf(description_, sizeof(description_),
                  "BNO08x IMU (I2C @0x%02X)",
                  static_cast<unsigned>(i2c_device.GetDeviceAddress()));
}

Bno08xHandler::Bno08xHandler(BaseSpi& spi_device,
                             const Bno08xConfig& config,
                             BaseGpio* reset_gpio,
                             BaseGpio* int_gpio,
                             BaseGpio* wake_gpio) noexcept
    : driver_ops_(std::make_unique<Bno08xDriverImpl<HalSpiBno08xComm>>(
          HalSpiBno08xComm(spi_device, reset_gpio, int_gpio, wake_gpio)))
    , config_(config)
    , interface_type_(BNO085Interface::SPI) {
    std::snprintf(description_, sizeof(description_),
                  "BNO08x IMU (SPI)");
}

// --- Initialization ---

Bno08xError Bno08xHandler::Initialize() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        last_error_ = Bno08xError::MUTEX_LOCK_FAILED;
        return last_error_;
    }

    if (initialized_) {
        last_error_ = Bno08xError::SUCCESS;
        return last_error_;
    }

    if (!driver_ops_) {
        last_error_ = Bno08xError::INITIALIZATION_FAILED;
        return last_error_;
    }

    // Perform hardware reset via the driver (uses RSTN pin if wired)
    driver_ops_->HardwareReset(10);

    // Initialize the SH-2 protocol
    if (!driver_ops_->Begin()) {
        last_error_ = Bno08xError::SENSOR_NOT_RESPONDING;
        return last_error_;
    }

    // Set internal callback that forwards to user callback
    driver_ops_->SetCallback([this](const SensorEvent& event) {
        // Forward to user callback (no extra lock needed - called from Update()
        // which already holds the mutex, and the recursive mutex allows re-entry)
        if (user_callback_) {
            user_callback_(event);
        }
    });

    // Apply initial sensor configuration
    if (!applyConfigLocked(config_)) {
        // Non-fatal: sensor is still initialized, some sensors may have failed
        last_error_ = Bno08xError::COMMUNICATION_FAILED;
        initialized_ = true;
        return last_error_;
    }

    initialized_ = true;
    last_error_ = Bno08xError::SUCCESS;
    return last_error_;
}

Bno08xError Bno08xHandler::Deinitialize() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        last_error_ = Bno08xError::MUTEX_LOCK_FAILED;
        return last_error_;
    }

    if (!initialized_) {
        last_error_ = Bno08xError::SUCCESS;
        return last_error_;
    }

    // Clear the driver callback
    if (driver_ops_) {
        driver_ops_->SetCallback(nullptr);
    }

    user_callback_ = nullptr;
    initialized_ = false;
    last_error_ = Bno08xError::SUCCESS;
    return last_error_;
}

bool Bno08xHandler::IsInitialized() const noexcept {
    MutexLockGuard lock(handler_mutex_);
    return lock.IsLocked() && initialized_;
}

// --- Update ---

Bno08xError Bno08xHandler::Update() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        last_error_ = Bno08xError::MUTEX_LOCK_FAILED;
        return last_error_;
    }

    if (!initialized_ || !driver_ops_) {
        last_error_ = Bno08xError::NOT_INITIALIZED;
        return last_error_;
    }

    // Pump the SH-2 service loop (dispatches callbacks internally)
    driver_ops_->Update();

    // Check for driver errors
    int driver_error = driver_ops_->GetLastError();
    if (driver_error != 0) {
        last_error_ = mapDriverError(driver_error);
    } else {
        last_error_ = Bno08xError::SUCCESS;
    }

    return last_error_;
}

bool Bno08xHandler::HasNewData(BNO085Sensor sensor) const noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked() || !initialized_ || !driver_ops_) {
        return false;
    }
    return driver_ops_->HasNewData(sensor);
}

// ============================================================================
//  SENSOR DATA READING
// ============================================================================

Bno08xError Bno08xHandler::readVectorSensor(BNO085Sensor sensor,
                                             Bno08xVector3& out) const noexcept {
    SensorEvent event = driver_ops_->GetLatest(sensor);
    out.x = event.vector.x;
    out.y = event.vector.y;
    out.z = event.vector.z;
    out.accuracy = event.vector.accuracy;
    out.timestamp_us = event.timestamp;
    out.valid = true;
    return Bno08xError::SUCCESS;
}

Bno08xError Bno08xHandler::ReadAcceleration(Bno08xVector3& acceleration) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;
    return readVectorSensor(BNO085Sensor::Accelerometer, acceleration);
}

Bno08xError Bno08xHandler::ReadGyroscope(Bno08xVector3& gyroscope) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;
    return readVectorSensor(BNO085Sensor::Gyroscope, gyroscope);
}

Bno08xError Bno08xHandler::ReadMagnetometer(Bno08xVector3& magnetometer) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;
    return readVectorSensor(BNO085Sensor::Magnetometer, magnetometer);
}

Bno08xError Bno08xHandler::ReadLinearAcceleration(Bno08xVector3& linear_accel) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;
    return readVectorSensor(BNO085Sensor::LinearAcceleration, linear_accel);
}

Bno08xError Bno08xHandler::ReadGravity(Bno08xVector3& gravity) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;
    return readVectorSensor(BNO085Sensor::Gravity, gravity);
}

Bno08xError Bno08xHandler::ReadQuaternion(Bno08xQuaternion& quaternion) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;

    SensorEvent event = driver_ops_->GetLatest(BNO085Sensor::RotationVector);
    quaternion.w = event.rotation.w;
    quaternion.x = event.rotation.x;
    quaternion.y = event.rotation.y;
    quaternion.z = event.rotation.z;
    quaternion.accuracy = event.rotation.accuracy;
    quaternion.timestamp_us = event.timestamp;
    quaternion.valid = true;

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

Bno08xError Bno08xHandler::ReadImuData(Bno08xImuData& imu_data) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;

    // Read all available sensor data (no extra mutex needed, already locked)
    readVectorSensor(BNO085Sensor::Accelerometer, imu_data.acceleration);
    readVectorSensor(BNO085Sensor::Gyroscope, imu_data.gyroscope);
    readVectorSensor(BNO085Sensor::Magnetometer, imu_data.magnetometer);
    readVectorSensor(BNO085Sensor::LinearAcceleration, imu_data.linear_acceleration);
    readVectorSensor(BNO085Sensor::Gravity, imu_data.gravity);

    // Read rotation quaternion
    SensorEvent rv_event = driver_ops_->GetLatest(BNO085Sensor::RotationVector);
    imu_data.rotation.w = rv_event.rotation.w;
    imu_data.rotation.x = rv_event.rotation.x;
    imu_data.rotation.y = rv_event.rotation.y;
    imu_data.rotation.z = rv_event.rotation.z;
    imu_data.rotation.accuracy = rv_event.rotation.accuracy;
    imu_data.rotation.timestamp_us = rv_event.timestamp;
    imu_data.rotation.valid = true;

    // Derive Euler angles from quaternion
    QuaternionToEuler(imu_data.rotation, imu_data.euler);

    imu_data.timestamp_us = rv_event.timestamp;
    imu_data.valid = true;

    return Bno08xError::SUCCESS;
}

// ============================================================================
//  ACTIVITY AND GESTURE DETECTION
// ============================================================================

Bno08xError Bno08xHandler::ReadActivityData(Bno08xActivityData& activity_data) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;

    // Read tap detector
    SensorEvent tap_event = driver_ops_->GetLatest(BNO085Sensor::TapDetector);
    activity_data.tap_detected = tap_event.detected;
    activity_data.double_tap = tap_event.tap.doubleTap;
    activity_data.tap_direction = tap_event.tap.direction;

    // Read step counter
    SensorEvent step_event = driver_ops_->GetLatest(BNO085Sensor::StepCounter);
    activity_data.step_count = step_event.stepCount;

    // Read step detector
    SensorEvent step_det = driver_ops_->GetLatest(BNO085Sensor::StepDetector);
    activity_data.step_detected = step_det.detected;

    // Read shake detector
    SensorEvent shake_event = driver_ops_->GetLatest(BNO085Sensor::ShakeDetector);
    activity_data.shake_detected = shake_event.detected;

    // Read pickup detector
    SensorEvent pickup_event = driver_ops_->GetLatest(BNO085Sensor::PickupDetector);
    activity_data.pickup_detected = pickup_event.detected;

    // Read significant motion
    SensorEvent motion_event = driver_ops_->GetLatest(BNO085Sensor::SignificantMotion);
    activity_data.significant_motion = motion_event.detected;

    // Read stability classifier
    SensorEvent stability_event = driver_ops_->GetLatest(BNO085Sensor::StabilityClassifier);
    activity_data.stability_class = static_cast<uint8_t>(stability_event.vector.x);

    activity_data.timestamp_us = tap_event.timestamp;
    activity_data.valid = true;

    return Bno08xError::SUCCESS;
}

// ============================================================================
//  CALIBRATION STATUS
// ============================================================================

Bno08xError Bno08xHandler::ReadCalibrationStatus(Bno08xCalibrationStatus& status) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;

    // Accuracy is embedded in each sensor report's accuracy field
    SensorEvent accel = driver_ops_->GetLatest(BNO085Sensor::Accelerometer);
    SensorEvent gyro = driver_ops_->GetLatest(BNO085Sensor::Gyroscope);
    SensorEvent mag = driver_ops_->GetLatest(BNO085Sensor::Magnetometer);
    SensorEvent rv = driver_ops_->GetLatest(BNO085Sensor::RotationVector);

    status.accelerometer_accuracy = accel.vector.accuracy;
    status.gyroscope_accuracy = gyro.vector.accuracy;
    status.magnetometer_accuracy = mag.vector.accuracy;
    status.rotation_accuracy = rv.rotation.accuracy;

    // Consider fully calibrated if all core sensors have accuracy >= 2
    status.fully_calibrated =
        (status.accelerometer_accuracy >= 2) &&
        (status.gyroscope_accuracy >= 2) &&
        (status.magnetometer_accuracy >= 2);

    return Bno08xError::SUCCESS;
}

// ============================================================================
//  SENSOR CONFIGURATION
// ============================================================================

Bno08xError Bno08xHandler::EnableSensor(BNO085Sensor sensor, uint32_t interval_ms,
                                         float sensitivity) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;

    if (driver_ops_->EnableSensor(sensor, interval_ms, sensitivity)) {
        return Bno08xError::SUCCESS;
    }

    int driver_error = driver_ops_->GetLastError();
    last_error_ = mapDriverError(driver_error);
    return last_error_;
}

Bno08xError Bno08xHandler::DisableSensor(BNO085Sensor sensor) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;

    if (driver_ops_->DisableSensor(sensor)) {
        return Bno08xError::SUCCESS;
    }

    int driver_error = driver_ops_->GetLastError();
    last_error_ = mapDriverError(driver_error);
    return last_error_;
}

Bno08xError Bno08xHandler::ApplyConfiguration(const Bno08xConfig& config) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!initialized_ || !driver_ops_) return Bno08xError::NOT_INITIALIZED;

    config_ = config;
    if (applyConfigLocked(config)) {
        return Bno08xError::SUCCESS;
    }
    return Bno08xError::COMMUNICATION_FAILED;
}

const Bno08xConfig& Bno08xHandler::GetConfiguration() const noexcept {
    return config_;
}

bool Bno08xHandler::applyConfigLocked(const Bno08xConfig& config) noexcept {
    bool all_ok = true;

    // Helper lambda: enable sensor if flag is true, disable otherwise
    auto configureSensor = [&](bool enable, BNO085Sensor sensor,
                               uint32_t interval_ms, float sensitivity = 0.0f) {
        if (enable) {
            if (!driver_ops_->EnableSensor(sensor, interval_ms, sensitivity)) {
                all_ok = false;
            }
        }
    };

    // Core sensors
    configureSensor(config.enable_accelerometer,
                    BNO085Sensor::Accelerometer, config.accelerometer_interval_ms);
    configureSensor(config.enable_gyroscope,
                    BNO085Sensor::Gyroscope, config.gyroscope_interval_ms);
    configureSensor(config.enable_magnetometer,
                    BNO085Sensor::Magnetometer, config.magnetometer_interval_ms);
    configureSensor(config.enable_rotation_vector,
                    BNO085Sensor::RotationVector, config.rotation_interval_ms);
    configureSensor(config.enable_linear_acceleration,
                    BNO085Sensor::LinearAcceleration, config.linear_accel_interval_ms);
    configureSensor(config.enable_gravity,
                    BNO085Sensor::Gravity, config.gravity_interval_ms);
    configureSensor(config.enable_game_rotation,
                    BNO085Sensor::GameRotationVector, config.game_rotation_interval_ms);

    // Event/activity sensors (interval 0 = on-change)
    configureSensor(config.enable_tap_detector,
                    BNO085Sensor::TapDetector, 0);
    configureSensor(config.enable_step_counter,
                    BNO085Sensor::StepCounter, 0);
    configureSensor(config.enable_step_detector,
                    BNO085Sensor::StepDetector, 0);
    configureSensor(config.enable_shake_detector,
                    BNO085Sensor::ShakeDetector, 0);
    configureSensor(config.enable_pickup_detector,
                    BNO085Sensor::PickupDetector, 0);
    configureSensor(config.enable_significant_motion,
                    BNO085Sensor::SignificantMotion, 0);
    configureSensor(config.enable_stability_classifier,
                    BNO085Sensor::StabilityClassifier, 0);

    return all_ok;
}

// ============================================================================
//  HARDWARE CONTROL
// ============================================================================

Bno08xError Bno08xHandler::HardwareReset(uint32_t reset_duration_ms) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!driver_ops_) return Bno08xError::NOT_INITIALIZED;

    driver_ops_->HardwareReset(reset_duration_ms);
    return Bno08xError::SUCCESS;
}

Bno08xError Bno08xHandler::SetBootPin(bool boot_state) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!driver_ops_) return Bno08xError::NOT_INITIALIZED;

    driver_ops_->SetBootPin(boot_state);
    return Bno08xError::SUCCESS;
}

Bno08xError Bno08xHandler::SetWakePin(bool wake_state) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) return Bno08xError::MUTEX_LOCK_FAILED;
    if (!driver_ops_) return Bno08xError::NOT_INITIALIZED;

    driver_ops_->SetWakePin(wake_state);
    return Bno08xError::SUCCESS;
}

// ============================================================================
//  CALLBACK MANAGEMENT
// ============================================================================

void Bno08xHandler::SetSensorCallback(SensorCallback callback) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (lock.IsLocked()) {
        user_callback_ = std::move(callback);
    }
}

void Bno08xHandler::ClearSensorCallback() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (lock.IsLocked()) {
        user_callback_ = nullptr;
    }
}

// ============================================================================
//  UTILITY METHODS
// ============================================================================

void Bno08xHandler::QuaternionToEuler(const Bno08xQuaternion& quaternion,
                                       Bno08xEulerAngles& euler_angles) noexcept {
    if (!quaternion.valid) {
        euler_angles.valid = false;
        return;
    }

    float w = quaternion.w;
    float x = quaternion.x;
    float y = quaternion.y;
    float z = quaternion.z;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    euler_angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    if (std::abs(sinp) >= 1.0f) {
        euler_angles.pitch = std::copysign(static_cast<float>(M_PI) / 2.0f, sinp);
    } else {
        euler_angles.pitch = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    euler_angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    euler_angles.accuracy = quaternion.accuracy;
    euler_angles.timestamp_us = quaternion.timestamp_us;
    euler_angles.valid = true;
}

BNO085Interface Bno08xHandler::GetInterfaceType() const noexcept {
    return interface_type_;
}

Bno08xError Bno08xHandler::GetLastError() const noexcept {
    MutexLockGuard lock(handler_mutex_);
    return lock.IsLocked() ? last_error_ : Bno08xError::MUTEX_LOCK_FAILED;
}

int Bno08xHandler::GetLastDriverError() const noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked() || !driver_ops_) return -1;
    return driver_ops_->GetLastError();
}

const char* Bno08xHandler::GetDescription() const noexcept {
    return description_;
}

Bno08xError Bno08xHandler::mapDriverError(int sh2_error) noexcept {
    switch (sh2_error) {
        case 0:  return Bno08xError::SUCCESS;            // SH2_OK
        case -2: return Bno08xError::INVALID_PARAMETER;  // SH2_ERR_BAD_PARAM
        case -4: return Bno08xError::COMMUNICATION_FAILED; // SH2_ERR_IO
        case -5: return Bno08xError::SENSOR_NOT_RESPONDING; // SH2_ERR_HUB
        case -6: return Bno08xError::TIMEOUT;            // SH2_ERR_TIMEOUT
        default: return Bno08xError::COMMUNICATION_FAILED; // SH2_ERR
    }
}

// ============================================================================
//  DIAGNOSTICS
// ============================================================================

void Bno08xHandler::DumpDiagnostics() const noexcept {
    static constexpr const char* TAG = "Bno08xHandler";

    Logger::GetInstance().Info(TAG, "=== BNO08X HANDLER DIAGNOSTICS ===");

    MutexLockGuard lock(handler_mutex_);
    if (!lock.IsLocked()) {
        Logger::GetInstance().Info(TAG, "  Failed to acquire mutex for diagnostics");
        return;
    }

    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", initialized_ ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Last Error: %s", Bno08xErrorToString(last_error_));
    Logger::GetInstance().Info(TAG, "  Description: %s", description_);

    // Interface
    const char* iface_str = "UNKNOWN";
    switch (interface_type_) {
        case BNO085Interface::I2C:     iface_str = "I2C"; break;
        case BNO085Interface::SPI:     iface_str = "SPI"; break;
        case BNO085Interface::UART:    iface_str = "UART"; break;
        case BNO085Interface::UARTRVC: iface_str = "UART-RVC"; break;
    }
    Logger::GetInstance().Info(TAG, "  Interface: %s", iface_str);

    // Driver status
    Logger::GetInstance().Info(TAG, "Driver:");
    if (driver_ops_) {
        Logger::GetInstance().Info(TAG, "  Instance: ACTIVE");
        Logger::GetInstance().Info(TAG, "  Last SH2 Error: %d", driver_ops_->GetLastError());
    } else {
        Logger::GetInstance().Info(TAG, "  Instance: NOT_CREATED");
    }

    // Sensor Configuration
    Logger::GetInstance().Info(TAG, "Sensor Configuration:");
    Logger::GetInstance().Info(TAG, "  Accelerometer: %s (%lu ms)",
        config_.enable_accelerometer ? "ON" : "OFF",
        static_cast<unsigned long>(config_.accelerometer_interval_ms));
    Logger::GetInstance().Info(TAG, "  Gyroscope: %s (%lu ms)",
        config_.enable_gyroscope ? "ON" : "OFF",
        static_cast<unsigned long>(config_.gyroscope_interval_ms));
    Logger::GetInstance().Info(TAG, "  Magnetometer: %s (%lu ms)",
        config_.enable_magnetometer ? "ON" : "OFF",
        static_cast<unsigned long>(config_.magnetometer_interval_ms));
    Logger::GetInstance().Info(TAG, "  Rotation Vector: %s (%lu ms)",
        config_.enable_rotation_vector ? "ON" : "OFF",
        static_cast<unsigned long>(config_.rotation_interval_ms));
    Logger::GetInstance().Info(TAG, "  Linear Accel: %s (%lu ms)",
        config_.enable_linear_acceleration ? "ON" : "OFF",
        static_cast<unsigned long>(config_.linear_accel_interval_ms));
    Logger::GetInstance().Info(TAG, "  Gravity: %s (%lu ms)",
        config_.enable_gravity ? "ON" : "OFF",
        static_cast<unsigned long>(config_.gravity_interval_ms));
    Logger::GetInstance().Info(TAG, "  Game Rotation: %s (%lu ms)",
        config_.enable_game_rotation ? "ON" : "OFF",
        static_cast<unsigned long>(config_.game_rotation_interval_ms));

    // Activity sensors
    Logger::GetInstance().Info(TAG, "Activity Sensors:");
    Logger::GetInstance().Info(TAG, "  Tap Detector: %s",
        config_.enable_tap_detector ? "ON" : "OFF");
    Logger::GetInstance().Info(TAG, "  Step Counter: %s",
        config_.enable_step_counter ? "ON" : "OFF");
    Logger::GetInstance().Info(TAG, "  Shake Detector: %s",
        config_.enable_shake_detector ? "ON" : "OFF");
    Logger::GetInstance().Info(TAG, "  Pickup Detector: %s",
        config_.enable_pickup_detector ? "ON" : "OFF");
    Logger::GetInstance().Info(TAG, "  Significant Motion: %s",
        config_.enable_significant_motion ? "ON" : "OFF");
    Logger::GetInstance().Info(TAG, "  Stability Classifier: %s",
        config_.enable_stability_classifier ? "ON" : "OFF");

    // Calibration status (if initialized)
    if (initialized_ && driver_ops_) {
        SensorEvent accel = driver_ops_->GetLatest(BNO085Sensor::Accelerometer);
        SensorEvent gyro = driver_ops_->GetLatest(BNO085Sensor::Gyroscope);
        SensorEvent mag = driver_ops_->GetLatest(BNO085Sensor::Magnetometer);
        SensorEvent rv = driver_ops_->GetLatest(BNO085Sensor::RotationVector);

        Logger::GetInstance().Info(TAG, "Calibration Accuracy:");
        Logger::GetInstance().Info(TAG, "  Accelerometer: %u/3", accel.vector.accuracy);
        Logger::GetInstance().Info(TAG, "  Gyroscope: %u/3", gyro.vector.accuracy);
        Logger::GetInstance().Info(TAG, "  Magnetometer: %u/3", mag.vector.accuracy);
        Logger::GetInstance().Info(TAG, "  Rotation Vector: %u/3", rv.rotation.accuracy);
    }

    // Callback status
    Logger::GetInstance().Info(TAG, "Callback: %s",
        user_callback_ ? "REGISTERED" : "NONE");

    // Memory estimate
    size_t estimated_memory = sizeof(*this);
    Logger::GetInstance().Info(TAG, "Estimated Memory: %u bytes",
        static_cast<unsigned>(estimated_memory));

    Logger::GetInstance().Info(TAG, "=== END BNO08X HANDLER DIAGNOSTICS ===");
}
