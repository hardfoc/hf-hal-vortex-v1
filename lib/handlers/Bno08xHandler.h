/**
 * @file Bno08xHandler.h
 * @brief Unified handler for BNO08x IMU sensor family with multi-interface support.
 *
 * @details
 * This file provides the complete HAL-level integration for a BNO08x IMU sensor.
 * It bridges the HardFOC base interfaces (BaseI2c, BaseSpi, BaseGpio) with the
 * templated BNO085<CommType> driver from the hf-bno08x-driver library.
 *
 * ## Architecture Overview
 *
 * The file contains four layers:
 *
 * 1. **CRTP Communication Adapters** (HalI2cBno08xComm, HalSpiBno08xComm):
 *    Bridge HardFOC BaseI2c/BaseSpi device-centric interfaces with the BNO08x
 *    driver's CRTP-based bno08x::CommInterface. Includes all 12 required methods
 *    (bus I/O, timing, and 5 pin control signals).
 *
 * 2. **Type-Erased Driver Interface** (IBno08xDriverOps):
 *    Internal abstract class that hides the BNO085<CommType> template from the
 *    handler. Provides virtual methods for all driver operations. The one-vtable
 *    cost is negligible for an IMU updating at ~100 Hz.
 *
 * 3. **Template Driver Wrapper** (Bno08xDriverImpl<CommType>):
 *    Owns both the CRTP comm adapter and the BNO085 driver instance, implementing
 *    IBno08xDriverOps by delegating to the real driver.
 *
 * 4. **Bno08xHandler** (main class):
 *    Non-templated facade that owns a type-erased driver via unique_ptr.
 *    Provides:
 *    - Lazy initialization with hardware reset sequence
 *    - Complete SH-2 sensor data access (9-DOF fusion, gestures, activity)
 *    - Callback management for event-driven operation
 *    - Individual sensor enable/disable with configurable intervals
 *    - Hardware control (reset, boot, wake pins)
 *    - Thread-safe operations with RtosMutex protection
 *    - Comprehensive diagnostics
 *
 * ## Ownership Model
 *
 * The handler owns all its internal resources:
 * - One IBno08xDriverOps (type-erased driver + comm adapter)
 *
 * ## Initialization Sequence
 *
 * @code
 * // 1. Obtain a BaseI2c device reference (address pre-configured, e.g. 0x4A)
 * // 2. Construct the handler
 * Bno08xHandler handler(i2c_device);
 *
 * // 3. Initialize
 * if (handler.Initialize() != Bno08xError::SUCCESS) { return; }
 *
 * // 4. Enable sensors
 * handler.EnableSensor(BNO085Sensor::RotationVector, 10);  // 100 Hz
 * handler.EnableSensor(BNO085Sensor::Accelerometer, 20);   // 50 Hz
 *
 * // 5. Set callback for event-driven data
 * handler.SetSensorCallback([](const SensorEvent& e) { ... });
 *
 * // 6. Service loop
 * while (true) { handler.Update(); }
 * @endcode
 *
 * ## Thread Safety
 *
 * The handler uses RtosMutex (recursive) for thread-safe access to all
 * handler-level operations.
 *
 * @see BNO085             Templated driver from hf-bno08x-driver
 * @see bno08x::CommInterface  CRTP communication base class
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#ifndef COMPONENT_HANDLER_BNO08X_HANDLER_H_
#define COMPONENT_HANDLER_BNO08X_HANDLER_H_

#include <cstdint>
#include <memory>
#include <array>
#include <functional>
#include "core/hf-core-drivers/external/hf-bno08x-driver/inc/bno08x.hpp"
#include "core/hf-core-drivers/external/hf-bno08x-driver/inc/bno08x_comm_interface.hpp"
#include "base/BaseI2c.h"
#include "base/BaseSpi.h"
#include "base/BaseGpio.h"
#include "utils/RtosMutex.h"

// ============================================================================
//  BNO08X ERROR CODES
// ============================================================================

/**
 * @brief BNO08x handler error codes for consistent error reporting.
 */
enum class Bno08xError : uint8_t {
    SUCCESS = 0,                ///< Operation completed successfully
    NOT_INITIALIZED,            ///< Handler or sensor not initialized
    INITIALIZATION_FAILED,      ///< Begin() or config apply failed
    INVALID_PARAMETER,          ///< Bad argument (e.g. invalid sensor)
    COMMUNICATION_FAILED,       ///< I2C/SPI or SH-2 I/O error
    SENSOR_NOT_RESPONDING,      ///< Hub/sensor did not respond
    CALIBRATION_FAILED,         ///< Calibration request failed
    FIRMWARE_UPDATE_FAILED,     ///< DFU transfer failed
    TIMEOUT,                    ///< Operation timed out
    MUTEX_LOCK_FAILED,          ///< Could not acquire handler mutex
    INVALID_INTERFACE,          ///< Wrong interface type for operation
    SENSOR_NOT_ENABLED,         ///< Sensor not enabled for read
    DATA_NOT_AVAILABLE,         ///< No new data for requested sensor
    HARDWARE_ERROR,             ///< GPIO or hardware fault
    UNSUPPORTED_OPERATION       ///< Feature not supported
};

/**
 * @brief Convert Bno08xError to string for debugging.
 */
constexpr const char* Bno08xErrorToString(Bno08xError error) noexcept {
    switch (error) {
        case Bno08xError::SUCCESS:                return "Success";
        case Bno08xError::NOT_INITIALIZED:        return "Not initialized";
        case Bno08xError::INITIALIZATION_FAILED:   return "Initialization failed";
        case Bno08xError::INVALID_PARAMETER:       return "Invalid parameter";
        case Bno08xError::COMMUNICATION_FAILED:    return "Communication failed";
        case Bno08xError::SENSOR_NOT_RESPONDING:   return "Sensor not responding";
        case Bno08xError::CALIBRATION_FAILED:      return "Calibration failed";
        case Bno08xError::FIRMWARE_UPDATE_FAILED:  return "Firmware update failed";
        case Bno08xError::TIMEOUT:                 return "Timeout";
        case Bno08xError::MUTEX_LOCK_FAILED:       return "Mutex lock failed";
        case Bno08xError::INVALID_INTERFACE:       return "Invalid interface";
        case Bno08xError::SENSOR_NOT_ENABLED:      return "Sensor not enabled";
        case Bno08xError::DATA_NOT_AVAILABLE:      return "Data not available";
        case Bno08xError::HARDWARE_ERROR:          return "Hardware error";
        case Bno08xError::UNSUPPORTED_OPERATION:   return "Unsupported operation";
        default: return "Unknown error";
    }
}

// ============================================================================
//  CRTP COMMUNICATION ADAPTERS
// ============================================================================

/**
 * @brief I2C CRTP communication adapter for BNO08x.
 *
 * Bridges the HardFOC BaseI2c interface with the bno08x::CommInterface CRTP base.
 * The I2C device address is pre-configured on the BaseI2c instance.
 */
class HalI2cBno08xComm : public bno08x::CommInterface<HalI2cBno08xComm> {
public:
    /**
     * @brief Construct I2C adapter.
     * @param i2c Reference to BaseI2c implementation (address pre-configured)
     * @param reset_gpio Optional GPIO for hardware reset (RSTN, active-low)
     * @param int_gpio Optional GPIO for interrupt monitoring (INT, active-low)
     */
    explicit HalI2cBno08xComm(BaseI2c& i2c,
                              BaseGpio* reset_gpio = nullptr,
                              BaseGpio* int_gpio = nullptr) noexcept
        : i2c_(i2c), reset_gpio_(reset_gpio), int_gpio_(int_gpio) {}

    // -- bno08x::CommInterface required methods --

    /** @return BNO085Interface::I2C */
    BNO085Interface GetInterfaceType() noexcept { return BNO085Interface::I2C; }

    /** Initialize I2C bus and optional RSTN/INT GPIOs. @return true on success */
    bool Open() noexcept;
    /** Release bus (no-op; bus may be shared). */
    void Close() noexcept;
    /** @param data Buffer to send. @param length Byte count. @return Bytes written or -1 on error */
    int  Write(const uint8_t* data, uint32_t length) noexcept;
    /** @param data Buffer to fill. @param length Max bytes. @return Bytes read, 0 if no data, -1 on error */
    int  Read(uint8_t* data, uint32_t length) noexcept;
    /** Check INT pin (active-low). @return true if data ready or no INT pin */
    bool DataAvailable() noexcept;
    /** Block for @p ms milliseconds. @param ms Delay in ms */
    void Delay(uint32_t ms) noexcept;
    /** @return Monotonic time in microseconds */
    uint32_t GetTimeUs() noexcept;

    /** RSTN pin: true = assert reset (LOW), false = release. @param state Assert reset */
    void SetReset(bool state) noexcept;
    /** BOOTN pin for DFU (no-op if not wired). @param state true = LOW */
    void SetBoot(bool state) noexcept;
    /** WAKE pin (I2C: no-op). @param state true = drive LOW */
    void SetWake(bool state) noexcept;
    /** PS0 protocol-select (no-op if hard-wired). @param state Logic level */
    void SetPS0(bool state) noexcept;
    /** PS1 protocol-select (no-op if hard-wired). @param state Logic level */
    void SetPS1(bool state) noexcept;

private:
    BaseI2c& i2c_;
    BaseGpio* reset_gpio_;
    BaseGpio* int_gpio_;
};

/**
 * @brief SPI CRTP communication adapter for BNO08x.
 *
 * Bridges the HardFOC BaseSpi interface with the bno08x::CommInterface CRTP base.
 */
class HalSpiBno08xComm : public bno08x::CommInterface<HalSpiBno08xComm> {
public:
    /**
     * @brief Construct SPI adapter.
     * @param spi Reference to BaseSpi implementation
     * @param reset_gpio Optional GPIO for hardware reset (RSTN, active-low)
     * @param int_gpio Optional GPIO for interrupt monitoring (INT, active-low)
     * @param wake_gpio Optional GPIO for wake control (SPI mode)
     */
    explicit HalSpiBno08xComm(BaseSpi& spi,
                              BaseGpio* reset_gpio = nullptr,
                              BaseGpio* int_gpio = nullptr,
                              BaseGpio* wake_gpio = nullptr) noexcept
        : spi_(spi), reset_gpio_(reset_gpio), int_gpio_(int_gpio), wake_gpio_(wake_gpio) {}

    // -- bno08x::CommInterface required methods --

    /** @return BNO085Interface::SPI */
    BNO085Interface GetInterfaceType() noexcept { return BNO085Interface::SPI; }

    /** Initialize SPI bus and optional RSTN/INT/WAKE GPIOs. @return true on success */
    bool Open() noexcept;
    /** Release bus (no-op; bus may be shared). */
    void Close() noexcept;
    /** @param data Buffer to send. @param length Byte count. @return Bytes written or -1 on error */
    int  Write(const uint8_t* data, uint32_t length) noexcept;
    /** @param data Buffer to fill. @param length Max bytes. @return Bytes read, 0 if no data, -1 on error */
    int  Read(uint8_t* data, uint32_t length) noexcept;
    /** Check INT pin (active-low). @return true if data ready or no INT pin */
    bool DataAvailable() noexcept;
    /** Block for @p ms milliseconds. @param ms Delay in ms */
    void Delay(uint32_t ms) noexcept;
    /** @return Monotonic time in microseconds */
    uint32_t GetTimeUs() noexcept;

    /** RSTN pin: true = assert reset (LOW), false = release. @param state Assert reset */
    void SetReset(bool state) noexcept;
    /** BOOTN pin for DFU (no-op if not wired). @param state true = LOW */
    void SetBoot(bool state) noexcept;
    /** WAKE pin: true = drive LOW (wake sensor). @param state Assert wake */
    void SetWake(bool state) noexcept;
    /** PS0 protocol-select (no-op if hard-wired). @param state Logic level */
    void SetPS0(bool state) noexcept;
    /** PS1 protocol-select (no-op if hard-wired). @param state Logic level */
    void SetPS1(bool state) noexcept;

private:
    BaseSpi& spi_;
    BaseGpio* reset_gpio_;
    BaseGpio* int_gpio_;
    BaseGpio* wake_gpio_;
};

// ============================================================================
//  TYPE-ERASED DRIVER INTERFACE (internal)
// ============================================================================

/**
 * @brief Internal abstract interface for type-erasing the BNO085<CommType> template.
 *
 * This enables the handler to work with both I2C and SPI transports
 * without exposing template parameters to handler users.
 */
class IBno08xDriverOps {
public:
    virtual ~IBno08xDriverOps() = default;

    virtual bool Begin() noexcept = 0;
    virtual void Update() noexcept = 0;
    virtual bool EnableSensor(BNO085Sensor sensor, uint32_t interval_ms,
                              float sensitivity) noexcept = 0;
    virtual bool DisableSensor(BNO085Sensor sensor) noexcept = 0;
    virtual void SetCallback(SensorCallback cb) noexcept = 0;
    virtual bool HasNewData(BNO085Sensor sensor) const noexcept = 0;
    virtual SensorEvent GetLatest(BNO085Sensor sensor) const noexcept = 0;
    virtual int  GetLastError() const noexcept = 0;
    virtual void HardwareReset(uint32_t lowMs) noexcept = 0;
    virtual void SetBootPin(bool state) noexcept = 0;
    virtual void SetWakePin(bool state) noexcept = 0;
    virtual void SelectInterface(BNO085Interface iface) noexcept = 0;
    virtual BNO085Interface GetInterfaceType() noexcept = 0;
};

/**
 * @brief Concrete type-erased driver wrapper.
 *
 * Owns both the CRTP comm adapter (as a member) and the BNO085 driver instance.
 * The comm adapter is declared first to ensure it outlives the driver (which
 * stores a reference to it).
 *
 * @tparam CommType  The CRTP communication adapter type.
 */
template <typename CommType>
class Bno08xDriverImpl : public IBno08xDriverOps {
public:
    /**
     * @brief Construct with a pre-built comm adapter (moved in).
     */
    explicit Bno08xDriverImpl(CommType&& comm) noexcept
        : comm_(std::move(comm)), driver_(comm_) {}

    bool Begin() noexcept override { return driver_.Begin(); }
    void Update() noexcept override { driver_.Update(); }

    bool EnableSensor(BNO085Sensor sensor, uint32_t interval_ms,
                      float sensitivity) noexcept override {
        return driver_.EnableSensor(sensor, interval_ms, sensitivity);
    }

    bool DisableSensor(BNO085Sensor sensor) noexcept override {
        return driver_.DisableSensor(sensor);
    }

    void SetCallback(SensorCallback cb) noexcept override {
        driver_.SetCallback(std::move(cb));
    }

    bool HasNewData(BNO085Sensor sensor) const noexcept override {
        return driver_.HasNewData(sensor);
    }

    SensorEvent GetLatest(BNO085Sensor sensor) const noexcept override {
        return driver_.GetLatest(sensor);
    }

    int GetLastError() const noexcept override {
        return driver_.GetLastError();
    }

    void HardwareReset(uint32_t lowMs) noexcept override {
        driver_.HardwareReset(lowMs);
    }

    void SetBootPin(bool state) noexcept override {
        driver_.SetBootPin(state);
    }

    void SetWakePin(bool state) noexcept override {
        driver_.SetWakePin(state);
    }

    void SelectInterface(BNO085Interface iface) noexcept override {
        driver_.SelectInterface(iface);
    }

    BNO085Interface GetInterfaceType() noexcept override {
        return comm_.GetInterfaceType();
    }

private:
    CommType comm_;                  ///< CRTP comm adapter (must outlive driver_)
    BNO085<CommType> driver_;        ///< BNO085 driver instance
};

// ============================================================================
//  SENSOR DATA STRUCTURES
// ============================================================================

/**
 * @brief Enhanced vector with timestamp and accuracy.
 */
struct Bno08xVector3 {
    float x{0};                  ///< X component
    float y{0};                  ///< Y component
    float z{0};                  ///< Z component
    uint8_t accuracy{0};         ///< Sensor accuracy (0-3)
    uint64_t timestamp_us{0};    ///< Timestamp in microseconds
    bool valid{false};           ///< Data validity flag
};

/**
 * @brief Enhanced quaternion with timestamp and accuracy.
 */
struct Bno08xQuaternion {
    float w{1};                  ///< Real component
    float x{0};                  ///< i component
    float y{0};                  ///< j component
    float z{0};                  ///< k component
    uint8_t accuracy{0};         ///< Sensor accuracy (0-3)
    uint64_t timestamp_us{0};    ///< Timestamp in microseconds
    bool valid{false};           ///< Data validity flag
};

/**
 * @brief Euler angles derived from quaternion.
 */
struct Bno08xEulerAngles {
    float roll{0};               ///< Roll angle in radians
    float pitch{0};              ///< Pitch angle in radians
    float yaw{0};                ///< Yaw angle in radians
    uint8_t accuracy{0};         ///< Derived accuracy
    uint64_t timestamp_us{0};    ///< Timestamp in microseconds
    bool valid{false};           ///< Data validity flag
};

/**
 * @brief Complete IMU sensor data structure.
 */
struct Bno08xImuData {
    Bno08xVector3 acceleration;        ///< Calibrated acceleration (m/s^2)
    Bno08xVector3 gyroscope;           ///< Calibrated angular velocity (rad/s)
    Bno08xVector3 magnetometer;        ///< Calibrated magnetic field (uT)
    Bno08xVector3 linear_acceleration; ///< Linear acceleration (no gravity)
    Bno08xVector3 gravity;             ///< Gravity vector
    Bno08xQuaternion rotation;         ///< Orientation quaternion
    Bno08xEulerAngles euler;           ///< Euler angles
    uint64_t timestamp_us{0};          ///< Overall timestamp
    bool valid{false};                 ///< Overall validity flag
};

/**
 * @brief Activity and gesture detection data.
 */
struct Bno08xActivityData {
    bool tap_detected{false};        ///< Single/double tap detected
    bool double_tap{false};          ///< True if double tap
    uint8_t tap_direction{0};        ///< Tap direction (0-5)
    uint32_t step_count{0};          ///< Total step count
    bool step_detected{false};       ///< New step detected
    bool shake_detected{false};      ///< Shake gesture detected
    bool pickup_detected{false};     ///< Pickup gesture detected
    bool significant_motion{false};  ///< Significant motion detected
    uint8_t stability_class{0};      ///< Stability classification
    uint64_t timestamp_us{0};        ///< Timestamp
    bool valid{false};               ///< Data validity
};

/**
 * @brief Sensor calibration status.
 */
struct Bno08xCalibrationStatus {
    uint8_t accelerometer_accuracy{0};  ///< Accelerometer calibration (0-3)
    uint8_t gyroscope_accuracy{0};      ///< Gyroscope calibration (0-3)
    uint8_t magnetometer_accuracy{0};   ///< Magnetometer calibration (0-3)
    uint8_t rotation_accuracy{0};       ///< Rotation vector accuracy (0-3)
    bool fully_calibrated{false};       ///< True if all sensors at accuracy >= 2
};

/**
 * @brief BNO08x configuration structure.
 *
 * Used by ApplyConfiguration() and passed to constructors. Interval values
 * are in milliseconds; use 0 for on-change sensors (tap, step, etc.).
 */
struct Bno08xConfig {
    // Sensor enable flags
    bool enable_accelerometer{true};       ///< Calibrated acceleration (m/s^2)
    bool enable_gyroscope{true};           ///< Calibrated angular velocity (rad/s)
    bool enable_magnetometer{true};        ///< Calibrated magnetic field (uT)
    bool enable_rotation_vector{true};    ///< Fused orientation quaternion
    bool enable_linear_acceleration{false}; ///< Acceleration minus gravity
    bool enable_gravity{false};            ///< Gravity vector
    bool enable_game_rotation{false};      ///< Orientation without magnetometer

    // Activity detection (interval 0 = on-change)
    bool enable_tap_detector{false};      ///< Single/double tap events
    bool enable_step_counter{false};       ///< Cumulative step count
    bool enable_step_detector{false};      ///< Per-step events
    bool enable_shake_detector{false};     ///< Shake gesture
    bool enable_pickup_detector{false};    ///< Pickup gesture
    bool enable_significant_motion{false}; ///< Significant motion event
    bool enable_stability_classifier{false}; ///< Stability state (on table, etc.)

    // Sensor intervals (milliseconds, 0 = on-change for event sensors)
    uint32_t accelerometer_interval_ms{50};     ///< 20 Hz default
    uint32_t gyroscope_interval_ms{50};         ///< 20 Hz default
    uint32_t magnetometer_interval_ms{100};     ///< 10 Hz default
    uint32_t rotation_interval_ms{50};          ///< 20 Hz default
    uint32_t linear_accel_interval_ms{50};      ///< 20 Hz default
    uint32_t gravity_interval_ms{100};          ///< 10 Hz default
    uint32_t game_rotation_interval_ms{50};     ///< 20 Hz default
};

// ============================================================================
//  BNO08X HANDLER CLASS
// ============================================================================

/**
 * @brief Unified handler for BNO08x IMU sensor family.
 *
 * Provides a comprehensive, non-templated interface for BNO08x sensors with:
 * - Lazy initialization pattern
 * - Exception-free design with noexcept methods
 * - Thread-safe operations with recursive mutex protection
 * - Bridge pattern integration with BaseI2c/BaseSpi via CRTP adapters
 * - Type erasure to hide template complexity from users
 * - Complete SH-2 mode sensor access (9-DOF, gestures, activity)
 * - Configurable sensor enable/disable and interval management
 * - Hardware control (reset, boot, wake, interface selection)
 */
class Bno08xHandler {
public:
    // ========================================================================
    //  CONSTRUCTION AND LIFECYCLE
    // ========================================================================

    /**
     * @brief Construct BNO08x handler with I2C interface.
     * @param i2c_device Reference to BaseI2c implementation (address pre-configured)
     * @param config Initial sensor configuration
     * @param reset_gpio Optional GPIO for hardware reset
     * @param int_gpio Optional GPIO for interrupt monitoring
     */
    explicit Bno08xHandler(BaseI2c& i2c_device,
                           const Bno08xConfig& config = Bno08xConfig{},
                           BaseGpio* reset_gpio = nullptr,
                           BaseGpio* int_gpio = nullptr) noexcept;

    /**
     * @brief Construct BNO08x handler with SPI interface.
     * @param spi_device Reference to BaseSpi implementation
     * @param config Initial sensor configuration
     * @param reset_gpio Optional GPIO for hardware reset
     * @param int_gpio Optional GPIO for interrupt monitoring
     * @param wake_gpio Optional GPIO for wake control (SPI mode)
     */
    explicit Bno08xHandler(BaseSpi& spi_device,
                           const Bno08xConfig& config = Bno08xConfig{},
                           BaseGpio* reset_gpio = nullptr,
                           BaseGpio* int_gpio = nullptr,
                           BaseGpio* wake_gpio = nullptr) noexcept;

    ~Bno08xHandler() noexcept = default;

    // Non-copyable
    Bno08xHandler(const Bno08xHandler&) = delete;
    Bno08xHandler& operator=(const Bno08xHandler&) = delete;

    // Movable
    Bno08xHandler(Bno08xHandler&&) noexcept = default;
    Bno08xHandler& operator=(Bno08xHandler&&) noexcept = default;

    // ========================================================================
    //  INITIALIZATION AND STATUS
    // ========================================================================

    /**
     * @brief Initialize the BNO08x sensor.
     *
     * Performs hardware reset (if GPIO available), calls Begin() on the driver,
     * and applies the initial sensor configuration.
     *
     * @return Bno08xError::SUCCESS if successful, error code otherwise
     */
    Bno08xError Initialize() noexcept;

    /**
     * @brief Deinitialize the sensor.
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError Deinitialize() noexcept;

    /**
     * @brief Check if sensor is initialized and ready.
     * @return true if Initialize() succeeded and driver is ready
     */
    bool IsInitialized() const noexcept;

    /**
     * @brief Update sensor - must be called regularly to pump the SH-2 service loop.
     *
     * Call this every 5-10 ms (or faster) for optimal data throughput.
     * Sensor callbacks are dispatched from within this method.
     *
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError Update() noexcept;

    /**
     * @brief Check if new data is available for a specific sensor.
     * @param sensor Sensor to check
     * @return True if new unread data is available
     */
    bool HasNewData(BNO085Sensor sensor) const noexcept;

    // ========================================================================
    //  SENSOR DATA READING
    // ========================================================================

    /**
     * @brief Read complete IMU data structure.
     *
     * Aggregates latest data from all enabled core sensors (accel, gyro, mag,
     * linear accel, gravity, rotation vector) into a single structure.
     *
     * @param imu_data Output structure for IMU data
     * @return Bno08xError::SUCCESS if at least some data was populated
     */
    Bno08xError ReadImuData(Bno08xImuData& imu_data) noexcept;

    /**
     * @brief Read calibrated acceleration vector (m/s^2).
     * @param acceleration Output: x,y,z, accuracy, timestamp, valid
     * @return Bno08xError::SUCCESS on success
     */
    Bno08xError ReadAcceleration(Bno08xVector3& acceleration) noexcept;

    /**
     * @brief Read calibrated gyroscope data (rad/s).
     * @param gyroscope Output: x,y,z, accuracy, timestamp, valid
     * @return Bno08xError::SUCCESS on success
     */
    Bno08xError ReadGyroscope(Bno08xVector3& gyroscope) noexcept;

    /**
     * @brief Read calibrated magnetometer data (uT).
     * @param magnetometer Output: x,y,z, accuracy, timestamp, valid
     * @return Bno08xError::SUCCESS on success
     */
    Bno08xError ReadMagnetometer(Bno08xVector3& magnetometer) noexcept;

    /**
     * @brief Read orientation quaternion (rotation vector).
     * @param quaternion Output: w,x,y,z, accuracy, timestamp, valid
     * @return Bno08xError::SUCCESS on success
     */
    Bno08xError ReadQuaternion(Bno08xQuaternion& quaternion) noexcept;

    /**
     * @brief Read Euler angles in radians (derived from rotation vector).
     * @param euler_angles Output: roll, pitch, yaw, accuracy, timestamp, valid
     * @return Bno08xError::SUCCESS on success
     */
    Bno08xError ReadEulerAngles(Bno08xEulerAngles& euler_angles) noexcept;

    /**
     * @brief Read linear acceleration (m/s^2, gravity removed).
     * @param linear_accel Output: x,y,z, accuracy, timestamp, valid
     * @return Bno08xError::SUCCESS on success
     */
    Bno08xError ReadLinearAcceleration(Bno08xVector3& linear_accel) noexcept;

    /**
     * @brief Read gravity vector (m/s^2).
     * @param gravity Output: x,y,z, accuracy, timestamp, valid
     * @return Bno08xError::SUCCESS on success
     */
    Bno08xError ReadGravity(Bno08xVector3& gravity) noexcept;

    // ========================================================================
    //  ACTIVITY AND GESTURE DETECTION
    // ========================================================================

    /**
     * @brief Read activity and gesture detection data.
     *
     * Aggregates latest tap, step, shake, pickup, and stability data.
     * Only populated for sensors that are enabled.
     * @param activity_data Output: tap, step count, shake, pickup, etc.
     * @return Bno08xError::SUCCESS on success
     */
    Bno08xError ReadActivityData(Bno08xActivityData& activity_data) noexcept;

    // ========================================================================
    //  CALIBRATION STATUS
    // ========================================================================

    /**
     * @brief Read calibration status from latest sensor accuracy values.
     *
     * Accuracy is embedded in each sensor report (0=unreliable, 3=calibrated).
     * This method reads the latest accuracy values from the most recently
     * received reports.
     * @param status Output: per-sensor accuracy (0-3) and fully_calibrated flag
     * @return Bno08xError::SUCCESS on success
     */
    Bno08xError ReadCalibrationStatus(Bno08xCalibrationStatus& status) noexcept;

    // ========================================================================
    //  SENSOR CONFIGURATION
    // ========================================================================

    /**
     * @brief Enable a specific sensor with the given interval.
     * @param sensor Sensor to enable
     * @param interval_ms Reporting interval in milliseconds (0 = on-change)
     * @param sensitivity Change sensitivity (for on-change sensors, default 0)
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError EnableSensor(BNO085Sensor sensor, uint32_t interval_ms,
                             float sensitivity = 0.0f) noexcept;

    /**
     * @brief Disable a specific sensor.
     * @param sensor Sensor to disable
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError DisableSensor(BNO085Sensor sensor) noexcept;

    /**
     * @brief Apply a complete configuration to the sensor.
     *
     * Enables/disables sensors according to the config flags and intervals.
     *
     * @param config New configuration to apply
     * @return Bno08xError::SUCCESS if all sensor enables succeeded
     */
    Bno08xError ApplyConfiguration(const Bno08xConfig& config) noexcept;

    /**
     * @brief Get the current configuration.
     * @return Const reference to the active Bno08xConfig
     */
    const Bno08xConfig& GetConfiguration() const noexcept;

    // ========================================================================
    //  HARDWARE CONTROL
    // ========================================================================

    /**
     * @brief Perform hardware reset via RSTN pin.
     *
     * Drives RSTN LOW for the specified duration, then releases it and
     * waits for the sensor to boot.
     *
     * @param reset_duration_ms Duration to hold reset low (default 10 ms)
     * @return Bno08xError::SUCCESS if reset GPIO is available
     */
    Bno08xError HardwareReset(uint32_t reset_duration_ms = 10) noexcept;

    /**
     * @brief Control the BOOTN pin for DFU mode.
     * @param boot_state true = drive LOW (enter bootloader), false = HIGH
     */
    Bno08xError SetBootPin(bool boot_state) noexcept;

    /**
     * @brief Control the WAKE pin (SPI mode).
     * @param wake_state true = drive LOW (wake sensor), false = release
     */
    Bno08xError SetWakePin(bool wake_state) noexcept;

    // ========================================================================
    //  CALLBACK MANAGEMENT
    // ========================================================================

    /**
     * @brief Set callback for sensor events.
     *
     * The callback is invoked from within Update() whenever a sensor report
     * arrives. Only one callback can be active at a time.
     *
     * @param callback Callback function for sensor events
     */
    void SetSensorCallback(SensorCallback callback) noexcept;

    /**
     * @brief Clear sensor event callback.
     */
    void ClearSensorCallback() noexcept;

    // ========================================================================
    //  UTILITY METHODS
    // ========================================================================

    /**
     * @brief Convert quaternion to Euler angles (roll, pitch, yaw in radians).
     * @param quaternion Input quaternion (must have valid == true for output)
     * @param euler_angles Output: roll, pitch, yaw, accuracy, timestamp, valid
     */
    static void QuaternionToEuler(const Bno08xQuaternion& quaternion,
                                  Bno08xEulerAngles& euler_angles) noexcept;

    /**
     * @brief Get the communication interface type.
     * @return BNO085Interface::I2C or BNO085Interface::SPI
     */
    BNO085Interface GetInterfaceType() const noexcept;

    /**
     * @brief Get the last handler error code.
     * @return Last Bno08xError from a handler API call
     */
    Bno08xError GetLastError() const noexcept;

    /**
     * @brief Get the last driver-level SH-2 error code.
     * @return SH2_OK (0) or negative value from sh2_err.h; -1 if not initialized
     */
    int GetLastDriverError() const noexcept;

    /**
     * @brief Get a human-readable description of the handler.
     * @return String e.g. "BNO08x IMU (I2C @0x4A)" or "BNO08x IMU (SPI)"
     */
    const char* GetDescription() const noexcept;

    /**
     * @brief Get default sensor configuration.
     * @return Default Bno08xConfig structure with sensible defaults
     */
    static Bno08xConfig GetDefaultConfig() noexcept { return Bno08xConfig{}; }

    /**
     * @brief Dump comprehensive diagnostics to the logger.
     */
    void DumpDiagnostics() const noexcept;

private:
    // ========================================================================
    //  PRIVATE MEMBERS
    // ========================================================================

    std::unique_ptr<IBno08xDriverOps> driver_ops_; ///< Type-erased driver
    Bno08xConfig config_;                          ///< Current configuration
    mutable RtosMutex handler_mutex_;              ///< Thread safety mutex
    bool initialized_{false};                      ///< Initialization state
    mutable Bno08xError last_error_{Bno08xError::SUCCESS}; ///< Last error
    BNO085Interface interface_type_;               ///< I2C or SPI
    SensorCallback user_callback_;                 ///< User's sensor callback
    char description_[64]{};                       ///< Description string

    // ========================================================================
    //  PRIVATE HELPERS
    // ========================================================================

    /**
     * @brief Internal: read a vector-type sensor (no mutex, no init check).
     */
    Bno08xError readVectorSensor(BNO085Sensor sensor,
                                 Bno08xVector3& out) const noexcept;

    /**
     * @brief Internal: apply configuration (assumes mutex is held).
     */
    bool applyConfigLocked(const Bno08xConfig& config) noexcept;

    /**
     * @brief Internal: map SH-2 error code to Bno08xError.
     */
    static Bno08xError mapDriverError(int sh2_error) noexcept;
};

// ============================================================================
//  FACTORY METHODS
// ============================================================================

/**
 * @brief Create BNO08x handler with I2C interface.
 * @param i2c_device BaseI2c instance (address pre-configured)
 * @param config Sensor configuration (default: Bno08xConfig{})
 * @param reset_gpio Optional RSTN GPIO (active-low)
 * @param int_gpio Optional INT GPIO (active-low, data ready)
 * @return Unique pointer to Bno08xHandler
 */
inline std::unique_ptr<Bno08xHandler> CreateBno08xHandlerI2c(
    BaseI2c& i2c_device,
    const Bno08xConfig& config = Bno08xConfig{},
    BaseGpio* reset_gpio = nullptr,
    BaseGpio* int_gpio = nullptr) noexcept {
    return std::make_unique<Bno08xHandler>(i2c_device, config, reset_gpio, int_gpio);
}

/**
 * @brief Create BNO08x handler with SPI interface.
 * @param spi_device BaseSpi instance
 * @param config Sensor configuration (default: Bno08xConfig{})
 * @param reset_gpio Optional RSTN GPIO (active-low)
 * @param int_gpio Optional INT GPIO (active-low, data ready)
 * @param wake_gpio Optional WAKE GPIO (active-low, SPI mode)
 * @return Unique pointer to Bno08xHandler
 */
inline std::unique_ptr<Bno08xHandler> CreateBno08xHandlerSpi(
    BaseSpi& spi_device,
    const Bno08xConfig& config = Bno08xConfig{},
    BaseGpio* reset_gpio = nullptr,
    BaseGpio* int_gpio = nullptr,
    BaseGpio* wake_gpio = nullptr) noexcept {
    return std::make_unique<Bno08xHandler>(spi_device, config, reset_gpio, int_gpio, wake_gpio);
}

#endif // COMPONENT_HANDLER_BNO08X_HANDLER_H_
