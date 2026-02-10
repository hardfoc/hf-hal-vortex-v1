/**
 * @file Bno08xHandler.h
 * @brief Unified handler for BNO08x IMU sensor family with multi-interface support.
 *
 * This class provides a modern, unified interface for BNO08x IMU sensors following
 * the same architectural excellence as TMC9660Handler, PCAL95555Handler, and AS5047UHandler:
 * - Bridge pattern for BaseI2c/BaseSpi integration
 * - Lazy initialization with lightweight construction
 * - Shared pointer management for safe memory handling
 * - Complete exception-free design with noexcept methods
 * - Thread-safe operations with RtosMutex protection
 * - Comprehensive error handling and validation
 * - High-level IMU abstraction with fusion algorithms
 * - Advanced BNO08x features (calibration, gesture detection, activity classification)
 * - Multi-interface support (I2C, SPI, UART)
 * - Factory methods for different communication interfaces
 *
 * Features:
 * - 9-DOF sensor fusion (accelerometer, gyroscope, magnetometer)
 * - Quaternion and Euler angle orientation
 * - Linear acceleration and gravity vectors
 * - Advanced motion detection (tap, shake, step counting)
 * - Activity classification and gesture recognition
 * - Calibration management and sensor accuracy monitoring
 * - Multiple sensor reports with configurable intervals
 * - Hardware control (reset, wake, interface selection)
 * - Firmware update capabilities
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 * @copyright HardFOC
 */

#ifndef COMPONENT_HANDLER_BNO08X_HANDLER_H_
#define COMPONENT_HANDLER_BNO08X_HANDLER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <array>
#include <functional>
#include "utils-and-drivers/hf-core-drivers/external/hf-bno08x-driver/src/BNO085.hpp"
#include "utils-and-drivers/hf-core-drivers/external/hf-bno08x-driver/src/BNO085_Transport.hpp"
#include "base/BaseI2c.h"
#include "base/BaseSpi.h"
#include "base/BaseGpio.h"
#include "utils/RtosMutex.h"

//======================================================//
// BNO08X ERROR CODES
//======================================================//

/**
 * @brief BNO08x handler error codes for consistent error reporting
 */
enum class Bno08xError : uint8_t {
    SUCCESS = 0,
    NOT_INITIALIZED,
    INITIALIZATION_FAILED,
    INVALID_PARAMETER,
    COMMUNICATION_FAILED,
    SENSOR_NOT_RESPONDING,
    CALIBRATION_FAILED,
    FIRMWARE_UPDATE_FAILED,
    TIMEOUT,
    MUTEX_LOCK_FAILED,
    INVALID_INTERFACE,
    SENSOR_NOT_ENABLED,
    DATA_NOT_AVAILABLE,
    HARDWARE_ERROR
};

/**
 * @brief Convert Bno08xError to string for debugging
 */
constexpr const char* Bno08xErrorToString(Bno08xError error) noexcept {
    switch (error) {
        case Bno08xError::SUCCESS: return "Success";
        case Bno08xError::NOT_INITIALIZED: return "Not initialized";
        case Bno08xError::INITIALIZATION_FAILED: return "Initialization failed";
        case Bno08xError::INVALID_PARAMETER: return "Invalid parameter";
        case Bno08xError::COMMUNICATION_FAILED: return "Communication failed";
        case Bno08xError::SENSOR_NOT_RESPONDING: return "Sensor not responding";
        case Bno08xError::CALIBRATION_FAILED: return "Calibration failed";
        case Bno08xError::FIRMWARE_UPDATE_FAILED: return "Firmware update failed";
        case Bno08xError::TIMEOUT: return "Timeout";
        case Bno08xError::MUTEX_LOCK_FAILED: return "Mutex lock failed";
        case Bno08xError::INVALID_INTERFACE: return "Invalid interface";
        case Bno08xError::SENSOR_NOT_ENABLED: return "Sensor not enabled";
        case Bno08xError::DATA_NOT_AVAILABLE: return "Data not available";
        case Bno08xError::HARDWARE_ERROR: return "Hardware error";
        default: return "Unknown error";
    }
}

//======================================================//
// BNO08X TRANSPORT ADAPTERS
//======================================================//

/**
 * @brief I2C transport adapter connecting BaseI2c interface to BNO08x IBNO085Transport interface.
 * 
 * This adapter implements the IBNO085Transport virtual interface using a HardFOC
 * BaseI2c implementation, enabling the BNO08x driver to work with any I2C
 * controller that inherits from BaseI2c.
 */
class Bno08xI2cAdapter : public IBNO085Transport {
public:
    /**
     * @brief Construct I2C adapter with BaseI2c interface
     * @param i2c_interface Reference to BaseI2c implementation
     * @param reset_gpio Optional GPIO for hardware reset control
     * @param int_gpio Optional GPIO for interrupt monitoring
     */
    explicit Bno08xI2cAdapter(BaseI2c& i2c_interface, 
                             BaseGpio* reset_gpio = nullptr,
                             BaseGpio* int_gpio = nullptr) noexcept;

    // IBNO085Transport interface implementation
    bool open() override;
    void close() override;
    int write(const uint8_t* data, uint32_t length) override;
    int read(uint8_t* data, uint32_t length) override;
    bool dataAvailable() override;
    void delay(uint32_t ms) override;
    uint32_t getTimeUs() override;
    void setReset(bool state) override;
    void setBoot(bool state) override;

private:
    BaseI2c& i2c_interface_;
    BaseGpio* reset_gpio_;
    BaseGpio* int_gpio_;
    bool is_open_;
};

/**
 * @brief SPI transport adapter connecting BaseSpi interface to BNO08x IBNO085Transport interface.
 * 
 * This adapter implements the IBNO085Transport virtual interface using a HardFOC
 * BaseSpi implementation, enabling the BNO08x driver to work with any SPI
 * controller that inherits from BaseSpi.
 */
class Bno08xSpiAdapter : public IBNO085Transport {
public:
    /**
     * @brief Construct SPI adapter with BaseSpi interface
     * @param spi_interface Reference to BaseSpi implementation
     * @param wake_gpio Optional GPIO for wake control
     * @param reset_gpio Optional GPIO for hardware reset control
     * @param int_gpio Optional GPIO for interrupt monitoring
     */
    explicit Bno08xSpiAdapter(BaseSpi& spi_interface,
                             BaseGpio* wake_gpio = nullptr,
                             BaseGpio* reset_gpio = nullptr,
                             BaseGpio* int_gpio = nullptr) noexcept;

    // IBNO085Transport interface implementation
    bool open() override;
    void close() override;
    int write(const uint8_t* data, uint32_t length) override;
    int read(uint8_t* data, uint32_t length) override;
    bool dataAvailable() override;
    void delay(uint32_t ms) override;
    uint32_t getTimeUs() override;
    void setReset(bool state) override;
    void setBoot(bool state) override;
    void setWake(bool state) override;

private:
    BaseSpi& spi_interface_;
    BaseGpio* wake_gpio_;
    BaseGpio* reset_gpio_;
    BaseGpio* int_gpio_;
    bool is_open_;
};

//======================================================//
// BNO08X SENSOR DATA STRUCTURES
//======================================================//

/**
 * @brief Enhanced vector with timestamp and accuracy
 */
struct Bno08xVector3 {
    float x;                    ///< X component
    float y;                    ///< Y component  
    float z;                    ///< Z component
    uint8_t accuracy;           ///< Sensor accuracy (0-3)
    uint64_t timestamp_us;      ///< Timestamp in microseconds
    bool valid;                 ///< Data validity flag

    Bno08xVector3() : x(0), y(0), z(0), accuracy(0), timestamp_us(0), valid(false) {}
};

/**
 * @brief Enhanced quaternion with timestamp and accuracy
 */
struct Bno08xQuaternion {
    float w;                    ///< Real component
    float x;                    ///< i component
    float y;                    ///< j component
    float z;                    ///< k component
    uint8_t accuracy;           ///< Sensor accuracy (0-3)
    uint64_t timestamp_us;      ///< Timestamp in microseconds
    bool valid;                 ///< Data validity flag

    Bno08xQuaternion() : w(1), x(0), y(0), z(0), accuracy(0), timestamp_us(0), valid(false) {}
};

/**
 * @brief Euler angles derived from quaternion
 */
struct Bno08xEulerAngles {
    float roll;                 ///< Roll angle in radians
    float pitch;                ///< Pitch angle in radians
    float yaw;                  ///< Yaw angle in radians
    uint8_t accuracy;           ///< Derived accuracy
    uint64_t timestamp_us;      ///< Timestamp in microseconds
    bool valid;                 ///< Data validity flag

    Bno08xEulerAngles() : roll(0), pitch(0), yaw(0), accuracy(0), timestamp_us(0), valid(false) {}
};

/**
 * @brief Complete IMU sensor data structure
 */
struct Bno08xImuData {
    Bno08xVector3 acceleration;        ///< Calibrated acceleration (m/s²)
    Bno08xVector3 gyroscope;           ///< Calibrated angular velocity (rad/s)
    Bno08xVector3 magnetometer;        ///< Calibrated magnetic field (uT)
    Bno08xVector3 linear_acceleration; ///< Linear acceleration (no gravity)
    Bno08xVector3 gravity;             ///< Gravity vector
    Bno08xQuaternion rotation;         ///< Orientation quaternion
    Bno08xEulerAngles euler;           ///< Euler angles
    uint64_t timestamp_us;             ///< Overall timestamp
    bool valid;                        ///< Overall validity flag
};

/**
 * @brief Activity and gesture detection data
 */
struct Bno08xActivityData {
    bool tap_detected;          ///< Single/double tap detected
    bool double_tap;            ///< True if double tap
    uint8_t tap_direction;      ///< Tap direction (0-5)
    uint32_t step_count;        ///< Total step count
    bool step_detected;         ///< New step detected
    bool shake_detected;        ///< Shake gesture detected
    bool pickup_detected;       ///< Pickup gesture detected
    bool significant_motion;    ///< Significant motion detected
    uint8_t stability_class;    ///< Stability classification
    uint8_t activity_class;     ///< Activity classification
    uint64_t timestamp_us;      ///< Timestamp
    bool valid;                 ///< Data validity
};

/**
 * @brief Sensor calibration status
 */
struct Bno08xCalibrationStatus {
    uint8_t accelerometer_accuracy;    ///< Accelerometer calibration (0-3)
    uint8_t gyroscope_accuracy;        ///< Gyroscope calibration (0-3)
    uint8_t magnetometer_accuracy;     ///< Magnetometer calibration (0-3)
    uint8_t system_accuracy;           ///< Overall system accuracy (0-3)
    bool calibration_complete;         ///< True if all sensors calibrated
    uint32_t calibration_time_s;       ///< Time spent calibrating
};

/**
 * @brief BNO08x configuration structure
 */
struct Bno08xConfig {
    // Sensor enable flags
    bool enable_accelerometer;         ///< Enable calibrated accelerometer
    bool enable_gyroscope;             ///< Enable calibrated gyroscope
    bool enable_magnetometer;          ///< Enable calibrated magnetometer
    bool enable_rotation_vector;       ///< Enable rotation vector
    bool enable_linear_acceleration;   ///< Enable linear acceleration
    bool enable_gravity;               ///< Enable gravity vector
    bool enable_game_rotation;         ///< Enable game rotation vector
    
    // Activity detection
    bool enable_tap_detector;          ///< Enable tap detection
    bool enable_step_counter;          ///< Enable step counting
    bool enable_shake_detector;        ///< Enable shake detection
    bool enable_pickup_detector;       ///< Enable pickup detection
    bool enable_significant_motion;    ///< Enable significant motion
    bool enable_activity_classifier;   ///< Enable activity classification
    
    // Sensor intervals (milliseconds)
    uint32_t accelerometer_interval_ms;
    uint32_t gyroscope_interval_ms;
    uint32_t magnetometer_interval_ms;
    uint32_t rotation_interval_ms;
    uint32_t linear_accel_interval_ms;
    uint32_t gravity_interval_ms;
    
    // Interface selection
    BNO085Interface interface_type;    ///< Communication interface
    
    // Calibration settings
    bool auto_calibration;             ///< Enable automatic calibration
    float calibration_timeout_s;       ///< Calibration timeout
};

//======================================================//
// BNO08X HANDLER CLASS
//======================================================//

/**
 * @brief Unified handler for BNO08x IMU sensor family.
 * 
 * This class provides a comprehensive interface for BNO08x sensors with:
 * - Lazy initialization pattern for optimal memory usage
 * - Shared pointer management for safe cross-component sharing
 * - Complete exception-free design for embedded reliability
 * - Thread-safe operations with mutex protection
 * - Bridge pattern integration with BaseI2c/BaseSpi
 * - High-level IMU sensor abstraction
 * - Advanced BNO08x features and diagnostics
 * 
 * Architecture follows the same excellence as other HardFOC handlers.
 */
class Bno08xHandler {
public:
    //======================================================//
    // CONSTRUCTION AND LIFECYCLE
    //======================================================//

    /**
     * @brief Construct BNO08x handler with I2C interface
     * @param i2c_interface Reference to BaseI2c implementation
     * @param config Initial sensor configuration
     * @param reset_gpio Optional GPIO for hardware reset
     * @param int_gpio Optional GPIO for interrupt monitoring
     */
    explicit Bno08xHandler(BaseI2c& i2c_interface, 
                          const Bno08xConfig& config = GetDefaultConfig(),
                          BaseGpio* reset_gpio = nullptr,
                          BaseGpio* int_gpio = nullptr) noexcept;

    /**
     * @brief Construct BNO08x handler with SPI interface
     * @param spi_interface Reference to BaseSpi implementation
     * @param config Initial sensor configuration
     * @param wake_gpio Optional GPIO for wake control
     * @param reset_gpio Optional GPIO for hardware reset
     * @param int_gpio Optional GPIO for interrupt monitoring
     */
    explicit Bno08xHandler(BaseSpi& spi_interface,
                          const Bno08xConfig& config = GetDefaultConfig(),
                          BaseGpio* wake_gpio = nullptr,
                          BaseGpio* reset_gpio = nullptr,
                          BaseGpio* int_gpio = nullptr) noexcept;

    /**
     * @brief Destructor - automatically handles cleanup
     */
    ~Bno08xHandler() noexcept = default;

    // Disable copy construction and assignment
    Bno08xHandler(const Bno08xHandler&) = delete;
    Bno08xHandler& operator=(const Bno08xHandler&) = delete;

    // Enable move construction and assignment
    Bno08xHandler(Bno08xHandler&&) noexcept = default;
    Bno08xHandler& operator=(Bno08xHandler&&) noexcept = default;

    //======================================================//
    // INITIALIZATION AND STATUS
    //======================================================//

    /**
     * @brief Initialize the BNO08x sensor (lazy initialization)
     * @return Bno08xError::SUCCESS if successful, error code otherwise
     */
    Bno08xError Initialize() noexcept;

    /**
     * @brief Deinitialize the sensor and free resources
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError Deinitialize() noexcept;

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
     * @brief Get shared pointer to BNO085 driver for advanced operations
     * @return Shared pointer to BNO085 driver or nullptr if not initialized
     */
    std::shared_ptr<BNO085> GetSensor() noexcept;
    /**
     * @brief Update sensor - must be called regularly for proper operation
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError Update() noexcept;

    /**
     * @brief Check if new data is available for a specific sensor
     * @param sensor Sensor to check
     * @return True if new data is available
     */
    bool HasNewData(BNO085Sensor sensor) const noexcept;

    //======================================================//
    // SENSOR DATA READING
    //======================================================//

    /**
     * @brief Read complete IMU data structure
     * @param imu_data Output structure for IMU data
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError ReadImuData(Bno08xImuData& imu_data) noexcept;

    /**
     * @brief Read calibrated acceleration vector
     * @param acceleration Output acceleration vector (m/s²)
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError ReadAcceleration(Bno08xVector3& acceleration) noexcept;

    /**
     * @brief Read calibrated gyroscope data
     * @param gyroscope Output angular velocity (rad/s)
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError ReadGyroscope(Bno08xVector3& gyroscope) noexcept;

    /**
     * @brief Read calibrated magnetometer data
     * @param magnetometer Output magnetic field (µT)
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError ReadMagnetometer(Bno08xVector3& magnetometer) noexcept;

    /**
     * @brief Read orientation quaternion
     * @param quaternion Output orientation quaternion
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError ReadQuaternion(Bno08xQuaternion& quaternion) noexcept;

    /**
     * @brief Read Euler angles
     * @param euler_angles Output Euler angles (roll, pitch, yaw)
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError ReadEulerAngles(Bno08xEulerAngles& euler_angles) noexcept;

    /**
     * @brief Read linear acceleration (acceleration without gravity)
     * @param linear_accel Output linear acceleration
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError ReadLinearAcceleration(Bno08xVector3& linear_accel) noexcept;

    /**
     * @brief Read gravity vector
     * @param gravity Output gravity vector
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError ReadGravity(Bno08xVector3& gravity) noexcept;

    //======================================================//
    // ACTIVITY AND GESTURE DETECTION
    //======================================================//

    /**
     * @brief Read activity and gesture detection data
     * @param activity_data Output activity data structure
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError ReadActivityData(Bno08xActivityData& activity_data) noexcept;

    /**
     * @brief Check if new step was detected
     * @param step_detected Output step detection flag
     * @param step_count Output total step count
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError CheckStepDetection(bool& step_detected, uint32_t& step_count) noexcept;

    /**
     * @brief Check if tap was detected
     * @param tap_detected Output tap detection flag
     * @param double_tap Output double tap flag
     * @param direction Output tap direction
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError CheckTapDetection(bool& tap_detected, bool& double_tap, uint8_t& direction) noexcept;

    /**
     * @brief Check for gesture detection
     * @param shake Output shake detection
     * @param pickup Output pickup detection
     * @param significant_motion Output significant motion
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError CheckGestureDetection(bool& shake, bool& pickup, bool& significant_motion) noexcept;

    //======================================================//
    // CALIBRATION AND ACCURACY
    //======================================================//

    /**
     * @brief Read calibration status
     * @param calibration_status Output calibration status
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError ReadCalibrationStatus(Bno08xCalibrationStatus& calibration_status) noexcept;

    /**
     * @brief Start calibration process
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError StartCalibration() noexcept;

    /**
     * @brief Save current calibration to sensor
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError SaveCalibration() noexcept;

    /**
     * @brief Check if all sensors are properly calibrated
     * @param calibrated Output calibration status
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError IsCalibrated(bool& calibrated) noexcept;

    //======================================================//
    // SENSOR CONFIGURATION
    //======================================================//

    /**
     * @brief Enable specific sensor
     * @param sensor Sensor to enable
     * @param interval_ms Reporting interval in milliseconds
     * @param sensitivity Change sensitivity (for on-change sensors)
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError EnableSensor(BNO085Sensor sensor, uint32_t interval_ms, float sensitivity = 0.0f) noexcept;

    /**
     * @brief Disable specific sensor
     * @param sensor Sensor to disable
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError DisableSensor(BNO085Sensor sensor) noexcept;

    /**
     * @brief Update sensor configuration
     * @param config New configuration to apply
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError UpdateConfiguration(const Bno08xConfig& config) noexcept;

    /**
     * @brief Get current sensor configuration
     * @param config Output current configuration
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError GetConfiguration(Bno08xConfig& config) noexcept;

    //======================================================//
    // HARDWARE CONTROL
    //======================================================//

    /**
     * @brief Perform hardware reset
     * @param reset_duration_ms Duration to hold reset low
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError HardwareReset(uint32_t reset_duration_ms = 10) noexcept;

    /**
     * @brief Control wake pin (SPI mode only)
     * @param wake_state Wake pin state
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError SetWakePin(bool wake_state) noexcept;

    /**
     * @brief Control boot pin for DFU mode
     * @param boot_state Boot pin state
     * @return Bno08xError::SUCCESS if successful
     */
    Bno08xError SetBootPin(bool boot_state) noexcept;

    //======================================================//
    // CALLBACK MANAGEMENT
    //======================================================//

    /**
     * @brief Set callback for sensor events
     * @param callback Callback function for sensor events
     */
    void SetSensorCallback(SensorCallback callback) noexcept;

    /**
     * @brief Clear sensor event callback
     */
    void ClearSensorCallback() noexcept;

    //======================================================//
    // UTILITY METHODS
    //======================================================//

    /**
     * @brief Convert quaternion to Euler angles
     * @param quaternion Input quaternion
     * @param euler_angles Output Euler angles
     */
    static void QuaternionToEuler(const Bno08xQuaternion& quaternion, Bno08xEulerAngles& euler_angles) noexcept;

    /**
     * @brief Get default sensor configuration
     * @return Default Bno08xConfig structure
     */
    static Bno08xConfig GetDefaultConfig() noexcept;

    /**
     * @brief Get sensor description string
     * @return Description of the sensor
     */
    const char* GetDescription() const noexcept;

    /**
     * @brief Get last error code
     * @return Last error that occurred
     */
    Bno08xError GetLastError() const noexcept;

    /**
     * @brief Get interface type being used
     * @return Communication interface type
     */
    BNO085Interface GetInterfaceType() const noexcept;

private:
    //======================================================//
    // PRIVATE MEMBERS
    //======================================================//

    std::unique_ptr<IBNO085Transport> transport_adapter_; ///< Transport adapter
    std::shared_ptr<BNO085> bno08x_sensor_;              ///< BNO08x driver instance
    Bno08xConfig config_;                                 ///< Sensor configuration
    mutable RtosMutex handler_mutex_;                     ///< Thread safety mutex
    bool initialized_;                                    ///< Initialization state
    mutable Bno08xError last_error_;                      ///< Last error code
    char description_[64];                                ///< Sensor description
    SensorCallback sensor_callback_;                      ///< User sensor callback

    // Interface type and GPIO references
    BNO085Interface interface_type_;
    BaseGpio* reset_gpio_;
    BaseGpio* wake_gpio_;
    BaseGpio* int_gpio_;

    //======================================================//
    // PRIVATE HELPER METHODS
    //======================================================//

    /**
     * @brief Validate sensor state
     * @return True if sensor is in valid state
     */
    bool ValidateSensor() const noexcept;

    /**
     * @brief Apply configuration to sensor
     * @param config Configuration to apply
     * @return True if successful
     */
    bool ApplyConfiguration(const Bno08xConfig& config) noexcept;

    /**
     * @brief Internal sensor event handler
     * @param event Sensor event from BNO08x
     */
    void HandleSensorEvent(const SensorEvent& event) noexcept;

    /**
     * @brief Convert SensorEvent to internal structures
     * @param event Input sensor event
     * @param sensor Sensor type
     * @return True if conversion successful
     */
    bool ConvertSensorEvent(const SensorEvent& event, BNO085Sensor sensor) noexcept;

public:
    /**
     * @brief Dump comprehensive diagnostics and statistics to log as INFO level.
     * Logs BNO08x sensor status, communication health, and measurement statistics.
     */
    void DumpDiagnostics() const noexcept;
};

//======================================================//
// FACTORY METHODS
//======================================================//

/**
 * @brief Create BNO08x handler with I2C interface
 * @param i2c_interface Reference to I2C interface
 * @param config Sensor configuration
 * @param reset_gpio Optional reset GPIO
 * @param int_gpio Optional interrupt GPIO
 * @return Unique pointer to BNO08x handler
 */
std::unique_ptr<Bno08xHandler> CreateBno08xHandlerI2c(
    BaseI2c& i2c_interface,
    const Bno08xConfig& config = Bno08xHandler::GetDefaultConfig(),
    BaseGpio* reset_gpio = nullptr,
    BaseGpio* int_gpio = nullptr) noexcept;

/**
 * @brief Create BNO08x handler with SPI interface
 * @param spi_interface Reference to SPI interface
 * @param config Sensor configuration
 * @param wake_gpio Optional wake GPIO
 * @param reset_gpio Optional reset GPIO
 * @param int_gpio Optional interrupt GPIO
 * @return Unique pointer to BNO08x handler
 */
std::unique_ptr<Bno08xHandler> CreateBno08xHandlerSpi(
    BaseSpi& spi_interface,
    const Bno08xConfig& config = Bno08xHandler::GetDefaultConfig(),
    BaseGpio* wake_gpio = nullptr,
    BaseGpio* reset_gpio = nullptr,
    BaseGpio* int_gpio = nullptr) noexcept;

#endif // COMPONENT_HANDLER_BNO08X_HANDLER_H_
