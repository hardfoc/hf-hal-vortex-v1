/**
 * @file TemperatureManager.h
 * @brief Advanced temperature sensor management system for the HardFOC platform.
 * 
 * @details This class provides a comprehensive temperature sensor management system that
 *          manages onboard temperature sensors including ESP32 internal temperature sensor
 *          (via EspTemperature) and NTC temperature sensors (via NtcTemperatureHandler).
 *          It provides index-based and string-view-based access to all temperature sensors.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 * 
 * Key Features:
 * - Multi-sensor temperature management (ESP32 internal, NTC thermistors)
 * - Index-based and string-view-based sensor access
 * - Thread-safe operation with comprehensive error handling
 * - Automatic sensor registration and discovery
 * - Advanced diagnostics and health monitoring
 * - Batch operations for multiple sensors
 * - Hardware resource validation and conflict detection
 * - String-based sensor identification for extensibility
 * - Smart sensor categorization and mapping
 * - Complete BaseTemperature function coverage through string-based routing
 * 
 * Architecture:
 * - Uses string_view for sensor identification (extensible)
 * - Integrates with platform mapping for hardware mapping
 * - Supports all temperature sensor types defined in BaseTemperature
 * - Provides unified BaseTemperature interface for all sensor operations
 * - Handler-based sensor creation for proper ownership
 * - Routes all BaseTemperature functions through string-based API
 * 
 * @note This class is thread-safe and designed for concurrent access from multiple tasks.
 * @note All sensor operations use string_view identifiers for maximum flexibility.
 */

#ifndef COMPONENT_HANDLER_TEMPERATURE_MANAGER_H_
#define COMPONENT_HANDLER_TEMPERATURE_MANAGER_H_

#include "CommonIDs.h"
#include "ThingsToString.h"
#include "base/BaseTemperature.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspTemperature.h"
#include "handlers/ntc/NtcTemperatureHandler.h"
#include "handlers/tmc9660/Tmc9660Handler.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseAdc.h"
#include "managers/AdcManager.h"
#include "managers/MotorController.h"
#include "handlers/logger/Logger.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsAbstraction.h"

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <string_view>
#include <unordered_map>
#include <optional>

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class AdcManager;

/**
 * @brief Wrapper class for TMC9660 temperature sensor to integrate with TemperatureManager.
 * 
 * This class provides a BaseTemperature interface wrapper around the TMC9660Handler's
 * internal temperature sensor, allowing it to be managed by the TemperatureManager.
 */
class Tmc9660TemperatureWrapper : public BaseTemperature {
public:
    /**
     * @brief Constructor that takes a reference to TMC9660Handler.
     * @param tmc9660_handler Reference to TMC9660Handler instance
     */
    explicit Tmc9660TemperatureWrapper(Tmc9660Handler& tmc9660_handler) noexcept;
    
    /**
     * @brief Destructor.
     */
    ~Tmc9660TemperatureWrapper() override = default;
    
    // Pure virtual methods from BaseTemperature
    bool Initialize() noexcept override;
    bool Deinitialize() noexcept override;
    hf_temp_err_t ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept override;
    hf_temp_err_t GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept override;
    hf_u32_t GetCapabilities() const noexcept override;

private:
    Tmc9660Handler& tmc9660_handler_;  ///< Reference to the TMC9660Handler instance
};

//==============================================================================
// TEMPERATURE SENSOR INFORMATION STRUCTURES
//==============================================================================

/**
 * @brief Structure containing comprehensive temperature sensor information.
 */
struct TempSensorInfo {
    std::string_view name;                      ///< Human-readable name (string_view to static data)
    std::unique_ptr<BaseTemperature> sensor;    ///< Temperature sensor instance (unique ownership)
    hf_temp_sensor_type_t sensor_type;          ///< Type of temperature sensor
    uint8_t sensor_id;                          ///< Unique sensor identifier
    bool is_registered;                         ///< Registration status
    
    // Hardware configuration
    float min_temp_celsius;                     ///< Minimum temperature range
    float max_temp_celsius;                     ///< Maximum temperature range
    float resolution_celsius;                   ///< Temperature resolution
    float accuracy_celsius;                     ///< Typical accuracy
    
    // Statistics and monitoring
    uint32_t access_count;                      ///< Number of times accessed
    uint32_t error_count;                       ///< Number of errors encountered
    uint64_t last_access_time;                  ///< Timestamp of last access
    float last_reading_celsius;                 ///< Last temperature reading
    
    /**
     * @brief Constructor for TempSensorInfo.
     */
    TempSensorInfo(std::string_view n, std::unique_ptr<BaseTemperature> s, 
                   hf_temp_sensor_type_t type, uint8_t id,
                   float min_temp = -40.0f, float max_temp = 125.0f, 
                   float res = 0.1f, float acc = 1.0f) noexcept
        : name(n), sensor(std::move(s)), sensor_type(type), sensor_id(id), is_registered(true),
          min_temp_celsius(min_temp), max_temp_celsius(max_temp), resolution_celsius(res), accuracy_celsius(acc),
          access_count(0), error_count(0), last_access_time(0), last_reading_celsius(0.0f) {}
    
    // Disable copy operations due to unique_ptr
    TempSensorInfo(const TempSensorInfo&) = delete;
    TempSensorInfo& operator=(const TempSensorInfo&) = delete;
    
    // Enable move operations
    TempSensorInfo(TempSensorInfo&&) = default;
    TempSensorInfo& operator=(TempSensorInfo&&) = default;
};

/**
 * @brief Structure for temperature sensor batch operation configuration.
 */
struct TempBatchOperation {
    std::vector<std::string_view> sensor_names; ///< Sensor names to operate on
    std::vector<uint8_t> samples_per_sensor;    ///< Samples per sensor
    std::vector<uint16_t> intervals_ms;         ///< Intervals between samples in ms
    bool use_individual_specs;                  ///< Use individual specs or common settings
    uint8_t common_samples;                     ///< Common number of samples (if not using individual specs)
    uint16_t common_interval_ms;                ///< Common sampling interval (if not using individual specs)
    
    /**
     * @brief Constructor for simple batch read.
     */
    TempBatchOperation(std::vector<std::string_view> names, uint8_t samples = 1, uint16_t interval_ms = 0) noexcept
        : sensor_names(std::move(names)), use_individual_specs(false),
          common_samples(samples), common_interval_ms(interval_ms) {}
    
    /**
     * @brief Constructor for advanced batch read with individual specs.
     */
    TempBatchOperation(std::vector<std::string_view> names, std::vector<uint8_t> samples, std::vector<uint16_t> intervals) noexcept
        : sensor_names(std::move(names)), samples_per_sensor(std::move(samples)), 
          intervals_ms(std::move(intervals)), use_individual_specs(true),
          common_samples(1), common_interval_ms(0) {}
};

/**
 * @brief Structure for temperature sensor batch operation results.
 */
struct TempBatchResult {
    std::vector<std::string_view> sensor_names; ///< Sensor names operated on
    std::vector<float> temperatures_celsius;    ///< Resulting temperature readings in Celsius
    std::vector<float> temperatures_fahrenheit; ///< Resulting temperature readings in Fahrenheit
    std::vector<hf_temp_reading_t> readings;    ///< Complete reading information
    std::vector<hf_temp_err_t> results;         ///< Individual operation results
    hf_temp_err_t overall_result;               ///< Overall operation result
    uint32_t total_time_ms;                     ///< Total operation time
    
    /**
     * @brief Check if all operations were successful.
     */
    [[nodiscard]] bool AllSuccessful() const noexcept {
        return overall_result == hf_temp_err_t::TEMP_SUCCESS;
    }
    
    /**
     * @brief Get success rate as percentage.
     */
    [[nodiscard]] float GetSuccessRate() const noexcept {
        if (results.empty()) return 0.0f;
        uint32_t successCount = 0;
        for (const auto& result : results) {
            if (result == hf_temp_err_t::TEMP_SUCCESS) ++successCount;
        }
        return (static_cast<float>(successCount) / static_cast<float>(results.size())) * 100.0f;
    }
};

/**
 * @brief Structure for temperature system diagnostics.
 */
struct TempSystemDiagnostics {
    bool system_healthy;                           ///< Overall system health
    uint32_t total_sensors_registered;             ///< Total sensors registered
    uint32_t sensors_by_type[8];                   ///< Sensors per type (internal, NTC, etc.)
    uint32_t total_operations;                     ///< Total operations performed
    uint32_t successful_operations;                ///< Successful operations
    uint32_t failed_operations;                    ///< Failed operations
    uint32_t communication_errors;                 ///< Communication errors
    uint32_t hardware_errors;                      ///< Hardware errors
    uint64_t system_uptime_ms;                     ///< System uptime
    hf_temp_err_t last_error;                      ///< Last error encountered
    float system_min_temp_celsius;                 ///< System-wide minimum temperature
    float system_max_temp_celsius;                 ///< System-wide maximum temperature
    float system_avg_temp_celsius;                 ///< System-wide average temperature
};

//==============================================================================
// MAIN TEMPERATURE MANAGER CLASS
//==============================================================================

/**
 * @class TemperatureManager
 * @brief Advanced temperature sensor management system for the HardFOC platform.
 * 
 * This class provides a comprehensive temperature sensor management system that
 * manages onboard temperature sensors including ESP32 internal temperature sensor
 * and NTC temperature sensors. It uses string_view identifiers to provide a
 * unified, hardware-agnostic API that routes all BaseTemperature functions
 * through string-based sensor identification.
 * 
 * Thread Safety:
 * - All public methods are thread-safe
 * - Uses internal mutex for protection
 * - Atomic operations where appropriate
 * 
 * Error Handling:
 * - Core operations return hf_temp_err_t for detailed error codes
 * - Comprehensive error codes via ResultCode enum
 * - Detailed error descriptions and diagnostics
 * 
 * Performance:
 * - Optimized for common operations
 * - Batch operations for multiple sensors
 * - Cached state information
 * - Lazy initialization of hardware resources
 * 
 * Platform Integration:
 * - Automatic sensor discovery and registration
 * - Hardware resource validation
 * - Conflict detection and resolution
 * - Multi-sensor coordination
 * - Smart sensor categorization
 * 
 * Function Coverage:
 * - Complete BaseTemperature function coverage through string-based routing
 * - All temperature operations available via string_view sensor names
 * - Consistent API design with proper camelCase naming
 */
class TemperatureManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================
    
    /**
     * @brief Get the singleton instance.
     * @return Reference to the temperature manager instance
     */
    static TemperatureManager& GetInstance() noexcept;
    
    /**
     * @brief Ensure the temperature manager system is initialized.
     * @return hf_temp_err_t::TEMP_SUCCESS if initialization successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t EnsureInitialized() noexcept;
    
    /**
     * @brief Shutdown the temperature manager system.
     * @return hf_temp_err_t::TEMP_SUCCESS if shutdown successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t Shutdown() noexcept;
    
    //==========================================================================
    // SENSOR REGISTRATION AND MANAGEMENT
    //==========================================================================
    
    /**
     * @brief Register an ESP32 internal temperature sensor.
     * @param name Sensor name identifier
     * @param config ESP32 temperature sensor configuration
     * @return hf_temp_err_t::TEMP_SUCCESS if registration successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t RegisterEspTemperatureSensor(std::string_view name, 
                                                             const esp_temp_config_t& config = ESP_TEMP_CONFIG_DEFAULT()) noexcept;
    
    /**
     * @brief Register an NTC temperature sensor.
     * @param name Sensor name identifier
     * @param adc_channel_name ADC channel name for the NTC sensor
     * @param config NTC temperature sensor configuration
     * @return hf_temp_err_t::TEMP_SUCCESS if registration successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t RegisterNtcTemperatureSensor(std::string_view name, 
                                                             std::string_view adc_channel_name,
                                                             const ntc_temp_handler_config_t& config = NTC_TEMP_HANDLER_CONFIG_DEFAULT()) noexcept;
    
    /**
     * @brief Register a TMC9660 internal temperature sensor.
     * @param name Sensor name identifier
     * @param tmc9660_handler Reference to TMC9660Handler instance
     * @return hf_temp_err_t::TEMP_SUCCESS if registration successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t RegisterTmc9660TemperatureSensor(std::string_view name, 
                                                                 Tmc9660Handler& tmc9660_handler) noexcept;

private:
    /**
     * @brief Automatically register known onboard temperature sensors.
     * @details This function is called during initialization to register:
     * - ESP32 internal temperature sensor
     * - TMC9660 AIN3 thermistor temperature sensor (if available)
     * - TMC9660 internal chip temperature sensor (if available)
     */
    void RegisterOnboardTemperatureSensors() noexcept;
    
    /**
     * @brief Register TMC9660 AIN3 thermistor temperature sensor.
     * @details Registers the NTC thermistor connected to TMC9660 AIN3 channel.
     */
    void RegisterTmc9660ThermistorSensor() noexcept;
    
    /**
     * @brief Register TMC9660 internal chip temperature sensor.
     * @details Registers the internal temperature sensor of the TMC9660 chip.
     */
    void RegisterTmc9660InternalSensor() noexcept;
    
    /**
     * @brief Automatically register onboard temperature sensors (ESP32 internal and TMC9660 internal).
     * @return hf_temp_err_t::TEMP_SUCCESS if registration successful, error code otherwise
     * @note This is called automatically during Initialize() to ensure onboard sensors are always available
     */
    [[nodiscard]] hf_temp_err_t RegisterOnboardTemperatureSensors() noexcept;
    
    /**
     * @brief Unregister a temperature sensor.
     * @param name Sensor name identifier
     * @return hf_temp_err_t::TEMP_SUCCESS if unregistration successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t UnregisterSensor(std::string_view name) noexcept;
    
    /**
     * @brief Check if a sensor is registered.
     * @param name Sensor name identifier
     * @return true if sensor is registered, false otherwise
     */
    [[nodiscard]] bool IsSensorRegistered(std::string_view name) const noexcept;
    
    /**
     * @brief Get the number of registered sensors.
     * @return Number of registered sensors
     */
    [[nodiscard]] uint32_t GetSensorCount() const noexcept;
    
    /**
     * @brief Get sensor information by name.
     * @param name Sensor name identifier
     * @return Pointer to sensor info if found, nullptr otherwise
     */
    [[nodiscard]] const TempSensorInfo* GetSensorInfo(std::string_view name) const noexcept;
    
    /**
     * @brief Get sensor information by index.
     * @param index Sensor index (0-based)
     * @return Pointer to sensor info if valid index, nullptr otherwise
     */
    [[nodiscard]] const TempSensorInfo* GetSensorInfoByIndex(uint32_t index) const noexcept;
    
    /**
     * @brief Get all registered sensor names.
     * @return Vector of sensor names
     */
    [[nodiscard]] std::vector<std::string_view> GetSensorNames() const noexcept;
    
    //==========================================================================
    // CORE TEMPERATURE OPERATIONS (STRING-BASED)
    //==========================================================================
    
    /**
     * @brief Read temperature in Celsius from a sensor.
     * @param sensor_name Sensor name identifier
     * @param temperature_celsius Pointer to store temperature value
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureCelsius(std::string_view sensor_name, float* temperature_celsius) noexcept;
    
    /**
     * @brief Read temperature in Fahrenheit from a sensor.
     * @param sensor_name Sensor name identifier
     * @param temperature_fahrenheit Pointer to store temperature value
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureFahrenheit(std::string_view sensor_name, float* temperature_fahrenheit) noexcept;
    
    /**
     * @brief Read temperature in Kelvin from a sensor.
     * @param sensor_name Sensor name identifier
     * @param temperature_kelvin Pointer to store temperature value
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureKelvin(std::string_view sensor_name, float* temperature_kelvin) noexcept;
    
    /**
     * @brief Read temperature with full information from a sensor.
     * @param sensor_name Sensor name identifier
     * @param reading Pointer to store complete reading information
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperature(std::string_view sensor_name, hf_temp_reading_t* reading) noexcept;
    
    /**
     * @brief Read temperature in specified unit from a sensor.
     * @param sensor_name Sensor name identifier
     * @param temperature Pointer to store temperature value
     * @param unit Desired temperature unit
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureUnit(std::string_view sensor_name, float* temperature, hf_temp_unit_t unit) noexcept;
    
    //==========================================================================
    // CORE TEMPERATURE OPERATIONS (INDEX-BASED)
    //==========================================================================
    
    /**
     * @brief Read temperature in Celsius from a sensor by index.
     * @param index Sensor index (0-based)
     * @param temperature_celsius Pointer to store temperature value
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureCelsiusByIndex(uint32_t index, float* temperature_celsius) noexcept;
    
    /**
     * @brief Read temperature in Fahrenheit from a sensor by index.
     * @param index Sensor index (0-based)
     * @param temperature_fahrenheit Pointer to store temperature value
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureFahrenheitByIndex(uint32_t index, float* temperature_fahrenheit) noexcept;
    
    /**
     * @brief Read temperature in Kelvin from a sensor by index.
     * @param index Sensor index (0-based)
     * @param temperature_kelvin Pointer to store temperature value
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureKelvinByIndex(uint32_t index, float* temperature_kelvin) noexcept;
    
    /**
     * @brief Read temperature with full information from a sensor by index.
     * @param index Sensor index (0-based)
     * @param reading Pointer to store complete reading information
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureByIndex(uint32_t index, hf_temp_reading_t* reading) noexcept;
    
    //==========================================================================
    // BATCH OPERATIONS
    //==========================================================================
    
    /**
     * @brief Perform batch temperature reading operation.
     * @param operation Batch operation configuration
     * @return Batch operation results
     */
    [[nodiscard]] TempBatchResult PerformBatchRead(const TempBatchOperation& operation) noexcept;
    
    /**
     * @brief Read temperatures from multiple sensors in Celsius.
     * @param sensor_names Vector of sensor names
     * @param temperatures_celsius Vector to store temperature values
     * @return hf_temp_err_t::TEMP_SUCCESS if all successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadMultipleTemperaturesCelsius(const std::vector<std::string_view>& sensor_names,
                                                                std::vector<float>& temperatures_celsius) noexcept;
    
    /**
     * @brief Read temperatures from multiple sensors in Fahrenheit.
     * @param sensor_names Vector of sensor names
     * @param temperatures_fahrenheit Vector to store temperature values
     * @return hf_temp_err_t::TEMP_SUCCESS if all successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadMultipleTemperaturesFahrenheit(const std::vector<std::string_view>& sensor_names,
                                                                   std::vector<float>& temperatures_fahrenheit) noexcept;
    
    //==========================================================================
    // ADVANCED FEATURES (STRING-BASED)
    //==========================================================================
    
    /**
     * @brief Set temperature thresholds for a sensor.
     * @param sensor_name Sensor name identifier
     * @param low_threshold_celsius Low temperature threshold
     * @param high_threshold_celsius High temperature threshold
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t SetThresholds(std::string_view sensor_name, 
                                              float low_threshold_celsius, 
                                              float high_threshold_celsius) noexcept;
    
    /**
     * @brief Get temperature thresholds for a sensor.
     * @param sensor_name Sensor name identifier
     * @param low_threshold_celsius Pointer to store low threshold
     * @param high_threshold_celsius Pointer to store high threshold
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t GetThresholds(std::string_view sensor_name,
                                              float* low_threshold_celsius,
                                              float* high_threshold_celsius) const noexcept;
    
    /**
     * @brief Enable threshold monitoring for a sensor.
     * @param sensor_name Sensor name identifier
     * @param callback Callback function for threshold events
     * @param user_data User data to pass to callback
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t EnableThresholdMonitoring(std::string_view sensor_name,
                                                          hf_temp_threshold_callback_t callback,
                                                          void* user_data) noexcept;
    
    /**
     * @brief Disable threshold monitoring for a sensor.
     * @param sensor_name Sensor name identifier
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t DisableThresholdMonitoring(std::string_view sensor_name) noexcept;
    
    /**
     * @brief Start continuous monitoring for a sensor.
     * @param sensor_name Sensor name identifier
     * @param sample_rate_hz Sampling rate in Hz
     * @param callback Callback function for each reading
     * @param user_data User data to pass to callback
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t StartContinuousMonitoring(std::string_view sensor_name,
                                                          hf_u32_t sample_rate_hz,
                                                          hf_temp_reading_callback_t callback,
                                                          void* user_data) noexcept;
    
    /**
     * @brief Stop continuous monitoring for a sensor.
     * @param sensor_name Sensor name identifier
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t StopContinuousMonitoring(std::string_view sensor_name) noexcept;
    
    /**
     * @brief Check if continuous monitoring is active for a sensor.
     * @param sensor_name Sensor name identifier
     * @return true if monitoring is active, false otherwise
     */
    [[nodiscard]] bool IsMonitoringActive(std::string_view sensor_name) const noexcept;
    
    /**
     * @brief Calibrate a sensor.
     * @param sensor_name Sensor name identifier
     * @param reference_temperature_celsius Known reference temperature
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t Calibrate(std::string_view sensor_name, float reference_temperature_celsius) noexcept;
    
    /**
     * @brief Set calibration offset for a sensor.
     * @param sensor_name Sensor name identifier
     * @param offset_celsius Calibration offset in Celsius
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t SetCalibrationOffset(std::string_view sensor_name, float offset_celsius) noexcept;
    
    /**
     * @brief Get calibration offset for a sensor.
     * @param sensor_name Sensor name identifier
     * @param offset_celsius Pointer to store calibration offset
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t GetCalibrationOffset(std::string_view sensor_name, float* offset_celsius) const noexcept;
    
    /**
     * @brief Reset calibration for a sensor.
     * @param sensor_name Sensor name identifier
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ResetCalibration(std::string_view sensor_name) noexcept;
    
    //==========================================================================
    // INFORMATION AND DIAGNOSTICS
    //==========================================================================
    
    /**
     * @brief Get sensor information for a specific sensor.
     * @param sensor_name Sensor name identifier
     * @param info Pointer to store sensor information
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t GetSensorInfo(std::string_view sensor_name, hf_temp_sensor_info_t* info) const noexcept;
    
    /**
     * @brief Get sensor capabilities for a specific sensor.
     * @param sensor_name Sensor name identifier
     * @return Capabilities flags (hf_temp_capabilities_t)
     */
    [[nodiscard]] hf_u32_t GetSensorCapabilities(std::string_view sensor_name) const noexcept;
    
    /**
     * @brief Check if a sensor supports a specific capability.
     * @param sensor_name Sensor name identifier
     * @param capability Capability to check
     * @return true if supported, false otherwise
     */
    [[nodiscard]] bool HasSensorCapability(std::string_view sensor_name, hf_temp_capabilities_t capability) const noexcept;
    
    /**
     * @brief Get statistics for a specific sensor.
     * @param sensor_name Sensor name identifier
     * @param statistics Reference to statistics structure to fill
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t GetSensorStatistics(std::string_view sensor_name, hf_temp_statistics_t& statistics) noexcept;
    
    /**
     * @brief Get diagnostics for a specific sensor.
     * @param sensor_name Sensor name identifier
     * @param diagnostics Reference to diagnostics structure to fill
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t GetSensorDiagnostics(std::string_view sensor_name, hf_temp_diagnostics_t& diagnostics) noexcept;
    
    /**
     * @brief Get system-wide diagnostics.
     * @param diagnostics Reference to system diagnostics structure to fill
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t GetSystemDiagnostics(TempSystemDiagnostics& diagnostics) noexcept;
    
    /**
     * @brief Reset statistics for a specific sensor.
     * @param sensor_name Sensor name identifier
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ResetSensorStatistics(std::string_view sensor_name) noexcept;
    
    /**
     * @brief Reset diagnostics for a specific sensor.
     * @param sensor_name Sensor name identifier
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ResetSensorDiagnostics(std::string_view sensor_name) noexcept;
    
    //==========================================================================
    // UTILITY FUNCTIONS
    //==========================================================================
    
    /**
     * @brief Get sensors by type.
     * @param sensor_type Type of sensors to find
     * @return Vector of sensor names of the specified type
     */
    [[nodiscard]] std::vector<std::string_view> GetSensorsByType(hf_temp_sensor_type_t sensor_type) const noexcept;
    
    /**
     * @brief Get sensors with specific capability.
     * @param capability Capability to check for
     * @return Vector of sensor names with the specified capability
     */
    [[nodiscard]] std::vector<std::string_view> GetSensorsWithCapability(hf_temp_capabilities_t capability) const noexcept;
    
    /**
     * @brief Find sensors within temperature range.
     * @param min_temp_celsius Minimum temperature
     * @param max_temp_celsius Maximum temperature
     * @return Vector of sensor names within the specified range
     */
    [[nodiscard]] std::vector<std::string_view> FindSensorsInRange(float min_temp_celsius, float max_temp_celsius) const noexcept;
    
    /**
     * @brief Get system-wide temperature statistics.
     * @param min_temp_celsius Pointer to store minimum temperature
     * @param max_temp_celsius Pointer to store maximum temperature
     * @param avg_temp_celsius Pointer to store average temperature
     * @return hf_temp_err_t::TEMP_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t GetSystemTemperatureStats(float* min_temp_celsius, 
                                                          float* max_temp_celsius, 
                                                          float* avg_temp_celsius) noexcept;

private:
    //==========================================================================
    // PRIVATE CONSTRUCTOR AND DESTRUCTOR
    //==========================================================================
    
    TemperatureManager() noexcept;
    ~TemperatureManager() noexcept;
    
    // Disable copy and assignment
    TemperatureManager(const TemperatureManager&) = delete;
    TemperatureManager& operator=(const TemperatureManager&) = delete;
    
    //==========================================================================
    // PRIVATE HELPER METHODS
    //==========================================================================
    
    /**
     * @brief Initialize the temperature manager system.
     * @return hf_temp_err_t::TEMP_SUCCESS if initialization successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t Initialize() noexcept;
    
    /**
     * @brief Deinitialize the temperature manager system.
     * @return hf_temp_err_t::TEMP_SUCCESS if deinitialization successful, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t Deinitialize() noexcept;
    
    /**
     * @brief Find sensor by name.
     * @param name Sensor name identifier
     * @return Iterator to sensor info if found, end() otherwise
     */
    [[nodiscard]] std::vector<TempSensorInfo>::iterator FindSensor(std::string_view name) noexcept;
    
    /**
     * @brief Find sensor by name (const version).
     * @param name Sensor name identifier
     * @return Iterator to sensor info if found, end() otherwise
     */
    [[nodiscard]] std::vector<TempSensorInfo>::const_iterator FindSensor(std::string_view name) const noexcept;
    
    /**
     * @brief Update sensor access statistics.
     * @param sensor_info Pointer to sensor info
     * @param operation_successful Whether the operation was successful
     */
    void UpdateSensorStatistics(TempSensorInfo* sensor_info, bool operation_successful) noexcept;
    
    /**
     * @brief Update system diagnostics.
     */
    void UpdateSystemDiagnostics() noexcept;
    
    //==========================================================================
    // PRIVATE MEMBER VARIABLES
    //==========================================================================
    
    mutable std::mutex mutex_;                           ///< Mutex for thread safety
    std::atomic<bool> initialized_;                      ///< Initialization status
    std::vector<TempSensorInfo> sensors_;                ///< Registered sensors
    std::unordered_map<std::string_view, uint32_t> sensor_map_; ///< Name to index mapping
    std::atomic<uint8_t> next_sensor_id_;                ///< Next available sensor ID
    TempSystemDiagnostics system_diagnostics_;           ///< System diagnostics
    std::atomic<uint64_t> system_start_time_ms_;         ///< System start time
};

#endif // COMPONENT_HANDLER_TEMPERATURE_MANAGER_H_ 