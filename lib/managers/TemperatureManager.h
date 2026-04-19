/**
 * @file TemperatureManager.h
 * @brief Temperature sensor management for the HardFOC Vortex V1 platform.
 *
 * @details The Vortex board has three temperature sources:
 *            1. ESP32 internal temperature sensor (on-chip)
 *            2. NTC thermistor via TMC9660 AIN3 ADC channel
 *            3. TMC9660 internal chip temperature sensor
 *
 *          Fixed-size array of TempEntry structs, no heap vectors or maps.
 *          Sensor drivers heap-allocated via unique_ptr (singleton lifetime).
 *
 * @version 2.0
 */

#ifndef VORTEX_TEMPERATURE_MANAGER_H_
#define VORTEX_TEMPERATURE_MANAGER_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string_view>

#include "base/BaseTemperature.h"
#include "mcu/esp32/EspTemperature.h"
#include "handlers/ntc/NtcTemperatureHandler.h"
#include "RtosMutex.h"

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class Tmc9660Handler;
class MotorController;
class AdcManager;

//==============================================================================
// TMC9660 TEMPERATURE WRAPPER
//==============================================================================

/**
 * @brief Adapter wrapping Tmc9660Handler's internal temperature into BaseTemperature.
 */
class Tmc9660TemperatureWrapper : public BaseTemperature {
public:
    /**
     * @brief Construct the wrapper around an existing Tmc9660Handler.
     * @param handler Reference to the TMC9660 handler (must outlive this wrapper).
     */
    explicit Tmc9660TemperatureWrapper(Tmc9660Handler& handler) noexcept;
    ~Tmc9660TemperatureWrapper() override = default;

    /** @brief Initialise the underlying TMC9660 temperature path. */
    bool Initialize() noexcept override;

    /** @brief Deinitialise the underlying TMC9660 temperature path. */
    bool Deinitialize() noexcept override;

    /**
     * @brief Read the TMC9660 chip temperature in Celsius.
     * @param temperature_celsius Pointer to store the reading.
     * @return TEMP_SUCCESS on success.
     */
    hf_temp_err_t ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept override;

    /**
     * @brief Retrieve sensor information for the TMC9660.
     * @param info Pointer to the info struct to fill.
     * @return TEMP_SUCCESS on success.
     */
    hf_temp_err_t GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept override;

    /**
     * @brief Get capability flags for this sensor.
     * @return Bitmask of supported capabilities.
     */
    hf_u32_t GetCapabilities() const noexcept override;

private:
    Tmc9660Handler& handler_;
};

//==============================================================================
// SENSOR TYPE ENUM
//==============================================================================

/**
 * @brief Identifies the physical temperature sensor type on the Vortex board.
 */
enum class VortexTempSensorType : uint8_t {
    ESP32_INTERNAL = 0, ///< ESP32 on-chip temperature sensor
    NTC_THERMISTOR = 1, ///< External NTC thermistor via TMC9660 AIN3
    TMC9660_CHIP   = 2, ///< TMC9660 internal chip temperature
    SENSOR_COUNT        ///< Number of sensor types (not a valid sensor)
};

/**
 * @brief Convert VortexTempSensorType to a human-readable string.
 * @param t Sensor type to convert.
 * @return Null-terminated string representation.
 */
constexpr const char* VortexTempSensorTypeToString(VortexTempSensorType t) noexcept {
    switch (t) {
        case VortexTempSensorType::ESP32_INTERNAL: return "ESP32 Internal";
        case VortexTempSensorType::NTC_THERMISTOR: return "NTC Thermistor";
        case VortexTempSensorType::TMC9660_CHIP:   return "TMC9660 Chip";
        default:                                    return "Unknown";
    }
}

//==============================================================================
// TEMPERATURE ENTRY
//==============================================================================

/**
 * @brief A single temperature sensor entry in the fixed-size registry.
 */
struct TempEntry {
    std::string_view name;              ///< Human-readable name
    VortexTempSensorType sensor_type{VortexTempSensorType::SENSOR_COUNT}; ///< Physical sensor type
    bool registered{false};             ///< Whether this entry is active
    float last_reading_celsius{0.0f};   ///< Most recent cached reading in °C
    uint32_t read_count{0};             ///< Cumulative successful reads
    uint32_t error_count{0};            ///< Cumulative failed reads
};

//==============================================================================
// DIAGNOSTICS STRUCTURE
//==============================================================================

/**
 * @brief System-level temperature diagnostics (compatible with old API).
 */
/**
 * @brief Temperature subsystem diagnostics snapshot.
 */
struct TempSystemDiagnostics {
    bool system_healthy;                    ///< True when all registered sensors are responsive
    uint32_t total_sensors_registered;      ///< Number of sensors currently registered
    uint32_t sensors_by_type[8];            ///< Count of sensors per VortexTempSensorType index
    uint32_t total_operations;              ///< Cumulative read attempts across all sensors
    uint32_t successful_operations;         ///< Cumulative successful reads
    uint32_t failed_operations;             ///< Cumulative failed reads
    uint32_t communication_errors;          ///< Errors attributed to bus communication failures
    uint32_t hardware_errors;               ///< Errors attributed to hardware faults
    uint64_t system_uptime_ms;              ///< Milliseconds since temperature subsystem init
    hf_temp_err_t last_error;               ///< Most recent error code from any operation
    float system_min_temp_celsius;          ///< Minimum temperature across all sensors (°C)
    float system_max_temp_celsius;          ///< Maximum temperature across all sensors (°C)
    float system_avg_temp_celsius;          ///< Average temperature across all sensors (°C)
};

//==============================================================================
// TEMPERATURE MANAGER
//==============================================================================

/**
 * @class TemperatureManager
 * @brief Singleton manager for all temperature sensors on Vortex V1.
 *
 * @details Manages three temperature sources: ESP32 internal sensor,
 *          NTC thermistor via TMC9660 AIN3, and TMC9660 internal chip
 *          temperature. Sensors are registered in a fixed-size array;
 *          readings are returned in Celsius or Fahrenheit.
 *
 * @note Call EnsureInitialized() once before any reads. All public methods
 *       are thread-safe (guarded by an internal RTOS mutex).
 */
class TemperatureManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================

    /** @brief Return the singleton TemperatureManager instance. */
    static TemperatureManager& GetInstance() noexcept;

    TemperatureManager(const TemperatureManager&) = delete;
    TemperatureManager& operator=(const TemperatureManager&) = delete;
    TemperatureManager(TemperatureManager&&) = delete;
    TemperatureManager& operator=(TemperatureManager&&) = delete;

    /**
     * @brief Initialise the temperature subsystem if not already initialised.
     * @return true on success or if already initialised.
     */
    [[nodiscard]] bool EnsureInitialized() noexcept;

    /**
     * @brief Release all temperature sensor resources and reset to uninitialised state.
     * @return true on success.
     */
    [[nodiscard]] bool Deinitialize() noexcept;

    /**
     * @brief Check whether the temperature subsystem has been initialised.
     * @return true if initialised and ready for reads.
     */
    [[nodiscard]] bool IsInitialized() const noexcept { return initialized_.load(std::memory_order_acquire); }

    //==========================================================================
    // TEMPERATURE READING
    //==========================================================================

    /**
     * @brief Read temperature in Celsius from a named sensor.
     * @param name  "ESP32_INTERNAL", "NTC_THERMISTOR", or "MOTOR_TEMP"
     * @param temperature_celsius  Reference to store temperature value
     * @return TEMP_SUCCESS on success, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureCelsius(std::string_view name, float& temperature_celsius) noexcept;

    /**
     * @brief Read temperature in Fahrenheit from a named sensor.
     * @param name                  Sensor name ("ESP32_INTERNAL", "NTC_THERMISTOR", or "MOTOR_TEMP").
     * @param temperature_fahrenheit Output temperature in °F.
     * @return TEMP_SUCCESS on success.
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureFahrenheit(std::string_view name, float& temperature_fahrenheit) noexcept;

    //==========================================================================
    // CONVENIENCE READING — cross-HAL compatible
    //==========================================================================

    /**
     * @brief Convenience: read the ESP32 internal temperature sensor.
     * @param temperature_c Output temperature in °C.
     * @return TEMP_SUCCESS on success.
     */
    [[nodiscard]] hf_temp_err_t ReadInternalTemperature(float& temperature_c) noexcept;

    /**
     * @brief Convenience: read the NTC thermistor temperature.
     * @param temperature_c Output temperature in °C.
     * @return TEMP_SUCCESS on success.
     */
    [[nodiscard]] hf_temp_err_t ReadNtcTemperature(float& temperature_c) noexcept;

    /**
     * @brief Convenience: read the TMC9660 motor controller temperature.
     * @param temperature_c Output temperature in °C.
     * @return TEMP_SUCCESS on success.
     */
    [[nodiscard]] hf_temp_err_t ReadMotorTemperature(float& temperature_c) noexcept;

    /**
     * @brief Read all sensors and compute min, max, and average temperatures.
     * @param[out] min_c Minimum temperature across all sensors (°C).
     * @param[out] max_c Maximum temperature across all sensors (°C).
     * @param[out] avg_c Average temperature across all sensors (°C).
     * @return TEMP_SUCCESS on success.
     */
    [[nodiscard]] hf_temp_err_t GetSystemTemperatureStats(float& min_c, float& max_c, float& avg_c) noexcept;

    //==========================================================================
    // SENSOR INFO
    //==========================================================================

    /** @brief Get the number of registered temperature sensors. */
    [[nodiscard]] size_t GetSensorCount() const noexcept { return registered_count_; }

    /**
     * @brief Look up a sensor entry by name.
     * @param name Sensor name.
     * @return Pointer to the TempEntry, or nullptr if not found.
     */
    [[nodiscard]] const TempEntry* GetSensorEntry(std::string_view name) const noexcept;

    //==========================================================================
    // DIAGNOSTICS
    //==========================================================================

    /**
     * @brief Populate a TempSystemDiagnostics snapshot.
     * @param[out] diagnostics Diagnostics structure to fill.
     * @return TEMP_SUCCESS on success.
     */
    [[nodiscard]] hf_temp_err_t GetSystemDiagnostics(TempSystemDiagnostics& diagnostics) const noexcept;

    /**
     * @brief Get the most recent error code from any temperature operation.
     * @return Last hf_temp_err_t set by any API call
     */
    [[nodiscard]] hf_temp_err_t GetLastError() const noexcept { return last_error_.load(std::memory_order_acquire); }

    /** @brief Log temperature sensor statistics and health info to the console. */
    void DumpStatistics() const noexcept;

private:
    TemperatureManager() noexcept = default;
    ~TemperatureManager();

    [[nodiscard]] bool Initialize() noexcept;
    bool InitializeEsp32Internal() noexcept;
    bool InitializeNtcThermistor() noexcept;
    bool InitializeTmc9660Chip() noexcept;

    [[nodiscard]] size_t FindByName(std::string_view name) const noexcept;

    //==========================================================================
    // STORAGE
    //==========================================================================

    static constexpr size_t kMaxSensors = static_cast<size_t>(VortexTempSensorType::SENSOR_COUNT);
    static constexpr size_t kInvalidIndex = SIZE_MAX;

    std::array<TempEntry, kMaxSensors> entries_{};
    size_t registered_count_{0};
    uint64_t init_time_ms_{0};

    // Named sensor driver instances
    std::unique_ptr<EspTemperature>          esp32_temp_;
    std::unique_ptr<NtcTemperatureHandler>   ntc_handler_;
    std::unique_ptr<Tmc9660TemperatureWrapper> tmc9660_temp_;

    std::atomic<bool> initialized_{false};
    std::atomic<hf_temp_err_t> last_error_{hf_temp_err_t::TEMP_SUCCESS};
    mutable RtosMutex mutex_;
};

//==============================================================================
// CONVENIENCE
//==============================================================================

/**
 * @brief Convenience accessor — equivalent to TemperatureManager::GetInstance().
 * @return Reference to the singleton TemperatureManager.
 */
[[nodiscard]] inline TemperatureManager& GetTemperatureManager() noexcept {
    return TemperatureManager::GetInstance();
}

#endif // VORTEX_TEMPERATURE_MANAGER_H_
