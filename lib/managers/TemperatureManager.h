/**
 * @file TemperatureManager.h
 * @brief Temperature sensor management for the HardFOC Vortex V1 platform.
 *
 * @details The Vortex board has three temperature sources:
 *            1. ESP32-C6 internal temperature sensor (on-chip)
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
    explicit Tmc9660TemperatureWrapper(Tmc9660Handler& handler) noexcept;
    ~Tmc9660TemperatureWrapper() override = default;

    // BaseTemperature interface
    bool Initialize() noexcept override;
    bool Deinitialize() noexcept override;
    hf_temp_err_t ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept override;
    hf_temp_err_t GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept override;
    hf_u32_t GetCapabilities() const noexcept override;

private:
    Tmc9660Handler& handler_;
};

//==============================================================================
// SENSOR TYPE ENUM
//==============================================================================

enum class VortexTempSensorType : uint8_t {
    ESP32_INTERNAL = 0,
    NTC_THERMISTOR = 1,
    TMC9660_CHIP   = 2,
    SENSOR_COUNT
};

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

struct TempEntry {
    std::string_view name;
    VortexTempSensorType sensor_type{VortexTempSensorType::SENSOR_COUNT};
    bool registered{false};
    float last_reading_celsius{0.0f};
    uint32_t read_count{0};
    uint32_t error_count{0};
};

//==============================================================================
// DIAGNOSTICS STRUCTURE
//==============================================================================

/**
 * @brief System-level temperature diagnostics (compatible with old API).
 */
struct TempSystemDiagnostics {
    bool system_healthy;
    uint32_t total_sensors_registered;
    uint32_t sensors_by_type[8];
    uint32_t total_operations;
    uint32_t successful_operations;
    uint32_t failed_operations;
    uint32_t communication_errors;
    uint32_t hardware_errors;
    uint64_t system_uptime_ms;
    hf_temp_err_t last_error;
    float system_min_temp_celsius;
    float system_max_temp_celsius;
    float system_avg_temp_celsius;
};

//==============================================================================
// TEMPERATURE MANAGER
//==============================================================================

class TemperatureManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================

    static TemperatureManager& GetInstance() noexcept;

    TemperatureManager(const TemperatureManager&) = delete;
    TemperatureManager& operator=(const TemperatureManager&) = delete;
    TemperatureManager(TemperatureManager&&) = delete;
    TemperatureManager& operator=(TemperatureManager&&) = delete;

    [[nodiscard]] bool EnsureInitialized() noexcept;
    [[nodiscard]] bool IsInitialized() const noexcept { return initialized_.load(std::memory_order_acquire); }

    //==========================================================================
    // TEMPERATURE READING
    //==========================================================================

    /**
     * @brief Read temperature in Celsius from a named sensor.
     * @param name  "ESP32_INTERNAL", "NTC_THERMISTOR", or "MOTOR_TEMP"
     * @param temperature_celsius  Pointer to store temperature value
     * @return TEMP_SUCCESS on success, error code otherwise
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureCelsius(std::string_view name, float* temperature_celsius) noexcept;

    /**
     * @brief Read temperature in Fahrenheit.
     */
    [[nodiscard]] hf_temp_err_t ReadTemperatureFahrenheit(std::string_view name, float* temperature_fahrenheit) noexcept;

    //==========================================================================
    // SENSOR INFO
    //==========================================================================

    [[nodiscard]] size_t GetSensorCount() const noexcept { return registered_count_; }

    //==========================================================================
    // DIAGNOSTICS
    //==========================================================================

    [[nodiscard]] hf_temp_err_t GetSystemDiagnostics(TempSystemDiagnostics& diagnostics) noexcept;
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
    mutable RtosMutex mutex_;
};

//==============================================================================
// CONVENIENCE
//==============================================================================

[[nodiscard]] inline TemperatureManager& GetTemperatureManager() noexcept {
    return TemperatureManager::GetInstance();
}

#endif // VORTEX_TEMPERATURE_MANAGER_H_
