/**
 * @file TemperatureManager.cpp
 * @brief Implementation of the Vortex V1 temperature manager.
 *
 * @details Three temperature sources:
 *          1. ESP32 internal (EspTemperature)
 *          2. NTC thermistor via TMC9660 AIN3 (NtcTemperatureHandler)
 *          3. TMC9660 chip internal (Tmc9660TemperatureWrapper)
 *
 * @version 2.0
 */

#include "TemperatureManager.h"
#include "AdcManager.h"
#include "MotorController.h"
#include "handlers/tmc9660/Tmc9660Handler.h"
#include "handlers/logger/Logger.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

static constexpr const char* TAG = "VortexTemp";

//==============================================================================
// TMC9660 TEMPERATURE WRAPPER
//==============================================================================

Tmc9660TemperatureWrapper::Tmc9660TemperatureWrapper(Tmc9660Handler& handler) noexcept
    : handler_(handler) {}

bool Tmc9660TemperatureWrapper::Initialize() noexcept {
    if (IsInitialized()) return true;

    if (!handler_.IsDriverReady()) {
        Logger::GetInstance().Error(TAG, "TMC9660 handler not ready");
        return false;
    }

    Tmc9660Handler::Temperature& tw = handler_.temperature();
    if (!tw.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "TMC9660 temperature init failed");
        return false;
    }

    initialized_ = true;
    current_state_ = HF_TEMP_STATE_INITIALIZED;
    return true;
}

bool Tmc9660TemperatureWrapper::Deinitialize() noexcept {
    if (!IsInitialized()) return true;
    initialized_ = false;
    current_state_ = HF_TEMP_STATE_UNINITIALIZED;
    return true;
}

hf_temp_err_t Tmc9660TemperatureWrapper::ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept {
    if (!temperature_celsius) return TEMP_ERR_NULL_POINTER;
    if (!IsInitialized())    return TEMP_ERR_NOT_INITIALIZED;
    if (!handler_.IsDriverReady()) return TEMP_ERR_RESOURCE_UNAVAILABLE;

    return handler_.temperature().ReadTemperatureCelsius(temperature_celsius);
}

hf_temp_err_t Tmc9660TemperatureWrapper::GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept {
    if (!info)            return TEMP_ERR_NULL_POINTER;
    if (!IsInitialized()) return TEMP_ERR_NOT_INITIALIZED;
    return handler_.temperature().GetSensorInfo(info);
}

hf_u32_t Tmc9660TemperatureWrapper::GetCapabilities() const noexcept {
    if (!IsInitialized()) return HF_TEMP_CAP_NONE;
    return handler_.temperature().GetCapabilities();
}

//==============================================================================
// SINGLETON
//==============================================================================

TemperatureManager& TemperatureManager::GetInstance() noexcept {
    static TemperatureManager instance;
    return instance;
}

TemperatureManager::~TemperatureManager() {
    // unique_ptrs clean up automatically
}

//==============================================================================
// INITIALIZATION
//==============================================================================

bool TemperatureManager::EnsureInitialized() noexcept {
    if (initialized_.load(std::memory_order_acquire)) return true;
    MutexLockGuard lock(mutex_);
    if (initialized_.load(std::memory_order_acquire)) return true;
    bool ok = Initialize();
    initialized_.store(ok, std::memory_order_release);
    return ok;
}

bool TemperatureManager::Initialize() noexcept {
    Logger::GetInstance().Info(TAG, "Initializing Vortex temperature manager");

    init_time_ms_ = os_get_elapsed_time_msec();
    registered_count_ = 0;

    bool esp_ok  = InitializeEsp32Internal();
    bool ntc_ok  = InitializeNtcThermistor();
    bool tmc_ok  = InitializeTmc9660Chip();

    Logger::GetInstance().Info(TAG, "Temperature manager initialized (ESP32=%s, NTC=%s, TMC9660=%s, total=%zu)",
                              esp_ok ? "OK" : "FAIL",
                              ntc_ok ? "OK" : "FAIL",
                              tmc_ok ? "OK" : "FAIL",
                              registered_count_);

    return registered_count_ > 0;
}

bool TemperatureManager::InitializeEsp32Internal() noexcept {
    esp32_temp_ = std::make_unique<EspTemperature>();
    if (!esp32_temp_) {
        Logger::GetInstance().Error(TAG, "Failed to allocate EspTemperature");
        return false;
    }

    if (!esp32_temp_->EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "EspTemperature init failed");
        esp32_temp_.reset();
        return false;
    }

    auto& entry = entries_[registered_count_++];
    entry.name = "ESP32_INTERNAL";
    entry.sensor_type = VortexTempSensorType::ESP32_INTERNAL;
    entry.registered = true;

    Logger::GetInstance().Info(TAG, "  Registered ESP32 internal temperature sensor");
    return true;
}

bool TemperatureManager::InitializeNtcThermistor() noexcept {
    auto& adc = AdcManager::GetInstance();
    if (!adc.EnsureInitialized()) {
        Logger::GetInstance().Warn(TAG, "ADC manager not available — skipping NTC");
        return false;
    }

    BaseAdc* adc_driver = adc.Get("ADC_TMC9660_AIN3");
    if (!adc_driver) {
        Logger::GetInstance().Warn(TAG, "TMC9660 AIN3 ADC channel not available — skipping NTC");
        return false;
    }

    // NTC handler config: 10k NTC, 10k pullup, 3.3V, B=3950
    ntc_temp_handler_config_t ntc_cfg = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    ntc_cfg.enable_threshold_monitoring = true;
    ntc_cfg.low_threshold_celsius = 0.0f;
    ntc_cfg.high_threshold_celsius = 85.0f;

    ntc_handler_ = std::make_unique<NtcTemperatureHandler>(adc_driver, ntc_cfg);
    if (!ntc_handler_) {
        Logger::GetInstance().Error(TAG, "Failed to allocate NTC handler");
        return false;
    }

    if (!ntc_handler_->EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "NTC handler init failed");
        ntc_handler_.reset();
        return false;
    }

    auto& entry = entries_[registered_count_++];
    entry.name = "NTC_THERMISTOR";
    entry.sensor_type = VortexTempSensorType::NTC_THERMISTOR;
    entry.registered = true;

    Logger::GetInstance().Info(TAG, "  Registered NTC thermistor (TMC9660 AIN3)");
    return true;
}

bool TemperatureManager::InitializeTmc9660Chip() noexcept {
    auto& mc = MotorController::GetInstance();
    if (!mc.EnsureInitialized()) {
        Logger::GetInstance().Warn(TAG, "MotorController not available — skipping TMC9660 temp");
        return false;
    }

    Tmc9660Handler* handler = mc.handler(MotorController::ONBOARD_TMC9660_INDEX);
    if (!handler || !handler->IsDriverReady()) {
        Logger::GetInstance().Warn(TAG, "TMC9660 handler not ready — skipping chip temp");
        return false;
    }

    tmc9660_temp_ = std::make_unique<Tmc9660TemperatureWrapper>(*handler);
    if (!tmc9660_temp_->Initialize()) {
        Logger::GetInstance().Error(TAG, "TMC9660 temp wrapper init failed");
        tmc9660_temp_.reset();
        return false;
    }

    auto& entry = entries_[registered_count_++];
    entry.name = "MOTOR_TEMP";
    entry.sensor_type = VortexTempSensorType::TMC9660_CHIP;
    entry.registered = true;

    Logger::GetInstance().Info(TAG, "  Registered TMC9660 chip temperature sensor");
    return true;
}

//==============================================================================
// LOOKUP
//==============================================================================

size_t TemperatureManager::FindByName(std::string_view name) const noexcept {
    for (size_t i = 0; i < registered_count_; ++i) {
        if (entries_[i].registered && entries_[i].name == name) return i;
    }
    return kInvalidIndex;
}

//==============================================================================
// TEMPERATURE READING
//==============================================================================

hf_temp_err_t TemperatureManager::ReadTemperatureCelsius(std::string_view name,
                                                         float* temperature_celsius) noexcept {
    if (!temperature_celsius) return TEMP_ERR_NULL_POINTER;
    if (!initialized_.load(std::memory_order_acquire)) return TEMP_ERR_NOT_INITIALIZED;

    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return TEMP_ERR_SENSOR_NOT_AVAILABLE;

    MutexLockGuard lock(mutex_);
    auto& entry = entries_[idx];
    BaseTemperature* sensor = nullptr;

    switch (entry.sensor_type) {
        case VortexTempSensorType::ESP32_INTERNAL: sensor = esp32_temp_.get();   break;
        case VortexTempSensorType::NTC_THERMISTOR: sensor = ntc_handler_.get();  break;
        case VortexTempSensorType::TMC9660_CHIP:   sensor = tmc9660_temp_.get(); break;
        default: return TEMP_ERR_SENSOR_NOT_AVAILABLE;
    }
    if (!sensor) return TEMP_ERR_SENSOR_NOT_AVAILABLE;

    hf_temp_err_t err = sensor->ReadTemperatureCelsius(temperature_celsius);
    if (err == TEMP_SUCCESS) {
        entry.last_reading_celsius = *temperature_celsius;
        entry.read_count++;
    } else {
        entry.error_count++;
    }
    return err;
}

hf_temp_err_t TemperatureManager::ReadTemperatureFahrenheit(std::string_view name,
                                                            float* temperature_fahrenheit) noexcept {
    if (!temperature_fahrenheit) return TEMP_ERR_NULL_POINTER;
    float celsius = 0.0f;
    hf_temp_err_t err = ReadTemperatureCelsius(name, &celsius);
    if (err == TEMP_SUCCESS) {
        *temperature_fahrenheit = celsius * 1.8f + 32.0f;
    }
    return err;
}

//==============================================================================
// DIAGNOSTICS
//==============================================================================

hf_temp_err_t TemperatureManager::GetSystemDiagnostics(TempSystemDiagnostics& d) noexcept {
    if (!initialized_.load(std::memory_order_acquire)) {
        return TEMP_ERR_NOT_INITIALIZED;
    }

    MutexLockGuard lock(mutex_);

    d = {};
    d.total_sensors_registered = static_cast<uint32_t>(registered_count_);
    d.system_min_temp_celsius = FLT_MAX;
    d.system_max_temp_celsius = -FLT_MAX;
    d.system_avg_temp_celsius = 0.0f;

    float sum = 0.0f;
    uint32_t valid = 0;

    for (size_t i = 0; i < registered_count_; ++i) {
        const auto& e = entries_[i];
        d.total_operations += e.read_count + e.error_count;
        d.successful_operations += e.read_count;
        d.failed_operations += e.error_count;

        uint8_t type_idx = static_cast<uint8_t>(e.sensor_type);
        if (type_idx < std::size(d.sensors_by_type)) {
            d.sensors_by_type[type_idx]++;
        }

        if (e.read_count > 0) {
            sum += e.last_reading_celsius;
            if (e.last_reading_celsius < d.system_min_temp_celsius)
                d.system_min_temp_celsius = e.last_reading_celsius;
            if (e.last_reading_celsius > d.system_max_temp_celsius)
                d.system_max_temp_celsius = e.last_reading_celsius;
            ++valid;
        }
    }

    if (valid > 0) {
        d.system_avg_temp_celsius = sum / static_cast<float>(valid);
    } else {
        d.system_min_temp_celsius = 0.0f;
        d.system_max_temp_celsius = 0.0f;
    }

    d.system_uptime_ms = (init_time_ms_ > 0) ? (os_get_elapsed_time_msec() - init_time_ms_) : 0;
    d.system_healthy = (d.failed_operations == 0) && (registered_count_ > 0);
    d.last_error = TEMP_SUCCESS;

    return TEMP_SUCCESS;
}

void TemperatureManager::DumpStatistics() const noexcept {
    auto& log = Logger::GetInstance();
    log.Info(TAG, "=== Vortex Temperature Manager Statistics ===");
    log.Info(TAG, "  Sensors registered: %zu / %zu", registered_count_, kMaxSensors);

    for (size_t i = 0; i < registered_count_; ++i) {
        const auto& e = entries_[i];
        log.Info(TAG, "  [%.*s] type=%s last=%.2f°C reads=%u errs=%u",
                 static_cast<int>(e.name.size()), e.name.data(),
                 VortexTempSensorTypeToString(e.sensor_type),
                 static_cast<double>(e.last_reading_celsius),
                 e.read_count, e.error_count);
    }
    log.Info(TAG, "=== End Vortex Temperature Manager Statistics ===");
}
