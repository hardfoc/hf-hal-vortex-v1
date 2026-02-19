/**
 * @file AdcManager.cpp
 * @brief Implementation of the Vortex V1 ADC manager.
 *
 * @details All 15 functional ADC channels are TMC9660-hosted. A single
 *          Tmc9660AdcWrapper serves all channels, routing by hardware
 *          channel ID at read time.
 *
 * @version 2.0
 */

#include "AdcManager.h"
#include "MotorController.h"
#include "handlers/tmc9660/Tmc9660AdcWrapper.h"
#include "handlers/logger/Logger.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"

static constexpr const char* TAG = "VortexAdc";

// Destructor defined here where Tmc9660AdcWrapper is a complete type
AdcManager::~AdcManager() = default;

//==============================================================================
// SINGLETON
//==============================================================================

AdcManager& AdcManager::GetInstance() noexcept {
    static AdcManager instance;
    return instance;
}

//==============================================================================
// INITIALIZATION
//==============================================================================

bool AdcManager::EnsureInitialized() noexcept {
    if (initialized_.load(std::memory_order_acquire)) return true;
    MutexLockGuard lock(mutex_);
    if (initialized_.load(std::memory_order_acquire)) return true;
    bool ok = Initialize();
    initialized_.store(ok, std::memory_order_release);
    return ok;
}

bool AdcManager::Initialize() noexcept {
    Logger::GetInstance().Info(TAG, "Initializing Vortex ADC manager");

    //==========================================================================
    // MotorController dependency — required for Tmc9660Handler access
    //==========================================================================

    motor_controller_ = &MotorController::GetInstance();
    if (!motor_controller_ || !motor_controller_->IsInitialized()) {
        Logger::GetInstance().Error(TAG, "MotorController not available or not initialized");
        return false;
    }

    Tmc9660Handler* handler = motor_controller_->handler(0);
    if (!handler) {
        Logger::GetInstance().Error(TAG, "TMC9660 handler not available from MotorController");
        return false;
    }

    //==========================================================================
    // Create single Tmc9660AdcWrapper — all channels route through this
    //==========================================================================

    tmc9660_adc_ = std::make_unique<Tmc9660AdcWrapper>(*handler);
    if (!tmc9660_adc_) {
        Logger::GetInstance().Error(TAG, "Failed to allocate Tmc9660AdcWrapper");
        return false;
    }

    //==========================================================================
    // Register channels from pincfg — TMC9660 channels only
    //==========================================================================

    for (size_t i = 0; i < HF_ADC_MAPPING_SIZE; ++i) {
        const auto& mapping = HF_ADC_MAPPING[i];
        const auto fc = static_cast<HfFunctionalAdcChannel>(mapping.functional_channel);
        const auto chip = static_cast<HfAdcChipType>(mapping.chip_type);

        // Only TMC9660 channels — ESP32 internal not used on Vortex
        if (chip != HfAdcChipType::TMC9660_CONTROLLER) {
            continue;
        }

        // Only unit 0 ADC unit 0 (single onboard TMC9660)
        if (mapping.chip_unit != 0 || mapping.adc_unit != 0) {
            continue;
        }

        if (registered_count_ >= kMaxChannels) {
            Logger::GetInstance().Warn(TAG, "Max ADC channels reached (%zu)", kMaxChannels);
            break;
        }

        auto& entry = entries_[registered_count_];
        entry.functional_channel = fc;
        entry.name = to_string(fc);
        entry.adc_unit = mapping.adc_unit;
        entry.hw_channel = mapping.physical_channel;
        entry.voltage_divider = mapping.voltage_divider;
        entry.driver = tmc9660_adc_.get();
        entry.registered = true;
        ++registered_count_;

        Logger::GetInstance().Info(TAG, "  Registered ADC: %.*s (ch=%u div=%.1f)",
                                  static_cast<int>(entry.name.size()), entry.name.data(),
                                  entry.hw_channel,
                                  static_cast<double>(entry.voltage_divider));
    }

    Logger::GetInstance().Info(TAG, "ADC manager initialized (%zu TMC9660 channels)", registered_count_);
    return registered_count_ > 0;
}

//==============================================================================
// LOOKUP
//==============================================================================

size_t AdcManager::FindByName(std::string_view name) const noexcept {
    for (size_t i = 0; i < registered_count_; ++i) {
        if (entries_[i].registered && entries_[i].name == name) return i;
    }
    return kInvalidIndex;
}

BaseAdc* AdcManager::Get(std::string_view name) noexcept {
    if (!initialized_.load(std::memory_order_acquire)) return nullptr;
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return nullptr;
    entries_[idx].access_count++;
    return entries_[idx].driver;
}

bool AdcManager::Contains(std::string_view name) const noexcept {
    if (!initialized_.load(std::memory_order_acquire)) return false;
    return FindByName(name) != kInvalidIndex;
}

size_t AdcManager::Size() const noexcept {
    return registered_count_;
}

//==============================================================================
// READING OPERATIONS
//==============================================================================

hf_adc_err_t AdcManager::ReadChannelV(std::string_view name, float& voltage,
                                      hf_u8_t numOfSamplesToAvg,
                                      hf_time_t timeBetweenSamples) noexcept {
    if (!initialized_.load(std::memory_order_acquire))
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;

    size_t idx = FindByName(name);
    if (idx == kInvalidIndex)
        return hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND;

    auto& entry = entries_[idx];
    MutexLockGuard lock(mutex_);

    float raw_voltage = 0.0f;
    hf_adc_err_t err = entry.driver->ReadChannelV(
        entry.hw_channel, raw_voltage, numOfSamplesToAvg, timeBetweenSamples);

    if (err == hf_adc_err_t::ADC_SUCCESS) {
        voltage = raw_voltage * entry.voltage_divider;
        entry.access_count++;
    } else {
        entry.error_count++;
    }
    return err;
}

//==============================================================================
// DIAGNOSTICS
//==============================================================================

hf_adc_err_t AdcManager::GetSystemDiagnostics(AdcSystemDiagnostics& diag) const noexcept {
    if (!initialized_.load(std::memory_order_acquire))
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;

    diag.total_channels_registered = static_cast<uint32_t>(registered_count_);
    diag.channels_by_chip[0] = 0; // No ESP32 ADC channels on Vortex
    diag.channels_by_chip[1] = static_cast<uint32_t>(registered_count_); // All TMC9660

    diag.total_operations = 0;
    diag.successful_operations = 0;
    diag.failed_operations = 0;

    for (size_t i = 0; i < registered_count_; ++i) {
        diag.total_operations += entries_[i].access_count + entries_[i].error_count;
        diag.successful_operations += entries_[i].access_count;
        diag.failed_operations += entries_[i].error_count;
    }

    diag.system_healthy = (registered_count_ > 0) && (diag.failed_operations == 0);
    diag.system_uptime_ms = os_get_elapsed_time_msec();
    diag.last_error = hf_adc_err_t::ADC_SUCCESS;

    return hf_adc_err_t::ADC_SUCCESS;
}

void AdcManager::DumpStatistics() const noexcept {
    auto& log = Logger::GetInstance();
    log.Info(TAG, "=== Vortex ADC Manager Statistics ===");
    log.Info(TAG, "  Channels registered: %zu / %zu", registered_count_, kMaxChannels);

    for (size_t i = 0; i < registered_count_; ++i) {
        const auto& e = entries_[i];
        log.Info(TAG, "  [%.*s] ch=%u div=%.1f reads=%u errs=%u",
                 static_cast<int>(e.name.size()), e.name.data(),
                 e.hw_channel,
                 static_cast<double>(e.voltage_divider),
                 e.access_count, e.error_count);
    }
    log.Info(TAG, "=== End Vortex ADC Manager Statistics ===");
}
