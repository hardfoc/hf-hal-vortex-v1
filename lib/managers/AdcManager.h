/**
 * @file AdcManager.h
 * @brief ADC management for the HardFOC Vortex V1 platform.
 *
 * @details The Vortex board uses TMC9660-hosted ADC channels exclusively
 *          (no ESP32-C6 internal ADC channels are active). All 15 functional
 *          ADC channels are routed through a single Tmc9660AdcWrapper that
 *          delegates to the MotorController-owned Tmc9660Handler.
 *
 *          Follows the Flux ownership pattern: fixed-size array of AdcEntry
 *          structs (no heap unordered_map), single RtosMutex, unique_ptr
 *          for the TMC9660 wrapper.
 *
 * @version 2.0
 */

#ifndef VORTEX_ADC_MANAGER_H_
#define VORTEX_ADC_MANAGER_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string_view>

#include "base/BaseAdc.h"
#include "RtosMutex.h"
#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"

// Forward declarations
class MotorController;
class Tmc9660Handler;
class Tmc9660AdcWrapper;

//==============================================================================
// ADC ENTRY
//==============================================================================

/**
 * @brief A single ADC channel entry in the fixed-size registry.
 */
struct AdcEntry {
    HfFunctionalAdcChannel functional_channel{HfFunctionalAdcChannel::HF_FUNCTIONAL_ADC_COUNT};
    std::string_view name;              ///< Human-readable name from pincfg
    BaseAdc* driver{nullptr};           ///< Pointer to the ADC driver (owned by the wrapper unique_ptr)
    uint8_t adc_unit{0};                ///< ADC unit on the chip
    uint8_t hw_channel{0};              ///< Hardware channel within the unit
    float voltage_divider{1.0f};        ///< Divider ratio for scaled readings
    bool registered{false};             ///< Whether this entry is active

    // Statistics
    uint32_t access_count{0};
    uint32_t error_count{0};
};

//==============================================================================
// ADC SYSTEM DIAGNOSTICS
//==============================================================================

/**
 * @brief ADC system diagnostics (compatible with example code).
 */
struct AdcSystemDiagnostics {
    bool system_healthy{false};
    uint32_t total_channels_registered{0};
    uint32_t channels_by_chip[2]{};       ///< [0]=ESP32, [1]=TMC9660
    uint32_t total_operations{0};
    uint32_t successful_operations{0};
    uint32_t failed_operations{0};
    uint32_t communication_errors{0};
    uint32_t hardware_errors{0};
    uint64_t system_uptime_ms{0};
    hf_adc_err_t last_error{hf_adc_err_t::ADC_SUCCESS};
};

//==============================================================================
// ADC MANAGER
//==============================================================================

class AdcManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================

    static AdcManager& GetInstance() noexcept;

    AdcManager(const AdcManager&) = delete;
    AdcManager& operator=(const AdcManager&) = delete;
    AdcManager(AdcManager&&) = delete;
    AdcManager& operator=(AdcManager&&) = delete;

    [[nodiscard]] bool EnsureInitialized() noexcept;
    [[nodiscard]] bool IsInitialized() const noexcept { return initialized_.load(std::memory_order_acquire); }

    //==========================================================================
    // CHANNEL ACCESS
    //==========================================================================

    /** @brief Get the BaseAdc driver for a named channel. Returns nullptr if not found. */
    [[nodiscard]] BaseAdc* Get(std::string_view name) noexcept;

    /** @brief Check if a named ADC channel exists. */
    [[nodiscard]] bool Contains(std::string_view name) const noexcept;

    /** @brief Number of registered ADC channels. */
    [[nodiscard]] size_t Size() const noexcept;

    //==========================================================================
    // READING OPERATIONS (compatible with existing example API)
    //==========================================================================

    /**
     * @brief Read voltage from a channel by name.
     * @param name               Channel name
     * @param voltage            Output: voltage reading
     * @param numOfSamplesToAvg  Number of samples to average (default 1)
     * @param timeBetweenSamples Time between samples in ms (default 0)
     * @return ADC error code
     */
    [[nodiscard]] hf_adc_err_t ReadChannelV(std::string_view name, float& voltage,
                                            hf_u8_t numOfSamplesToAvg = 1,
                                            hf_time_t timeBetweenSamples = 0) noexcept;

    //==========================================================================
    // DIAGNOSTICS
    //==========================================================================

    [[nodiscard]] hf_adc_err_t GetSystemDiagnostics(AdcSystemDiagnostics& diagnostics) const noexcept;
    void DumpStatistics() const noexcept;

private:
    AdcManager() noexcept = default;
    ~AdcManager();

    bool Initialize() noexcept;

    /** @brief Find entry index by string name. Returns kInvalidIndex if not found. */
    [[nodiscard]] size_t FindByName(std::string_view name) const noexcept;

    //==========================================================================
    // STORAGE — entries are fixed-size; Tmc9660AdcWrapper is heap-allocated
    //           via unique_ptr (singleton lifetime)
    //==========================================================================

    static constexpr size_t kMaxChannels =
        static_cast<size_t>(HfFunctionalAdcChannel::HF_FUNCTIONAL_ADC_COUNT);
    static constexpr size_t kInvalidIndex = SIZE_MAX;

    std::array<AdcEntry, kMaxChannels> entries_{};
    size_t registered_count_{0};

    /// Single TMC9660 ADC wrapper — all channels route through this.
    std::unique_ptr<Tmc9660AdcWrapper> tmc9660_adc_;

    /// MotorController dependency (non-owning pointer, set at init).
    MotorController* motor_controller_{nullptr};

    std::atomic<bool> initialized_{false};
    mutable RtosMutex mutex_;
};

//==============================================================================
// CONVENIENCE
//==============================================================================

[[nodiscard]] inline AdcManager& GetAdcManager() noexcept {
    return AdcManager::GetInstance();
}

#endif // VORTEX_ADC_MANAGER_H_
