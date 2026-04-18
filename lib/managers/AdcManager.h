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
    HfFunctionalAdcChannel functional_channel{HfFunctionalAdcChannel::HF_FUNCTIONAL_ADC_COUNT}; ///< Functional enum for this channel
    std::string_view name;              ///< Human-readable name from pincfg
    BaseAdc* driver{nullptr};           ///< Pointer to the ADC driver (owned by the wrapper unique_ptr)
    uint8_t adc_unit{0};                ///< ADC unit on the chip
    uint8_t hw_channel{0};              ///< Hardware channel within the unit
    float voltage_divider{1.0f};        ///< Divider ratio for scaled readings
    bool registered{false};             ///< Whether this entry is active

    // Statistics
    uint32_t access_count{0};           ///< Total successful read operations on this channel
    uint32_t error_count{0};            ///< Total failed read operations on this channel
};

//==============================================================================
// ADC SYSTEM DIAGNOSTICS
//==============================================================================

/**
 * @brief ADC system diagnostics (compatible with example code).
 */
struct AdcSystemDiagnostics {
    bool system_healthy{false};                 ///< True when all registered channels are operational
    uint32_t total_channels_registered{0};      ///< Number of channels currently registered
    uint32_t channels_by_chip[2]{};             ///< [0]=ESP32, [1]=TMC9660
    uint32_t total_operations{0};               ///< Cumulative read attempts across all channels
    uint32_t successful_operations{0};          ///< Cumulative successful reads
    uint32_t failed_operations{0};              ///< Cumulative failed reads
    uint32_t communication_errors{0};           ///< Errors attributed to bus communication failures
    uint32_t hardware_errors{0};                ///< Errors attributed to hardware faults
    uint64_t system_uptime_ms{0};               ///< Milliseconds since ADC subsystem initialisation
    hf_adc_err_t last_error{hf_adc_err_t::ADC_SUCCESS}; ///< Most recent error code from any operation
};

//==============================================================================
// ADC MANAGER
//==============================================================================

/**
 * @class AdcManager
 * @brief Singleton manager for all analogue-to-digital channels on Vortex V1.
 *
 * @details Provides a uniform API over the ESP32 internal ADC and the TMC9660
 *          motor-controller ADC. Channels are registered from the pincfg
 *          X-MACRO table at initialisation and looked up by name or by
 *          functional enum.
 *
 * @note Call EnsureInitialized() once before any reads. All public methods
 *       are thread-safe (guarded by an internal RTOS mutex).
 */
class AdcManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================

    /** @brief Return the singleton AdcManager instance. */
    static AdcManager& GetInstance() noexcept;

    AdcManager(const AdcManager&) = delete;
    AdcManager& operator=(const AdcManager&) = delete;
    AdcManager(AdcManager&&) = delete;
    AdcManager& operator=(AdcManager&&) = delete;

    /**
     * @brief Initialise the ADC subsystem if not already initialised.
     * @return true on success or if already initialised.
     */
    [[nodiscard]] bool EnsureInitialized() noexcept;

    /**
     * @brief Release all ADC resources and reset to uninitialised state.
     * @return true on success.
     */
    [[nodiscard]] bool Deinitialize() noexcept;

    /**
     * @brief Check whether the ADC subsystem has been initialised.
     * @return true if initialised and ready for reads.
     */
    [[nodiscard]] bool IsInitialized() const noexcept { return initialized_.load(std::memory_order_acquire); }

    //==========================================================================
    // CHANNEL ACCESS
    //==========================================================================

    /**
     * @brief Get the BaseAdc driver for a named channel.
     * @param name Channel name from the pincfg table (linear scan).
     * @return Pointer to the BaseAdc driver, or nullptr if not found.
     */
    [[nodiscard]] BaseAdc* Get(std::string_view name) noexcept;

    /**
     * @brief Get the BaseAdc driver by functional enum — O(1) lookup.
     * @param channel Functional ADC channel enum.
     * @return Pointer to the BaseAdc driver, or nullptr if not found.
     */
    [[nodiscard]] BaseAdc* Get(HfFunctionalAdcChannel channel) noexcept;

    /**
     * @brief Check whether a named ADC channel is registered.
     * @param name Channel name to look up.
     * @return true if the channel exists in the registry.
     */
    [[nodiscard]] bool Contains(std::string_view name) const noexcept;

    /**
     * @brief Get the number of registered ADC channels.
     * @return Channel count.
     */
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

    /**
     * @brief Read voltage from a channel by functional enum.
     * @param channel           Functional ADC channel enum.
     * @param voltage           Output: voltage reading.
     * @param numOfSamplesToAvg Number of samples to average (default 1).
     * @param timeBetweenSamples Time between samples in ms (default 0).
     * @return ADC error code.
     */
    [[nodiscard]] hf_adc_err_t ReadChannelV(HfFunctionalAdcChannel channel, float& voltage,
                                            hf_u8_t numOfSamplesToAvg = 1,
                                            hf_time_t timeBetweenSamples = 0) noexcept;

    /**
     * @brief Read raw ADC value from a channel by name.
     * @param name              Channel name (linear scan).
     * @param raw_value         Output: raw ADC count.
     * @param numOfSamplesToAvg Number of samples to average (default 1).
     * @return ADC error code.
     */
    [[nodiscard]] hf_adc_err_t ReadRaw(std::string_view name, uint32_t& raw_value,
                                       hf_u8_t numOfSamplesToAvg = 1) noexcept;

    /**
     * @brief Read raw ADC value from a channel by functional enum.
     * @param channel           Functional ADC channel enum.
     * @param raw_value         Output: raw ADC count.
     * @param numOfSamplesToAvg Number of samples to average (default 1).
     * @return ADC error code.
     */
    [[nodiscard]] hf_adc_err_t ReadRaw(HfFunctionalAdcChannel channel, uint32_t& raw_value,
                                       hf_u8_t numOfSamplesToAvg = 1) noexcept;

    //==========================================================================
    // CROSS-HAL API ALIASES — ReadVoltage (Flux-compatible)
    //==========================================================================

    /**
     * @brief Read voltage from a channel by name (Flux-compatible alias for ReadChannelV).
     * @param name    Channel name
     * @param voltage Output: voltage reading
     * @param samples Number of samples to average (default 1)
     * @return ADC error code
     */
    [[nodiscard]] hf_adc_err_t ReadVoltage(std::string_view name, float& voltage,
                                           hf_u8_t samples = 1) noexcept {
        return ReadChannelV(name, voltage, samples, 0);
    }

    /**
     * @brief Read voltage from a channel by enum (Flux-compatible alias for ReadVoltage).
     * @param channel Functional ADC channel enum
     * @param voltage Output: voltage reading
     * @param samples Number of samples to average (default 1)
     * @return ADC error code
     */
    [[nodiscard]] hf_adc_err_t ReadVoltage(HfFunctionalAdcChannel channel, float& voltage,
                                           hf_u8_t samples = 1) noexcept {
        return ReadChannelV(channel, voltage, samples, 0);
    }

    //==========================================================================
    // DIAGNOSTICS
    //==========================================================================

    /**
     * @brief Populate an AdcSystemDiagnostics snapshot.
     * @param[out] diagnostics Diagnostics structure to fill.
     * @return ADC_SUCCESS on success.
     */
    [[nodiscard]] hf_adc_err_t GetSystemDiagnostics(AdcSystemDiagnostics& diagnostics) const noexcept;

    /**
     * @brief Get the most recent error code from any ADC operation.
     * @return Last hf_adc_err_t set by any API call
     */
    [[nodiscard]] hf_adc_err_t GetLastError() const noexcept { return last_error_.load(std::memory_order_acquire); }

    /** @brief Log ADC channel statistics and health info to the console. */
    void DumpStatistics() const noexcept;

private:
    AdcManager() noexcept = default;
    ~AdcManager();

    bool Initialize() noexcept;

    /** @brief Find entry index by string name. Returns kInvalidIndex if not found. */
    [[nodiscard]] size_t FindByName(std::string_view name) const noexcept;

    /** @brief Find entry index by functional enum. Returns kInvalidIndex if not found. */
    [[nodiscard]] size_t FindByEnum(HfFunctionalAdcChannel channel) const noexcept;

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
    std::atomic<hf_adc_err_t> last_error_{hf_adc_err_t::ADC_SUCCESS};
    mutable RtosMutex mutex_;
};

//==============================================================================
// CONVENIENCE
//==============================================================================

/**
 * @brief Convenience accessor — equivalent to AdcManager::GetInstance().
 * @return Reference to the singleton AdcManager.
 */
[[nodiscard]] inline AdcManager& GetAdcManager() noexcept {
    return AdcManager::GetInstance();
}

#endif // VORTEX_ADC_MANAGER_H_
