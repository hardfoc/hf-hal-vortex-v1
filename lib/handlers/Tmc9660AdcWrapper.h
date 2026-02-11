/**
 * @file Tmc9660AdcWrapper.h
 * @brief Thin BaseAdc delegation wrapper for manager-layer ownership of TMC9660 ADC.
 *
 * @details
 * ## Purpose
 *
 * The TMC9660's ADC functionality is implemented by Tmc9660Handler::Adc (an inner
 * class of Tmc9660Handler that inherits from BaseAdc). The handler owns this inner
 * class instance and exposes it via Tmc9660Handler::adc().
 *
 * However, the AdcManager needs to own a std::unique_ptr<BaseAdc> that it can
 * register, manage, and destroy independently. Since Tmc9660Handler::Adc is owned
 * by the handler (and cannot be transferred), this wrapper class exists to bridge
 * the ownership gap:
 *
 * @code
 *  ┌──────────────┐      ┌─────────────────────┐      ┌───────────────────────┐
 *  │  AdcManager  │ owns │  Tmc9660AdcWrapper  │ refs │  Tmc9660Handler::Adc  │
 *  │              │─────>│  (BaseAdc)          │─────>│  (BaseAdc impl)       │
 *  │  unique_ptr  │      │  delegates all calls│      │  reads TMC9660 driver │
 *  └──────────────┘      └─────────────────────┘      └───────────────────────┘
 * @endcode
 *
 * Every BaseAdc virtual method in this class is a one-line delegation to
 * handler_.adc().Method(). No additional logic, buffering, or transformation
 * is performed.
 *
 * ## Lifetime Requirements
 *
 * The Tmc9660Handler referenced by this wrapper must remain alive for the entire
 * lifetime of the wrapper. This is naturally satisfied when MotorController owns
 * the handler and AdcManager creates the wrapper from that handler, since both
 * managers are singletons with matched lifetimes.
 *
 * ## Analogous Patterns
 *
 * - Tmc9660TemperatureWrapper (in TemperatureManager.h) does the same for
 *   BaseTemperature, delegating to Tmc9660Handler::Temperature.
 *
 * @see Tmc9660Handler::Adc  The actual ADC implementation this class delegates to.
 * @see AdcManager::CreateTmc9660AdcWrapper  Factory that creates instances of this class.
 *
 * @author HardFOC Team
 * @date 2025
 */

#ifndef COMPONENT_HANDLER_TMC9660_ADC_WRAPPER_H_
#define COMPONENT_HANDLER_TMC9660_ADC_WRAPPER_H_

#include "base/BaseAdc.h"

// Forward declaration to avoid including the full handler header.
class Tmc9660Handler;

/**
 * @class Tmc9660AdcWrapper
 * @brief Delegation wrapper that adapts Tmc9660Handler::Adc for external ownership.
 *
 * @details
 * This is a pure delegation class. Every BaseAdc method simply calls the
 * corresponding method on the Tmc9660Handler's internal Adc instance via
 * `handler_.adc()`. It exists solely to allow AdcManager to own a
 * `std::unique_ptr<BaseAdc>` that refers to TMC9660 ADC functionality
 * without transferring ownership of the handler's internal objects.
 *
 * @note This class does NOT own or manage the Tmc9660Handler. The handler
 *       must outlive all Tmc9660AdcWrapper instances that reference it.
 *
 * @see Tmc9660Handler::Adc  Inner class with the actual implementation.
 */
class Tmc9660AdcWrapper : public BaseAdc {
public:
    /**
     * @brief Construct the ADC delegation wrapper.
     * @param handler Reference to a live Tmc9660Handler whose adc() will be used.
     * @warning The handler must remain valid for the lifetime of this wrapper.
     */
    explicit Tmc9660AdcWrapper(Tmc9660Handler& handler) noexcept;

    /** @brief Default destructor. Does not affect the handler or its ADC. */
    ~Tmc9660AdcWrapper() override = default;

    /// Non-copyable, non-movable (reference semantics -- cannot rebind handler_).
    Tmc9660AdcWrapper(const Tmc9660AdcWrapper&) = delete;
    Tmc9660AdcWrapper& operator=(const Tmc9660AdcWrapper&) = delete;
    Tmc9660AdcWrapper(Tmc9660AdcWrapper&&) = delete;
    Tmc9660AdcWrapper& operator=(Tmc9660AdcWrapper&&) = delete;

    /// @name BaseAdc Delegation Methods
    /// @brief Each method delegates directly to handler_.adc().Method().
    /// @{

    /** @copydoc BaseAdc::Initialize */
    bool Initialize() noexcept override;

    /** @copydoc BaseAdc::Deinitialize */
    bool Deinitialize() noexcept override;

    /** @copydoc BaseAdc::GetMaxChannels */
    hf_u8_t GetMaxChannels() const noexcept override;

    /** @copydoc BaseAdc::IsChannelAvailable */
    bool IsChannelAvailable(hf_channel_id_t channel_id) const noexcept override;

    /** @copydoc BaseAdc::ReadChannelV */
    hf_adc_err_t ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                             hf_u8_t numOfSamplesToAvg = 1,
                             hf_time_t timeBetweenSamples = 0) noexcept override;

    /** @copydoc BaseAdc::ReadChannelCount */
    hf_adc_err_t ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                 hf_u8_t numOfSamplesToAvg = 1,
                                 hf_time_t timeBetweenSamples = 0) noexcept override;

    /** @copydoc BaseAdc::ReadChannel */
    hf_adc_err_t ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                           float& channel_reading_v, hf_u8_t numOfSamplesToAvg = 1,
                           hf_time_t timeBetweenSamples = 0) noexcept override;

    /** @copydoc BaseAdc::ReadMultipleChannels */
    hf_adc_err_t ReadMultipleChannels(const hf_channel_id_t* channel_ids, hf_u8_t num_channels,
                                     hf_u32_t* readings, float* voltages) noexcept override;

    /** @copydoc BaseAdc::GetStatistics */
    hf_adc_err_t GetStatistics(hf_adc_statistics_t& statistics) noexcept override;

    /** @copydoc BaseAdc::GetDiagnostics */
    hf_adc_err_t GetDiagnostics(hf_adc_diagnostics_t& diagnostics) noexcept override;

    /** @copydoc BaseAdc::ResetStatistics */
    hf_adc_err_t ResetStatistics() noexcept override;

    /** @copydoc BaseAdc::ResetDiagnostics */
    hf_adc_err_t ResetDiagnostics() noexcept override;

    /// @}

private:
    Tmc9660Handler& handler_; ///< Reference to the TMC9660 handler (not owned).
};

#endif // COMPONENT_HANDLER_TMC9660_ADC_WRAPPER_H_
