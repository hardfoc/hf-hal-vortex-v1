/**
 * @file Tmc9660AdcWrapper.h
 * @brief Simple wrapper that delegates to TMC9660Handler::Adc.
 * 
 * @details This wrapper provides access to TMC9660Handler::Adc which already
 *          implements BaseAdc. The wrapper simply delegates all calls to the
 *          handler's ADC instance.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 */

#ifndef COMPONENT_HANDLER_TMC9660_ADC_WRAPPER_H_
#define COMPONENT_HANDLER_TMC9660_ADC_WRAPPER_H_

#include "base/BaseAdc.h"

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class Tmc9660Handler;

//==============================================================================
// TMC9660 ADC WRAPPER CLASS
//==============================================================================

/**
 * @class Tmc9660AdcWrapper
 * @brief Simple wrapper that delegates to TMC9660Handler::Adc.
 * 
 * @details This class provides a BaseAdc interface by delegating all calls
 *          to the TMC9660Handler::Adc instance, which already implements BaseAdc.
 */
class Tmc9660AdcWrapper : public BaseAdc {
public:
    //==============================================//
    // CONSTRUCTION
    //==============================================//

    /**
     * @brief Constructor for TMC9660 ADC wrapper.
     * @param handler Reference to TMC9660 handler
     */
    explicit Tmc9660AdcWrapper(Tmc9660Handler& handler) noexcept;

    /**
     * @brief Destructor.
     */
    ~Tmc9660AdcWrapper() override = default;

    // Disable copy operations
    Tmc9660AdcWrapper(const Tmc9660AdcWrapper&) = delete;
    Tmc9660AdcWrapper& operator=(const Tmc9660AdcWrapper&) = delete;

    // Allow move operations
    Tmc9660AdcWrapper(Tmc9660AdcWrapper&&) noexcept = default;
    Tmc9660AdcWrapper& operator=(Tmc9660AdcWrapper&&) noexcept = default;

    //==============================================//
    // BASE ADC IMPLEMENTATION (DELEGATION)
    //==============================================//

    bool Initialize() noexcept override;
    bool Deinitialize() noexcept override;
    hf_u8_t GetMaxChannels() const noexcept override;
    bool IsChannelAvailable(hf_channel_id_t channel_id) const noexcept override;
    hf_adc_err_t ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                             hf_u8_t numOfSamplesToAvg = 1,
                             hf_time_t timeBetweenSamples = 0) noexcept override;
    hf_adc_err_t ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                 hf_u8_t numOfSamplesToAvg = 1,
                                 hf_time_t timeBetweenSamples = 0) noexcept override;
    hf_adc_err_t ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                           float& channel_reading_v, hf_u8_t numOfSamplesToAvg = 1,
                           hf_time_t timeBetweenSamples = 0) noexcept override;
    hf_adc_err_t ReadMultipleChannels(const hf_channel_id_t* channel_ids, hf_u8_t num_channels,
                                     hf_u32_t* readings, float* voltages) noexcept override;
    hf_adc_err_t GetStatistics(hf_adc_statistics_t& statistics) noexcept override;
    hf_adc_err_t GetDiagnostics(hf_adc_diagnostics_t& diagnostics) noexcept override;
    hf_adc_err_t ResetStatistics() noexcept override;
    hf_adc_err_t ResetDiagnostics() noexcept override;

private:
    Tmc9660Handler& handler_; ///< Reference to TMC9660 handler
};

#endif // COMPONENT_HANDLER_TMC9660_ADC_WRAPPER_H_ 