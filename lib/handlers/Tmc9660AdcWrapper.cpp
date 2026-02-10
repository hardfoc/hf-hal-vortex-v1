/**
 * @file Tmc9660AdcWrapper.cpp
 * @brief Implementation of TMC9660 ADC wrapper that delegates to TMC9660Handler::Adc.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 */

#include "Tmc9660AdcWrapper.h"
#include "Tmc9660Handler.h"

//==============================================================================
// CONSTRUCTION
//==============================================================================

Tmc9660AdcWrapper::Tmc9660AdcWrapper(Tmc9660Handler& handler) noexcept
    : handler_(handler) {
}

//==============================================================================
// BASE ADC IMPLEMENTATION (DELEGATION)
//==============================================================================

bool Tmc9660AdcWrapper::Initialize() noexcept {
    return handler_.adc().Initialize();
}

bool Tmc9660AdcWrapper::Deinitialize() noexcept {
    return handler_.adc().Deinitialize();
}

hf_u8_t Tmc9660AdcWrapper::GetMaxChannels() const noexcept {
    return handler_.adc().GetMaxChannels();
}

bool Tmc9660AdcWrapper::IsChannelAvailable(hf_channel_id_t channel_id) const noexcept {
    return handler_.adc().IsChannelAvailable(channel_id);
}

hf_adc_err_t Tmc9660AdcWrapper::ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                                           hf_u8_t numOfSamplesToAvg,
                                           hf_time_t timeBetweenSamples) noexcept {
    return handler_.adc().ReadChannelV(channel_id, channel_reading_v, numOfSamplesToAvg, timeBetweenSamples);
}

hf_adc_err_t Tmc9660AdcWrapper::ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                               hf_u8_t numOfSamplesToAvg,
                                               hf_time_t timeBetweenSamples) noexcept {
    return handler_.adc().ReadChannelCount(channel_id, channel_reading_count, numOfSamplesToAvg, timeBetweenSamples);
}

hf_adc_err_t Tmc9660AdcWrapper::ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                         float& channel_reading_v, hf_u8_t numOfSamplesToAvg,
                                         hf_time_t timeBetweenSamples) noexcept {
    return handler_.adc().ReadChannel(channel_id, channel_reading_count, channel_reading_v, numOfSamplesToAvg, timeBetweenSamples);
}

hf_adc_err_t Tmc9660AdcWrapper::ReadMultipleChannels(const hf_channel_id_t* channel_ids, hf_u8_t num_channels,
                                                   hf_u32_t* readings, float* voltages) noexcept {
    return handler_.adc().ReadMultipleChannels(channel_ids, num_channels, readings, voltages);
}

hf_adc_err_t Tmc9660AdcWrapper::GetStatistics(hf_adc_statistics_t& statistics) noexcept {
    return handler_.adc().GetStatistics(statistics);
}

hf_adc_err_t Tmc9660AdcWrapper::GetDiagnostics(hf_adc_diagnostics_t& diagnostics) noexcept {
    return handler_.adc().GetDiagnostics(diagnostics);
}

hf_adc_err_t Tmc9660AdcWrapper::ResetStatistics() noexcept {
    return handler_.adc().ResetStatistics();
}

hf_adc_err_t Tmc9660AdcWrapper::ResetDiagnostics() noexcept {
    return handler_.adc().ResetDiagnostics();
} 