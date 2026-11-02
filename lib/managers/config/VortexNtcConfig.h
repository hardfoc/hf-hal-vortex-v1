/**
 * @file VortexNtcConfig.h
 * @brief Board NTC thermistor wiring: which TMC9660 ADC input it is on and how the handler is set up.
 *
 * @details The board NTC sits on TMC9660 AIN3 (`ADC_TMC9660_AIN3` in the pin map). Every TMC9660 ADC
 *          channel is served by one shared `Tmc9660AdcWrapper`, which routes by hardware channel id at
 *          read time, so the NTC handler must be told the channel explicitly: the handler's default
 *          (`adc_channel = 0`) would read AIN0. The channel here is read from the pin map, not repeated.
 *
 *          Ownership: constants only. Thread safety: none needed. MCU-free.
 */

#ifndef VORTEX_NTC_CONFIG_H_
#define VORTEX_NTC_CONFIG_H_

#include <cstddef>
#include <cstdint>

#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"
#include "handlers/ntc/NtcTemperatureHandler.h"

namespace vortex::config {

/** @brief Functional ADC channel the board NTC is wired to. */
inline constexpr HfFunctionalAdcChannel kBoardNtcAdcChannel = HfFunctionalAdcChannel::TMC9660_AIN3;

/** @brief Hardware channel id of the board NTC on the shared TMC9660 ADC wrapper, from the pin map. */
[[nodiscard]] constexpr uint8_t BoardNtcHwChannel() noexcept {
    return HF_ADC_MAPPING[static_cast<size_t>(kBoardNtcAdcChannel)].physical_channel;
}

static_assert(HF_ADC_MAPPING[static_cast<size_t>(kBoardNtcAdcChannel)].chip_type ==
                  static_cast<uint8_t>(HfAdcChipType::TMC9660_CONTROLLER),
              "Board NTC must be on a TMC9660 ADC input");
static_assert(BoardNtcHwChannel() == 3, "Board NTC is on TMC9660 AIN3");

/** @brief Over-temperature window the board NTC reports against (degrees Celsius). */
inline constexpr float kBoardNtcLowThresholdC = 0.0f;
inline constexpr float kBoardNtcHighThresholdC = 85.0f;

/**
 * @brief NTC handler configuration for the board thermistor.
 * @details `enable_threshold_monitoring` stays false on purpose: with it set,
 *          NtcTemperatureHandler::Initialize() calls SetThresholds(), which calls
 *          EnsureInitialized() before the handler is marked initialised, and recurses until the stack
 *          overflows. Apply the window after initialisation with ApplyBoardNtcThresholds().
 * @return Handler defaults with the AIN3 channel id and the board threshold window.
 */
[[nodiscard]] inline ntc_temp_handler_config_t MakeBoardNtcConfig() noexcept {
    ntc_temp_handler_config_t cfg = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    cfg.adc_channel = BoardNtcHwChannel();
    cfg.enable_threshold_monitoring = false;
    cfg.low_threshold_celsius = kBoardNtcLowThresholdC;
    cfg.high_threshold_celsius = kBoardNtcHighThresholdC;
    return cfg;
}

/**
 * @brief Set the board over-temperature window on an initialised NTC handler.
 * @param ntc Handler built from MakeBoardNtcConfig() whose EnsureInitialized() returned true.
 * @return true when the window was accepted.
 */
inline bool ApplyBoardNtcThresholds(NtcTemperatureHandler& ntc) noexcept {
    return ntc.SetThresholds(kBoardNtcLowThresholdC, kBoardNtcHighThresholdC) == hf_temp_err_t::TEMP_SUCCESS;
}

}  // namespace vortex::config

#endif  // VORTEX_NTC_CONFIG_H_
