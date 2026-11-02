/**
 * @file test_board_config.cpp
 * @brief Board wiring facts the HAL and the examples derive from the pin map.
 *
 * The board NTC is on TMC9660 AIN3, read through the shared Tmc9660AdcWrapper (one BaseAdc for all
 * TMC9660 channels, routed by channel id). The NTC handler's default channel is 0, so a handler built
 * without the board config reads AIN0. The examples' raw GPIO numbers come from the same pin map.
 */

#include "HfCoreMocks.h"
#include "HostTest.h"
#include "NtcTemperatureHandler.h"
#include "config/VortexNtcConfig.h"
#include "vortex_board_pins.hpp"

using hf::host::MockAdc;

namespace {

/** Divider voltage for an NTC resistance, with the handler's default divider and reference. */
float DividerVoltage(const ntc_temp_handler_config_t& cfg, float r_ntc) {
    return cfg.reference_voltage * r_ntc / (cfg.voltage_divider_series_resistance + r_ntc);
}

}  // namespace

HF_TEST(NtcConfig_UsesTmc9660Ain3FromThePinMap) {
    const auto& row = HF_ADC_MAPPING[static_cast<size_t>(HfFunctionalAdcChannel::TMC9660_AIN3)];
    CHECK_EQ(vortex::config::BoardNtcHwChannel(), row.physical_channel);
    CHECK_EQ(vortex::config::MakeBoardNtcConfig().adc_channel, 3u);
    const ntc_temp_handler_config_t handler_default = NTC_TEMP_HANDLER_CONFIG_DEFAULT();
    CHECK(handler_default.adc_channel != vortex::config::BoardNtcHwChannel());
}

HF_TEST(NtcConfig_ReadsAin3NotAin0OnASharedAdc) {
    // One BaseAdc serving every channel, like Tmc9660AdcWrapper. AIN0 holds a voltage that would
    // read as a very different temperature, so reading the wrong channel cannot pass by accident.
    const ntc_temp_handler_config_t cfg = vortex::config::MakeBoardNtcConfig();
    MockAdc adc(16, cfg.reference_voltage, 16);
    adc.SetVoltage(0, DividerVoltage(cfg, 2000.0f));   // AIN0: hot
    adc.SetVoltage(3, DividerVoltage(cfg, 10000.0f));  // AIN3: 25 °C on a 10 kΩ NTC

    NtcTemperatureHandler ntc(&adc, cfg);
    REQUIRE(ntc.EnsureInitialized());
    float c = -100.0f;
    CHECK(ntc.ReadTemperatureCelsius(&c) == hf_temp_err_t::TEMP_SUCCESS);
    CHECK_NEAR(c, 25.0, 1.0);
    CHECK(adc.ReadCount(3) >= 1u);
    CHECK_EQ(adc.ReadCount(0), 0u);
}

HF_TEST(NtcConfig_ThresholdWindowIsAppliedAfterInit) {
    // enable_threshold_monitoring in the handler config makes Initialize() recurse into
    // EnsureInitialized() (stack overflow), so the window is applied once the handler is up.
    const ntc_temp_handler_config_t cfg = vortex::config::MakeBoardNtcConfig();
    CHECK(!cfg.enable_threshold_monitoring);
    MockAdc adc(16, cfg.reference_voltage, 16);
    adc.SetVoltage(3, DividerVoltage(cfg, 10000.0f));
    NtcTemperatureHandler ntc(&adc, cfg);
    REQUIRE(ntc.EnsureInitialized());
    CHECK(vortex::config::ApplyBoardNtcThresholds(ntc));
    float lo = 0.0f;
    float hi = 0.0f;
    CHECK(ntc.GetThresholds(&lo, &hi) == hf_temp_err_t::TEMP_SUCCESS);
    CHECK_NEAR(lo, 0.0, 1e-6);
    CHECK_NEAR(hi, 85.0, 1e-6);
}

HF_TEST(BoardPins_MatchThePinMap) {
    const auto gpio = [](HfFunctionalGpioPin p) {
        return static_cast<int>(HF_GPIO_MAPPING[static_cast<size_t>(p)].physical_pin);
    };
    CHECK_EQ(vortex_board_pins::kI2cSdaGpio, gpio(HfFunctionalGpioPin::I2C_SDA));
    CHECK_EQ(vortex_board_pins::kI2cSclGpio, gpio(HfFunctionalGpioPin::I2C_SCL));
    CHECK_EQ(vortex_board_pins::kSpiMisoGpio, gpio(HfFunctionalGpioPin::SPI2_MISO));
    CHECK_EQ(vortex_board_pins::kSpiCsTmc9660Gpio, gpio(HfFunctionalGpioPin::SPI2_CS_TMC9660));
    CHECK_EQ(vortex_board_pins::kUartTxGpio, gpio(HfFunctionalGpioPin::UART_TXD));
    CHECK_EQ(vortex_board_pins::kWs2812DataGpio, gpio(HfFunctionalGpioPin::WS2812_LED_DAT));
    CHECK_EQ(vortex_board_pins::kTwaiRxGpio, gpio(HfFunctionalGpioPin::TWAI_RX));
}

HF_TEST(BoardPins_KnownValuesOnVortexV1) {
    // The values the board was built with; a pin-map change shows up here as well as on the board.
    CHECK_EQ(vortex_board_pins::kI2cSdaGpio, 21);
    CHECK_EQ(vortex_board_pins::kI2cSclGpio, 22);
    CHECK_EQ(vortex_board_pins::kSpiSckGpio, 6);
    CHECK_EQ(vortex_board_pins::kSpiCsAs5047Gpio, 20);
    CHECK_EQ(vortex_board_pins::kWs2812DataGpio, 3);
    CHECK_EQ(vortex_board_pins::kTwaiTxGpio, 14);
}
