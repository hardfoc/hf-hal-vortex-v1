/**
 * @file vortex_board_pins.hpp
 * @brief ESP32-C6 GPIO numbers for Vortex V1 — must match
 *        hf_functional_pin_config_vortex_v1.hpp (COMM / LED only).
 *
 * Use for examples that need raw IDF APIs (e.g. I2C probe). HAL managers resolve
 * the same nets via hf-pincfg at runtime.
 */
#pragma once

#include <cstdint>

namespace vortex_board_pins {

// I2C (shared: PCAL95555, BNO08x, PCA9685 when populated)
inline constexpr int kI2cSdaGpio = 21;
inline constexpr int kI2cSclGpio = 22;

// SPI2 (TMC9660, AS5047U, external CS)
inline constexpr int kSpiMisoGpio = 2;
inline constexpr int kSpiMosiGpio = 7;
inline constexpr int kSpiSckGpio = 6;
inline constexpr int kSpiCsTmc9660Gpio = 18;
inline constexpr int kSpiCsAs5047Gpio = 20;

// UART (TMC9660 TMCL)
inline constexpr int kUartRxGpio = 4;
inline constexpr int kUartTxGpio = 5;

// Status LED (WS2812 data — HAL LedManager uses same mapping)
inline constexpr int kWs2812DataGpio = 3;

// TWAI
inline constexpr int kTwaiTxGpio = 14;
inline constexpr int kTwaiRxGpio = 15;

}  // namespace vortex_board_pins
