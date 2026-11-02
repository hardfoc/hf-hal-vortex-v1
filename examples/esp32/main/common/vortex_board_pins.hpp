/**
 * @file vortex_board_pins.hpp
 * @brief ESP32-C6 GPIO numbers of the Vortex V1 COMM / LED nets, for examples that call raw IDF APIs
 *        (for example an I2C probe that runs before the HAL).
 *
 * @details Every number is read at compile time from the functional pin map
 *          (`hf_functional_pin_config_vortex_v1.hpp`, the single source; see ADR 0003), so it cannot
 *          drift from what the HAL managers use. A `static_assert` checks that each net is an ESP32
 *          pin, because a net that moves to the PCAL95555 has no ESP32 GPIO number.
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"

namespace vortex_board_pins {

namespace detail {

/** @brief The pin-map row of a functional pin. */
constexpr const HfGpioMapping& Row(HfFunctionalGpioPin pin) noexcept {
    return HF_GPIO_MAPPING[static_cast<size_t>(pin)];
}

/** @brief True when the functional pin is an ESP32-C6 GPIO (not an expander or TMC9660 pin). */
constexpr bool IsEspPin(HfFunctionalGpioPin pin) noexcept {
    return Row(pin).chip_type == static_cast<uint8_t>(HfGpioChipType::ESP32_INTERNAL);
}

/** @brief ESP32-C6 GPIO number of a functional pin. */
constexpr int Gpio(HfFunctionalGpioPin pin) noexcept { return static_cast<int>(Row(pin).physical_pin); }

}  // namespace detail

#define VORTEX_BOARD_PIN(name, functional)                                                  \
    static_assert(detail::IsEspPin(HfFunctionalGpioPin::functional),                        \
                  #functional " is not an ESP32-C6 pin in the Vortex V1 pin map");          \
    inline constexpr int name = detail::Gpio(HfFunctionalGpioPin::functional)

// I2C (shared: PCAL95555, BNO08x, PCA9685 when populated)
VORTEX_BOARD_PIN(kI2cSdaGpio, I2C_SDA);
VORTEX_BOARD_PIN(kI2cSclGpio, I2C_SCL);
VORTEX_BOARD_PIN(kI2cPcal95555IntGpio, I2C_PCAL95555_INT);

// SPI2 (TMC9660, AS5047U, external CS)
VORTEX_BOARD_PIN(kSpiMisoGpio, SPI2_MISO);
VORTEX_BOARD_PIN(kSpiMosiGpio, SPI2_MOSI);
VORTEX_BOARD_PIN(kSpiSckGpio, SPI2_SCK);
VORTEX_BOARD_PIN(kSpiCsTmc9660Gpio, SPI2_CS_TMC9660);
VORTEX_BOARD_PIN(kSpiCsAs5047Gpio, SPI2_CS_AS5047);
VORTEX_BOARD_PIN(kSpiCsExt1Gpio, EXT_GPIO_CS_1);
VORTEX_BOARD_PIN(kSpiCsExt2Gpio, EXT_GPIO_CS_2);

// UART (TMC9660 TMCL)
VORTEX_BOARD_PIN(kUartRxGpio, UART_RXD);
VORTEX_BOARD_PIN(kUartTxGpio, UART_TXD);

// Status LED (WS2812 data — the HAL LedManager uses the same row)
VORTEX_BOARD_PIN(kWs2812DataGpio, WS2812_LED_DAT);

// TWAI
VORTEX_BOARD_PIN(kTwaiTxGpio, TWAI_TX);
VORTEX_BOARD_PIN(kTwaiRxGpio, TWAI_RX);

#undef VORTEX_BOARD_PIN

}  // namespace vortex_board_pins
