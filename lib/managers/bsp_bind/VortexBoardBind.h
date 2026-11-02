/**
 * @file VortexBoardBind.h
 * @brief The board binding: the only way the Vortex managers reach MCU peripherals.
 *
 * @details Managers above this header are MCU-free: they include `Base*` interfaces, hf-core handlers and
 *          this file, never an ESP-IDF, FreeRTOS or `mcu/esp32/` header. Each function here is implemented
 *          once per platform, and CMake links exactly one implementation:
 *
 *          | Directory              | Platform                         | Built by                    |
 *          | ---------------------- | -------------------------------- | --------------------------- |
 *          | `bsp_bind/esp32c6/`    | ESP32-C6 (Esp* wraps, RMT LED)    | the ESP-IDF component       |
 *          | `bsp_bind/host/`       | hf-core host mocks               | `tests/host`                |
 *
 *          A future STM32 board adds its own directory (for example `bsp_bind/stm32h7/`) and nothing above it changes. `tools/lint/
 *          lint_hal_boundary.sh` enforces the rule.
 *
 *          Ownership: every object a function returns lives in static storage inside the binding and stays
 *          valid until the matching Close / CloseAll call; the binding never allocates after Open returns.
 *          Thread safety: none. Each function is called by one owning manager, under that manager's lock
 *          (CommChannelsManager for buses, GpioManager for MCU pins, NvsManager, TemperatureManager,
 *          LedManager). Error handling: Open functions return nullptr / false / a negative index on failure.
 */

#ifndef VORTEX_BOARD_BIND_H_
#define VORTEX_BOARD_BIND_H_

#include <cstddef>
#include <cstdint>

#include "base/BaseCan.h"
#include "base/BaseGpio.h"
#include "base/BaseI2c.h"
#include "base/BaseNvs.h"
#include "base/BaseSpi.h"
#include "base/BaseTemperature.h"
#include "base/BaseUart.h"
#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"

namespace vortex::bsp {

//==============================================================================
// Bus specifications (MCU-neutral; the binding maps them to its own driver config)
//==============================================================================

/** @brief UART port, framing 8N1, no flow control. */
struct UartSpec {
    uint8_t port{1};
    uint32_t baud{115200};
    int tx_pin{-1};
    int rx_pin{-1};
    uint16_t tx_buffer_bytes{256};
    uint16_t rx_buffer_bytes{512};
    uint32_t timeout_ms{100};
};

/** @brief SPI master bus (one per board on Vortex: SPI2). */
struct SpiBusSpec {
    uint8_t host{1};  ///< Controller index; 1 is SPI2 on the ESP32-C6
    int miso_pin{-1};
    int mosi_pin{-1};
    int sclk_pin{-1};
    uint32_t timeout_ms{1000};
};

/** @brief One chip select on the SPI bus. */
struct SpiDeviceSpec {
    int cs_pin{-1};
    uint32_t clock_hz{10'000'000};
    uint8_t mode{0};  ///< SPI mode 0..3 (CPOL/CPHA)
};

/** @brief I2C master bus. */
struct I2cBusSpec {
    uint8_t port{0};
    int sda_pin{-1};
    int scl_pin{-1};
    bool internal_pullups{false};
};

/** @brief One 7-bit device on the I2C bus. */
struct I2cDeviceSpec {
    uint16_t address{0};
    uint32_t scl_hz{100'000};
    uint32_t scl_wait_us{0};  ///< Clock-stretch allowance; 0 = driver default
};

/** @brief Classic CAN controller. */
struct CanSpec {
    int tx_pin{-1};
    int rx_pin{-1};
    uint32_t bitrate{500'000};
};

//==============================================================================
// UART
//==============================================================================

/** @brief Open the board UART. @return the port, or nullptr on failure. */
BaseUart* UartOpen(const UartSpec& spec) noexcept;
/** @brief Close the UART opened by UartOpen() (idempotent). */
void UartClose() noexcept;

//==============================================================================
// SPI
//==============================================================================

/** @brief Initialise the SPI bus. @return true on success. */
bool SpiOpen(const SpiBusSpec& spec) noexcept;
/** @brief Add a chip select. @return the device index (>= 0) or -1. Call after SpiOpen(). */
int SpiAddDevice(const SpiDeviceSpec& spec) noexcept;
/** @brief The device at @p index, or nullptr. */
BaseSpi* SpiDevice(int index) noexcept;
/** @brief Remove every device and release the bus (idempotent). */
void SpiClose() noexcept;

//==============================================================================
// I2C
//==============================================================================

/** @brief Initialise the I2C bus. @return true on success. */
bool I2cOpen(const I2cBusSpec& spec) noexcept;
/** @brief Add a device. @return the device index (>= 0) or -1. */
int I2cAddDevice(const I2cDeviceSpec& spec) noexcept;
/** @brief The device at @p index, or nullptr. */
BaseI2c* I2cDevice(int index) noexcept;
/** @brief Remove one device. @return true if it existed. */
bool I2cRemoveDevice(int index) noexcept;
/** @brief Address-only probe. @return true if a device acknowledged @p address. */
bool I2cProbe(uint8_t address, uint32_t timeout_ms) noexcept;
/**
 * @brief Probe every address in [first, last].
 * @param out Receives the 7-bit addresses that acknowledged.
 * @param max Capacity of @p out.
 * @return Number of addresses written.
 */
size_t I2cScan(uint8_t* out, size_t max, uint8_t first, uint8_t last, uint32_t timeout_ms) noexcept;
/** @brief Recover a stuck bus (clock pulses + re-init). @return true on success. */
bool I2cReset() noexcept;
/** @brief Remove every device and release the bus (idempotent). */
void I2cClose() noexcept;

//==============================================================================
// CAN
//==============================================================================

/** @brief Open the CAN controller. @return the port, or nullptr on failure. */
BaseCan* CanOpen(const CanSpec& spec) noexcept;
/** @brief Close the CAN controller (idempotent). */
void CanClose() noexcept;

//==============================================================================
// MCU GPIO (pins of the pin map whose chip is the host MCU)
//==============================================================================

/**
 * @brief Create the MCU GPIO for a functional pin (one slot per pin; a second call returns the same object).
 * @return The pin, initialised; nullptr if the row is not an MCU pin or init failed.
 */
BaseGpio* McuGpioOpen(HfFunctionalGpioPin pin, hf_gpio_direction_t direction, hf_gpio_active_state_t active,
                      hf_gpio_output_mode_t output_mode, hf_gpio_pull_mode_t pull) noexcept;
/** @brief Destroy every MCU GPIO created by McuGpioOpen(). Pointers handed out become invalid. */
void McuGpioCloseAll() noexcept;

//==============================================================================
// NVS (three fixed namespaces)
//==============================================================================

/** @brief The persistent-storage regions of the board. */
enum class NvsRegion : uint8_t {
    App = 0,      ///< namespace "vortex"
    Calibration,  ///< namespace "vortex_cal"
    Config,       ///< namespace "vortex_cfg"
    Count
};

/** @brief Namespace name of a region. */
[[nodiscard]] constexpr const char* NvsNamespace(NvsRegion region) noexcept {
    switch (region) {
        case NvsRegion::App:         return "vortex";
        case NvsRegion::Calibration: return "vortex_cal";
        case NvsRegion::Config:      return "vortex_cfg";
        default:                     return "";
    }
}

/** @brief The NVS object of a region. Constructing it touches no hardware; call EnsureInitialized() on it. */
BaseNvs& Nvs(NvsRegion region) noexcept;

//==============================================================================
// MCU die temperature
//==============================================================================

/** @brief The MCU's on-chip temperature sensor (not yet initialised), or nullptr if the MCU has none. */
BaseTemperature* McuTemperature() noexcept;

//==============================================================================
// Status LED (one WS2812 on Vortex V1)
//==============================================================================

/** @brief Open the addressable status LED chain on @p data_pin. @return true on success. */
bool StatusLedOpen(int data_pin, uint32_t led_count) noexcept;
/**
 * @brief Latch colours onto the chain.
 * @param rgb One 0xRRGGBB value per LED.
 * @param count Number of entries in @p rgb (at most the count given to StatusLedOpen()).
 * @param brightness Global brightness 0..255 applied by the LED driver.
 * @return true when the frame was sent.
 */
bool StatusLedShow(const uint32_t* rgb, uint32_t count, uint8_t brightness) noexcept;
/** @brief Turn the chain off and release it (idempotent). */
void StatusLedClose() noexcept;

}  // namespace vortex::bsp

#endif  // VORTEX_BOARD_BIND_H_
