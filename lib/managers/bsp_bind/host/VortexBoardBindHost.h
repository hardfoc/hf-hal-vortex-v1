/**
 * @file VortexBoardBindHost.h
 * @brief Host implementation of the Vortex board binding, over the hf-core host mocks.
 *
 * @details Built only by `tests/host`. Every bus, pin and store the managers open through
 *          VortexBoardBind.h is an `hf::host::Mock*` object that a test can reach through the accessors
 *          below to script it (responders, injected faults) and inspect what the manager sent. Reset()
 *          closes everything and clears the scripted failures; call it at the start of each test case.
 *
 *          Thread safety: none (host tests are single-threaded).
 */

#ifndef VORTEX_BOARD_BIND_HOST_H_
#define VORTEX_BOARD_BIND_HOST_H_

#include <cstdint>
#include <vector>

#include "HfCoreMocks.h"
#include "bsp_bind/VortexBoardBind.h"

namespace vortex::bsp::host {

/** @brief Every Open() call the managers made, and what the binding was asked to configure. */
struct OpenLog {
    UartSpec uart{};
    SpiBusSpec spi{};
    std::vector<SpiDeviceSpec> spi_devices;
    I2cBusSpec i2c{};
    std::vector<I2cDeviceSpec> i2c_devices;
    CanSpec can{};
    int led_pin{-1};
    uint32_t led_count{0};
};

/** @brief One frame latched onto the status LED. */
struct LedFrame {
    uint32_t rgb;
    uint8_t brightness;
};

/** @brief Which Open() calls fail (the binding returns failure without creating the object). */
struct Failures {
    bool uart{false};
    bool spi{false};
    bool i2c{false};
    bool can{false};
    bool led{false};
    bool led_show{false};
};

/** @brief Close every object, forget the log and the failures, and remove every simulated I2C chip. */
void Reset() noexcept;

/** @brief Failure injection for the next Open() calls (cleared by Reset()). */
Failures& Fail() noexcept;

/** @brief What the managers asked the binding for since the last Reset(). */
const OpenLog& Log() noexcept;

/**
 * @brief Simulate a chip that acknowledges @p address on the I2C bus (probe and scan see it).
 * @details Call before the device is added (before EnsureInitialized()). A device added at an address
 *          that is not present NACKs every transfer, as on a board without that chip; a test that wants
 *          a chip model sets one on I2cAt(address) after marking it present.
 */
void SetI2cPresent(uint8_t address, bool present = true) noexcept;

hf::host::MockUart* Uart() noexcept;
hf::host::MockCan* Can() noexcept;
/** @brief SPI device by the index SpiAddDevice() returned. */
hf::host::MockSpi* Spi(int index) noexcept;
/** @brief I2C device by the index I2cAddDevice() returned. */
hf::host::MockI2c* I2c(int index) noexcept;
/** @brief I2C device added at @p address, or nullptr. */
hf::host::MockI2c* I2cAt(uint8_t address) noexcept;
/** @brief MCU GPIO of a functional pin, or nullptr if McuGpioOpen() was not called for it. */
hf::host::MockGpio* Gpio(HfFunctionalGpioPin pin) noexcept;
hf::host::MockNvs& NvsMock(NvsRegion region) noexcept;
hf::host::MockTemperature& Temperature() noexcept;
/** @brief Frames latched onto the status LED since the last Reset(). */
const std::vector<LedFrame>& LedFrames() noexcept;
/** @brief True while the status LED is open. */
bool LedOpen() noexcept;

}  // namespace vortex::bsp::host

#endif  // VORTEX_BOARD_BIND_HOST_H_
