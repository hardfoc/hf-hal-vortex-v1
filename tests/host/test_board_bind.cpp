/**
 * @file test_board_bind.cpp
 * @brief The managers reach the board only through bsp_bind/, and configure it as the pin map says.
 *
 * Runs Vortex::EnsureInitialized() on the host binding (bsp_bind/host/), then reads back what the
 * managers asked the binding for: bus pins, SPI modes, I2C devices, UART framing, CAN bit rate, the
 * status LED pin and the NVS namespaces. No chip answers, so the TMC9660, IMU and expander stay down
 * and the facade reports them as warnings rather than failing.
 */

#include "HostTest.h"
#include "VortexBoardBindHost.h"
#include "api/Vortex.h"
#include "managers/CommChannelsManager.h"
#include "managers/LedManager.h"
#include "managers/NvsManager.h"

namespace bind = vortex::bsp;
namespace host = vortex::bsp::host;

namespace {

int Pin(HfFunctionalGpioPin p) { return static_cast<int>(HF_GPIO_MAPPING[static_cast<size_t>(p)].physical_pin); }

/** One bring-up for the whole file: the managers are process-wide singletons. */
Vortex& Booted() {
    static bool done = false;
    auto& v = Vortex::GetInstance();
    if (!done) {
        host::Reset();
        done = true;
        (void)v.EnsureInitialized();
    }
    return v;
}

}  // namespace

HF_TEST(Bind_CoreSubsystemsComeUpOnTheHostBinding) {
    Vortex& v = Booted();
    CHECK(v.IsInitialized());
    const auto status = v.GetComponentInitializationStatus();
    CHECK(status[0]);  // NVS
    CHECK(status[1]);  // comms
    CHECK(status[2]);  // GPIO
    CHECK(status[7]);  // LEDs
}

HF_TEST(Bind_DegradedSubsystemsAreReportedAsWarnings) {
    Vortex& v = Booted();
    const char* warnings[VortexBootReport::kCapacity] = {};
    const size_t n = v.GetSystemWarnings(warnings, VortexBootReport::kCapacity);
    CHECK(n >= 1u);  // at least: no TMC9660 on the host
    for (size_t i = 0; i < n; ++i) {
        CHECK(warnings[i] != nullptr);
    }
}

HF_TEST(Bind_SpiBusAndChipSelectsFollowThePinMap) {
    (void)Booted();
    const auto& log = host::Log();
    CHECK_EQ(log.spi.host, 1u);
    CHECK_EQ(log.spi.miso_pin, Pin(HfFunctionalGpioPin::SPI2_MISO));
    CHECK_EQ(log.spi.mosi_pin, Pin(HfFunctionalGpioPin::SPI2_MOSI));
    CHECK_EQ(log.spi.sclk_pin, Pin(HfFunctionalGpioPin::SPI2_SCK));
    REQUIRE(log.spi_devices.size() == 4u);
    CHECK_EQ(log.spi_devices[0].cs_pin, Pin(HfFunctionalGpioPin::SPI2_CS_TMC9660));
    CHECK_EQ(log.spi_devices[0].mode, 3u);  // TMC9660
    CHECK_EQ(log.spi_devices[1].cs_pin, Pin(HfFunctionalGpioPin::SPI2_CS_AS5047));
    CHECK_EQ(log.spi_devices[1].mode, 1u);  // AS5047U
    CHECK_EQ(log.spi_devices[2].mode, 0u);
    CHECK_EQ(log.spi_devices[3].mode, 0u);
    for (const auto& d : log.spi_devices) {
        CHECK_EQ(d.clock_hz, 10'000'000u);
    }
    auto& comms = CommChannelsManager::GetInstance();
    CHECK(comms.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER) != nullptr);
    CHECK(comms.GetSpiDevice(SpiDeviceId::DEVICE_COUNT) == nullptr);
}

HF_TEST(Bind_I2cDevicesAndClockStretchForTheBno08x) {
    (void)Booted();
    const auto& log = host::Log();
    CHECK_EQ(log.i2c.sda_pin, Pin(HfFunctionalGpioPin::I2C_SDA));
    CHECK_EQ(log.i2c.scl_pin, Pin(HfFunctionalGpioPin::I2C_SCL));
    REQUIRE(log.i2c_devices.size() >= 2u);
    CHECK_EQ(log.i2c_devices[0].address, 0x4Au);  // BNO08x
    CHECK_EQ(log.i2c_devices[0].scl_wait_us, 50000u);
    CHECK_EQ(log.i2c_devices[1].address, 0x20u);  // PCAL95555
    CHECK_EQ(log.i2c_devices[1].scl_hz, 100000u);
}

HF_TEST(Bind_UartCanAndLedSettings) {
    (void)Booted();
    const auto& log = host::Log();
    CHECK_EQ(log.uart.port, 1u);
    CHECK_EQ(log.uart.baud, 115200u);
    CHECK_EQ(log.uart.tx_pin, Pin(HfFunctionalGpioPin::UART_TXD));
    CHECK_EQ(log.uart.rx_pin, Pin(HfFunctionalGpioPin::UART_RXD));
    CHECK_EQ(log.can.bitrate, 500000u);
    CHECK_EQ(log.can.tx_pin, Pin(HfFunctionalGpioPin::TWAI_TX));
    CHECK_EQ(log.led_pin, Pin(HfFunctionalGpioPin::WS2812_LED_DAT));
    CHECK_EQ(log.led_count, 1u);
    CHECK(host::Uart() != nullptr);
    CHECK(host::Can() != nullptr);
}

HF_TEST(Bind_NvsNamespacesAreTheBoardRegions) {
    (void)Booted();
    auto& nvs = NvsManager::GetInstance();
    CHECK(nvs.IsInitialized());
    CHECK(&nvs.App() == &host::NvsMock(bind::NvsRegion::App));
    CHECK(&nvs.Calibration() == &host::NvsMock(bind::NvsRegion::Calibration));
    CHECK(&nvs.Config() == &host::NvsMock(bind::NvsRegion::Config));
}

HF_TEST(Bind_StatusLedFramesGoThroughTheBinding) {
    (void)Booted();
    auto& leds = LedManager::GetInstance();
    const size_t before = host::LedFrames().size();
    CHECK(leds.SetColor(LedColors::RED) == LedError::SUCCESS);
    REQUIRE(host::LedFrames().size() == before + 1);
    CHECK_EQ(host::LedFrames().back().rgb, 0xFF0000u);
    CHECK(leds.SetBrightness(40) == LedError::SUCCESS);
    CHECK_EQ(host::LedFrames().back().brightness, 40u);
    CHECK_EQ(host::LedFrames().back().rgb, 0xFF0000u);  // colour kept

    host::Fail().led_show = true;
    CHECK(leds.SetColor(LedColors::GREEN) == LedError::HARDWARE_ERROR);
    host::Fail().led_show = false;
}

HF_TEST(Bind_ShutdownClosesWhatInitOpened) {
    Vortex& v = Booted();
    CHECK(v.Shutdown());
    CHECK(!v.IsInitialized());
    CHECK(host::Uart() == nullptr);
    CHECK(host::Can() == nullptr);
    CHECK(!host::LedOpen());
    CHECK(CommChannelsManager::GetInstance().GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER) == nullptr);
}
