/**
 * @file VortexBoardBindHost.cpp
 * @brief Host implementation of VortexBoardBind.h over hf-core's host mocks (tests/host only).
 */

#include "bsp_bind/host/VortexBoardBindHost.h"

#include "RtosMutex.h"  // RtosTime: FakeClock on the host

#include <algorithm>
#include <array>
#include <optional>
#include <cstdint>
#include <set>

namespace vortex::bsp {
namespace {

using hf::host::MockCan;
using hf::host::MockGpio;
using hf::host::MockI2c;
using hf::host::MockNvs;
using hf::host::MockSpi;
using hf::host::MockTemperature;
using hf::host::MockUart;

constexpr size_t kMaxSpiDevices = 8;
constexpr size_t kMaxI2cDevices = 12;
constexpr size_t kGpioSlots = static_cast<size_t>(HfFunctionalGpioPin::HF_FUNCTIONAL_GPIO_COUNT);

struct I2cSlot {
    std::optional<MockI2c> device;
    uint8_t address{0};
};

struct State {
    host::OpenLog log;
    host::Failures fail;
    std::optional<MockUart> uart;
    std::optional<MockCan> can;
    bool spi_open{false};
    std::array<std::optional<MockSpi>, kMaxSpiDevices> spi;
    bool i2c_open{false};
    std::array<I2cSlot, kMaxI2cDevices> i2c;
    std::set<uint8_t> i2c_present;
    std::array<std::optional<MockGpio>, kGpioSlots> gpio;
    std::array<MockNvs, static_cast<size_t>(NvsRegion::Count)> nvs{
        MockNvs{NvsNamespace(NvsRegion::App)}, MockNvs{NvsNamespace(NvsRegion::Calibration)},
        MockNvs{NvsNamespace(NvsRegion::Config)}};
    MockTemperature temperature;
    bool led_open{false};
    std::vector<host::LedFrame> led_frames;
};

State& S() noexcept {
    static State state;
    return state;
}

}  // namespace

BaseUart* UartOpen(const UartSpec& spec) noexcept {
    auto& s = S();
    s.log.uart = spec;
    if (s.fail.uart) {
        return nullptr;
    }
    if (!s.uart) {
        s.uart.emplace(static_cast<hf_port_num_t>(spec.port));
    }
    return &*s.uart;
}
void UartClose() noexcept { S().uart.reset(); }

bool SpiOpen(const SpiBusSpec& spec) noexcept {
    auto& s = S();
    s.log.spi = spec;
    s.spi_open = !s.fail.spi;
    return s.spi_open;
}
int SpiAddDevice(const SpiDeviceSpec& spec) noexcept {
    auto& s = S();
    if (!s.spi_open) {
        return -1;
    }
    for (size_t i = 0; i < s.spi.size(); ++i) {
        if (!s.spi[i]) {
            s.spi[i].emplace();
            s.log.spi_devices.push_back(spec);
            return static_cast<int>(i);
        }
    }
    return -1;
}
BaseSpi* SpiDevice(int index) noexcept {
    auto& s = S();
    if (!s.spi_open || index < 0 || static_cast<size_t>(index) >= s.spi.size() || !s.spi[index]) {
        return nullptr;
    }
    return &*s.spi[index];
}
void SpiClose() noexcept {
    auto& s = S();
    for (auto& d : s.spi) {
        d.reset();
    }
    s.spi_open = false;
}

bool I2cOpen(const I2cBusSpec& spec) noexcept {
    auto& s = S();
    s.log.i2c = spec;
    s.i2c_open = !s.fail.i2c;
    return s.i2c_open;
}
int I2cAddDevice(const I2cDeviceSpec& spec) noexcept {
    auto& s = S();
    if (!s.i2c_open) {
        return -1;
    }
    for (size_t i = 0; i < s.i2c.size(); ++i) {
        if (!s.i2c[i].device) {
            s.i2c[i].device.emplace(spec.address);
            s.i2c[i].address = static_cast<uint8_t>(spec.address);
            if (s.i2c_present.count(static_cast<uint8_t>(spec.address)) == 0) {
                // No chip at this address: every transfer is NACKed, as on a bare board.
                s.i2c[i].device->FailNext(hf_i2c_err_t::I2C_ERR_DEVICE_NACK, SIZE_MAX);
            }
            s.log.i2c_devices.push_back(spec);
            return static_cast<int>(i);
        }
    }
    return -1;
}
BaseI2c* I2cDevice(int index) noexcept {
    auto& s = S();
    if (!s.i2c_open || index < 0 || static_cast<size_t>(index) >= s.i2c.size() || !s.i2c[index].device) {
        return nullptr;
    }
    return &*s.i2c[index].device;
}
bool I2cRemoveDevice(int index) noexcept {
    auto& s = S();
    if (index < 0 || static_cast<size_t>(index) >= s.i2c.size() || !s.i2c[index].device) {
        return false;
    }
    s.i2c[index].device.reset();
    return true;
}
bool I2cProbe(uint8_t address, uint32_t) noexcept {
    auto& s = S();
    return s.i2c_open && s.i2c_present.count(address) != 0;
}
size_t I2cScan(uint8_t* out, size_t max, uint8_t first, uint8_t last, uint32_t) noexcept {
    auto& s = S();
    if (!s.i2c_open || out == nullptr) {
        return 0;
    }
    size_t n = 0;
    for (uint8_t a : s.i2c_present) {
        if (a >= first && a <= last && n < max) {
            out[n++] = a;
        }
    }
    return n;
}
bool I2cReset() noexcept { return S().i2c_open; }
void I2cClose() noexcept {
    auto& s = S();
    for (auto& d : s.i2c) {
        d.device.reset();
    }
    s.i2c_open = false;
}

BaseCan* CanOpen(const CanSpec& spec) noexcept {
    auto& s = S();
    s.log.can = spec;
    if (s.fail.can) {
        return nullptr;
    }
    if (!s.can) {
        s.can.emplace();
    }
    return &*s.can;
}
void CanClose() noexcept { S().can.reset(); }

BaseGpio* McuGpioOpen(HfFunctionalGpioPin pin, hf_gpio_direction_t direction, hf_gpio_active_state_t active,
                      hf_gpio_output_mode_t, hf_gpio_pull_mode_t) noexcept {
    const size_t index = static_cast<size_t>(pin);
    const HfGpioMapping* row = GetGpioMapping(pin);
    if (index >= kGpioSlots || row == nullptr ||
        row->chip_type != static_cast<uint8_t>(HfGpioChipType::ESP32_INTERNAL)) {
        return nullptr;
    }
    auto& slot = S().gpio[index];
    if (!slot) {
        slot.emplace(static_cast<hf_pin_num_t>(row->physical_pin), direction, active);
        if (!slot->EnsureInitialized()) {
            slot.reset();
            return nullptr;
        }
    }
    return &*slot;
}
void McuGpioCloseAll() noexcept {
    for (auto& g : S().gpio) {
        g.reset();
    }
}

BaseNvs& Nvs(NvsRegion region) noexcept { return host::NvsMock(region); }

BaseTemperature* McuTemperature() noexcept { return &S().temperature; }

uint64_t MonotonicMicros() noexcept { return RtosTime::GetCurrentTimeUs(); }

bool StatusLedOpen(int data_pin, uint32_t led_count) noexcept {
    auto& s = S();
    s.log.led_pin = data_pin;
    s.log.led_count = led_count;
    s.led_open = !s.fail.led;
    return s.led_open;
}
bool StatusLedShow(const uint32_t* rgb, uint32_t count, uint8_t brightness) noexcept {
    auto& s = S();
    if (!s.led_open || rgb == nullptr || count == 0 || s.fail.led_show) {
        return false;
    }
    s.led_frames.push_back({rgb[0], brightness});
    return true;
}
void StatusLedClose() noexcept { S().led_open = false; }

namespace host {

void Reset() noexcept {
    UartClose();
    SpiClose();
    I2cClose();
    CanClose();
    McuGpioCloseAll();
    StatusLedClose();
    auto& s = S();
    s.log = OpenLog{};
    s.fail = Failures{};
    s.i2c_present.clear();
    s.led_frames.clear();
    s.led_frames.reserve(4096);  // so a test can count the HAL's own allocations
    for (size_t i = 0; i < s.nvs.size(); ++i) {
        (void)s.nvs[i].EnsureDeinitialized();
    }
    (void)s.temperature.EnsureDeinitialized();
    s.temperature.SetInitializeResult(true);
    s.temperature.Fail(hf_temp_err_t::TEMP_SUCCESS);
    s.temperature.SetCelsius(25.0f);
}

Failures& Fail() noexcept { return S().fail; }
const OpenLog& Log() noexcept { return S().log; }

void SetI2cPresent(uint8_t address, bool present) noexcept {
    if (present) {
        S().i2c_present.insert(address);
    } else {
        S().i2c_present.erase(address);
    }
}

MockUart* Uart() noexcept { return S().uart ? &*S().uart : nullptr; }
MockCan* Can() noexcept { return S().can ? &*S().can : nullptr; }
MockSpi* Spi(int index) noexcept { return static_cast<MockSpi*>(SpiDevice(index)); }
MockI2c* I2c(int index) noexcept { return static_cast<MockI2c*>(I2cDevice(index)); }
MockI2c* I2cAt(uint8_t address) noexcept {
    for (auto& d : S().i2c) {
        if (d.device && d.address == address) {
            return &*d.device;
        }
    }
    return nullptr;
}
MockGpio* Gpio(HfFunctionalGpioPin pin) noexcept {
    const size_t index = static_cast<size_t>(pin);
    return (index < kGpioSlots && S().gpio[index]) ? &*S().gpio[index] : nullptr;
}
MockNvs& NvsMock(NvsRegion region) noexcept {
    const size_t i = std::min(static_cast<size_t>(region), static_cast<size_t>(NvsRegion::Count) - 1);
    return S().nvs[i];
}
MockTemperature& Temperature() noexcept { return S().temperature; }
const std::vector<LedFrame>& LedFrames() noexcept { return S().led_frames; }
bool LedOpen() noexcept { return S().led_open; }

}  // namespace host
}  // namespace vortex::bsp
