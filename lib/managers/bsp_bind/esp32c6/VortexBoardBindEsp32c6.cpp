/**
 * @file VortexBoardBindEsp32c6.cpp
 * @brief ESP32-C6 implementation of the Vortex board binding (VortexBoardBind.h).
 *
 * @details The only translation unit of the HAL that constructs ESP32 peripherals: EspSpiBus, EspI2cBus,
 *          EspUart, EspCan, EspGpio, EspNvs, EspTemperature, and the WS2812 status LED through hf-core's
 *          Ws2812Handler. Every object lives in function-local static storage (`std::optional`, emplaced
 *          on Open and reset on Close), so the binding itself allocates nothing; the ESP-IDF drivers
 *          underneath allocate their queues inside Open.
 *
 *          Thread safety: none; each function is called by its owning manager under that manager's lock.
 */

#include "bsp_bind/VortexBoardBind.h"
#include "bsp_bind/esp32c6/VortexBoardBindEsp32c6.h"

#include <algorithm>
#include <array>
#include <optional>
#include <vector>

#include "handlers/ws2812/Ws2812Handler.h"
#include "mcu/esp32/EspCan.h"
#include "mcu/esp32/EspGpio.h"
#include "mcu/esp32/EspI2c.h"
#include "mcu/esp32/EspNvs.h"
#include "mcu/esp32/EspSpi.h"
#include "mcu/esp32/EspTemperature.h"
#include "mcu/esp32/EspUart.h"

namespace vortex::bsp {
namespace {

std::optional<EspUart>& UartSlot() noexcept {
    static std::optional<EspUart> slot;
    return slot;
}
std::optional<EspSpiBus>& SpiSlot() noexcept {
    static std::optional<EspSpiBus> slot;
    return slot;
}
std::optional<EspI2cBus>& I2cSlot() noexcept {
    static std::optional<EspI2cBus> slot;
    return slot;
}
std::optional<EspCan>& CanSlot() noexcept {
    static std::optional<EspCan> slot;
    return slot;
}

constexpr size_t kGpioSlots = static_cast<size_t>(HfFunctionalGpioPin::HF_FUNCTIONAL_GPIO_COUNT);
std::array<std::optional<EspGpio>, kGpioSlots>& GpioSlots() noexcept {
    static std::array<std::optional<EspGpio>, kGpioSlots> slots;
    return slots;
}

std::optional<Ws2812Handler>& LedSlot() noexcept {
    static std::optional<Ws2812Handler> slot;
    return slot;
}

}  // namespace

//==============================================================================
// UART
//==============================================================================

BaseUart* UartOpen(const UartSpec& spec) noexcept {
    auto& slot = UartSlot();
    if (slot) {
        return &*slot;
    }
    hf_uart_config_t cfg = {};
    cfg.port_number = spec.port;
    cfg.baud_rate = spec.baud;
    cfg.data_bits = hf_uart_data_bits_t::HF_UART_DATA_8_BITS;
    cfg.parity = hf_uart_parity_t::HF_UART_PARITY_DISABLE;
    cfg.stop_bits = hf_uart_stop_bits_t::HF_UART_STOP_BITS_1;
    cfg.flow_control = hf_uart_flow_ctrl_t::HF_UART_HW_FLOWCTRL_DISABLE;
    cfg.tx_pin = spec.tx_pin;
    cfg.rx_pin = spec.rx_pin;
    cfg.rts_pin = HF_UART_IO_UNUSED;
    cfg.cts_pin = HF_UART_IO_UNUSED;
    cfg.tx_buffer_size = spec.tx_buffer_bytes;
    cfg.rx_buffer_size = spec.rx_buffer_bytes;
    cfg.event_queue_size = 20;
    cfg.operating_mode = hf_uart_operating_mode_t::HF_UART_MODE_INTERRUPT;
    cfg.timeout_ms = spec.timeout_ms;
    cfg.enable_pattern_detection = false;
    cfg.enable_wakeup = false;
    cfg.enable_loopback = false;
    slot.emplace(cfg);
    if (!slot->EnsureInitialized()) {
        slot.reset();
        return nullptr;
    }
    return &*slot;
}

void UartClose() noexcept { UartSlot().reset(); }

//==============================================================================
// SPI
//==============================================================================

bool SpiOpen(const SpiBusSpec& spec) noexcept {
    auto& slot = SpiSlot();
    if (slot) {
        return true;
    }
    hf_spi_bus_config_t cfg = {};
    cfg.host = static_cast<hf_host_id_t>(spec.host);
    cfg.miso_pin = spec.miso_pin;
    cfg.mosi_pin = spec.mosi_pin;
    cfg.sclk_pin = spec.sclk_pin;
    cfg.dma_channel = 0;  // auto-select
    cfg.timeout_ms = spec.timeout_ms;
    cfg.use_iomux = true;
    slot.emplace(cfg);
    if (!slot->Initialize()) {
        slot.reset();
        return false;
    }
    return true;
}

int SpiAddDevice(const SpiDeviceSpec& spec) noexcept {
    auto& slot = SpiSlot();
    if (!slot || spec.mode > 3) {
        return -1;
    }
    hf_spi_device_config_t dev = {};
    dev.cs_pin = spec.cs_pin;
    dev.clock_speed_hz = spec.clock_hz;
    dev.mode = static_cast<hf_spi_mode_t>(spec.mode);
    return slot->CreateDevice(dev);
}

BaseSpi* SpiDevice(int index) noexcept {
    auto& slot = SpiSlot();
    return (slot && index >= 0) ? slot->GetDevice(index) : nullptr;
}

void SpiClose() noexcept { SpiSlot().reset(); }

//==============================================================================
// I2C
//==============================================================================

bool I2cOpen(const I2cBusSpec& spec) noexcept {
    auto& slot = I2cSlot();
    if (slot) {
        return true;
    }
    hf_i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = static_cast<i2c_port_t>(spec.port);
    cfg.sda_io_num = static_cast<hf_pin_num_t>(spec.sda_pin);
    cfg.scl_io_num = static_cast<hf_pin_num_t>(spec.scl_pin);
    cfg.flags.enable_internal_pullup = spec.internal_pullups;
    cfg.clk_source = hf_i2c_clock_source_t::HF_I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = hf_i2c_glitch_filter_t::HF_I2C_GLITCH_FILTER_7_CYCLES;
    cfg.trans_queue_depth = 0;  // synchronous
    cfg.intr_priority = 0;
    cfg.flags.allow_pd = false;
    slot.emplace(cfg);
    if (!slot->Initialize()) {
        slot.reset();
        return false;
    }
    return true;
}

int I2cAddDevice(const I2cDeviceSpec& spec) noexcept {
    auto& slot = I2cSlot();
    if (!slot) {
        return -1;
    }
    hf_i2c_device_config_t dev = {};
    dev.device_address = spec.address;
    dev.dev_addr_length = hf_i2c_address_bits_t::HF_I2C_ADDR_7_BIT;
    dev.scl_speed_hz = spec.scl_hz;
    dev.disable_ack_check = false;
    if (spec.scl_wait_us != 0) {
        dev.scl_wait_us = spec.scl_wait_us;
    }
    return slot->CreateDevice(dev);
}

BaseI2c* I2cDevice(int index) noexcept {
    auto& slot = I2cSlot();
    return (slot && index >= 0) ? slot->GetDevice(index) : nullptr;
}

bool I2cRemoveDevice(int index) noexcept {
    auto& slot = I2cSlot();
    return slot && index >= 0 && slot->RemoveDevice(index);
}

bool I2cProbe(uint8_t address, uint32_t timeout_ms) noexcept {
    auto& slot = I2cSlot();
    return slot && slot->ProbeDevice(static_cast<hf_u16_t>(address), timeout_ms);
}

size_t I2cScan(uint8_t* out, size_t max, uint8_t first, uint8_t last, uint32_t timeout_ms) noexcept {
    auto& slot = I2cSlot();
    if (!slot || out == nullptr || max == 0) {
        return 0;
    }
    // ScanDevices() reports through a vector; a diagnostic path, not a control path.
    std::vector<hf_u16_t> found;
    const size_t total = slot->ScanDevices(found, first, last, timeout_ms);
    const size_t n = std::min(total, max);
    for (size_t i = 0; i < n; ++i) {
        out[i] = static_cast<uint8_t>(found[i] & 0x7F);
    }
    return n;
}

bool I2cReset() noexcept {
    auto& slot = I2cSlot();
    return slot && slot->ResetBus();
}

void I2cClose() noexcept { I2cSlot().reset(); }

//==============================================================================
// CAN
//==============================================================================

BaseCan* CanOpen(const CanSpec& spec) noexcept {
    auto& slot = CanSlot();
    if (slot) {
        return &*slot;
    }
    hf_esp_can_config_t cfg = {};
    cfg.controller_id = hf_can_controller_id_t::HF_CAN_CONTROLLER_0;
    cfg.tx_pin = static_cast<hf_pin_num_t>(spec.tx_pin);
    cfg.rx_pin = static_cast<hf_pin_num_t>(spec.rx_pin);
    cfg.baud_rate = spec.bitrate;
    slot.emplace(cfg);
    if (!slot->EnsureInitialized()) {
        slot.reset();
        return nullptr;
    }
    return &*slot;
}

void CanClose() noexcept { CanSlot().reset(); }

namespace esp32c6 {

EspCan* CanController() noexcept {
    auto& slot = CanSlot();
    return slot ? &*slot : nullptr;
}

}  // namespace esp32c6

//==============================================================================
// MCU GPIO
//==============================================================================

BaseGpio* McuGpioOpen(HfFunctionalGpioPin pin, hf_gpio_direction_t direction, hf_gpio_active_state_t active,
                      hf_gpio_output_mode_t output_mode, hf_gpio_pull_mode_t pull) noexcept {
    const size_t index = static_cast<size_t>(pin);
    const HfGpioMapping* row = GetGpioMapping(pin);
    if (index >= kGpioSlots || row == nullptr ||
        row->chip_type != static_cast<uint8_t>(HfGpioChipType::ESP32_INTERNAL)) {
        return nullptr;
    }
    auto& slot = GpioSlots()[index];
    if (!slot) {
        slot.emplace(static_cast<hf_pin_num_t>(row->physical_pin), direction, active, output_mode, pull);
        if (!slot->EnsureInitialized()) {
            slot.reset();
            return nullptr;
        }
    }
    return &*slot;
}

void McuGpioCloseAll() noexcept {
    for (auto& slot : GpioSlots()) {
        slot.reset();
    }
}

//==============================================================================
// NVS
//==============================================================================

BaseNvs& Nvs(NvsRegion region) noexcept {
    static EspNvs app(NvsNamespace(NvsRegion::App));
    static EspNvs calibration(NvsNamespace(NvsRegion::Calibration));
    static EspNvs config(NvsNamespace(NvsRegion::Config));
    switch (region) {
        case NvsRegion::Calibration: return calibration;
        case NvsRegion::Config:      return config;
        default:                     return app;
    }
}

//==============================================================================
// MCU die temperature
//==============================================================================

BaseTemperature* McuTemperature() noexcept {
    static EspTemperature sensor;
    return &sensor;
}

//==============================================================================
// Status LED
//==============================================================================

bool StatusLedOpen(int data_pin, uint32_t led_count) noexcept {
    auto& slot = LedSlot();
    if (slot && slot->IsInitialized()) {
        return true;
    }
    Ws2812Handler::Config cfg{};
    cfg.gpio_pin = static_cast<gpio_num_t>(data_pin);
    cfg.num_leds = led_count;
    cfg.led_type = LedType::RGB;
    cfg.brightness = 255;
    cfg.rmt_channel = 0;
    // RMT symbol timings the Vortex LED has been brought up with (t0h, t1h, t0l, t1l).
    cfg.t0h = 14;
    cfg.t1h = 52;
    cfg.t0l = 52;
    cfg.t1l = 52;
    slot.emplace(cfg);
    if (slot->Initialize() != ESP_OK) {
        slot.reset();
        return false;
    }
    return true;
}

bool StatusLedShow(const uint32_t* rgb, uint32_t count, uint8_t brightness) noexcept {
    auto& slot = LedSlot();
    if (!slot || rgb == nullptr) {
        return false;
    }
    return slot->visitDriver([&](WS2812Strip& strip) {
        const uint32_t n = std::min<uint32_t>(count, slot->GetNumLeds());
        strip.SetBrightness(brightness);
        for (uint32_t i = 0; i < n; ++i) {
            strip.SetPixel(i, rgb[i]);
        }
        return strip.Show() == ESP_OK;
    });
}

void StatusLedClose() noexcept {
    auto& slot = LedSlot();
    if (slot) {
        (void)slot->Deinitialize();
        slot.reset();
    }
}

}  // namespace vortex::bsp
