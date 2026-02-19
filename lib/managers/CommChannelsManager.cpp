/**
 * @file CommChannelsManager.cpp
 * @brief Implementation of the Vortex V1 communication channels manager.
 *
 * @details Configures four communication buses for the Vortex board:
 *          - UART : TMC9660 TMCL protocol (115200 8N1)
 *          - SPI  : TMC9660 (Mode 3) + AS5047U (Mode 1) + 2 ext (Mode 0)
 *          - I2C  : BNO08x IMU (0x4A) + PCAL95555 (0x20) @ 400 kHz
 *          - CAN  : TWAI 500 kbps
 *
 * @version 2.0
 */

#include "CommChannelsManager.h"

#include "mcu/esp32/EspSpi.h"
#include "mcu/esp32/EspI2c.h"
#include "mcu/esp32/EspUart.h"
#include "mcu/esp32/EspCan.h"
#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"
#include "handlers/logger/Logger.h"

static constexpr const char* TAG = "VortexComm";

/// CS pin configuration — order matches SpiDeviceId enum.
static constexpr HfFunctionalGpioPin kSpiCsPins[] = {
    HfFunctionalGpioPin::SPI2_CS_TMC9660,
    HfFunctionalGpioPin::SPI2_CS_AS5047,
    HfFunctionalGpioPin::EXT_GPIO_CS_1,
    HfFunctionalGpioPin::EXT_GPIO_CS_2,
};
static_assert(sizeof(kSpiCsPins) / sizeof(kSpiCsPins[0]) ==
                  static_cast<size_t>(SpiDeviceId::DEVICE_COUNT),
              "CS pin count must match SpiDeviceId::DEVICE_COUNT");

// Destructor defined here where bus types are complete
CommChannelsManager::~CommChannelsManager() = default;

//==============================================================================
// SINGLETON
//==============================================================================

CommChannelsManager& CommChannelsManager::GetInstance() noexcept {
    static CommChannelsManager instance;
    return instance;
}

CommChannelsManager::CommChannelsManager() noexcept {
    spi_device_indices_.fill(-1);
    i2c_builtin_indices_.fill(-1);
}

//==============================================================================
// INITIALIZATION
//==============================================================================

bool CommChannelsManager::EnsureInitialized() noexcept {
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    MutexLockGuard lock(mutex_);
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    bool ok = Initialize();
    initialized_.store(ok, std::memory_order_release);
    return ok;
}

bool CommChannelsManager::Initialize() noexcept {
    Logger::GetInstance().Info(TAG, "Initializing Vortex V1 communication channels");

    //==========================================================================
    // UART — TMC9660 TMCL protocol (115200 8N1)
    //==========================================================================
    {
        auto* tx_map = GetGpioMapping(HfFunctionalGpioPin::UART_TXD);
        auto* rx_map = GetGpioMapping(HfFunctionalGpioPin::UART_RXD);

        if (tx_map && rx_map) {
            hf_uart_config_t uart_cfg = {};
            uart_cfg.port_number     = 0;
            uart_cfg.baud_rate       = 115200;
            uart_cfg.data_bits       = hf_uart_data_bits_t::HF_UART_DATA_8_BITS;
            uart_cfg.parity          = hf_uart_parity_t::HF_UART_PARITY_DISABLE;
            uart_cfg.stop_bits       = hf_uart_stop_bits_t::HF_UART_STOP_BITS_1;
            uart_cfg.flow_control    = hf_uart_flow_ctrl_t::HF_UART_HW_FLOWCTRL_DISABLE;
            uart_cfg.tx_pin          = tx_map->physical_pin;
            uart_cfg.rx_pin          = rx_map->physical_pin;
            uart_cfg.rts_pin         = HF_UART_IO_UNUSED;
            uart_cfg.cts_pin         = HF_UART_IO_UNUSED;
            uart_cfg.tx_buffer_size  = 256;
            uart_cfg.rx_buffer_size  = 512;
            uart_cfg.event_queue_size = 20;
            uart_cfg.operating_mode  = hf_uart_operating_mode_t::HF_UART_MODE_INTERRUPT;
            uart_cfg.timeout_ms      = 100;
            uart_cfg.enable_pattern_detection = false;
            uart_cfg.enable_wakeup   = false;
            uart_cfg.enable_loopback = false;

            uart_bus_ = std::make_unique<EspUart>(uart_cfg);
            if (uart_bus_ && uart_bus_->EnsureInitialized()) {
                uart_bus_valid_ = true;
                Logger::GetInstance().Info(TAG, "UART configured: 115200 8N1, TX=GPIO%d, RX=GPIO%d",
                                          tx_map->physical_pin, rx_map->physical_pin);
            } else {
                Logger::GetInstance().Error(TAG, "UART initialization failed");
                uart_bus_.reset();
            }
        } else {
            Logger::GetInstance().Error(TAG, "UART pin mapping missing");
        }
    }

    //==========================================================================
    // SPI BUS (SPI2_HOST) — TMC9660 + AS5047U + 2 external slots
    //==========================================================================
    {
        auto* miso_map = GetGpioMapping(HfFunctionalGpioPin::SPI2_MISO);
        auto* mosi_map = GetGpioMapping(HfFunctionalGpioPin::SPI2_MOSI);
        auto* sck_map  = GetGpioMapping(HfFunctionalGpioPin::SPI2_SCK);

        if (miso_map && mosi_map && sck_map) {
            hf_spi_bus_config_t spi_cfg = {};
            spi_cfg.host        = static_cast<hf_host_id_t>(1); // SPI2_HOST
            spi_cfg.miso_pin    = miso_map->physical_pin;
            spi_cfg.mosi_pin    = mosi_map->physical_pin;
            spi_cfg.sclk_pin    = sck_map->physical_pin;
            spi_cfg.dma_channel = 0;    // auto-select
            spi_cfg.timeout_ms  = 1000;
            spi_cfg.use_iomux   = true;

            spi_bus_ = std::make_unique<EspSpiBus>(spi_cfg);
            if (spi_bus_ && spi_bus_->Initialize()) {
                spi_bus_valid_ = true;

                // Create devices for each CS pin (order matches SpiDeviceId enum)
                for (uint8_t i = 0; i < kSpiDeviceCount; ++i) {
                    auto* cs_map = GetGpioMapping(kSpiCsPins[i]);
                    if (!cs_map) continue;

                    hf_spi_device_config_t dev_cfg = {};
                    dev_cfg.cs_pin         = cs_map->physical_pin;
                    dev_cfg.clock_speed_hz = 10'000'000; // 10 MHz default

                    // Per-device SPI mode
                    if (i == static_cast<uint8_t>(SpiDeviceId::TMC9660_MOTOR_CONTROLLER)) {
                        dev_cfg.mode = hf_spi_mode_t::HF_SPI_MODE_3;
                    } else if (i == static_cast<uint8_t>(SpiDeviceId::AS5047U_POSITION_ENCODER)) {
                        dev_cfg.mode = hf_spi_mode_t::HF_SPI_MODE_1;
                    } else {
                        dev_cfg.mode = hf_spi_mode_t::HF_SPI_MODE_0;
                    }

                    int idx = spi_bus_->CreateDevice(dev_cfg);
                    spi_device_indices_[i] = idx;
                    Logger::GetInstance().Info(TAG, "SPI device %d added (CS=GPIO%d, mode=%d, idx=%d)",
                                              i, cs_map->physical_pin,
                                              static_cast<int>(dev_cfg.mode), idx);
                }
            } else {
                Logger::GetInstance().Error(TAG, "SPI bus initialization failed");
                spi_bus_.reset();
            }
        } else {
            Logger::GetInstance().Error(TAG, "SPI pin mapping missing");
        }
    }

    //==========================================================================
    // I2C BUS — BNO08x IMU (0x4A) + PCAL95555 GPIO expander (0x20)
    //==========================================================================
    {
        auto* sda_map = GetGpioMapping(HfFunctionalGpioPin::I2C_SDA);
        auto* scl_map = GetGpioMapping(HfFunctionalGpioPin::I2C_SCL);

        if (sda_map && scl_map) {
            hf_i2c_master_bus_config_t i2c_cfg = {};
            i2c_cfg.i2c_port   = I2C_NUM_0;
            i2c_cfg.sda_io_num = static_cast<hf_pin_num_t>(sda_map->physical_pin);
            i2c_cfg.scl_io_num = static_cast<hf_pin_num_t>(scl_map->physical_pin);
            i2c_cfg.flags.enable_internal_pullup = sda_map->has_pull && scl_map->has_pull;
            i2c_cfg.clk_source = hf_i2c_clock_source_t::HF_I2C_CLK_SRC_DEFAULT;
            i2c_cfg.glitch_ignore_cnt = hf_i2c_glitch_filter_t::HF_I2C_GLITCH_FILTER_7_CYCLES;
            i2c_cfg.trans_queue_depth = 8;
            i2c_cfg.intr_priority = 5;
            i2c_cfg.flags.allow_pd = false;

            i2c_bus_ = std::make_unique<EspI2cBus>(i2c_cfg);
            if (i2c_bus_ && i2c_bus_->Initialize()) {
                i2c_bus_valid_ = true;
                Logger::GetInstance().Info(TAG, "I2C bus configured (SDA=GPIO%d, SCL=GPIO%d, 400kHz)",
                                          sda_map->physical_pin, scl_map->physical_pin);

                // Device 0: BNO08x IMU @ 0x4A
                {
                    hf_i2c_device_config_t dev = {};
                    dev.device_address   = 0x4A;
                    dev.dev_addr_length  = hf_i2c_address_bits_t::HF_I2C_ADDR_7_BIT;
                    dev.scl_speed_hz     = 400000;
                    dev.disable_ack_check = false;

                    int idx = i2c_bus_->CreateDevice(dev);
                    i2c_builtin_indices_[static_cast<uint8_t>(I2cDeviceId::BNO08X_IMU)] = idx;
                    Logger::GetInstance().Info(TAG, "BNO08x IMU I2C device added (addr=0x4A, idx=%d)", idx);
                }

                // Device 1: PCAL95555 GPIO expander @ 0x20
                {
                    hf_i2c_device_config_t dev = {};
                    dev.device_address   = 0x20;
                    dev.dev_addr_length  = hf_i2c_address_bits_t::HF_I2C_ADDR_7_BIT;
                    dev.scl_speed_hz     = 400000;
                    dev.disable_ack_check = false;

                    int idx = i2c_bus_->CreateDevice(dev);
                    i2c_builtin_indices_[static_cast<uint8_t>(I2cDeviceId::PCAL9555_GPIO_EXPANDER)] = idx;
                    Logger::GetInstance().Info(TAG, "PCAL95555 I2C device added (addr=0x20, idx=%d)", idx);
                }
            } else {
                Logger::GetInstance().Error(TAG, "I2C bus initialization failed");
                i2c_bus_.reset();
            }
        } else {
            Logger::GetInstance().Error(TAG, "I2C pin mapping missing");
        }
    }

    //==========================================================================
    // CAN (TWAI) BUS — 500 kbps
    //==========================================================================
    {
        auto* tx_map = GetGpioMapping(HfFunctionalGpioPin::TWAI_TX);
        auto* rx_map = GetGpioMapping(HfFunctionalGpioPin::TWAI_RX);

        if (tx_map && rx_map) {
            hf_esp_can_config_t can_cfg = {};
            can_cfg.controller_id = hf_can_controller_id_t::HF_CAN_CONTROLLER_0;
            can_cfg.tx_pin    = tx_map->physical_pin;
            can_cfg.rx_pin    = rx_map->physical_pin;
            can_cfg.baud_rate = 500000;

            can_bus_ = std::make_unique<EspCan>(can_cfg);
            if (can_bus_ && can_bus_->EnsureInitialized()) {
                can_bus_valid_ = true;
                Logger::GetInstance().Info(TAG, "CAN bus configured (TX=GPIO%d, RX=GPIO%d, 500kbps)",
                                          tx_map->physical_pin, rx_map->physical_pin);
            } else {
                Logger::GetInstance().Warn(TAG, "CAN bus init failed — non-critical");
                can_bus_.reset();
            }
        }
    }

    //==========================================================================
    // RESULT
    //==========================================================================

    Logger::GetInstance().Info(TAG, "Vortex V1 comm channels initialized (UART=%s, SPI=%s, I2C=%s, CAN=%s)",
                              uart_bus_valid_ ? "OK" : "FAIL",
                              spi_bus_valid_  ? "OK" : "FAIL",
                              i2c_bus_valid_  ? "OK" : "FAIL",
                              can_bus_valid_  ? "OK" : "FAIL");

    // At least SPI or I2C must succeed (hard requirements for motor + sensor)
    return spi_bus_valid_ || i2c_bus_valid_;
}

//==============================================================================
// SPI ACCESSORS
//==============================================================================

BaseSpi* CommChannelsManager::GetSpiDevice(SpiDeviceId device_id) noexcept {
    return GetSpiDevice(static_cast<uint8_t>(device_id));
}

BaseSpi* CommChannelsManager::GetSpiDevice(uint8_t device_index) noexcept {
    if (!initialized_.load(std::memory_order_acquire) || !spi_bus_valid_) return nullptr;
    if (device_index >= kSpiDeviceCount) return nullptr;
    int idx = spi_device_indices_[device_index];
    if (idx < 0) return nullptr;
    return spi_bus_->GetDevice(idx);
}

//==============================================================================
// I2C ACCESSORS
//==============================================================================

BaseI2c* CommChannelsManager::GetI2cDevice(I2cDeviceId device_id) noexcept {
    if (!initialized_.load(std::memory_order_acquire) || !i2c_bus_valid_) return nullptr;
    uint8_t id = static_cast<uint8_t>(device_id);
    if (id >= kI2cBuiltinCount) return nullptr;
    int idx = i2c_builtin_indices_[id];
    if (idx < 0) return nullptr;
    return i2c_bus_->GetDevice(idx);
}

BaseI2c* CommChannelsManager::GetI2cDevice(uint8_t bus_index, int device_index) noexcept {
    if (bus_index != 0) return nullptr; // only Bus 0 supported
    if (!initialized_.load(std::memory_order_acquire) || !i2c_bus_valid_) return nullptr;
    if (device_index < 0) return nullptr;
    return i2c_bus_->GetDevice(device_index);
}

//==============================================================================
// RUNTIME I2C DEVICE MANAGEMENT (for ImuManager)
//==============================================================================

int CommChannelsManager::CreateI2cDevice(uint8_t device_address, uint32_t speed_hz) noexcept {
    if (!initialized_.load(std::memory_order_acquire) || !i2c_bus_valid_ || !i2c_bus_) {
        Logger::GetInstance().Error(TAG, "Cannot create I2C device: bus not available");
        return -1;
    }

    // Check if device already exists at this address
    if (HasI2cDeviceAtAddress(0, device_address)) {
        Logger::GetInstance().Warn(TAG, "I2C device already exists at address 0x%02X", device_address);
        // Return existing device index
        for (uint8_t i = 0; i < kI2cBuiltinCount; ++i) {
            // Check built-in addresses (0x4A for BNO08x, 0x20 for PCAL95555)
            // We don't store addresses in the array, so we check by identity
        }
        for (auto& slot : i2c_runtime_slots_) {
            if (slot.active && slot.address == device_address) {
                return slot.device_index;
            }
        }
        return -1; // Shouldn't reach here
    }

    // Find a free runtime slot
    RuntimeI2cSlot* free_slot = nullptr;
    for (auto& slot : i2c_runtime_slots_) {
        if (!slot.active) {
            free_slot = &slot;
            break;
        }
    }
    if (!free_slot) {
        Logger::GetInstance().Error(TAG, "No free I2C runtime slots (max %d)", kMaxI2cRuntimeDevices);
        return -1;
    }

    // Create device on the I2C bus
    hf_i2c_device_config_t dev_cfg = {};
    dev_cfg.device_address   = device_address;
    dev_cfg.dev_addr_length  = hf_i2c_address_bits_t::HF_I2C_ADDR_7_BIT;
    dev_cfg.scl_speed_hz     = speed_hz;
    dev_cfg.disable_ack_check = false;

    int device_index = i2c_bus_->CreateDevice(dev_cfg);
    if (device_index < 0) {
        Logger::GetInstance().Error(TAG, "Failed to create I2C device at 0x%02X", device_address);
        return -1;
    }

    // Track in runtime slot
    free_slot->device_index = device_index;
    free_slot->address      = device_address;
    free_slot->active       = true;

    Logger::GetInstance().Info(TAG, "Created runtime I2C device (addr=0x%02X, idx=%d)",
                              device_address, device_index);
    return device_index;
}

bool CommChannelsManager::HasI2cDeviceAtAddress(uint8_t bus_index, uint8_t device_address) const noexcept {
    if (bus_index != 0) return false;

    // Check built-in devices
    static constexpr uint8_t kBuiltinAddresses[] = {0x4A, 0x20}; // BNO08x, PCAL95555
    for (auto addr : kBuiltinAddresses) {
        if (addr == device_address) return true;
    }

    // Check runtime devices
    for (const auto& slot : i2c_runtime_slots_) {
        if (slot.active && slot.address == device_address) return true;
    }

    return false;
}

bool CommChannelsManager::RemoveI2cDevice(int device_index) noexcept {
    if (device_index < 0) return false;

    // Only allow removal of runtime-created devices
    for (auto& slot : i2c_runtime_slots_) {
        if (slot.active && slot.device_index == device_index) {
            // Remove from bus
            if (i2c_bus_) {
                i2c_bus_->RemoveDevice(device_index);
            }
            Logger::GetInstance().Info(TAG, "Removed runtime I2C device (addr=0x%02X, idx=%d)",
                                      slot.address, device_index);
            slot = RuntimeI2cSlot{}; // Reset to default
            return true;
        }
    }

    // Reject removal of built-in devices
    for (uint8_t i = 0; i < kI2cBuiltinCount; ++i) {
        if (i2c_builtin_indices_[i] == device_index) {
            Logger::GetInstance().Error(TAG, "Cannot remove built-in I2C device %d", device_index);
            return false;
        }
    }

    Logger::GetInstance().Error(TAG, "I2C device index %d not found", device_index);
    return false;
}

//==============================================================================
// UART / CAN ACCESSORS
//==============================================================================

BaseUart* CommChannelsManager::GetUartBus() noexcept {
    if (!initialized_.load(std::memory_order_acquire) || !uart_bus_valid_) return nullptr;
    return uart_bus_.get();
}

BaseCan* CommChannelsManager::GetCanBus() noexcept {
    if (!initialized_.load(std::memory_order_acquire) || !can_bus_valid_) return nullptr;
    return can_bus_.get();
}

//==============================================================================
// DIAGNOSTICS
//==============================================================================

void CommChannelsManager::DumpStatistics() const noexcept {
    auto& log = Logger::GetInstance();
    log.Info(TAG, "=== Vortex CommChannels Statistics ===");
    log.Info(TAG, "  UART bus: %s", uart_bus_valid_ ? "OK" : "NOT CONFIGURED");
    log.Info(TAG, "  SPI bus:  %s", spi_bus_valid_  ? "OK" : "NOT CONFIGURED");
    for (uint8_t i = 0; i < kSpiDeviceCount; ++i) {
        log.Info(TAG, "    SPI device %d: idx=%d", i, spi_device_indices_[i]);
    }
    log.Info(TAG, "  I2C bus:  %s", i2c_bus_valid_  ? "OK" : "NOT CONFIGURED");
    for (uint8_t i = 0; i < kI2cBuiltinCount; ++i) {
        log.Info(TAG, "    I2C built-in %d: idx=%d", i, i2c_builtin_indices_[i]);
    }
    uint8_t runtime_count = 0;
    for (const auto& slot : i2c_runtime_slots_) {
        if (slot.active) {
            log.Info(TAG, "    I2C runtime: addr=0x%02X, idx=%d", slot.address, slot.device_index);
            ++runtime_count;
        }
    }
    log.Info(TAG, "    I2C runtime devices: %d/%d", runtime_count, kMaxI2cRuntimeDevices);
    log.Info(TAG, "  CAN bus:  %s", can_bus_valid_  ? "OK" : "NOT CONFIGURED");
    log.Info(TAG, "=== End Vortex CommChannels Statistics ===");
}

//==============================================================================
// DEINITIALIZE
//==============================================================================

bool CommChannelsManager::Deinitialize() noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return true;

    Logger::GetInstance().Info(TAG, "Deinitializing Vortex CommChannels manager");

    // Clean up runtime I2C devices first
    for (auto& slot : i2c_runtime_slots_) {
        if (slot.active && i2c_bus_) {
            i2c_bus_->RemoveDevice(slot.device_index);
        }
        slot = RuntimeI2cSlot{};
    }

    spi_bus_.reset();
    spi_bus_valid_ = false;

    i2c_bus_.reset();
    i2c_bus_valid_ = false;

    uart_bus_.reset();
    uart_bus_valid_ = false;

    can_bus_.reset();
    can_bus_valid_ = false;

    initialized_.store(false, std::memory_order_release);
    return true;
}
