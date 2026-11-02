/**
 * @file CommChannelsManager.cpp
 * @brief Implementation of the Vortex V1 communication channels manager.
 *
 * @details Configures four communication buses for the Vortex board through the board
 *          binding (bsp_bind/VortexBoardBind.h); no MCU type appears here:
 *          - UART : TMC9660 TMCL protocol (115200 8N1)
 *          - SPI  : TMC9660 (Mode 3) + AS5047U (Mode 1) + 2 ext (Mode 0)
 *          - I2C  : BNO08x IMU (0x4A) + PCAL95555 (0x20) @ 100 kHz; BNO device uses extended
 *                   scl_wait_us for SH-2 clock stretching (see add_builtin_device)
 *          - CAN  : TWAI 500 kbps
 *
 * @version 2.1
 */

#include "CommChannelsManager.h"

#include "bsp_bind/VortexBoardBind.h"
#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"
#include "handlers/logger/Logger.h"

#include <algorithm>

static constexpr const char* TAG = "VortexComm";
/// TWAI nominal bit rate. Step 7 of the CANopen plan reads it from the LSS record instead.
static constexpr uint32_t kCanBitRate = 500000U;

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

    //--------------------------------------------------------------------------
    // 1. UART — TMC9660 TMCL (115200 8N1, port 1)
    //--------------------------------------------------------------------------
    {
        auto* tx_map = GetGpioMapping(HfFunctionalGpioPin::UART_TXD);
        auto* rx_map = GetGpioMapping(HfFunctionalGpioPin::UART_RXD);
        if (tx_map && rx_map) {
            vortex::bsp::UartSpec spec{};
            spec.port = 1;
            spec.baud = 115200;
            spec.tx_pin = tx_map->physical_pin;
            spec.rx_pin = rx_map->physical_pin;
            uart_bus_ = vortex::bsp::UartOpen(spec);
            uart_bus_valid_ = (uart_bus_ != nullptr);
            if (uart_bus_valid_) {
                Logger::GetInstance().Info(TAG, "UART configured: 115200 8N1, TX=GPIO%d, RX=GPIO%d",
                                          tx_map->physical_pin, rx_map->physical_pin);
            } else {
                Logger::GetInstance().Error(TAG, "UART initialization failed");
            }
        } else {
            Logger::GetInstance().Error(TAG, "UART pin mapping missing");
        }
    }

    //--------------------------------------------------------------------------
    // 2. SPI2 bus — TMC9660, AS5047U and the two external chip selects
    //--------------------------------------------------------------------------
    {
        auto* miso_map = GetGpioMapping(HfFunctionalGpioPin::SPI2_MISO);
        auto* mosi_map = GetGpioMapping(HfFunctionalGpioPin::SPI2_MOSI);
        auto* sck_map  = GetGpioMapping(HfFunctionalGpioPin::SPI2_SCK);
        if (miso_map && mosi_map && sck_map) {
            vortex::bsp::SpiBusSpec bus{};
            bus.host = 1;  // SPI2
            bus.miso_pin = miso_map->physical_pin;
            bus.mosi_pin = mosi_map->physical_pin;
            bus.sclk_pin = sck_map->physical_pin;
            bus.timeout_ms = 1000;
            if (vortex::bsp::SpiOpen(bus)) {
                spi_bus_valid_ = true;
                for (uint8_t i = 0; i < kSpiDeviceCount; ++i) {
                    auto* cs_map = GetGpioMapping(kSpiCsPins[i]);
                    if (!cs_map) continue;
                    vortex::bsp::SpiDeviceSpec dev{};
                    dev.cs_pin = cs_map->physical_pin;
                    dev.clock_hz = 10'000'000;  // 10 MHz default
                    dev.mode = (i == static_cast<uint8_t>(SpiDeviceId::TMC9660_MOTOR_CONTROLLER)) ? 3
                             : (i == static_cast<uint8_t>(SpiDeviceId::AS5047U_POSITION_ENCODER)) ? 1
                             : 0;
                    const int idx = vortex::bsp::SpiAddDevice(dev);
                    spi_device_indices_[i] = idx;
                    Logger::GetInstance().Info(TAG, "SPI device %d added (CS=GPIO%d, mode=%d, idx=%d)",
                                              i, cs_map->physical_pin, dev.mode, idx);
                }
            } else {
                Logger::GetInstance().Error(TAG, "SPI bus initialization failed");
            }
        } else {
            Logger::GetInstance().Error(TAG, "SPI pin mapping missing");
        }
    }

    //--------------------------------------------------------------------------
    // 3. I2C bus — BNO08x IMU + PCAL95555 expander
    //--------------------------------------------------------------------------
    {
        auto* sda_map = GetGpioMapping(HfFunctionalGpioPin::I2C_SDA);
        auto* scl_map = GetGpioMapping(HfFunctionalGpioPin::I2C_SCL);
        if (sda_map && scl_map) {
            vortex::bsp::I2cBusSpec bus{};
            bus.port = 0;
            bus.sda_pin = sda_map->physical_pin;
            bus.scl_pin = scl_map->physical_pin;
            bus.internal_pullups = sda_map->has_pull && scl_map->has_pull;
            if (vortex::bsp::I2cOpen(bus)) {
                i2c_bus_valid_ = true;
                Logger::GetInstance().Info(TAG, "I2C bus configured (SDA=GPIO%d, SCL=GPIO%d, sync mode)",
                                          sda_map->physical_pin, scl_map->physical_pin);

                // 100 kHz is safer for PCAL95555 / BNO08x on long traces and weak pull-ups.
                constexpr uint32_t kBuiltinSclHz = 100000;
                auto add_builtin_device = [&](I2cDeviceId id, uint8_t addr,
                                              const char* name) noexcept {
                    vortex::bsp::I2cDeviceSpec dev{};
                    dev.address = addr;
                    dev.scl_hz = kBuiltinSclHz;
                    // BNO08x SH-2 clock-stretches well past the default (see hf-bno08x-driver ESP32 examples).
                    dev.scl_wait_us = (id == I2cDeviceId::BNO08X_IMU) ? 50000u : 0u;
                    const int idx = vortex::bsp::I2cAddDevice(dev);
                    i2c_builtin_indices_[static_cast<uint8_t>(id)] = idx;
                    if (idx < 0) {
                        Logger::GetInstance().Error(TAG,
                                                    "%s I2C device add FAILED (addr=0x%02X)",
                                                    name, addr);
                        return;
                    }
                    BaseI2c* dev_iface = vortex::bsp::I2cDevice(idx);
                    const bool inited = dev_iface && dev_iface->EnsureInitialized();
                    Logger::GetInstance().Info(
                        TAG,
                        "%s I2C device added (addr=0x%02X, idx=%d, scl=%lu Hz, scl_wait_us=%lu, "
                        "ready=%s)",
                        name, addr, idx, static_cast<unsigned long>(kBuiltinSclHz),
                        static_cast<unsigned long>(dev.scl_wait_us), inited ? "yes" : "no");
                };

                add_builtin_device(I2cDeviceId::BNO08X_IMU, 0x4A, "BNO08x IMU");
                add_builtin_device(I2cDeviceId::PCAL9555_GPIO_EXPANDER, 0x20,
                                   "PCAL95555");
            } else {
                Logger::GetInstance().Error(TAG, "I2C bus initialization failed");
            }
        } else {
            Logger::GetInstance().Error(TAG, "I2C pin mapping missing");
        }
    }

    //--------------------------------------------------------------------------
    // 4. CAN — TWAI 500 kbps (non-critical)
    //--------------------------------------------------------------------------
    {
        auto* tx_map = GetGpioMapping(HfFunctionalGpioPin::TWAI_TX);
        auto* rx_map = GetGpioMapping(HfFunctionalGpioPin::TWAI_RX);
        if (tx_map && rx_map) {
            vortex::bsp::CanSpec spec{};
            spec.tx_pin = tx_map->physical_pin;
            spec.rx_pin = rx_map->physical_pin;
            spec.bitrate = kCanBitRate;
            can_bus_ = vortex::bsp::CanOpen(spec);
            can_bus_valid_ = (can_bus_ != nullptr);
            if (can_bus_valid_) {
                Logger::GetInstance().Info(TAG, "CAN bus configured (TX=GPIO%d, RX=GPIO%d, 500kbps)",
                                          tx_map->physical_pin, rx_map->physical_pin);
            } else {
                Logger::GetInstance().Warn(TAG, "CAN bus init failed — non-critical");
            }
        }
    }

    Logger::GetInstance().Info(TAG, "Vortex V1 comm channels initialized (UART=%s, SPI=%s, I2C=%s, CAN=%s)",
                              uart_bus_valid_ ? "OK" : "FAIL",
                              spi_bus_valid_  ? "OK" : "FAIL",
                              i2c_bus_valid_  ? "OK" : "FAIL",
                              can_bus_valid_  ? "OK" : "FAIL");

    // At minimum SPI or I2C must work for the system to be useful. Otherwise fail closed:
    // close whatever did open, so a failed manager holds no bus.
    if (!spi_bus_valid_ && !i2c_bus_valid_) {
        ReleaseLocked();
        UpdateLastError(CommError::BUS_NOT_AVAILABLE);
        return false;
    }
    UpdateLastError(CommError::SUCCESS);
    return true;
}

//==============================================================================
// SPI ACCESSORS
//==============================================================================

BaseSpi* CommChannelsManager::GetSpiDevice(SpiDeviceId device_id) noexcept {
    return GetSpiDevice(static_cast<uint8_t>(device_id));
}

BaseSpi* CommChannelsManager::GetSpiDevice(uint8_t device_index) noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire) || !spi_bus_valid_) return nullptr;
    if (device_index >= kSpiDeviceCount) return nullptr;
    int idx = spi_device_indices_[device_index];
    if (idx < 0) return nullptr;
    return vortex::bsp::SpiDevice(idx);
}

//==============================================================================
// I2C ACCESSORS
//==============================================================================

BaseI2c* CommChannelsManager::GetI2cDevice(I2cDeviceId device_id) noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire) || !i2c_bus_valid_) return nullptr;
    uint8_t id = static_cast<uint8_t>(device_id);
    if (id >= kI2cBuiltinCount) return nullptr;
    int idx = i2c_builtin_indices_[id];
    if (idx < 0) return nullptr;
    return vortex::bsp::I2cDevice(idx);
}

BaseI2c* CommChannelsManager::GetI2cDevice(uint8_t bus_index, int device_index) noexcept {
    MutexLockGuard lock(mutex_);
    if (bus_index != 0) return nullptr; // only Bus 0 supported
    if (!initialized_.load(std::memory_order_acquire) || !i2c_bus_valid_) return nullptr;
    if (device_index < 0) return nullptr;
    return vortex::bsp::I2cDevice(device_index);
}

//==============================================================================
// RUNTIME I2C DEVICE MANAGEMENT (for ImuManager)
//==============================================================================

int CommChannelsManager::CreateI2cDevice(uint8_t device_address, uint32_t speed_hz) noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire) || !i2c_bus_valid_) {
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
    vortex::bsp::I2cDeviceSpec dev{};
    dev.address = device_address;
    dev.scl_hz = speed_hz;

    int device_index = vortex::bsp::I2cAddDevice(dev);
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
    MutexLockGuard lock(mutex_);
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
    MutexLockGuard lock(mutex_);
    if (device_index < 0) return false;

    // Only allow removal of runtime-created devices
    for (auto& slot : i2c_runtime_slots_) {
        if (slot.active && slot.device_index == device_index) {
            // Remove from bus
            (void)vortex::bsp::I2cRemoveDevice(device_index);
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
// I2C BUS DIAGNOSTICS
//==============================================================================

bool CommChannelsManager::I2cProbeAddress(uint8_t address, uint32_t timeout_ms) noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire) || !i2c_bus_valid_) {
        return false;
    }
    return vortex::bsp::I2cProbe(address, timeout_ms);
}

size_t CommChannelsManager::I2cScanBus(uint8_t* out, size_t max_results) noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire) || !i2c_bus_valid_) {
        return 0;
    }
    if (out == nullptr || max_results == 0) {
        return 0;
    }

    return vortex::bsp::I2cScan(out, max_results, 0x08, 0x77, /*timeout_ms*/ 25);
}

bool CommChannelsManager::I2cResetBus() noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire) || !i2c_bus_valid_) {
        return false;
    }
    return vortex::bsp::I2cReset();
}

//==============================================================================
// UART / CAN ACCESSORS
//==============================================================================

BaseUart* CommChannelsManager::GetUartBus() noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire) || !uart_bus_valid_) return nullptr;
    return uart_bus_;
}

BaseCan* CommChannelsManager::GetCanBus() noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire) || !can_bus_valid_) return nullptr;
    return can_bus_;
}

uint32_t CommChannelsManager::GetCanBitRate() const noexcept { return kCanBitRate; }

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

CommError CommChannelsManager::GetSystemDiagnostics(CommSystemDiagnostics& diagnostics) const noexcept {
    if (!initialized_.load(std::memory_order_acquire)) {
        return CommError::NOT_INITIALIZED;
    }

    MutexLockGuard lock(mutex_);

    diagnostics.system_initialized = true;
    diagnostics.spi_bus_valid = spi_bus_valid_;
    diagnostics.i2c_bus_valid = i2c_bus_valid_;
    diagnostics.uart_bus_valid = uart_bus_valid_;
    diagnostics.can_bus_valid = can_bus_valid_;
    diagnostics.last_error = last_error_.load(std::memory_order_acquire);

    // Count valid SPI devices
    diagnostics.spi_device_count = 0;
    for (uint8_t i = 0; i < kSpiDeviceCount; ++i) {
        if (spi_device_indices_[i] >= 0) ++diagnostics.spi_device_count;
    }

    // Count active runtime I2C devices
    diagnostics.i2c_runtime_device_count = 0;
    for (const auto& slot : i2c_runtime_slots_) {
        if (slot.active) ++diagnostics.i2c_runtime_device_count;
    }

    diagnostics.system_healthy = spi_bus_valid_ || i2c_bus_valid_;
    return CommError::SUCCESS;
}

//==============================================================================
// DEINITIALIZE
//==============================================================================

bool CommChannelsManager::Deinitialize() noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return true;

    Logger::GetInstance().Info(TAG, "Deinitializing Vortex CommChannels manager");

    ReleaseLocked();
    initialized_.store(false, std::memory_order_release);
    return true;
}

void CommChannelsManager::ReleaseLocked() noexcept {
    // Clean up runtime I2C devices first
    for (auto& slot : i2c_runtime_slots_) {
        if (slot.active) {
            (void)vortex::bsp::I2cRemoveDevice(slot.device_index);
        }
        slot = RuntimeI2cSlot{};
    }

    vortex::bsp::SpiClose();
    spi_device_indices_.fill(-1);
    spi_bus_valid_ = false;

    vortex::bsp::I2cClose();
    i2c_builtin_indices_.fill(-1);
    i2c_bus_valid_ = false;

    vortex::bsp::UartClose();
    uart_bus_ = nullptr;
    uart_bus_valid_ = false;

    vortex::bsp::CanClose();
    can_bus_ = nullptr;
    can_bus_valid_ = false;
}
