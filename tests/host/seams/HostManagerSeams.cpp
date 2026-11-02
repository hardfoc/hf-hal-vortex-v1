/**
 * @file HostManagerSeams.cpp
 * @brief Host stand-ins for CommChannelsManager and GpioManager, so managers that use them link.
 *
 * @details CommChannelsManager.cpp and GpioManager.cpp construct ESP32 peripherals (EspSpi, EspI2c,
 *          EspGpio, ...) and so do not build on the host yet. These definitions give the host build
 *          the few members other managers call: both managers report "not initialised" and hand out
 *          no buses, which is what a test that passes its own BaseSpi to a manager needs. They are
 *          test-only and are replaced by the host board binding once the managers bind through
 *          bsp_bind/.
 */

#include "CommChannelsManager.h"
#include "GpioManager.h"
#include "handlers/pcal95555/Pcal95555Handler.h"

// CommChannelsManager holds unique_ptrs to these ESP32 bus classes. The host never creates one,
// but its constructor and destructor need complete types.
class EspSpiBus {};
class EspI2cBus {};
class EspUart {};
class EspCan {};

CommChannelsManager::CommChannelsManager() noexcept {
    spi_device_indices_.fill(-1);
    i2c_builtin_indices_.fill(-1);
}

CommChannelsManager::~CommChannelsManager() = default;

CommChannelsManager& CommChannelsManager::GetInstance() noexcept {
    static CommChannelsManager instance;
    return instance;
}

bool CommChannelsManager::EnsureInitialized() noexcept { return false; }

BaseSpi* CommChannelsManager::GetSpiDevice(SpiDeviceId) noexcept { return nullptr; }

GpioManager& GpioManager::GetInstance() noexcept {
    static GpioManager instance;
    return instance;
}

bool GpioManager::EnsureInitialized() noexcept { return false; }
