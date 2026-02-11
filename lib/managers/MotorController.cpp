#include "MotorController.h"
#include "CommChannelsManager.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"
#include "handlers/Logger.h"
#include <algorithm>
#include <string>

MotorController& MotorController::GetInstance() {
    static MotorController instance;
    return instance;
}

MotorController::MotorController() 
    : onboardDeviceCreated_(false), initialized_(false), deviceMutex_() {
    // Initialize all device slots as empty and not active
    tmcHandlers_.fill(nullptr);
    deviceInitialized_.fill(false);
    deviceActive_.fill(false);
}

bool MotorController::Initialize() {
    MutexLockGuard lock(deviceMutex_);
    
    // Initialize CommChannelsManager first
    auto& commManager = CommChannelsManager::GetInstance();
    if (!commManager.EnsureInitialized()) {
        return false;
    }
    
    // Initialize all active devices
    bool allSuccess = true;
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i] && tmcHandlers_[i]) {
            deviceInitialized_[i] = tmcHandlers_[i]->Initialize();
            if (!deviceInitialized_[i]) {
                allSuccess = false;
            }
        }
    }
    
    return allSuccess;
}

// ---------------------------------------------------------------------------
// Onboard Device Creation (SPI)
// ---------------------------------------------------------------------------

bool MotorController::CreateOnboardDevice(BaseSpi& spiInterface, 
                                        uint8_t address,
                                        const Tmc9660ControlPins& pins,
                                        const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (deviceActive_[ONBOARD_TMC9660_INDEX]) {
        return false; // Onboard device already exists
    }
    
    // Create onboard TMC9660 handler with SPI and control pins
    auto onboardHandler = std::make_unique<Tmc9660Handler>(
        spiInterface, 
        pins.rst, pins.drv_en, pins.faultn, pins.wake,
        address, 
        bootCfg ? bootCfg : &Tmc9660Handler::kDefaultBootConfig
    );
    
    tmcHandlers_[ONBOARD_TMC9660_INDEX] = std::move(onboardHandler);
    deviceActive_[ONBOARD_TMC9660_INDEX] = true;
    deviceInitialized_[ONBOARD_TMC9660_INDEX] = false;
    onboardDeviceCreated_ = true;
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        deviceInitialized_[ONBOARD_TMC9660_INDEX] = tmcHandlers_[ONBOARD_TMC9660_INDEX]->Initialize();
    }
    
    return true;
}

// ---------------------------------------------------------------------------
// Onboard Device Creation (UART)
// ---------------------------------------------------------------------------

bool MotorController::CreateOnboardDevice(BaseUart& uartInterface,
                                        uint8_t address,
                                        const Tmc9660ControlPins& pins,
                                        const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (deviceActive_[ONBOARD_TMC9660_INDEX]) {
        return false; // Onboard device already exists
    }
    
    // Create onboard TMC9660 handler with UART and control pins
    auto onboardHandler = std::make_unique<Tmc9660Handler>(
        uartInterface, 
        pins.rst, pins.drv_en, pins.faultn, pins.wake,
        address, 
        bootCfg ? bootCfg : &Tmc9660Handler::kDefaultBootConfig
    );
    
    tmcHandlers_[ONBOARD_TMC9660_INDEX] = std::move(onboardHandler);
    deviceActive_[ONBOARD_TMC9660_INDEX] = true;
    deviceInitialized_[ONBOARD_TMC9660_INDEX] = false;
    onboardDeviceCreated_ = true;
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        deviceInitialized_[ONBOARD_TMC9660_INDEX] = tmcHandlers_[ONBOARD_TMC9660_INDEX]->Initialize();
    }
    
    return true;
}

// ---------------------------------------------------------------------------
// External Device Creation (SPI via CommChannelsManager)
// ---------------------------------------------------------------------------

bool MotorController::CreateExternalDevice(uint8_t csDeviceIndex, 
                                         SpiDeviceId spiDeviceId, 
                                         uint8_t address,
                                         const Tmc9660ControlPins& pins,
                                         const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        return false; // Invalid device index for external device
    }
    
    if (deviceActive_[csDeviceIndex]) {
        return false; // Device slot already occupied
    }
    
    // Get SPI interface from CommChannelsManager
    auto& commManager = CommChannelsManager::GetInstance();
    if (!commManager.EnsureInitialized()) {
        return false; // CommChannelsManager not initialized
    }
    
    BaseSpi* spiInterface = commManager.GetSpiDevice(spiDeviceId);
    if (!spiInterface) {
        return false; // Invalid SPI device ID or interface not available
    }
    
    // Create external TMC9660 handler with SPI and control pins
    auto externalHandler = std::make_unique<Tmc9660Handler>(
        *spiInterface, 
        pins.rst, pins.drv_en, pins.faultn, pins.wake,
        address, 
        bootCfg ? bootCfg : &Tmc9660Handler::kDefaultBootConfig
    );
    
    tmcHandlers_[csDeviceIndex] = std::move(externalHandler);
    deviceActive_[csDeviceIndex] = true;
    deviceInitialized_[csDeviceIndex] = false;
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        deviceInitialized_[csDeviceIndex] = tmcHandlers_[csDeviceIndex]->Initialize();
    }
    
    return true;
}

// ---------------------------------------------------------------------------
// External Device Creation (UART)
// ---------------------------------------------------------------------------

bool MotorController::CreateExternalDevice(uint8_t csDeviceIndex,
                                         BaseUart& uartInterface,
                                         uint8_t address,
                                         const Tmc9660ControlPins& pins,
                                         const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        return false; // Invalid device index for external device
    }
    
    if (deviceActive_[csDeviceIndex]) {
        return false; // Device slot already occupied
    }
    
    // Create external TMC9660 handler with UART and control pins
    auto externalHandler = std::make_unique<Tmc9660Handler>(
        uartInterface, 
        pins.rst, pins.drv_en, pins.faultn, pins.wake,
        address, 
        bootCfg ? bootCfg : &Tmc9660Handler::kDefaultBootConfig
    );
    
    tmcHandlers_[csDeviceIndex] = std::move(externalHandler);
    deviceActive_[csDeviceIndex] = true;
    deviceInitialized_[csDeviceIndex] = false;
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        deviceInitialized_[csDeviceIndex] = tmcHandlers_[csDeviceIndex]->Initialize();
    }
    
    return true;
}

// ---------------------------------------------------------------------------
// Device Deletion
// ---------------------------------------------------------------------------

bool MotorController::DeleteExternalDevice(uint8_t csDeviceIndex) {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        return false; // Cannot delete onboard device or invalid index
    }
    
    if (!deviceActive_[csDeviceIndex]) {
        return false; // Device doesn't exist
    }
    
    // Reset device slot
    tmcHandlers_[csDeviceIndex].reset();
    deviceActive_[csDeviceIndex] = false;
    deviceInitialized_[csDeviceIndex] = false;
    
    return true;
}

// ---------------------------------------------------------------------------
// Utility & Query Methods
// ---------------------------------------------------------------------------

bool MotorController::IsExternalDeviceIndex(uint8_t csDeviceIndex) const noexcept {
    return (csDeviceIndex == EXTERNAL_DEVICE_1_INDEX || 
            csDeviceIndex == EXTERNAL_DEVICE_2_INDEX);
}

uint8_t MotorController::GetDeviceCount() const noexcept {
    MutexLockGuard lock(deviceMutex_);
    
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i]) {
            count++;
        }
    }
    return count;
}

bool MotorController::IsDeviceValid(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(deviceMutex_);
    return (deviceIndex < MAX_TMC9660_DEVICES) && deviceActive_[deviceIndex];
}

bool MotorController::IsExternalSlotAvailable(uint8_t csDeviceIndex) const noexcept {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        return false;
    }
    
    return !deviceActive_[csDeviceIndex];
}

std::vector<uint8_t> MotorController::GetActiveDeviceIndices() const noexcept {
    MutexLockGuard lock(deviceMutex_);
    
    std::vector<uint8_t> activeIndices;
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i]) {
            activeIndices.push_back(i);
        }
    }
    return activeIndices;
}

// ---------------------------------------------------------------------------
// Handler Access
// ---------------------------------------------------------------------------

Tmc9660Handler* MotorController::handler(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(deviceMutex_);
    
    if (deviceIndex >= MAX_TMC9660_DEVICES || !deviceActive_[deviceIndex] || !tmcHandlers_[deviceIndex]) {
        return nullptr; // Invalid or inactive device
    }
    
    return tmcHandlers_[deviceIndex].get();
}

// ---------------------------------------------------------------------------
// Device Initialization
// ---------------------------------------------------------------------------

std::vector<bool> MotorController::InitializeAllDevices() {
    MutexLockGuard lock(deviceMutex_);
    
    std::vector<bool> results;
    
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i] && tmcHandlers_[i]) {
            deviceInitialized_[i] = tmcHandlers_[i]->Initialize();
            results.push_back(deviceInitialized_[i]);
        }
    }
    
    return results;
}

std::vector<bool> MotorController::GetInitializationStatus() const {
    MutexLockGuard lock(deviceMutex_);
    
    std::vector<bool> status;
    
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i]) {
            status.push_back(deviceInitialized_[i]);
        }
    }
    
    return status;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

void MotorController::DumpStatistics() const noexcept {
    static constexpr const char* TAG = "MotorController";
    
    Logger::GetInstance().Info(TAG, "=== MOTOR CONTROLLER STATISTICS ===");
    
    MutexLockGuard lock(deviceMutex_);
    
    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", initialized_ ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Onboard Device Created: %s", onboardDeviceCreated_ ? "YES" : "NO");
    
    // Device Statistics
    int active_devices = 0;
    int initialized_devices = 0;
    
    Logger::GetInstance().Info(TAG, "Device Status Summary:");
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i] && tmcHandlers_[i] != nullptr) {
            bool is_initialized = deviceInitialized_[i];
            const char* device_type;
            
            switch (i) {
                case ONBOARD_TMC9660_INDEX:
                    device_type = "Onboard TMC9660";
                    break;
                case EXTERNAL_DEVICE_1_INDEX:
                    device_type = "External Device 1";
                    break;
                case EXTERNAL_DEVICE_2_INDEX:
                    device_type = "External Device 2";
                    break;
                default:
                    device_type = "Unknown Device";
                    break;
            }
            
            Logger::GetInstance().Info(TAG, "  Device %d (%s): ACTIVE, %s", 
                i, device_type,
                is_initialized ? "INITIALIZED" : "NOT_INITIALIZED");
            
            active_devices++;
            if (is_initialized) initialized_devices++;
            
            // Dump handler-level diagnostics
            if (is_initialized) {
                tmcHandlers_[i]->DumpDiagnostics();
            }
        } else if (i == ONBOARD_TMC9660_INDEX) {
            Logger::GetInstance().Info(TAG, "  Device %d (Onboard TMC9660): NOT_ACTIVE", i);
        } else if (i == EXTERNAL_DEVICE_1_INDEX || i == EXTERNAL_DEVICE_2_INDEX) {
            Logger::GetInstance().Info(TAG, "  Device %d (External Device %d): NOT_CREATED", 
                i, i == EXTERNAL_DEVICE_1_INDEX ? 1 : 2);
        }
    }
    
    // Overall Statistics
    Logger::GetInstance().Info(TAG, "Overall Statistics:");
    Logger::GetInstance().Info(TAG, "  Active Devices: %d", active_devices);
    Logger::GetInstance().Info(TAG, "  Initialized Devices: %d", initialized_devices);
    Logger::GetInstance().Info(TAG, "  Max Possible Devices: %d", MAX_TMC9660_DEVICES);
    
    // Device Index Information
    auto active_indices = GetActiveDeviceIndices();
    if (!active_indices.empty()) {
        Logger::GetInstance().Info(TAG, "  Active Device Indices: ");
        for (size_t i = 0; i < active_indices.size(); ++i) {
            Logger::GetInstance().Info(TAG, "    [%d]", active_indices[i]);
        }
    }
    
    // Memory Usage
    Logger::GetInstance().Info(TAG, "Memory Usage:");
    size_t handler_memory = 0;
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (tmcHandlers_[i] != nullptr) {
            handler_memory += sizeof(Tmc9660Handler);
        }
    }
    Logger::GetInstance().Info(TAG, "  Handler Memory: %d bytes", static_cast<int>(handler_memory));
    
    // Hardware Configuration
    Logger::GetInstance().Info(TAG, "Hardware Configuration:");
    Logger::GetInstance().Info(TAG, "  Communication: SPI/UART via CommChannelsManager");
    Logger::GetInstance().Info(TAG, "  Onboard CS: TMC9660_MOTOR_CONTROLLER");
    Logger::GetInstance().Info(TAG, "  External CS 1: EXTERNAL_DEVICE_1");
    Logger::GetInstance().Info(TAG, "  External CS 2: EXTERNAL_DEVICE_2");
    
    // System Status
    bool system_healthy = initialized_ && (active_devices > 0) && (initialized_devices == active_devices);
    Logger::GetInstance().Info(TAG, "System Status: %s", 
        system_healthy ? "HEALTHY" : "DEGRADED");
    
    Logger::GetInstance().Info(TAG, "=== END MOTOR CONTROLLER STATISTICS ===");
}
