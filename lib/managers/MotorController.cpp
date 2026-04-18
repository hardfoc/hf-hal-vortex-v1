#include "MotorController.h"
#include "CommChannelsManager.h"
#include "RtosMutex.h"
#include "handlers/logger/Logger.h"
#include <algorithm>
#include <string>

MotorController& MotorController::GetInstance() {
    static MotorController instance;
    return instance;
}

MotorController::MotorController() 
    : onboardDeviceCreated_(false), initialized_(false), last_error_(MotorError::SUCCESS), deviceMutex_() {
    // Initialize all device slots as empty and not active
    for (auto& handler : tmcHandlers_) {
        handler.reset();
    }
    deviceInitialized_.fill(false);
    deviceActive_.fill(false);
    for (auto& c : deviceErrorCounts_) c.store(0, std::memory_order_relaxed);
}

bool MotorController::Initialize() {
    MutexLockGuard lock(deviceMutex_);
    
    // Initialize CommChannelsManager first
    auto& commManager = CommChannelsManager::GetInstance();
    if (!commManager.EnsureInitialized()) {
        UpdateLastError(MotorError::DEPENDENCY_NOT_READY);
        return false;
    }
    
    // Initialize all active devices
    bool allSuccess = true;
    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        if (deviceActive_[i] && tmcHandlers_[i]) {
            deviceInitialized_[i] = tmcHandlers_[i]->Initialize();
            if (!deviceInitialized_[i]) {
                UpdateLastError(MotorError::INITIALIZATION_FAILED, i);
                allSuccess = false;
            }
        }
    }
    
    return allSuccess;
}

// ---------------------------------------------------------------------------
// Onboard Device Creation (SPI)
// ---------------------------------------------------------------------------

MotorError MotorController::CreateOnboardDevice(BaseSpi& spiInterface, 
                                        uint8_t address,
                                        const Tmc9660ControlPins& pins,
                                        const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (deviceActive_[ONBOARD_TMC9660_INDEX]) {
        UpdateLastError(MotorError::DEVICE_ALREADY_EXISTS, ONBOARD_TMC9660_INDEX);
        return MotorError::DEVICE_ALREADY_EXISTS;
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
        if (!deviceInitialized_[ONBOARD_TMC9660_INDEX]) {
            UpdateLastError(MotorError::INITIALIZATION_FAILED, ONBOARD_TMC9660_INDEX);
        }
    }
    
    return MotorError::SUCCESS;
}

// ---------------------------------------------------------------------------
// Onboard Device Creation (UART)
// ---------------------------------------------------------------------------

MotorError MotorController::CreateOnboardDevice(BaseUart& uartInterface,
                                        uint8_t address,
                                        const Tmc9660ControlPins& pins,
                                        const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (deviceActive_[ONBOARD_TMC9660_INDEX]) {
        UpdateLastError(MotorError::DEVICE_ALREADY_EXISTS, ONBOARD_TMC9660_INDEX);
        return MotorError::DEVICE_ALREADY_EXISTS;
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
        if (!deviceInitialized_[ONBOARD_TMC9660_INDEX]) {
            UpdateLastError(MotorError::INITIALIZATION_FAILED, ONBOARD_TMC9660_INDEX);
        }
    }
    
    return MotorError::SUCCESS;
}

// ---------------------------------------------------------------------------
// External Device Creation (SPI via CommChannelsManager)
// ---------------------------------------------------------------------------

MotorError MotorController::CreateExternalDevice(uint8_t csDeviceIndex, 
                                         SpiDeviceId spiDeviceId, 
                                         uint8_t address,
                                         const Tmc9660ControlPins& pins,
                                         const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        UpdateLastError(MotorError::INVALID_DEVICE_INDEX, csDeviceIndex);
        return MotorError::INVALID_DEVICE_INDEX;
    }
    
    if (deviceActive_[csDeviceIndex]) {
        UpdateLastError(MotorError::DEVICE_ALREADY_EXISTS, csDeviceIndex);
        return MotorError::DEVICE_ALREADY_EXISTS;
    }
    
    // Get SPI interface from CommChannelsManager
    auto& commManager = CommChannelsManager::GetInstance();
    if (!commManager.EnsureInitialized()) {
        UpdateLastError(MotorError::DEPENDENCY_NOT_READY, csDeviceIndex);
        return MotorError::DEPENDENCY_NOT_READY;
    }
    
    BaseSpi* spiInterface = commManager.GetSpiDevice(spiDeviceId);
    if (!spiInterface) {
        UpdateLastError(MotorError::COMMUNICATION_FAILED, csDeviceIndex);
        return MotorError::COMMUNICATION_FAILED;
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
        if (!deviceInitialized_[csDeviceIndex]) {
            UpdateLastError(MotorError::INITIALIZATION_FAILED, csDeviceIndex);
        }
    }
    
    return MotorError::SUCCESS;
}

// ---------------------------------------------------------------------------
// External Device Creation (UART)
// ---------------------------------------------------------------------------

MotorError MotorController::CreateExternalDevice(uint8_t csDeviceIndex,
                                         BaseUart& uartInterface,
                                         uint8_t address,
                                         const Tmc9660ControlPins& pins,
                                         const tmc9660::BootloaderConfig* bootCfg) {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        UpdateLastError(MotorError::INVALID_DEVICE_INDEX, csDeviceIndex);
        return MotorError::INVALID_DEVICE_INDEX;
    }
    
    if (deviceActive_[csDeviceIndex]) {
        UpdateLastError(MotorError::DEVICE_ALREADY_EXISTS, csDeviceIndex);
        return MotorError::DEVICE_ALREADY_EXISTS;
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
        if (!deviceInitialized_[csDeviceIndex]) {
            UpdateLastError(MotorError::INITIALIZATION_FAILED, csDeviceIndex);
        }
    }
    
    return MotorError::SUCCESS;
}

// ---------------------------------------------------------------------------
// Device Deletion
// ---------------------------------------------------------------------------

MotorError MotorController::DeleteExternalDevice(uint8_t csDeviceIndex) {
    MutexLockGuard lock(deviceMutex_);
    
    if (!IsExternalDeviceIndex(csDeviceIndex)) {
        UpdateLastError(MotorError::CANNOT_DELETE_ONBOARD, csDeviceIndex);
        return MotorError::CANNOT_DELETE_ONBOARD;
    }
    
    if (!deviceActive_[csDeviceIndex]) {
        UpdateLastError(MotorError::DEVICE_NOT_FOUND, csDeviceIndex);
        return MotorError::DEVICE_NOT_FOUND;
    }
    
    // Reset device slot
    tmcHandlers_[csDeviceIndex].reset();
    deviceActive_[csDeviceIndex] = false;
    deviceInitialized_[csDeviceIndex] = false;
    
    return MotorError::SUCCESS;
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
// Error Tracking
// ---------------------------------------------------------------------------

void MotorController::UpdateLastError(MotorError error, int deviceIndex) noexcept {
    last_error_.store(error, std::memory_order_release);
    if (deviceIndex >= 0 && deviceIndex < MAX_TMC9660_DEVICES && error != MotorError::SUCCESS) {
        deviceErrorCounts_[deviceIndex].fetch_add(1, std::memory_order_relaxed);
    }
}

MotorError MotorController::GetSystemDiagnostics(MotorSystemDiagnostics& diagnostics) const noexcept {
    if (!initialized_.load(std::memory_order_acquire)) {
        return MotorError::NOT_INITIALIZED;
    }

    MutexLockGuard lock(deviceMutex_);

    diagnostics.system_initialized = true;
    diagnostics.last_error = last_error_.load(std::memory_order_acquire);
    diagnostics.active_device_count = 0;
    diagnostics.initialized_device_count = 0;

    for (uint8_t i = 0; i < MAX_TMC9660_DEVICES; ++i) {
        diagnostics.devices[i].active = deviceActive_[i];
        diagnostics.devices[i].initialized = deviceInitialized_[i];
        diagnostics.devices[i].error_count = deviceErrorCounts_[i].load(std::memory_order_relaxed);
        if (deviceActive_[i]) ++diagnostics.active_device_count;
        if (deviceInitialized_[i]) ++diagnostics.initialized_device_count;
    }

    diagnostics.system_healthy = (diagnostics.active_device_count > 0) &&
                                 (diagnostics.initialized_device_count == diagnostics.active_device_count);
    return MotorError::SUCCESS;
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
