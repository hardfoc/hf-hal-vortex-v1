#include "EncoderManager.h"

// As5047u handler for unified encoder interface
#include "utils-and-drivers/driver-handlers/As5047uHandler.h"

// Communication manager for SPI access
#include "CommChannelsManager.h"

// GPIO manager for interrupt pin access
#include "GpioManager.h"

// Logger for unified logging
#include "utils-and-drivers/driver-handlers/Logger.h"

// Base interfaces
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseSpi.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseGpio.h"

// Platform mapping for functional pin definitions
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include/hf_platform_mapping.hpp"

// RtosMutex for thread safety
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"

// Standard library
#include <iostream>
#include <atomic>

// ESP-IDF for semaphores
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

//==============================================================================
// ENCODERMANAGER IMPLEMENTATION
//==============================================================================

EncoderManager& EncoderManager::GetInstance() noexcept {
    static EncoderManager instance;
    return instance;
}

EncoderManager::EncoderManager() noexcept 
    : initialized_(false), manager_mutex_(), onboard_device_created_(false) {
    // Initialize all device slots as empty and not active
    as5047u_handlers_.fill(nullptr);
    device_initialized_.fill(false);
    device_active_.fill(false);
}

EncoderManager::~EncoderManager() noexcept {
    Deinitialize();
}

bool EncoderManager::EnsureInitialized() noexcept {
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    MutexLockGuard lock(manager_mutex_);
    
    // Double-check after acquiring lock
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    return Initialize();
}

bool EncoderManager::IsInitialized() const noexcept {
    return initialized_.load(std::memory_order_acquire);
}

bool EncoderManager::Deinitialize() noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (!initialized_.load(std::memory_order_acquire)) {
        return true;
    }

    Logger::GetInstance().Info("EncoderManager", "Deinitializing Encoder Manager");

    // Reset monitoring statistics
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        measurement_counts_[i].store(0);
        communication_error_counts_[i].store(0);
    }

    // Deinitialize all AS5047U handlers
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i] && as5047u_handlers_[i]) {
            as5047u_handlers_[i]->Deinitialize();
            as5047u_handlers_[i].reset();
            device_active_[i] = false;
            device_initialized_[i] = false;
            Logger::GetInstance().Info("EncoderManager", "AS5047U handler %u deinitialized", i);
        }
    }

    initialized_.store(false, std::memory_order_release);
    comm_manager_ = nullptr;
    gpio_manager_ = nullptr;
    onboard_device_created_ = false;

    Logger::GetInstance().Info("EncoderManager", "Encoder Manager deinitialized");
    return true;
}

//**************************************************************************//
//**                  HANDLER AND DRIVER MANAGEMENT                       **//
//**************************************************************************//

As5047uHandler* EncoderManager::GetAs5047uHandler(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (deviceIndex >= MAX_ENCODER_DEVICES || !device_active_[deviceIndex] || !as5047u_handlers_[deviceIndex]) {
        return nullptr;
    }
    
    return as5047u_handlers_[deviceIndex].get();
}

std::shared_ptr<AS5047U> EncoderManager::GetAs5047uDriver(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (deviceIndex >= MAX_ENCODER_DEVICES || !device_active_[deviceIndex] || !as5047u_handlers_[deviceIndex]) {
        return nullptr;
    }
    
    if (!device_initialized_[deviceIndex]) {
        return nullptr;
    }
    
    return as5047u_handlers_[deviceIndex]->GetSensor();
}

//**************************************************************************//
//**                  DEVICES MANAGEMENT METHODS                           **//
//**************************************************************************//

bool EncoderManager::CreateExternalAs5047uDevice(uint8_t deviceIndex, 
                                               SpiDeviceId spiDeviceId,
                                               const As5047uConfig& config) {
    MutexLockGuard lock(manager_mutex_);
    
    if (!IsExternalDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("EncoderManager", "Invalid device index %u for external device", deviceIndex);
        return false;
    }
    
    if (device_active_[deviceIndex]) {
        Logger::GetInstance().Error("EncoderManager", "Device slot %u already occupied", deviceIndex);
        return false;
    }
    
    // Get CommChannelsManager instance
    if (!comm_manager_ || !comm_manager_->IsInitialized()) {
        Logger::GetInstance().Error("EncoderManager", "CommChannelsManager not initialized");
        return false;
    }
    
    // Get SPI interface
    BaseSpi* spi_interface = comm_manager_->GetSpiDevice(spiDeviceId);
    if (!spi_interface) {
        Logger::GetInstance().Error("EncoderManager", "Failed to get SPI device for ID %d", static_cast<int>(spiDeviceId));
        return false;
    }
    
    // Create AS5047U handler with SPI interface
    auto handler = std::make_unique<As5047uHandler>(*spi_interface, config);
    if (!handler) {
        Logger::GetInstance().Error("EncoderManager", "Failed to create AS5047U handler for device %u", deviceIndex);
        return false;
    }
    
    // Store handler and mark as active
    as5047u_handlers_[deviceIndex] = std::move(handler);
    device_active_[deviceIndex] = true;
    device_initialized_[deviceIndex] = false;
    
    // Initialize if manager is already initialized
    if (initialized_.load(std::memory_order_acquire)) {
        As5047uError init_result = as5047u_handlers_[deviceIndex]->Initialize();
        device_initialized_[deviceIndex] = (init_result == As5047uError::SUCCESS);
        
        if (device_initialized_[deviceIndex]) {
            Logger::GetInstance().Info("EncoderManager", "External AS5047U device %u created and initialized successfully", deviceIndex);
        } else {
            Logger::GetInstance().Error("EncoderManager", "External AS5047U device %u created but initialization failed: %s", 
                                      deviceIndex, As5047uErrorToString(init_result));
        }
    } else {
        Logger::GetInstance().Info("EncoderManager", "External AS5047U device %u created (will be initialized later)", deviceIndex);
    }
    
    return true;
}

bool EncoderManager::CreateExternalAs5047uDevice(uint8_t deviceIndex, 
                                               BaseSpi& spi_interface,
                                               const As5047uConfig& config) {
    MutexLockGuard lock(manager_mutex_);
    
    if (!IsExternalDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("EncoderManager", "Invalid device index %u for external device", deviceIndex);
        return false;
    }
    
    if (device_active_[deviceIndex]) {
        Logger::GetInstance().Error("EncoderManager", "Device slot %u already occupied", deviceIndex);
        return false;
    }
    
    // Create AS5047U handler with direct SPI interface
    auto handler = std::make_unique<As5047uHandler>(spi_interface, config);
    if (!handler) {
        Logger::GetInstance().Error("EncoderManager", "Failed to create AS5047U handler for device %u", deviceIndex);
        return false;
    }
    
    // Store handler and mark as active
    as5047u_handlers_[deviceIndex] = std::move(handler);
    device_active_[deviceIndex] = true;
    device_initialized_[deviceIndex] = false;
    
    // Initialize if manager is already initialized
    if (initialized_.load(std::memory_order_acquire)) {
        As5047uError init_result = as5047u_handlers_[deviceIndex]->Initialize();
        device_initialized_[deviceIndex] = (init_result == As5047uError::SUCCESS);
        
        if (device_initialized_[deviceIndex]) {
            Logger::GetInstance().Info("EncoderManager", "External AS5047U device %u (direct SPI) created and initialized successfully", deviceIndex);
        } else {
            Logger::GetInstance().Error("EncoderManager", "External AS5047U device %u (direct SPI) created but initialization failed: %s", 
                                      deviceIndex, As5047uErrorToString(init_result));
        }
    } else {
        Logger::GetInstance().Info("EncoderManager", "External AS5047U device %u (direct SPI) created (will be initialized later)", deviceIndex);
    }
    
    return true;
}

bool EncoderManager::DeleteExternalDevice(uint8_t deviceIndex) {
    MutexLockGuard lock(manager_mutex_);
    
    if (!IsExternalDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("EncoderManager", "Cannot delete onboard device or invalid index %u", deviceIndex);
        return false;
    }
    
    if (!device_active_[deviceIndex]) {
        Logger::GetInstance().Warning("EncoderManager", "Device slot %u is already empty", deviceIndex);
        return true; // Consider this success
    }
    
    // Deinitialize and clean up handler
    if (as5047u_handlers_[deviceIndex]) {
        as5047u_handlers_[deviceIndex]->Deinitialize();
        as5047u_handlers_[deviceIndex].reset();
    }
    
    device_active_[deviceIndex] = false;
    device_initialized_[deviceIndex] = false;
    
    Logger::GetInstance().Info("EncoderManager", "External AS5047U device %u deleted successfully", deviceIndex);
    return true;
}

uint8_t EncoderManager::GetDeviceCount() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i]) {
            count++;
        }
    }
    return count;
}

bool EncoderManager::IsDeviceValid(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    return (deviceIndex < MAX_ENCODER_DEVICES) && device_active_[deviceIndex];
}

bool EncoderManager::IsExternalSlotAvailable(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (!IsExternalDeviceIndex(deviceIndex)) {
        return false;
    }
    
    return !device_active_[deviceIndex];
}

std::vector<uint8_t> EncoderManager::GetActiveDeviceIndices() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    std::vector<uint8_t> active_indices;
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i]) {
            active_indices.push_back(i);
        }
    }
    return active_indices;
}

std::vector<bool> EncoderManager::InitializeAllDevices() {
    MutexLockGuard lock(manager_mutex_);
    
    std::vector<bool> results;
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i] && as5047u_handlers_[i]) {
            As5047uError init_result = as5047u_handlers_[i]->Initialize();
            device_initialized_[i] = (init_result == As5047uError::SUCCESS);
            results.push_back(device_initialized_[i]);
            
            if (device_initialized_[i]) {
                Logger::GetInstance().Info("EncoderManager", "AS5047U device %u initialized successfully", i);
            } else {
                Logger::GetInstance().Error("EncoderManager", "AS5047U device %u initialization failed: %s", 
                                          i, As5047uErrorToString(init_result));
            }
        }
    }
    
    return results;
}

std::vector<bool> EncoderManager::GetInitializationStatus() const {
    MutexLockGuard lock(manager_mutex_);
    
    std::vector<bool> status;
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i]) {
            status.push_back(device_initialized_[i]);
        }
    }
    return status;
}

//**************************************************************************//
//**                  DEVICE INFORMATION METHODS                          **//
//**************************************************************************//

std::vector<std::string> EncoderManager::GetAvailableDevices() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    std::vector<std::string> devices;
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i]) {
            std::string device_name = "AS5047U-" + std::to_string(i);
            if (i == ONBOARD_ENCODER_INDEX) {
                device_name += " (Onboard)";
            } else {
                device_name += " (External)";
            }
            devices.push_back(device_name);
        }
    }
    return devices;
}

std::string EncoderManager::GetDeviceType(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (deviceIndex >= MAX_ENCODER_DEVICES || !device_active_[deviceIndex]) {
        return "Unknown";
    }
    
    return "AS5047U";
}

//**************************************************************************//
//**                  HIGH-LEVEL ENCODER OPERATIONS                       **//
//**************************************************************************//

As5047uError EncoderManager::ReadAngle(uint8_t deviceIndex, uint16_t& angle) noexcept {
    As5047uHandler* handler = GetAs5047uHandler(deviceIndex);
    if (!handler) {
        return As5047uError::NOT_INITIALIZED;
    }
    
    As5047uError result = handler->ReadAngle(angle);
    if (result == As5047uError::SUCCESS) {
        measurement_counts_[deviceIndex].fetch_add(1, std::memory_order_acq_rel);
    } else {
        communication_error_counts_[deviceIndex].fetch_add(1, std::memory_order_acq_rel);
    }
    
    return result;
}

As5047uError EncoderManager::ReadAngleDegrees(uint8_t deviceIndex, double& angle_degrees) noexcept {
    uint16_t angle_lsb;
    As5047uError result = ReadAngle(deviceIndex, angle_lsb);
    if (result == As5047uError::SUCCESS) {
        angle_degrees = As5047uHandler::LSBToDegrees(angle_lsb);
    }
    return result;
}

As5047uError EncoderManager::ReadVelocityRPM(uint8_t deviceIndex, double& velocity_rpm) noexcept {
    As5047uHandler* handler = GetAs5047uHandler(deviceIndex);
    if (!handler) {
        return As5047uError::NOT_INITIALIZED;
    }
    
    As5047uError result = handler->ReadVelocityRPM(velocity_rpm);
    if (result == As5047uError::SUCCESS) {
        measurement_counts_[deviceIndex].fetch_add(1, std::memory_order_acq_rel);
    } else {
        communication_error_counts_[deviceIndex].fetch_add(1, std::memory_order_acq_rel);
    }
    
    return result;
}

As5047uError EncoderManager::ReadDiagnostics(uint8_t deviceIndex, As5047uDiagnostics& diagnostics) noexcept {
    As5047uHandler* handler = GetAs5047uHandler(deviceIndex);
    if (!handler) {
        return As5047uError::NOT_INITIALIZED;
    }
    
    return handler->ReadDiagnostics(diagnostics);
}

As5047uError EncoderManager::SetZeroPosition(uint8_t deviceIndex, uint16_t zero_position) noexcept {
    As5047uHandler* handler = GetAs5047uHandler(deviceIndex);
    if (!handler) {
        return As5047uError::NOT_INITIALIZED;
    }
    
    return handler->SetZeroPosition(zero_position);
}

std::vector<As5047uError> EncoderManager::ReadAllAngles(std::vector<uint16_t>& angles, 
                                                       std::vector<uint8_t>& device_indices) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    angles.clear();
    device_indices.clear();
    std::vector<As5047uError> errors;
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i] && device_initialized_[i] && as5047u_handlers_[i]) {
            uint16_t angle;
            As5047uError error = as5047u_handlers_[i]->ReadAngle(angle);
            
            angles.push_back(angle);
            device_indices.push_back(i);
            errors.push_back(error);
        }
    }
    
    return errors;
}

std::vector<As5047uError> EncoderManager::ReadAllVelocities(std::vector<double>& velocities_rpm, 
                                                           std::vector<uint8_t>& device_indices) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    velocities_rpm.clear();
    device_indices.clear();
    std::vector<As5047uError> errors;
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i] && device_initialized_[i] && as5047u_handlers_[i]) {
            double velocity;
            As5047uError error = as5047u_handlers_[i]->ReadVelocityRPM(velocity);
            
            velocities_rpm.push_back(velocity);
            device_indices.push_back(i);
            errors.push_back(error);
        }
    }
    
    return errors;
}

bool EncoderManager::CheckAllDevicesHealth() noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i] && device_initialized_[i] && as5047u_handlers_[i]) {
            As5047uDiagnostics diagnostics;
            As5047uError error = as5047u_handlers_[i]->ReadDiagnostics(diagnostics);
            
            if (error != As5047uError::SUCCESS || !diagnostics.communication_ok || !diagnostics.magnetic_field_ok) {
                return false;
            }
        }
    }
    
    return true;
}

//**************************************************************************//
//**                  PRIVATE IMPLEMENTATION METHODS                      **//
//**************************************************************************//

bool EncoderManager::Initialize() noexcept {
    Logger::GetInstance().Info("EncoderManager", "Initializing Encoder Manager");
    
    // Get reference to CommChannelsManager
    comm_manager_ = &CommChannelsManager::GetInstance();
    if (!comm_manager_->EnsureInitialized()) {
        Logger::GetInstance().Error("EncoderManager", "Failed to initialize CommChannelsManager");
        return false;
    }
    
    // Get reference to GpioManager
    gpio_manager_ = &GpioManager::GetInstance();
    if (!gpio_manager_->EnsureInitialized()) {
        Logger::GetInstance().Error("EncoderManager", "Failed to initialize GpioManager");
        return false;
    }
    
    // Initialize onboard AS5047U device
    if (!InitializeOnboardAs5047uDevice()) {
        Logger::GetInstance().Warning("EncoderManager", "Failed to initialize onboard AS5047U device");
        // Continue anyway - this is not fatal
    }
    
    // Initialize monitoring statistics
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        measurement_counts_[i].store(0);
        communication_error_counts_[i].store(0);
    }
    
    // Initialize all active devices
    InitializeAllDevices();
    
    initialized_.store(true, std::memory_order_release);
    Logger::GetInstance().Info("EncoderManager", "Encoder Manager initialized successfully");
    
    return true;
}

bool EncoderManager::InitializeOnboardAs5047uDevice() noexcept {
    if (onboard_device_created_) {
        return true; // Already created
    }
    
    // Get onboard AS5047U SPI device
    BaseSpi* spi_interface = comm_manager_->GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
    if (!spi_interface) {
        Logger::GetInstance().Error("EncoderManager", "Failed to get onboard AS5047U SPI interface");
        return false;
    }
    
    // Create onboard AS5047U handler with default config
    auto handler = std::make_unique<As5047uHandler>(*spi_interface, As5047uHandler::GetDefaultConfig());
    if (!handler) {
        Logger::GetInstance().Error("EncoderManager", "Failed to create onboard AS5047U handler");
        return false;
    }
    
    // Store handler and mark as active
    as5047u_handlers_[ONBOARD_ENCODER_INDEX] = std::move(handler);
    device_active_[ONBOARD_ENCODER_INDEX] = true;
    device_initialized_[ONBOARD_ENCODER_INDEX] = false;
    onboard_device_created_ = true;
    
    Logger::GetInstance().Info("EncoderManager", "Onboard AS5047U device created successfully");
    return true;
}

uint32_t EncoderManager::GetMeasurementCount(uint8_t deviceIndex) const noexcept {
    if (deviceIndex >= MAX_ENCODER_DEVICES) {
        return 0;
    }
    return measurement_counts_[deviceIndex].load(std::memory_order_acquire);
}

uint32_t EncoderManager::GetCommunicationErrorCount(uint8_t deviceIndex) const noexcept {
    if (deviceIndex >= MAX_ENCODER_DEVICES) {
        return 0;
    }
    return communication_error_counts_[deviceIndex].load(std::memory_order_acquire);
}

bool EncoderManager::IsExternalDeviceIndex(uint8_t deviceIndex) const noexcept {
    return (deviceIndex == EXTERNAL_ENCODER_1_INDEX || 
            deviceIndex == EXTERNAL_ENCODER_2_INDEX || 
            deviceIndex == EXTERNAL_ENCODER_3_INDEX);
}

//**************************************************************************//
//**                  SENSOR MONITORING METHODS                           **//
//**************************************************************************//

// Note: AS5047U is a continuous reading sensor that doesn't use interrupts.
// Measurement and error tracking is handled through the monitoring methods above.

void EncoderManager::DumpStatistics() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    Logger::GetInstance().Info("EncoderManager", "=== ENCODER MANAGER STATISTICS ===");
    Logger::GetInstance().Info("EncoderManager", "Initialized: %s", initialized_.load() ? "YES" : "NO");
    Logger::GetInstance().Info("EncoderManager", "Active Devices: %u/%u", GetDeviceCount(), MAX_ENCODER_DEVICES);
    
    // Dump individual device statistics
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i]) {
            std::string device_type = (i == ONBOARD_ENCODER_INDEX) ? "Onboard" : "External";
            Logger::GetInstance().Info("EncoderManager", "--- Device %u (%s) ---", i, device_type.c_str());
            Logger::GetInstance().Info("EncoderManager", "  Active: YES");
            Logger::GetInstance().Info("EncoderManager", "  Initialized: %s", device_initialized_[i] ? "YES" : "NO");
            Logger::GetInstance().Info("EncoderManager", "  Measurements: %u", measurement_counts_[i].load());
            Logger::GetInstance().Info("EncoderManager", "  Comm Errors: %u", communication_error_counts_[i].load());
            
            if (device_initialized_[i] && as5047u_handlers_[i]) {
                // Dump handler-specific diagnostics
                as5047u_handlers_[i]->DumpDiagnostics();
            }
        }
    }
}