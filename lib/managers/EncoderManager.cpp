#include "EncoderManager.h"

// As5047u handler for unified encoder interface
#include "handlers/as5047u/As5047uHandler.h"

// Communication manager for SPI access
#include "CommChannelsManager.h"

// GPIO manager for interrupt pin access
#include "GpioManager.h"

// Logger for unified logging
#include "handlers/logger/Logger.h"

// Base interfaces
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseSpi.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseGpio.h"

// Platform mapping for functional pin definitions

// RtosMutex for thread safety
#include "RtosMutex.h"

// Standard library
#include <atomic>

// ESP-IDF for semaphores

//==============================================================================
// ENCODERMANAGER IMPLEMENTATION
//==============================================================================

EncoderManager& EncoderManager::GetInstance() noexcept {
    static EncoderManager instance;
    return instance;
}

EncoderManager::EncoderManager() noexcept
    : initialized_(false), manager_mutex_() {
    // Initialize all device slots as empty and not active
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
    
    return &*as5047u_handlers_[deviceIndex];
}

as5047u::AS5047U<As5047uSpiAdapter>* EncoderManager::GetAs5047uDriver(uint8_t deviceIndex) noexcept {
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

bool EncoderManager::CreateAs5047uDevice(uint8_t deviceIndex,
                                         SpiDeviceId spiDeviceId,
                                         const As5047uConfig& config) noexcept {
    MutexLockGuard lock(manager_mutex_);

    if (!IsValidDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("EncoderManager", "Invalid device index %u", deviceIndex);
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
    // Store handler and mark as active
    as5047u_handlers_[deviceIndex].emplace(*spi_interface, config);
    device_active_[deviceIndex] = true;
    device_initialized_[deviceIndex] = false;
    
    // Initialize if manager is already initialized
    if (initialized_.load(std::memory_order_acquire)) {
        device_initialized_[deviceIndex] = as5047u_handlers_[deviceIndex]->Initialize();

        if (device_initialized_[deviceIndex]) {
            Logger::GetInstance().Info("EncoderManager", "AS5047U device %u created and initialized", deviceIndex);
        } else {
            Logger::GetInstance().Error("EncoderManager", "AS5047U device %u created but init failed (flags=0x%04X)",
                                      deviceIndex, static_cast<uint16_t>(as5047u_handlers_[deviceIndex]->GetLastError()));
        }
    } else {
        Logger::GetInstance().Info("EncoderManager", "AS5047U device %u created (will be initialized later)", deviceIndex);
    }

    return true;
}

bool EncoderManager::CreateAs5047uDevice(uint8_t deviceIndex,
                                         BaseSpi& spi_interface,
                                         const As5047uConfig& config) noexcept {
    MutexLockGuard lock(manager_mutex_);

    if (!IsValidDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("EncoderManager", "Invalid device index %u", deviceIndex);
        return false;
    }
    
    if (device_active_[deviceIndex]) {
        Logger::GetInstance().Error("EncoderManager", "Device slot %u already occupied", deviceIndex);
        return false;
    }
    
    // Create AS5047U handler with direct SPI interface
    // Store handler and mark as active
    as5047u_handlers_[deviceIndex].emplace(spi_interface, config);
    device_active_[deviceIndex] = true;
    device_initialized_[deviceIndex] = false;
    
    // Initialize if manager is already initialized
    if (initialized_.load(std::memory_order_acquire)) {
        device_initialized_[deviceIndex] = as5047u_handlers_[deviceIndex]->Initialize();

        if (device_initialized_[deviceIndex]) {
            Logger::GetInstance().Info("EncoderManager", "AS5047U device %u (direct SPI) created and initialized", deviceIndex);
        } else {
            Logger::GetInstance().Error("EncoderManager", "AS5047U device %u (direct SPI) init failed (flags=0x%04X)",
                                      deviceIndex, static_cast<uint16_t>(as5047u_handlers_[deviceIndex]->GetLastError()));
        }
    } else {
        Logger::GetInstance().Info("EncoderManager", "AS5047U device %u (direct SPI) created (will init later)", deviceIndex);
    }
    
    return true;
}

bool EncoderManager::DeleteDevice(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(manager_mutex_);

    if (!IsValidDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("EncoderManager", "Invalid device index %u", deviceIndex);
        return false;
    }

    if (!device_active_[deviceIndex]) {
        Logger::GetInstance().Warn("EncoderManager", "Device slot %u is already empty", deviceIndex);
        return true;
    }

    if (as5047u_handlers_[deviceIndex]) {
        as5047u_handlers_[deviceIndex]->Deinitialize();
        as5047u_handlers_[deviceIndex].reset();
    }

    device_active_[deviceIndex] = false;
    device_initialized_[deviceIndex] = false;

    Logger::GetInstance().Info("EncoderManager", "AS5047U device %u deleted", deviceIndex);
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

    if (!IsValidDeviceIndex(deviceIndex)) {
        return false;
    }

    return !device_active_[deviceIndex];
}

void EncoderManager::GetActiveDeviceIndices(std::array<uint8_t, MAX_ENCODER_DEVICES>& out, size_t& count) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    count = 0;
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i]) {
            out[count++] = i;
        }
    }
}

size_t EncoderManager::InitializeAllDevices(std::array<bool, MAX_ENCODER_DEVICES>& results) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    results.fill(false);
    size_t success_count = 0;
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i] && as5047u_handlers_[i]) {
            device_initialized_[i] = as5047u_handlers_[i]->Initialize();
            results[i] = device_initialized_[i];
            
            if (device_initialized_[i]) {
                ++success_count;
                Logger::GetInstance().Info("EncoderManager", "AS5047U device %u initialized successfully", i);
            } else {
                Logger::GetInstance().Error("EncoderManager", "AS5047U device %u initialization failed (driver_flags=0x%04X)", 
                                          i, static_cast<uint16_t>(as5047u_handlers_[i]->GetLastError()));
            }
        }
    }
    
    return success_count;
}

void EncoderManager::GetInitializationStatus(std::array<bool, MAX_ENCODER_DEVICES>& status) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        status[i] = device_active_[i] ? device_initialized_[i] : false;
    }
}

//**************************************************************************//
//**                  DEVICE INFORMATION METHODS                          **//
//**************************************************************************//

void EncoderManager::GetAvailableDevices(std::array<const char*, MAX_ENCODER_DEVICES>& out, size_t& count) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    count = 0;
    static constexpr const char* kDeviceNames[] = {
        "AS5047U-0",
        "AS5047U-1",
        "AS5047U-2",
        "AS5047U-3"
    };
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i]) {
            out[count++] = kDeviceNames[i];
        }
    }
}

const char* EncoderManager::GetDeviceType(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (deviceIndex >= MAX_ENCODER_DEVICES || !device_active_[deviceIndex]) {
        return nullptr;
    }
    
    return "AS5047U";
}

//**************************************************************************//
//**                  DRIVER ERROR MAPPING                                **//
//**************************************************************************//

EncoderError EncoderManager::mapDriverError(AS5047U_Error sticky_flags) noexcept {
    auto flags = static_cast<uint16_t>(sticky_flags);
    
    // Communication errors (highest severity)
    if (flags & static_cast<uint16_t>(AS5047U_Error::CrcError)) {
        return EncoderError::CRC_ERROR;
    }
    if (flags & static_cast<uint16_t>(AS5047U_Error::FramingError)) {
        return EncoderError::FRAMING_ERROR;
    }
    if (flags & static_cast<uint16_t>(AS5047U_Error::CommandError)) {
        return EncoderError::SPI_COMMUNICATION_FAILED;
    }
    
    // Sensor errors
    if (flags & static_cast<uint16_t>(AS5047U_Error::CordicOverflow)) {
        return EncoderError::SENSOR_ERROR;
    }
    if (flags & static_cast<uint16_t>(AS5047U_Error::OffCompError)) {
        return EncoderError::SENSOR_ERROR;
    }
    if (flags & static_cast<uint16_t>(AS5047U_Error::WatchdogError)) {
        return EncoderError::SENSOR_ERROR;
    }
    if (flags & static_cast<uint16_t>(AS5047U_Error::P2ramError)) {
        return EncoderError::SENSOR_ERROR;
    }
    
    // Warnings (P2RAM warning, AGC warning, MagHalf are non-fatal)
    return EncoderError::SENSOR_ERROR;
}

//**************************************************************************//
//**                  HIGH-LEVEL ENCODER OPERATIONS                       **//
//**************************************************************************//

// Every single-device operation below holds manager_mutex_ for its whole duration, so the
// slot cannot be deleted or re-created underneath it, and reaches the driver through
// As5047uHandler::visitDriver(), which holds the handler mutex for the SPI exchange.
// Lock order: manager_mutex_ -> handler mutex (never the reverse).

As5047uHandler* EncoderManager::ReadyHandlerLocked(uint8_t deviceIndex) noexcept {
    if (!IsValidDeviceIndex(deviceIndex) || !device_active_[deviceIndex] ||
        !device_initialized_[deviceIndex] || !as5047u_handlers_[deviceIndex]) {
        return nullptr;
    }
    return &*as5047u_handlers_[deviceIndex];
}

EncoderError EncoderManager::FinishRead(uint8_t deviceIndex, AS5047U_Error sticky,
                                        bool count_measurement) noexcept {
    if (sticky != AS5047U_Error::None) {
        communication_error_counts_[deviceIndex].fetch_add(1, std::memory_order_acq_rel);
        const EncoderError err = mapDriverError(sticky);
        last_error_.store(err, std::memory_order_release);
        return err;
    }
    if (count_measurement) {
        measurement_counts_[deviceIndex].fetch_add(1, std::memory_order_acq_rel);
    }
    return EncoderError::SUCCESS;
}

EncoderError EncoderManager::ReadAngle(uint8_t deviceIndex, uint16_t& angle) noexcept {
    MutexLockGuard lock(manager_mutex_);
    As5047uHandler* handler = ReadyHandlerLocked(deviceIndex);
    if (!handler) {
        return EncoderError::NOT_INITIALIZED;
    }
    bool ran = false;
    AS5047U_Error sticky = AS5047U_Error::None;
    handler->visitDriver([&](auto& sensor) {
        ran = true;
        angle = sensor.GetAngle();
        sticky = sensor.GetStickyErrorFlags();
    });
    if (!ran) {
        return EncoderError::NOT_INITIALIZED;
    }
    return FinishRead(deviceIndex, sticky, true);
}

EncoderError EncoderManager::ReadAngleDegrees(uint8_t deviceIndex, double& angle_degrees) noexcept {
    MutexLockGuard lock(manager_mutex_);
    As5047uHandler* handler = ReadyHandlerLocked(deviceIndex);
    if (!handler) {
        return EncoderError::NOT_INITIALIZED;
    }
    bool ran = false;
    AS5047U_Error sticky = AS5047U_Error::None;
    handler->visitDriver([&](auto& sensor) {
        ran = true;
        angle_degrees = static_cast<double>(sensor.GetAngle(as5047u::AngleUnit::Degrees));
        sticky = sensor.GetStickyErrorFlags();
    });
    if (!ran) {
        return EncoderError::NOT_INITIALIZED;
    }
    return FinishRead(deviceIndex, sticky, true);
}

EncoderError EncoderManager::ReadVelocityRPM(uint8_t deviceIndex, double& velocity_rpm) noexcept {
    MutexLockGuard lock(manager_mutex_);
    As5047uHandler* handler = ReadyHandlerLocked(deviceIndex);
    if (!handler) {
        return EncoderError::NOT_INITIALIZED;
    }
    bool ran = false;
    AS5047U_Error sticky = AS5047U_Error::None;
    handler->visitDriver([&](auto& sensor) {
        ran = true;
        velocity_rpm = static_cast<double>(sensor.GetVelocity(as5047u::VelocityUnit::Rpm));
        sticky = sensor.GetStickyErrorFlags();
    });
    if (!ran) {
        return EncoderError::NOT_INITIALIZED;
    }
    return FinishRead(deviceIndex, sticky, true);
}

EncoderError EncoderManager::ReadDiagnostics(uint8_t deviceIndex, As5047uDiagnostics& diagnostics) noexcept {
    MutexLockGuard lock(manager_mutex_);
    As5047uHandler* handler = ReadyHandlerLocked(deviceIndex);
    if (!handler) {
        return EncoderError::NOT_INITIALIZED;
    }
    // One ERRFL read (it clears on read). AGC and magnitude are not part of As5047uDiagnostics;
    // ReadFieldStrength() reads them when a caller wants them.
    bool ran = false;
    uint16_t error_flags = 0;
    handler->visitDriver([&](auto& sensor) {
        ran = true;
        error_flags = sensor.GetErrorFlags();
        (void)sensor.GetStickyErrorFlags();  // drop transport flags raised by this read
    });
    if (!ran) {
        return EncoderError::NOT_INITIALIZED;
    }

    const auto has = [error_flags](AS5047U_Error bit) noexcept {
        return (error_flags & static_cast<uint16_t>(bit)) != 0;
    };
    diagnostics.last_error_flags = error_flags;
    diagnostics.agc_warning = has(AS5047U_Error::AgcWarning);
    diagnostics.cordic_overflow = has(AS5047U_Error::CordicOverflow);
    diagnostics.offset_compensation_ok = !has(AS5047U_Error::OffCompError);
    diagnostics.magnetic_field_ok = !has(AS5047U_Error::MagHalf);
    diagnostics.communication_ok = !has(AS5047U_Error::CrcError) && !has(AS5047U_Error::FramingError) &&
                                   !has(AS5047U_Error::CommandError);
    diagnostics.communication_errors = communication_error_counts_[deviceIndex].load(std::memory_order_acquire);
    diagnostics.total_measurements = measurement_counts_[deviceIndex].load(std::memory_order_acquire);
    return EncoderError::SUCCESS;
}

EncoderError EncoderManager::ReadFieldStrength(uint8_t deviceIndex, EncoderFieldStrength& field) noexcept {
    MutexLockGuard lock(manager_mutex_);
    As5047uHandler* handler = ReadyHandlerLocked(deviceIndex);
    if (!handler) {
        return EncoderError::NOT_INITIALIZED;
    }
    bool ran = false;
    AS5047U_Error sticky = AS5047U_Error::None;
    handler->visitDriver([&](auto& sensor) {
        ran = true;
        field.agc = sensor.GetAGC();
        field.magnitude = sensor.GetMagnitude();
        sticky = sensor.GetStickyErrorFlags();
    });
    if (!ran) {
        return EncoderError::NOT_INITIALIZED;
    }
    return FinishRead(deviceIndex, sticky, false);
}

EncoderError EncoderManager::SetZeroPosition(uint8_t deviceIndex, uint16_t zero_position) noexcept {
    MutexLockGuard lock(manager_mutex_);
    As5047uHandler* handler = ReadyHandlerLocked(deviceIndex);
    if (!handler) {
        return EncoderError::NOT_INITIALIZED;
    }
    bool ran = false;
    bool ok = false;
    AS5047U_Error sticky = AS5047U_Error::None;
    handler->visitDriver([&](auto& sensor) {
        ran = true;
        ok = sensor.SetZeroPosition(zero_position);
        sticky = sensor.GetStickyErrorFlags();
    });
    if (!ran) {
        return EncoderError::NOT_INITIALIZED;
    }
    if (!ok) {
        const EncoderError err =
            (sticky != AS5047U_Error::None) ? mapDriverError(sticky) : EncoderError::SPI_COMMUNICATION_FAILED;
        last_error_.store(err, std::memory_order_release);
        return err;
    }
    return EncoderError::SUCCESS;
}

void EncoderManager::ReadAllAngles(std::array<uint16_t, MAX_ENCODER_DEVICES>& angles,
                                   std::array<uint8_t, MAX_ENCODER_DEVICES>& device_indices,
                                   std::array<EncoderError, MAX_ENCODER_DEVICES>& errors,
                                   size_t& count) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    count = 0;
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i] && device_initialized_[i] && as5047u_handlers_[i]) {
            uint16_t angle = 0;
            EncoderError error = ReadAngle(i, angle);
            
            angles[count] = angle;
            device_indices[count] = i;
            errors[count] = error;
            ++count;
        }
    }
}

void EncoderManager::ReadAllVelocities(std::array<double, MAX_ENCODER_DEVICES>& velocities_rpm,
                                       std::array<uint8_t, MAX_ENCODER_DEVICES>& device_indices,
                                       std::array<EncoderError, MAX_ENCODER_DEVICES>& errors,
                                       size_t& count) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    count = 0;
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i] && device_initialized_[i] && as5047u_handlers_[i]) {
            double velocity = 0.0;
            EncoderError error = ReadVelocityRPM(i, velocity);
            
            velocities_rpm[count] = velocity;
            device_indices[count] = i;
            errors[count] = error;
            ++count;
        }
    }
}

bool EncoderManager::CheckAllDevicesHealth() noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i] && device_initialized_[i] && as5047u_handlers_[i]) {
            As5047uDiagnostics diagnostics{};
            EncoderError error = ReadDiagnostics(i, diagnostics);
            
            if (error != EncoderError::SUCCESS || !diagnostics.communication_ok || !diagnostics.magnetic_field_ok) {
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
    
    // Vortex V1 has no onboard encoder — users add external AS5047U devices
    // via CreateExternalAs5047uDevice() with their own SPI CS line.

    // Initialize monitoring statistics
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        measurement_counts_[i].store(0);
        communication_error_counts_[i].store(0);
    }
    
    // Initialize all active devices
    {
        std::array<bool, MAX_ENCODER_DEVICES> init_results{};
        InitializeAllDevices(init_results);
    }
    
    initialized_.store(true, std::memory_order_release);
    Logger::GetInstance().Info("EncoderManager", "Encoder Manager initialized successfully");
    
    return true;
}

// InitializeOnboardAs5047uDevice() removed — Vortex V1 has no onboard encoder.
// Users call CreateAs5047uDevice() to register their encoder(s).

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

// IsExternalDeviceIndex() removed — all slots are equivalent (no onboard encoder).

//**************************************************************************//
//**                  SENSOR MONITORING METHODS                           **//
//**************************************************************************//

// Note: AS5047U is a continuous reading sensor that doesn't use interrupts.
// Measurement and error tracking is handled through the monitoring methods above.

EncoderError EncoderManager::GetSystemDiagnostics(EncoderSystemDiagnostics& diagnostics) const noexcept {
    if (!initialized_.load(std::memory_order_acquire)) {
        return EncoderError::NOT_INITIALIZED;
    }

    MutexLockGuard lock(manager_mutex_);

    diagnostics.system_initialized = true;
    diagnostics.last_error = last_error_.load(std::memory_order_acquire);
    diagnostics.active_device_count = 0;
    diagnostics.initialized_device_count = 0;

    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        diagnostics.devices[i].active = device_active_[i];
        diagnostics.devices[i].initialized = device_initialized_[i];
        diagnostics.devices[i].measurement_count = measurement_counts_[i].load(std::memory_order_relaxed);
        diagnostics.devices[i].communication_error_count = communication_error_counts_[i].load(std::memory_order_relaxed);
        if (device_active_[i]) ++diagnostics.active_device_count;
        if (device_initialized_[i]) ++diagnostics.initialized_device_count;
    }

    diagnostics.system_healthy = (diagnostics.active_device_count > 0) &&
                                 (diagnostics.initialized_device_count == diagnostics.active_device_count);
    return EncoderError::SUCCESS;
}

void EncoderManager::DumpStatistics() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    Logger::GetInstance().Info("EncoderManager", "=== ENCODER MANAGER STATISTICS ===");
    Logger::GetInstance().Info("EncoderManager", "Initialized: %s", initialized_.load() ? "YES" : "NO");
    Logger::GetInstance().Info("EncoderManager", "Active Devices: %u/%u", GetDeviceCount(), MAX_ENCODER_DEVICES);
    
    for (uint8_t i = 0; i < MAX_ENCODER_DEVICES; ++i) {
        if (device_active_[i]) {
            Logger::GetInstance().Info("EncoderManager", "--- Device %u ---", i);
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