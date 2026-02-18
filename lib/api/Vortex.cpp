/**
 * @file Vortex.cpp
 * @brief Implementation of the unified Vortex API singleton.
 * 
 * @details This file implements the Vortex API singleton with proper initialization
 *          order, dependency management, and comprehensive error handling.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 */

#include "Vortex.h"

#include <array>
#include <cstring>

// Include all component handler headers
#include "managers/CommChannelsManager.h"
#include "managers/GpioManager.h"
#include "managers/AdcManager.h"
#include "managers/MotorController.h"
#include "managers/ImuManager.h"
#include "managers/EncoderManager.h"
#include "managers/LedManager.h"
#include "managers/TemperatureManager.h"

// Include OS abstraction for timing
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsAbstraction.h"

//==============================================================================
// SINGLETON IMPLEMENTATION
//==============================================================================

Vortex& Vortex::GetInstance() noexcept {
    static Vortex instance;
    return instance;
}

//==============================================================================
// CONSTRUCTOR AND DESTRUCTOR
//==============================================================================

Vortex::Vortex() noexcept
    : initialized_(false),
      initialization_start_time_(0),
      initialization_end_time_(0),
      comms_initialized_(false),
      gpio_initialized_(false),
      motors_initialized_(false),
      adc_initialized_(false),
      imu_initialized_(false),
      encoders_initialized_(false),
      leds_initialized_(false),
      temp_initialized_(false),
      diagnostics_(),
      system_warnings_{},
      system_warnings_count_(0),
      failed_components_{},
      failed_components_count_(0),
      // Initialize references to singletons (these are references, not owned)
      comms_ref_(CommChannelsManager::GetInstance()),
      gpio_ref_(GpioManager::GetInstance()),
      motors_ref_(MotorController::GetInstance()),
      adc_ref_(AdcManager::GetInstance()),
      imu_ref_(ImuManager::GetInstance()),
      encoders_ref_(EncoderManager::GetInstance()),
      leds_ref_(LedManager::GetInstance()),
      temp_ref_(TemperatureManager::GetInstance()),
      // Initialize public references (these point to the private references)
      comms(comms_ref_),
      gpio(gpio_ref_),
      motors(motors_ref_),
      adc(adc_ref_),
      imu(imu_ref_),
      encoders(encoders_ref_),
      leds(leds_ref_),
      temp(temp_ref_) {
    
    // Initialize Logger first (this is always available)
    Logger::GetInstance().Initialize();
    
    Logger::GetInstance().Info("Vortex", "Vortex API singleton created");
}

//==============================================================================
// PUBLIC INITIALIZATION METHODS
//==============================================================================

bool Vortex::EnsureInitialized() noexcept {
    if (initialized_) {
        return true;
    }
    
    Logger::GetInstance().Info("Vortex", "Starting Vortex API initialization");
    
    // Record initialization start time
    initialization_start_time_ = os_time_get();
    
    // Initialize all components in proper order
    bool success = InitializeAllComponents();
    
    // Record initialization end time
    initialization_end_time_ = os_time_get();
    
    if (success) {
        initialized_ = true;
        UpdateSystemDiagnostics();
        
        uint64_t init_time_ms = GetInitializationTimeMs();
        Logger::GetInstance().Info("Vortex", "Vortex API initialization completed successfully in %llu ms", init_time_ms);
        
        // Log component status
        Logger::GetInstance().Info("Vortex", "Component Status - Comms: %s, GPIO: %s, Motors: %s, ADC: %s, IMU: %s, Encoders: %s, LEDs: %s, Temp: %s",
            comms_initialized_ ? "OK" : "FAIL",
            gpio_initialized_ ? "OK" : "FAIL",
            motors_initialized_ ? "OK" : "FAIL",
            adc_initialized_ ? "OK" : "FAIL",
            imu_initialized_ ? "OK" : "FAIL",
            encoders_initialized_ ? "OK" : "FAIL",
            leds_initialized_ ? "OK" : "FAIL",
            temp_initialized_ ? "OK" : "FAIL");
    } else {
        Logger::GetInstance().Error("Vortex", "Vortex API initialization failed");
        
        // Log failed components
        if (failed_components_count_ > 0) {
            char failed_list[256] = {};
            size_t pos = 0;
            for (size_t i = 0; i < failed_components_count_; ++i) {
                if (pos > 0 && pos + 2 < sizeof(failed_list)) {
                    failed_list[pos++] = ',';
                    failed_list[pos++] = ' ';
                }
                size_t len = std::strlen(failed_components_[i]);
                if (pos + len < sizeof(failed_list) - 1) {
                    std::memcpy(failed_list + pos, failed_components_[i], len);
                    pos += len;
                }
            }
            failed_list[pos] = '\0';
            Logger::GetInstance().Error("Vortex", "Failed components: %s", failed_list);
        }
    }
    
    return success;
}

bool Vortex::GetSystemDiagnostics(VortexSystemDiagnostics& diagnostics) const noexcept {
    if (!initialized_) {
        return false;
    }
    
    // Update diagnostics with current state
    const_cast<Vortex*>(this)->UpdateSystemDiagnostics();
    
    // Copy current diagnostics
    diagnostics = diagnostics_;
    return true;
}

std::array<bool, 8> Vortex::GetComponentInitializationStatus() const noexcept {
    return {{
        comms_initialized_.load(),
        gpio_initialized_.load(),
        motors_initialized_.load(),
        adc_initialized_.load(),
        imu_initialized_.load(),
        encoders_initialized_.load(),
        leds_initialized_.load(),
        temp_initialized_.load()
    }};
}

size_t Vortex::GetFailedComponents(const char* out_names[], size_t max_entries) const noexcept {
    size_t count = (failed_components_count_ < max_entries) ? failed_components_count_ : max_entries;
    for (size_t i = 0; i < count; ++i) {
        out_names[i] = failed_components_[i];
    }
    return count;
}

size_t Vortex::GetSystemWarnings(const char* out_warnings[], size_t max_entries) const noexcept {
    size_t count = (system_warnings_count_ < max_entries) ? system_warnings_count_ : max_entries;
    for (size_t i = 0; i < count; ++i) {
        out_warnings[i] = system_warnings_[i];
    }
    return count;
}

//==============================================================================
// PUBLIC UTILITY METHODS
//==============================================================================

uint64_t Vortex::GetSystemUptimeMs() const noexcept {
    return os_time_get() * portTICK_PERIOD_MS;
}

uint64_t Vortex::GetInitializationTimeMs() const noexcept {
    if (initialization_start_time_ == 0 || initialization_end_time_ == 0) {
        return 0;
    }
    return (initialization_end_time_ - initialization_start_time_) * portTICK_PERIOD_MS;
}

void Vortex::DumpSystemStatistics() const noexcept {
    if (!initialized_) {
        Logger::GetInstance().Warn("Vortex", "Cannot dump statistics - Vortex API not initialized");
        return;
    }
    
    Logger::GetInstance().Info("Vortex", "=== Vortex API System Statistics ===");
    Logger::GetInstance().Info("Vortex", "System Uptime: %llu ms", GetSystemUptimeMs());
    Logger::GetInstance().Info("Vortex", "Initialization Time: %llu ms", GetInitializationTimeMs());
    
    // Dump statistics from all component handlers
    comms_ref_.DumpStatistics();
    gpio_ref_.DumpStatistics();
    motors_ref_.DumpStatistics();
    adc_ref_.DumpStatistics();
    imu_ref_.DumpStatistics();
    encoders_ref_.DumpStatistics();
    leds_ref_.DumpStatistics();
    temp_ref_.DumpStatistics();
    
    Logger::GetInstance().Info("Vortex", "=== End Vortex API System Statistics ===");
}

bool Vortex::PerformHealthCheck() noexcept {
    if (!initialized_) {
        Logger::GetInstance().Warn("Vortex", "Cannot perform health check - Vortex API not initialized");
        return false;
    }
    
    Logger::GetInstance().Info("Vortex", "Performing Vortex API health check");
    
    // Check each component's health
    bool comms_healthy = comms_initialized_ && comms_ref_.IsInitialized();
    bool gpio_healthy = gpio_initialized_ && gpio_ref_.IsInitialized();
    bool motors_healthy = motors_initialized_ && motors_ref_.IsInitialized();
    bool adc_healthy = adc_initialized_ && adc_ref_.IsInitialized();
    bool imu_healthy = imu_initialized_ && imu_ref_.IsInitialized();
    bool encoders_healthy = encoders_initialized_ && encoders_ref_.IsInitialized();
    bool leds_healthy = leds_initialized_ && leds_ref_.IsInitialized();
    bool temp_healthy = temp_initialized_ && temp_ref_.IsInitialized();
    
    bool overall_healthy = comms_healthy && gpio_healthy && motors_healthy && 
                          adc_healthy && imu_healthy && encoders_healthy && 
                          leds_healthy && temp_healthy;
    
    Logger::GetInstance().Info("Vortex", "Health Check Results - Overall: %s", overall_healthy ? "HEALTHY" : "UNHEALTHY");
    Logger::GetInstance().Info("Vortex", "  Comms: %s, GPIO: %s, Motors: %s, ADC: %s", 
        comms_healthy ? "OK" : "FAIL", gpio_healthy ? "OK" : "FAIL", 
        motors_healthy ? "OK" : "FAIL", adc_healthy ? "OK" : "FAIL");
    Logger::GetInstance().Info("Vortex", "  IMU: %s, Encoders: %s, LEDs: %s, Temp: %s", 
        imu_healthy ? "OK" : "FAIL", encoders_healthy ? "OK" : "FAIL", 
        leds_healthy ? "OK" : "FAIL", temp_healthy ? "OK" : "FAIL");
    
    return overall_healthy;
}

const char* Vortex::GetSystemVersion() const noexcept {
    return "Vortex API v1.0.0 - HardFOC Platform";
}

//==============================================================================
// PRIVATE INITIALIZATION METHODS
//==============================================================================

bool Vortex::InitializeAllComponents() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing all Vortex components in dependency order");
    
    // Step 1: Initialize communication channels (foundation)
    if (!InitializeComms()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize communication channels");
        return false;
    }
    
    // Step 2: Initialize GPIO management (depends on CommChannelsManager)
    if (!InitializeGpio()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize GPIO management");
        return false;
    }
    
    // Step 3: Initialize motor controllers (depends on CommChannelsManager)
    if (!InitializeMotors()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize motor controllers");
        return false;
    }
    
    // Step 4: Initialize ADC management (depends on MotorController)
    if (!InitializeAdc()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize ADC management");
        return false;
    }
    
    // Step 5: Initialize IMU management (depends on CommChannelsManager, GpioManager)
    if (!InitializeImu()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize IMU management");
        return false;
    }
    
    // Step 6: Initialize encoder management (depends on CommChannelsManager, GpioManager)
    if (!InitializeEncoders()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize encoder management");
        return false;
    }
    
    // Step 7: Initialize LED management (independent)
    if (!InitializeLeds()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize LED management");
        return false;
    }
    
    // Step 8: Initialize temperature management (depends on AdcManager, MotorController)
    if (!InitializeTemperature()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize temperature management");
        return false;
    }
    
    Logger::GetInstance().Info("Vortex", "All Vortex components initialized successfully");
    return true;
}

bool Vortex::InitializeComms() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing communication channels");
    
    bool success = comms_ref_.EnsureInitialized();
    comms_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "Communication channels initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "Communication channels initialization failed");
        if (failed_components_count_ < kMaxFailedComponents)
            failed_components_[failed_components_count_++] = "CommChannelsManager";
    }
    
    return success;
}

bool Vortex::InitializeGpio() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing GPIO management");
    
    bool success = gpio_ref_.EnsureInitialized();
    gpio_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "GPIO management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "GPIO management initialization failed");
        if (failed_components_count_ < kMaxFailedComponents)
            failed_components_[failed_components_count_++] = "GpioManager";
    }
    
    return success;
}

bool Vortex::InitializeMotors() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing motor controllers");
    
    bool success = motors_ref_.EnsureInitialized();
    motors_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "Motor controllers initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "Motor controllers initialization failed");
        if (failed_components_count_ < kMaxFailedComponents)
            failed_components_[failed_components_count_++] = "MotorController";
    }
    
    return success;
}

bool Vortex::InitializeAdc() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing ADC management");
    
    bool success = adc_ref_.EnsureInitialized();
    adc_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "ADC management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "ADC management initialization failed");
        if (failed_components_count_ < kMaxFailedComponents)
            failed_components_[failed_components_count_++] = "AdcManager";
    }
    
    return success;
}

bool Vortex::InitializeImu() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing IMU management");
    
    bool success = imu_ref_.EnsureInitialized();
    imu_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "IMU management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "IMU management initialization failed");
        if (failed_components_count_ < kMaxFailedComponents)
            failed_components_[failed_components_count_++] = "ImuManager";
    }
    
    return success;
}

bool Vortex::InitializeEncoders() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing encoder management");
    
    bool success = encoders_ref_.EnsureInitialized();
    encoders_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "Encoder management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "Encoder management initialization failed");
        if (failed_components_count_ < kMaxFailedComponents)
            failed_components_[failed_components_count_++] = "EncoderManager";
    }
    
    return success;
}

bool Vortex::InitializeLeds() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing LED management");
    
    bool success = leds_ref_.EnsureInitialized();
    leds_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "LED management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "LED management initialization failed");
        if (failed_components_count_ < kMaxFailedComponents)
            failed_components_[failed_components_count_++] = "LedManager";
    }
    
    return success;
}

bool Vortex::InitializeTemperature() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing temperature management");
    
    bool success = temp_ref_.EnsureInitialized();
    temp_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "Temperature management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "Temperature management initialization failed");
        if (failed_components_count_ < kMaxFailedComponents)
            failed_components_[failed_components_count_++] = "TemperatureManager";
    }
    
    return success;
}

//==============================================================================
// PRIVATE UTILITY METHODS
//==============================================================================

void Vortex::UpdateSystemDiagnostics() noexcept {
    // Update component status
    diagnostics_.comms_initialized = comms_initialized_;
    diagnostics_.gpio_initialized = gpio_initialized_;
    diagnostics_.motors_initialized = motors_initialized_;
    diagnostics_.adc_initialized = adc_initialized_;
    diagnostics_.imu_initialized = imu_initialized_;
    diagnostics_.encoders_initialized = encoders_initialized_;
    diagnostics_.leds_initialized = leds_initialized_;
    diagnostics_.temp_initialized = temp_initialized_;
    
    // Count initialized components
    diagnostics_.initialized_components = 0;
    if (comms_initialized_) diagnostics_.initialized_components++;
    if (gpio_initialized_) diagnostics_.initialized_components++;
    if (motors_initialized_) diagnostics_.initialized_components++;
    if (adc_initialized_) diagnostics_.initialized_components++;
    if (imu_initialized_) diagnostics_.initialized_components++;
    if (encoders_initialized_) diagnostics_.initialized_components++;
    if (leds_initialized_) diagnostics_.initialized_components++;
    if (temp_initialized_) diagnostics_.initialized_components++;
    
    // Calculate failed components
    diagnostics_.failed_components = diagnostics_.total_components - diagnostics_.initialized_components;
    
    // Update timing information
    diagnostics_.initialization_time_ms = GetInitializationTimeMs();
    diagnostics_.system_uptime_ms = GetSystemUptimeMs();
    
    // Update system health
    diagnostics_.system_healthy = (diagnostics_.failed_components == 0) && initialized_;
    
    // Update failed components list
    diagnostics_.failed_components_list_count = failed_components_count_;
    for (size_t i = 0; i < failed_components_count_ && i < VortexSystemDiagnostics::kMaxDiagEntries; ++i) {
        diagnostics_.failed_components_list[i] = failed_components_[i];
    }
    
    // Update warnings
    diagnostics_.warnings_count = system_warnings_count_;
    for (size_t i = 0; i < system_warnings_count_ && i < VortexSystemDiagnostics::kMaxDiagEntries; ++i) {
        diagnostics_.warnings[i] = system_warnings_[i];
    }
}

void Vortex::AddSystemWarning(const char* warning) noexcept {
    if (system_warnings_count_ < kMaxWarnings) {
        system_warnings_[system_warnings_count_++] = warning;
    }
    Logger::GetInstance().Warn("Vortex", "Vortex API Warning: %s", warning);
} 