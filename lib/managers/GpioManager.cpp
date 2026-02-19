/**
 * @file GpioManager.cpp
 * @brief Advanced GPIO management system implementation for the HardFOC platform.
 * 
 * @details This implementation provides comprehensive GPIO management with:
 *          - String-based pin identification for extensibility
 *          - Complete BaseGpio function coverage through routing
 *          - Smart pin categorization (CORE, COMM, GPIO)
 *          - Handler-aware GPIO creation and ownership
 *          - Thread-safe operations with comprehensive error handling
 *          - Platform mapping integration for automatic pin discovery
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 */

#include "GpioManager.h"
#include "CommChannelsManager.h"
#include "MotorController.h"
#include "mcu/esp32/EspGpio.h"
#include "RtosMutex.h"
#include "handlers/logger/Logger.h"

#include <algorithm>
#include <cstring>
#include <sstream>
#include <iomanip>

//==============================================================================
// STATIC INSTANCE
//==============================================================================

GpioManager& GpioManager::GetInstance() noexcept {
    static GpioManager instance;
    return instance;
}

//==============================================================================
// INITIALIZATION AND LIFECYCLE
//==============================================================================

bool GpioManager::EnsureInitialized() noexcept {
    if (is_initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    MutexLockGuard lock(mutex_);
    
    // Double-check after acquiring lock
    if (is_initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    return Initialize();
}

bool GpioManager::Shutdown() noexcept {
    // Quick check without lock first
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    // Acquire main mutex only for state change
    {
        MutexLockGuard lock(mutex_);
        
        if (!is_initialized_.load(std::memory_order_acquire)) {
            return true;
        }
        
        // Mark as not initialized early to prevent new operations
        is_initialized_.store(false, std::memory_order_release);
    }
    
    // Clear resources with separate lock (no need for main mutex anymore)
    {
        MutexLockGuard registry_lock(registry_mutex_);
        gpio_registry_.clear();
    }
    
    // Reset statistics (atomic operations, no locks needed)
    total_operations_.store(0, std::memory_order_relaxed);
    successful_operations_.store(0, std::memory_order_relaxed);
    failed_operations_.store(0, std::memory_order_relaxed);
    communication_errors_.store(0, std::memory_order_relaxed);
    hardware_errors_.store(0, std::memory_order_relaxed);
    last_error_.store(hf_gpio_err_t::GPIO_SUCCESS, std::memory_order_relaxed);
    
    return true;
}

bool GpioManager::IsInitialized() const noexcept {
    return is_initialized_.load(std::memory_order_acquire);
}

hf_gpio_err_t GpioManager::GetSystemDiagnostics(GpioSystemDiagnostics& diagnostics) const noexcept {
    // Quick check without lock
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    }
    
    // Fill diagnostics with atomic values (completely lock-free)
    uint32_t total_ops = total_operations_.load(std::memory_order_relaxed);
    uint32_t failed_ops = failed_operations_.load(std::memory_order_relaxed);
    uint32_t comm_errors = communication_errors_.load(std::memory_order_relaxed);
    uint32_t hw_errors = hardware_errors_.load(std::memory_order_relaxed);
    
    diagnostics.total_operations = total_ops;
    diagnostics.successful_operations = successful_operations_.load(std::memory_order_relaxed);
    diagnostics.failed_operations = failed_ops;
    diagnostics.communication_errors = comm_errors;
    diagnostics.hardware_errors = hw_errors;
    diagnostics.last_error = last_error_.load(std::memory_order_relaxed);
    
    // Calculate uptime
    uint64_t start_time = system_start_time_.load(std::memory_order_relaxed);
    if (start_time > 0) {
        // In a real implementation, you'd get current time and calculate difference
        diagnostics.system_uptime_ms = 0; // Placeholder
    } else {
        diagnostics.system_uptime_ms = 0;
    }
    
    // Single scoped lock for registry access only
    {
        MutexLockGuard registry_lock(registry_mutex_);
        diagnostics.total_pins_registered = static_cast<uint32_t>(gpio_registry_.size());
        
        // Initialize arrays
        std::fill(std::begin(diagnostics.pins_by_chip), std::end(diagnostics.pins_by_chip), 0);
        std::fill(std::begin(diagnostics.pins_by_category), std::end(diagnostics.pins_by_category), 0);
        
        // Count pins by chip type (simplified counting)
        for (const auto& [name, gpio] : gpio_registry_) {
            if (gpio) {
                diagnostics.pins_by_chip[0]++; // ESP32_INTERNAL (simplified)
            }
        }
    }
    
    // Determine overall system health
    diagnostics.system_healthy = (failed_ops == 0) && (comm_errors == 0) && (hw_errors == 0);
    
    return hf_gpio_err_t::GPIO_SUCCESS;
}

//==============================================================================
// GPIO REGISTRATION AND MANAGEMENT
//==============================================================================

hf_gpio_err_t GpioManager::RegisterGpio(std::string_view name, std::shared_ptr<BaseGpio> gpio) noexcept {
    hf_gpio_err_t validation_result = ValidatePinName(name);
    if (validation_result != hf_gpio_err_t::GPIO_SUCCESS) {
        UpdateLastError(validation_result);
        UpdateStatistics(false);
        return validation_result;
    }
    
    MutexLockGuard registry_lock(registry_mutex_);
    
    // Check for duplicate registration
    if (gpio_registry_.find(name) != gpio_registry_.end()) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_PIN_ALREADY_REGISTERED);
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_ALREADY_REGISTERED;
    }
    
    // Register the GPIO
    gpio_registry_[name] = std::move(gpio);
    UpdateStatistics(true);
    return hf_gpio_err_t::GPIO_SUCCESS;
}

std::shared_ptr<BaseGpio> GpioManager::Get(std::string_view name) noexcept {
    std::shared_ptr<BaseGpio> result;
    
    {
        MutexLockGuard registry_lock(registry_mutex_);
        auto it = gpio_registry_.find(name);
        if (it != gpio_registry_.end()) {
            result = it->second;
        }
    }
    
    // Update statistics outside the lock (atomic operations)
    UpdateStatistics(result != nullptr);
    return result;
}

bool GpioManager::Contains(std::string_view name) const noexcept {
    MutexLockGuard registry_lock(registry_mutex_);
    return gpio_registry_.find(name) != gpio_registry_.end();
}

size_t GpioManager::Size() const noexcept {
    MutexLockGuard registry_lock(registry_mutex_);
    return gpio_registry_.size();
}

void GpioManager::LogAllRegisteredGpios() const noexcept {
    MutexLockGuard registry_lock(registry_mutex_);
    
    // In a real implementation, you'd use your logging system
    // For now, we'll just update statistics
    for (const auto& [name, gpio] : gpio_registry_) {
        // Log each registered GPIO
        (void)name; // Suppress unused variable warning
        (void)gpio; // Suppress unused variable warning
    }
}

//==============================================================================
// BASIC GPIO OPERATIONS (Complete BaseGpio Coverage)
//==============================================================================

hf_gpio_err_t GpioManager::Set(std::string_view name, bool value) noexcept {
    // Get GPIO with minimal lock duration
    std::shared_ptr<BaseGpio> gpio;
    {
        MutexLockGuard registry_lock(registry_mutex_);
        auto it = gpio_registry_.find(name);
        if (it != gpio_registry_.end()) {
            gpio = it->second;
        }
    }
    
    if (!gpio) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND);
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    // Perform operation without locks
    hf_gpio_err_t result = value ? gpio->SetActive() : gpio->SetInactive();
    
    // Update statistics (atomic operations, no locks needed)
    UpdateLastError(result);
    UpdateStatistics(result == hf_gpio_err_t::GPIO_SUCCESS);
    return result;
}

hf_gpio_err_t GpioManager::SetActive(std::string_view name) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND);
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = gpio->SetActive();
    UpdateLastError(result);
    UpdateStatistics(result == hf_gpio_err_t::GPIO_SUCCESS);
    return result;
}

hf_gpio_err_t GpioManager::SetInactive(std::string_view name) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = gpio->SetInactive();
    UpdateStatistics(result == hf_gpio_err_t::GPIO_SUCCESS);
    return result;
}

hf_gpio_err_t GpioManager::Read(std::string_view name, bool& state) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    bool is_active;
    hf_gpio_err_t result = gpio->IsActive(is_active);
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        state = is_active;
        UpdateStatistics(true);
        return hf_gpio_err_t::GPIO_SUCCESS;
    }
    
    UpdateStatistics(false);
    return result;
}

hf_gpio_err_t GpioManager::Toggle(std::string_view name) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = gpio->Toggle();
    UpdateStatistics(result == hf_gpio_err_t::GPIO_SUCCESS);
    return result;
}

hf_gpio_err_t GpioManager::IsActive(std::string_view name, bool& active) noexcept {
    return Read(name, active);
}

//==============================================================================
// PIN CONFIGURATION (Complete BaseGpio Coverage)
//==============================================================================

hf_gpio_err_t GpioManager::SetDirection(std::string_view name, hf_gpio_direction_t direction) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = gpio->SetDirection(direction);
    bool success = (result == hf_gpio_err_t::GPIO_SUCCESS);
    UpdateStatistics(success);
    return result;
}

hf_gpio_err_t GpioManager::SetPullMode(std::string_view name, hf_gpio_pull_mode_t pull_mode) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = gpio->SetPullMode(pull_mode);
    bool success = (result == hf_gpio_err_t::GPIO_SUCCESS);
    UpdateStatistics(success);
    return result;
}

hf_gpio_err_t GpioManager::SetOutputMode(std::string_view name, hf_gpio_output_mode_t output_mode) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = gpio->SetOutputMode(output_mode);
    bool success = (result == hf_gpio_err_t::GPIO_SUCCESS);
    UpdateStatistics(success);
    return result;
}

hf_gpio_err_t GpioManager::GetDirection(std::string_view name, hf_gpio_direction_t& direction) const noexcept {
    MutexLockGuard registry_lock(registry_mutex_);
    
    auto it = gpio_registry_.find(name);
    if (it == gpio_registry_.end()) {
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    direction = it->second->GetDirection();
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t GpioManager::GetPullMode(std::string_view name, hf_gpio_pull_mode_t& pull_mode) const noexcept {
    MutexLockGuard registry_lock(registry_mutex_);
    
    auto it = gpio_registry_.find(name);
    if (it == gpio_registry_.end()) {
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    pull_mode = it->second->GetPullMode();
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t GpioManager::GetOutputMode(std::string_view name, hf_gpio_output_mode_t& output_mode) const noexcept {
    MutexLockGuard registry_lock(registry_mutex_);
    
    auto it = gpio_registry_.find(name);
    if (it == gpio_registry_.end()) {
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    output_mode = it->second->GetOutputMode();
    return hf_gpio_err_t::GPIO_SUCCESS;
}

//==============================================================================
// INTERRUPT SUPPORT (Complete BaseGpio Coverage)
//==============================================================================

hf_gpio_err_t GpioManager::ConfigureInterrupt(std::string_view name,
                                             hf_gpio_interrupt_trigger_t trigger,
                                             InterruptCallback callback,
                                             void* user_data) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = gpio->ConfigureInterrupt(trigger, callback, user_data);
    bool success = (result == hf_gpio_err_t::GPIO_SUCCESS);
    UpdateStatistics(success);
    return result;
}

hf_gpio_err_t GpioManager::EnableInterrupt(std::string_view name) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = gpio->EnableInterrupt();
    bool success = (result == hf_gpio_err_t::GPIO_SUCCESS);
    UpdateStatistics(success);
    return result;
}

hf_gpio_err_t GpioManager::DisableInterrupt(std::string_view name) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = gpio->DisableInterrupt();
    bool success = (result == hf_gpio_err_t::GPIO_SUCCESS);
    UpdateStatistics(success);
    return result;
}

bool GpioManager::SupportsInterrupts(std::string_view name) const noexcept {
    MutexLockGuard registry_lock(registry_mutex_);
    
    auto it = gpio_registry_.find(name);
    if (it == gpio_registry_.end()) {
        return false;
    }
    
    return (it->second->SupportsInterrupts() == hf_gpio_err_t::GPIO_SUCCESS);
}

//==============================================================================
// STATISTICS AND DIAGNOSTICS (Complete BaseGpio Coverage)
//==============================================================================

hf_gpio_err_t GpioManager::GetStatistics(std::string_view name, hf_gpio_statistics_t& statistics) const noexcept {
    MutexLockGuard registry_lock(registry_mutex_);
    
    auto it = gpio_registry_.find(name);
    if (it == gpio_registry_.end()) {
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = it->second->GetStatistics(statistics);
    return result;
}

hf_gpio_err_t GpioManager::ResetStatistics(std::string_view name) noexcept {
    auto gpio = Get(name);
    if (!gpio) {
        UpdateStatistics(false);
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    }
    
    hf_gpio_err_t result = gpio->ResetStatistics();
    UpdateStatistics(result == hf_gpio_err_t::GPIO_SUCCESS);
    return result;
}

//==============================================================================
// BATCH OPERATIONS
//==============================================================================

GpioBatchResult GpioManager::BatchWrite(const GpioBatchOperation& operation) noexcept {
    GpioBatchResult result;
    result.pin_names = operation.pin_names;
    result.states = operation.states;
    result.results.reserve(operation.pin_names.size());
    
    bool all_successful = true;
    
    for (size_t i = 0; i < operation.pin_names.size(); ++i) {
        if (i < operation.states.size()) {
            auto err = Set(operation.pin_names[i], operation.states[i]);
            result.results.push_back(err);
            if (err != hf_gpio_err_t::GPIO_SUCCESS) {
                all_successful = false;
            }
        } else {
            result.results.push_back(hf_gpio_err_t::GPIO_ERR_FAILURE);
            all_successful = false;
        }
    }
    
    result.overall_result = all_successful ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
    return result;
}

GpioBatchResult GpioManager::BatchRead(const std::vector<std::string_view>& pin_names) noexcept {
    GpioBatchResult result;
    result.pin_names = pin_names;
    result.states.reserve(pin_names.size());
    result.results.reserve(pin_names.size());
    
    bool all_successful = true;
    
    for (const auto& name : pin_names) {
        bool state;
        auto err = Read(name, state);
        result.states.push_back(state);
        result.results.push_back(err);
        if (err != hf_gpio_err_t::GPIO_SUCCESS) {
            all_successful = false;
        }
    }
    
    result.overall_result = all_successful ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
    return result;
}

GpioBatchResult GpioManager::SetMultipleActive(const std::vector<std::string_view>& pin_names) noexcept {
    GpioBatchResult result;
    result.pin_names = pin_names;
    result.results.reserve(pin_names.size());
    
    bool all_successful = true;
    
    for (const auto& name : pin_names) {
        auto err = SetActive(name);
        result.results.push_back(err);
        if (err != hf_gpio_err_t::GPIO_SUCCESS) {
            all_successful = false;
        }
    }
    
    result.overall_result = all_successful ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
    return result;
}

GpioBatchResult GpioManager::SetMultipleInactive(const std::vector<std::string_view>& pin_names) noexcept {
    GpioBatchResult result;
    result.pin_names = pin_names;
    result.results.reserve(pin_names.size());
    
    bool all_successful = true;
    
    for (const auto& name : pin_names) {
        auto err = SetInactive(name);
        result.results.push_back(err);
        if (err != hf_gpio_err_t::GPIO_SUCCESS) {
            all_successful = false;
        }
    }
    
    result.overall_result = all_successful ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
    return result;
}

//==============================================================================
// SYSTEM INFORMATION
//==============================================================================GetSystemUptimeMs

hf_gpio_err_t GpioManager::ResetAllPins() noexcept {
    MutexLockGuard registry_lock(registry_mutex_);
    
    bool all_successful = true;
    
    for (auto& [name, gpio] : gpio_registry_) {
        // Only reset output pins
        if (gpio->IsOutput()) {
            hf_gpio_err_t result = gpio->SetInactive();
            if (result != hf_gpio_err_t::GPIO_SUCCESS) {
                all_successful = false;
            }
        }
    }
    
    UpdateStatistics(all_successful);
    return all_successful ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

//==============================================================================
// PRIVATE IMPLEMENTATION
//==============================================================================

bool GpioManager::Initialize() noexcept {
    // Set system start time
    system_start_time_.store(0, std::memory_order_release); // In real implementation, get current time
    
    // Get CommChannelsManager instance
    auto& comm_manager = CommChannelsManager::GetInstance();
    if (!comm_manager.IsInitialized()) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED);
        return false;
    }
    
    // Get MotorController instance for TMC9660 handler access
    motor_controller_ = &MotorController::GetInstance();
    if (!motor_controller_->EnsureInitialized()) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED);
        // Don't fail initialization, just log warning
    }
    
    // Initialize GPIOs from platform mapping
    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE; ++i) {
        const auto& mapping = HF_GPIO_MAPPING[i];
        
        // Skip CORE and COMM pins - they should not be registered as GPIOs
        if (!should_register_as_gpio(static_cast<HfPinCategory>(mapping.category))) {
            continue;
        }
        
        // Filter pins based on current hardware configuration
        // Only register pins that are actually available on the current board
        bool should_register = false;
        
        switch (static_cast<HfGpioChipType>(mapping.chip_type)) {
            case HfGpioChipType::ESP32_INTERNAL:
                // ESP32 internal: only unit 0, bank 0
                should_register = (mapping.chip_unit == 0 && mapping.gpio_bank == 0);
                break;
                
            case HfGpioChipType::PCAL95555_EXPANDER:
                // PCAL95555: only unit 0, bank 0 (single expander on board)
                should_register = (mapping.chip_unit == 0 && mapping.gpio_bank == 0);
                break;
                
            case HfGpioChipType::TMC9660_CONTROLLER:
                // TMC9660: only unit 0, bank 0 (single controller on board)
                should_register = (mapping.chip_unit == 0 && mapping.gpio_bank == 0);
                break;
                
            default:
                UpdateLastError(hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER);
                continue;
        }
        
        if (!should_register) {
            Logger::GetInstance().Info("GpioManager", "Skipping GPIO %.*s (chip_unit=%d, bank=%d) - not available on current hardware", 
                               static_cast<int>(mapping.name.length()), mapping.name.data(), 
                               mapping.chip_unit, mapping.gpio_bank);
            continue;
        }
        
        // Create GPIO based on chip type
        std::shared_ptr<BaseGpio> gpio;
        
        switch (static_cast<HfGpioChipType>(mapping.chip_type)) {
            case HfGpioChipType::ESP32_INTERNAL:
                gpio = CreateEsp32GpioPin(mapping.physical_pin, mapping.is_inverted, 
                                        mapping.has_pull, mapping.pull_is_up, mapping.is_push_pull, 
                                        mapping.max_current_ma);
                break;
                
            case HfGpioChipType::PCAL95555_EXPANDER:
                gpio = CreatePcal95555GpioPin(mapping.physical_pin, mapping.chip_unit, mapping.is_inverted, 
                                            mapping.has_pull, mapping.pull_is_up, mapping.is_push_pull, 
                                            mapping.max_current_ma);
                break;
                
            case HfGpioChipType::TMC9660_CONTROLLER:
                gpio = CreateTmc9660GpioPin(mapping.physical_pin, mapping.chip_unit, mapping.is_inverted, 
                                          mapping.has_pull, mapping.pull_is_up, mapping.is_push_pull, 
                                          mapping.max_current_ma);
                break;
                
            default:
                UpdateLastError(hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER);
                continue;
        }
        
        if (gpio) {
            // Register the GPIO with its string name
            RegisterGpio(mapping.name, std::move(gpio));
            Logger::GetInstance().Info("GpioManager", "Registered GPIO %.*s (chip_unit=%d, bank=%d)", 
                               static_cast<int>(mapping.name.length()), mapping.name.data(), 
                               mapping.chip_unit, mapping.gpio_bank);
        } else {
            UpdateLastError(hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT);
            Logger::GetInstance().Error("GpioManager", "Failed to create GPIO %.*s (chip_unit=%d, bank=%d)", 
                                static_cast<int>(mapping.name.length()), mapping.name.data(), 
                                mapping.chip_unit, mapping.gpio_bank);
        }
    }
    
    is_initialized_.store(true, std::memory_order_release);
    return true;
}

bool GpioManager::EnsurePcal95555Handler() noexcept {
    MutexLockGuard lock(pcal_handler_mutex_);
    
    if (pcal95555_handler_) {
        return true;
    }
    
    // Get CommChannelsManager instance
    auto& comm_manager = CommChannelsManager::GetInstance();
    if (!comm_manager.IsInitialized()) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED);
        return false;
    }
    
    // Get I2C device for PCAL95555
    auto i2c_device = comm_manager.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c_device) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT);
        return false;
    }
    
    // Create PCAL95555 handler (lazy-initialized via EnsureInitialized)
    pcal95555_handler_ = std::make_unique<Pcal95555Handler>(*i2c_device);
    if (!pcal95555_handler_->EnsureInitialized()) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT);
        pcal95555_handler_.reset();
        return false;
    }
    
    return true;
}

Tmc9660Handler* GpioManager::GetTmc9660Handler(uint8_t device_index) noexcept {
    // Get MotorController instance if not already cached
    if (!motor_controller_) {
        motor_controller_ = &MotorController::GetInstance();
    }
    
    // Ensure MotorController is initialized
    if (!motor_controller_->EnsureInitialized()) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED);
        return nullptr;
    }
    
    // Get handler from MotorController
    return motor_controller_->handler(device_index);
}

std::shared_ptr<BaseGpio> GpioManager::CreateEsp32GpioPin(hf_u8_t pin_id, bool is_inverted, 
                                                         bool has_pull, bool pull_is_up, bool is_push_pull,
                                                         hf_u32_t max_current_ma) noexcept {
    // Create ESP32 GPIO with proper configuration
    auto gpio = std::make_shared<EspGpio>(static_cast<hf_pin_num_t>(pin_id));
    
    // Configure pull mode based on primitive values
    if (has_pull) {
        hf_gpio_pull_mode_t pull_mode = pull_is_up ? 
            hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP : 
            hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN;
        gpio->SetPullMode(pull_mode);
    } else {
        gpio->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING);
    }
    
    // Configure output mode based on primitive value
    hf_gpio_output_mode_t output_mode = is_push_pull ? 
        hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL : 
        hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_OPEN_DRAIN;
    gpio->SetOutputMode(output_mode);
    
    // Set active state based on inversion
    gpio->SetActiveState(is_inverted ? hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW 
                                    : hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH);
    
    // Initialize the GPIO
            if (!gpio->Initialize()) {
            UpdateLastError(hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT);
            return nullptr;
        }
    
    return gpio;
}

std::shared_ptr<BaseGpio> GpioManager::CreatePcal95555GpioPin(hf_u8_t pin_id, hf_u8_t unit_number, bool is_inverted, 
                                                             bool has_pull, bool pull_is_up, bool is_push_pull,
                                                             hf_u32_t max_current_ma) noexcept {
    if (!EnsurePcal95555Handler()) {
        return nullptr;
    }
    
    // Determine pull mode from primitive values
    hf_gpio_pull_mode_t pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
    if (has_pull) {
        pull_mode = pull_is_up ? hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP 
                               : hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN;
    }
    
    // Determine output mode from primitive value
    hf_gpio_output_mode_t output_mode_val = is_push_pull 
        ? hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL 
        : hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_OPEN_DRAIN;
    
    // Determine active state from inversion
    hf_gpio_active_state_t active_state = is_inverted 
        ? hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW 
        : hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH;
    
    // Use PCAL95555 handler's factory method with full configuration
    // Note: unit_number is handled at handler construction level, not per-pin
    auto gpio = pcal95555_handler_->CreateGpioPin(
        static_cast<hf_pin_num_t>(pin_id),
        hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,  // Default to input; caller configures later
        active_state,
        output_mode_val,
        pull_mode);
    
    if (!gpio) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT);
        return nullptr;
    }
    
    return gpio;
}

std::shared_ptr<BaseGpio> GpioManager::CreateTmc9660GpioPin(hf_u8_t pin_id, hf_u8_t device_index, bool is_inverted, 
                                                           bool has_pull, bool pull_is_up, bool is_push_pull,
                                                           hf_u32_t max_current_ma) noexcept {
    // Get TMC9660 handler from MotorController using specified device index
    Tmc9660Handler* handler = GetTmc9660Handler(device_index);
    if (!handler) {
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT);
        return nullptr;
    }
    
    // Validate that pin_id maps to a supported TMC9660 GPIO (17 or 18)
    if (pin_id != 17 && pin_id != 18) {
        Logger::GetInstance().Error("GpioManager", "TMC9660 only supports GPIO17 and GPIO18, got pin_id=%u", pin_id);
        UpdateLastError(hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER);
        return nullptr;
    }
    
    // Get a reference to the handler's Gpio wrapper (BaseGpio subclass).
    // The Gpio instance is owned by the Tmc9660Handler and its lifetime is
    // guaranteed by MotorController (singleton). We create a non-owning
    // shared_ptr via the aliasing constructor so GpioManager can store it
    // uniformly without taking ownership.
    BaseGpio& gpio_ref = handler->gpio(pin_id);
    
    // Aliasing shared_ptr: shares ownership with an empty control block (no-op
    // deleter) while pointing to the handler-owned Gpio object.
    std::shared_ptr<BaseGpio> gpio(std::shared_ptr<BaseGpio>{}, &gpio_ref);
    
    // Apply configuration from the pin mapping table
    if (is_inverted) {
        gpio->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW);
    }
    
    Logger::GetInstance().Info("GpioManager", "TMC9660 GPIO%u created (device_index=%u)", pin_id, device_index);
    return gpio;
}

void GpioManager::UpdateStatistics(bool success) noexcept {
    total_operations_.fetch_add(1, std::memory_order_relaxed);
    
    if (success) {
        successful_operations_.fetch_add(1, std::memory_order_relaxed);
    } else {
        failed_operations_.fetch_add(1, std::memory_order_relaxed);
    }
}

void GpioManager::UpdateLastError(hf_gpio_err_t error_code) noexcept {
    last_error_.store(error_code, std::memory_order_relaxed);
    
    // Update specific error counters (atomic operations, no locks needed)
    switch (error_code) {
        case hf_gpio_err_t::GPIO_ERR_COMMUNICATION_FAILURE:
            communication_errors_.fetch_add(1, std::memory_order_relaxed);
            break;
        case hf_gpio_err_t::GPIO_ERR_HARDWARE_FAULT:
            hardware_errors_.fetch_add(1, std::memory_order_relaxed);
            break;
        default:
            break;
    }
}

hf_gpio_err_t GpioManager::ValidatePinName(std::string_view name) noexcept {
    // Check for empty name
    if (name.empty()) {
        return hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER;
    }
    
    // Check maximum length (conserve memory with reasonable limit)
    static constexpr size_t MAX_PIN_NAME_LENGTH = 32;
    if (name.length() > MAX_PIN_NAME_LENGTH) {
        return hf_gpio_err_t::GPIO_ERR_OUT_OF_MEMORY;  // Name too long
    }
    
    // Check for reserved prefixes using compile-time array
    static constexpr std::array<std::string_view, 4> RESERVED_PREFIXES = {{
        "CORE_", "COMM_", "SYS_", "INTERNAL_"
    }};
    
    for (const auto& prefix : RESERVED_PREFIXES) {
        if (name.length() >= prefix.length() && 
            name.substr(0, prefix.length()) == prefix) {
            return hf_gpio_err_t::GPIO_ERR_PERMISSION_DENIED;  // Reserved prefix
        }
    }
    
    // Check if name starts with a digit (invalid for most naming conventions)
    if (std::isdigit(static_cast<unsigned char>(name[0]))) {
        return hf_gpio_err_t::GPIO_ERR_INVALID_ARG;  // Invalid format
    }
    
    // Check for valid characters (alphanumeric, underscore, hyphen only)
    for (char c : name) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-') {
            return hf_gpio_err_t::GPIO_ERR_INVALID_ARG;  // Invalid characters
        }
    }
    
    return hf_gpio_err_t::GPIO_SUCCESS;
}

void GpioManager::DumpStatistics() const noexcept {
    static constexpr const char* TAG = "GpioManager";
    
    Logger::GetInstance().Info(TAG, "=== GPIO MANAGER STATISTICS ===");
    
    MutexLockGuard lock(mutex_);
    
    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", is_initialized_.load() ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Total Pins Registered: %d", static_cast<int>(gpio_registry_.size()));
    Logger::GetInstance().Info(TAG, "  Active Pins: %d", GetActivePinCount());
    
    // Operation Statistics
    uint32_t total_ops = total_operations_.load();
    uint32_t success_ops = successful_operations_.load();
    uint32_t failed_ops = failed_operations_.load();
    
    Logger::GetInstance().Info(TAG, "Operation Statistics:");
    Logger::GetInstance().Info(TAG, "  Total Operations: %d", total_ops);
    Logger::GetInstance().Info(TAG, "  Successful Operations: %d", success_ops);
    Logger::GetInstance().Info(TAG, "  Failed Operations: %d", failed_ops);
    
    if (total_ops > 0) {
        float success_rate = (float)success_ops / total_ops * 100.0f;
        Logger::GetInstance().Info(TAG, "  Success Rate: %.2f%%", success_rate);
    }
    
    // Error Statistics
    Logger::GetInstance().Info(TAG, "Error Statistics:");
    Logger::GetInstance().Info(TAG, "  Communication Errors: %d", communication_errors_.load());
    Logger::GetInstance().Info(TAG, "  Hardware Errors: %d", hardware_errors_.load());
    Logger::GetInstance().Info(TAG, "  Last Error: %s", HfGpioErrToString(last_error_.load()).data());
    
    // Pin Statistics Summary
    Logger::GetInstance().Info(TAG, "Pin Statistics Summary:");
    std::vector<std::pair<std::string, hf_gpio_statistics_t>> pin_stats;
    
    for (const auto& pair : gpio_registry_) {
        if (pair.second) {
            hf_gpio_statistics_t stats;
            if (pair.second->GetStatistics(stats) == hf_gpio_err_t::GPIO_SUCCESS) {
                pin_stats.emplace_back(std::string(pair.first), stats);
            }
        }
    }
    
    // Sort by total operations (highest activity first)
    std::sort(pin_stats.begin(), pin_stats.end(), 
        [](const auto& a, const auto& b) {
            return a.second.totalOperations > b.second.totalOperations;
        });
    
    int pins_to_show = std::min(5, static_cast<int>(pin_stats.size()));
    if (pins_to_show > 0) {
        Logger::GetInstance().Info(TAG, "  Top %d Most Active Pins:", pins_to_show);
        for (int i = 0; i < pins_to_show; ++i) {
            const auto& pin = pin_stats[i];
            Logger::GetInstance().Info(TAG, "    %s: ops=%d success=%d failed=%d state_changes=%d", 
                pin.first.c_str(), 
                static_cast<int>(pin.second.totalOperations),
                static_cast<int>(pin.second.successfulOperations),
                static_cast<int>(pin.second.failedOperations),
                static_cast<int>(pin.second.stateChanges));
        }
    } else {
        Logger::GetInstance().Info(TAG, "  No pin statistics available");
    }
    
    Logger::GetInstance().Info(TAG, "=== END GPIO MANAGER STATISTICS ===");
}

int GpioManager::GetActivePinCount() const noexcept {
    int count = 0;
    for (const auto& pair : gpio_registry_) {
        if (pair.second) {
            count++;
        }
    }
    return count;
} 