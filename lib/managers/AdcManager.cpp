/**
 * @file AdcManager.cpp
 * @brief Advanced ADC management system implementation for the HardFOC platform.
 * 
 * @details This file contains the complete implementation of the AdcManager class
 *          that provides a comprehensive ADC management system integrating with
 *          the platform mapping system to automatically manage ADC channels from
 *          multiple hardware sources (ESP32 internal ADC, TMC9660 ADC).
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 * 
 * Implementation Features:
 * - Platform mapping integration for automatic channel discovery
 * - Multi-chip ADC management with proper ownership
 * - Thread-safe operations with comprehensive error handling
 * - Complete BaseAdc function coverage through string-based routing
 * - Advanced diagnostics and health monitoring
 * - Batch operations for performance optimization
 * - Hardware resource validation and conflict detection
 * - Smart channel categorization and mapping
 * 
 * Architecture:
 * - Uses string_view for channel identification (extensible)
 * - Integrates with platform mapping for hardware mapping
 * - Supports all HardwareChip types defined in platform mapping
 * - Provides unified BaseAdc interface for all channel operations
 * - Handler-based ADC creation for proper ownership
 * - Routes all BaseAdc functions through string-based API
 * 
 * @note This implementation is thread-safe and designed for concurrent access.
 * @note All channel operations use string_view identifiers for maximum flexibility.
 */

#include "AdcManager.h"
#include "MotorController.h"
#include "RtosMutex.h"

// Logger for unified logging
#include "handlers/logger/Logger.h"

#include <algorithm>
#include <cstring>
#include <sstream>
#include <iomanip>

//==============================================================================
// STATIC MEMBER INITIALIZATION
//==============================================================================

// Static instance for singleton pattern
static AdcManager* g_adc_manager_instance = nullptr;

//==============================================================================
// SINGLETON AND LIFECYCLE IMPLEMENTATION
//==============================================================================

AdcManager& AdcManager::GetInstance() noexcept {
    if (g_adc_manager_instance == nullptr) {
        static AdcManager instance;
        g_adc_manager_instance = &instance;
    }
    return *g_adc_manager_instance;
}

bool AdcManager::EnsureInitialized() noexcept {
    // Quick check without lock first
    if (is_initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    // Only lock if we need to initialize
    std::lock_guard<RtosMutex> lock(mutex_);
    
    // Double-check after acquiring lock
    if (is_initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    return Initialize() == hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::Shutdown() noexcept {
    // Quick check without lock first
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_SUCCESS;
    }
    
    // Acquire main mutex only for state change
    {
        std::lock_guard<RtosMutex> lock(mutex_);
        
        // Double-check after acquiring lock
        if (!is_initialized_.load(std::memory_order_acquire)) {
            return hf_adc_err_t::ADC_SUCCESS;
        }
        
        // Mark as not initialized early to prevent new operations
        is_initialized_.store(false, std::memory_order_release);
    }
    
    // Clear resources with separate locks (no need for main mutex anymore)
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        adc_registry_.clear();
    }
    
    {
        std::lock_guard<RtosMutex> esp32_lock(esp32_adc_mutex_);
        for (auto& h : esp32_adc_handlers_) h.reset();
    }
    
    // Reset statistics (atomic operations, no locks needed)
    total_operations_.store(0, std::memory_order_relaxed);
    successful_operations_.store(0, std::memory_order_relaxed);
    failed_operations_.store(0, std::memory_order_relaxed);
    communication_errors_.store(0, std::memory_order_relaxed);
    hardware_errors_.store(0, std::memory_order_relaxed);
    last_error_.store(hf_adc_err_t::ADC_SUCCESS, std::memory_order_relaxed);
    
    return hf_adc_err_t::ADC_SUCCESS;
}

bool AdcManager::IsInitialized() const noexcept {
    return is_initialized_.load();
}

hf_adc_err_t AdcManager::GetSystemDiagnostics(AdcSystemDiagnostics& diagnostics) const noexcept {
    // Quick check without lock
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Fill diagnostics with atomic values (completely lock-free)
    uint64_t start_time = system_start_time_.load(std::memory_order_relaxed);
    uint64_t now = RtosTime::GetCurrentTimeUs() / 1000;
    uint32_t total_ops = total_operations_.load(std::memory_order_relaxed);
    uint32_t failed_ops = failed_operations_.load(std::memory_order_relaxed);
    
    diagnostics.system_healthy = (total_ops == 0) ? true : (failed_ops < total_ops * 0.1);
    diagnostics.total_operations = total_ops;
    diagnostics.successful_operations = successful_operations_.load(std::memory_order_relaxed);
    diagnostics.failed_operations = failed_ops;
    diagnostics.communication_errors = communication_errors_.load(std::memory_order_relaxed);
    diagnostics.hardware_errors = hardware_errors_.load(std::memory_order_relaxed);
    diagnostics.system_uptime_ms = now - start_time;
    diagnostics.last_error = last_error_.load(std::memory_order_relaxed);
    
    // Single scoped lock for registry access only
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        
        uint32_t esp32_count = 0;
        uint32_t tmc9660_count = 0;
        uint32_t total_count = 0;
        
        for (const auto& [name, channel_info] : adc_registry_) {
            if (channel_info && channel_info->is_registered) {
                total_count++;
                if (channel_info->hardware_chip == HfAdcChipType::ESP32_INTERNAL) {
                    esp32_count++;
                } else if (channel_info->hardware_chip == HfAdcChipType::TMC9660_CONTROLLER) {
                    tmc9660_count++;
                }
            }
        }
        
        diagnostics.total_channels_registered = total_count;
        diagnostics.channels_by_chip[0] = esp32_count;
        diagnostics.channels_by_chip[1] = tmc9660_count;
    }
    
    return hf_adc_err_t::ADC_SUCCESS;
}

//==============================================================================
// CHANNEL REGISTRATION AND MANAGEMENT
//==============================================================================

hf_adc_err_t AdcManager::RegisterChannel(std::string_view name, std::unique_ptr<BaseAdc> adc) noexcept {
    if (!is_initialized_.load()) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Validate channel name
    hf_adc_err_t validation_result = ValidateChannelName(name);
    if (validation_result != hf_adc_err_t::ADC_SUCCESS) {
        return validation_result;
    }
    
    // Validate ADC driver
    if (!adc) {
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
    
    // Check if channel already exists
    if (adc_registry_.find(name) != adc_registry_.end()) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_ALREADY_REGISTERED;
    }
    
    // Create channel info with default values
    // Note: We'll need to determine the actual hardware chip and channel ID
    // from the platform mapping or ADC driver itself
    auto channel_info = std::make_unique<AdcChannelInfo>(
        name, std::move(adc), static_cast<HfFunctionalAdcChannel>(0), 
        HfAdcChipType::ESP32_INTERNAL, 0, 3.3f, 12, 3300, 1.0f
    );
    
    // Register the channel
    adc_registry_[name] = std::move(channel_info);
    
    return hf_adc_err_t::ADC_SUCCESS;
}

BaseAdc* AdcManager::Get(std::string_view name) noexcept {
    // Quick check without lock
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return nullptr;
    }
    
    std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
    
    auto it = adc_registry_.find(name);
    if (it != adc_registry_.end() && it->second && it->second->is_registered) {
        return it->second->adc_driver.get();
    }
    
    return nullptr;
}

bool AdcManager::Contains(std::string_view name) const noexcept {
    if (!is_initialized_.load()) {
        return false;
    }
    
    std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
    
    auto it = adc_registry_.find(name);
    return (it != adc_registry_.end() && it->second && it->second->is_registered);
}

size_t AdcManager::Size() const noexcept {
    if (!is_initialized_.load()) {
        return 0;
    }
    
    std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
    
    size_t count = 0;
    for (const auto& [name, channel_info] : adc_registry_) {
        if (channel_info && channel_info->is_registered) {
            count++;
        }
    }
    
    return count;
}

void AdcManager::LogAllRegisteredChannels() const noexcept {
    if (!is_initialized_.load()) {
                 Logger::GetInstance().Info("AdcManager", "System not initialized");
        return;
    }
    
    std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
    
    Logger::GetInstance().Info("AdcManager", "Registered Channels (%zu total):", adc_registry_.size());
    
    for (const auto& [name, channel_info] : adc_registry_) {
        if (channel_info && channel_info->is_registered) {
            Logger::GetInstance().Info("AdcManager", "  - %.*s (Chip: %d, Channel: %d, Access: %u, Errors: %u)",
                               static_cast<int>(name.length()), name.data(),
                               static_cast<int>(channel_info->hardware_chip),
                               channel_info->hardware_channel_id,
                               channel_info->access_count,
                               channel_info->error_count);
        }
    }
}

//==============================================================================
// BASIC READING OPERATIONS (Complete BaseAdc Coverage)
//==============================================================================

hf_adc_err_t AdcManager::ReadChannelV(std::string_view name, float& voltage,
                                     hf_u8_t numOfSamplesToAvg, hf_time_t timeBetweenSamples) noexcept {
    // Quick check without lock
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Extract required data in single lock scope
    BaseAdc* adc_driver = nullptr;
    uint8_t hardware_channel_id = 0;
    AdcChannelInfo* channel_info = nullptr;
    uint64_t current_time = RtosTime::GetCurrentTimeUs() / 1000;
    
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        auto it = adc_registry_.find(name);
        if (it != adc_registry_.end() && it->second && it->second->is_registered) {
            channel_info = it->second.get();
            adc_driver = channel_info->adc_driver.get();
            hardware_channel_id = channel_info->hardware_channel_id;
            
            // Update access statistics while we have the lock
            channel_info->access_count++;
            channel_info->last_access_time = current_time;
        }
    }
    
    if (!channel_info || !adc_driver) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND);
        UpdateStatistics(false);
        return hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Perform the read operation (no locks needed for ADC operation)
    hf_adc_err_t result = adc_driver->ReadChannelV(
        hardware_channel_id, voltage, numOfSamplesToAvg, timeBetweenSamples);
    
    // Update statistics (atomic operations, no lock needed)
    UpdateStatistics(result == hf_adc_err_t::ADC_SUCCESS);
    
    if (result != hf_adc_err_t::ADC_SUCCESS) {
        // Single lock for error count update
        {
            std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
            channel_info->error_count++;
        }
        UpdateLastError(result);
    }
    
    return result;
}

hf_adc_err_t AdcManager::ReadChannelCount(std::string_view name, hf_u32_t& value,
                                         hf_u8_t numOfSamplesToAvg, hf_time_t timeBetweenSamples) noexcept {
    if (!is_initialized_.load()) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Find channel info with minimal lock scope
    BaseAdc* adc_driver = nullptr;
    uint8_t hardware_channel_id = 0;
    AdcChannelInfo* channel_info = nullptr;
    
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        auto it = adc_registry_.find(name);
        if (it != adc_registry_.end() && it->second && it->second->is_registered) {
            channel_info = it->second.get();
            adc_driver = channel_info->adc_driver.get();
            hardware_channel_id = channel_info->hardware_channel_id;
        }
    }
    
    if (!channel_info || !adc_driver) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND);
        return hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Update access statistics (quick lock for statistics update)
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        channel_info->access_count++;
        channel_info->last_access_time = RtosTime::GetCurrentTimeUs() / 1000;
    }
    
    // Perform the read operation (no locks needed for ADC operation)
    hf_adc_err_t result = adc_driver->ReadChannelCount(
        hardware_channel_id, value, numOfSamplesToAvg, timeBetweenSamples);
    
    // Update statistics
    UpdateStatistics(result == hf_adc_err_t::ADC_SUCCESS);
    if (result != hf_adc_err_t::ADC_SUCCESS) {
        // Quick lock for error count update
        {
            std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
            channel_info->error_count++;
        }
        UpdateLastError(result);
    }
    
    return result;
}

hf_adc_err_t AdcManager::ReadChannel(std::string_view name, hf_u32_t& raw_value, float& voltage,
                                    hf_u8_t numOfSamplesToAvg, hf_time_t timeBetweenSamples) noexcept {
    if (!is_initialized_.load()) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Find channel info with minimal lock scope
    BaseAdc* adc_driver = nullptr;
    uint8_t hardware_channel_id = 0;
    AdcChannelInfo* channel_info = nullptr;
    
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        auto it = adc_registry_.find(name);
        if (it != adc_registry_.end() && it->second && it->second->is_registered) {
            channel_info = it->second.get();
            adc_driver = channel_info->adc_driver.get();
            hardware_channel_id = channel_info->hardware_channel_id;
        }
    }
    
    if (!channel_info || !adc_driver) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND);
        return hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Update access statistics (quick lock for statistics update)
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        channel_info->access_count++;
        channel_info->last_access_time = RtosTime::GetCurrentTimeUs() / 1000;
    }
    
    // Perform the read operation (no locks needed for ADC operation)
    hf_adc_err_t result = adc_driver->ReadChannel(
        hardware_channel_id, raw_value, voltage, numOfSamplesToAvg, timeBetweenSamples);
    
    // Update statistics
    UpdateStatistics(result == hf_adc_err_t::ADC_SUCCESS);
    if (result != hf_adc_err_t::ADC_SUCCESS) {
        // Quick lock for error count update
        {
            std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
            channel_info->error_count++;
        }
        UpdateLastError(result);
    }
    
    return result;
}

hf_adc_err_t AdcManager::ReadMultipleChannels(const std::string_view* channel_names, hf_u8_t num_channels,
                                             hf_u32_t* raw_values, float* voltages) noexcept {
    if (!is_initialized_.load()) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    if (!channel_names || !raw_values || !voltages || num_channels == 0) {
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    // Read each channel individually
    for (hf_u8_t i = 0; i < num_channels; i++) {
        hf_adc_err_t result = ReadChannel(channel_names[i], raw_values[i], voltages[i]);
        if (result != hf_adc_err_t::ADC_SUCCESS) {
            // Continue reading other channels but return the first error
            return result;
        }
    }
    
    return hf_adc_err_t::ADC_SUCCESS;
}

//==============================================================================
// BATCH OPERATIONS
//==============================================================================

AdcBatchResult AdcManager::BatchRead(const AdcBatchOperation& operation) noexcept {
    AdcBatchResult result;
    
    if (!is_initialized_.load()) {
        result.overall_result = hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
        return result;
    }
    
    if (operation.channel_names.empty()) {
        result.overall_result = hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
        return result;
    }
    
    // Initialize result structure
    result.channel_names = operation.channel_names;
    result.voltages.resize(operation.channel_names.size());
    result.raw_values.resize(operation.channel_names.size());
    result.results.resize(operation.channel_names.size());
    
    // Record start time
    uint64_t start_time = RtosTime::GetCurrentTimeUs() / 1000;
    
    // Read each channel
    bool all_successful = true;
    for (size_t i = 0; i < operation.channel_names.size(); i++) {
        uint8_t samples = operation.use_individual_specs ? 
            (i < operation.samples_per_channel.size() ? operation.samples_per_channel[i] : 1) :
            operation.common_samples;
        
        uint16_t interval = operation.use_individual_specs ?
            (i < operation.intervals_ms.size() ? operation.intervals_ms[i] : 0) :
            operation.common_interval_ms;
        
        result.results[i] = ReadChannel(operation.channel_names[i], 
                                       result.raw_values[i], result.voltages[i], 
                                       samples, interval);
        
        if (result.results[i] != hf_adc_err_t::ADC_SUCCESS) {
            all_successful = false;
        }
    }
    
    // Calculate total time
    uint64_t end_time = RtosTime::GetCurrentTimeUs() / 1000;
    result.total_time_ms = static_cast<uint32_t>(end_time - start_time);
    
    // Set overall result
    result.overall_result = all_successful ? hf_adc_err_t::ADC_SUCCESS : hf_adc_err_t::ADC_ERR_INITIALIZATION_FAILED;
    
    return result;
}

AdcBatchResult AdcManager::BatchRead(const std::vector<std::string_view>& channel_names,
                                    uint8_t samples_per_channel, uint16_t interval_ms) noexcept {
    AdcBatchOperation operation(channel_names, samples_per_channel, interval_ms);
    return BatchRead(operation);
}

AdcBatchResult AdcManager::ReadAllChannels() noexcept {
    if (!is_initialized_.load()) {
        AdcBatchResult result;
        result.overall_result = hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
        return result;
    }
    
    // Get all registered channel names with minimal lock scope
    std::vector<std::string_view> channel_names;
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        channel_names.reserve(adc_registry_.size());
        for (const auto& [name, channel_info] : adc_registry_) {
            if (channel_info && channel_info->is_registered) {
                channel_names.push_back(name);
            }
        }
    }
    
    return BatchRead(channel_names);
}

//==============================================================================
// STATISTICS AND DIAGNOSTICS (Complete BaseAdc Coverage)
//==============================================================================

hf_adc_err_t AdcManager::GetStatistics(std::string_view name, hf_adc_statistics_t& statistics) const noexcept {
    if (!is_initialized_.load()) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Find channel info with minimal lock scope
    BaseAdc* adc_driver = nullptr;
    uint8_t hardware_channel_id = 0;
    
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        auto it = adc_registry_.find(name);
        if (it != adc_registry_.end() && it->second && it->second->is_registered) {
            adc_driver = it->second->adc_driver.get();
            hardware_channel_id = it->second->hardware_channel_id;
        }
    }
    
    if (!adc_driver) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Get statistics from the ADC driver (no locks needed)
    return adc_driver->GetStatistics(statistics);
}

hf_adc_err_t AdcManager::ResetStatistics(std::string_view name) noexcept {
    if (!is_initialized_.load()) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Find channel info with minimal lock scope
    BaseAdc* adc_driver = nullptr;
    uint8_t hardware_channel_id = 0;
    AdcChannelInfo* channel_info = nullptr;
    
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        auto it = adc_registry_.find(name);
        if (it != adc_registry_.end() && it->second && it->second->is_registered) {
            channel_info = it->second.get();
            adc_driver = channel_info->adc_driver.get();
            hardware_channel_id = channel_info->hardware_channel_id;
        }
    }
    
    if (!channel_info || !adc_driver) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Reset local statistics (quick lock for statistics update)
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        channel_info->access_count = 0;
        channel_info->error_count = 0;
        channel_info->last_access_time = 0;
    }
    
    // Reset statistics in the ADC driver (no locks needed)
    return adc_driver->ResetStatistics();
}

hf_adc_err_t AdcManager::GetDiagnostics(std::string_view name, hf_adc_diagnostics_t& diagnostics) const noexcept {
    if (!is_initialized_.load()) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Find channel info with minimal lock scope
    BaseAdc* adc_driver = nullptr;
    uint8_t hardware_channel_id = 0;
    
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        auto it = adc_registry_.find(name);
        if (it != adc_registry_.end() && it->second && it->second->is_registered) {
            adc_driver = it->second->adc_driver.get();
            hardware_channel_id = it->second->hardware_channel_id;
        }
    }
    
    if (!adc_driver) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Get diagnostics from the ADC driver (no locks needed)
    return adc_driver->GetDiagnostics(diagnostics);
}

hf_adc_err_t AdcManager::ResetDiagnostics(std::string_view name) noexcept {
    if (!is_initialized_.load()) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Find channel info with minimal lock scope
    BaseAdc* adc_driver = nullptr;
    uint8_t hardware_channel_id = 0;
    
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        auto it = adc_registry_.find(name);
        if (it != adc_registry_.end() && it->second && it->second->is_registered) {
            adc_driver = it->second->adc_driver.get();
            hardware_channel_id = it->second->hardware_channel_id;
        }
    }
    
    if (!adc_driver) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Reset diagnostics in the ADC driver (no locks needed)
    return adc_driver->ResetDiagnostics();
}

hf_adc_err_t AdcManager::GetSystemHealth(std::string& health_info) const noexcept {
    if (!is_initialized_.load()) {
        health_info = "AdcManager: System not initialized";
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    AdcSystemDiagnostics diagnostics;
    hf_adc_err_t result = GetSystemDiagnostics(diagnostics);
    if (result != hf_adc_err_t::ADC_SUCCESS) {
        return result;
    }
    
    std::ostringstream oss;
    oss << "AdcManager Health Report:\n";
    oss << "  System Healthy: " << (diagnostics.system_healthy ? "YES" : "NO") << "\n";
    oss << "  Total Channels: " << diagnostics.total_channels_registered << "\n";
    oss << "  ESP32 Channels: " << diagnostics.channels_by_chip[0] << "\n";
    oss << "  TMC9660 Channels: " << diagnostics.channels_by_chip[1] << "\n";
    oss << "  Total Operations: " << diagnostics.total_operations << "\n";
    oss << "  Successful: " << diagnostics.successful_operations << "\n";
    oss << "  Failed: " << diagnostics.failed_operations << "\n";
    oss << "  Success Rate: " << std::fixed << std::setprecision(1) 
        << (diagnostics.total_operations > 0 ? 
            (static_cast<float>(diagnostics.successful_operations) / diagnostics.total_operations * 100.0f) : 0.0f) 
        << "%\n";
    oss << "  Uptime: " << (diagnostics.system_uptime_ms / 1000) << "s\n";
    oss << "  Last Error: " << static_cast<int>(diagnostics.last_error);
    
    health_info = oss.str();
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::GetSystemStatistics(std::string& stats) const noexcept {
    if (!is_initialized_.load()) {
        stats = "AdcManager: System not initialized";
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    std::ostringstream oss;
    oss << "AdcManager Statistics:\n";
    oss << "  Total Operations: " << total_operations_.load() << "\n";
    oss << "  Successful: " << successful_operations_.load() << "\n";
    oss << "  Failed: " << failed_operations_.load() << "\n";
    oss << "  Communication Errors: " << communication_errors_.load() << "\n";
    oss << "  Hardware Errors: " << hardware_errors_.load() << "\n";
    oss << "  Last Error: " << static_cast<int>(last_error_.load()) << "\n";
    
    // Channel-specific statistics
    std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
    oss << "  Channel Details:\n";
    for (const auto& [name, channel_info] : adc_registry_) {
        if (channel_info && channel_info->is_registered) {
            oss << "    " << std::string(name) << ": " 
                << channel_info->access_count << " accesses, "
                << channel_info->error_count << " errors\n";
        }
    }
    
    stats = oss.str();
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::ResetAllChannels() noexcept {
    if (!is_initialized_.load()) {
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    // Collect channel information with minimal lock scope
    std::vector<std::pair<BaseAdc*, uint8_t>> adc_operations;
    
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        
        adc_operations.reserve(adc_registry_.size());
        
        for (auto& [name, channel_info] : adc_registry_) {
            if (channel_info && channel_info->is_registered) {
                // Reset local statistics while holding lock
                channel_info->access_count = 0;
                channel_info->error_count = 0;
                channel_info->last_access_time = 0;
                
                // Collect ADC operations to perform outside lock
                adc_operations.emplace_back(channel_info->adc_driver.get(), 
                                          channel_info->hardware_channel_id);
            }
        }
    } // Lock released here
    
    // Perform ADC reset operations without holding locks
    for (const auto& [adc_driver, channel_id] : adc_operations) {
        if (adc_driver) {
            adc_driver->ResetStatistics();
            adc_driver->ResetDiagnostics();
        }
    }
    
    // Reset global statistics (atomic operations, no locks needed)
    total_operations_.store(0);
    successful_operations_.store(0);
    failed_operations_.store(0);
    communication_errors_.store(0);
    hardware_errors_.store(0);
    last_error_.store(hf_adc_err_t::ADC_SUCCESS);
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::PerformSelfTest(std::string& result) noexcept {
    if (!is_initialized_.load()) {
        result = "AdcManager: System not initialized";
        return hf_adc_err_t::ADC_ERR_NOT_INITIALIZED;
    }
    
    std::ostringstream oss;
    oss << "AdcManager Self-Test Results:\n";
    
    // Test 1: Check if any channels are registered
    size_t channel_count = Size();
    oss << "  Channels Registered: " << channel_count << " ";
    if (channel_count > 0) {
        oss << "✓\n";
    } else {
        oss << "✗ (No channels registered)\n";
        result = oss.str();
        return hf_adc_err_t::ADC_ERR_CHANNEL_NOT_FOUND;
    }
    
    // Test 2: Test reading from each channel
    std::vector<std::tuple<std::string_view, BaseAdc*, uint8_t>> test_channels;
    
    {
        std::lock_guard<RtosMutex> registry_lock(registry_mutex_);
        test_channels.reserve(adc_registry_.size());
        
        for (const auto& [name, channel_info] : adc_registry_) {
            if (channel_info && channel_info->is_registered) {
                test_channels.emplace_back(name, channel_info->adc_driver.get(), 
                                         channel_info->hardware_channel_id);
            }
        }
    } // Lock released here
    
    int successful_reads = 0;
    int total_tests = static_cast<int>(test_channels.size());
    
    // Perform ADC tests without holding locks
    for (const auto& [name, adc_driver, channel_id] : test_channels) {
        if (adc_driver) {
            // Try to read from the channel
            float voltage;
            hf_u32_t raw_value;
            hf_adc_err_t read_result = adc_driver->ReadChannel(channel_id, raw_value, voltage, 1, 0);
            
            if (read_result == hf_adc_err_t::ADC_SUCCESS) {
                successful_reads++;
                oss << "    " << std::string(name) << ": ✓ (V=" << std::fixed << std::setprecision(3) 
                    << voltage << "V, Raw=" << raw_value << ")\n";
            } else {
                oss << "    " << std::string(name) << ": ✗ (Error=" << static_cast<int>(read_result) << ")\n";
            }
        }
    }
    
    // Test 3: Overall assessment
    oss << "  Read Test: " << successful_reads << "/" << total_tests << " successful ";
    if (successful_reads == total_tests) {
        oss << "✓\n";
    } else {
        oss << "✗\n";
    }
    
    // Test 4: System health check
    AdcSystemDiagnostics diagnostics;
    hf_adc_err_t health_result = GetSystemDiagnostics(diagnostics);
    if (health_result == hf_adc_err_t::ADC_SUCCESS && diagnostics.system_healthy) {
        oss << "  System Health: ✓\n";
    } else {
        oss << "  System Health: ✗\n";
    }
    
    result = oss.str();
    
    // Return success if all tests passed
    if (successful_reads == total_tests && health_result == hf_adc_err_t::ADC_SUCCESS && diagnostics.system_healthy) {
        return hf_adc_err_t::ADC_SUCCESS;
    } else {
        return hf_adc_err_t::ADC_ERR_SYSTEM_ERROR;
    }
}

//==============================================================================
// PRIVATE HELPER METHODS
//==============================================================================

hf_adc_err_t AdcManager::Initialize() noexcept {
    // Get MotorController instance
    motor_controller_ = &MotorController::GetInstance();
    if (!motor_controller_) {
        UpdateLastError(hf_adc_err_t::ADC_ERR_HARDWARE_FAULT);
        return hf_adc_err_t::ADC_ERR_HARDWARE_FAULT;
    }
    
    // Record system start time
    system_start_time_.store(RtosTime::GetCurrentTimeUs() / 1000);
    
    // Register channels from platform mapping
    hf_adc_err_t registration_result = RegisterPlatformChannels();
    if (registration_result != hf_adc_err_t::ADC_SUCCESS) {
        UpdateLastError(registration_result);
        return registration_result;
    }
    
    // Mark as initialized
    is_initialized_.store(true);
    
    Logger::GetInstance().Info("AdcManager", "Initialized with %zu channels", Size());
    
    return hf_adc_err_t::ADC_SUCCESS;
}

std::unique_ptr<EspAdc> AdcManager::CreateEsp32Adc(uint8_t unit_id, float reference_voltage) noexcept {
    // Create EspAdc configuration
    hf_adc_unit_config_t config;
    config.unit_id = unit_id;
    config.mode = hf_adc_mode_t::ONESHOT; // Default to oneshot mode
    
    // Use new with error checking instead of make_unique to avoid exceptions
    EspAdc* raw_ptr = new (std::nothrow) EspAdc(config);
    if (raw_ptr) {
        return std::unique_ptr<EspAdc>(raw_ptr);
    }
    return nullptr;
}

std::unique_ptr<BaseAdc> AdcManager::CreateTmc9660AdcWrapper(uint8_t device_index) noexcept {
    Tmc9660Handler* handler = GetTmc9660Handler(device_index);
    if (!handler) {
        return nullptr;
    }
    
    // Create a wrapper that delegates to the handler's ADC
    // Use new with error checking instead of make_unique to avoid exceptions
    Tmc9660AdcWrapper* raw_ptr = new (std::nothrow) Tmc9660AdcWrapper(*handler);
    if (raw_ptr) {
        return std::unique_ptr<BaseAdc>(raw_ptr);
    }
    return nullptr;
}

Tmc9660Handler* AdcManager::GetTmc9660Handler(uint8_t device_index) noexcept {
    if (!motor_controller_) {
        return nullptr;
    }
    
    // Get the handler from MotorController
    return motor_controller_->handler(device_index);
}

void AdcManager::UpdateStatistics(bool success) noexcept {
    total_operations_.fetch_add(1, std::memory_order_relaxed);
    
    if (success) {
        successful_operations_.fetch_add(1, std::memory_order_relaxed);
    } else {
        failed_operations_.fetch_add(1, std::memory_order_relaxed);
    }
}

void AdcManager::UpdateLastError(hf_adc_err_t error_code) noexcept {
    last_error_.store(error_code, std::memory_order_relaxed);
    
    // Update specific error counters (atomic operations, no locks needed)
    switch (error_code) {
        case hf_adc_err_t::ADC_ERR_COMMUNICATION_FAILURE:
            communication_errors_.fetch_add(1, std::memory_order_relaxed);
            break;
        case hf_adc_err_t::ADC_ERR_HARDWARE_FAULT:
            hardware_errors_.fetch_add(1, std::memory_order_relaxed);
            break;
        default:
            break;
    }
}

hf_adc_err_t AdcManager::ValidateChannelName(std::string_view name) noexcept {
    if (name.empty()) {
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    // Check for reserved prefixes
    if (name.substr(0, 5) == "CORE_" || name.substr(0, 5) == "COMM_" || 
        name.substr(0, 4) == "SYS_" || name.substr(0, 9) == "INTERNAL_") {
        return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
    }
    
    // Check for valid characters (alphanumeric and underscore only)
    for (char c : name) {
        if (!std::isalnum(c) && c != '_') {
            return hf_adc_err_t::ADC_ERR_INVALID_PARAMETER;
        }
    }
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t AdcManager::RegisterPlatformChannels() noexcept {
    Logger::GetInstance().Info("AdcManager", "Registering platform channels...");
    
    hf_adc_err_t overall_result = hf_adc_err_t::ADC_SUCCESS;
    uint32_t registered_count = 0;
    uint32_t failed_count = 0;
    
    // Iterate through all ADC channels defined in the platform mapping
    for (size_t i = 0; i < HF_ADC_MAPPING_SIZE; ++i) {
        const auto& mapping = HF_ADC_MAPPING[i];
        const auto& channel_name = HfFunctionalAdcChannelNames[i];
        
        // Skip ESP32 internal ADC channels for now (as per user requirements)
        if (mapping.chip_type == static_cast<uint8_t>(HfAdcChipType::ESP32_INTERNAL)) {
            Logger::GetInstance().Info("AdcManager", "Skipping ESP32 internal channel %.*s (not currently used)", 
                               static_cast<int>(channel_name.length()), channel_name.data());
            continue;
        }
        
        // Filter channels based on current hardware configuration
        // Only register channels that are actually available on the current board
        bool should_register = false;
        
        switch (static_cast<HfAdcChipType>(mapping.chip_type)) {
            case HfAdcChipType::ESP32_INTERNAL:
                // ESP32 internal: currently not used (as per user requirements)
                should_register = false;
                break;
                
            case HfAdcChipType::TMC9660_CONTROLLER:
                // TMC9660: only unit 0, ADC unit 0 (single controller on board)
                should_register = (mapping.chip_unit == 0 && mapping.adc_unit == 0);
                break;
                
            default:
                Logger::GetInstance().Error("AdcManager", "Unknown chip type %d for channel %.*s", 
                                   mapping.chip_type, static_cast<int>(channel_name.length()), channel_name.data());
                continue;
        }
        
        if (!should_register) {
            Logger::GetInstance().Info("AdcManager", "Skipping ADC channel %.*s (chip_unit=%d, adc_unit=%d) - not available on current hardware", 
                               static_cast<int>(channel_name.length()), channel_name.data(), 
                               mapping.chip_unit, mapping.adc_unit);
            continue;
        }
        
        // Handle TMC9660 channels
        if (mapping.chip_type == static_cast<uint8_t>(HfAdcChipType::TMC9660_CONTROLLER)) {
            // Create TMC9660 ADC wrapper with the correct device index
            auto tmc9660_wrapper = CreateTmc9660AdcWrapper(mapping.chip_unit);
            if (tmc9660_wrapper) {
                hf_adc_err_t result = RegisterChannel(channel_name, std::move(tmc9660_wrapper));
                if (result == hf_adc_err_t::ADC_SUCCESS) {
                    Logger::GetInstance().Info("AdcManager", "Registered TMC9660 channel %.*s (chip_unit=%d, adc_unit=%d)", 
                                       static_cast<int>(channel_name.length()), channel_name.data(), 
                                       mapping.chip_unit, mapping.adc_unit);
                    registered_count++;
                } else {
                    Logger::GetInstance().Info("AdcManager", "Failed to register TMC9660 channel %.*s (error: %d)", 
                                       static_cast<int>(channel_name.length()), channel_name.data(), static_cast<int>(result));
                    failed_count++;
                    overall_result = result; // Keep track of first error
                }
            } else {
                Logger::GetInstance().Info("AdcManager", "Failed to create TMC9660 wrapper for channel %.*s (chip_unit=%d, adc_unit=%d)", 
                                   static_cast<int>(channel_name.length()), channel_name.data(), 
                                   mapping.chip_unit, mapping.adc_unit);
                failed_count++;
                overall_result = hf_adc_err_t::ADC_ERR_HARDWARE_FAULT;
            }
        }
    }
    
    Logger::GetInstance().Info("AdcManager", "Platform registration complete - %d registered, %d failed", 
                       registered_count, failed_count);
    
    return overall_result;
}

void AdcManager::DumpStatistics() const noexcept {
    static constexpr const char* TAG = "AdcManager";
    
    Logger::GetInstance().Info(TAG, "=== ADC MANAGER STATISTICS ===");
    
    MutexLockGuard lock(mutex_);
    
    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", is_initialized_.load() ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Total Channels Registered: %d", static_cast<int>(adc_registry_.size()));
    
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
    Logger::GetInstance().Info(TAG, "  Last Error: %d", static_cast<int>(last_error_.load()));
    
    // Channel Statistics Summary
    Logger::GetInstance().Info(TAG, "Channel Statistics Summary:");
    
    struct ChannelStatEntry {
        std::string_view name;
        hf_adc_statistics_t stats;
    };
    std::vector<ChannelStatEntry> channel_stats;
    
    for (const auto& [name, channel_info] : adc_registry_) {
        if (channel_info && channel_info->adc_driver) {
            hf_adc_statistics_t stats{};
            if (channel_info->adc_driver->GetStatistics(stats) == hf_adc_err_t::ADC_SUCCESS) {
                channel_stats.push_back({name, stats});
            }
        }
    }
    
    // Sort by total conversions
    std::sort(channel_stats.begin(), channel_stats.end(), 
        [](const auto& a, const auto& b) {
            return a.stats.totalConversions > b.stats.totalConversions;
        });
    
    int channels_to_show = std::min(5, static_cast<int>(channel_stats.size()));
    if (channels_to_show > 0) {
        Logger::GetInstance().Info(TAG, "  Top %d Most Active Channels:", channels_to_show);
        for (int i = 0; i < channels_to_show; ++i) {
            const auto& channel = channel_stats[i];
            Logger::GetInstance().Info(TAG, "    %.*s: %lu ops (success: %lu, failed: %lu)", 
                static_cast<int>(channel.name.length()), channel.name.data(),
                static_cast<unsigned long>(channel.stats.totalConversions),
                static_cast<unsigned long>(channel.stats.successfulConversions),
                static_cast<unsigned long>(channel.stats.failedConversions));
        }
    } else {
        Logger::GetInstance().Info(TAG, "  No channel statistics available");
    }
    
    // System Health Summary
    std::string health_info;
    if (GetSystemHealth(health_info) == hf_adc_err_t::ADC_SUCCESS) {
        Logger::GetInstance().Info(TAG, "System Health Summary:");
        Logger::GetInstance().Info(TAG, "  %s", health_info.c_str());
    }
    
    Logger::GetInstance().Info(TAG, "=== END ADC MANAGER STATISTICS ===");
}
