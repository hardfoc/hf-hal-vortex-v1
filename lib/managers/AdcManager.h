/**
 * @file AdcManager.h
 * @brief Advanced ADC management system for the HardFOC platform.
 * 
 * @details This class provides a comprehensive ADC management system that integrates
 *          with the platform mapping system to automatically manage ADC channels from multiple
 *          hardware sources (ESP32-C6 internal ADC, TMC9660 ADC) based on functional channel
 *          identifiers and hardware chip mappings.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 * 
 * Key Features:
 * - Platform mapping integration for automatic channel discovery
 * - Multi-chip ADC management (ESP32 internal, TMC9660)
 * - Functional channel abstraction with hardware-agnostic API
 * - Thread-safe operation with comprehensive error handling
 * - Automatic channel registration based on platform configuration
 * - Advanced diagnostics and health monitoring
 * - Batch operations for performance optimization
 * - Hardware resource validation and conflict detection
 * - String-based channel identification for extensibility
 * - Smart channel categorization and mapping
 * - Complete BaseAdc function coverage through string-based routing
 * 
 * Architecture:
 * - Uses string_view for channel identification (extensible)
 * - Integrates with platform mapping for hardware mapping
 * - Supports all HardwareChip types defined in platform mapping
 * - Provides unified BaseAdc interface for all channel operations
 * - Handler-based ADC creation for proper ownership
 * - Routes all BaseAdc functions through string-based API
 * 
 * @note This class is thread-safe and designed for concurrent access from multiple tasks.
 * @note All channel operations use string_view identifiers for maximum flexibility.
 */

#ifndef COMPONENT_HANDLER_ADC_MANAGER_H_
#define COMPONENT_HANDLER_ADC_MANAGER_H_

#include "CommonIDs.h"
#include "ThingsToString.h"
#include "base/BaseAdc.h"
#include "MotorController.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspAdc.h"
#include "Tmc9660AdcWrapper.h"

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <string_view>
#include <unordered_map>
#include <optional>

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class Tmc9660Handler;

//==============================================================================
// ADC INFORMATION STRUCTURES
//==============================================================================

/**
 * @brief Structure containing comprehensive ADC channel information with platform mapping.
 */
struct AdcChannelInfo {
    std::string_view name;                      ///< Human-readable name (string_view to static data)
    std::unique_ptr<BaseAdc> adc_driver;        ///< ADC driver instance (unique ownership)
    HfFunctionalAdcChannel functional_channel;  ///< Functional channel identifier
    HfAdcChipType hardware_chip;                ///< Hardware chip identifier
    uint8_t hardware_channel_id;                ///< Hardware channel ID within the chip
    bool is_registered;                         ///< Registration status
    
    // Hardware configuration
    float reference_voltage;                    ///< Reference voltage for conversion
    uint32_t resolution_bits;                   ///< ADC resolution in bits
    uint32_t max_voltage_mv;                    ///< Maximum voltage in millivolts
    float voltage_divider;                      ///< Voltage divider ratio
    
    // Statistics and monitoring
    uint32_t access_count;                      ///< Number of times accessed
    uint32_t error_count;                       ///< Number of errors encountered
    uint64_t last_access_time;                  ///< Timestamp of last access
    
    /**
     * @brief Constructor for AdcChannelInfo.
     */
    AdcChannelInfo(std::string_view n, std::unique_ptr<BaseAdc> driver, 
                   HfFunctionalAdcChannel fc, HfAdcChipType chip, uint8_t ch_id,
                   float ref_volt = 3.3f, uint32_t res = 12, uint32_t max_v = 3300, float div = 1.0f) noexcept
        : name(n), adc_driver(std::move(driver)), functional_channel(fc), hardware_chip(chip),
          hardware_channel_id(ch_id), is_registered(true), reference_voltage(ref_volt),
          resolution_bits(res), max_voltage_mv(max_v), voltage_divider(div),
          access_count(0), error_count(0), last_access_time(0) {}
    
    // Disable copy operations due to unique_ptr
    AdcChannelInfo(const AdcChannelInfo&) = delete;
    AdcChannelInfo& operator=(const AdcChannelInfo&) = delete;
    
    // Enable move operations
    AdcChannelInfo(AdcChannelInfo&&) = default;
    AdcChannelInfo& operator=(AdcChannelInfo&&) = default;
};

/**
 * @brief Structure for ADC batch operation specifications.
 */
struct AdcBatchOperation {
    std::vector<std::string_view> channel_names;    ///< Channel names to operate on
    std::vector<uint8_t> samples_per_channel;       ///< Samples per channel
    std::vector<uint16_t> intervals_ms;             ///< Intervals between samples in ms
    bool use_individual_specs;                      ///< Use individual specs or common settings
    uint8_t common_samples;                         ///< Common number of samples (if not using individual specs)
    uint16_t common_interval_ms;                    ///< Common sampling interval (if not using individual specs)
    
    /**
     * @brief Constructor for simple batch read.
     */
    AdcBatchOperation(std::vector<std::string_view> names, uint8_t samples = 1, uint16_t interval_ms = 0) noexcept
        : channel_names(std::move(names)), use_individual_specs(false),
          common_samples(samples), common_interval_ms(interval_ms) {}
    
    /**
     * @brief Constructor for advanced batch read with individual specs.
     */
    AdcBatchOperation(std::vector<std::string_view> names, std::vector<uint8_t> samples, std::vector<uint16_t> intervals) noexcept
        : channel_names(std::move(names)), samples_per_channel(std::move(samples)), 
          intervals_ms(std::move(intervals)), use_individual_specs(true),
          common_samples(1), common_interval_ms(0) {}
};

/**
 * @brief Structure for ADC batch operation results.
 */
struct AdcBatchResult {
    std::vector<std::string_view> channel_names;    ///< Channel names operated on
    std::vector<float> voltages;                    ///< Resulting voltage readings
    std::vector<uint32_t> raw_values;               ///< Raw ADC values
    std::vector<hf_adc_err_t> results;              ///< Individual operation results
    hf_adc_err_t overall_result;                    ///< Overall operation result
    uint32_t total_time_ms;                         ///< Total operation time
    
    /**
     * @brief Check if all operations were successful.
     */
    [[nodiscard]] bool AllSuccessful() const noexcept {
        return overall_result == hf_adc_err_t::ADC_SUCCESS;
    }
    
    /**
     * @brief Get success rate as percentage.
     */
    [[nodiscard]] float GetSuccessRate() const noexcept {
        if (results.empty()) return 0.0f;
        uint32_t successCount = 0;
        for (const auto& result : results) {
            if (result == hf_adc_err_t::ADC_SUCCESS) ++successCount;
        }
        return (static_cast<float>(successCount) / static_cast<float>(results.size())) * 100.0f;
    }
};

/**
 * @brief Structure for ADC system diagnostics.
 */
struct AdcSystemDiagnostics {
    bool system_healthy;                           ///< Overall system health
    uint32_t total_channels_registered;            ///< Total channels registered
    uint32_t channels_by_chip[2];                  ///< Channels per chip (ESP32, TMC9660)
    uint32_t total_operations;                     ///< Total operations performed
    uint32_t successful_operations;                ///< Successful operations
    uint32_t failed_operations;                    ///< Failed operations
    uint32_t communication_errors;                 ///< Communication errors
    uint32_t hardware_errors;                      ///< Hardware errors
    uint64_t system_uptime_ms;                     ///< System uptime
    hf_adc_err_t last_error;                       ///< Last error encountered
};

//==============================================================================
// MAIN ADC MANAGER CLASS
//==============================================================================

/**
 * @class AdcManager
 * @brief Advanced ADC management system for the HardFOC platform.
 * 
 * This class provides a comprehensive ADC management system that integrates
 * with the platform mapping system to automatically manage ADC channels from multiple
 * hardware sources. It uses string_view identifiers and hardware chip
 * mappings to provide a unified, hardware-agnostic API that routes all
 * BaseAdc functions through string-based channel identification.
 * 
 * Thread Safety:
 * - All public methods are thread-safe
 * - Uses internal mutex for protection
 * - Atomic operations where appropriate
 * 
 * Error Handling:
 * - Core operations return hf_adc_err_t for detailed error codes
 * - Comprehensive error codes via ResultCode enum
 * - Detailed error descriptions and diagnostics
 * 
 * Performance:
 * - Optimized for common operations
 * - Batch operations for multiple channels
 * - Cached state information
 * - Lazy initialization of hardware resources
 * 
 * Platform Integration:
 * - Automatic channel discovery via platform mapping
 * - Hardware resource validation
 * - Conflict detection and resolution
 * - Multi-chip coordination
 * - Smart channel categorization
 * 
 * Function Coverage:
 * - Complete BaseAdc function coverage through string-based routing
 * - All ADC operations available via string_view channel names
 * - Consistent API design with proper camelCase naming
 */
class AdcManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================
    
    /**
     * @brief Get the singleton instance.
     * @return Reference to the ADC manager instance
     */
    static AdcManager& GetInstance() noexcept;
    
    /**
     * @brief Ensure the ADC manager system is initialized.
     * @return hf_adc_err_t::ADC_SUCCESS if initialization successful, error code otherwise
     */
    [[nodiscard]] hf_adc_err_t EnsureInitialized() noexcept;
    
    /**
     * @brief Shutdown the ADC manager system.
     * @return hf_adc_err_t::ADC_SUCCESS if shutdown successful, error code otherwise
     */
    [[nodiscard]] hf_adc_err_t Shutdown() noexcept;
    
    /**
     * @brief Check if the ADC system is initialized.
     * @return true if initialized, false otherwise
     */
    [[nodiscard]] bool IsInitialized() const noexcept;
    
    /**
     * @brief Get system diagnostics and health information.
     * @param diagnostics Reference to store system diagnostics
     * @return hf_adc_err_t::ADC_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_adc_err_t GetSystemDiagnostics(AdcSystemDiagnostics& diagnostics) const noexcept;
    
    //==========================================================================
    // CHANNEL REGISTRATION AND MANAGEMENT
    //==========================================================================
    
    /**
     * @brief Register an ADC channel with the system using string_view identifier.
     * @param name Channel name (must be static string or outlive the manager)
     * @param adc Unique pointer to ADC driver
     * @return hf_adc_err_t::ADC_SUCCESS if successful, error code otherwise
     * @note Channel names must be static strings (string literals or static arrays)
     * @note Reserved prefixes (CORE_, COMM_, SYS_, INTERNAL_) are not allowed
     */
    [[nodiscard]] hf_adc_err_t RegisterChannel(std::string_view name, std::unique_ptr<BaseAdc> adc) noexcept;
    
    /**
     * @brief Get an ADC channel by name.
     * @param name Channel name
     * @return Pointer to BaseAdc or nullptr if not found
     */
    [[nodiscard]] BaseAdc* Get(std::string_view name) noexcept;
    
    /**
     * @brief Check if an ADC channel is registered.
     * @param name Channel name
     * @return true if registered, false otherwise
     */
    [[nodiscard]] bool Contains(std::string_view name) const noexcept;
    
    /**
     * @brief Get count of registered channels.
     * @return Number of registered channels
     */
    [[nodiscard]] size_t Size() const noexcept;
    
    /**
     * @brief Log all registered ADC channels for debugging.
     */
    void LogAllRegisteredChannels() const noexcept;
    
    //==========================================================================
    // BASIC READING OPERATIONS (Complete BaseAdc Coverage)
    //==========================================================================
    
    /**
     * @brief Read a single ADC channel by name.
     * @param name Channel name
     * @param voltage Reference to store the voltage reading
     * @param numOfSamplesToAvg Number of samples to average (default 1)
     * @param timeBetweenSamples Time between samples in ms (default 0)
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ReadChannelV(std::string_view name, float& voltage,
                                           hf_u8_t numOfSamplesToAvg = 1,
                                           hf_time_t timeBetweenSamples = 0) noexcept;
    
    /**
     * @brief Read raw ADC count value (no conversion).
     * @param name Channel name
     * @param value Reference to store the raw count
     * @param numOfSamplesToAvg Number of samples to average (default 1)
     * @param timeBetweenSamples Time between samples in ms (default 0)
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ReadChannelCount(std::string_view name, hf_u32_t& value,
                                               hf_u8_t numOfSamplesToAvg = 1,
                                               hf_time_t timeBetweenSamples = 0) noexcept;
    
    /**
     * @brief Read both raw count and voltage from a channel.
     * @param name Channel name
     * @param raw_value Reference to store the raw count
     * @param voltage Reference to store the voltage reading
     * @param numOfSamplesToAvg Number of samples to average (default 1)
     * @param timeBetweenSamples Time between samples in ms (default 0)
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ReadChannel(std::string_view name, hf_u32_t& raw_value, float& voltage,
                                          hf_u8_t numOfSamplesToAvg = 1,
                                          hf_time_t timeBetweenSamples = 0) noexcept;
    
    /**
     * @brief Read multiple channels simultaneously.
     * @param channel_names Array of channel names
     * @param num_channels Number of channels
     * @param raw_values Array to store raw readings
     * @param voltages Array to store voltage readings
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ReadMultipleChannels(const std::string_view* channel_names, hf_u8_t num_channels,
                                                   hf_u32_t* raw_values, float* voltages) noexcept;
    
    //==========================================================================
    // BATCH OPERATIONS
    //==========================================================================
    
    /**
     * @brief Perform batch read operations on multiple channels.
     * @param operation Batch operation specification
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] AdcBatchResult BatchRead(const AdcBatchOperation& operation) noexcept;
    
    /**
     * @brief Read multiple ADC channels by name.
     * @param channel_names Vector of channel names to read
     * @param samples_per_channel Number of samples per channel (default 1)
     * @param interval_ms Time between samples in ms (default 0)
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] AdcBatchResult BatchRead(const std::vector<std::string_view>& channel_names,
                                          uint8_t samples_per_channel = 1,
                                          uint16_t interval_ms = 0) noexcept;
    
    /**
     * @brief Read all registered channels.
     * @return Batch operation results with readings from all channels
     */
    [[nodiscard]] AdcBatchResult ReadAllChannels() noexcept;
    
    //==========================================================================
    // STATISTICS AND DIAGNOSTICS (Complete BaseAdc Coverage)
    //==========================================================================
    
    /**
     * @brief Get channel statistics.
     * @param name Channel name
     * @param statistics Reference to store channel statistics
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t GetStatistics(std::string_view name, BaseAdc::AdcStatistics& statistics) const noexcept;
    
    /**
     * @brief Reset channel statistics.
     * @param name Channel name
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ResetStatistics(std::string_view name) noexcept;
    
    /**
     * @brief Get channel diagnostics.
     * @param name Channel name
     * @param diagnostics Reference to store channel diagnostics
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t GetDiagnostics(std::string_view name, BaseAdc::AdcDiagnostics& diagnostics) const noexcept;
    
    /**
     * @brief Reset channel diagnostics.
     * @param name Channel name
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ResetDiagnostics(std::string_view name) noexcept;
    
    /**
     * @brief Get system health information.
     * @param health_info Reference to store health information
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t GetSystemHealth(std::string& health_info) const noexcept;
    
    /**
     * @brief Get detailed system statistics.
     * @param stats Reference to store statistics
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t GetSystemStatistics(std::string& stats) const noexcept;
    
    /**
     * @brief Reset all channel statistics and diagnostics.
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t ResetAllChannels() noexcept;
    
    /**
     * @brief Dump comprehensive system statistics to log as INFO level.
     * Logs all channel statistics, system health, and operation counters.
     */
    void DumpStatistics() const noexcept;
    
    /**
     * @brief Perform system self-test.
     * @param result Reference to store test results
     * @return hf_adc_err_t operation result
     */
    [[nodiscard]] hf_adc_err_t PerformSelfTest(std::string& result) noexcept;

private:
    //==========================================================================
    // PRIVATE MEMBERS
    //==========================================================================
    
    /**
     * @brief Initialize the ADC manager system with platform channel registration.
     * 
     * This private method performs the actual initialization including:
     * - MotorController dependency validation
     * - Smart channel categorization and registration
     * - Lazy initialization of hardware handlers
     * - Thread-safe initialization with RtosMutex
     * 
     * @return hf_adc_err_t initialization result
     */
    [[nodiscard]] hf_adc_err_t Initialize() noexcept;
    
    // ===============================
    // SYSTEM STATE
    // ===============================
    
    /**
     * @brief System initialization state (atomic for thread safety).
     */
    std::atomic<bool> is_initialized_{false};
    
    /**
     * @brief Main system mutex for thread-safe operations.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex mutex_;
    
    // ===============================
    // ADC STORAGE
    // ===============================
    
    /**
     * @brief String-based ADC registry with unique_ptr ownership.
     * Uses string_view keys pointing to static strings for embedded efficiency.
     * Protected by RtosMutex for thread-safe access.
     */
    std::unordered_map<std::string_view, std::unique_ptr<AdcChannelInfo>> adc_registry_;
    mutable RtosMutex registry_mutex_;  ///< RtosMutex for registry access
    
    // ===============================
    // HARDWARE HANDLERS (LAZY INITIALIZATION)
    // ===============================
    
    /**
     * @brief ESP32 ADC handlers (lazy initialized when first accessed).
     * Multiple instances for multi-unit boards (ADC1, ADC2).
     * Uses EspAdcWrapper for BaseAdc interface compliance.
     */
    std::array<std::unique_ptr<EspAdcWrapper>, 2> esp32_adc_handlers_;
    mutable RtosMutex esp32_adc_mutex_;  ///< Mutex for ESP32 ADC handler initialization
    
    /**
     * @brief Reference to MotorController for TMC9660 handler access.
     * MotorController owns all Tmc9660Handler instances - AdcManager gets access through it.
     * This ensures proper ownership hierarchy and prevents duplicate handlers.
     * 
     * Ownership Model:
     * - MotorController: Sole owner of Tmc9660Handler instances (unique_ptr)
     * - AdcManager: Consumer that gets access via raw pointers
     * - Handler lifetime: Tied to MotorController lifecycle
     */
    MotorController* motor_controller_ = nullptr;
    
    // ===============================
    // SYSTEM STATISTICS
    // ===============================
    
    /**
     * @brief Total operations performed (atomic for thread safety).
     */
    std::atomic<uint32_t> total_operations_{0};
    
    /**
     * @brief Successful operations count (atomic for thread safety).
     */
    std::atomic<uint32_t> successful_operations_{0};
    
    /**
     * @brief Failed operations count (atomic for thread safety).
     */
    std::atomic<uint32_t> failed_operations_{0};
    
    /**
     * @brief Communication errors count (atomic for thread safety).
     */
    std::atomic<uint32_t> communication_errors_{0};
    
    /**
     * @brief Hardware errors count (atomic for thread safety).
     */
    std::atomic<uint32_t> hardware_errors_{0};
    
    /**
     * @brief System start time for uptime calculations (atomic for thread safety).
     */
    std::atomic<uint64_t> system_start_time_{0};
    
    // ===============================
    // ERROR TRACKING
    // ===============================
    
    /**
     * @brief Thread-safe access to error tracking.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex error_mutex_;
    
    /**
     * @brief Last error encountered (for diagnostics only).
     */
    std::atomic<hf_adc_err_t> last_error_{hf_adc_err_t::ADC_SUCCESS};
    
    // ===============================
    // PRIVATE HELPER METHODS
    // ===============================
    
    /**
     * @brief Create ESP32 ADC instance for a specific unit.
     * @param unit_id ADC unit ID (0=ADC1, 1=ADC2)
     * @param reference_voltage Reference voltage in volts
     * @return Unique pointer to EspAdc
     */
    [[nodiscard]] std::unique_ptr<EspAdc> CreateEsp32Adc(uint8_t unit_id, float reference_voltage) noexcept;
    
    /**
     * @brief Create TMC9660 ADC wrapper for a specific device.
     * @param device_index Device index (0=first device, 1=second device, etc.)
     * @return Unique pointer to TMC9660 ADC wrapper
     */
    [[nodiscard]] std::unique_ptr<BaseAdc> CreateTmc9660AdcWrapper(uint8_t device_index = 0) noexcept;
    
    /**
     * @brief Get TMC9660 handler for a specific device.
     * @param device_index Device index
     * @return Pointer to TMC9660 handler or nullptr if not available
     */
    [[nodiscard]] Tmc9660Handler* GetTmc9660Handler(uint8_t device_index = 0) noexcept;
    
    /**
     * @brief Update global statistics.
     * @param success Whether the operation was successful
     */
    void UpdateStatistics(bool success) noexcept;
    
    /**
     * @brief Update last error.
     * @param error_code Error code to store
     */
    void UpdateLastError(hf_adc_err_t error_code) noexcept;
    
    /**
     * @brief Validate channel name.
     * @param name Channel name to validate
     * @return hf_adc_err_t validation result
     */
    [[nodiscard]] static hf_adc_err_t ValidateChannelName(std::string_view name) noexcept;
    
    /**
     * @brief Register channels from platform mapping.
     * @return hf_adc_err_t registration result
     */
    [[nodiscard]] hf_adc_err_t RegisterPlatformChannels() noexcept;
};

//==============================================================================
// GLOBAL ACCESS FUNCTION
//==============================================================================

/**
 * @brief Get global ADC manager instance.
 * @return Reference to the ADC manager instance
 */
[[nodiscard]] inline AdcManager& GetAdcManager() noexcept {
    return AdcManager::GetInstance();
}

#endif // COMPONENT_HANDLER_ADC_MANAGER_H_
