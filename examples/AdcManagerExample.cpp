/**
 * @file AdcManagerExample.cpp
 * @brief Comprehensive example demonstrating the new AdcManager capabilities.
 * 
 * @details This example shows how to use the refactored AdcManager with:
 * - Platform mapping integration for automatic channel discovery
 * - ESP32 internal ADC and TMC9660 ADC support
 * - Complete BaseAdc function coverage through string-based routing
 * - Batch operations and advanced diagnostics
 * - Thread-safe operations and error handling
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 * 
 * Key Features Demonstrated:
 * - Automatic initialization and channel registration
 * - Single channel reading operations
 * - Batch reading operations
 * - Statistics and diagnostics
 * - System health monitoring
 * - Error handling and recovery
 * - Platform mapping integration
 * - TMC9660 ADC channel types (AIN, Current, Voltage, Temperature, Motor Data)
 * 
 * @note This example assumes the AdcManager has been properly initialized
 *       with platform mapping and hardware handlers.
 */

#include "AdcManager.h"
#include "MotorController.h"
#include "handlers/logger/Logger.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/ConsolePort.h"

#include <vector>
#include <string>

//==============================================================================
// EXAMPLE FUNCTIONS
//==============================================================================

/**
 * @brief Demonstrate basic ADC channel reading operations.
 */
void DemonstrateBasicReading() {
    Logger::GetInstance().Info("AdcManagerExample", "=== Basic ADC Reading Operations ===");
    
    // Get ADC manager instance
    auto& adc_manager = GetAdcManager();
    
    // Ensure system is initialized
    hf_adc_err_t init_result = adc_manager.EnsureInitialized();
    if (init_result != hf_adc_err_t::ADC_SUCCESS) {
        Logger::GetInstance().Error("AdcManagerExample", "Failed to initialize AdcManager: %d", static_cast<int>(init_result));
        return;
    }
    
    Logger::GetInstance().Info("AdcManagerExample", "AdcManager initialized successfully");
    
    // Log all registered channels
    adc_manager.LogAllRegisteredChannels();
    
    // Example channel names from platform mapping
    std::vector<std::string_view> test_channels = {
        "TMC9660_AIN3",           // Temperature sensor (connected)
        "TMC9660_CURRENT_I0",     // Current sense I0
        "TMC9660_SUPPLY_VOLTAGE", // Supply voltage
        "TMC9660_CHIP_TEMPERATURE", // Chip temperature
        "TMC9660_MOTOR_CURRENT"   // Motor current
    };
    
    // Test reading from each channel
    for (const auto& channel_name : test_channels) {
        if (adc_manager.Contains(channel_name)) {
            Logger::GetInstance().Info("AdcManagerExample", "Reading from channel: %.*s", 
                               static_cast<int>(channel_name.length()), channel_name.data());
            
            // Read voltage only
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("AdcManagerExample", "  Voltage: %.3fV", voltage);
            } else {
                Logger::GetInstance().Error("AdcManagerExample", "  Error reading voltage: %d", static_cast<int>(result));
            }
            
            // Read raw count only
            hf_u32_t raw_value;
            result = adc_manager.ReadChannelCount(channel_name, raw_value);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("AdcManagerExample", "  Raw Count: %u", raw_value);
            } else {
                Logger::GetInstance().Error("AdcManagerExample", "  Error reading raw count: %d", static_cast<int>(result));
            }
            
            // Read both voltage and raw count
            hf_u32_t raw_value2;
            float voltage2;
            result = adc_manager.ReadChannel(channel_name, raw_value2, voltage2);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("AdcManagerExample", "  Combined Read - Voltage: %.3fV, Raw: %u", voltage2, raw_value2);
            } else {
                Logger::GetInstance().Error("AdcManagerExample", "  Error reading combined: %d", static_cast<int>(result));
            }
        } else {
            Logger::GetInstance().Error("AdcManagerExample", "Channel %.*s not found", 
                               static_cast<int>(channel_name.length()), channel_name.data());
        }
    }
}

/**
 * @brief Demonstrate TMC9660-specific channel reading.
 */
void DemonstrateTmc9660Channels() {
    Logger::GetInstance().Info("AdcManagerExample", "=== TMC9660 ADC Channel Reading ===");
    
    auto& adc_manager = GetAdcManager();
    
    // Test AIN channels (external analog inputs)
    Logger::GetInstance().Info("AdcManagerExample", "--- AIN Channels (External Analog Inputs) ---");
    for (int i = 0; i <= 3; ++i) {
        std::string channel_name = "TMC9660_AIN" + std::to_string(i);
        if (adc_manager.Contains(channel_name)) {
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("AdcManagerExample", "AIN%d: %.3fV", i, voltage);
            } else {
                Logger::GetInstance().Error("AdcManagerExample", "AIN%d: Error %d", i, static_cast<int>(result));
            }
        }
    }
    
    // Test current sense channels
    Logger::GetInstance().Info("AdcManagerExample", "--- Current Sense Channels ---");
    for (int i = 0; i <= 3; ++i) {
        std::string channel_name = "TMC9660_CURRENT_I" + std::to_string(i);
        if (adc_manager.Contains(channel_name)) {
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("AdcManagerExample", "Current I%d: %.3fV", i, voltage);
            } else {
                Logger::GetInstance().Error("AdcManagerExample", "Current I%d: Error %d", i, static_cast<int>(result));
            }
        }
    }
    
    // Test voltage monitoring channels
    Logger::GetInstance().Info("AdcManagerExample", "--- Voltage Monitoring Channels ---");
    std::vector<std::string> voltage_channels = {"TMC9660_SUPPLY_VOLTAGE", "TMC9660_DRIVER_VOLTAGE"};
    for (const auto& channel_name : voltage_channels) {
        if (adc_manager.Contains(channel_name)) {
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("AdcManagerExample", "%s: %.3fV", channel_name.c_str(), voltage);
            } else {
                Logger::GetInstance().Error("AdcManagerExample", "%s: Error %d", channel_name.c_str(), static_cast<int>(result));
            }
        }
    }
    
    // Test temperature channels
    Logger::GetInstance().Info("AdcManagerExample", "--- Temperature Channels ---");
    std::vector<std::string> temp_channels = {"TMC9660_CHIP_TEMPERATURE", "TMC9660_EXTERNAL_TEMPERATURE"};
    for (const auto& channel_name : temp_channels) {
        if (adc_manager.Contains(channel_name)) {
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("AdcManagerExample", "%s: %.1f°C", channel_name.c_str(), voltage); // voltage contains temperature
            } else {
                Logger::GetInstance().Error("AdcManagerExample", "%s: Error %d", channel_name.c_str(), static_cast<int>(result));
            }
        }
    }
    
    // Test motor data channels
    Logger::GetInstance().Info("AdcManagerExample", "--- Motor Data Channels ---");
    std::vector<std::string> motor_channels = {"TMC9660_MOTOR_CURRENT", "TMC9660_MOTOR_VELOCITY", "TMC9660_MOTOR_POSITION"};
    for (const auto& channel_name : motor_channels) {
        if (adc_manager.Contains(channel_name)) {
            float voltage;
            hf_adc_err_t result = adc_manager.ReadChannelV(channel_name, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                if (channel_name == "TMC9660_MOTOR_CURRENT") {
                    Logger::GetInstance().Info("AdcManagerExample", "%s: %.3fA", channel_name.c_str(), voltage);
                } else {
                    Logger::GetInstance().Info("AdcManagerExample", "%s: %.0f", channel_name.c_str(), voltage);
                }
            } else {
                Logger::GetInstance().Error("AdcManagerExample", "%s: Error %d", channel_name.c_str(), static_cast<int>(result));
            }
        }
    }
}

/**
 * @brief Demonstrate batch reading operations.
 */
void DemonstrateBatchReading() {
    Logger::GetInstance().Info("AdcManagerExample", "=== Batch Reading Operations ===");
    
    auto& adc_manager = GetAdcManager();
    
    // Define channels to read in batch
    std::vector<std::string_view> batch_channels = {
        "TMC9660_AIN3",
        "TMC9660_CURRENT_I0", 
        "TMC9660_SUPPLY_VOLTAGE",
        "TMC9660_CHIP_TEMPERATURE"
    };
    
    // Read multiple channels simultaneously
    std::vector<hf_u32_t> raw_values(batch_channels.size());
    std::vector<float> voltages(batch_channels.size());
    
    hf_adc_err_t result = adc_manager.ReadMultipleChannels(
        batch_channels.data(), 
        static_cast<hf_u8_t>(batch_channels.size()),
        raw_values.data(),
        voltages.data()
    );
    
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        Logger::GetInstance().Info("AdcManagerExample", "Batch read successful:");
        for (size_t i = 0; i < batch_channels.size(); ++i) {
            Logger::GetInstance().Info("AdcManagerExample", "  %.*s: Raw=%u, Voltage=%.3fV",
                               static_cast<int>(batch_channels[i].length()), batch_channels[i].data(),
                               raw_values[i], voltages[i]);
        }
    } else {
        Logger::GetInstance().Error("AdcManagerExample", "Batch read failed: %d", static_cast<int>(result));
    }
}

/**
 * @brief Demonstrate statistics and diagnostics.
 */
void DemonstrateStatisticsAndDiagnostics() {
    Logger::GetInstance().Info("AdcManagerExample", "=== Statistics and Diagnostics ===");
    
    auto& adc_manager = GetAdcManager();
    
    // Get system statistics
    AdcSystemStatistics system_stats;
    hf_adc_err_t result = adc_manager.GetSystemStatistics(system_stats);
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        Logger::GetInstance().Info("AdcManagerExample", "System Statistics:");
        Logger::GetInstance().Info("AdcManagerExample", "  Total Operations: %u", system_stats.total_operations);
        Logger::GetInstance().Info("AdcManagerExample", "  Successful Operations: %u", system_stats.successful_operations);
        Logger::GetInstance().Info("AdcManagerExample", "  Failed Operations: %u", system_stats.failed_operations);
        Logger::GetInstance().Info("AdcManagerExample", "  Communication Errors: %u", system_stats.communication_errors);
        Logger::GetInstance().Info("AdcManagerExample", "  Hardware Errors: %u", system_stats.hardware_errors);
    }
    
    // Get system health
    AdcSystemHealth system_health;
    result = adc_manager.GetSystemHealth(system_health);
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        Logger::GetInstance().Info("AdcManagerExample", "System Health:");
        Logger::GetInstance().Info("AdcManagerExample", "  Overall Health: %s", system_health.overall_healthy ? "Healthy" : "Unhealthy");
        Logger::GetInstance().Info("AdcManagerExample", "  Active Channels: %u", system_health.active_channels);
        Logger::GetInstance().Info("AdcManagerExample", "  Error Rate: %.2f%%", system_health.error_rate_percent);
        Logger::GetInstance().Info("AdcManagerExample", "  Uptime: %llu seconds", system_health.uptime_seconds);
    }
    
    // Get channel-specific statistics
    std::string test_channel = "TMC9660_AIN3";
    if (adc_manager.Contains(test_channel)) {
        BaseAdc::AdcStatistics channel_stats;
        result = adc_manager.GetStatistics(test_channel, channel_stats);
        if (result == hf_adc_err_t::ADC_SUCCESS) {
            Logger::GetInstance().Info("AdcManagerExample", "Channel Statistics for %s:", test_channel.c_str());
            Logger::GetInstance().Info("AdcManagerExample", "  Total Conversions: %u", channel_stats.totalConversions);
            Logger::GetInstance().Info("AdcManagerExample", "  Successful Conversions: %u", channel_stats.successfulConversions);
            Logger::GetInstance().Info("AdcManagerExample", "  Failed Conversions: %u", channel_stats.failedConversions);
            Logger::GetInstance().Info("AdcManagerExample", "  Average Conversion Time: %u us", channel_stats.averageConversionTimeUs);
        }
    }
}

/**
 * @brief Demonstrate error handling and recovery.
 */
void DemonstrateErrorHandling() {
    Logger::GetInstance().Info("AdcManagerExample", "=== Error Handling and Recovery ===");
    
    auto& adc_manager = GetAdcManager();
    
    // Try to read from a non-existent channel
    std::string non_existent_channel = "NON_EXISTENT_CHANNEL";
    float voltage;
    hf_adc_err_t result = adc_manager.ReadChannelV(non_existent_channel, voltage);
    Logger::GetInstance().Error("AdcManagerExample", "Reading from non-existent channel: %d", static_cast<int>(result));
    
    // Try to read from a valid channel with invalid parameters
    std::string valid_channel = "TMC9660_AIN3";
    if (adc_manager.Contains(valid_channel)) {
        result = adc_manager.ReadChannelV(valid_channel, voltage, 0, 1000); // Invalid sample count
        Logger::GetInstance().Error("AdcManagerExample", "Reading with invalid parameters: %d", static_cast<int>(result));
    }
    
    // Demonstrate recovery by reading from a valid channel
    if (adc_manager.Contains(valid_channel)) {
        result = adc_manager.ReadChannelV(valid_channel, voltage);
        Logger::GetInstance().Info("AdcManagerExample", "Recovery read from valid channel: %d (voltage=%.3fV)", 
                           static_cast<int>(result), voltage);
    }
}

/**
 * @brief Main example function.
 */
void RunAdcManagerExample() {
    Logger::GetInstance().Info("AdcManagerExample", "=== AdcManager Example - Enhanced TMC9660 Support ===");
    
    // Demonstrate all features
    DemonstrateBasicReading();
    DemonstrateTmc9660Channels();
    DemonstrateBatchReading();
    DemonstrateStatisticsAndDiagnostics();
    DemonstrateErrorHandling();
    
    Logger::GetInstance().Info("AdcManagerExample", "=== Example Complete ===");
}

//==============================================================================
// MAIN FUNCTION (for standalone testing)
//==============================================================================

#ifdef ADC_MANAGER_EXAMPLE_STANDALONE
int main() {
    RunAdcManagerExample();
    return 0;
}
#endif 