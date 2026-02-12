/**
 * @file ComprehensiveTemperatureExample.cpp
 * @brief Comprehensive example demonstrating all temperature sensor types in the HardFOC system.
 *
 * This example shows how to:
 * - Initialize and use ESP32 internal temperature sensor
 * - Initialize and use NTC thermistor temperature sensor
 * - Initialize and use TMC9660 internal temperature sensor
 * - Monitor multiple temperature sensors simultaneously
 * - Implement temperature-based safety monitoring
 * - Use batch operations for efficient temperature reading
 * - Handle temperature thresholds and alerts
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "managers/TemperatureManager.h"
#include "managers/AdcManager.h"
#include "handlers/logger/Logger.h"
#include "handlers/tmc9660/Tmc9660Handler.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsAbstraction.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

static const char* TAG = "CompTempExample";

//==============================================================================
// EXAMPLE CONFIGURATION
//==============================================================================

// Temperature thresholds for safety monitoring
static constexpr float CRITICAL_TEMP_CELSIUS = 85.0f;
static constexpr float WARNING_TEMP_CELSIUS = 70.0f;
static constexpr float NORMAL_TEMP_CELSIUS = 50.0f;

// Sampling configuration
static constexpr uint32_t TEMP_SAMPLE_RATE_HZ = 2;  // Sample every 500ms
static constexpr uint32_t SAFETY_CHECK_INTERVAL_MS = 1000;  // Safety check every 1s

//==============================================================================
// GLOBAL STATE
//==============================================================================

struct TemperatureState {
    float esp32_temp;
    float ntc_temp;
    float tmc9660_temp;
    bool esp32_available;
    bool ntc_available;
    bool tmc9660_available;
    uint32_t sample_count;
    uint64_t last_sample_time;
};

static TemperatureState g_temp_state = {};

//==============================================================================
// CALLBACK FUNCTIONS
//==============================================================================

void temperature_threshold_callback(BaseTemperature* sensor, float temperature_celsius, 
                                   uint32_t threshold_type, void* user_data) {
    const char* sensor_name = static_cast<const char*>(user_data);
    const char* threshold_name = (threshold_type == 0) ? "LOW" : "HIGH";
    
    Logger::GetInstance().Warn(TAG, "Temperature threshold exceeded: %s = %.2f°C (%s)", 
             sensor_name, temperature_celsius, threshold_name);
    
    // In a real application, you might:
    // - Trigger safety shutdown
    // - Send alerts
    // - Log the event
    // - Activate cooling systems
}

void temperature_reading_callback(BaseTemperature* sensor, const hf_temp_reading_t* reading, 
                                 void* user_data) {
    if (reading && reading->is_valid) {
        const char* sensor_name = static_cast<const char*>(user_data);
        Logger::GetInstance().Info(TAG, "Continuous reading: %s = %.2f°C (accuracy: %.2f°C)", 
                 sensor_name, reading->temperature_celsius, reading->accuracy_celsius);
    }
}

//==============================================================================
// TEMPERATURE MONITORING FUNCTIONS
//==============================================================================

void initialize_temperature_system() {
    Logger::GetInstance().Info(TAG, "=== Initializing Temperature Monitoring System ===");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    AdcManager& adc_manager = AdcManager::GetInstance();
    MotorController& motor_controller = MotorController::GetInstance();
    
    // Initialize managers
    if (temp_manager.EnsureInitialized() != TEMP_SUCCESS) {
        Logger::GetInstance().Error(TAG, "Failed to initialize temperature manager");
        return;
    }
    
    if (adc_manager.EnsureInitialized() != hf_adc_err_t::ADC_SUCCESS) {
        Logger::GetInstance().Error(TAG, "Failed to initialize ADC manager");
        return;
    }
    
    if (motor_controller.EnsureInitialized()) {
        Logger::GetInstance().Info(TAG, "Motor controller initialized successfully");
    } else {
        Logger::GetInstance().Warn(TAG, "Motor controller initialization failed (TMC9660 sensors may not be available)");
    }
    
    Logger::GetInstance().Info(TAG, "Temperature monitoring system initialized successfully");
    Logger::GetInstance().Info(TAG, "Note: Onboard temperature sensors are automatically registered during initialization");
}

void check_esp32_temperature_sensor() {
    Logger::GetInstance().Info(TAG, "--- Checking ESP32 Internal Temperature Sensor ---");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Check if ESP32 internal temperature sensor is available (should be auto-registered)
    if (temp_manager.IsSensorRegistered("esp32_internal")) {
        g_temp_state.esp32_available = true;
        Logger::GetInstance().Info(TAG, "ESP32 internal temperature sensor is available (auto-registered)");
        
        // Set up threshold monitoring for ESP32
        temp_manager.SetThresholds("esp32_internal", 0.0f, WARNING_TEMP_CELSIUS);
        temp_manager.EnableThresholdMonitoring("esp32_internal", temperature_threshold_callback, 
                                             const_cast<char*>("ESP32"));
        
        // Start continuous monitoring
        temp_manager.StartContinuousMonitoring("esp32_internal", TEMP_SAMPLE_RATE_HZ, 
                                             temperature_reading_callback, 
                                             const_cast<char*>("ESP32"));
    } else {
        Logger::GetInstance().Warn(TAG, "ESP32 internal temperature sensor not available");
        g_temp_state.esp32_available = false;
    }
}

void register_ntc_temperature_sensor() {
    Logger::GetInstance().Info(TAG, "--- Registering NTC Thermistor Temperature Sensor ---");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // NTC configuration for NTCG163JFT103FT1S
    ntc_temp_handler_config_t ntc_config = NTC_TEMP_HANDLER_CONFIG_DEFAULT_NTCG163JFT103FT1S();
    ntc_config.enable_threshold_monitoring = true;
    ntc_config.low_threshold_celsius = 0.0f;
    ntc_config.high_threshold_celsius = WARNING_TEMP_CELSIUS;
    
    // Register NTC temperature sensor (assuming ADC channel "ntc_thermistor" exists)
    if (temp_manager.RegisterNtcTemperatureSensor("ntc_sensor", "ntc_thermistor", ntc_config) == TEMP_SUCCESS) {
        g_temp_state.ntc_available = true;
        Logger::GetInstance().Info(TAG, "NTC temperature sensor registered successfully");
        
        // Set up threshold monitoring for NTC
        temp_manager.SetThresholds("ntc_sensor", 0.0f, WARNING_TEMP_CELSIUS);
        temp_manager.EnableThresholdMonitoring("ntc_sensor", temperature_threshold_callback, 
                                             const_cast<char*>("NTC"));
        
        // Start continuous monitoring
        temp_manager.StartContinuousMonitoring("ntc_sensor", TEMP_SAMPLE_RATE_HZ, 
                                             temperature_reading_callback, 
                                             const_cast<char*>("NTC"));
    } else {
        Logger::GetInstance().Error(TAG, "Failed to register NTC temperature sensor");
        g_temp_state.ntc_available = false;
    }
}

void check_tmc9660_temperature_sensor() {
    Logger::GetInstance().Info(TAG, "--- Checking TMC9660 Internal Temperature Sensor ---");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Check if TMC9660 internal temperature sensor is available (should be auto-registered)
    if (temp_manager.IsSensorRegistered("tmc9660_internal")) {
        g_temp_state.tmc9660_available = true;
        Logger::GetInstance().Info(TAG, "TMC9660 internal temperature sensor is available (auto-registered)");
        
        // Set up threshold monitoring for TMC9660
        temp_manager.SetThresholds("tmc9660_internal", 0.0f, WARNING_TEMP_CELSIUS);
        temp_manager.EnableThresholdMonitoring("tmc9660_internal", temperature_threshold_callback, 
                                             const_cast<char*>("TMC9660"));
        
        // Start continuous monitoring
        temp_manager.StartContinuousMonitoring("tmc9660_internal", TEMP_SAMPLE_RATE_HZ, 
                                             temperature_reading_callback, 
                                             const_cast<char*>("TMC9660"));
    } else {
        Logger::GetInstance().Warn(TAG, "TMC9660 internal temperature sensor not available (MotorController may not be initialized)");
        g_temp_state.tmc9660_available = false;
    }
}

void read_all_temperatures() {
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Read ESP32 temperature
    if (g_temp_state.esp32_available) {
        if (temp_manager.ReadTemperatureCelsius("esp32_internal", &g_temp_state.esp32_temp) == TEMP_SUCCESS) {
            Logger::GetInstance().Info(TAG, "ESP32 Temperature: %.2f°C", g_temp_state.esp32_temp);
        } else {
            Logger::GetInstance().Warn(TAG, "Failed to read ESP32 temperature");
        }
    }
    
    // Read NTC temperature (TMC9660 AIN3 thermistor)
    if (g_temp_state.ntc_available) {
        if (temp_manager.ReadTemperatureCelsius("tmc9660_thermistor", &g_temp_state.ntc_temp) == TEMP_SUCCESS) {
            Logger::GetInstance().Info(TAG, "TMC9660 AIN3 Thermistor Temperature: %.2f°C", g_temp_state.ntc_temp);
        } else {
            Logger::GetInstance().Warn(TAG, "Failed to read TMC9660 AIN3 thermistor temperature");
        }
    }
    
    // Read TMC9660 internal chip temperature
    if (g_temp_state.tmc9660_available) {
        if (temp_manager.ReadTemperatureCelsius("tmc9660_chip", &g_temp_state.tmc9660_temp) == TEMP_SUCCESS) {
            Logger::GetInstance().Info(TAG, "TMC9660 Internal Chip Temperature: %.2f°C", g_temp_state.tmc9660_temp);
        } else {
            Logger::GetInstance().Warn(TAG, "Failed to read TMC9660 internal chip temperature");
        }
    }
    
    g_temp_state.sample_count++;
    g_temp_state.last_sample_time = os_time_get();
}

void perform_batch_temperature_reading() {
    Logger::GetInstance().Info(TAG, "--- Performing Batch Temperature Reading ---");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Collect available sensor names
    std::vector<std::string_view> sensor_names;
    if (g_temp_state.esp32_available) {
        sensor_names.push_back("esp32_internal");
    }
    if (g_temp_state.ntc_available) {
        sensor_names.push_back("tmc9660_thermistor");
    }
    if (g_temp_state.tmc9660_available) {
        sensor_names.push_back("tmc9660_chip");
    }
    
    if (sensor_names.empty()) {
        Logger::GetInstance().Warn(TAG, "No temperature sensors available for batch reading");
        return;
    }
    
    // Perform batch reading
    std::vector<float> temperatures_celsius;
    if (temp_manager.ReadMultipleTemperaturesCelsius(sensor_names, temperatures_celsius) == TEMP_SUCCESS) {
        Logger::GetInstance().Info(TAG, "Batch temperature readings:");
        for (size_t i = 0; i < sensor_names.size(); ++i) {
            Logger::GetInstance().Info(TAG, "  %s: %.2f°C", std::string(sensor_names[i]).c_str(), temperatures_celsius[i]);
        }
    } else {
        Logger::GetInstance().Error(TAG, "Batch temperature reading failed");
    }
}

void check_temperature_safety() {
    Logger::GetInstance().Info(TAG, "--- Temperature Safety Check ---");
    
    bool critical_temp_detected = false;
    bool warning_temp_detected = false;
    
    // Check ESP32 temperature
    if (g_temp_state.esp32_available) {
        if (g_temp_state.esp32_temp >= CRITICAL_TEMP_CELSIUS) {
            Logger::GetInstance().Error(TAG, "CRITICAL: ESP32 temperature = %.2f°C", g_temp_state.esp32_temp);
            critical_temp_detected = true;
        } else if (g_temp_state.esp32_temp >= WARNING_TEMP_CELSIUS) {
            Logger::GetInstance().Warn(TAG, "WARNING: ESP32 temperature = %.2f°C", g_temp_state.esp32_temp);
            warning_temp_detected = true;
        }
    }
    
    // Check TMC9660 AIN3 thermistor temperature
    if (g_temp_state.ntc_available) {
        if (g_temp_state.ntc_temp >= CRITICAL_TEMP_CELSIUS) {
            Logger::GetInstance().Error(TAG, "CRITICAL: TMC9660 AIN3 thermistor temperature = %.2f°C", g_temp_state.ntc_temp);
            critical_temp_detected = true;
        } else if (g_temp_state.ntc_temp >= WARNING_TEMP_CELSIUS) {
            Logger::GetInstance().Warn(TAG, "WARNING: TMC9660 AIN3 thermistor temperature = %.2f°C", g_temp_state.ntc_temp);
            warning_temp_detected = true;
        }
    }
    
    // Check TMC9660 internal chip temperature
    if (g_temp_state.tmc9660_available) {
        if (g_temp_state.tmc9660_temp >= CRITICAL_TEMP_CELSIUS) {
            Logger::GetInstance().Error(TAG, "CRITICAL: TMC9660 internal chip temperature = %.2f°C", g_temp_state.tmc9660_temp);
            critical_temp_detected = true;
        } else if (g_temp_state.tmc9660_temp >= WARNING_TEMP_CELSIUS) {
            Logger::GetInstance().Warn(TAG, "WARNING: TMC9660 internal chip temperature = %.2f°C", g_temp_state.tmc9660_temp);
            warning_temp_detected = true;
        }
    }
    
    // Take action based on temperature conditions
    if (critical_temp_detected) {
        Logger::GetInstance().Error(TAG, "CRITICAL TEMPERATURE DETECTED - EMERGENCY SHUTDOWN RECOMMENDED");
        // In a real application, you would:
        // - Trigger emergency shutdown
        // - Activate emergency cooling
        // - Send emergency alerts
        // - Log critical event
    } else if (warning_temp_detected) {
        Logger::GetInstance().Warn(TAG, "WARNING TEMPERATURE DETECTED - MONITOR CLOSELY");
        // In a real application, you would:
        // - Increase cooling
        // - Reduce load
        // - Send warning alerts
        // - Log warning event
    } else {
        Logger::GetInstance().Info(TAG, "All temperatures within normal range");
    }
}

void display_system_statistics() {
    Logger::GetInstance().Info(TAG, "--- System Temperature Statistics ---");
    
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    
    // Get system diagnostics
    TempSystemDiagnostics system_diagnostics = {};
    if (temp_manager.GetSystemDiagnostics(system_diagnostics) == TEMP_SUCCESS) {
        Logger::GetInstance().Info(TAG, "System Diagnostics:");
        Logger::GetInstance().Info(TAG, "  Total Sensors: %u", system_diagnostics.total_sensors_registered);
        Logger::GetInstance().Info(TAG, "  Total Operations: %u", system_diagnostics.total_operations);
        Logger::GetInstance().Info(TAG, "  Successful Operations: %u", system_diagnostics.successful_operations);
        Logger::GetInstance().Info(TAG, "  Failed Operations: %u", system_diagnostics.failed_operations);
        Logger::GetInstance().Info(TAG, "  System Min Temp: %.2f°C", system_diagnostics.system_min_temp_celsius);
        Logger::GetInstance().Info(TAG, "  System Max Temp: %.2f°C", system_diagnostics.system_max_temp_celsius);
        Logger::GetInstance().Info(TAG, "  System Avg Temp: %.2f°C", system_diagnostics.system_avg_temp_celsius);
        Logger::GetInstance().Info(TAG, "  System Uptime: %llu ms", system_diagnostics.system_uptime_ms);
        
        if (system_diagnostics.total_operations > 0) {
            float success_rate = (float)system_diagnostics.successful_operations / system_diagnostics.total_operations * 100.0f;
            Logger::GetInstance().Info(TAG, "  Success Rate: %.2f%%", success_rate);
        }
    }
    
    // Display sensor-specific statistics
    if (g_temp_state.esp32_available) {
        hf_temp_statistics_t esp32_stats = {};
        if (temp_manager.GetSensorStatistics("esp32_internal", esp32_stats) == TEMP_SUCCESS) {
            Logger::GetInstance().Info(TAG, "ESP32 Sensor Statistics:");
            Logger::GetInstance().Info(TAG, "  Total Operations: %u", esp32_stats.total_operations);
            Logger::GetInstance().Info(TAG, "  Temperature Readings: %u", esp32_stats.temperature_readings);
            Logger::GetInstance().Info(TAG, "  Min Temperature: %.2f°C", esp32_stats.min_temperature_celsius);
            Logger::GetInstance().Info(TAG, "  Max Temperature: %.2f°C", esp32_stats.max_temperature_celsius);
            Logger::GetInstance().Info(TAG, "  Avg Temperature: %.2f°C", esp32_stats.avg_temperature_celsius);
        }
    }
    
    if (g_temp_state.ntc_available) {
        hf_temp_statistics_t ntc_stats = {};
        if (temp_manager.GetSensorStatistics("tmc9660_thermistor", ntc_stats) == TEMP_SUCCESS) {
            Logger::GetInstance().Info(TAG, "TMC9660 AIN3 Thermistor Statistics:");
            Logger::GetInstance().Info(TAG, "  Total Operations: %u", ntc_stats.total_operations);
            Logger::GetInstance().Info(TAG, "  Temperature Readings: %u", ntc_stats.temperature_readings);
            Logger::GetInstance().Info(TAG, "  Min Temperature: %.2f°C", ntc_stats.min_temperature_celsius);
            Logger::GetInstance().Info(TAG, "  Max Temperature: %.2f°C", ntc_stats.max_temperature_celsius);
            Logger::GetInstance().Info(TAG, "  Avg Temperature: %.2f°C", ntc_stats.avg_temperature_celsius);
        }
    }
    
    if (g_temp_state.tmc9660_available) {
        hf_temp_statistics_t tmc9660_stats = {};
        if (temp_manager.GetSensorStatistics("tmc9660_chip", tmc9660_stats) == TEMP_SUCCESS) {
            Logger::GetInstance().Info(TAG, "TMC9660 Sensor Statistics:");
            Logger::GetInstance().Info(TAG, "  Total Operations: %u", tmc9660_stats.total_operations);
            Logger::GetInstance().Info(TAG, "  Temperature Readings: %u", tmc9660_stats.temperature_readings);
            Logger::GetInstance().Info(TAG, "  Min Temperature: %.2f°C", tmc9660_stats.min_temperature_celsius);
            Logger::GetInstance().Info(TAG, "  Max Temperature: %.2f°C", tmc9660_stats.max_temperature_celsius);
            Logger::GetInstance().Info(TAG, "  Avg Temperature: %.2f°C", tmc9660_stats.avg_temperature_celsius);
        }
    }
}

//==============================================================================
// MAIN EXAMPLE FUNCTION
//==============================================================================

extern "C" void app_main() {
    Logger::GetInstance().Info(TAG, "Comprehensive Temperature Monitoring System Starting...");
    
    // Initialize logger
    Logger::GetInstance().Initialize();
    
    // Initialize temperature system
    initialize_temperature_system();
    
    // Check which sensors are available (automatically registered during initialization)
    TemperatureManager& temp_manager = TemperatureManager::GetInstance();
    g_temp_state.esp32_available = temp_manager.IsSensorRegistered("esp32_internal");
    g_temp_state.ntc_available = temp_manager.IsSensorRegistered("tmc9660_thermistor");
    g_temp_state.tmc9660_available = temp_manager.IsSensorRegistered("tmc9660_chip");
    
    // Display sensor availability
    Logger::GetInstance().Info(TAG, "Automatically Registered Sensors:");
    Logger::GetInstance().Info(TAG, "  ESP32 Internal: %s", g_temp_state.esp32_available ? "AVAILABLE" : "NOT AVAILABLE");
    Logger::GetInstance().Info(TAG, "  TMC9660 AIN3 Thermistor: %s", g_temp_state.ntc_available ? "AVAILABLE" : "NOT AVAILABLE");
    Logger::GetInstance().Info(TAG, "  TMC9660 Internal Chip: %s", g_temp_state.tmc9660_available ? "AVAILABLE" : "NOT AVAILABLE");
    
    // Main monitoring loop
    uint32_t loop_count = 0;
    const uint32_t max_loops = 20; // Run for 20 iterations
    
    while (loop_count < max_loops) {
        Logger::GetInstance().Info(TAG, "=== Monitoring Cycle %u/%u ===", loop_count + 1, max_loops);
        
        // Read all temperatures
        read_all_temperatures();
        
        // Perform batch reading demonstration
        if (loop_count % 5 == 0) { // Every 5th cycle
            perform_batch_temperature_reading();
        }
        
        // Check temperature safety
        check_temperature_safety();
        
        // Display statistics periodically
        if (loop_count % 10 == 0) { // Every 10th cycle
            display_system_statistics();
        }
        
        // Wait before next cycle
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2 second cycle
        loop_count++;
    }
    
    Logger::GetInstance().Info(TAG, "Comprehensive Temperature Monitoring System Completed!");
    Logger::GetInstance().Info(TAG, "Total samples taken: %u", g_temp_state.sample_count);
} 