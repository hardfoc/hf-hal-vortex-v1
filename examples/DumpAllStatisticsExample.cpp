/**
 * @file DumpAllStatisticsExample.cpp
 * @brief Example demonstrating how to dump statistics from all singleton managers.
 * 
 * This example shows how to use the DumpStatistics() method available on all
 * singleton managers to get comprehensive system health and performance information.
 * This is useful for debugging, monitoring, and system diagnostics.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 * @copyright HardFOC
 */

#include "managers/LedManager.h"
#include "managers/GpioManager.h"
#include "managers/AdcManager.h"
#include "managers/CommChannelsManager.h"
#include "managers/ImuManager.h"
#include "managers/EncoderManager.h"
#include "managers/MotorController.h"
#include "handlers/logger/Logger.h"
#include "handlers/tmc9660/Tmc9660Handler.h"
#include "handlers/as5047u/As5047uHandler.h"
#include "handlers/bno08x/Bno08xHandler.h"
#include "handlers/pcal95555/Pcal95555Handler.h"
#include "utils/OsAbstraction.h"

#include <cstdio>

//==============================================================================
// EXAMPLE CONFIGURATION
//==============================================================================

static constexpr const char* TAG = "DumpAllStatisticsExample";

//==============================================================================
// STATISTICS DUMPING FUNCTIONS
//==============================================================================

/**
 * @brief Dump statistics from all singleton managers.
 * 
 * This function calls DumpStatistics() on all available singleton managers
 * to provide a comprehensive view of the system state, performance metrics,
 * and health information.
 */
void DumpAllSystemStatistics() noexcept {
    Logger::GetInstance().Info(TAG, "=== SYSTEM-WIDE STATISTICS DUMP ===");
    Logger::GetInstance().Info(TAG, "Starting comprehensive statistics dump of all managers...");
    
    // Add separator for clarity
    Logger::GetInstance().Info(TAG, "");
    Logger::GetInstance().Info(TAG, "########################################");
    Logger::GetInstance().Info(TAG, "#     SINGLETON MANAGERS STATISTICS   #");
    Logger::GetInstance().Info(TAG, "########################################");
    Logger::GetInstance().Info(TAG, "");
    
    // 1. Logger Statistics (first, since others depend on it)
    Logger::GetInstance().Info(TAG, "--- LOGGER MANAGER ---");
    Logger::GetInstance().DumpStatistics();
    Logger::GetInstance().Info(TAG, "");
    
    // 2. LED Manager Statistics
    Logger::GetInstance().Info(TAG, "--- LED MANAGER ---");
    LedManager::GetInstance().DumpStatistics();
    Logger::GetInstance().Info(TAG, "");
    
    // 3. GPIO Manager Statistics
    Logger::GetInstance().Info(TAG, "--- GPIO MANAGER ---");
    GpioManager::GetInstance().DumpStatistics();
    Logger::GetInstance().Info(TAG, "");
    
    // 4. ADC Manager Statistics
    Logger::GetInstance().Info(TAG, "--- ADC MANAGER ---");
    AdcManager::GetInstance().DumpStatistics();
    Logger::GetInstance().Info(TAG, "");
    
    // 5. Communication Channels Manager Statistics
    Logger::GetInstance().Info(TAG, "--- COMM CHANNELS MANAGER ---");
    CommChannelsManager::GetInstance().DumpStatistics();
    Logger::GetInstance().Info(TAG, "");
    
    // 6. IMU Manager Statistics
    Logger::GetInstance().Info(TAG, "--- IMU MANAGER ---");
    ImuManager::GetInstance().DumpStatistics();
    Logger::GetInstance().Info(TAG, "");
    
    // 7. Encoder Manager Statistics
    Logger::GetInstance().Info(TAG, "--- ENCODER MANAGER ---");
    EncoderManager::GetInstance().DumpStatistics();
    Logger::GetInstance().Info(TAG, "");
    
    // 8. Motor Controller Statistics
    Logger::GetInstance().Info(TAG, "--- MOTOR CONTROLLER ---");
    MotorController::GetInstance().DumpStatistics();
    Logger::GetInstance().Info(TAG, "");
    
    // Summary
    Logger::GetInstance().Info(TAG, "########################################");
    Logger::GetInstance().Info(TAG, "#        STATISTICS DUMP COMPLETE     #");
    Logger::GetInstance().Info(TAG, "########################################");
    Logger::GetInstance().Info(TAG, "All singleton manager statistics have been dumped.");
    Logger::GetInstance().Info(TAG, "Check the log output above for detailed system information.");
}

/**
 * @brief Dump statistics from a specific manager.
 * 
 * @param manager_name Name of the manager to dump statistics for
 */
void DumpSpecificManagerStatistics(const char* manager_name) noexcept {
    Logger::GetInstance().Info(TAG, "=== SPECIFIC MANAGER STATISTICS ===");
    Logger::GetInstance().Info(TAG, "Dumping statistics for: %s", manager_name);
    
    if (strcmp(manager_name, "led") == 0 || strcmp(manager_name, "LedManager") == 0) {
        LedManager::GetInstance().DumpStatistics();
    }
    else if (strcmp(manager_name, "gpio") == 0 || strcmp(manager_name, "GpioManager") == 0) {
        GpioManager::GetInstance().DumpStatistics();
    }
    else if (strcmp(manager_name, "adc") == 0 || strcmp(manager_name, "AdcManager") == 0) {
        AdcManager::GetInstance().DumpStatistics();
    }
    else if (strcmp(manager_name, "comm") == 0 || strcmp(manager_name, "CommChannelsManager") == 0) {
        CommChannelsManager::GetInstance().DumpStatistics();
    }
    else if (strcmp(manager_name, "imu") == 0 || strcmp(manager_name, "ImuManager") == 0) {
        ImuManager::GetInstance().DumpStatistics();
    }
    else if (strcmp(manager_name, "encoder") == 0 || strcmp(manager_name, "EncoderManager") == 0) {
        EncoderManager::GetInstance().DumpStatistics();
    }
    else if (strcmp(manager_name, "motor") == 0 || strcmp(manager_name, "MotorController") == 0) {
        MotorController::GetInstance().DumpStatistics();
    }
    else if (strcmp(manager_name, "logger") == 0 || strcmp(manager_name, "Logger") == 0) {
        Logger::GetInstance().DumpStatistics();
    }
    else {
        Logger::GetInstance().Error(TAG, "Unknown manager: %s", manager_name);
        Logger::GetInstance().Info(TAG, "Available managers: led, gpio, adc, comm, imu, encoder, motor, logger");
    }
}

/**
 * @brief Initialize all managers before dumping statistics.
 * 
 * This ensures all managers are properly initialized so their statistics
 * are meaningful and complete.
 */
void InitializeAllManagers() noexcept {
    Logger::GetInstance().Info(TAG, "Initializing all managers for statistics demo...");
    
    // Initialize Logger first
    if (!Logger::GetInstance().IsInitialized()) {
        Logger::GetInstance().Initialize();
    }
    
    // Initialize LED Manager
    if (!LedManager::GetInstance().EnsureInitialized()) {
        Logger::GetInstance().Warn(TAG, "LED Manager initialization failed");
    }
    
    // Initialize GPIO Manager
    if (!GpioManager::GetInstance().EnsureInitialized()) {
        Logger::GetInstance().Warn(TAG, "GPIO Manager initialization failed");
    }
    
    // Initialize ADC Manager
    if (!AdcManager::GetInstance().EnsureInitialized()) {
        Logger::GetInstance().Warn(TAG, "ADC Manager initialization failed");
    }
    
    // Initialize Communication Channels Manager
    if (!CommChannelsManager::GetInstance().EnsureInitialized()) {
        Logger::GetInstance().Warn(TAG, "CommChannels Manager initialization failed");
    }
    
    // Initialize IMU Manager
    if (!ImuManager::GetInstance().EnsureInitialized()) {
        Logger::GetInstance().Warn(TAG, "IMU Manager initialization failed");
    }
    
    // Initialize Encoder Manager
    if (!EncoderManager::GetInstance().EnsureInitialized()) {
        Logger::GetInstance().Warn(TAG, "Encoder Manager initialization failed");
    }
    
    // Initialize Motor Controller
    if (!MotorController::GetInstance().EnsureInitialized()) {
        Logger::GetInstance().Warn(TAG, "Motor Controller initialization failed");
    }
    
    Logger::GetInstance().Info(TAG, "Manager initialization complete");
}

/**
 * @brief Dump diagnostics from all handler classes with diagnostic capabilities.
 * 
 * This function demonstrates how to dump diagnostics from hardware handler classes
 * that have DumpDiagnostics() methods available.
 */
void DumpAllHandlerDiagnostics() noexcept {
    Logger::GetInstance().Info(TAG, "=== HARDWARE HANDLER DIAGNOSTICS ===");
    Logger::GetInstance().Info(TAG, "Dumping diagnostics from all hardware handlers...");
    
    // TMC9660 Handler Diagnostics
    Logger::GetInstance().Info(TAG, "--- TMC9660 HANDLER ---");
    // Note: This would require access to a TMC9660Handler instance
    // In practice, you would get this from the MotorController
    // auto& motor_controller = MotorController::GetInstance();
    // auto* tmc_handler = motor_controller.handler(0);
    // if (tmc_handler) {
    //     tmc_handler->DumpDiagnostics();
    // }
    Logger::GetInstance().Info(TAG, "TMC9660 handler diagnostics would be called here");
    Logger::GetInstance().Info(TAG, "");
    
    // AS5047U Handler Diagnostics
    Logger::GetInstance().Info(TAG, "--- AS5047U HANDLER ---");
    // Note: This would require access to an AS5047U Handler instance
    Logger::GetInstance().Info(TAG, "AS5047U handler diagnostics would be called here");
    Logger::GetInstance().Info(TAG, "");
    
    // BNO08x Handler Diagnostics
    Logger::GetInstance().Info(TAG, "--- BNO08X HANDLER ---");
    // Note: This would require access to a BNO08x Handler instance
    Logger::GetInstance().Info(TAG, "BNO08x handler diagnostics would be called here");
    Logger::GetInstance().Info(TAG, "");
    
    // PCAL95555 Handler Diagnostics
    Logger::GetInstance().Info(TAG, "--- PCAL95555 HANDLER ---");
    // Note: This would require access to a PCAL95555 Handler instance
    Logger::GetInstance().Info(TAG, "PCAL95555 handler diagnostics would be called here");
    Logger::GetInstance().Info(TAG, "");
    
    Logger::GetInstance().Info(TAG, "Handler diagnostics dump complete");
    Logger::GetInstance().Info(TAG, "Note: Actual handler diagnostics require active handler instances");
}

/**
 * @brief Demonstrate periodic statistics dumping.
 * 
 * This function shows how to periodically dump statistics for monitoring
 * during system operation.
 * 
 * @param interval_ms Interval between dumps in milliseconds
 * @param dump_count Number of dumps to perform
 */
void DemonstratePeriodicStatisticsDump(uint32_t interval_ms = 5000, int dump_count = 3) noexcept {
    Logger::GetInstance().Info(TAG, "=== PERIODIC STATISTICS DEMO ===");
    Logger::GetInstance().Info(TAG, "Performing %d statistics dumps every %d ms", dump_count, interval_ms);
    
    for (int i = 0; i < dump_count; ++i) {
        Logger::GetInstance().Info(TAG, "--- DUMP %d of %d ---", i + 1, dump_count);
        
        // Dump a subset of managers for brevity
        Logger::GetInstance().Info(TAG, "LED Manager Status:");
        LedManager::GetInstance().DumpStatistics();
        
        Logger::GetInstance().Info(TAG, "GPIO Manager Status:");
        GpioManager::GetInstance().DumpStatistics();
        
        if (i < dump_count - 1) {
            Logger::GetInstance().Info(TAG, "Waiting %d ms until next dump...", interval_ms);
            OsAbstraction::DelayMs(interval_ms);
        }
    }
    
    Logger::GetInstance().Info(TAG, "Periodic statistics demo complete");
}

//==============================================================================
// MAIN EXAMPLE FUNCTIONS
//==============================================================================

/**
 * @brief Main function demonstrating comprehensive statistics dumping.
 */
void RunDumpAllStatisticsExample() noexcept {
    Logger::GetInstance().Info(TAG, "Starting DumpAllStatistics Example");
    Logger::GetInstance().Info(TAG, "This example demonstrates how to dump statistics from all singleton managers");
    
    // Initialize all managers first
    InitializeAllManagers();
    
    // Wait a moment for initialization to complete
    OsAbstraction::DelayMs(1000);
    
    // Perform some operations to generate statistics
    Logger::GetInstance().Info(TAG, "Performing sample operations to generate statistics...");
    
    // LED operations
    auto& led_manager = LedManager::GetInstance();
    led_manager.SetColor(LedColors::RED);
    OsAbstraction::DelayMs(100);
    led_manager.SetColor(LedColors::GREEN);
    OsAbstraction::DelayMs(100);
    led_manager.StartAnimation(LedAnimation::BLINK, LedColors::BLUE);
    OsAbstraction::DelayMs(500);
    led_manager.StopAnimation();
    
    // GPIO operations
    auto& gpio_manager = GpioManager::GetInstance();
    // Note: Add GPIO operations here if needed
    
    // Wait a moment for operations to complete
    OsAbstraction::DelayMs(500);
    
    // Now dump all statistics
    DumpAllSystemStatistics();
    
    // Demonstrate handler diagnostics
    Logger::GetInstance().Info(TAG, "");
    DumpAllHandlerDiagnostics();
    
    // Demonstrate specific manager dump
    Logger::GetInstance().Info(TAG, "");
    Logger::GetInstance().Info(TAG, "--- DEMONSTRATING SPECIFIC MANAGER DUMP ---");
    DumpSpecificManagerStatistics("led");
    
    // Demonstrate periodic dumping (commented out to avoid long delays)
    // DemonstratePeriodicStatisticsDump(2000, 2);
    
    Logger::GetInstance().Info(TAG, "DumpAllStatistics Example completed successfully");
}

//==============================================================================
// USAGE INSTRUCTIONS
//==============================================================================

/*
 * To use this statistics dumping functionality in your application:
 * 
 * 1. Include this file or copy the relevant functions:
 *    #include "examples/DumpAllStatisticsExample.cpp"
 * 
 * 2. Call the main example function:
 *    RunDumpAllStatisticsExample();
 * 
 * 3. Or use individual functions for specific needs:
 *    
 *    // Dump all manager statistics
 *    DumpAllSystemStatistics();
 *    
 *    // Dump specific manager statistics
 *    DumpSpecificManagerStatistics("led");
 *    
 *    // Periodic monitoring
 *    DemonstratePeriodicStatisticsDump(10000, 5); // Every 10 seconds, 5 times
 * 
 * 4. For debugging specific issues:
 *    - Use DumpSpecificManagerStatistics() to focus on problem areas
 *    - Use DemonstratePeriodicStatisticsDump() to monitor during testing
 *    - Use DumpAllSystemStatistics() for comprehensive system health checks
 * 
 * Key Features Demonstrated:
 * - Comprehensive system health monitoring
 * - Manager-specific statistics analysis
 * - Hardware handler diagnostics
 * - Operation performance metrics
 * - Error and success rate tracking
 * - Memory usage analysis
 * - Hardware configuration status
 * - Communication interface status
 * - Sensor calibration and health data
 * - Thread-safe statistics collection
 * - Robust statistics dumping with error handling
 * 
 * Use Cases:
 * - System debugging and troubleshooting
 * - Performance monitoring and optimization
 * - Health checks during operation
 * - Development and testing validation
 * - Production system monitoring
 * - Capacity planning and resource analysis
 */