/**
 * @file LedManagerExample.cpp
 * @brief Example demonstrating the LedManager for a single WS2812 LED.
 * 
 * This example shows how to use the LedManager to control a single WS2812 LED
 * with various animations and status indications. It demonstrates the basic
 * operations, animation capabilities, and status indication patterns.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 * @copyright HardFOC
 */

#include "managers/LedManager.h"
#include "handlers/logger/Logger.h"
#include "utils/RtosMutex.h"
#include "utils/OsAbstraction.h"

#include <cstdio>
#include <cstring>

//==============================================================================
// EXAMPLE CONFIGURATION
//==============================================================================

static constexpr const char* TAG = "LedManagerExample";

// Animation demonstration intervals (in milliseconds)
static constexpr uint32_t DEMO_INTERVAL_MS = 3000;  // 3 seconds per animation
static constexpr uint32_t STATUS_INTERVAL_MS = 2000; // 2 seconds per status

//==============================================================================
// EXAMPLE FUNCTIONS
//==============================================================================

/**
 * @brief Demonstrate basic LED color operations
 */
void DemonstrateBasicOperations() noexcept {
    Logger::GetInstance().Info(TAG, "=== Basic LED Operations Demo ===");
    
    auto& led_manager = GetLedManager();
    
    // Ensure LED manager is initialized
    if (!led_manager.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize LED manager");
        return;
    }
    
    // Turn off LED first
    led_manager.TurnOff();
    OsAbstraction::DelayMs(500);
    
    // Set different colors (single LED)
    Logger::GetInstance().Info(TAG, "Setting LED 0 to RED");
    led_manager.SetColor(LedColors::RED, 0);
    OsAbstraction::DelayMs(1000);
    
    Logger::GetInstance().Info(TAG, "Setting LED 0 to GREEN");
    led_manager.SetColor(LedColors::GREEN, 0);
    OsAbstraction::DelayMs(1000);
    
    Logger::GetInstance().Info(TAG, "Setting LED 0 to BLUE");
    led_manager.SetColor(LedColors::BLUE, 0);
    OsAbstraction::DelayMs(1000);
    
    Logger::GetInstance().Info(TAG, "Setting LED 0 to WHITE");
    led_manager.SetColor(LedColors::WHITE, 0);
    OsAbstraction::DelayMs(1000);
    
    // Demonstrate multi-LED capabilities (if multiple LEDs are configured)
    uint32_t led_count = led_manager.GetLedCount();
    if (led_count > 1) {
        Logger::GetInstance().Info(TAG, "Setting all %d LEDs to YELLOW", led_count);
        led_manager.SetAllLeds(LedColors::YELLOW);
        OsAbstraction::DelayMs(1000);
        
        Logger::GetInstance().Info(TAG, "Setting all %d LEDs to CYAN", led_count);
        led_manager.SetAllLeds(LedColors::CYAN);
        OsAbstraction::DelayMs(1000);
    }
    
    // Test brightness control with percentage
    Logger::GetInstance().Info(TAG, "Testing brightness control with percentage");
    for (uint8_t brightness = 100; brightness > 0; brightness -= 20) {
        led_manager.SetBrightnessPercent(brightness);
        OsAbstraction::DelayMs(200);
    }
    
    // Test max brightness configuration
    Logger::GetInstance().Info(TAG, "Testing max brightness configuration");
    
    // Set max brightness to 50% of full brightness
    led_manager.SetMaxBrightness(128); // 50% of 255
    Logger::GetInstance().Info(TAG, "Set max brightness to 128 (50%% of full)");
    
    // Now 100% brightness will be 128 instead of 255
    led_manager.SetBrightnessPercent(100);
    OsAbstraction::DelayMs(500);
    
    // Test 50% brightness with new max
    led_manager.SetBrightnessPercent(50);
    OsAbstraction::DelayMs(500);
    
    // Reset max brightness to full
    led_manager.SetMaxBrightness(255);
    led_manager.SetBrightnessPercent(100);
    OsAbstraction::DelayMs(500);
    
    // Turn off
    led_manager.TurnOff();
    Logger::GetInstance().Info(TAG, "Basic operations demo completed");
}

/**
 * @brief Demonstrate LED animations
 */
void DemonstrateAnimations() noexcept {
    Logger::GetInstance().Info(TAG, "=== LED Animations Demo ===");
    
    auto& led_manager = GetLedManager();
    
    // Ensure LED manager is initialized
    if (!led_manager.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize LED manager");
        return;
    }
    
    // Turn off LED first
    led_manager.TurnOff();
    OsAbstraction::DelayMs(500);
    
    // Test different animations
    Logger::GetInstance().Info(TAG, "Starting BLINK animation");
    led_manager.StartAnimation(LedAnimation::BLINK, LedColors::YELLOW);
    OsAbstraction::DelayMs(DEMO_INTERVAL_MS);
    
    Logger::GetInstance().Info(TAG, "Starting BREATH animation");
    led_manager.StartAnimation(LedAnimation::BREATH, LedColors::CYAN);
    OsAbstraction::DelayMs(DEMO_INTERVAL_MS);
    
    Logger::GetInstance().Info(TAG, "Starting RAINBOW animation");
    led_manager.StartAnimation(LedAnimation::RAINBOW);
    OsAbstraction::DelayMs(DEMO_INTERVAL_MS);
    
    // Stop animation
    led_manager.StopAnimation();
    Logger::GetInstance().Info(TAG, "Animations demo completed");
}

/**
 * @brief Demonstrate status indication patterns
 */
void DemonstrateStatusIndications() noexcept {
    Logger::GetInstance().Info(TAG, "=== Status Indications Demo ===");
    
    auto& led_manager = GetLedManager();
    
    // Ensure LED manager is initialized
    if (!led_manager.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize LED manager");
        return;
    }
    
    // Turn off LED first
    led_manager.TurnOff();
    OsAbstraction::DelayMs(500);
    
    // Test different status indications
    Logger::GetInstance().Info(TAG, "Indicating BOOT status");
    led_manager.IndicateBoot();
    OsAbstraction::DelayMs(STATUS_INTERVAL_MS);
    
    Logger::GetInstance().Info(TAG, "Indicating READY status");
    led_manager.IndicateReady();
    OsAbstraction::DelayMs(STATUS_INTERVAL_MS);
    
    Logger::GetInstance().Info(TAG, "Indicating WARNING status");
    led_manager.IndicateWarning();
    OsAbstraction::DelayMs(STATUS_INTERVAL_MS);
    
    Logger::GetInstance().Info(TAG, "Indicating ERROR status");
    led_manager.IndicateError();
    OsAbstraction::DelayMs(STATUS_INTERVAL_MS);
    
    Logger::GetInstance().Info(TAG, "Indicating CALIBRATION status");
    led_manager.IndicateCalibration();
    OsAbstraction::DelayMs(STATUS_INTERVAL_MS);
    
    // Turn off
    led_manager.TurnOff();
    Logger::GetInstance().Info(TAG, "Status indications demo completed");
}

/**
 * @brief Demonstrate system diagnostics
 */
void DemonstrateDiagnostics() noexcept {
    Logger::GetInstance().Info(TAG, "=== System Diagnostics Demo ===");
    
    auto& led_manager = GetLedManager();
    
    // Ensure LED manager is initialized
    if (!led_manager.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize LED manager");
        return;
    }
    
    // Get system diagnostics
    LedSystemDiagnostics diagnostics;
    LedError result = led_manager.GetSystemDiagnostics(diagnostics);
    
    if (result == LedError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "System Diagnostics:");
        Logger::GetInstance().Info(TAG, "  System Healthy: %s", diagnostics.system_healthy ? "Yes" : "No");
        Logger::GetInstance().Info(TAG, "  LED Initialized: %s", diagnostics.led_initialized ? "Yes" : "No");
        Logger::GetInstance().Info(TAG, "  Animation Active: %s", diagnostics.animation_active ? "Yes" : "No");
        Logger::GetInstance().Info(TAG, "  Current Animation: %s", LedAnimationToString(diagnostics.current_animation));
        Logger::GetInstance().Info(TAG, "  Total Operations: %d", diagnostics.total_operations);
        Logger::GetInstance().Info(TAG, "  Successful Operations: %d", diagnostics.successful_operations);
        Logger::GetInstance().Info(TAG, "  Failed Operations: %d", diagnostics.failed_operations);
        Logger::GetInstance().Info(TAG, "  Animation Cycles: %d", diagnostics.animation_cycles);
        Logger::GetInstance().Info(TAG, "  System Uptime: %llu ms", diagnostics.system_uptime_ms);
        Logger::GetInstance().Info(TAG, "  Last Error: %s", LedErrorToString(diagnostics.last_error));
        Logger::GetInstance().Info(TAG, "  Current Brightness: %d (raw)", diagnostics.current_brightness);
    
    // Get brightness percentage for display
    uint8_t brightness_percent;
    if (led_manager.GetCurrentBrightnessPercent(brightness_percent) == LedError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "  Current Brightness: %d%%", brightness_percent);
    }
        Logger::GetInstance().Info(TAG, "  Current Color: 0x%06X", diagnostics.current_color);
    } else {
        Logger::GetInstance().Error(TAG, "Failed to get system diagnostics: %s", LedErrorToString(result));
    }
    
    // Log current state
    led_manager.LogCurrentState();
    
    Logger::GetInstance().Info(TAG, "Diagnostics demo completed");
}

/**
 * @brief Demonstrate animation update loop
 */
void DemonstrateAnimationLoop() noexcept {
    Logger::GetInstance().Info(TAG, "=== Animation Loop Demo ===");
    
    auto& led_manager = GetLedManager();
    
    // Ensure LED manager is initialized
    if (!led_manager.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize LED manager");
        return;
    }
    
    // Start a breathing animation
    Logger::GetInstance().Info(TAG, "Starting breathing animation with update loop");
    led_manager.StartAnimation(LedAnimation::BREATH, LedColors::MAGENTA);
    
    // Run animation update loop for 5 seconds
    uint64_t start_time = OsAbstraction::GetTimeMs();
    uint64_t end_time = start_time + 5000; // 5 seconds
    
    while (OsAbstraction::GetTimeMs() < end_time) {
        // Update animation (should be called periodically)
        led_manager.UpdateAnimation();
        
        // Small delay to prevent excessive CPU usage
        OsAbstraction::DelayMs(50);
    }
    
    // Stop animation
    led_manager.StopAnimation();
    Logger::GetInstance().Info(TAG, "Animation loop demo completed");
}

/**
 * @brief Demonstrate error handling
 */
void DemonstrateErrorHandling() noexcept {
    Logger::GetInstance().Info(TAG, "=== Error Handling Demo ===");
    
    auto& led_manager = GetLedManager();
    
    // Ensure LED manager is initialized
    if (!led_manager.EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize LED manager");
        return;
    }
    
    // Test invalid operations
    Logger::GetInstance().Info(TAG, "Testing error handling with invalid operations");
    
    // Try to set invalid brightness percentage
    LedError result = led_manager.SetBrightnessPercent(150); // Invalid brightness > 100%
    if (result != LedError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "Expected error for invalid brightness percentage: %s", LedErrorToString(result));
    }
    
    // Try to get current color before setting any
    LedColor color;
    result = led_manager.GetCurrentColor(color, 0);
    if (result == LedError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "Current LED 0 color: RGB(%d, %d, %d)", color.red, color.green, color.blue);
    }
    
    // Get LED count
    uint32_t led_count = led_manager.GetLedCount();
    Logger::GetInstance().Info(TAG, "LED strip has %d LEDs", led_count);
    
    // Get brightness information
    uint8_t brightness_percent, brightness_raw, max_brightness;
    result = led_manager.GetCurrentBrightnessPercent(brightness_percent);
    if (result == LedError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "Current brightness: %d%%", brightness_percent);
    }
    
    result = led_manager.GetCurrentBrightnessRaw(brightness_raw);
    if (result == LedError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "Current brightness raw: %d", brightness_raw);
    }
    
    result = led_manager.GetMaxBrightness(max_brightness);
    if (result == LedError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "Max brightness: %d", max_brightness);
    }
    
    // Test animation state queries
    bool is_active = led_manager.IsAnimationActive();
    Logger::GetInstance().Info(TAG, "Animation active: %s", is_active ? "Yes" : "No");
    
    LedAnimation current_anim;
    result = led_manager.GetCurrentAnimation(current_anim);
    if (result == LedError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "Current animation: %s", LedAnimationToString(current_anim));
    }
    
    Logger::GetInstance().Info(TAG, "Error handling demo completed");
}

//==============================================================================
// MAIN EXAMPLE FUNCTION
//==============================================================================

/**
 * @brief Main example function demonstrating LedManager usage
 */
void RunLedManagerExample() noexcept {
    Logger::GetInstance().Info(TAG, "Starting LedManager Example");
    Logger::GetInstance().Info(TAG, "This example demonstrates a single WS2812 LED with animations");
    
    // Run all demonstrations
    DemonstrateBasicOperations();
    OsAbstraction::DelayMs(1000);
    
    DemonstrateAnimations();
    OsAbstraction::DelayMs(1000);
    
    DemonstrateStatusIndications();
    OsAbstraction::DelayMs(1000);
    
    DemonstrateDiagnostics();
    OsAbstraction::DelayMs(1000);
    
    DemonstrateAnimationLoop();
    OsAbstraction::DelayMs(1000);
    
    DemonstrateErrorHandling();
    OsAbstraction::DelayMs(1000);
    
    // Final cleanup
    auto& led_manager = GetLedManager();
    led_manager.TurnOff();
    
    Logger::GetInstance().Info(TAG, "LedManager Example completed successfully");
}

//==============================================================================
// USAGE INSTRUCTIONS
//==============================================================================

/*
 * To use this example in your main application:
 * 
 * 1. Include the LedManager header:
 *    #include "managers/LedManager.h"
 * 
 * 2. Call the example function:
 *    RunLedManagerExample();
 * 
 * 3. Or use individual functions:
 *    auto& led_manager = GetLedManager();
 *    led_manager.EnsureInitialized();
 *    led_manager.IndicateBoot();
 * 
 * 4. For continuous operation, call UpdateAnimation() periodically:
 *    while (true) {
 *        led_manager.UpdateAnimation();
 *        OsAbstraction::DelayMs(50);
 *    }
 * 
 * Key Features Demonstrated:
 * - Basic color and brightness control
 * - Animation effects (blink, breath, rainbow)
 * - Status indication patterns
 * - System diagnostics and health monitoring
 * - Error handling and validation
 * - Thread-safe operations
 * - Integration with unified Logger system
 * 
 * Hardware Requirements:
 * - ESP32-C6 with RMT support
 * - Single WS2812 LED connected to GPIO pin defined in platform mapping (WS2812_LED_DAT)
 * - Default RMT channel 0 allocation
 * - Platform mapping integration for automatic pin discovery
 */ 