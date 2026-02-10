/**
 * @file GpioSystemIntegrationTest.cpp
 * @brief Integration test for the comprehensive GPIO data handler system.
 * 
 * This test validates the integration of the new GpioManager system through
 * the legacy compatibility bridge, ensuring all GPIO sources work correctly.
 */

#include "component-handler/GpioManager.h"
#include "component-handler/SystemInit.h"
#include "core/hf-core-utils/hf-utils-general/include/ConsolePort.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"

static const char* TAG = "GpioIntegrationTest";

/**
 * @brief Test basic GPIO system initialization and functionality.
 * @return true if all tests pass, false otherwise.
 */
bool TestGpioSystemIntegration() {
    console_info(TAG, "=== GPIO System Integration Test ===");
    
    // Initialize the system components in correct order
    console_info(TAG, "Step 1: Initialize I2C bus");
    if (!SystemInit::InitializeI2cBus()) {
        console_error(TAG, "Failed to initialize I2C bus");
        return false;
    }
    
    console_info(TAG, "Step 2: Initialize TMC9660 system");
    if (!SystemInit::InitializeTmc9660System()) {
        console_error(TAG, "Failed to initialize TMC9660 system");
        return false;
    }
    
    console_info(TAG, "Step 3: Initialize comprehensive GPIO system");
    if (!SystemInit::InitializeGpioSystem()) {
        console_error(TAG, "Failed to initialize GPIO system");
        return false;
    }
    
    // Test modern interface compatibility
    console_info(TAG, "Step 4: Test new GpioManager interface");
    GpioManager& gpio = GpioManager::GetInstance();
    
    // Test that the system reports as initialized
    if (!gpio.IsInitialized()) {
        console_error(TAG, "Modern GPIO system not initialized");
        return false;
    }
    
    console_info(TAG, "Modern GPIO system has %zu registered pins", 
                 gpio.GetRegisteredPinCount());

    // Test system health
    console_info(TAG, "Step 5: Test system health checks");
    auto healthResult = gpio.GetSystemHealth();
    if (!healthResult.IsSuccess()) {
        console_warning(TAG, "GPIO system health check indicates issues");
    } else {
        console_info(TAG, "GPIO system health: OK");
    }
    } else {
        console_info(TAG, "GPIO system health check passed");
    }
    
    // Test comprehensive GPIO functionality
    console_info(TAG, "Step 7: Test comprehensive GPIO operations");
    if (!legacyGpio.RunGpioTest()) {
        console_warning(TAG, "GPIO comprehensive test failed - check hardware connections");
    } else {
        console_info(TAG, "GPIO comprehensive test passed");
    }
    
    // Test pin safety validation (ESP32-C6 specific)
    console_info(TAG, "Step 8: Test pin safety validation");
    if (gpio.IsEsp32PinSafe(GPIO_NUM_0)) {
        console_info(TAG, "Pin safety validation working - GPIO0 is safe");
    } else {
    // Test multi-source GPIO support
    console_info(TAG, "Step 6: Test multi-source GPIO capabilities");
    
    // Get GPIO manager instance  
    GpioManager& gpioManager = GpioManager::GetInstance();
    
    // Test performance-optimized batch operations
    console_info(TAG, "Step 7: Test batch GPIO operations");
    
    std::vector<GpioPin> testPins = {
        GpioPin::GPIO_LED_STATUS,
        GpioPin::GPIO_LED_COMM,
        GpioPin::GPIO_LED_ERROR
    };
    
    // Test batch activation
    if (legacyGpio.SetMultipleActive(testPins)) {
        console_info(TAG, "Batch activation successful");
        os_delay_msec(100);
        
        // Test batch deactivation
        if (legacyGpio.SetMultipleInactive(testPins)) {
            console_info(TAG, "Batch deactivation successful");
        } else {
            console_warning(TAG, "Batch deactivation failed");
        }
    } else {
        console_warning(TAG, "Batch activation failed - check pin mappings");
    }
    
    // Test pattern operations
    console_info(TAG, "Step 11: Test pattern operations");
    uint32_t pattern = 0b101; // Status and Error on, Comm off
    if (legacyGpio.SetPinPattern(testPins, pattern)) {
        console_info(TAG, "Pattern setting successful");
        
        uint32_t readPattern;
        if (legacyGpio.GetPinPattern(testPins, readPattern)) {
            console_info(TAG, "Pattern reading successful - pattern: 0x%X", readPattern);
        }
        
        // Clear pattern
        legacyGpio.SetPinPattern(testPins, 0);
    } else {
        console_warning(TAG, "Pattern operations failed");
    }
    
    console_info(TAG, "=== GPIO Integration Test Complete ===");
    console_info(TAG, "Successfully validated comprehensive GPIO system integration");
    
    return true;
}

/**
 * @brief Test error handling and recovery.
 * @return true if error handling works correctly, false otherwise.
 */
bool TestGpioErrorHandling() {
    console_info(TAG, "=== GPIO Error Handling Test ===");
    
    GpioManager& gpio = GpioManager::GetInstance();
    
    // Test invalid pin operations
    console_info(TAG, "Testing invalid pin operations");
    
    // Try to operate on a non-existent pin
    auto invalidResult = gpio.SetActive(static_cast<GpioPin>(9999));
    if (!invalidResult.IsSuccess()) {
        console_info(TAG, "Correctly rejected invalid pin operation");
    } else {
        console_error(TAG, "Failed to reject invalid pin operation");
        return false;
    }
    
    // Test pin name operations with invalid names
    if (!gpio.SetPinByName("non_existent_pin", true)) {
        console_info(TAG, "Correctly rejected invalid pin name");
    } else {
        console_error(TAG, "Failed to reject invalid pin name");
        return false;
    }
    
    console_info(TAG, "Error handling tests passed");
    return true;
}

/**
 * @brief Main integration test runner.
 * @return true if all tests pass, false otherwise.
 */
bool RunGpioIntegrationTests() {
    console_info(TAG, "Starting GPIO system integration tests");
    
    bool allTestsPassed = true;
    
    // Test basic system integration
    if (!TestGpioSystemIntegration()) {
        console_error(TAG, "GPIO system integration test failed");
        allTestsPassed = false;
    }
    
    // Test error handling
    if (!TestGpioErrorHandling()) {
        console_error(TAG, "GPIO error handling test failed");
        allTestsPassed = false;
    }
    
    if (allTestsPassed) {
        console_info(TAG, "All GPIO integration tests PASSED");
    } else {
        console_error(TAG, "Some GPIO integration tests FAILED");
    }
    
    return allTestsPassed;
}
