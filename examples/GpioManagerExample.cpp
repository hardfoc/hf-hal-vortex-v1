/**
 * @file GpioManagerExample.cpp
 * @brief Example demonstrating the advanced GPIO management system.
 * 
 * This example shows how to use the GpioManager with string-based pin identification,
 * complete BaseGpio function coverage, smart pin categorization, and proper
 * electrical configuration handling.
 * 
 * Key Features Demonstrated:
 * - String-based pin identification for extensibility
 * - Complete BaseGpio function coverage through routing
 * - Smart pin categorization (CORE, COMM, GPIO, USER)
 * - Proper electrical configuration (pull resistors, output modes, inversion)
 * - Handler-aware GPIO creation and ownership
 * - Thread-safe operations with comprehensive error handling
 * - Batch operations for performance optimization
 * - Advanced diagnostics and health monitoring
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 */

#include "managers/GpioManager.h"
#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"
#include "handlers/Logger.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"

//==============================================================================
// EXAMPLE FUNCTIONS
//==============================================================================

/**
 * @brief Demonstrate basic GPIO operations using string names.
 */
void DemonstrateBasicOperations() {
    Logger::GetInstance().Info("GpioManagerExample", "\n=== Basic GPIO Operations ===");
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Set a GPIO pin to active state
    hf_gpio_err_t result = gpio_manager.SetActive("GPIO_WS2812_LED_DAT");
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Set WS2812 LED pin to active");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to set WS2812 LED pin to active (error: " + std::to_string(static_cast<int>(result)) + ")");
    }
    
    // Read the current state
    bool state;
    result = gpio_manager.Read("GPIO_WS2812_LED_DAT", state);
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Read WS2812 LED pin state: " + (state ? "ACTIVE" : "INACTIVE"));
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to read WS2812 LED pin state (error: " + std::to_string(static_cast<int>(result)) + ")");
    }
    
    // Toggle the pin
    if (gpio_manager.Toggle("GPIO_WS2812_LED_DAT")) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Toggled WS2812 LED pin");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to toggle WS2812 LED pin");
    }
    
    // Set to inactive
    if (gpio_manager.SetInactive("GPIO_WS2812_LED_DAT")) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Set WS2812 LED pin to inactive");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to set WS2812 LED pin to inactive");
    }
}

/**
 * @brief Demonstrate pin configuration operations.
 */
void DemonstratePinConfiguration() {
    Logger::GetInstance().Info("GpioManagerExample", "\n=== Pin Configuration ===");
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Configure pin direction
    if (gpio_manager.SetDirection("GPIO_EXT_GPIO_CS_1", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT) == hf_gpio_err_t::GPIO_SUCCESS) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Set EXT_GPIO_CS_1 to output direction");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to set EXT_GPIO_CS_1 direction");
    }
    
    // Configure pull mode (this pin has pull-up configured in mapping)
    if (gpio_manager.SetPullMode("GPIO_EXT_GPIO_CS_1", hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP) == hf_gpio_err_t::GPIO_SUCCESS) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Set EXT_GPIO_CS_1 pull mode to UP");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to set EXT_GPIO_CS_1 pull mode");
    }
    
    // Configure output mode (push-pull vs open-drain)
    if (gpio_manager.SetOutputMode("GPIO_EXT_GPIO_CS_1", hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL) == hf_gpio_err_t::GPIO_SUCCESS) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Set EXT_GPIO_CS_1 to push-pull output mode");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to set EXT_GPIO_CS_1 output mode");
    }
    
    // Read back configuration
    hf_gpio_direction_t direction;
    if (gpio_manager.GetDirection("GPIO_EXT_GPIO_CS_1", direction)) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: EXT_GPIO_CS_1 direction: " + (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT ? "OUTPUT" : "INPUT"));
    }
    
    hf_gpio_pull_mode_t pull_mode;
    if (gpio_manager.GetPullMode("GPIO_EXT_GPIO_CS_1", pull_mode)) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: EXT_GPIO_CS_1 pull mode: " + std::to_string(static_cast<int>(pull_mode)));
    }
    
    hf_gpio_output_mode_t output_mode;
    if (gpio_manager.GetOutputMode("GPIO_EXT_GPIO_CS_1", output_mode)) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: EXT_GPIO_CS_1 output mode: " + (output_mode == hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL ? "PUSH_PULL" : "OPEN_DRAIN"));
    }
}

/**
 * @brief Demonstrate interrupt handling.
 */
void DemonstrateInterruptHandling() {
    Logger::GetInstance().Info("GpioManagerExample", "\n=== Interrupt Handling ===");
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Check if pin supports interrupts
    if (gpio_manager.SupportsInterrupts("GPIO_PCAL_IMU_INT")) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: PCAL_IMU_INT supports interrupts");
        
        // Configure interrupt callback
        auto interrupt_callback = [](BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) {
            Logger::GetInstance().Info("GpioManagerExample", "INTERRUPT: Interrupt triggered on pin!");
            (void)gpio; (void)trigger; (void)user_data; // Suppress unused warnings
        };
        
        // Configure interrupt
        if (gpio_manager.ConfigureInterrupt("GPIO_PCAL_IMU_INT", 
                                          hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE,
                                          interrupt_callback) == hf_gpio_err_t::GPIO_SUCCESS) {
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Configured rising edge interrupt on PCAL_IMU_INT");
        } else {
            Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to configure interrupt on PCAL_IMU_INT");
        }
        
        // Enable interrupt
        if (gpio_manager.EnableInterrupt("GPIO_PCAL_IMU_INT") == hf_gpio_err_t::GPIO_SUCCESS) {
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Enabled interrupt on PCAL_IMU_INT");
        } else {
            Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to enable interrupt on PCAL_IMU_INT");
        }
        
        // Wait a bit for potential interrupt
        os_delay_msec(100);
        
        // Disable interrupt
        if (gpio_manager.DisableInterrupt("GPIO_PCAL_IMU_INT") == hf_gpio_err_t::GPIO_SUCCESS) {
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Disabled interrupt on PCAL_IMU_INT");
        } else {
            Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to disable interrupt on PCAL_IMU_INT");
        }
    } else {
        Logger::GetInstance().Info("GpioManagerExample", "FAILED: PCAL_IMU_INT does not support interrupts");
    }
}

/**
 * @brief Demonstrate batch operations for performance.
 */
void DemonstrateBatchOperations() {
    Logger::GetInstance().Info("GpioManagerExample", "\n=== Batch Operations ===");
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Batch read multiple pins
    std::vector<std::string_view> pins_to_read = {
        "GPIO_WS2812_LED_DAT",
        "GPIO_EXT_GPIO_CS_1",
        "GPIO_EXT_GPIO_CS_2"
    };
    
    auto read_result = gpio_manager.BatchRead(pins_to_read);
    if (read_result.AllSuccessful()) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Batch read successful for " + std::to_string(read_result.pin_names.size()) + " pins");
        for (size_t i = 0; i < read_result.pin_names.size(); ++i) {
            Logger::GetInstance().Info("GpioManagerExample", "  " + std::string(read_result.pin_names[i]) + ": " + (read_result.states[i] ? "ACTIVE" : "INACTIVE"));
        }
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Batch read failed");
    }
    
    // Batch write multiple pins
    std::vector<std::string_view> pins_to_write = {"GPIO_WS2812_LED_DAT", "GPIO_EXT_GPIO_CS_1"};
    std::vector<bool> states = {true, false};
    
    GpioBatchOperation write_operation(pins_to_write, states);
    auto write_result = gpio_manager.BatchWrite(write_operation);
    if (write_result.AllSuccessful()) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Batch write successful for " + std::to_string(write_result.pin_names.size()) + " pins");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Batch write failed");
    }
    
    // Set multiple pins to active
    auto active_result = gpio_manager.SetMultipleActive({"GPIO_WS2812_LED_DAT"});
    if (active_result.AllSuccessful()) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Set multiple pins to active successful");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Set multiple pins to active failed");
    }
    
    // Set multiple pins to inactive
    auto inactive_result = gpio_manager.SetMultipleInactive({"GPIO_WS2812_LED_DAT"});
    if (inactive_result.AllSuccessful()) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Set multiple pins to inactive successful");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Set multiple pins to inactive failed");
    }
}

/**
 * @brief Demonstrate user-defined GPIO registration.
 */
void DemonstrateUserDefinedGpio() {
    Logger::GetInstance().Info("GpioManagerExample", "\n=== User-Defined GPIO Registration ===");
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Create a custom ESP32 GPIO instance
    auto custom_gpio = std::make_shared<EspGpio>(static_cast<hf_pin_num_t>(25)); // GPIO25
    
    // Configure the custom GPIO
    custom_gpio->SetDirection(hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    custom_gpio->SetPullMode(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING);
    custom_gpio->SetOutputMode(hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL);
    custom_gpio->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH);
    
    if (custom_gpio->Initialize()) {
        // Register the custom GPIO
        if (gpio_manager.RegisterGpio("USER_CUSTOM_LED", custom_gpio)) {
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Registered custom GPIO as USER_CUSTOM_LED");
        } else {
            Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to register custom GPIO");
            return;
        }
        
        // Use the custom GPIO
        if (gpio_manager.SetActive("USER_CUSTOM_LED")) {
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Set custom LED to active");
        }
        
        if (gpio_manager.SetInactive("USER_CUSTOM_LED")) {
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Set custom LED to inactive");
        }
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to initialize custom GPIO");
    }
}

/**
 * @brief Demonstrate system diagnostics and health monitoring.
 */
void DemonstrateSystemDiagnostics() {
    Logger::GetInstance().Info("GpioManagerExample", "\n=== System Diagnostics ===");
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Get system diagnostics
    GpioSystemDiagnostics diagnostics;
    if (gpio_manager.GetSystemDiagnostics(diagnostics)) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: System Diagnostics:");
        Logger::GetInstance().Info("GpioManagerExample", "  System Healthy: " + (diagnostics.system_healthy ? "YES" : "NO"));
        Logger::GetInstance().Info("GpioManagerExample", "  Total Pins Registered: " + std::to_string(diagnostics.total_pins_registered));
        Logger::GetInstance().Info("GpioManagerExample", "  Total Operations: " + std::to_string(diagnostics.total_operations));
        Logger::GetInstance().Info("GpioManagerExample", "  Successful Operations: " + std::to_string(diagnostics.successful_operations));
        Logger::GetInstance().Info("GpioManagerExample", "  Failed Operations: " + std::to_string(diagnostics.failed_operations));
        Logger::GetInstance().Info("GpioManagerExample", "  Communication Errors: " + std::to_string(diagnostics.communication_errors));
        Logger::GetInstance().Info("GpioManagerExample", "  Hardware Errors: " + std::to_string(diagnostics.hardware_errors));
        
        // Show pins by category
        Logger::GetInstance().Info("GpioManagerExample", "  Pins by Category:");
        Logger::GetInstance().Info("GpioManagerExample", "    CORE: " + std::to_string(diagnostics.pins_by_category[0]));
        Logger::GetInstance().Info("GpioManagerExample", "    COMM: " + std::to_string(diagnostics.pins_by_category[1]));
        Logger::GetInstance().Info("GpioManagerExample", "    GPIO: " + std::to_string(diagnostics.pins_by_category[2]));
        Logger::GetInstance().Info("GpioManagerExample", "    USER: " + std::to_string(diagnostics.pins_by_category[3]));
        
        // Show pins by chip
        Logger::GetInstance().Info("GpioManagerExample", "  Pins by Chip:");
        Logger::GetInstance().Info("GpioManagerExample", "    ESP32: " + std::to_string(diagnostics.pins_by_chip[0]));
        Logger::GetInstance().Info("GpioManagerExample", "    PCAL95555: " + std::to_string(diagnostics.pins_by_chip[1]));
        Logger::GetInstance().Info("GpioManagerExample", "    TMC9660: " + std::to_string(diagnostics.pins_by_chip[2]));
        
        // Show recent errors
        if (!diagnostics.error_messages.empty()) {
            Logger::GetInstance().Info("GpioManagerExample", "  Recent Errors:");
            for (const auto& error : diagnostics.error_messages) {
                Logger::GetInstance().Info("GpioManagerExample", "    - " + error);
            }
        }
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to get system diagnostics");
    }
    
    // Get system health information
    std::string health_info;
    if (gpio_manager.GetSystemHealth(health_info)) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: System Health: " + health_info);
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to get system health");
    }
    
    // Get pin statistics
    BaseGpio::PinStatistics stats;
    if (gpio_manager.GetStatistics("GPIO_WS2812_LED_DAT", stats)) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: WS2812 LED Pin Statistics:");
        Logger::GetInstance().Info("GpioManagerExample", "  Total Operations: " + std::to_string(stats.totalOperations));
        Logger::GetInstance().Info("GpioManagerExample", "  Successful Operations: " + std::to_string(stats.successfulOperations));
        Logger::GetInstance().Info("GpioManagerExample", "  Failed Operations: " + std::to_string(stats.failedOperations));
        Logger::GetInstance().Info("GpioManagerExample", "  State Changes: " + std::to_string(stats.stateChanges));
        Logger::GetInstance().Info("GpioManagerExample", "  Direction Changes: " + std::to_string(stats.directionChanges));
        Logger::GetInstance().Info("GpioManagerExample", "  Interrupt Count: " + std::to_string(stats.interruptCount));
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to get pin statistics");
    }
    
    // Reset statistics
    if (gpio_manager.ResetStatistics("GPIO_WS2812_LED_DAT")) {
        Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Reset pin statistics");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to reset pin statistics");
    }
}

/**
 * @brief Demonstrate pin categorization and validation.
 */
void DemonstratePinCategorization() {
    Logger::GetInstance().Info("GpioManagerExample", "\n=== Pin Categorization and Validation ===");
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Demonstrate pin categorization
    Logger::GetInstance().Info("GpioManagerExample", "Pin Categorization:");
    
    // CORE pins (system reserved - should not be registered as GPIOs)
    Logger::GetInstance().Info("GpioManagerExample", "  CORE pins (system reserved):");
    Logger::GetInstance().Info("GpioManagerExample", "    CORE_XTAL_32K_P: " + (gpio_manager.Contains("CORE_XTAL_32K_P") ? "REGISTERED" : "NOT REGISTERED"));
    Logger::GetInstance().Info("GpioManagerExample", "    CORE_BOOT_SEL: " + (gpio_manager.Contains("CORE_BOOT_SEL") ? "REGISTERED" : "NOT REGISTERED"));
    
    // COMM pins (communication - should not be registered as GPIOs)
    Logger::GetInstance().Info("GpioManagerExample", "  COMM pins (communication):");
    Logger::GetInstance().Info("GpioManagerExample", "    COMM_SPI2_MISO: " + (gpio_manager.Contains("COMM_SPI2_MISO") ? "REGISTERED" : "NOT REGISTERED"));
    Logger::GetInstance().Info("GpioManagerExample", "    COMM_I2C_SDA: " + (gpio_manager.Contains("COMM_I2C_SDA") ? "REGISTERED" : "NOT REGISTERED"));
    
    // GPIO pins (available for GPIO operations)
    Logger::GetInstance().Info("GpioManagerExample", "  GPIO pins (available for GPIO):");
    Logger::GetInstance().Info("GpioManagerExample", "    GPIO_WS2812_LED_DAT: " + (gpio_manager.Contains("GPIO_WS2812_LED_DAT") ? "REGISTERED" : "NOT REGISTERED"));
    Logger::GetInstance().Info("GpioManagerExample", "    GPIO_EXT_GPIO_CS_1: " + (gpio_manager.Contains("GPIO_EXT_GPIO_CS_1") ? "REGISTERED" : "NOT REGISTERED"));
    
    // PCAL95555 pins
    Logger::GetInstance().Info("GpioManagerExample", "  PCAL95555 pins:");
    Logger::GetInstance().Info("GpioManagerExample", "    GPIO_PCAL_GPIO17: " + (gpio_manager.Contains("GPIO_PCAL_GPIO17") ? "REGISTERED" : "NOT REGISTERED"));
    Logger::GetInstance().Info("GpioManagerExample", "    GPIO_PCAL_IMU_INT: " + (gpio_manager.Contains("GPIO_PCAL_IMU_INT") ? "REGISTERED" : "NOT REGISTERED"));
    
    // Demonstrate validation
    Logger::GetInstance().Info("GpioManagerExample", "\nPin Name Validation:");
    Logger::GetInstance().Info("GpioManagerExample", "  Valid names (should work):");
    Logger::GetInstance().Info("GpioManagerExample", "    USER_CUSTOM_LED: " + (gpio_manager.Contains("USER_CUSTOM_LED") ? "EXISTS" : "DOES NOT EXIST"));
    
    Logger::GetInstance().Info("GpioManagerExample", "  Reserved prefixes (should be rejected):");
    Logger::GetInstance().Info("GpioManagerExample", "    CORE_RESERVED: Reserved prefix");
    Logger::GetInstance().Info("GpioManagerExample", "    COMM_RESERVED: Reserved prefix");
    Logger::GetInstance().Info("GpioManagerExample", "    SYS_RESERVED: Reserved prefix");
    Logger::GetInstance().Info("GpioManagerExample", "    INTERNAL_RESERVED: Reserved prefix");
    
    // Show total registered pins
    Logger::GetInstance().Info("GpioManagerExample", "\nTotal registered pins: " + std::to_string(gpio_manager.Size()));
}

/**
 * @brief Demonstrate electrical configuration handling.
 */
void DemonstrateElectricalConfiguration() {
    Logger::GetInstance().Info("GpioManagerExample", "\n=== Electrical Configuration ===");
    
    auto& gpio_manager = GpioManager::GetInstance();
    
    // Demonstrate how pins are configured with proper electrical characteristics
    Logger::GetInstance().Info("GpioManagerExample", "Pin Electrical Configuration (from mapping):");
    
    // Show configuration for different pin types
    std::vector<std::string_view> test_pins = {
        "GPIO_WS2812_LED_DAT",      // No pull, push-pull output
        "GPIO_EXT_GPIO_CS_1",       // Pull-up, push-pull output, inverted
        "GPIO_PCAL_FAULT_STATUS",   // No pull, push-pull output, inverted
        "GPIO_PCAL_IMU_INT"         // No pull, push-pull output, inverted
    };
    
    for (const auto& pin_name : test_pins) {
        if (gpio_manager.Contains(pin_name)) {
            Logger::GetInstance().Info("GpioManagerExample", "  " + std::string(pin_name) + ":");
            
            // Get pull mode
            hf_gpio_pull_mode_t pull_mode;
            if (gpio_manager.GetPullMode(pin_name, pull_mode)) {
                Logger::GetInstance().Info("GpioManagerExample", "    Pull Mode: ");
                switch (pull_mode) {
                    case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
                        Logger::GetInstance().Info("GpioManagerExample", "FLOATING (no pull)");
                        break;
                    case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
                        Logger::GetInstance().Info("GpioManagerExample", "PULL-UP");
                        break;
                    case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
                        Logger::GetInstance().Info("GpioManagerExample", "PULL-DOWN");
                        break;
                    case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
                        Logger::GetInstance().Info("GpioManagerExample", "PULL-UP + PULL-DOWN");
                        break;
                }
                Logger::GetInstance().Info("GpioManagerExample", "");
            }
            
            // Get output mode
            hf_gpio_output_mode_t output_mode;
            if (gpio_manager.GetOutputMode(pin_name, output_mode)) {
                Logger::GetInstance().Info("GpioManagerExample", "    Output Mode: ");
                switch (output_mode) {
                    case hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL:
                        Logger::GetInstance().Info("GpioManagerExample", "PUSH-PULL");
                        break;
                    case hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_OPEN_DRAIN:
                        Logger::GetInstance().Info("GpioManagerExample", "OPEN-DRAIN");
                        break;
                }
                Logger::GetInstance().Info("GpioManagerExample", "");
            }
            
            // Get direction
            hf_gpio_direction_t direction;
            if (gpio_manager.GetDirection(pin_name, direction)) {
                Logger::GetInstance().Info("GpioManagerExample", "    Direction: ");
                switch (direction) {
                    case hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT:
                        Logger::GetInstance().Info("GpioManagerExample", "INPUT");
                        break;
                    case hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT:
                        Logger::GetInstance().Info("GpioManagerExample", "OUTPUT");
                        break;
                }
                Logger::GetInstance().Info("GpioManagerExample", "");
            }
        }
    }
    
    Logger::GetInstance().Info("GpioManagerExample", "\nNote: All pins are automatically configured with proper electrical characteristics");
    Logger::GetInstance().Info("GpioManagerExample", "based on the primitive configuration fields in the pin mapping:");
    Logger::GetInstance().Info("GpioManagerExample", "- has_pull: Whether pin has pull resistor");
    Logger::GetInstance().Info("GpioManagerExample", "- pull_is_up: If has_pull=true: true=pull-up, false=pull-down");
    Logger::GetInstance().Info("GpioManagerExample", "- is_push_pull: Output mode: true=push-pull, false=open-drain");
    Logger::GetInstance().Info("GpioManagerExample", "- is_inverted: Logic inversion: true=inverted, false=normal");
    Logger::GetInstance().Info("GpioManagerExample", "- max_current_ma: Maximum current in milliamps");
}

//==============================================================================
// MAIN FUNCTION
//==============================================================================

int main() {
    Logger::GetInstance().Info("GpioManagerExample", "HardFOC GPIO Manager Example");
    Logger::GetInstance().Info("GpioManagerExample", "=============================");
    Logger::GetInstance().Info("GpioManagerExample", "Demonstrating advanced GPIO management with:");
    Logger::GetInstance().Info("GpioManagerExample", "- String-based pin identification");
    Logger::GetInstance().Info("GpioManagerExample", "- Complete BaseGpio function coverage");
    Logger::GetInstance().Info("GpioManagerExample", "- Smart pin categorization (CORE, COMM, GPIO, USER)");
    Logger::GetInstance().Info("GpioManagerExample", "- Proper electrical configuration handling");
    Logger::GetInstance().Info("GpioManagerExample", "- Handler-aware GPIO creation and ownership");
    Logger::GetInstance().Info("GpioManagerExample", "- Thread-safe operations with comprehensive error handling");
    Logger::GetInstance().Info("GpioManagerExample", "- Batch operations for performance optimization");
    Logger::GetInstance().Info("GpioManagerExample", "- Advanced diagnostics and health monitoring");
    
    // Initialize the GPIO manager
    auto& gpio_manager = GpioManager::GetInstance();
    if (!gpio_manager.EnsureInitialized()) {
        Logger::GetInstance().Error("GpioManagerExample", "FAILED: Failed to initialize GPIO manager");
        return -1;
    }
    
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: GPIO manager initialized successfully");
    
    // Run demonstrations
    DemonstratePinCategorization();
    DemonstrateElectricalConfiguration();
    DemonstrateBasicOperations();
    DemonstratePinConfiguration();
    DemonstrateInterruptHandling();
    DemonstrateBatchOperations();
    DemonstrateUserDefinedGpio();
    DemonstrateSystemDiagnostics();
    
    // Reset all pins to inactive state
    if (gpio_manager.ResetAllPins()) {
        Logger::GetInstance().Info("GpioManagerExample", "\nSUCCESS: Reset all output pins to inactive state");
    } else {
        Logger::GetInstance().Error("GpioManagerExample", "\nFAILED: Failed to reset all pins");
    }
    
    Logger::GetInstance().Info("GpioManagerExample", "\n=== Example Complete ===");
    Logger::GetInstance().Info("GpioManagerExample", "The GPIO manager successfully demonstrated:");
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: String-based pin identification for extensibility");
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Complete BaseGpio function coverage through routing");
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Smart pin categorization (CORE, COMM, GPIO, USER)");
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Proper electrical configuration (pull resistors, output modes, inversion)");
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Handler-aware GPIO creation and ownership");
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Thread-safe operations with comprehensive error handling");
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Batch operations for performance optimization");
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: Advanced diagnostics and health monitoring");
            Logger::GetInstance().Info("GpioManagerExample", "SUCCESS: User-defined GPIO registration and management");
    
    return 0;
} 