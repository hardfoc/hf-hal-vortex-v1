/**
 * @file BoardAwareTmc9660Example.cpp
 * @brief Example demonstrating board-aware TMC9660 device management with MotorController.
 * 
 * This example shows how to:
 * 1. Initialize the MotorController (automatically creates onboard TMC9660)
 * 2. Create external TMC9660 devices using SPI device IDs
 * 3. Access devices by their board-defined indices
 * 4. Safely delete external devices when needed
 * 5. Handle device enumeration and validation
 * 
 * Board Configuration:
 * - Index 0: Onboard TMC9660 (SPI2_CS_TMC9660) - Auto-created, cannot be deleted
 * - Index 1: AS5047U Position Encoder (SPI2_CS_AS5047) - Not used for TMC9660
 * - Index 2: External Device 1 (EXT_GPIO_CS_1) - Dynamic TMC9660 creation/deletion
 * - Index 3: External Device 2 (EXT_GPIO_CS_2) - Dynamic TMC9660 creation/deletion
 * 
 * New Features:
 * - Automatic onboard device creation via CommChannelsManager integration
 * - SPI device ID specification instead of manual interface management
 * - Simplified initialization with one-line setup
 * 
 * @author HardFOC Team
 * @date 2025
 */

#include "MotorController.h"
#include "CommChannelsManager.h"
#include "handlers/Logger.h"
#include <memory>

/**
 * @brief Initialize the motor controller system with onboard device
 */
void initializeMotorControllerSystem() {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "=== Initializing Motor Controller System ===");
    
    auto& motorController = MotorController::GetInstance();
    
    // Initialize the system - this automatically creates the onboard TMC9660
    bool initSuccess = motorController.Initialize();
    if (initSuccess) {
        Logger::GetInstance().Info("BoardAwareTmc9660Example", "SUCCESS: Motor controller system initialized successfully!");
                Logger::GetInstance().Info("BoardAwareTmc9660Example", "SUCCESS: Onboard TMC9660 device created at index " 
                   + std::to_string(static_cast<int>(MotorController::ONBOARD_TMC9660_INDEX)));
    } else {
        Logger::GetInstance().Info("BoardAwareTmc9660Example", "FAILED: Motor controller system initialization failed!");
        return;
    }
    
    // Show active devices
    auto activeDevices = motorController.GetActiveDeviceIndices();
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "Active devices: " << activeDevices);
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "(Total: " << static_cast<int>(motorController.GetDeviceCount()) << ")");
}

/**
 * @brief Create external TMC9660 devices on available CS lines
 */
void createExternalDevices() {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "\n=== Creating External TMC9660 Devices ===");
    
    auto& motorController = MotorController::GetInstance();
    
    // Check available external slots
    bool slot2Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_1_INDEX);
    bool slot3Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_2_INDEX);
    
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "External slot 2 (EXT_GPIO_CS_1): " << (slot2Available ? "Available" : "Occupied"));
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "External slot 3 (EXT_GPIO_CS_2): " << (slot3Available ? "Available" : "Occupied"));
    
    // Create external device 1 using SPI device ID
    if (slot2Available) {
        bool created = motorController.CreateExternalDevice(
            MotorController::EXTERNAL_DEVICE_1_INDEX, 
            SpiDeviceId::EXTERNAL_DEVICE_1, 
            0x02  // Device address 2
        );
        if (created) {
                        Logger::GetInstance().Info("BoardAwareTmc9660Example", "SUCCESS: External TMC9660 device 1 created at index " 
                       + std::to_string(static_cast<int>(MotorController::EXTERNAL_DEVICE_1_INDEX)));
        } else {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "FAILED: Failed to create external TMC9660 device 1");
        }
    }
    
    // Create external device 2 using SPI device ID
    if (slot3Available) {
        bool created = motorController.CreateExternalDevice(
            MotorController::EXTERNAL_DEVICE_2_INDEX,
            SpiDeviceId::EXTERNAL_DEVICE_2,
            0x03  // Device address 3
        );
        if (created) {
                        Logger::GetInstance().Info("BoardAwareTmc9660Example", "SUCCESS: External TMC9660 device 2 created at index " 
                       + std::to_string(static_cast<int>(MotorController::EXTERNAL_DEVICE_2_INDEX)));
        } else {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "FAILED: Failed to create external TMC9660 device 2");
        }
        } else {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "FAILED: SPI interface for external device 2 not available");
        }
    }
    
    // Alternative: Create external device using UART interface
    if (commManager.GetUartCount() > 0) {
        Logger::GetInstance().Info("BoardAwareTmc9660Example", "Note: External devices can also be created with UART interface using CreateExternalDevice()");
    }
    
    // Show updated device count
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "Total active devices: " << static_cast<int>(motorController.GetDeviceCount()));
}

/**
 * @brief Control specific devices by their board indices
 */
void controlSpecificDevices() {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "\n=== Controlling Specific Devices ===");
    
    auto& motorController = MotorController::GetInstance();
    
    // Control onboard TMC9660 (always available) - no exceptions
    if (motorController.IsDeviceValid(MotorController::ONBOARD_TMC9660_INDEX)) {
        auto* onboardHandler = motorController.handler(MotorController::ONBOARD_TMC9660_INDEX);
        if (onboardHandler) {
            // Access GPIO and ADC through the handler
            auto& gpio17_onboard = onboardHandler->gpio(17);
            gpio17_onboard.SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "SUCCESS: Set GPIO17 on onboard TMC9660 (index 0) to HIGH");
            
            auto& adc_onboard = onboardHandler->adc();
            float voltage = 0.0f;
            hf_adc_err_t result = adc_onboard.ReadChannelV(0, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("BoardAwareTmc9660Example", "SUCCESS: ADC Channel 0 on onboard TMC9660: " + std::to_string(voltage) + "V");
            }
        } else {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "FAILED: Failed to get onboard TMC9660 handler");
        }
    }
    
    // Control external device 1 (if available) - no exceptions
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_1_INDEX)) {
        auto* ext1Handler = motorController.handler(MotorController::EXTERNAL_DEVICE_1_INDEX);
        if (ext1Handler) {
            // Access GPIO and driver through the handler
            auto& gpio18_ext1 = ext1Handler->gpio(18);
            gpio18_ext1.SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_LOW);
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "SUCCESS: Set GPIO18 on external device 1 (index 2) to LOW");
            
            // Access TMC9660 driver directly for advanced control - no exceptions
            auto driver_ext1 = motorController.driver(MotorController::EXTERNAL_DEVICE_1_INDEX);
            if (driver_ext1) {
                Logger::GetInstance().Info("BoardAwareTmc9660Example", "SUCCESS: Direct access to external TMC9660 driver 1 successful");
                // Use driver_ext1 for advanced TMC9660 operations...
            } else {
                Logger::GetInstance().Info("BoardAwareTmc9660Example", "FAILED: External TMC9660 driver 1 not available");
            }
        } else {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "FAILED: Failed to get external device 1 handler");
        }
    }
    
    // Control external device 2 (if available) - no exceptions
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_2_INDEX)) {
        auto* ext2Handler = motorController.handler(MotorController::EXTERNAL_DEVICE_2_INDEX);
        if (ext2Handler) {
            // Access ADC through the handler
            auto& adc_ext2 = ext2Handler->adc();
            float voltage = 0.0f;
            hf_adc_err_t result = adc_ext2.ReadChannelV(1, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("BoardAwareTmc9660Example", "SUCCESS: ADC Channel 1 on external device 2: " + std::to_string(voltage) + "V");
            }
        } else {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "FAILED: Failed to get external device 2 handler");
        }
    }
}

/**
 * @brief Demonstrate device enumeration and validation
 */
void enumerateAndValidateDevices() {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "\n=== Device Enumeration and Validation ===");
    
    auto& motorController = MotorController::GetInstance();
    
    // Get all active device indices
    auto activeDevices = motorController.GetActiveDeviceIndices();
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "Active device indices: " << activeDevices);
    
    // Check each possible device slot
    for (uint8_t i = 0; i < MotorController::MAX_TMC9660_DEVICES; ++i) {
        Logger::GetInstance().Info("BoardAwareTmc9660Example", "Device slot " << static_cast<int>(i) << ": ");
        
        if (i == 0) {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "Onboard TMC9660 - ");
        } else if (i == 1) {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "AS5047U Encoder - ");
        } else if (i == 2) {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "External Device 1 - ");
        } else if (i == 3) {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "External Device 2 - ");
        }
        
        if (motorController.IsDeviceValid(i)) {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "ACTIVE");
            auto* handler = motorController.handler(i);
            if (handler) {
                Logger::GetInstance().Info("BoardAwareTmc9660Example", " (Handler ready)");
            } else {
                Logger::GetInstance().Info("BoardAwareTmc9660Example", " (Handler not available)");
            }
            }
        } else {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", "INACTIVE");
        }
        Logger::GetInstance().Info("BoardAwareTmc9660Example", "");
    }
    
    // Check initialization status
    auto initStatus = motorController.GetInitializationStatus();
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "Initialization status: ");
    for (size_t i = 0; i < initStatus.size(); ++i) {
        Logger::GetInstance().Info("BoardAwareTmc9660Example", (initStatus[i] ? "OK" : "FAILED") << " ");
    }
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "");
}

/**
 * @brief Demonstrate external device deletion
 */
void demonstrateDeviceDeletion() {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "\n=== External Device Deletion ===");
    
    auto& motorController = MotorController::GetInstance();
    
    // Try to delete onboard device (should fail)
    bool onboardDeleted = motorController.DeleteExternalDevice(MotorController::ONBOARD_TMC9660_INDEX);
        Logger::GetInstance().Info("BoardAwareTmc9660Example", "Attempt to delete onboard device (index 0): " 
               + (onboardDeleted ? "SUCCESS" : "FAILED (as expected)"));
    
    // Delete external device 1 if it exists
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_1_INDEX)) {
        bool ext1Deleted = motorController.DeleteExternalDevice(MotorController::EXTERNAL_DEVICE_1_INDEX);
                Logger::GetInstance().Info("BoardAwareTmc9660Example", "Delete external device 1 (index 2): " 
                   + (ext1Deleted ? "SUCCESS" : "FAILED"));
    } else {
        Logger::GetInstance().Info("BoardAwareTmc9660Example", "External device 1 (index 2): Not present, cannot delete");
    }
    
    // Show updated device count
    auto activeDevices = motorController.GetActiveDeviceIndices();
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "Remaining active devices: " << activeDevices);
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "(Total: " << static_cast<int>(motorController.GetDeviceCount()) << ")");
    
    // Verify external device 1 slot is now available
    bool slot2Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_1_INDEX);
        Logger::GetInstance().Info("BoardAwareTmc9660Example", "External slot 2 (EXT_GPIO_CS_1) availability: " 
               + (slot2Available ? "Available for new device" : "Still occupied"));
}

/**
 * @brief Main example function
 */
int main() {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "=== Board-Aware TMC9660 Device Management Example ===");
    
    // Initialize the motor controller system (creates onboard device)
    initializeMotorControllerSystem();
    
    // Create external devices on available CS lines
    createExternalDevices();
    
    // Enumerate and validate all devices
    enumerateAndValidateDevices();
    
    // Control specific devices by their board indices
    controlSpecificDevices();
    
    // Demonstrate external device deletion
    demonstrateDeviceDeletion();
    
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "\n=== Example completed ===");
    return 0;
}
