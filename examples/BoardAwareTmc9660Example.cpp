/**
 * @file BoardAwareTmc9660Example.cpp
 * @brief Example demonstrating board-aware TMC9660 device management with MotorController.
 * 
 * This example shows how to:
 * 1. Create the onboard TMC9660 device with control pins and SPI interface
 * 2. Initialize the MotorController (initializes all registered devices)
 * 3. Create external TMC9660 devices using SPI device IDs and control pins
 * 4. Access devices by their board-defined indices
 * 5. Use the visitDriver() pattern for direct TMC9660 driver access
 * 6. Safely delete external devices when needed
 * 7. Handle device enumeration and validation
 * 
 * Board Configuration:
 * - Index 0: Onboard TMC9660 (SPI2_CS_TMC9660) - Created explicitly with control pins
 * - Index 1: AS5047U Position Encoder (SPI2_CS_AS5047) - Not used for TMC9660
 * - Index 2: External Device 1 (EXT_GPIO_CS_1) - Dynamic TMC9660 creation/deletion
 * - Index 3: External Device 2 (EXT_GPIO_CS_2) - Dynamic TMC9660 creation/deletion
 * 
 * New Features (TMC9660 driver update):
 * - Control pin GPIO references (RST, DRV_EN, FAULTN, WAKE) required for each device
 * - visitDriver() template for type-safe access to the underlying TMC9660 driver
 * - Explicit device creation before Initialize()
 * 
 * @author HardFOC Team
 * @date 2025
 */

#include "MotorController.h"
#include "CommChannelsManager.h"
#include "handlers/logger/Logger.h"
#include <memory>

// =============================================================================
// NOTE: In a real application, these GPIO objects would come from your board
// configuration / GpioManager. The examples below use placeholder references
// to illustrate the API. You must provide actual BaseGpio instances that are
// configured for the correct pins and direction before passing them here.
// =============================================================================

/**
 * @brief Initialize the motor controller system with onboard device.
 * 
 * Demonstrates the new two-step initialization:
 *   1. CreateOnboardDevice() with SPI interface + control pin GPIOs
 *   2. Initialize() to run bootloader init on all registered devices
 *
 * @param onboard_pins Pre-configured GPIO control pins for the onboard TMC9660
 */
void initializeMotorControllerSystem(Tmc9660ControlPins& onboard_pins) {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "=== Initializing Motor Controller System ===");
    
    auto& motorController = MotorController::GetInstance();
    auto& commManager = CommChannelsManager::GetInstance();
    
    // Ensure CommChannelsManager is ready
    if (!commManager.EnsureInitialized()) {
        Logger::GetInstance().Error("BoardAwareTmc9660Example", "FAILED: CommChannelsManager initialization failed!");
        return;
    }
    
    // Get the onboard TMC9660 SPI interface
    BaseSpi* spiInterface = commManager.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (!spiInterface) {
        Logger::GetInstance().Error("BoardAwareTmc9660Example", "FAILED: Onboard TMC9660 SPI interface not available!");
        return;
    }
    
    // Step 1: Create the onboard TMC9660 device with control pins
    bool created = motorController.CreateOnboardDevice(
        *spiInterface,
        0x01,            // Default TMC9660 address
        onboard_pins     // Host-side GPIO control pins (RST, DRV_EN, FAULTN, WAKE)
        // Uses kDefaultBootConfig by default
    );
    
    if (!created) {
        Logger::GetInstance().Error("BoardAwareTmc9660Example", "FAILED: Could not create onboard TMC9660 device!");
        return;
    }
    
    Logger::GetInstance().Info("BoardAwareTmc9660Example", 
        "SUCCESS: Onboard TMC9660 device registered at index %d",
        static_cast<int>(MotorController::ONBOARD_TMC9660_INDEX));
    
    // Step 2: Initialize all registered devices (runs bootloader init sequence)
    bool initSuccess = motorController.Initialize();
    if (initSuccess) {
        Logger::GetInstance().Info("BoardAwareTmc9660Example", "SUCCESS: Motor controller system initialized!");
    } else {
        Logger::GetInstance().Error("BoardAwareTmc9660Example", "FAILED: Motor controller initialization failed!");
        return;
    }
    
    // Show active devices
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "Active device count: %d",
        static_cast<int>(motorController.GetDeviceCount()));
}

/**
 * @brief Create external TMC9660 devices on available CS lines.
 * 
 * Each external device requires its own set of control pin GPIOs.
 *
 * @param ext1_pins Control pins for external device 1 (or nullptr to skip)
 * @param ext2_pins Control pins for external device 2 (or nullptr to skip)
 */
void createExternalDevices(Tmc9660ControlPins* ext1_pins, Tmc9660ControlPins* ext2_pins) {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "=== Creating External TMC9660 Devices ===");
    
    auto& motorController = MotorController::GetInstance();
    
    // Check available external slots
    bool slot2Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_1_INDEX);
    bool slot3Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_2_INDEX);
    
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "External slot 2 (EXT_GPIO_CS_1): %s",
        slot2Available ? "Available" : "Occupied");
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "External slot 3 (EXT_GPIO_CS_2): %s",
        slot3Available ? "Available" : "Occupied");
    
    // Create external device 1 using SPI device ID + control pins
    if (slot2Available && ext1_pins) {
        bool created = motorController.CreateExternalDevice(
            MotorController::EXTERNAL_DEVICE_1_INDEX, 
            SpiDeviceId::EXTERNAL_DEVICE_1, 
            0x02,        // Device address 2
            *ext1_pins   // Control pin GPIOs for this device
        );
        if (created) {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", 
                "SUCCESS: External TMC9660 device 1 created at index %d",
                static_cast<int>(MotorController::EXTERNAL_DEVICE_1_INDEX));
        } else {
            Logger::GetInstance().Error("BoardAwareTmc9660Example", 
                "FAILED: Failed to create external TMC9660 device 1");
        }
    }
    
    // Create external device 2 using SPI device ID + control pins
    if (slot3Available && ext2_pins) {
        bool created = motorController.CreateExternalDevice(
            MotorController::EXTERNAL_DEVICE_2_INDEX,
            SpiDeviceId::EXTERNAL_DEVICE_2,
            0x03,        // Device address 3
            *ext2_pins   // Control pin GPIOs for this device
        );
        if (created) {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", 
                "SUCCESS: External TMC9660 device 2 created at index %d",
                static_cast<int>(MotorController::EXTERNAL_DEVICE_2_INDEX));
        } else {
            Logger::GetInstance().Error("BoardAwareTmc9660Example", 
                "FAILED: Failed to create external TMC9660 device 2");
        }
    }
    
    // Show updated device count
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "Total active devices: %d",
        static_cast<int>(motorController.GetDeviceCount()));
}

/**
 * @brief Control specific devices by their board indices.
 * 
 * Demonstrates:
 * - Accessing handler GPIO and ADC wrappers
 * - Using visitDriver() for direct TMC9660 subsystem access
 */
void controlSpecificDevices() {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "=== Controlling Specific Devices ===");
    
    auto& motorController = MotorController::GetInstance();
    
    // Control onboard TMC9660 (always available)
    if (motorController.IsDeviceValid(MotorController::ONBOARD_TMC9660_INDEX)) {
        auto* onboardHandler = motorController.handler(MotorController::ONBOARD_TMC9660_INDEX);
        if (onboardHandler) {
            // Access GPIO through the handler
            auto& gpio17_onboard = onboardHandler->gpio(17);
            gpio17_onboard.SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
            Logger::GetInstance().Info("BoardAwareTmc9660Example", 
                "SUCCESS: Set GPIO17 on onboard TMC9660 (index 0) to HIGH");
            
            // Access ADC through the handler
            auto& adc_onboard = onboardHandler->adc();
            float voltage = 0.0f;
            hf_adc_err_t result = adc_onboard.ReadChannelV(0, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("BoardAwareTmc9660Example", 
                    "SUCCESS: ADC Channel 0 on onboard TMC9660: %.3fV", voltage);
            }
            
            // Set motor velocity through visitDriver() for direct subsystem access
            motorController.visitDriver([](auto& driver) {
                driver.velocityControl.setTargetVelocity(1000);
            }, MotorController::ONBOARD_TMC9660_INDEX);
            Logger::GetInstance().Info("BoardAwareTmc9660Example", 
                "Set target velocity to 1000 on onboard TMC9660");
        }
    }
    
    // Control external device 1 using visitDriver() for advanced operations
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_1_INDEX)) {
        auto* ext1Handler = motorController.handler(MotorController::EXTERNAL_DEVICE_1_INDEX);
        if (ext1Handler) {
            // Access GPIO through the handler
            auto& gpio18_ext1 = ext1Handler->gpio(18);
            gpio18_ext1.SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_LOW);
            Logger::GetInstance().Info("BoardAwareTmc9660Example", 
                "SUCCESS: Set GPIO18 on external device 1 (index 2) to LOW");
            
            // Use visitDriver() for direct access to the typed TMC9660 driver.
            // The lambda receives a reference to the actual TMC9660<CommType> instance,
            // giving access to all subsystems (motorConfig, velocityControl, etc.)
            motorController.visitDriver([](auto& driver) {
                // Direct subsystem access - these methods are on the typed driver
                driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC, 7);
                driver.velocityControl.setTargetVelocity(500);
                Logger::GetInstance().Info("BoardAwareTmc9660Example", 
                    "SUCCESS: Direct TMC9660 driver access on external device 1");
            }, MotorController::EXTERNAL_DEVICE_1_INDEX);
        }
    }
    
    // Control external device 2 (if available)
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_2_INDEX)) {
        auto* ext2Handler = motorController.handler(MotorController::EXTERNAL_DEVICE_2_INDEX);
        if (ext2Handler) {
            // Access ADC through the handler
            auto& adc_ext2 = ext2Handler->adc();
            float voltage = 0.0f;
            hf_adc_err_t result = adc_ext2.ReadChannelV(1, voltage);
            if (result == hf_adc_err_t::ADC_SUCCESS) {
                Logger::GetInstance().Info("BoardAwareTmc9660Example", 
                    "SUCCESS: ADC Channel 1 on external device 2: %.3fV", voltage);
            }
        }
    }
}

/**
 * @brief Demonstrate device enumeration and validation
 */
void enumerateAndValidateDevices() {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "=== Device Enumeration and Validation ===");
    
    auto& motorController = MotorController::GetInstance();
    
    // Check each possible device slot
    for (uint8_t i = 0; i < MotorController::MAX_TMC9660_DEVICES; ++i) {
        const char* slot_name;
        switch (i) {
            case 0:  slot_name = "Onboard TMC9660";      break;
            case 1:  slot_name = "AS5047U Encoder";       break;
            case 2:  slot_name = "External Device 1";     break;
            case 3:  slot_name = "External Device 2";     break;
            default: slot_name = "Unknown";               break;
        }
        
        if (motorController.IsDeviceValid(i)) {
            auto* h = motorController.handler(i);
            Logger::GetInstance().Info("BoardAwareTmc9660Example", 
                "  Slot %d (%s): ACTIVE, Handler %s", 
                i, slot_name, h ? "ready" : "not available");
        } else {
            Logger::GetInstance().Info("BoardAwareTmc9660Example", 
                "  Slot %d (%s): INACTIVE", i, slot_name);
        }
    }
    
    // Check initialization status
    auto initStatus = motorController.GetInitializationStatus();
    for (size_t i = 0; i < initStatus.size(); ++i) {
        Logger::GetInstance().Info("BoardAwareTmc9660Example", 
            "  Init status [%d]: %s", static_cast<int>(i), initStatus[i] ? "OK" : "FAILED");
    }
}

/**
 * @brief Demonstrate external device deletion
 */
void demonstrateDeviceDeletion() {
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "=== External Device Deletion ===");
    
    auto& motorController = MotorController::GetInstance();
    
    // Try to delete onboard device (should fail - only external devices can be deleted)
    bool onboardDeleted = motorController.DeleteExternalDevice(MotorController::ONBOARD_TMC9660_INDEX);
    Logger::GetInstance().Info("BoardAwareTmc9660Example", 
        "Attempt to delete onboard device (index 0): %s",
        onboardDeleted ? "SUCCESS" : "FAILED (as expected)");
    
    // Delete external device 1 if it exists
    if (motorController.IsDeviceValid(MotorController::EXTERNAL_DEVICE_1_INDEX)) {
        bool ext1Deleted = motorController.DeleteExternalDevice(MotorController::EXTERNAL_DEVICE_1_INDEX);
        Logger::GetInstance().Info("BoardAwareTmc9660Example", 
            "Delete external device 1 (index 2): %s",
            ext1Deleted ? "SUCCESS" : "FAILED");
    } else {
        Logger::GetInstance().Info("BoardAwareTmc9660Example", 
            "External device 1 (index 2): Not present, cannot delete");
    }
    
    // Show updated device count
    Logger::GetInstance().Info("BoardAwareTmc9660Example", "Remaining active devices: %d",
        static_cast<int>(motorController.GetDeviceCount()));
    
    // Verify external device 1 slot is now available
    bool slot2Available = motorController.IsExternalSlotAvailable(MotorController::EXTERNAL_DEVICE_1_INDEX);
    Logger::GetInstance().Info("BoardAwareTmc9660Example", 
        "External slot 2 (EXT_GPIO_CS_1): %s",
        slot2Available ? "Available for new device" : "Still occupied");
}

/**
 * @brief Main example function.
 * 
 * @note In a real application, the BaseGpio objects for control pins would come
 *       from your board configuration (e.g., GpioManager or platform-specific init).
 *       This example shows the API flow but does not include actual GPIO creation.
 */
// int main() {
//     Logger::GetInstance().Info("BoardAwareTmc9660Example", 
//         "=== Board-Aware TMC9660 Device Management Example ===");
//     
//     // --- Board-specific setup (pseudo-code) ---
//     // These GPIO objects must be created and configured for your board before use.
//     // For example, on the TMC9660-3PH-EVAL board with ESP32:
//     //   RST    -> GPIO 4  (output, active high)
//     //   DRV_EN -> GPIO 16 (output, active high)
//     //   FAULTN -> GPIO 35 (input, active low)
//     //   WAKE   -> GPIO 17 (output, active low)
//     //
//     // BaseGpio rst_gpio  = ...;  // Configure for your board
//     // BaseGpio drv_en_gpio = ...;
//     // BaseGpio faultn_gpio = ...;
//     // BaseGpio wake_gpio = ...;
//     //
//     // Tmc9660ControlPins onboard_pins {
//     //     .rst    = rst_gpio,
//     //     .drv_en = drv_en_gpio,
//     //     .faultn = faultn_gpio,
//     //     .wake   = wake_gpio
//     // };
//     //
//     // initializeMotorControllerSystem(onboard_pins);
//     // createExternalDevices(nullptr, nullptr);  // Pass pin structs for external devices
//     // enumerateAndValidateDevices();
//     // controlSpecificDevices();
//     // demonstrateDeviceDeletion();
//     
//     Logger::GetInstance().Info("BoardAwareTmc9660Example", "=== Example completed ===");
//     return 0;
// }
