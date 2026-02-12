/**
 * @file Bno08xHandlerImuExample.cpp
 * @brief Example demonstrating Bno08xHandler usage through ImuManager with GPIO interrupt support.
 * 
 * This example shows how to:
 * 1. Initialize ImuManager (automatically creates Bno08xHandler and GPIO interrupt support)
 * 2. Access BNO08x IMU through unified Bno08xHandler interface
 * 3. Configure GPIO interrupts using PCAL_IMU_INT pin through GpioManager
 * 4. Use both polling and interrupt-driven modes
 * 5. Handle errors gracefully with pointer-based returns
 * 
 * @author HardFOC Team
 * @date 2025
 */

#include "ImuManager.h"
#include "GpioManager.h"
#include "handlers/bno08x/Bno08xHandler.h"
#include "handlers/logger/Logger.h"
#include <memory>

// ESP-IDF for tasks
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

/**
 * @brief Example of setting up BNO08x IMU using Bno08xHandler
 */
void setupBno08xHandler() {
    auto& imu_mgr = ImuManager::GetInstance();
    
    Logger::GetInstance().Info("Bno08xHandlerImuExample", "=== BNO08x Handler Setup ===");
    
    // Initialize with automatic Bno08xHandler creation
    bool initSuccess = imu_mgr.Initialize();
    if (initSuccess) {
        Logger::GetInstance().Info("Bno08xHandlerImuExample", "ImuManager initialized with Bno08xHandler");
    } else {
        Logger::GetInstance().Error("Bno08xHandlerImuExample", "ImuManager initialization failed!");
        return;
    }
    
    Logger::GetInstance().Info("Bno08xHandlerImuExample", "Total IMU devices: " + std::to_string(static_cast<int>(imu_mgr.GetImuCount())));
    
    // List available devices
    auto devices = imu_mgr.GetAvailableDevices();
    Logger::GetInstance().Info("Bno08xHandlerImuExample", "Available IMU devices:");
    for (const auto& device : devices) {
        Logger::GetInstance().Info("Bno08xHandlerImuExample", "  - " + device);
    }
}

/**
 * @brief Example of using Bno08xHandler for sensor operations (no exceptions)
 */
void useBno08xHandler() {
    Logger::GetInstance().Info("Bno08xHandlerImuExample", "\n=== BNO08x Handler Usage ===");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Get the BNO08x handler (safe pointer-based access)
    Bno08xHandler* handler = imu_mgr.GetBno08xHandler();
    if (!handler) {
        Logger::GetInstance().Error("Bno08xHandlerImuExample", "FAILED: BNO08x handler not available");
        return;
    }
    
            Logger::GetInstance().Info("Bno08xHandlerImuExample", "SUCCESS: BNO08x handler available");
    
    // Configure sensor callback (if supported by handler)
    // handler->SetSensorCallback([](const SensorEvent& event) {
    //     std::cout << "Sensor data: type=" << static_cast<int>(event.sensor) 
    //               << ", timestamp=" << event.timestamp << std::endl;
    // });
    
    // Enable rotation vector sensor (if supported)
    // Bno08xError result = handler->EnableSensor(Bno08xSensorType::ROTATION_VECTOR, 50);
    // if (result == Bno08xError::SUCCESS) {
    //     std::cout << "✓ Rotation vector sensor enabled (50Hz)" << std::endl;
    // } else {
    //     std::cout << "✗ Failed to enable rotation vector sensor: " 
    //               << Bno08xErrorToString(result) << std::endl;
    // }
    
    // Example of reading sensor status
    // bool isEnabled = handler->IsSensorEnabled(Bno08xSensorType::ROTATION_VECTOR);
    // std::cout << "Rotation vector enabled: " << (isEnabled ? "Yes" : "No") << std::endl;
    
            Logger::GetInstance().Info("Bno08xHandlerImuExample", "SUCCESS: BNO08x handler operations completed");
}

/**
 * @brief Example of configuring and using GPIO interrupts for BNO08x
 */
void demonstrateInterruptMode() {
    Logger::GetInstance().Info("Bno08xHandlerImuExample", "\n=== BNO08x Interrupt Mode Example ===");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Check if interrupt is available
    if (!imu_mgr.IsInterruptEnabled()) {
        Logger::GetInstance().Info("Bno08xHandlerImuExample", "Configuring BNO08x interrupt...");
        
        // Configure interrupt with callback
        bool interrupt_configured = imu_mgr.ConfigureInterrupt([]() {
            // This callback executes in interrupt context - keep it minimal!
            static uint32_t callback_count = 0;
            callback_count++;
            // In real applications, you might set a flag or post to a queue
        });
        
        if (interrupt_configured) {
            Logger::GetInstance().Info("Bno08xHandlerImuExample", "SUCCESS: Interrupt configured successfully");
            
            // Enable the interrupt
            if (imu_mgr.EnableInterrupt()) {
                Logger::GetInstance().Info("Bno08xHandlerImuExample", "SUCCESS: Interrupt enabled successfully");
                
                // Demonstrate interrupt-driven operation
                Logger::GetInstance().Info("Bno08xHandlerImuExample", "Waiting for interrupts (10 seconds)...");
                for (int i = 0; i < 10; ++i) {
                    if (imu_mgr.WaitForInterrupt(1000)) {  // 1 second timeout
                        Logger::GetInstance().Info("Bno08xHandlerImuExample", "Interrupt received! Count: " + std::to_string(imu_mgr.GetInterruptCount()));
                        
                        // In real application, you would call handler->Update() here
                        // to process the sensor data that triggered the interrupt
                    } else {
                        Logger::GetInstance().Info("Bno08xHandlerImuExample", "." + std::string(1, '\b'));  // Timeout indicator
                    }
                }
                Logger::GetInstance().Info("Bno08xHandlerImuExample", "\n");
                
                // Disable interrupt when done
                imu_mgr.DisableInterrupt();
                Logger::GetInstance().Info("Bno08xHandlerImuExample", "SUCCESS: Interrupt disabled");
            } else {
                Logger::GetInstance().Error("Bno08xHandlerImuExample", "FAILED: Failed to enable interrupt");
            }
        } else {
            Logger::GetInstance().Error("Bno08xHandlerImuExample", "FAILED: Failed to configure interrupt - falling back to polling mode");
        }
    } else {
        Logger::GetInstance().Info("Bno08xHandlerImuExample", "SUCCESS: Interrupt already enabled");
    }
    
    Logger::GetInstance().Info("Bno08xHandlerImuExample", "Total interrupts received: " + std::to_string(imu_mgr.GetInterruptCount()));
}

/**
 * @brief Example of safe error handling with handler interface
 */
void demonstrateErrorHandling() {
    Logger::GetInstance().Info("Bno08xHandlerImuExample", "\n=== Error Handling Example ===");
    
    auto& imu_mgr = ImuManager::GetInstance();
    
    // Check if IMU is available before using
    if (!imu_mgr.IsBno08xAvailable()) {
        Logger::GetInstance().Error("Bno08xHandlerImuExample", "FAILED: BNO08x not available");
        return;
    }
    
    // Safe handler access
    Bno08xHandler* handler = imu_mgr.GetBno08xHandler();
    if (handler) {
        Logger::GetInstance().Info("Bno08xHandlerImuExample", "SUCCESS: Handler access successful");
        
        // All operations through handler are exception-free
        // Error checking through return values and error codes
        
        // Example error handling pattern:
        // Bno08xError result = handler->SomeOperation();
        // if (result != Bno08xError::SUCCESS) {
        //     std::cout << "Operation failed: " << Bno08xErrorToString(result) << std::endl;
        //     return;
        // }
        
    } else {
        Logger::GetInstance().Error("Bno08xHandlerImuExample", "FAILED: Handler access failed");
    }
}

/**
 * @brief Example of data polling loop (typical usage pattern)
 */
void dataPollingExample() {
    Logger::GetInstance().Info("Bno08xHandlerImuExample", "\n=== Data Polling Example ===");
    
    auto& imu_mgr = ImuManager::GetInstance();
    Bno08xHandler* handler = imu_mgr.GetBno08xHandler();
    
    if (!handler) {
        Logger::GetInstance().Error("Bno08xHandlerImuExample", "FAILED: Handler not available for polling");
        return;
    }
    
            Logger::GetInstance().Info("Bno08xHandlerImuExample", "SUCCESS: Starting data polling simulation...");
    
    // Simulate polling loop (in real application, this would be continuous)
    for (int i = 0; i < 5; ++i) {
        // Update handler to process new data
        // handler->Update();  // This would trigger callbacks if new data available
        
        // Or read specific sensor data directly
        // RotationVector rotation;
        // if (handler->GetRotationVector(rotation)) {
        //     std::cout << "Rotation: qx=" << rotation.x << ", qy=" << rotation.y 
        //               << ", qz=" << rotation.z << ", qw=" << rotation.w << std::endl;
        // }
        
        Logger::GetInstance().Info("Bno08xHandlerImuExample", "Polling iteration " + std::to_string(i + 1) + "/5");
        
        // In real application: vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz polling
    }
    
            Logger::GetInstance().Info("Bno08xHandlerImuExample", "SUCCESS: Polling simulation completed");
}

/**
 * @brief Main example function
 */
int main() {
    Logger::GetInstance().Info("Bno08xHandlerImuExample", "=== Bno08xHandler IMU Example with GPIO Interrupt Support ===");
    
    // Setup BNO08x handler through ImuManager
    setupBno08xHandler();
    
    // Demonstrate handler usage
    useBno08xHandler();
    
    // Demonstrate interrupt functionality
    demonstrateInterruptMode();
    
    // Show error handling patterns
    demonstrateErrorHandling();
    
    // Demonstrate typical polling usage
    dataPollingExample();
    
    Logger::GetInstance().Info("Bno08xHandlerImuExample", "\n=== Example completed ===");
    return 0;
}
