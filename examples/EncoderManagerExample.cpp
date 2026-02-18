/**
 * @file EncoderManagerExample.cpp
 * @brief Comprehensive example demonstrating EncoderManager usage with AS5047U encoder handlers.
 *
 * This example shows how to:
 * - Initialize the EncoderManager singleton (following ImuManager pattern)
 * - Access onboard AS5047U encoder (automatic initialization)
 * - Create external AS5047U devices dynamically
 * - Read angle and velocity measurements from multiple encoders
 * - Monitor encoder health and diagnostics
 * - Handle multiple encoders simultaneously
 * - Use encoder data for closed-loop control applications
 * - Handle interrupts for high-frequency encoder updates
 *
 * Hardware Requirements:
 * - HardFOC board with onboard AS5047U encoder
 * - Optional: External AS5047U encoders connected via SPI
 * - Proper magnetic field for sensor operation (4-7mm magnet distance)
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 * @copyright HardFOC
 */

#include <iomanip>
#include <vector>
#include <cmath>
#include <functional>

#include "managers/EncoderManager.h"
#include "managers/CommChannelsManager.h"
#include "handlers/logger/Logger.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"

// Example configuration
constexpr uint8_t EXAMPLE_RUNTIME_SECONDS = 30;
constexpr uint16_t MEASUREMENT_INTERVAL_MS = 50;
constexpr double VELOCITY_THRESHOLD_RPM = 5.0;

/**
 * @brief Basic encoder access example (mirrors ImuManager pattern)
 */
void BasicEncoderExample() {
    Logger::GetInstance().Info("EncoderExample", "=== BASIC ENCODER EXAMPLE ===");
    
    // Get EncoderManager singleton and ensure initialization (like ImuManager)
    auto& encoder_mgr = EncoderManager::GetInstance();
    if (!encoder_mgr.EnsureInitialized()) {
        Logger::GetInstance().Error("EncoderExample", "Failed to initialize EncoderManager");
        return;
    }
    
    Logger::GetInstance().Info("EncoderExample", "EncoderManager initialized successfully");
    Logger::GetInstance().Info("EncoderExample", "Active encoders: %d", static_cast<int>(encoder_mgr.GetDeviceCount()));
    
    // Read angle via manager high-level API (device 0 = onboard)
    Logger::GetInstance().Info("EncoderExample", "Onboard AS5047U (Device 0):");
    
    uint16_t angle_lsb;
    As5047uError result = encoder_mgr.ReadAngle(0, angle_lsb);
    
    if (result == As5047uError::SUCCESS) {
        double angle_deg = static_cast<double>(angle_lsb) * (360.0 / 16384.0);
        Logger::GetInstance().Info("EncoderExample", "  Raw Angle: %u LSB", angle_lsb);
        Logger::GetInstance().Info("EncoderExample", "  Angle: %.2f°", angle_deg);
        
        // Read velocity via manager
        double velocity_rpm;
        if (encoder_mgr.ReadVelocityRPM(0, velocity_rpm) == As5047uError::SUCCESS) {
            Logger::GetInstance().Info("EncoderExample", "  Velocity: %.1f RPM", velocity_rpm);
        }
        
        // Read diagnostics via manager
        As5047uDiagnostics diagnostics;
        if (encoder_mgr.ReadDiagnostics(0, diagnostics) == As5047uError::SUCCESS) {
            Logger::GetInstance().Info("EncoderExample", "  Health Status:");
            Logger::GetInstance().Info("EncoderExample", "    Magnetic Field: %s", diagnostics.magnetic_field_ok ? "OK" : "ERROR");
            Logger::GetInstance().Info("EncoderExample", "    Communication: %s", diagnostics.communication_ok ? "OK" : "ERROR");
            Logger::GetInstance().Info("EncoderExample", "    AGC Warning: %s", diagnostics.agc_warning ? "YES" : "NO");
        }
    } else {
        Logger::GetInstance().Error("EncoderExample", "Failed to read onboard encoder: %s", As5047uErrorToString(result));
    }
    
    // Access driver directly for advanced operations
    auto sensor_driver = encoder_mgr.GetAs5047uDriver(0);
    if (sensor_driver) {
        Logger::GetInstance().Info("EncoderExample", "  Direct AS5047U driver access: Available");
    }
}

/**
 * @brief External device creation example (mirrors ImuManager external device pattern)
 */
void ExternalDeviceExample() {
    Logger::GetInstance().Info("EncoderExample", "=== EXTERNAL DEVICE EXAMPLE ===");
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Check external slot availability
    Logger::GetInstance().Info("EncoderExample", "External Device Slot Availability:");
    for (uint8_t i = 1; i <= 3; ++i) {
        bool available = encoder_mgr.IsExternalSlotAvailable(i);
        Logger::GetInstance().Info("EncoderExample", "  Slot %d: %s", static_cast<int>(i), available ? "AVAILABLE" : "OCCUPIED");
    }
    
    // Try to create external AS5047U devices (like ImuManager creates external BNO08x)
    Logger::GetInstance().Info("EncoderExample", "Attempting to create external AS5047U devices...");
    
    // Method 1: Using SPI device ID (like ImuManager SPI creation)
    /*
    bool ext1_created = encoder_mgr.CreateExternalAs5047uDevice(1, SpiDeviceId::EXTERNAL_DEVICE_1);
    if (ext1_created) {
        Logger::GetInstance().Info("EncoderExample", "External AS5047U device 1 created successfully");
        
        // Access the external handler
        // Set zero position via manager
        encoder_mgr.SetZeroPosition(1, 0);  // Set zero reference
        Logger::GetInstance().Info("EncoderExample", "External encoder 1 configured");
    } else {
        Logger::GetInstance().Error("EncoderExample", "Failed to create external AS5047U device 1");
    }
    */
    
    // Method 2: Using direct SPI interface (like ImuManager direct interface creation)
    /*
    auto& comm_mgr = CommChannelsManager::GetInstance();
    BaseSpi* external_spi = comm_mgr.GetSpiDevice(SpiDeviceId::EXTERNAL_DEVICE_2);
    if (external_spi) {
        bool ext2_created = encoder_mgr.CreateExternalAs5047uDevice(2, *external_spi);
        if (ext2_created) {
            Logger::GetInstance().Info("EncoderExample", "External AS5047U device 2 (direct SPI) created successfully");
        }
    }
    */
    
    Logger::GetInstance().Info("EncoderExample", "Note: External device creation requires actual hardware connections");
    
    // Show all active devices
    auto active_devices = encoder_mgr.GetActiveDeviceIndices();
    Logger::GetInstance().Info("EncoderExample", "Active encoder devices:");
    for (uint8_t device_idx : active_devices) {
        std::string device_type = encoder_mgr.GetDeviceType(device_idx);
        Logger::GetInstance().Info("EncoderExample", "  Device %d: %s", static_cast<int>(device_idx), device_type.c_str());
        if (device_idx == 0) {
            Logger::GetInstance().Info("EncoderExample", "    (Onboard)");
        } else {
            Logger::GetInstance().Info("EncoderExample", "    (External)");
        }
    }
}

/**
 * @brief High-level encoder operations example (using manager convenience methods)
 */
void HighLevelOperationsExample() {
    Logger::GetInstance().Info("EncoderExample", "=== HIGH-LEVEL OPERATIONS EXAMPLE ===");
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Read angle using manager convenience method
    double angle_degrees;
    As5047uError result = encoder_mgr.ReadAngleDegrees(0, angle_degrees);
    if (result == As5047uError::SUCCESS) {
        Logger::GetInstance().Info("EncoderExample", "Onboard encoder angle: %.2f°", angle_degrees);
    }
    
    // Read velocity using manager convenience method
    double velocity_rpm;
    result = encoder_mgr.ReadVelocityRPM(0, velocity_rpm);
    if (result == As5047uError::SUCCESS) {
        Logger::GetInstance().Info("EncoderExample", "Onboard encoder velocity: %.1f RPM", velocity_rpm);
    }
    
    // Set zero position using manager convenience method
    Logger::GetInstance().Info("EncoderExample", "Setting current position as zero reference...");
    uint16_t current_lsb = static_cast<uint16_t>(angle_degrees * (16384.0 / 360.0));
    result = encoder_mgr.SetZeroPosition(0, current_lsb);
    if (result == As5047uError::SUCCESS) {
        Logger::GetInstance().Info("EncoderExample", "Zero position set successfully");
        
        // Verify the change
        os_delay_msec(100);
        double new_angle;
        if (encoder_mgr.ReadAngleDegrees(0, new_angle) == As5047uError::SUCCESS) {
            Logger::GetInstance().Info("EncoderExample", "New angle reading: %.2f°", new_angle);
        }
    }
    
    // Read all active encoders at once
    std::vector<uint16_t> angles;
    std::vector<uint8_t> device_indices;
    std::vector<As5047uError> errors = encoder_mgr.ReadAllAngles(angles, device_indices);
    
    Logger::GetInstance().Info("EncoderExample", "All Active Encoders:");
    for (size_t i = 0; i < device_indices.size(); ++i) {
        if (errors[i] == As5047uError::SUCCESS) {
            double angle_deg = static_cast<double>(angles[i]) * (360.0 / 16384.0);
            Logger::GetInstance().Info("EncoderExample", "  Device %d: %.2f° (%u LSB)", 
                static_cast<int>(device_indices[i]), angle_deg, angles[i]);
        } else {
            Logger::GetInstance().Error("EncoderExample", "  Device %d: ERROR - %s", 
                static_cast<int>(device_indices[i]), As5047uErrorToString(errors[i]));
        }
    }
    
    // Check system health
    bool all_healthy = encoder_mgr.CheckAllDevicesHealth();
    Logger::GetInstance().Info("EncoderExample", "System Health: %s", all_healthy ? "HEALTHY" : "ISSUES DETECTED");
}

/**
 * @brief Continuous monitoring example with statistics (like ImuManager sensor monitoring)
 */
void ContinuousMonitoringExample() {
    Logger::GetInstance().Info("EncoderExample", "=== CONTINUOUS MONITORING EXAMPLE ===");
    Logger::GetInstance().Info("EncoderExample", "Monitoring encoders for %d seconds...", EXAMPLE_RUNTIME_SECONDS);
    Logger::GetInstance().Info("EncoderExample", "Rotate the encoder to see real-time data");
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    uint32_t start_time_ms = os_get_elapsed_time_msec();
    uint32_t last_measurement_ms = start_time_ms;
    
    // Statistics tracking
    uint32_t measurement_count = 0;
    double min_angle = 360.0;
    double max_angle = 0.0;
    double total_rotation = 0.0;
    double last_angle = 0.0;
    bool first_measurement = true;
    
    while (true) {
        uint32_t current_time_ms = os_get_elapsed_time_msec();
        uint32_t elapsed_seconds = (current_time_ms - start_time_ms) / 1000;
        
        if (elapsed_seconds >= EXAMPLE_RUNTIME_SECONDS) {
            break;
        }
        
        // Take measurement every MEASUREMENT_INTERVAL_MS
        uint32_t time_since_last_ms = current_time_ms - last_measurement_ms;
        if (time_since_last_ms >= MEASUREMENT_INTERVAL_MS) {
            double angle_degrees;
            double velocity_rpm;
            
            As5047uError angle_result = encoder_mgr.ReadAngleDegrees(0, angle_degrees);
            As5047uError velocity_result = encoder_mgr.ReadVelocityRPM(0, velocity_rpm);
            
            if (angle_result == As5047uError::SUCCESS) {
                measurement_count++;
                
                // Update statistics
                if (angle_degrees < min_angle) min_angle = angle_degrees;
                if (angle_degrees > max_angle) max_angle = angle_degrees;
                
                // Calculate total rotation (handle wraparound)
                if (!first_measurement) {
                    double angle_diff = angle_degrees - last_angle;
                    
                    // Handle 360° wraparound
                    if (angle_diff > 180.0) {
                        angle_diff -= 360.0;
                    } else if (angle_diff < -180.0) {
                        angle_diff += 360.0;
                    }
                    
                    total_rotation += std::abs(angle_diff);
                }
                
                last_angle = angle_degrees;
                first_measurement = false;
                
                // Display current readings
                Logger::GetInstance().Info("EncoderExample", "Time: %2ds | Angle: %6.1f° | ", elapsed_seconds, angle_degrees);
                
                if (velocity_result == As5047uError::SUCCESS) {
                    Logger::GetInstance().Info("EncoderExample", "Velocity: %6.1f RPM | ", velocity_rpm);
                    
                    // Indicate if encoder is moving
                    if (std::abs(velocity_rpm) > VELOCITY_THRESHOLD_RPM) {
                        Logger::GetInstance().Info("EncoderExample", "MOVING");
                    } else {
                        Logger::GetInstance().Info("EncoderExample", "STATIC");
                    }
                } else {
                    Logger::GetInstance().Info("EncoderExample", "Velocity: ERROR");
                }
            } else {
                Logger::GetInstance().Error("EncoderExample", "Encoder read error: %s", As5047uErrorToString(angle_result));
            }
            
            last_measurement_ms = current_time_ms;
        }
        
        os_delay_msec(10);
    }
    
    // Display final statistics
    Logger::GetInstance().Info("EncoderExample", "=== MONITORING STATISTICS ===");
    Logger::GetInstance().Info("EncoderExample", "Total Measurements: %u", measurement_count);
    Logger::GetInstance().Info("EncoderExample", "Measurement Rate: %.1f Hz", 
        (measurement_count * 1000.0 / (EXAMPLE_RUNTIME_SECONDS * 1000)));
    Logger::GetInstance().Info("EncoderExample", "Angle Range: %.1f° to %.1f°", min_angle, max_angle);
    Logger::GetInstance().Info("EncoderExample", "Total Rotation: %.1f°", total_rotation);
    Logger::GetInstance().Info("EncoderExample", "Average Rotation: %.2f°/s", 
        (total_rotation / EXAMPLE_RUNTIME_SECONDS));
}

/**
 * @brief Sensor monitoring example (AS5047U is a continuous reading sensor)
 */
void SensorMonitoringExample() {
    Logger::GetInstance().Info("EncoderExample", "=== SENSOR MONITORING EXAMPLE ===");
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Perform some measurements to generate statistics
    Logger::GetInstance().Info("EncoderExample", "Performing measurements to generate monitoring data...");
    
    for (int i = 0; i < 10; ++i) {
        double angle_degrees;
        As5047uError result = encoder_mgr.ReadAngleDegrees(0, angle_degrees);
        
        if (result == As5047uError::SUCCESS) {
            Logger::GetInstance().Info("EncoderExample", "Measurement %d: %.2f°", (i + 1), angle_degrees);
        } else {
            Logger::GetInstance().Error("EncoderExample", "Measurement %d: ERROR - %s", (i + 1), As5047uErrorToString(result));
        }
        
        os_delay_msec(100);
    }
    
    // Display monitoring statistics
    Logger::GetInstance().Info("EncoderExample", "Monitoring Statistics:");
    Logger::GetInstance().Info("EncoderExample", "  Measurements: %u", encoder_mgr.GetMeasurementCount(0));
    Logger::GetInstance().Info("EncoderExample", "  Communication Errors: %u", encoder_mgr.GetCommunicationErrorCount(0));
    
    // Note: AS5047U is a continuous reading sensor that doesn't use interrupts
    Logger::GetInstance().Info("EncoderExample", "Note: AS5047U is a continuous reading sensor - no interrupts needed!");
}

/**
 * @brief Device initialization and status example
 */
void DeviceStatusExample() {
    Logger::GetInstance().Info("EncoderExample", "=== DEVICE STATUS EXAMPLE ===");
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Initialize all devices
    Logger::GetInstance().Info("EncoderExample", "Initializing all devices...");
    std::vector<bool> init_results = encoder_mgr.InitializeAllDevices();
    
    Logger::GetInstance().Info("EncoderExample", "Initialization results:");
    auto active_indices = encoder_mgr.GetActiveDeviceIndices();
    for (size_t i = 0; i < init_results.size() && i < active_indices.size(); ++i) {
        Logger::GetInstance().Info("EncoderExample", "  Device %d: %s", 
            static_cast<int>(active_indices[i]), init_results[i] ? "SUCCESS" : "FAILED");
    }
    
    // Get initialization status
    std::vector<bool> status = encoder_mgr.GetInitializationStatus();
    Logger::GetInstance().Info("EncoderExample", "Current initialization status:");
    for (size_t i = 0; i < status.size() && i < active_indices.size(); ++i) {
        Logger::GetInstance().Info("EncoderExample", "  Device %d: %s", 
            static_cast<int>(active_indices[i]), status[i] ? "INITIALIZED" : "NOT INITIALIZED");
    }
    
    // Get available devices info
    auto available_devices = encoder_mgr.GetAvailableDevices();
    Logger::GetInstance().Info("EncoderExample", "Available devices:");
    for (const auto& device : available_devices) {
        Logger::GetInstance().Info("EncoderExample", "  %s", device.c_str());
    }
    
    // Check device validity
    Logger::GetInstance().Info("EncoderExample", "Device validity check:");
    for (uint8_t i = 0; i < 4; ++i) {
        bool valid = encoder_mgr.IsDeviceValid(i);
        Logger::GetInstance().Info("EncoderExample", "  Device %d: %s", static_cast<int>(i), valid ? "VALID" : "INVALID");
    }
    
    // Dump comprehensive statistics (like ImuManager)
    Logger::GetInstance().Info("EncoderExample", "=== COMPREHENSIVE STATISTICS ===");
    encoder_mgr.DumpStatistics();
}

/**
 * @brief Closed-loop control integration example
 */
void ClosedLoopControlExample() {
    Logger::GetInstance().Info("EncoderExample", "=== CLOSED-LOOP CONTROL EXAMPLE ===");
    Logger::GetInstance().Info("EncoderExample", "This example shows encoder feedback integration for motor control");
    
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Set target position
    double target_angle = 180.0; // 180 degrees
    double tolerance = 2.0;      // ±2 degree tolerance
    
    Logger::GetInstance().Info("EncoderExample", "Target Position: %.1f°", target_angle);
    Logger::GetInstance().Info("EncoderExample", "Position Tolerance: ±%.1f°", tolerance);
    
    // Simple position control loop (conceptual - would integrate with MotorController)
    for (int i = 0; i < 20; ++i) {
        double current_angle;
        As5047uError result = encoder_mgr.ReadAngleDegrees(0, current_angle);
        
        if (result == As5047uError::SUCCESS) {
            double position_error = target_angle - current_angle;
            
            // Handle angle wraparound
            if (position_error > 180.0) {
                position_error -= 360.0;
            } else if (position_error < -180.0) {
                position_error += 360.0;
            }
            
            Logger::GetInstance().Info("EncoderExample", "Step %2d: Current=%6.1f° | Error=%6.1f° | ", 
                i + 1, current_angle, position_error);
            
            if (std::abs(position_error) <= tolerance) {
                Logger::GetInstance().Info("EncoderExample", "TARGET REACHED SUCCESS");
                break;
            } else {
                // Calculate control output (simple proportional control)
                double control_gain = 0.1;
                double control_output = control_gain * position_error;
                Logger::GetInstance().Info("EncoderExample", "Control=%6.1f", control_output);
                
                // In a real system, this would be:
                // auto& motor_mgr = MotorController::GetInstance();
                // motor_mgr.SetVelocity(0, control_output);
            }
        } else {
            Logger::GetInstance().Error("EncoderExample", "Encoder read error: %s", As5047uErrorToString(result));
        }
        
        os_delay_msec(100);
    }
}

/**
 * @brief Main example function
 */
int main() {
    Logger::GetInstance().Info("EncoderExample", "=== ENCODER MANAGER EXAMPLE ===");
    Logger::GetInstance().Info("EncoderExample", "This example demonstrates EncoderManager usage following ImuManager patterns");
    Logger::GetInstance().Info("EncoderExample", "Hardware: HardFOC board with AS5047U encoder");
    
    try {
        // Initialize the logger
        Logger::GetInstance().Initialize();
        Logger::GetInstance().SetLogLevel("EncoderExample", LogLevel::INFO);
        
        // Run all examples
        BasicEncoderExample();
        ExternalDeviceExample();
        HighLevelOperationsExample();
        DeviceStatusExample();
        SensorMonitoringExample();
        ContinuousMonitoringExample();
        ClosedLoopControlExample();
        
        Logger::GetInstance().Info("EncoderExample", "=== EXAMPLE COMPLETED ===");
        Logger::GetInstance().Info("EncoderExample", "All encoder manager examples executed successfully!");
        
    } catch (const std::exception& e) {
        Logger::GetInstance().Error("EncoderExample", "Example failed with exception: %s", e.what());
        return -1;
    }
    
    return 0;
}

/**
 * @brief Simple usage demonstration for quick reference
 */
void QuickUsageExample() {
    // Get EncoderManager singleton (like ImuManager)
    auto& encoder_mgr = EncoderManager::GetInstance();
    
    // Ensure initialization (automatic onboard device creation)
    if (!encoder_mgr.EnsureInitialized()) {
        Logger::GetInstance().Error("EncoderExample", "Initialization failed");
        return;
    }
    
    // Read encoder angle via manager (device 0 = onboard)
    uint16_t angle;
    if (encoder_mgr.ReadAngle(0, angle) == As5047uError::SUCCESS) {
        Logger::GetInstance().Info("EncoderExample", "Encoder angle: %u LSB", angle);
    }
    
    // Or use high-level convenience methods
    double angle_degrees;
    if (encoder_mgr.ReadAngleDegrees(0, angle_degrees) == As5047uError::SUCCESS) {
        Logger::GetInstance().Info("EncoderExample", "Encoder angle: %.2f°", angle_degrees);
    }
    
    // Check system health
    bool healthy = encoder_mgr.CheckAllDevicesHealth();
    Logger::GetInstance().Info("EncoderExample", "System health: %s", healthy ? "OK" : "ERROR");
}