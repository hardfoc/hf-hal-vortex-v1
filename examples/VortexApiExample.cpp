/**
 * @file VortexApiExample.cpp
 * @brief Comprehensive example demonstrating the Vortex API singleton usage.
 * 
 * @details This example shows how to use the unified Vortex API to access all
 *          component handlers in a clean, intuitive way. It demonstrates proper
 *          initialization, error handling, and usage of all available systems.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 * 
 * Usage:
 * 1. Build and flash this example to your ESP32-C6 board
 * 2. Monitor the serial output to see the Vortex API in action
 * 3. Observe the initialization sequence and component status
 * 4. See how all systems work together through the unified API
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>

// Include the Vortex API
#include "api/Vortex.h"
// For direct sensor access (IBno08xDriverOps, BNO085Sensor)
#include "handlers/bno08x/Bno08xHandler.h"

//==============================================================================
// GLOBAL VARIABLES
//==============================================================================

static const char* TAG = "VortexApiExample";

//==============================================================================
// HELPER FUNCTIONS
//==============================================================================

/**
 * @brief Print system diagnostics information
 */
void PrintSystemDiagnostics(const VortexSystemDiagnostics& diagnostics) {
    printf("\n=== Vortex API System Diagnostics ===\n");
    printf("Overall System Health: %s\n", diagnostics.system_healthy ? "HEALTHY" : "UNHEALTHY");
    printf("Total Components: %u\n", diagnostics.total_components);
    printf("Initialized Components: %u\n", diagnostics.initialized_components);
    printf("Failed Components: %u\n", diagnostics.failed_components);
    printf("Initialization Time: %llu ms\n", diagnostics.initialization_time_ms);
    printf("System Uptime: %llu ms\n", diagnostics.system_uptime_ms);
    
    printf("\nComponent Status:\n");
    printf("  Communication Channels: %s\n", diagnostics.comms_initialized ? "OK" : "FAIL");
    printf("  GPIO Management: %s\n", diagnostics.gpio_initialized ? "OK" : "FAIL");
    printf("  Motor Controllers: %s\n", diagnostics.motors_initialized ? "OK" : "FAIL");
    printf("  ADC Management: %s\n", diagnostics.adc_initialized ? "OK" : "FAIL");
    printf("  IMU Management: %s\n", diagnostics.imu_initialized ? "OK" : "FAIL");
    printf("  Encoder Management: %s\n", diagnostics.encoders_initialized ? "OK" : "FAIL");
    printf("  LED Management: %s\n", diagnostics.leds_initialized ? "OK" : "FAIL");
    printf("  Temperature Management: %s\n", diagnostics.temp_initialized ? "OK" : "FAIL");
    
    if (diagnostics.failed_components_list_count > 0) {
        printf("\nFailed Components:\n");
        for (size_t i = 0; i < diagnostics.failed_components_list_count; ++i) {
            printf("  - %s\n", diagnostics.failed_components_list[i]);
        }
    }
    
    if (diagnostics.warnings_count > 0) {
        printf("\nSystem Warnings:\n");
        for (size_t i = 0; i < diagnostics.warnings_count; ++i) {
            printf("  - %s\n", diagnostics.warnings[i]);
        }
    }
    
    printf("=== End System Diagnostics ===\n\n");
}

/**
 * @brief Demonstrate communication channels usage
 */
void DemonstrateComms(Vortex& vortex) {
    printf("=== Communication Channels Demo ===\n");
    
    auto& comms = vortex.comms;
    
    // Get SPI device for motor controller
    auto* spi_device = comms.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
    if (spi_device) {
        printf("✓ TMC9660 SPI device available\n");
    } else {
        printf("✗ TMC9660 SPI device not available\n");
    }
    
    // Get I2C device for IMU
    auto* i2c_device = comms.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (i2c_device) {
        printf("✓ BNO08x I2C device available\n");
    } else {
        printf("✗ BNO08x I2C device not available\n");
    }
    
    // Get I2C device for GPIO expander
    auto* gpio_i2c = comms.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (gpio_i2c) {
        printf("✓ PCAL9555 I2C device available\n");
    } else {
        printf("✗ PCAL9555 I2C device not available\n");
    }
    
    printf("=== End Communication Channels Demo ===\n\n");
}

/**
 * @brief Demonstrate GPIO management usage
 */
void DemonstrateGpio(Vortex& vortex) {
    printf("=== GPIO Management Demo ===\n");
    
    auto& gpio = vortex.gpio;
    
    // Try to access some GPIO pins
    bool led_pin_available = gpio.Contains("LED_STATUS");
    printf("LED Status Pin Available: %s\n", led_pin_available ? "Yes" : "No");
    
    bool motor_enable_pin_available = gpio.Contains("MOTOR_ENABLE");
    printf("Motor Enable Pin Available: %s\n", motor_enable_pin_available ? "Yes" : "No");
    
    bool imu_interrupt_pin_available = gpio.Contains("IMU_INTERRUPT");
    printf("IMU Interrupt Pin Available: %s\n", imu_interrupt_pin_available ? "Yes" : "No");
    
    printf("=== End GPIO Management Demo ===\n\n");
}

/**
 * @brief Demonstrate motor controller usage
 */
void DemonstrateMotors(Vortex& vortex) {
    printf("=== Motor Controller Demo ===\n");
    
    auto& motors = vortex.motors;
    
    // Get the onboard motor controller handler
    auto* handler = motors.handler(0);
    if (handler) {
        printf("✓ Onboard TMC9660 handler available\n");
        
        // Access the underlying driver via visitor pattern
        motors.visitDriver([](auto& driver) {
            printf("✓ TMC9660 driver available (via visitDriver)\n");
        }, 0);
    } else {
        printf("✗ Onboard TMC9660 handler not available\n");
    }
    
    // Check device count
    uint8_t device_count = motors.GetDeviceCount();
    printf("Active Motor Devices: %u\n", device_count);
    
    printf("=== End Motor Controller Demo ===\n\n");
}

/**
 * @brief Demonstrate ADC management usage
 */
void DemonstrateAdc(Vortex& vortex) {
    printf("=== ADC Management Demo ===\n");
    
    auto& adc = vortex.adc;
    
    // Try to read from some ADC channels
    float voltage;
    if (adc.ReadChannelV("MOTOR_CURRENT", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        printf("✓ Motor Current ADC: %.3f V\n", voltage);
    } else {
        printf("✗ Motor Current ADC read failed\n");
    }
    
    if (adc.ReadChannelV("MOTOR_VOLTAGE", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        printf("✓ Motor Voltage ADC: %.3f V\n", voltage);
    } else {
        printf("✗ Motor Voltage ADC read failed\n");
    }
    
    if (adc.ReadChannelV("TEMP_SENSOR", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        printf("✓ Temperature Sensor ADC: %.3f V\n", voltage);
    } else {
        printf("✗ Temperature Sensor ADC read failed\n");
    }
    
    printf("=== End ADC Management Demo ===\n\n");
}

/**
 * @brief Demonstrate IMU management usage
 */
void DemonstrateImu(Vortex& vortex) {
    printf("=== IMU Management Demo ===\n");
    
    auto& imu = vortex.imu;
    
    // Get the onboard IMU handler (high-level API: Update, ReadQuaternion, etc.)
    auto* handler = imu.GetBno08xHandler(0);
    if (handler) {
        printf("✓ Onboard BNO08x handler available\n");
        // Get direct sensor driver for low-level API (Update, EnableSensor, GetLatest, SetCallback, ...)
        IBno08xDriverOps* sensor = imu.GetSensor(0);
        if (sensor && handler->IsInitialized()) {
            printf("✓ Sensor driver available (e.g. sensor->Update(), sensor->GetLatest(...))\n");
        }
    } else {
        printf("✗ Onboard BNO08x handler not available\n");
    }
    
    // Check device count
    uint8_t device_count = imu.GetDeviceCount();
    printf("Active IMU Devices: %u\n", device_count);
    
    printf("=== End IMU Management Demo ===\n\n");
}

/**
 * @brief Demonstrate encoder management usage
 */
void DemonstrateEncoders(Vortex& vortex) {
    printf("=== Encoder Management Demo ===\n");
    
    auto& encoders = vortex.encoders;
    
    // Get the onboard encoder handler
    auto* handler = encoders.GetAs5047uHandler(0);
    if (handler) {
        printf("✓ Onboard AS5047U handler available\n");
        
        // Try to read encoder angle
        uint16_t angle;
        if (encoders.ReadAngle(0, angle) == As5047uError::SUCCESS) {
            printf("✓ Encoder Angle: %u LSB\n", angle);
        } else {
            printf("✗ Encoder angle read failed\n");
        }
        
        // Try to read encoder velocity
        double velocity_rpm;
        if (encoders.ReadVelocityRPM(0, velocity_rpm) == As5047uError::SUCCESS) {
            printf("✓ Encoder Velocity: %.2f RPM\n", velocity_rpm);
        } else {
            printf("✗ Encoder velocity read failed\n");
        }
    } else {
        printf("✗ Onboard AS5047U handler not available\n");
    }
    
    // Check device count
    uint8_t device_count = encoders.GetDeviceCount();
    printf("Active Encoder Devices: %u\n", device_count);
    
    printf("=== End Encoder Management Demo ===\n\n");
}

/**
 * @brief Demonstrate LED management usage
 */
void DemonstrateLeds(Vortex& vortex) {
    printf("=== LED Management Demo ===\n");
    
    auto& leds = vortex.leds;
    
    // Set LED to green (system OK)
    if (leds.SetStatus(LedAnimation::STATUS_OK) == LedError::SUCCESS) {
        printf("✓ LED set to system OK (green)\n");
    } else {
        printf("✗ Failed to set LED status\n");
    }
    
    // Get current LED color
    LedColor current_color;
    if (leds.GetCurrentColor(current_color) == LedError::SUCCESS) {
        printf("✓ Current LED Color: R=%u, G=%u, B=%u\n", 
               current_color.red, current_color.green, current_color.blue);
    } else {
        printf("✗ Failed to get LED color\n");
    }
    
    // Get current brightness
    uint8_t brightness;
    if (leds.GetCurrentBrightnessPercent(brightness) == LedError::SUCCESS) {
        printf("✓ Current LED Brightness: %u%%\n", brightness);
    } else {
        printf("✗ Failed to get LED brightness\n");
    }
    
    printf("=== End LED Management Demo ===\n\n");
}

/**
 * @brief Demonstrate temperature management usage
 */
void DemonstrateTemperature(Vortex& vortex) {
    printf("=== Temperature Management Demo ===\n");
    
    auto& temp = vortex.temp;
    
    // Try to read from different temperature sensors
    float temperature;
    
    if (temp.ReadTemperatureCelsius("ESP32_INTERNAL", &temperature) == hf_temp_err_t::TEMP_SUCCESS) {
        printf("✓ ESP32 Internal Temperature: %.2f°C\n", temperature);
    } else {
        printf("✗ ESP32 Internal Temperature read failed\n");
    }
    
    if (temp.ReadTemperatureCelsius("MOTOR_TEMP", &temperature) == hf_temp_err_t::TEMP_SUCCESS) {
        printf("✓ Motor Temperature: %.2f°C\n", temperature);
    } else {
        printf("✗ Motor Temperature read failed\n");
    }
    
    if (temp.ReadTemperatureCelsius("NTC_THERMISTOR", &temperature) == hf_temp_err_t::TEMP_SUCCESS) {
        printf("✓ NTC Thermistor Temperature: %.2f°C\n", temperature);
    } else {
        printf("✗ NTC Thermistor Temperature read failed\n");
    }
    
    printf("=== End Temperature Management Demo ===\n\n");
}

//==============================================================================
// MAIN APPLICATION
//==============================================================================

extern "C" void app_main(void) {
    printf("\n");
    printf("========================================\n");
    printf("  Vortex API Example - HardFOC Platform\n");
    printf("========================================\n");
    printf("This example demonstrates the unified Vortex API\n");
    printf("that provides access to all component handlers.\n\n");
    
    // Get the Vortex API instance
    auto& vortex = Vortex::GetInstance();
    printf("✓ Vortex API instance obtained\n");
    
    // Ensure all systems are initialized
    printf("\nInitializing Vortex API...\n");
    if (vortex.EnsureInitialized()) {
        printf("✓ Vortex API initialization successful!\n");
        
        // Get system diagnostics
        VortexSystemDiagnostics diagnostics;
        if (vortex.GetSystemDiagnostics(diagnostics)) {
            PrintSystemDiagnostics(diagnostics);
        }
        
        // Demonstrate each component
        DemonstrateComms(vortex);
        DemonstrateGpio(vortex);
        DemonstrateMotors(vortex);
        DemonstrateAdc(vortex);
        DemonstrateImu(vortex);
        DemonstrateEncoders(vortex);
        DemonstrateLeds(vortex);
        DemonstrateTemperature(vortex);
        
        // Perform health check
        printf("=== System Health Check ===\n");
        if (vortex.PerformHealthCheck()) {
            printf("✓ System health check passed\n");
        } else {
            printf("✗ System health check failed\n");
        }
        printf("=== End Health Check ===\n\n");
        
        // Dump comprehensive statistics
        printf("Dumping system statistics...\n");
        vortex.DumpSystemStatistics();
        
        // Show system version
        printf("System Version: %s\n", vortex.GetSystemVersion());
        
        // Continuous operation demo
        printf("\n=== Continuous Operation Demo ===\n");
        printf("Running for 10 seconds to demonstrate continuous operation...\n");
        
        for (int i = 0; i < 10; i++) {
            printf("Tick %d/10 - System Uptime: %llu ms\n", i + 1, vortex.GetSystemUptimeMs());
            
            // Blink LED to show activity
            if (i % 2 == 0) {
                vortex.leds.SetStatus(LedAnimation::STATUS_OK);
            } else {
                vortex.leds.SetStatus(LedAnimation::STATUS_WARN);
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        // Final LED status
        vortex.leds.SetStatus(LedAnimation::STATUS_OK);
        
        printf("=== Demo Complete ===\n");
        printf("The Vortex API provides unified access to all HardFOC components!\n");
        
    } else {
        printf("✗ Vortex API initialization failed!\n");
        
        // Show what failed
        const char* failed_names[8];
        size_t failed_count = vortex.GetFailedComponents(failed_names, 8);
        if (failed_count > 0) {
            printf("Failed components:\n");
            for (size_t i = 0; i < failed_count; ++i) {
                printf("  - %s\n", failed_names[i]);
            }
        }
        
        const char* warning_strs[8];
        size_t warn_count = vortex.GetSystemWarnings(warning_strs, 8);
        if (warn_count > 0) {
            printf("System warnings:\n");
            for (size_t i = 0; i < warn_count; ++i) {
                printf("  - %s\n", warning_strs[i]);
            }
        }
    }
    
    printf("\nVortex API Example completed.\n");
    printf("Check the serial output for detailed information.\n");
} 