/**
 * @file VortexApiExample.cpp
 * @brief Comprehensive documentation example: Full Vortex API on Vortex V1.
 *
 * Demonstrates the complete Vortex singleton API, touching all 8 managers
 * in a single example that shows a realistic system startup, operation,
 * and monitoring sequence.
 *
 * Coverage:
 *   1. System initialization and diagnostics
 *   2. CommChannelsManager — SPI/I2C/UART device validation
 *   3. GpioManager — multi-chip GPIO queries (ESP32, PCAL95555, TMC9660)
 *   4. MotorController — TMC9660 motor device management
 *   5. AdcManager — TMC9660-hosted ADC reads (15 channels)
 *   6. ImuManager — BNO08x IMU reads
 *   7. EncoderManager — AS5047U encoder reads
 *   8. LedManager — status indication tied to system state
 *   9. TemperatureManager — thermal monitoring (ESP32, NTC, motor)
 *  10. Health monitoring loop — periodic diagnostics + LED status
 *  11. Graceful shutdown
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"

// Full manager headers
#include "managers/CommChannelsManager.h"
#include "managers/GpioManager.h"
#include "managers/MotorController.h"
#include "managers/AdcManager.h"
#include "managers/ImuManager.h"
#include "managers/EncoderManager.h"
#include "managers/LedManager.h"
#include "managers/TemperatureManager.h"

#include "esp_log.h"
#include "OsUtility.h"
static const char* TAG = "VortexApiExample";

//=============================================================================
// Phase 1: System Boot
//=============================================================================

static bool phase_boot() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PHASE 1: SYSTEM BOOT");
    ESP_LOGI(TAG, "========================================");

    auto& vortex = Vortex::GetInstance();

    bool ok = vortex.EnsureInitialized();
    ESP_LOGI(TAG, "EnsureInitialized: %s", ok ? "SUCCESS" : "PARTIAL/FAILED");
    ESP_LOGI(TAG, "IsInitialized:     %s", vortex.IsInitialized() ? "YES" : "NO");
    ESP_LOGI(TAG, "System version:    %s", vortex.GetSystemVersion());
    ESP_LOGI(TAG, "Init time:         %llu ms",
             static_cast<unsigned long long>(vortex.GetInitializationTimeMs()));

    auto status = vortex.GetComponentInitializationStatus();
    const char* names[] = {"Comms", "GPIO", "Motors", "ADC", "IMU", "Encoders", "LEDs", "Temp"};
    for (size_t i = 0; i < status.size(); ++i) {
        ESP_LOGI(TAG, "  %-10s: %s", names[i], status[i] ? "OK" : "FAILED");
    }

    VortexSystemDiagnostics diag{};
    if (vortex.GetSystemDiagnostics(diag)) {
        ESP_LOGI(TAG, "Diagnostics: %u/%u components OK",
                 diag.initialized_components, diag.total_components);
    }

    return ok;
}

//=============================================================================
// Phase 2: Motor Controller Check
//=============================================================================

static void phase_motor_check() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PHASE 2: MOTOR CONTROLLER");
    ESP_LOGI(TAG, "========================================");

    auto& motors = VORTEX_API.motors;

    ESP_LOGI(TAG, "Device count: %u", motors.GetDeviceCount());

    std::array<uint8_t, MotorController::MAX_TMC9660_DEVICES> indices{};
    size_t count = 0;
    motors.GetActiveDeviceIndices(indices, count);
    for (size_t i = 0; i < count; ++i) {
        ESP_LOGI(TAG, "  Active device index: %u", indices[i]);
    }

    MotorSystemDiagnostics mdiag{};
    if (motors.GetSystemDiagnostics(mdiag) == MotorError::SUCCESS) {
        ESP_LOGI(TAG, "Motor system healthy: %s", mdiag.system_healthy ? "YES" : "NO");
        ESP_LOGI(TAG, "Active devices: %u, Initialized: %u",
                 mdiag.active_device_count, mdiag.initialized_device_count);
    }
}

//=============================================================================
// Phase 3: ADC Monitoring
//=============================================================================

static void phase_adc_monitoring() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PHASE 3: ADC MONITORING");
    ESP_LOGI(TAG, "========================================");

    auto& adc = VORTEX_API.adc;

    ESP_LOGI(TAG, "ADC channels: %zu", adc.Size());

    float voltage = 0.0f;
    hf_adc_err_t err = adc.ReadVoltage("ADC_VM", voltage, 8);
    if (err == hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGI(TAG, "VM voltage: %.2f V", static_cast<double>(voltage));
    }

    AdcSystemDiagnostics adiag{};
    if (adc.GetSystemDiagnostics(adiag) == hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGI(TAG, "ADC healthy: %s, channels: %u",
                 adiag.system_healthy ? "YES" : "NO",
                 adiag.total_channels_registered);
    }
}

//=============================================================================
// Phase 4: IMU and Encoder
//=============================================================================

static void phase_imu_encoder() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PHASE 4: IMU & ENCODER");
    ESP_LOGI(TAG, "========================================");

    auto& imu = VORTEX_API.imu;
    auto& enc = VORTEX_API.encoders;

    ESP_LOGI(TAG, "IMU device count: %u", imu.GetDeviceCount());
    ESP_LOGI(TAG, "Encoder device count: %u", enc.GetDeviceCount());

    // Read single encoder angle in degrees
    double angle_deg = 0.0;
    if (enc.ReadAngleDegrees(0, angle_deg) == EncoderError::SUCCESS) {
        ESP_LOGI(TAG, "  Encoder[0] angle: %.3f deg", angle_deg);
    }

    // Read encoder velocity
    double vel_rpm = 0.0;
    if (enc.ReadVelocityRPM(0, vel_rpm) == EncoderError::SUCCESS) {
        ESP_LOGI(TAG, "  Encoder[0] velocity: %.3f RPM", vel_rpm);
    }

    // IMU — access through driver interface
    auto* sensor = imu.GetSensor(0);
    if (sensor) {
        ESP_LOGI(TAG, "  IMU[0] sensor available");
    } else {
        ESP_LOGW(TAG, "  IMU[0] sensor not available");
    }
}

//=============================================================================
// Phase 5: Temperature Monitoring
//=============================================================================

static void phase_temperature() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PHASE 5: TEMPERATURE");
    ESP_LOGI(TAG, "========================================");

    auto& temp = VORTEX_API.temp;

    const char* sensors[] = {"ESP32_INTERNAL", "NTC_THERMISTOR", "MOTOR_TEMP"};
    for (const char* name : sensors) {
        float celsius = 0.0f;
        auto err = temp.ReadTemperatureCelsius(name, celsius);
        if (err == hf_temp_err_t::TEMP_SUCCESS) {
            ESP_LOGI(TAG, "  %s: %.1f C", name, static_cast<double>(celsius));
        } else {
            ESP_LOGW(TAG, "  %s: read failed", name);
        }
    }
}

//=============================================================================
// Phase 6: Health Monitoring
//=============================================================================

static void phase_health_monitoring() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PHASE 6: HEALTH MONITORING (3 cycles)");
    ESP_LOGI(TAG, "========================================");

    auto& vortex = VORTEX_API;

    for (int cycle = 0; cycle < 3; ++cycle) {
        ESP_LOGI(TAG, "--- Monitor Cycle %d ---", cycle);

        ManagerHealthSnapshot snapshot{};
        if (vortex.CollectManagerHealth(snapshot)) {
            ESP_LOGI(TAG, "  Health: %zu managers, %zu errors, all_healthy=%s",
                     snapshot.count, snapshot.managers_with_errors,
                     snapshot.all_healthy ? "YES" : "NO");
            for (size_t i = 0; i < snapshot.count; ++i) {
                const auto& e = snapshot.entries[i];
                if (e.has_error) {
                    ESP_LOGW(TAG, "    %s: ERROR (code %u)", e.name, e.last_error_code);
                }
            }
        }

        bool healthy = vortex.PerformHealthCheck();
        ESP_LOGI(TAG, "  PerformHealthCheck: %s", healthy ? "PASS" : "FAIL");

        os_delay_msec(1000);
    }
}

//=============================================================================
// Phase 7: Shutdown
//=============================================================================

static void phase_shutdown() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PHASE 7: SHUTDOWN");
    ESP_LOGI(TAG, "========================================");

    auto& vortex = VORTEX_API;

    vortex.leds.StopAnimation();
    vortex.leds.TurnOff();

    ESP_LOGI(TAG, "Uptime: %llu ms",
             static_cast<unsigned long long>(vortex.GetSystemUptimeMs()));

    vortex.DumpSystemStatistics();

    vortex.Shutdown();
    ESP_LOGI(TAG, "Shutdown complete");
}

//=============================================================================
// app_main
//=============================================================================

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Vortex API Comprehensive Example");

    bool ok = phase_boot();
    if (!ok) ESP_LOGW(TAG, "Boot with warnings — continuing");
    os_delay_msec(200);

    VORTEX_API.leds.IndicateBoot();
    VORTEX_API.leds.UpdateAnimation();

    phase_motor_check();
    os_delay_msec(200);

    phase_adc_monitoring();
    os_delay_msec(200);

    phase_imu_encoder();
    os_delay_msec(200);

    phase_temperature();
    os_delay_msec(200);

    VORTEX_API.leds.IndicateReady();
    VORTEX_API.leds.UpdateAnimation();

    phase_health_monitoring();
    os_delay_msec(200);

    phase_shutdown();

    ESP_LOGI(TAG, "Vortex API comprehensive example complete");
}
