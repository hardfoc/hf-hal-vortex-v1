/**
 * @file EncoderManagerExample.cpp
 * @brief Documentation example: AS5047U encoder management on Vortex V1.
 *
 * Demonstrates the EncoderManager API:
 *   - Device discovery — fixed-size array of names
 *   - Single angle & velocity reads
 *   - Batch angle & velocity reads (fixed-size arrays)
 *   - Diagnostics & shutdown
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "managers/EncoderManager.h"

#include "esp_log.h"
#include <array>
#include "OsUtility.h"

static const char* TAG = "VortexEncoderExample";

static void example_discovery() {
    ESP_LOGI(TAG, "=== Encoder Discovery ===");
    auto& enc = VORTEX_API.encoders;

    ESP_LOGI(TAG, "Device count: %u", enc.GetDeviceCount());

    std::array<const char*, EncoderManager::MAX_ENCODER_DEVICES> names{};
    size_t count = 0;
    enc.GetAvailableDevices(names, count);
    for (size_t i = 0; i < count; ++i) {
        ESP_LOGI(TAG, "  [%zu] %s  type: %s", i, names[i],
                 enc.GetDeviceType(static_cast<uint8_t>(i)));
    }
}

static void example_single_read() {
    ESP_LOGI(TAG, "=== Single Device Read ===");
    auto& enc = VORTEX_API.encoders;

    if (enc.GetDeviceCount() == 0) {
        ESP_LOGW(TAG, "No encoders");
        return;
    }

    // Raw 14-bit angle
    uint16_t raw_angle = 0;
    if (enc.ReadAngle(0, raw_angle) == EncoderError::SUCCESS) {
        ESP_LOGI(TAG, "Encoder 0 raw angle: %u", raw_angle);
    }

    // Angle in degrees
    double angle_deg = 0.0;
    if (enc.ReadAngleDegrees(0, angle_deg) == EncoderError::SUCCESS) {
        ESP_LOGI(TAG, "Encoder 0 angle: %.3f deg", angle_deg);
    }

    // Velocity in RPM
    double velocity_rpm = 0.0;
    if (enc.ReadVelocityRPM(0, velocity_rpm) == EncoderError::SUCCESS) {
        ESP_LOGI(TAG, "Encoder 0 velocity: %.3f RPM", velocity_rpm);
    }
}

static void example_batch_read() {
    ESP_LOGI(TAG, "=== Batch Read ===");
    auto& enc = VORTEX_API.encoders;

    std::array<uint16_t, EncoderManager::MAX_ENCODER_DEVICES> angles{};
    std::array<uint8_t, EncoderManager::MAX_ENCODER_DEVICES> angle_devs{};
    std::array<EncoderError, EncoderManager::MAX_ENCODER_DEVICES> angle_errs{};
    size_t angle_count = 0;
    enc.ReadAllAngles(angles, angle_devs, angle_errs, angle_count);
    for (size_t i = 0; i < angle_count; ++i) {
        ESP_LOGI(TAG, "  Dev[%u] Angle: %u  (%s)", angle_devs[i], angles[i],
                 angle_errs[i] == EncoderError::SUCCESS ? "OK" : "ERR");
    }

    std::array<double, EncoderManager::MAX_ENCODER_DEVICES> velocities{};
    std::array<uint8_t, EncoderManager::MAX_ENCODER_DEVICES> vel_devs{};
    std::array<EncoderError, EncoderManager::MAX_ENCODER_DEVICES> vel_errs{};
    size_t vel_count = 0;
    enc.ReadAllVelocities(velocities, vel_devs, vel_errs, vel_count);
    for (size_t i = 0; i < vel_count; ++i) {
        ESP_LOGI(TAG, "  Dev[%u] Velocity: %.3f RPM  (%s)", vel_devs[i], velocities[i],
                 vel_errs[i] == EncoderError::SUCCESS ? "OK" : "ERR");
    }
}

static void example_init_batch() {
    ESP_LOGI(TAG, "=== Batch Init Status ===");
    auto& enc = VORTEX_API.encoders;

    std::array<bool, EncoderManager::MAX_ENCODER_DEVICES> status{};
    enc.GetInitializationStatus(status);
    for (size_t i = 0; i < EncoderManager::MAX_ENCODER_DEVICES; ++i) {
        ESP_LOGI(TAG, "  [%zu] init: %s", i, status[i] ? "YES" : "NO");
    }
}

static void example_diagnostics() {
    ESP_LOGI(TAG, "=== Encoder Diagnostics ===");
    auto& enc = VORTEX_API.encoders;
    enc.DumpStatistics();
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Vortex EncoderManager Example");
    Vortex::GetInstance().EnsureInitialized();

    example_discovery();
    os_delay_msec(100);
    example_single_read();
    os_delay_msec(100);
    example_batch_read();
    example_init_batch();
    example_diagnostics();

    VORTEX_API.encoders.Deinitialize();
    ESP_LOGI(TAG, "EncoderManager example complete");
}
