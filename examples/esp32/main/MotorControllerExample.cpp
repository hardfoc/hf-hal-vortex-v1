/**
 * @file MotorControllerExample.cpp
 * @brief Documentation example: TMC9660 motor control via MotorController.
 *
 * Demonstrates the MotorController API on Vortex V1:
 *   - Device initialization (single + batch)
 *   - Active device queries (fixed-size array API)
 *   - Individual device statistics
 *   - System diagnostics & dump
 *   - Deinitialize / shutdown
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "managers/MotorController.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <array>

static const char* TAG = "VortexMotorExample";

static void example_init_devices() {
    ESP_LOGI(TAG, "=== Device Initialization ===");
    auto& mc = VORTEX_API.motors;

    // Batch init — returns per-device status in a fixed-size array
    std::array<bool, MotorController::MAX_TMC9660_DEVICES> results{};
    size_t count = mc.InitializeAllDevices(results);
    ESP_LOGI(TAG, "Initialized %zu devices:", count);
    for (size_t i = 0; i < count; ++i) {
        ESP_LOGI(TAG, "  [%zu] %s", i, results[i] ? "OK" : "FAIL");
    }
}

static void example_active_devices() {
    ESP_LOGI(TAG, "=== Active Devices ===");
    auto& mc = VORTEX_API.motors;

    std::array<uint8_t, MotorController::MAX_TMC9660_DEVICES> indices{};
    size_t count = 0;
    mc.GetActiveDeviceIndices(indices, count);
    ESP_LOGI(TAG, "Active count: %zu", count);
    for (size_t i = 0; i < count; ++i) {
        ESP_LOGI(TAG, "  Device index: %u", indices[i]);
    }
}

static void example_init_status() {
    ESP_LOGI(TAG, "=== Initialization Status ===");
    auto& mc = VORTEX_API.motors;

    std::array<bool, MotorController::MAX_TMC9660_DEVICES> status{};
    mc.GetInitializationStatus(status);
    for (size_t i = 0; i < MotorController::MAX_TMC9660_DEVICES; ++i) {
        ESP_LOGI(TAG, "  [%zu] initialized: %s", i, status[i] ? "YES" : "NO");
    }
}

static void example_system_diag() {
    ESP_LOGI(TAG, "=== System Diagnostics ===" );
    auto& mc = VORTEX_API.motors;

    MotorSystemDiagnostics diag{};
    if (mc.GetSystemDiagnostics(diag) == MotorError::SUCCESS) {
        ESP_LOGI(TAG, "Healthy: %s, active: %u, init: %u",
                 diag.system_healthy ? "YES" : "NO",
                 diag.active_device_count, diag.initialized_device_count);
    }
}

static void example_diagnostics() {
    ESP_LOGI(TAG, "=== Motor System Diagnostics ===");
    auto& mc = VORTEX_API.motors;
    mc.DumpStatistics();
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Vortex MotorController Example");
    Vortex::GetInstance().EnsureInitialized();

    example_init_devices();
    vTaskDelay(pdMS_TO_TICKS(100));
    example_active_devices();
    example_init_status();
    vTaskDelay(pdMS_TO_TICKS(100));
    example_system_diag();
    example_diagnostics();

    // Graceful shutdown
    VORTEX_API.motors.Deinitialize();
    ESP_LOGI(TAG, "MotorController example complete");
}
