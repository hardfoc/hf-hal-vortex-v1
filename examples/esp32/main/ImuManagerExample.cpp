/**
 * @file ImuManagerExample.cpp
 * @brief Documentation example: BNO08x IMU management on Vortex V1.
 *
 * Demonstrates the ImuManager API:
 *   - Accessing the BNO08x driver via GetSensor()
 *   - Device discovery (fixed-size array of names)
 *   - Initialization status
 *   - Diagnostics & shutdown
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "managers/ImuManager.h"

#include "esp_log.h"
#include <array>
#include "OsUtility.h"

static const char* TAG = "VortexImuExample";

static void example_discovery() {
    ESP_LOGI(TAG, "=== IMU Discovery ===");
    auto& imu = VORTEX_API.imu;

    ESP_LOGI(TAG, "Device count: %u", imu.GetDeviceCount());

    std::array<const char*, ImuManager::MAX_IMU_DEVICES> names{};
    size_t count = 0;
    imu.GetAvailableDevices(names, count);
    for (size_t i = 0; i < count; ++i) {
        ESP_LOGI(TAG, "  [%zu] %s  type: %s", i, names[i],
                 imu.GetDeviceType(static_cast<uint8_t>(i)));
    }
}

static void example_sensor_access() {
    ESP_LOGI(TAG, "=== Sensor Access ===");
    auto& imu = VORTEX_API.imu;

    if (imu.GetDeviceCount() == 0) {
        ESP_LOGW(TAG, "No IMU devices");
        return;
    }

    // Get driver interface for onboard IMU (index 0)
    auto* sensor = imu.GetSensor(0);
    if (sensor) {
        ESP_LOGI(TAG, "  Onboard BNO08x sensor available");
        // Use sensor->Update(), sensor->HasNewData(), sensor->GetLatest(), etc.
        // See IBno08xDriverOps / Bno08xHandler.h for full API
    } else {
        ESP_LOGW(TAG, "  Onboard sensor not ready");
    }
}

static void example_diagnostics() {
    ESP_LOGI(TAG, "=== IMU Diagnostics ===");
    auto& imu = VORTEX_API.imu;

    std::array<bool, ImuManager::MAX_IMU_DEVICES> status{};
    imu.GetInitializationStatus(status);
    for (size_t i = 0; i < ImuManager::MAX_IMU_DEVICES; ++i) {
        ESP_LOGI(TAG, "  [%zu] init: %s", i, status[i] ? "YES" : "NO");
    }
    imu.DumpStatistics();
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Vortex ImuManager Example");
    Vortex::GetInstance().EnsureInitialized();

    example_discovery();
    os_delay_msec(100);
    example_sensor_access();
    os_delay_msec(100);
    example_diagnostics();

    VORTEX_API.imu.Deinitialize();
    ESP_LOGI(TAG, "ImuManager example complete");
}
