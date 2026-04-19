/**
 * @file TemperatureManagerExample.cpp
 * @brief Documentation example: Temperature monitoring on Vortex V1.
 *
 * Demonstrates the TemperatureManager API:
 *   - ESP32 internal sensor (Celsius / Fahrenheit)
 *   - NTC thermistor reading
 *   - Motor-winding temperature
 *   - Diagnostics & last-error
 *
 * Note: Vortex uses `float&` reference parameters, not pointers.
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "managers/TemperatureManager.h"

#include "esp_log.h"
#include "OsUtility.h"
static const char* TAG = "VortexTempExample";

static void example_internal_sensor() {
    ESP_LOGI(TAG, "=== Internal Sensor ===");
    auto& temp = VORTEX_API.temp;

    float celsius = 0.0f;
    auto err = temp.ReadTemperatureCelsius("ESP32_INTERNAL", celsius);
    if (err == hf_temp_err_t::TEMP_SUCCESS) {
        ESP_LOGI(TAG, "Internal: %.2f C", celsius);
    } else {
        ESP_LOGW(TAG, "Internal read failed: %d", static_cast<int>(err));
    }

    float fahr = 0.0f;
    err = temp.ReadTemperatureFahrenheit("ESP32_INTERNAL", fahr);
    if (err == hf_temp_err_t::TEMP_SUCCESS) {
        ESP_LOGI(TAG, "Internal: %.2f F", fahr);
    }
}

static void example_ntc_sensor() {
    ESP_LOGI(TAG, "=== NTC Thermistor ===");
    auto& temp = VORTEX_API.temp;

    float ntc_c = 0.0f;
    auto err = temp.ReadTemperatureCelsius("NTC_THERMISTOR", ntc_c);
    if (err == hf_temp_err_t::TEMP_SUCCESS) {
        ESP_LOGI(TAG, "NTC: %.2f C", ntc_c);
    } else {
        ESP_LOGW(TAG, "NTC read failed: %d", static_cast<int>(err));
    }
}

static void example_motor_temperature() {
    ESP_LOGI(TAG, "=== Motor Temperature ===");
    auto& temp = VORTEX_API.temp;

    float motor_c = 0.0f;
    auto err = temp.ReadTemperatureCelsius("MOTOR_TEMP", motor_c);
    if (err == hf_temp_err_t::TEMP_SUCCESS) {
        ESP_LOGI(TAG, "Motor winding: %.2f C", motor_c);
    } else {
        ESP_LOGW(TAG, "Motor temp read failed: %d", static_cast<int>(err));
    }
}

static void example_diagnostics() {
    ESP_LOGI(TAG, "=== Temperature Diagnostics ===");
    auto& temp = VORTEX_API.temp;

    TempSystemDiagnostics diag{};
    auto err = temp.GetSystemDiagnostics(diag);
    if (err == hf_temp_err_t::TEMP_SUCCESS) {
        ESP_LOGI(TAG, "Healthy: %s, Ops: %u/%u (ok/fail)",
                 diag.system_healthy ? "YES" : "NO",
                 diag.successful_operations, diag.failed_operations);
    }
    ESP_LOGI(TAG, "Last error: %d", static_cast<int>(temp.GetLastError()));
    temp.DumpStatistics();
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Vortex TemperatureManager Example");
    Vortex::GetInstance().EnsureInitialized();

    example_internal_sensor();
    os_delay_msec(100);
    example_ntc_sensor();
    os_delay_msec(100);
    example_motor_temperature();
    os_delay_msec(100);
    example_diagnostics();

    ESP_LOGI(TAG, "TemperatureManager example complete");
}
