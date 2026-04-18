/**
 * @file AdcManagerExample.cpp
 * @brief Documentation example: ADC reading on Vortex V1.
 *
 * Demonstrates the AdcManager API for the Vortex motor-control board:
 *   - Channel lookup by name and by functional enum
 *   - Voltage reading (auto-scaled through voltage divider)
 *   - Raw ADC count reading
 *   - Multi-sample averaging for noise reduction
 *   - System diagnostics
 *
 * Vortex V1 ADC channels (all TMC9660 hosted):
 *   - ADC_VM, ADC_PHASE_A, ADC_PHASE_B, ADC_PHASE_C, ...
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "managers/AdcManager.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "VortexAdcExample";

static void example_channel_registry() {
    ESP_LOGI(TAG, "=== Channel Registry ===");
    auto& adc = VORTEX_API.adc;
    ESP_LOGI(TAG, "Registered channels: %zu", adc.Size());
    ESP_LOGI(TAG, "  ADC_VM: %s", adc.Contains("ADC_VM") ? "found" : "NOT found");
}

static void example_voltage_reading() {
    ESP_LOGI(TAG, "=== Voltage Reading ===");
    auto& adc = VORTEX_API.adc;

    float vm = 0.0f;
    auto err = adc.ReadChannelV("ADC_VM", vm, 8);
    if (err == hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGI(TAG, "VM supply: %.2f V (8-sample avg)", static_cast<double>(vm));
    } else {
        ESP_LOGE(TAG, "VM read failed: %d", static_cast<int>(err));
    }
}

static void example_raw_reading() {
    ESP_LOGI(TAG, "=== Raw Count Reading ===");
    auto& adc = VORTEX_API.adc;

    uint32_t raw = 0;
    auto err = adc.ReadRaw("ADC_VM", raw, 4);
    if (err == hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGI(TAG, "VM raw count: %lu", static_cast<unsigned long>(raw));
    }
}

static void example_diagnostics() {
    ESP_LOGI(TAG, "=== ADC Diagnostics ===");
    auto& adc = VORTEX_API.adc;

    AdcSystemDiagnostics diag{};
    if (adc.GetSystemDiagnostics(diag) == hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGI(TAG, "Healthy: %s, Channels: %u, Ops: %lu",
                 diag.system_healthy ? "YES" : "NO",
                 diag.total_channels_registered,
                 static_cast<unsigned long>(diag.total_operations));
    }
    adc.DumpStatistics();
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Vortex AdcManager Example");
    Vortex::GetInstance().EnsureInitialized();

    example_channel_registry();
    vTaskDelay(pdMS_TO_TICKS(100));
    example_voltage_reading();
    vTaskDelay(pdMS_TO_TICKS(100));
    example_raw_reading();
    vTaskDelay(pdMS_TO_TICKS(100));
    example_diagnostics();

    ESP_LOGI(TAG, "AdcManager example complete");
}
