/**
 * @file LedManagerExample.cpp
 * @brief Documentation example: WS2812 addressable LED control on Vortex V1.
 *
 * Demonstrates the LedManager API:
 *   - SetColor (LedColor, RGB uint32_t)
 *   - Brightness control (percent, raw)
 *   - Built-in animations (RAINBOW, BLINK, BREATH, SOLID)
 *   - Status indication (Boot, Ready, Warning, Error)
 *   - Diagnostics & shutdown
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "managers/LedManager.h"

#include "esp_log.h"
#include "OsUtility.h"
static const char* TAG = "VortexLedExample";

static void example_solid_colours() {
    ESP_LOGI(TAG, "=== Solid Colours ===");
    auto& leds = VORTEX_API.leds;

    // Red
    leds.SetColor(LedColors::RED);
    ESP_LOGI(TAG, "Red");
    os_delay_msec(500);

    // Green
    leds.SetColor(LedColors::GREEN);
    ESP_LOGI(TAG, "Green");
    os_delay_msec(500);

    // Blue
    leds.SetColor(LedColors::BLUE);
    ESP_LOGI(TAG, "Blue");
    os_delay_msec(500);

    // Custom RGB value
    leds.SetColor(LedColor(255, 165, 0));  // Orange
    ESP_LOGI(TAG, "Orange");
    os_delay_msec(500);
}

static void example_brightness() {
    ESP_LOGI(TAG, "=== Brightness Ramp ===");
    auto& leds = VORTEX_API.leds;

    leds.SetColor(LedColors::WHITE);
    for (uint8_t pct = 10; pct <= 100; pct += 10) {
        leds.SetBrightnessPercent(pct);
        ESP_LOGI(TAG, "Brightness: %u%%", pct);
        os_delay_msec(100);
    }
    leds.SetBrightnessPercent(50);
}

static void example_animations() {
    ESP_LOGI(TAG, "=== Built-in Animations ===");
    auto& leds = VORTEX_API.leds;

    leds.StartAnimation(LedAnimation::RAINBOW);
    ESP_LOGI(TAG, "Rainbow (2 s)");
    for (int i = 0; i < 20; ++i) {
        leds.UpdateAnimation();
        os_delay_msec(100);
    }

    leds.StartAnimation(LedAnimation::BREATH, LedColors::CYAN);
    ESP_LOGI(TAG, "Breath cyan (2 s)");
    for (int i = 0; i < 20; ++i) {
        leds.UpdateAnimation();
        os_delay_msec(100);
    }

    leds.StopAnimation();
    leds.TurnOff();
}

static void example_status_indication() {
    ESP_LOGI(TAG, "=== Status Indication ===");
    auto& leds = VORTEX_API.leds;

    leds.IndicateBoot();
    leds.UpdateAnimation();
    ESP_LOGI(TAG, "BOOT");
    os_delay_msec(1000);

    leds.IndicateReady();
    leds.UpdateAnimation();
    ESP_LOGI(TAG, "READY");
    os_delay_msec(1000);

    leds.IndicateWarning();
    leds.UpdateAnimation();
    ESP_LOGI(TAG, "WARNING");
    os_delay_msec(1000);

    leds.IndicateError();
    leds.UpdateAnimation();
    ESP_LOGI(TAG, "ERROR");
    os_delay_msec(1000);

    leds.StopAnimation();
}

static void example_diagnostics() {
    ESP_LOGI(TAG, "=== LED Diagnostics ===");
    auto& leds = VORTEX_API.leds;

    LedSystemDiagnostics diag{};
    if (leds.GetSystemDiagnostics(diag) == LedError::SUCCESS) {
        ESP_LOGI(TAG, "Healthy: %s, ops: %u/%u/%u (total/ok/fail), anim: %s",
                 diag.system_healthy ? "YES" : "NO",
                 diag.total_operations, diag.successful_operations, diag.failed_operations,
                 diag.animation_active ? "YES" : "NO");
    }
    leds.DumpStatistics();
    ESP_LOGI(TAG, "Last error: %d", static_cast<int>(leds.GetLastError()));
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Vortex LedManager Example");
    Vortex::GetInstance().EnsureInitialized();

    example_solid_colours();
    example_brightness();
    example_animations();
    example_status_indication();
    example_diagnostics();

    VORTEX_API.leds.Shutdown();
    ESP_LOGI(TAG, "LedManager example complete");
}
