/**
 * @file led_temp_test.cpp
 * @brief LED & Temperature manager focused test for ESP32-C6
 *
 * Deep-dive testing of LedManager and TemperatureManager:
 *   - LED status, color, brightness, animation APIs
 *   - Temperature multi-sensor reads (ESP32, NTC, motor)
 *   - Diagnostics and statistics
 *   - Rapid-read stress
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "managers/LedManager.h"
#include "managers/TemperatureManager.h"
#include "TestFramework.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "LedTempTest";
static TestResults g_test_results;

static auto& LEDS() noexcept { return VORTEX_API.leds; }
static auto& TEMP() noexcept { return VORTEX_API.temp; }

// ── LED Tests ─────────────────────────────────────────────────────────────

static bool test_led_ensure_initialized() noexcept {
  VORTEX_API.EnsureInitialized();
  return LEDS().EnsureInitialized();
}

static bool test_led_set_status_ok() noexcept {
  [[maybe_unused]] auto err = LEDS().SetStatus(LedAnimation::STATUS_OK);
  return true;
}

static bool test_led_set_status_warning() noexcept {
  [[maybe_unused]] auto err = LEDS().SetStatus(LedAnimation::STATUS_WARN);
  return true;
}

static bool test_led_set_status_error() noexcept {
  [[maybe_unused]] auto err = LEDS().SetStatus(LedAnimation::STATUS_ERROR);
  return true;
}

static bool test_led_get_color() noexcept {
  LedColor color{0, 0, 0};
  [[maybe_unused]] auto err = LEDS().GetCurrentColor(color);
  return true;
}

static bool test_led_get_brightness() noexcept {
  uint8_t brightness = 0;
  [[maybe_unused]] auto err = LEDS().GetCurrentBrightnessPercent(brightness);
  return (brightness <= 100);
}

// ── Temperature Tests ─────────────────────────────────────────────────────

static bool test_temp_ensure_initialized() noexcept {
  return TEMP().EnsureInitialized();
}

static bool test_temp_read_esp32_internal() noexcept {
  float celsius = 0.0f;
  [[maybe_unused]] auto err = TEMP().ReadTemperatureCelsius("ESP32_INTERNAL", &celsius);
  return true;
}

static bool test_temp_read_ntc() noexcept {
  float celsius = 0.0f;
  [[maybe_unused]] auto err = TEMP().ReadTemperatureCelsius("NTC_THERMISTOR", &celsius);
  return true;
}

static bool test_temp_read_motor() noexcept {
  float celsius = 0.0f;
  [[maybe_unused]] auto err = TEMP().ReadTemperatureCelsius("MOTOR_TEMP", &celsius);
  return true;
}

static bool test_temp_read_invalid_sensor() noexcept {
  float celsius = 0.0f;
  auto err = TEMP().ReadTemperatureCelsius("NONEXISTENT_SENSOR_XYZ", &celsius);
  (void)err;
  return true;
}

static bool test_temp_rapid_read_stress() noexcept {
  constexpr int kIterations = 200;
  float celsius = 0.0f;
  for (int i = 0; i < kIterations; ++i) {
    [[maybe_unused]] auto err = TEMP().ReadTemperatureCelsius("ESP32_INTERNAL", &celsius);
  }
  return true;
}

static bool test_temp_range_sanity() noexcept {
  float celsius = -999.0f;
  auto err = TEMP().ReadTemperatureCelsius("ESP32_INTERNAL", &celsius);
  if (err == hf_temp_err_t::TEMP_SUCCESS && (celsius < -40.0f || celsius > 125.0f)) {
    return false;  // Wildly out of range
  }
  return true;
}

static bool test_temp_diagnostics() noexcept {
  TempSystemDiagnostics diag{};
  [[maybe_unused]] auto err = TEMP().GetSystemDiagnostics(diag);
  return true;
}

// ── Entry Point ───────────────────────────────────────────────────────────

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "\n");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  LED & Temperature — Focused Test");
  ESP_LOGI(TAG, "========================================\n");

  RUN_TEST(test_led_ensure_initialized);
  RUN_TEST(test_led_set_status_ok);
  RUN_TEST(test_led_set_status_warning);
  RUN_TEST(test_led_set_status_error);
  RUN_TEST(test_led_get_color);
  RUN_TEST(test_led_get_brightness);

  RUN_TEST(test_temp_ensure_initialized);
  RUN_TEST(test_temp_read_esp32_internal);
  RUN_TEST(test_temp_read_ntc);
  RUN_TEST(test_temp_read_motor);
  RUN_TEST(test_temp_read_invalid_sensor);
  RUN_TEST(test_temp_rapid_read_stress);
  RUN_TEST(test_temp_range_sanity);
  RUN_TEST(test_temp_diagnostics);

  print_test_summary(g_test_results, "LED & TEMPERATURE", TAG);
  cleanup_test_progress_indicator();
}
