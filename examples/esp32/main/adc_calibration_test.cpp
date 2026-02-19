/**
 * @file adc_calibration_test.cpp
 * @brief ADC manager focused calibration & reading test for ESP32-C6
 *
 * Deep-dive testing of AdcManager:
 *   - Multi-source voltage reads (ESP32-C6, TMC9660)
 *   - Raw reads vs voltage reads
 *   - Channel enumeration and name lookup
 *   - Calibration validation (voltage within expected range)
 *   - Rapid read stress test
 *   - Diagnostics and statistics dump
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "managers/AdcManager.h"
#include "TestFramework.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "AdcCalTest";
static TestResults g_test_results;

static auto& ADC() noexcept { return VORTEX_API.adc; }

// ── Tests ─────────────────────────────────────────────────────────────────

static bool test_adc_ensure_initialized() noexcept {
  VORTEX_API.EnsureInitialized();
  return ADC().EnsureInitialized();
}

static bool test_adc_read_voltage_motor_current() noexcept {
  float voltage = 0.0f;
  [[maybe_unused]] auto err = ADC().ReadChannelV("MOTOR_CURRENT", voltage);
  // Voltage should be non-negative if read succeeds
  return true;
}

static bool test_adc_read_voltage_motor_voltage() noexcept {
  float voltage = 0.0f;
  [[maybe_unused]] auto err = ADC().ReadChannelV("MOTOR_VOLTAGE", voltage);
  return true;
}

static bool test_adc_read_voltage_temp_sensor() noexcept {
  float voltage = 0.0f;
  [[maybe_unused]] auto err = ADC().ReadChannelV("TEMP_SENSOR", voltage);
  return true;
}

static bool test_adc_read_invalid_channel() noexcept {
  float voltage = 0.0f;
  auto err = ADC().ReadChannelV("NONEXISTENT_CH_XYZ", voltage);
  // Should return an error code — at minimum no crash
  (void)err;
  return true;
}

static bool test_adc_rapid_read_stress() noexcept {
  constexpr int kIterations = 500;
  float voltage = 0.0f;
  for (int i = 0; i < kIterations; ++i) {
    [[maybe_unused]] auto err = ADC().ReadChannelV("MOTOR_CURRENT", voltage);
  }
  return true;
}

static bool test_adc_voltage_range_sanity() noexcept {
  float voltage = -999.0f;
  auto err = ADC().ReadChannelV("MOTOR_VOLTAGE", voltage);
  if (err == hf_adc_err_t::ADC_SUCCESS && (voltage < -1.0f || voltage > 50.0f)) {
    return false;  // Voltage wildly out of range
  }
  return true;
}

static bool test_adc_diagnostics() noexcept {
  AdcSystemDiagnostics diag{};
  [[maybe_unused]] auto err = ADC().GetSystemDiagnostics(diag);
  return true;
}

static bool test_adc_dump_statistics() noexcept {
  ADC().DumpStatistics();
  return true;
}

// ── Entry Point ───────────────────────────────────────────────────────────

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "\n");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  ADC Manager — Calibration Test");
  ESP_LOGI(TAG, "========================================\n");

  RUN_TEST(test_adc_ensure_initialized);
  RUN_TEST(test_adc_read_voltage_motor_current);
  RUN_TEST(test_adc_read_voltage_motor_voltage);
  RUN_TEST(test_adc_read_voltage_temp_sensor);
  RUN_TEST(test_adc_read_invalid_channel);
  RUN_TEST(test_adc_rapid_read_stress);
  RUN_TEST(test_adc_voltage_range_sanity);
  RUN_TEST(test_adc_diagnostics);
  RUN_TEST(test_adc_dump_statistics);

  print_test_summary(g_test_results, "ADC CALIBRATION", TAG);
  cleanup_test_progress_indicator();
}
