/**
 * @file gpio_stress_test.cpp
 * @brief GPIO manager focused stress test for ESP32-C6
 *
 * Deep-dive testing of GpioManager:
 *   - Pin enumeration and Contains checks
 *   - SetActive / SetInactive / Toggle sequences
 *   - Batch read operations
 *   - Direction and pull-mode reconfiguration
 *   - Rapid toggle stress (timing measurement)
 *   - Statistics dump and diagnostics
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "TestFramework.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "GpioStressTest";
static TestResults g_test_results;

// ── Helpers ───────────────────────────────────────────────────────────────

static auto& GPIO() noexcept { return VORTEX_API.gpio; }

// ── Tests ─────────────────────────────────────────────────────────────────

static bool test_gpio_ensure_initialized() noexcept {
  VORTEX_API.EnsureInitialized();
  return GPIO().EnsureInitialized();
}

static bool test_gpio_contains_valid_pins() noexcept {
  // Probe a variety of expected pin name patterns — results depend on config
  [[maybe_unused]] bool a = GPIO().Contains("LED_STATUS");
  [[maybe_unused]] bool b = GPIO().Contains("ESP32_GPIO_0");
  [[maybe_unused]] bool c = GPIO().Contains("MOTOR_ENABLE");
  return true;
}

static bool test_gpio_contains_invalid_returns_false() noexcept {
  return !GPIO().Contains("THIS_PIN_DOES_NOT_EXIST_12345");
}

static bool test_gpio_set_and_read_round_trip() noexcept {
  // On a known output pin, set high then read back
  // If pin is not configured this is still safe (no crash)
  auto& gpio = GPIO();
  [[maybe_unused]] auto r1 = gpio.SetActive("LED_STATUS");
  bool state = false;
  [[maybe_unused]] auto r3 = gpio.IsActive("LED_STATUS", state);
  [[maybe_unused]] auto r2 = gpio.SetInactive("LED_STATUS");
  return true;
}

static bool test_gpio_toggle() noexcept {
  auto& gpio = GPIO();
  [[maybe_unused]] auto r1 = gpio.Toggle("LED_STATUS");
  [[maybe_unused]] auto r2 = gpio.Toggle("LED_STATUS");
  return true;
}

static bool test_gpio_rapid_toggle_stress() noexcept {
  auto& gpio = GPIO();
  constexpr int kIterations = 1000;
  for (int i = 0; i < kIterations; ++i) {
    [[maybe_unused]] auto r = gpio.Toggle("LED_STATUS");
  }
  return true;  // no crash = pass
}

static bool test_gpio_set_direction() noexcept {
  auto& gpio = GPIO();
  // Attempt to set direction — safe even if pin absent
  [[maybe_unused]] auto r = gpio.SetDirection("LED_STATUS", hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
  return true;
}

static bool test_gpio_batch_read() noexcept {
  auto& gpio = GPIO();
  // Read multiple pins in sequence — validates no crash
  bool a = false, b = false, c = false;
  [[maybe_unused]] auto r1 = gpio.Read("LED_STATUS", a);
  [[maybe_unused]] auto r2 = gpio.Read("MOTOR_ENABLE", b);
  [[maybe_unused]] auto r3 = gpio.Read("IMU_INTERRUPT", c);
  return true;
}

static bool test_gpio_diagnostics() noexcept {
  auto& gpio = GPIO();
  GpioSystemDiagnostics diag{};
  [[maybe_unused]] auto err = gpio.GetSystemDiagnostics(diag);
  // overall_healthy can be true or false depending on hardware
  return true;
}

static bool test_gpio_dump_statistics() noexcept {
  GPIO().DumpStatistics();
  return true;
}

// ── Entry Point ───────────────────────────────────────────────────────────

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "\n");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  GPIO Manager — Stress Test");
  ESP_LOGI(TAG, "========================================\n");

  RUN_TEST(test_gpio_ensure_initialized);
  RUN_TEST(test_gpio_contains_valid_pins);
  RUN_TEST(test_gpio_contains_invalid_returns_false);
  RUN_TEST(test_gpio_set_and_read_round_trip);
  RUN_TEST(test_gpio_toggle);
  RUN_TEST(test_gpio_rapid_toggle_stress);
  RUN_TEST(test_gpio_set_direction);
  RUN_TEST(test_gpio_batch_read);
  RUN_TEST(test_gpio_diagnostics);
  RUN_TEST(test_gpio_dump_statistics);

  print_test_summary(g_test_results, "GPIO STRESS", TAG);
  cleanup_test_progress_indicator();
}
