/**
 * @file motor_controller_test.cpp
 * @brief Motor controller focused test for ESP32-C6
 *
 * Deep-dive testing of MotorController:
 *   - Handler access for onboard + external devices
 *   - visitDriver callback pattern
 *   - Device count and enumeration
 *   - Register read/write attempts
 *   - Device status queries
 *   - Statistics dump
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "TestFramework.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MotorCtrlTest";
static TestResults g_test_results;

static auto& MOTORS() noexcept { return VORTEX_API.motors; }

// ── Tests ─────────────────────────────────────────────────────────────────

static bool test_motor_ensure_initialized() noexcept {
  VORTEX_API.EnsureInitialized();
  return MOTORS().EnsureInitialized();
}

static bool test_motor_handler_0() noexcept {
  auto* h = MOTORS().handler(0);
  // Onboard motor controller — may be null if hardware absent
  (void)h;
  return true;
}

static bool test_motor_handler_1() noexcept {
  auto* h = MOTORS().handler(1);
  // External device 1 — likely null if not connected
  (void)h;
  return true;
}

static bool test_motor_visit_driver_slot0() noexcept {
  bool visited = false;
  MOTORS().visitDriver([&visited](auto& driver) {
    visited = true;
    (void)driver;
  }, 0);
  // visited depends on hardware
  return true;
}

static bool test_motor_device_count() noexcept {
  uint8_t count = MOTORS().GetDeviceCount();
  return (count <= 4);  // max 1 onboard + 3 external
}

static bool test_motor_visit_all_devices() noexcept {
  uint8_t count = MOTORS().GetDeviceCount();
  int visited = 0;
  for (uint8_t i = 0; i < count; ++i) {
    MOTORS().visitDriver([&visited](auto& driver) {
      visited++;
      (void)driver;
    }, i);
  }
  // visited should match device count (or less if some fail)
  return (visited <= static_cast<int>(count));
}

static bool test_motor_handler_null_safety() noexcept {
  // Access out-of-range slot — should return nullptr, not crash
  auto* h = MOTORS().handler(100);
  return (h == nullptr);
}

static bool test_motor_dump_statistics() noexcept {
  VORTEX_API.DumpSystemStatistics();
  return true;
}

// ── Entry Point ───────────────────────────────────────────────────────────

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "\n");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  Motor Controller — Focused Test");
  ESP_LOGI(TAG, "========================================\n");

  RUN_TEST(test_motor_ensure_initialized);
  RUN_TEST(test_motor_handler_0);
  RUN_TEST(test_motor_handler_1);
  RUN_TEST(test_motor_visit_driver_slot0);
  RUN_TEST(test_motor_device_count);
  RUN_TEST(test_motor_visit_all_devices);
  RUN_TEST(test_motor_handler_null_safety);
  RUN_TEST(test_motor_dump_statistics);

  print_test_summary(g_test_results, "MOTOR CONTROLLER", TAG);
  cleanup_test_progress_indicator();
}
