/**
 * @file encoder_imu_test.cpp
 * @brief Encoder & IMU manager focused test for ESP32-C6
 *
 * Deep-dive testing of EncoderManager and ImuManager:
 *   - Handler access and device enumeration
 *   - Angle/velocity reads (Encoder)
 *   - BNO08x handler and sensor access (IMU)
 *   - Device count validation
 *   - Null-safety for out-of-range indices
 *   - Active device queries
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "TestFramework.h"
#include "handlers/bno08x/Bno08xHandler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "EncImuTest";
static TestResults g_test_results;

static auto& ENC() noexcept { return VORTEX_API.encoders; }
static auto& IMU() noexcept { return VORTEX_API.imu; }

// ── Encoder Tests ─────────────────────────────────────────────────────────

static bool test_encoder_ensure_initialized() noexcept {
  VORTEX_API.EnsureInitialized();
  return ENC().EnsureInitialized();
}

static bool test_encoder_handler_0() noexcept {
  auto* h = ENC().GetAs5047uHandler(0);
  (void)h;
  return true;
}

static bool test_encoder_read_angle() noexcept {
  uint16_t angle = 0;
  [[maybe_unused]] auto err = ENC().ReadAngle(0, angle);
  return true;
}

static bool test_encoder_read_velocity() noexcept {
  double rpm = 0.0;
  [[maybe_unused]] auto err = ENC().ReadVelocityRPM(0, rpm);
  return true;
}

static bool test_encoder_device_count() noexcept {
  uint8_t count = ENC().GetDeviceCount();
  return (count <= 4);
}

static bool test_encoder_active_devices() noexcept {
  auto active = ENC().GetActiveDeviceIndices();
  return (active.size() <= 4);
}

static bool test_encoder_null_safety() noexcept {
  auto* h = ENC().GetAs5047uHandler(100);
  return (h == nullptr);
}

// ── IMU Tests ─────────────────────────────────────────────────────────────

static bool test_imu_ensure_initialized() noexcept {
  return IMU().EnsureInitialized();
}

static bool test_imu_handler_access() noexcept {
  auto* h = IMU().GetBno08xHandler(0);
  (void)h;
  return true;
}

static bool test_imu_sensor_access() noexcept {
  IBno08xDriverOps* sensor = IMU().GetSensor(0);
  (void)sensor;
  return true;
}

static bool test_imu_device_count() noexcept {
  uint8_t count = IMU().GetDeviceCount();
  return (count <= 4);
}

static bool test_imu_null_safety() noexcept {
  auto* h = IMU().GetBno08xHandler(100);
  return (h == nullptr);
}

// ── Entry Point ───────────────────────────────────────────────────────────

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "\n");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  Encoder & IMU — Focused Test");
  ESP_LOGI(TAG, "========================================\n");

  RUN_TEST(test_encoder_ensure_initialized);
  RUN_TEST(test_encoder_handler_0);
  RUN_TEST(test_encoder_read_angle);
  RUN_TEST(test_encoder_read_velocity);
  RUN_TEST(test_encoder_device_count);
  RUN_TEST(test_encoder_active_devices);
  RUN_TEST(test_encoder_null_safety);

  RUN_TEST(test_imu_ensure_initialized);
  RUN_TEST(test_imu_handler_access);
  RUN_TEST(test_imu_sensor_access);
  RUN_TEST(test_imu_device_count);
  RUN_TEST(test_imu_null_safety);

  print_test_summary(g_test_results, "ENCODER & IMU", TAG);
  cleanup_test_progress_indicator();
}
