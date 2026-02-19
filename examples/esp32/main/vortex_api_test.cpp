/**
 * @file vortex_api_test.cpp
 * @brief Comprehensive Vortex HAL API test suite for ESP32-C6 (noexcept)
 *
 * This file exercises every public surface of the Vortex singleton:
 *   - Full initialization sequence via EnsureInitialized()
 *   - System diagnostics & health-check validation
 *   - CommChannelsManager: SPI / I2C device enumeration
 *   - GpioManager: pin-name lookup (Contains)
 *   - MotorController: handler access, visitDriver, device count
 *   - AdcManager: multi-source voltage reads (ReadChannelV)
 *   - ImuManager: BNO08x handler & sensor access
 *   - EncoderManager: AS5047U angle & velocity reads
 *   - LedManager: animation, colour, brightness queries
 *   - TemperatureManager: multi-sensor reads (ESP32, NTC, motor)
 *   - Utility methods: uptime, version, statistics dump
 *
 * All functions are noexcept – no exception handling used.
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "TestFramework.h"

// Full manager definitions needed for tests (Vortex.h only forward-declares)
#include "managers/AdcManager.h"
#include "managers/CommChannelsManager.h"
#include "managers/EncoderManager.h"
#include "managers/GpioManager.h"
#include "managers/ImuManager.h"
#include "managers/LedManager.h"
#include "managers/TemperatureManager.h"
#include "managers/MotorController.h"

// Handlers needed for type visibility in tests
#include "handlers/bno08x/Bno08xHandler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "VortexApiTest";
static TestResults g_test_results;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
static constexpr bool ENABLE_INIT_TESTS        = true;
static constexpr bool ENABLE_DIAGNOSTICS_TESTS = true;
static constexpr bool ENABLE_COMMS_TESTS       = true;
static constexpr bool ENABLE_GPIO_TESTS        = true;
static constexpr bool ENABLE_MOTOR_TESTS       = true;
static constexpr bool ENABLE_ADC_TESTS         = true;
static constexpr bool ENABLE_IMU_TESTS         = true;
static constexpr bool ENABLE_ENCODER_TESTS     = true;
static constexpr bool ENABLE_LED_TESTS         = true;
static constexpr bool ENABLE_TEMPERATURE_TESTS = true;
static constexpr bool ENABLE_UTILITY_TESTS     = true;

//=============================================================================
// INITIALIZATION TESTS
//=============================================================================

static bool test_singleton_access() noexcept {
  auto& v1 = Vortex::GetInstance();
  auto& v2 = Vortex::GetInstance();
  return (&v1 == &v2);
}

static bool test_ensure_initialized() noexcept {
  auto& vortex = VORTEX_API;
  return vortex.EnsureInitialized();
}

static bool test_is_initialized_after_init() noexcept {
  auto& vortex = VORTEX_API;
  vortex.EnsureInitialized();
  return vortex.IsInitialized();
}

static bool test_idempotent_init() noexcept {
  auto& vortex = VORTEX_API;
  bool first  = vortex.EnsureInitialized();
  bool second = vortex.EnsureInitialized();
  return (first == second);
}

//=============================================================================
// DIAGNOSTICS TESTS
//=============================================================================

static bool test_get_system_diagnostics() noexcept {
  auto& vortex = VORTEX_API;
  vortex.EnsureInitialized();

  VortexSystemDiagnostics diag;
  bool ok = vortex.GetSystemDiagnostics(diag);
  if (!ok) return false;

  // total_components should be 8 for Vortex
  if (diag.total_components != 8) return false;

  // initialized_components must be <= total
  if (diag.initialized_components > diag.total_components) return false;

  return true;
}

static bool test_component_init_status() noexcept {
  auto& vortex = VORTEX_API;
  vortex.EnsureInitialized();

  auto status = vortex.GetComponentInitializationStatus();
  // Should return exactly 8 entries
  return (status.size() == 8);
}

static bool test_perform_health_check() noexcept {
  auto& vortex = VORTEX_API;
  vortex.EnsureInitialized();
  // Just verify no crash; result depends on hardware
  [[maybe_unused]] bool healthy = vortex.PerformHealthCheck();
  return true;
}

static bool test_failed_components_api() noexcept {
  auto& vortex = VORTEX_API;
  vortex.EnsureInitialized();

  const char* names[8]{};
  size_t count = vortex.GetFailedComponents(names, 8);
  // count should be <= 8
  return (count <= 8);
}

static bool test_system_warnings_api() noexcept {
  auto& vortex = VORTEX_API;
  vortex.EnsureInitialized();

  const char* warnings[8]{};
  size_t count = vortex.GetSystemWarnings(warnings, 8);
  return (count <= 8);
}

//=============================================================================
// COMMS TESTS
//=============================================================================

static bool test_comms_spi_device_lookup() noexcept {
  auto& comms = VORTEX_API.comms;
  auto* dev = comms.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
  // Just verify API call succeeds – null is OK if hardware absent
  (void)dev;
  return true;
}

static bool test_comms_i2c_device_lookup() noexcept {
  auto& comms = VORTEX_API.comms;
  auto* dev = comms.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
  (void)dev;
  return true;
}

//=============================================================================
// GPIO TESTS
//=============================================================================

static bool test_gpio_contains() noexcept {
  auto& gpio = VORTEX_API.gpio;
  // Probe a few expected pin names – result depends on pin config
  [[maybe_unused]] bool a = gpio.Contains("LED_STATUS");
  [[maybe_unused]] bool b = gpio.Contains("MOTOR_ENABLE");
  [[maybe_unused]] bool c = gpio.Contains("IMU_INTERRUPT");
  return true;
}

//=============================================================================
// MOTOR TESTS
//=============================================================================

static bool test_motor_handler_access() noexcept {
  auto& motors = VORTEX_API.motors;
  auto* handler = motors.handler(0);
  (void)handler;
  return true;
}

static bool test_motor_visit_driver() noexcept {
  auto& motors = VORTEX_API.motors;
  bool visited = false;
  motors.visitDriver([&visited](auto& driver) {
    visited = true;
    (void)driver;
  }, 0);
  // visited may be false if no hardware – that is fine
  return true;
}

static bool test_motor_device_count() noexcept {
  auto& motors = VORTEX_API.motors;
  uint8_t count = motors.GetDeviceCount();
  // Sanity: shouldn't exceed some reasonable max
  return (count <= 8);
}

//=============================================================================
// ADC TESTS
//=============================================================================

static bool test_adc_read_channel() noexcept {
  auto& adc = VORTEX_API.adc;
  float voltage = 0.0f;
  // Try reading known channel names; failure is OK if hardware absent
  [[maybe_unused]] auto r1 = adc.ReadChannelV("MOTOR_CURRENT", voltage);
  [[maybe_unused]] auto r2 = adc.ReadChannelV("MOTOR_VOLTAGE", voltage);
  [[maybe_unused]] auto r3 = adc.ReadChannelV("TEMP_SENSOR", voltage);
  return true;
}

//=============================================================================
// IMU TESTS
//=============================================================================

static bool test_imu_handler_access() noexcept {
  auto& imu = VORTEX_API.imu;
  auto* handler = imu.GetBno08xHandler(0);
  (void)handler;
  return true;
}

static bool test_imu_sensor_access() noexcept {
  auto& imu = VORTEX_API.imu;
  IBno08xDriverOps* sensor = imu.GetSensor(0);
  (void)sensor;
  return true;
}

static bool test_imu_device_count() noexcept {
  auto& imu = VORTEX_API.imu;
  uint8_t count = imu.GetDeviceCount();
  return (count <= 4);
}

//=============================================================================
// ENCODER TESTS
//=============================================================================

static bool test_encoder_handler_access() noexcept {
  auto& enc = VORTEX_API.encoders;
  auto* handler = enc.GetAs5047uHandler(0);
  (void)handler;
  return true;
}

static bool test_encoder_read_angle() noexcept {
  auto& enc = VORTEX_API.encoders;
  uint16_t angle = 0;
  [[maybe_unused]] auto err = enc.ReadAngle(0, angle);
  return true;
}

static bool test_encoder_read_velocity() noexcept {
  auto& enc = VORTEX_API.encoders;
  double rpm = 0.0;
  [[maybe_unused]] auto err = enc.ReadVelocityRPM(0, rpm);
  return true;
}

static bool test_encoder_device_count() noexcept {
  auto& enc = VORTEX_API.encoders;
  uint8_t count = enc.GetDeviceCount();
  return (count <= 4);
}

//=============================================================================
// LED TESTS
//=============================================================================

static bool test_led_set_status() noexcept {
  auto& leds = VORTEX_API.leds;
  [[maybe_unused]] auto err = leds.SetStatus(LedAnimation::STATUS_OK);
  return true;
}

static bool test_led_get_color() noexcept {
  auto& leds = VORTEX_API.leds;
  LedColor color{};
  [[maybe_unused]] auto err = leds.GetCurrentColor(color);
  return true;
}

static bool test_led_get_brightness() noexcept {
  auto& leds = VORTEX_API.leds;
  uint8_t brightness = 0;
  [[maybe_unused]] auto err = leds.GetCurrentBrightnessPercent(brightness);
  return true;
}

//=============================================================================
// TEMPERATURE TESTS
//=============================================================================

static bool test_temp_read_esp32() noexcept {
  auto& temp = VORTEX_API.temp;
  float celsius = 0.0f;
  [[maybe_unused]] auto err = temp.ReadTemperatureCelsius("ESP32_INTERNAL", &celsius);
  return true;
}

static bool test_temp_read_ntc() noexcept {
  auto& temp = VORTEX_API.temp;
  float celsius = 0.0f;
  [[maybe_unused]] auto err = temp.ReadTemperatureCelsius("NTC_THERMISTOR", &celsius);
  return true;
}

static bool test_temp_read_motor() noexcept {
  auto& temp = VORTEX_API.temp;
  float celsius = 0.0f;
  [[maybe_unused]] auto err = temp.ReadTemperatureCelsius("MOTOR_TEMP", &celsius);
  return true;
}

//=============================================================================
// UTILITY TESTS
//=============================================================================

static bool test_system_uptime() noexcept {
  auto& vortex = VORTEX_API;
  uint64_t uptime = vortex.GetSystemUptimeMs();
  // Uptime should be > 0 once running
  return (uptime > 0);
}

static bool test_init_time() noexcept {
  auto& vortex = VORTEX_API;
  vortex.EnsureInitialized();
  uint64_t init_time = vortex.GetInitializationTimeMs();
  // Should have taken at least 1 ms
  return (init_time > 0);
}

static bool test_system_version() noexcept {
  auto& vortex = VORTEX_API;
  const char* ver = vortex.GetSystemVersion();
  return (ver != nullptr && ver[0] != '\0');
}

static bool test_dump_statistics() noexcept {
  auto& vortex = VORTEX_API;
  vortex.DumpSystemStatistics();
  return true;
}

//=============================================================================
// ENTRY POINT
//=============================================================================

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "\n");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  Vortex HAL — Comprehensive API Test");
  ESP_LOGI(TAG, "========================================\n");

  // ── Initialization ──────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_INIT_TESTS, "INITIALIZATION",
    RUN_TEST(test_singleton_access);
    RUN_TEST(test_ensure_initialized);
    RUN_TEST(test_is_initialized_after_init);
    RUN_TEST(test_idempotent_init);
  );

  // ── Diagnostics ─────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_DIAGNOSTICS_TESTS, "DIAGNOSTICS",
    RUN_TEST(test_get_system_diagnostics);
    RUN_TEST(test_component_init_status);
    RUN_TEST(test_perform_health_check);
    RUN_TEST(test_failed_components_api);
    RUN_TEST(test_system_warnings_api);
  );

  // ── Comms ───────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_COMMS_TESTS, "COMMUNICATION CHANNELS",
    RUN_TEST(test_comms_spi_device_lookup);
    RUN_TEST(test_comms_i2c_device_lookup);
  );

  // ── GPIO ────────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_GPIO_TESTS, "GPIO",
    RUN_TEST(test_gpio_contains);
  );

  // ── Motors ──────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_MOTOR_TESTS, "MOTOR CONTROLLER",
    RUN_TEST(test_motor_handler_access);
    RUN_TEST(test_motor_visit_driver);
    RUN_TEST(test_motor_device_count);
  );

  // ── ADC ─────────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_ADC_TESTS, "ADC",
    RUN_TEST(test_adc_read_channel);
  );

  // ── IMU ─────────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_IMU_TESTS, "IMU",
    RUN_TEST(test_imu_handler_access);
    RUN_TEST(test_imu_sensor_access);
    RUN_TEST(test_imu_device_count);
  );

  // ── Encoders ────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_ENCODER_TESTS, "ENCODERS",
    RUN_TEST(test_encoder_handler_access);
    RUN_TEST(test_encoder_read_angle);
    RUN_TEST(test_encoder_read_velocity);
    RUN_TEST(test_encoder_device_count);
  );

  // ── LEDs ────────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_LED_TESTS, "LEDS",
    RUN_TEST(test_led_set_status);
    RUN_TEST(test_led_get_color);
    RUN_TEST(test_led_get_brightness);
  );

  // ── Temperature ─────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_TEMPERATURE_TESTS, "TEMPERATURE",
    RUN_TEST(test_temp_read_esp32);
    RUN_TEST(test_temp_read_ntc);
    RUN_TEST(test_temp_read_motor);
  );

  // ── Utility ─────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_UTILITY_TESTS, "UTILITY",
    RUN_TEST(test_system_uptime);
    RUN_TEST(test_init_time);
    RUN_TEST(test_system_version);
    RUN_TEST(test_dump_statistics);
  );

  // ── Summary ─────────────────────────────────────────────────────────────
  print_test_summary(g_test_results, "VORTEX API", TAG);
  cleanup_test_progress_indicator();

  ESP_LOGI(TAG, "\nVortex HAL API test complete.");
}
