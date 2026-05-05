/**
 * @file vortex_api_test.cpp
 * @brief Primary hardware integration test for the Vortex HAL on ESP32 (noexcept)
 *
 * This is the default app (`app_config.yml`) and the main place to learn correct
 * Vortex API usage patterns that are also verified on real boards: init order,
 * diagnostics, comms/GPIO/ADC/IMU/encoder/motor/LED/temperature surfaces, and
 * board-relevant pin/channel names (see `main/common/vortex_board_pins.hpp`).
 *
 * Coverage intentionally subsumes the retired per-manager “documentation examples”:
 * GPIO registry + fault/user lines, ADC VM averaged/raw + diagnostics, comms
 * health dumps, LED solids/brightness/animations/status frames, and temperature
 * diagnostics alongside named sensor reads.
 *
 * For deeper single-subsystem runs, flash `led_temp_test`, `encoder_imu_test`,
 * `motor_controller_test`, or smoke apps listed in `docs/BENCH_MATRIX.md`.
 *
 * **Strict hardware mode:** `idf.py menuconfig` → *Component config* → *Main* →
 * *Vortex API test* → enable
 * `CONFIG_VORTEX_API_TEST_STRICT_HW` (and optionally `..._STRICT_SENSORS`) when
 * the bench matches `docs/BENCH_MATRIX.md` so failing wiring or supply shows up
 * as test failures instead of silent passes.
 *
 * All test bodies are noexcept.
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

#include "common/vortex_api_test_strict_hw.hpp"

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

  if (diag.total_components != static_cast<uint32_t>(kVortexManagerCount)) return false;

  // initialized_components must be <= total
  if (diag.initialized_components > diag.total_components) return false;

  return true;
}

static bool test_component_init_status() noexcept {
  auto& vortex = VORTEX_API;
  vortex.EnsureInitialized();

  auto status = vortex.GetComponentInitializationStatus();
  return (status.size() == kVortexManagerCount);
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

static bool test_collect_manager_health() noexcept {
  auto& vortex = VORTEX_API;
  vortex.EnsureInitialized();

  ManagerHealthSnapshot health{};
  bool ok = vortex.CollectManagerHealth(health);
  if (!ok) return false;

  if (health.count != kVortexManagerCount) return false;

  // Every entry should have a non-null name
  for (size_t i = 0; i < health.count; ++i) {
    if (health.entries[i].name == nullptr) return false;
  }
  return true;
}

//=============================================================================
// COMMS TESTS
//=============================================================================

static bool test_comms_get_last_error() noexcept {
  auto& comms = VORTEX_API.comms;
  [[maybe_unused]] CommError err = comms.GetLastError();
  return true;
}

static bool test_comms_system_diagnostics() noexcept {
  auto& comms = VORTEX_API.comms;
  CommSystemDiagnostics diag{};
  [[maybe_unused]] auto err = comms.GetSystemDiagnostics(diag);
  return true;
}

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

static bool test_comms_initialized_and_statistics_dump() noexcept {
  auto& comms = VORTEX_API.comms;
  if (vortex_api_test_strict::kHwChecks && !comms.IsInitialized()) {
    return false;
  }
  comms.DumpStatistics();
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

static bool test_gpio_registry_size_sane() noexcept {
  auto& gpio = VORTEX_API.gpio;
  const size_t n = gpio.Size();
  return (n > 0U && n < 512U);
}

static bool test_gpio_contains_board_routing_names() noexcept {
  auto& gpio = VORTEX_API.gpio;
  const bool has_fault = gpio.Contains("GPIO_TMC_FAULTN_0");
  const bool has_enc_cs = gpio.Contains("GPIO_ENCODER_CSN");
  const bool has_user = gpio.Contains("GPIO_USER_LED");
  if (vortex_api_test_strict::kHwChecks) {
    return has_fault && has_enc_cs && has_user;
  }
  (void)has_fault;
  (void)has_enc_cs;
  (void)has_user;
  return true;
}

static bool test_gpio_read_optional_fault_line() noexcept {
  auto& gpio = VORTEX_API.gpio;
  // Same convention as the old GpioManager example: true => line high => not faulted.
  bool not_faulted_line_high = false;
  const auto err = gpio.Read("GPIO_TMC_FAULTN_0", not_faulted_line_high);
  if (vortex_api_test_strict::kHwChecks) {
    return (err == hf_gpio_err_t::GPIO_SUCCESS) && not_faulted_line_high;
  }
  return true;
}

static bool test_gpio_toggle_user_led_best_effort() noexcept {
  auto& gpio = VORTEX_API.gpio;
  if (!gpio.Contains("GPIO_USER_LED")) {
    return !vortex_api_test_strict::kHwChecks;
  }
  const auto e1 = gpio.Toggle("GPIO_USER_LED");
  const auto e2 = gpio.Toggle("GPIO_USER_LED");
  if (vortex_api_test_strict::kHwChecks) {
    return (e1 == hf_gpio_err_t::GPIO_SUCCESS) && (e2 == hf_gpio_err_t::GPIO_SUCCESS);
  }
  return true;
}

static bool test_gpio_system_diagnostics_and_dump() noexcept {
  auto& gpio = VORTEX_API.gpio;
  GpioSystemDiagnostics diag{};
  (void)gpio.GetSystemDiagnostics(diag);
  gpio.DumpStatistics();
  return true;
}

//=============================================================================
// MOTOR TESTS
//=============================================================================

static bool test_motor_handler_access() noexcept {
  auto& motors = VORTEX_API.motors;
  auto* handler = motors.handler(0);
  if (vortex_api_test_strict::kSensorChecks && handler == nullptr) {
    return false;
  }
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
  if (count > 8) {
    return false;
  }
  if (vortex_api_test_strict::kSensorChecks && count < 1) {
    return false;
  }
  return true;
}

static bool test_motor_get_last_error() noexcept {
  auto& motors = VORTEX_API.motors;
  [[maybe_unused]] MotorError err = motors.GetLastError();
  return true;
}

static bool test_motor_system_diagnostics() noexcept {
  auto& motors = VORTEX_API.motors;
  MotorSystemDiagnostics diag{};
  [[maybe_unused]] auto err = motors.GetSystemDiagnostics(diag);
  // active_device_count should be <= 4
  return (diag.active_device_count <= 4);
}

//=============================================================================
// ADC TESTS
//=============================================================================

static bool test_adc_read_channel() noexcept {
  auto& adc = VORTEX_API.adc;
  float voltage = 0.0f;
  // Try reading known channel names; failure is OK if hardware absent
  [[maybe_unused]] auto r1 = adc.ReadVoltage("MOTOR_CURRENT", voltage);
  [[maybe_unused]] auto r2 = adc.ReadVoltage("MOTOR_VOLTAGE", voltage);
  [[maybe_unused]] auto r3 = adc.ReadVoltage("TEMP_SENSOR", voltage);
  return true;
}

static bool test_adc_registry_and_vm_lookup() noexcept {
  auto& adc = VORTEX_API.adc;
  const size_t n = adc.Size();
  const bool has_vm = adc.Contains("ADC_VM");
  if (vortex_api_test_strict::kHwChecks) {
    return (n > 0U) && has_vm;
  }
  (void)n;
  (void)has_vm;
  return true;
}

static bool test_adc_read_vm_averaged_and_raw() noexcept {
  auto& adc = VORTEX_API.adc;
  float vm = 0.0f;
  uint32_t raw = 0U;
  const auto rv = adc.ReadVoltage("ADC_VM", vm, 4, 0);
  const auto rr = adc.ReadRaw("ADC_VM", raw, 4, 0);
  if (vortex_api_test_strict::kHwChecks) {
    if (rv != hf_adc_err_t::ADC_SUCCESS || rr != hf_adc_err_t::ADC_SUCCESS) {
      return false;
    }
    if (vm < vortex_api_test_strict::kVmVoltsMin || vm > vortex_api_test_strict::kVmVoltsMax) {
      return false;
    }
  }
  return true;
}

static bool test_adc_system_diagnostics_and_dump() noexcept {
  auto& adc = VORTEX_API.adc;
  AdcSystemDiagnostics diag{};
  const auto err = adc.GetSystemDiagnostics(diag);
  adc.DumpStatistics();
  if (vortex_api_test_strict::kHwChecks) {
    return (err == hf_adc_err_t::ADC_SUCCESS) && diag.system_healthy;
  }
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
  if (vortex_api_test_strict::kSensorChecks && sensor == nullptr) {
    return false;
  }
  return true;
}

static bool test_imu_device_count() noexcept {
  auto& imu = VORTEX_API.imu;
  uint8_t count = imu.GetDeviceCount();
  if (count > 4) {
    return false;
  }
  if (vortex_api_test_strict::kSensorChecks && count < 1) {
    return false;
  }
  return true;
}

static bool test_imu_get_last_error() noexcept {
  auto& imu = VORTEX_API.imu;
  [[maybe_unused]] ImuError err = imu.GetLastError();
  return true;
}

static bool test_imu_system_diagnostics() noexcept {
  auto& imu = VORTEX_API.imu;
  ImuSystemDiagnostics diag{};
  [[maybe_unused]] auto err = imu.GetSystemDiagnostics(diag);
  return (diag.active_device_count <= 4);
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
  const auto err = enc.ReadAngle(0, angle);
  if (vortex_api_test_strict::kSensorChecks && err != EncoderError::SUCCESS) {
    return false;
  }
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
  if (count > 4) {
    return false;
  }
  if (vortex_api_test_strict::kSensorChecks && count < 1) {
    return false;
  }
  return true;
}

static bool test_encoder_get_last_error() noexcept {
  auto& enc = VORTEX_API.encoders;
  [[maybe_unused]] EncoderError err = enc.GetLastError();
  return true;
}

static bool test_encoder_system_diagnostics() noexcept {
  auto& enc = VORTEX_API.encoders;
  EncoderSystemDiagnostics diag{};
  [[maybe_unused]] auto err = enc.GetSystemDiagnostics(diag);
  return (diag.active_device_count <= 4);
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

static bool test_led_set_solid_palette() noexcept {
  auto& leds = VORTEX_API.leds;
  (void)leds.SetColor(LedColors::RED);
  (void)leds.SetColor(LedColors::GREEN);
  (void)leds.SetColor(LedColors::BLUE);
  (void)leds.SetColor(LedColor(255, 165, 0));
  return true;
}

static bool test_led_brightness_steps() noexcept {
  auto& leds = VORTEX_API.leds;
  (void)leds.SetColor(LedColors::WHITE);
  for (uint8_t pct = 20; pct <= 100; pct += 20) {
    (void)leds.SetBrightnessPercent(pct);
  }
  (void)leds.SetBrightnessPercent(80);
  return true;
}

static bool test_led_animation_rainbow_and_breath_updates() noexcept {
  auto& leds = VORTEX_API.leds;
  (void)leds.StartAnimation(LedAnimation::RAINBOW);
  for (int i = 0; i < 12; ++i) {
    (void)leds.UpdateAnimation();
  }
  (void)leds.StopAnimation();
  (void)leds.StartAnimation(LedAnimation::BREATH, LedColors::CYAN);
  for (int i = 0; i < 8; ++i) {
    (void)leds.UpdateAnimation();
  }
  (void)leds.StopAnimation();
  (void)leds.TurnOff();
  return true;
}

static bool test_led_indicate_lifecycle_one_frame_each() noexcept {
  auto& leds = VORTEX_API.leds;
  [[maybe_unused]] auto e0 = leds.IndicateBoot();
  (void)leds.UpdateAnimation();
  [[maybe_unused]] auto e1 = leds.IndicateReady();
  (void)leds.UpdateAnimation();
  [[maybe_unused]] auto e2 = leds.IndicateWarning();
  (void)leds.UpdateAnimation();
  [[maybe_unused]] auto e3 = leds.IndicateError();
  (void)leds.UpdateAnimation();
  (void)leds.StopAnimation();
  return true;
}

static bool test_led_system_diagnostics_dump_and_last_error() noexcept {
  auto& leds = VORTEX_API.leds;
  LedSystemDiagnostics diag{};
  const auto err = leds.GetSystemDiagnostics(diag);
  leds.DumpStatistics();
  (void)leds.GetLastError();
  if (vortex_api_test_strict::kHwChecks) {
    return (err == LedError::SUCCESS) && diag.system_healthy && diag.led_initialized;
  }
  return true;
}

//=============================================================================
// TEMPERATURE TESTS
//=============================================================================

static bool test_temp_read_esp32() noexcept {
  auto& temp = VORTEX_API.temp;
  float celsius = 0.0f;
  const auto err = temp.ReadTemperatureCelsius("ESP32_INTERNAL", celsius);
  if (vortex_api_test_strict::kHwChecks) {
    if (err != hf_temp_err_t::TEMP_SUCCESS) {
      return false;
    }
    if (celsius < -40.0f || celsius > 125.0f) {
      return false;
    }
  }
  return true;
}

static bool test_temp_read_ntc() noexcept {
  auto& temp = VORTEX_API.temp;
  float celsius = 0.0f;
  [[maybe_unused]] auto err = temp.ReadTemperatureCelsius("NTC_THERMISTOR", celsius);
  return true;
}

static bool test_temp_read_motor() noexcept {
  auto& temp = VORTEX_API.temp;
  float celsius = 0.0f;
  [[maybe_unused]] auto err = temp.ReadTemperatureCelsius("MOTOR_TEMP", celsius);
  return true;
}

static bool test_temp_system_diagnostics() noexcept {
  auto& temp = VORTEX_API.temp;
  TempSystemDiagnostics diag{};
  [[maybe_unused]] auto err = temp.GetSystemDiagnostics(diag);
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

/** Re-send the last solid colour so a WS2812 re-powered while the MCU stays up can latch again. */
static void vortex_api_test_led_idle_refresh_task(void* /*param*/) noexcept {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(750));
    auto& leds = VORTEX_API.leds;
    LedColor c{};
    if (leds.GetCurrentColor(c) != LedError::SUCCESS) {
      continue;
    }
    (void)leds.SetColor(c);
  }
}

//=============================================================================
// ENTRY POINT
//=============================================================================

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "\n");
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, "  Vortex HAL — Comprehensive API Test");
  ESP_LOGI(TAG, "========================================\n");
#if CONFIG_VORTEX_API_TEST_STRICT_HW
  ESP_LOGW(TAG, "STRICT_HW: VM rail, TMC fault line, GPIO registry, comms, LED, ESP32 temp — failures indicate bench/wiring.");
#endif
#if CONFIG_VORTEX_API_TEST_STRICT_SENSORS
  ESP_LOGW(TAG, "STRICT_SENSORS: IMU, encoder, motor device presence — requires populated I2C/SPI.");
#endif

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
    RUN_TEST(test_collect_manager_health);
  );

  // ── Comms ───────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_COMMS_TESTS, "COMMUNICATION CHANNELS",
    RUN_TEST(test_comms_get_last_error);
    RUN_TEST(test_comms_system_diagnostics);
    RUN_TEST(test_comms_spi_device_lookup);
    RUN_TEST(test_comms_i2c_device_lookup);
    RUN_TEST(test_comms_initialized_and_statistics_dump);
  );

  // ── GPIO ────────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_GPIO_TESTS, "GPIO",
    RUN_TEST(test_gpio_contains);
    RUN_TEST(test_gpio_registry_size_sane);
    RUN_TEST(test_gpio_contains_board_routing_names);
    RUN_TEST(test_gpio_read_optional_fault_line);
    RUN_TEST(test_gpio_toggle_user_led_best_effort);
    RUN_TEST(test_gpio_system_diagnostics_and_dump);
  );

  // ── Motors ──────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_MOTOR_TESTS, "MOTOR CONTROLLER",
    RUN_TEST(test_motor_handler_access);
    RUN_TEST(test_motor_visit_driver);
    RUN_TEST(test_motor_device_count);
    RUN_TEST(test_motor_get_last_error);
    RUN_TEST(test_motor_system_diagnostics);
  );

  // ── ADC ─────────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_ADC_TESTS, "ADC",
    RUN_TEST(test_adc_read_channel);
    RUN_TEST(test_adc_registry_and_vm_lookup);
    RUN_TEST(test_adc_read_vm_averaged_and_raw);
    RUN_TEST(test_adc_system_diagnostics_and_dump);
  );

  // ── IMU ─────────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_IMU_TESTS, "IMU",
    RUN_TEST(test_imu_handler_access);
    RUN_TEST(test_imu_sensor_access);
    RUN_TEST(test_imu_device_count);
    RUN_TEST(test_imu_get_last_error);
    RUN_TEST(test_imu_system_diagnostics);
  );

  // ── Encoders ────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_ENCODER_TESTS, "ENCODERS",
    RUN_TEST(test_encoder_handler_access);
    RUN_TEST(test_encoder_read_angle);
    RUN_TEST(test_encoder_read_velocity);
    RUN_TEST(test_encoder_device_count);
    RUN_TEST(test_encoder_get_last_error);
    RUN_TEST(test_encoder_system_diagnostics);
  );

  // ── LEDs ────────────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_LED_TESTS, "LEDS",
    RUN_TEST(test_led_set_status);
    RUN_TEST(test_led_get_color);
    RUN_TEST(test_led_get_brightness);
    RUN_TEST(test_led_set_solid_palette);
    RUN_TEST(test_led_brightness_steps);
    RUN_TEST(test_led_animation_rainbow_and_breath_updates);
    RUN_TEST(test_led_indicate_lifecycle_one_frame_each);
    RUN_TEST(test_led_system_diagnostics_dump_and_last_error);
  );

  // ── Temperature ─────────────────────────────────────────────────────────
  RUN_TEST_SECTION_IF_ENABLED(ENABLE_TEMPERATURE_TESTS, "TEMPERATURE",
    RUN_TEST(test_temp_read_esp32);
    RUN_TEST(test_temp_read_ntc);
    RUN_TEST(test_temp_read_motor);
    RUN_TEST(test_temp_system_diagnostics);
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

  // Calm idle colour (tests leave bright white on the last brightness step) + background
  // refresh: if only the LED 5 V rail drops while the MCU keeps running, nothing would
  // otherwise call RMT again until reset.
  {
    auto& leds = VORTEX_API.leds;
    (void)leds.StopAnimation();
    (void)leds.SetMaxBrightness(100);
    (void)leds.SetBrightnessPercent(14);
    (void)leds.SetColor(LedColor(0, 170, 0));
    ESP_LOGI(TAG, "Idle LED: dim green + ~750 ms refresh (WS2812 re-latch if LED supply cycles).");
  }
  static constexpr uint32_t kLedIdleTaskStack = 3072;
  (void)xTaskCreate(vortex_api_test_led_idle_refresh_task, "led_idle", kLedIdleTaskStack, nullptr,
                    tskIDLE_PRIORITY + 1, nullptr);

  ESP_LOGI(TAG, "\nVortex HAL API test complete.");
}
