/**
 * @file vortex_motor_comms_smoke.cpp
 * @brief TMC9660 comms smoke: PCAL95555 control-line readback, bootloader bring-up, telemetry (no motion).
 */
#include "api/Vortex.h"
#include "managers/GpioManager.h"
#include "managers/MotorController.h"
#include "vortex_motor_bench_common.hpp"

#include "esp_log.h"

static const char* TAG = "vortex_motor_comms";

static void log_pin_state(const char* name, BaseGpio* pin) noexcept {
  if (!pin) {
    ESP_LOGW(TAG, "  %s: (not registered)", name);
    return;
  }
  bool active = false;
  const auto err = pin->IsActive(active);
  if (err == hf_gpio_err_t::GPIO_SUCCESS) {
    ESP_LOGI(TAG, "  %s: logical_active=%d", name, static_cast<int>(active));
  } else {
    ESP_LOGW(TAG, "  %s: IsActive failed err=%d", name, static_cast<int>(err));
  }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Motor comms smoke — no shaft torque; 24 V nominal, ~1 A limit in motion apps only");

    if (!VORTEX_API.EnsureInitialized()) {
        ESP_LOGE(TAG, "Vortex init failed");
        return;
    }

    // PCAL95555 lines that feed HalSpiTmc9660Comm (RST/DRV_EN/FAULTN/WAKE) — must be readable after GpioManager init.
    ESP_LOGI(TAG, "PCAL95555 + TMC control net readback (post-init); GPIO registry size=%zu",
             VORTEX_API.gpio.Size());
    auto& gpio = VORTEX_API.gpio;
    // Name-based lookup matches pincfg `str` field (same as GpioManager::FindByName).
    log_pin_state("GPIO_PCAL_TMC_RST_CTRL", gpio.Get("GPIO_PCAL_TMC_RST_CTRL"));
    log_pin_state("GPIO_PCAL_TMC_DRV_EN", gpio.Get("GPIO_PCAL_TMC_DRV_EN"));
    log_pin_state("GPIO_PCAL_TMC_WAKE_CTRL", gpio.Get("GPIO_PCAL_TMC_WAKE_CTRL"));
    log_pin_state("GPIO_PCAL_TMC_FAULT_STATUS", gpio.Get("GPIO_PCAL_TMC_FAULT_STATUS"));
    log_pin_state("GPIO_PCAL_PWR_GOOD", gpio.Get("GPIO_PCAL_PWR_GOOD"));

    auto& motors = VORTEX_API.motors;
    ESP_LOGI(TAG, "MotorController snapshot: device_count=%u onboard_handler=%s",
             static_cast<unsigned>(motors.GetDeviceCount()),
             motors.handler(MotorController::ONBOARD_TMC9660_INDEX) ? "present" : "null");
    if (!motors.EnsureInitialized()) {
      ESP_LOGW(TAG, "MotorController not initialized (PCAL/SPI missing or TMC9660 bootloader failed)");
      motors.DumpStatistics();
      return;
    }

    ESP_LOGI(TAG, "MotorController OK — dumping stats then TMC9660 telemetry");
    motors.DumpStatistics();

    motors.visitDriver(
        [](auto& d) { vortex_motor_bench::log_motor_comms(TAG, d); }, MotorController::ONBOARD_TMC9660_INDEX);

    // Drive-observability hooks: one pipelined batch each.
    MotorTelemetrySnapshot snap{};
    const MotorError te = motors.ReadTelemetry(MotorTelemetrySet::Full, snap);
    ESP_LOGI(TAG, "ReadTelemetry(Full): %s, %u accesses, %u failed, %lu us; pos=%ld vel=%ld Vbus=%.1f V chip=%.1f C",
             MotorErrorToString(te), static_cast<unsigned>(snap.accesses), static_cast<unsigned>(snap.failed),
             static_cast<unsigned long>(snap.batch_duration_us), static_cast<long>(snap.ActualPosition()),
             static_cast<long>(snap.ActualVelocity()), static_cast<double>(snap.SupplyVoltageV()),
             static_cast<double>(snap.ChipTemperatureC()));
    MotorDriveDiagnostics diag{};
    const MotorError de = motors.ReadDriveDiagnostics(diag);
    ESP_LOGI(TAG, "ReadDriveDiagnostics: %s, severity=%u status=0x%08lX errors=0x%08lX gdrv=0x%08lX adc=0x%04lX "
                  "reset_seen=%d regulation_stopped=%d",
             MotorErrorToString(de), static_cast<unsigned>(diag.severity),
             static_cast<unsigned long>(diag.general_status), static_cast<unsigned long>(diag.general_errors),
             static_cast<unsigned long>(diag.gate_driver_errors), static_cast<unsigned long>(diag.adc_status),
             diag.chip_reset_detected ? 1 : 0, diag.regulation_stopped ? 1 : 0);

    ESP_LOGI(TAG, "Motor comms smoke done");
}
