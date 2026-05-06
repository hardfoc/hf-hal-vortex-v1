/**
 * @file vortex_bldc_open_loop.cpp
 * @brief Open-loop FOC voltage spin — full Vortex BLDC bring-up + telemetry.
 *
 * Sequence:
 *   1. Vortex + MotorController bring-up (`EnsureInitialized`).
 *   2. EvKit-style full TMC9660 BLDC config (gate driver, current sensing,
 *      motor, FOC PI, protection, ramp, brake, ADC cal, DRV_EN).
 *   3. Switch to `FOC_OPENLOOP_VOLTAGE_MODE`, command low velocity,
 *      poll telemetry while spinning.
 *   4. `motor_stop_safe()` — stop, SYSTEM_OFF, DRV_EN inactive.
 *
 * Tunables in `common/vortex_bench_safety.hpp`. Open-loop voltage mode does
 * not need a feedback sensor — useful as the first hardware power-on check
 * with the shaft secured.
 *
 * **Motion warning:** secure the motor; supply 12–24 V; stay near the
 * `kOpenLoopTargetVelocity` value until you confirm direction and current.
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_bldc_openloop";

extern "C" void app_main(void) {
    ESP_LOGW(TAG,
             "MOTION APP — 24V class, %u pole pairs, current cap %u mA, target=%ld units, spin %lu ms",
             static_cast<unsigned>(vortex_bench_safety::kDefaultPolePairs),
             static_cast<unsigned>(vortex_bench_safety::kMaxPhaseCurrentMa),
             static_cast<long>(vortex_bench_safety::kOpenLoopTargetVelocity),
             static_cast<unsigned long>(vortex_bench_safety::kOpenLoopSpinDurationMs));

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
        ESP_LOGE(TAG, "Init failed (Vortex / MotorController)");
        return;
    }

    auto& motors = VORTEX_API.motors;

    bool ready = false;
    motors.visitDriver(
        [&ready](auto& d) {
            ready = vortex_motor_bench::configure_complete_bldc(d, TAG);
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!ready) {
        ESP_LOGE(TAG, "BLDC config failed — aborting before commutation/DRV_EN engage");
        return;
    }

    motors.visitDriver(
        [](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE)) {
                ESP_LOGE(TAG, "setCommutationMode(FOC_OPENLOOP_VOLTAGE_MODE) failed");
                return;
            }
            ESP_LOGI(TAG, "Mode: FOC_OPENLOOP_VOLTAGE_MODE — ramping up");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kSettleAfterModeChangeMs));

    motors.visitDriver(
        [](auto& d) {
            if (!d.velocityControl.setTargetVelocity(vortex_bench_safety::kOpenLoopTargetVelocity)) {
                ESP_LOGE(TAG, "setTargetVelocity failed");
            }
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    // Spin + poll telemetry
    const uint32_t end_ms = vortex_bench_safety::kOpenLoopSpinDurationMs;
    const uint32_t step_ms = vortex_bench_safety::kTelemetryPollPeriodMs;
    for (uint32_t elapsed = 0; elapsed < end_ms; elapsed += step_ms) {
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        motors.visitDriver(
            [elapsed](auto& d) {
                char label[24];
                snprintf(label, sizeof(label), "spin@%lu", static_cast<unsigned long>(elapsed));
                vortex_motor_bench::log_telemetry(d, TAG, label);
            },
            MotorController::ONBOARD_TMC9660_INDEX);
    }

    motors.visitDriver(
        [](auto& d) {
            vortex_motor_bench::motor_stop_safe(d, TAG);
            vortex_motor_bench::log_fault_flags(d, TAG, "post-stop");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "Open-loop profile finished OK");
}
