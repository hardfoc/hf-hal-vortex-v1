/**
 * @file vortex_bldc_torque_foc_hall.cpp
 * @brief Hall-FOC torque-mode bench — direct Iq command (no velocity loop).
 *
 * Sequence:
 *   1. Vortex + MotorController bring-up.
 *   2. Full TMC9660 BLDC config with Hall sensor.
 *   3. `setCommutationMode(FOC_HALL_SENSOR)`.
 *   4. **Direct** torque command via `torqueFluxControl.setTargetTorque()` with
 *      `kTorqueModeTargetMa` (mA, signed) — current is closed-loop, velocity floats.
 *   5. Telemetry poll while running, stop with negative ramp, then `motor_stop_safe`.
 *
 * @note Same bootloader prerequisite as `vortex_bldc_velocity_foc_hall`:
 *       Hall must be enabled in the TMC9660 bootloader config (GPIO mux).
 *
 * @note **Mechanical safety:** in torque mode the motor will spin up freely if
 *       unloaded and may exceed the rated speed. Always run with a load (or
 *       low torque + secured shaft).
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>

static const char* TAG = "vortex_bldc_torq";

extern "C" void app_main(void) {
    ESP_LOGW(TAG,
             "MOTION APP — Hall FOC torque, target=%d mA, current cap %u mA, run %lu ms",
             static_cast<int>(vortex_bench_safety::kTorqueModeTargetMa),
             static_cast<unsigned>(vortex_bench_safety::kMaxPhaseCurrentMa),
             static_cast<unsigned long>(vortex_bench_safety::kFocSpinDurationMs));

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
        ESP_LOGE(TAG, "Init failed (Vortex / MotorController)");
        return;
    }
    auto& motors = VORTEX_API.motors;

    bool ready = false;
    motors.visitDriver(
        [&ready](auto& d) {
            ready = vortex_motor_bench::configure_complete_bldc(d, TAG);
            if (!ready) return;
            (void)vortex_motor_bench::configure_hall(d, TAG);
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!ready) {
        ESP_LOGE(TAG, "BLDC config failed");
        return;
    }

    motors.visitDriver(
        [](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_HALL_SENSOR)) {
                ESP_LOGE(TAG, "setCommutationMode(FOC_HALL_SENSOR) failed");
                return;
            }
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kSettleAfterModeChangeMs));

    motors.visitDriver(
        [](auto& d) {
            const int16_t target = vortex_bench_safety::kTorqueModeTargetMa;
            if (!d.torqueFluxControl.setTargetTorque(target)) {
                ESP_LOGE(TAG, "setTargetTorque(%d mA) failed", static_cast<int>(target));
            } else {
                ESP_LOGI(TAG, "Iq command = %d mA (closed-loop torque)", static_cast<int>(target));
            }
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    const uint32_t end_ms = vortex_bench_safety::kFocSpinDurationMs;
    const uint32_t step_ms = vortex_bench_safety::kTelemetryPollPeriodMs;
    for (uint32_t elapsed = 0; elapsed < end_ms; elapsed += step_ms) {
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        motors.visitDriver(
            [elapsed](auto& d) {
                char label[24];
                snprintf(label, sizeof(label), "torq@%lu", static_cast<unsigned long>(elapsed));
                vortex_motor_bench::log_telemetry(d, TAG, label);
            },
            MotorController::ONBOARD_TMC9660_INDEX);
    }

    motors.visitDriver(
        [](auto& d) {
            // Ramp current down before SYSTEM_OFF for a softer stop.
            (void)d.torqueFluxControl.setTargetTorque(0);
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    vTaskDelay(pdMS_TO_TICKS(150));

    motors.visitDriver(
        [](auto& d) {
            vortex_motor_bench::motor_stop_safe(d, TAG);
            vortex_motor_bench::log_fault_flags(d, TAG, "post-stop");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "Hall-FOC torque bench finished");
}
