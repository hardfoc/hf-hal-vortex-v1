/**
 * @file vortex_bldc_velocity_foc_abn.cpp
 * @brief ABN-FOC velocity loop bench — full bring-up + velocity profile.
 *
 * Sequence:
 *   1. Vortex + MotorController bring-up.
 *   2. Full TMC9660 BLDC config.
 *   3. `feedbackSense.configureAuto(AbnConfig{counts_per_rev = kAbnEncoderCountsPerRev})`
 *      with the EvKit `FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING` init method.
 *   4. `setCommutationMode(FOC_ABN)`, command velocity, telemetry sweep.
 *   5. `motor_stop_safe()`.
 *
 * @note **Bootloader prerequisite:** `bootcfg::ABN1Config::enable = true`
 *       with A/B/N pins set (EvKit defaults: GPIO8/13/14). Vortex's stock
 *       `Tmc9660Handler::kDefaultBootConfig` leaves ABN1 disabled — supply
 *       a custom `BootloaderConfig*` enabling it before this app will close
 *       the loop on a real encoder.
 *
 * @note Update `kAbnEncoderCountsPerRev` in `vortex_bench_safety.hpp` for
 *       your encoder; the `FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING` init does
 *       a brief shaft motion at start-up to align the encoder with phi_e.
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_bldc_abn";

extern "C" void app_main(void) {
    ESP_LOGW(TAG,
             "MOTION APP — ABN FOC velocity, target=%ld units, CPR=%lu, current cap %u mA",
             static_cast<long>(vortex_bench_safety::kAbnFocTargetVelocity),
             static_cast<unsigned long>(vortex_bench_safety::kAbnEncoderCountsPerRev),
             static_cast<unsigned>(vortex_bench_safety::kMaxPhaseCurrentMa));

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
            (void)vortex_motor_bench::configure_abn(d, TAG);
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!ready) {
        ESP_LOGE(TAG, "BLDC config failed");
        return;
    }

    motors.visitDriver(
        [](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_ABN)) {
                ESP_LOGE(TAG, "setCommutationMode(FOC_ABN) failed");
                return;
            }
            ESP_LOGI(TAG, "Mode: FOC_ABN — encoder init runs (forced phi_e zero swing, ~1 s)");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    // Encoder init can take ~1s to settle (init_delay default = 1000ms).
    vTaskDelay(pdMS_TO_TICKS(1500));

    motors.visitDriver(
        [](auto& d) {
            if (!d.velocityControl.setTargetVelocity(vortex_bench_safety::kAbnFocTargetVelocity)) {
                ESP_LOGE(TAG, "setTargetVelocity failed");
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
                snprintf(label, sizeof(label), "abn@%lu", static_cast<unsigned long>(elapsed));
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

    ESP_LOGI(TAG, "ABN-FOC velocity bench finished (CPR=%lu)",
             static_cast<unsigned long>(vortex_bench_safety::kAbnEncoderCountsPerRev));
}
