/**
 * @file vortex_bldc_open_loop.cpp
 * @brief Open-loop BLDC spin on the **onboard SPI** TMCL path (PCAL mux enables
 *        `SPI_COMM_EN` — see `Vortex::SetOnboardTmc9660Transport`). Uses the same Vortex power-stage
 *        workaround as `vortex_bldc_open_loop_uart`: disable UVW VGS-short
 *        protection defaults before entering `FOC_OPENLOOP_CURRENT_MODE`.
 *
 * For UART-only validation and richer telemetry, prefer
 * `vortex_bldc_open_loop_uart` and [`docs/BLDC_UART_BRINGUP.md`](../docs/BLDC_UART_BRINGUP.md).
 *
 * Sequence:
 *   1. `SetOnboardTmc9660Transport(Spi)` then `VORTEX_API.EnsureInitialized()` + `motors.IsInitialized()`.
 *   2. `configure_complete_bldc(..., with_oc_vgs_protection=false,
 *      with_gate_current_limits=false, skip_y2_phase=true)`.
 *   3. `disable_vortex_uvw_vgs_short_protection()` — see BLDC_UART_BRINGUP.
 *   4. Ramper for open loop, `OPENLOOP_CURRENT` **before** commutation mode,
 *      `clear_fault_flags`, `FOC_OPENLOOP_CURRENT_MODE`, settle, optional
 *      commutation read-back retry.
 *   5. Short `setTargetVelocity` ramp to `kOpenLoopTargetVelocity` (≥ `kOpenLoopMinCommandVelocity`),
 *      Erratum 4 coalesce delay, read back `COMMUTATION_MODE` / `TARGET_VELOCITY`.
 *   6. Telemetry poll loop; `motor_stop_safe()`.
 *
 * **Motion warning:** secure the motor; supply 12–24 V; see `vortex_bench_safety.hpp`.
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"
#include "vortex_motor_bench_probe.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_bldc_openloop";

namespace {
/// Short ramp so the first `TARGET_VELOCITY` is not a single large step (mirrors
/// `vortex_bldc_open_loop_uart` behavior; helps after mode change / Erratum 4 scheduling).
constexpr unsigned kVelRampSteps       = 4;
constexpr uint32_t kVelRampStepDelayMs = 120;
}  // namespace

extern "C" void app_main(void) {
    ESP_LOGW(TAG,
             "MOTION APP — SPI transport, %u pole pairs, I_open=%u mA, vel=%ld units, spin %lu ms",
             static_cast<unsigned>(vortex_bench_safety::kDefaultPolePairs),
             static_cast<unsigned>(vortex_bench_safety::kOpenLoopCurrentMa),
             static_cast<long>(vortex_bench_safety::kOpenLoopTargetVelocity),
             static_cast<unsigned long>(vortex_bench_safety::kOpenLoopSpinDurationMs));

    Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Spi);

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.IsInitialized()) {
        ESP_LOGE(TAG, "Init failed (Vortex / MotorController)");
        return;
    }

    vortex_motor_bench_probe::dump_pcal95555_lines(TAG, "pre-config");

    auto& motors = VORTEX_API.motors;

    bool ready = false;
    motors.visitDriver(
        [&ready](auto& d) {
            ready = vortex_motor_bench::configure_complete_bldc(
                d, TAG,
                vortex_bench_safety::kDefaultPolePairs,
                vortex_bench_safety::kPwmFrequencyHz,
                /*do_calibrate=*/true,
                /*enable_outputs=*/true,
                /*with_oc_vgs_protection=*/false,
                /*with_gate_current_limits=*/false,
                /*skip_y2_phase=*/true);
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!ready) {
        ESP_LOGE(TAG, "BLDC config failed — aborting before commutation/DRV_EN engage");
        return;
    }

    motors.visitDriver(
        [](auto& d) { vortex_motor_bench::disable_vortex_uvw_vgs_short_protection(d, TAG); },
        MotorController::ONBOARD_TMC9660_INDEX);

    bool motion_ok = false;
    motors.visitDriver(
        [&motion_ok](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            if (!vortex_motor_bench::configure_ramp_for_open_loop_voltage(d, TAG)) {
                return;
            }
            if (!d.torqueFluxControl.setOpenloopCurrent(vortex_bench_safety::kOpenLoopCurrentMa)) {
                ESP_LOGE(TAG, "setOpenloopCurrent(%u mA) failed",
                         static_cast<unsigned>(vortex_bench_safety::kOpenLoopCurrentMa));
                return;
            }
            vortex_motor_bench::clear_fault_flags(d);
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_CURRENT_MODE)) {
                ESP_LOGE(TAG, "setCommutationMode(FOC_OPENLOOP_CURRENT_MODE) failed");
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kSettleAfterModeChangeMs));

            uint32_t cm = 0xFFFFFFFFu;
            (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            if (cm != static_cast<uint32_t>(tmcl::CommutationMode::FOC_OPENLOOP_CURRENT_MODE)) {
                ESP_LOGW(TAG, "COMMUTATION_MODE=%u (expected 4), retry after fault clear",
                         static_cast<unsigned>(cm));
                vortex_motor_bench::log_fault_flags(d, TAG, "after-revert");
                vortex_motor_bench::clear_fault_flags(d);
                (void)d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_CURRENT_MODE);
                vTaskDelay(pdMS_TO_TICKS(50));
                (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            }
            motion_ok = (cm == static_cast<uint32_t>(tmcl::CommutationMode::FOC_OPENLOOP_CURRENT_MODE));
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    if (!motion_ok) {
        ESP_LOGE(TAG, "Could not arm open-loop current mode — abort");
        motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d, TAG); },
                           MotorController::ONBOARD_TMC9660_INDEX);
        return;
    }

    const int32_t v_target = vortex_bench_safety::kOpenLoopTargetVelocity;
    const int32_t v_floor  = vortex_bench_safety::kOpenLoopMinCommandVelocity;
    for (unsigned step = 1; step <= kVelRampSteps; ++step) {
        const int32_t v_raw =
            static_cast<int32_t>(static_cast<int64_t>(v_target) * static_cast<int>(step) /
                                 static_cast<int>(kVelRampSteps));
        const int32_t v_cmd = (v_raw < v_floor) ? v_floor : v_raw;
        bool wrote = false;
        motors.visitDriver(
            [v_cmd, &wrote](auto& d) { wrote = d.velocityControl.setTargetVelocity(v_cmd); },
            MotorController::ONBOARD_TMC9660_INDEX);
        if (!wrote) {
            ESP_LOGE(TAG, "setTargetVelocity(%ld) failed at ramp step %u/%u",
                     static_cast<long>(v_cmd), step, kVelRampSteps);
            motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d, TAG); },
                               MotorController::ONBOARD_TMC9660_INDEX);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(kVelRampStepDelayMs));
    }

    vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kErratum4TargetCoalesceMs));

    motors.visitDriver(
        [](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            uint32_t cm = 0, tv = 0;
            (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            (void)d.readParameter(tmcl::Parameters::TARGET_VELOCITY, tv);
            ESP_LOGI(TAG,
                     "post-velocity: COMMUTATION_MODE=%u (expect 4) TARGET_VELOCITY_rb=%ld "
                     "(Erratum 4 delay applied)",
                     static_cast<unsigned>(cm), static_cast<long>(static_cast<int32_t>(tv)));
            if (cm != static_cast<uint32_t>(tmcl::CommutationMode::FOC_OPENLOOP_CURRENT_MODE)) {
                ESP_LOGW(TAG,
                         "Commutation mode reverted — VGS SAP writes may have failed; check "
                         "\"VGS UVW short protection\" log line");
            }
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    const uint32_t end_ms = vortex_bench_safety::kOpenLoopSpinDurationMs;
    const uint32_t step_ms = vortex_bench_safety::kTelemetryPollPeriodMs;
    for (uint32_t elapsed = 0; elapsed < end_ms; elapsed += step_ms) {
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        motors.visitDriver(
            [elapsed](auto& d) {
                char label[24];
                snprintf(label, sizeof(label), "spin@%lu", static_cast<unsigned long>(elapsed));
                vortex_motor_bench::log_telemetry(d, TAG, label);
                vortex_motor_bench::log_open_loop_telemetry(d, TAG, label);
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
