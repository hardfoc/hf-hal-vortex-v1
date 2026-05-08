/**
 * @file vortex_bldc_telemetry_sweep.cpp
 * @brief No-motion telemetry / configuration smoke test for the TMC9660 power stage.
 *
 * Brings the chip all the way up to **DRV_EN active in SYSTEM_OFF** and then
 * polls telemetry without ever applying a commutation mode. Useful for:
 *   - Validating gate-driver / current-sense / protection auto-config without
 *     turning the motor.
 *   - Sanity-checking supply rail and chip temperature.
 *   - Confirming I²C/SPI/UART telemetry path under load when the motor is
 *     unplugged.
 *   - Exercising fault-flag readback + clear behaviour.
 *
 * Safe to run with **no motor connected** — DRV_EN is high but no PWM is
 * generated until a commutation mode is selected.
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_tmc_tlm";

// Bench diagnostics. Default OFF for the normal telemetry sweep. Flip the diag flags to repeat
// the FW051V100 NR 245-248 INVALID_VALUE investigation (see BLDC_UART_BRINGUP.md "Gate-current
// SAP rejection on FW051V100"); flip the `kBenchTry*` flags to pass them through to
// `configure_complete_bldc`. `kBenchProbeGateCurrentNoMotor` runs the EvKit-ordering probe
// (SAP NR 245/246 while MOTOR_TYPE is still NO_MOTOR) immediately after `EnsureInitialized`.
namespace {
constexpr bool kBenchTryGateCurrentLimits     = false;
constexpr bool kBenchTryOcVgsProtection       = false;
constexpr bool kBenchPreTmclGateCurrentDiag   = false;
constexpr bool kBenchPostTmclGateCurrentDiag  = false;
constexpr bool kBenchProbeGateCurrentNoMotor  = false;
}  // namespace

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "TMC9660 telemetry sweep (no commutation, no motion)");
    if (kBenchTryGateCurrentLimits || kBenchTryOcVgsProtection || kBenchPostTmclGateCurrentDiag) {
        ESP_LOGW(TAG,
                 "Bench flags: gate_current_limits=%d oc_vgs_protection=%d post_gate_tmcl_diag=%d",
                 kBenchTryGateCurrentLimits ? 1 : 0, kBenchTryOcVgsProtection ? 1 : 0,
                 kBenchPostTmclGateCurrentDiag ? 1 : 0);
    }

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
        ESP_LOGE(TAG, "Init failed (Vortex / MotorController)");
        return;
    }
    auto& motors = VORTEX_API.motors;

    bool ready = false;
    motors.visitDriver(
        [&ready](auto& d) {
            if (kBenchProbeGateCurrentNoMotor) {
                const bool accepted =
                    vortex_motor_bench::probe_gate_current_with_no_motor_type(d, TAG);
                ESP_LOGW(TAG, "[gate-diag/no-motor] datasheet-default acceptance with NO_MOTOR: %s",
                         accepted ? "YES (EvKit ordering matters!)" : "NO (firmware rule unrelated to motor type)");
            }
            if (kBenchPreTmclGateCurrentDiag) {
                vortex_motor_bench::diagnose_uvw_gate_current_tmcl(d, TAG, "pre-config");
            }
            ready = vortex_motor_bench::configure_complete_bldc(
                d, TAG, vortex_bench_safety::kDefaultPolePairs,
                vortex_bench_safety::kPwmFrequencyHz,
                /*do_calibrate=*/true,
                /*enable_outputs=*/true,
                kBenchTryOcVgsProtection,
                kBenchTryGateCurrentLimits,
                /*skip_y2_phase=*/true);
            if (ready && kBenchPostTmclGateCurrentDiag) {
                vortex_motor_bench::diagnose_uvw_gate_current_tmcl(d, TAG, "post-config");
            }
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!ready) {
        ESP_LOGE(TAG, "BLDC config failed");
        return;
    }

    motors.visitDriver(
        [](auto& d) {
            vortex_motor_bench::clear_fault_flags(d);
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    constexpr int kSamples = 10;
    for (int i = 0; i < kSamples; ++i) {
        vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kTelemetryPollPeriodMs));
        motors.visitDriver(
            [i](auto& d) {
                char label[24];
                snprintf(label, sizeof(label), "sample %d/%d", i + 1, kSamples);
                vortex_motor_bench::log_telemetry(d, TAG, label);
                vortex_motor_bench::log_fault_flags(d, TAG, "flags");
            },
            MotorController::ONBOARD_TMC9660_INDEX);
    }

    motors.visitDriver(
        [](auto& d) {
            vortex_motor_bench::motor_stop_safe(d, TAG);
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "Telemetry sweep complete");
}
