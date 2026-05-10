/**
 * @file vortex_bldc_open_loop.cpp
 * @brief Open-loop BLDC spin on the **onboard SPI** TMCL path (PCAL mux enables
 *        `SPI_COMM_EN` — see `Vortex::SetOnboardTmc9660Transport`). Uses the same Vortex power-stage
 *        workaround as `vortex_bldc_open_loop_uart`: disable UVW VGS-short
 *        protection defaults before entering `FOC_OPENLOOP_VOLTAGE_MODE`.
 *
 * For UART-only validation and richer telemetry, prefer
 * `vortex_bldc_open_loop_uart` and [`docs/BLDC_UART_BRINGUP.md`](../docs/BLDC_UART_BRINGUP.md).
 *
 * Sequence:
 *   1. `SetOnboardTmc9660Transport(Spi)` then `VORTEX_API.EnsureInitialized()` + `motors.IsInitialized()`.
 *   2. `configure_complete_bldc(..., with_oc_vgs_protection=true,
 *      with_gate_current_limits=false, skip_y2_phase=true)`.
 *   3. `disable_vortex_uvw_vgs_short_protection()` — see BLDC_UART_BRINGUP.
 *   4. Ramper for open loop, `OPENLOOP_VOLTAGE` **before** commutation mode,
 *      `clear_fault_flags`, `FOC_OPENLOOP_VOLTAGE_MODE`, settle, optional
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
#include "vortex_motor_diagnostics.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_bldc_openloop";

namespace {
/// Short ramp so the first `TARGET_VELOCITY` is not a single large step (mirrors
/// `vortex_bldc_open_loop_uart` behavior; helps after mode change / Erratum 4 scheduling).
constexpr unsigned kVelRampSteps       = 4;
constexpr uint32_t kVelRampStepDelayMs = 120;

/// Motor context used for every RPM ↔ internal-units conversion in this app.
/// Open-loop current mode uses `SAME_AS_COMMUTATION` velocity feedback (the
/// hardware ramper drives the chip's internal angle directly). The chip is
/// authoritative once initialized — `d.getMotorContext()` would return the
/// same values — but for an open-loop bench app we know the topology a
/// priori and want a constexpr context.
inline constexpr ::tmc9660::units::MotorContext kCtx{
    /*motor_type      =*/tmc9660::tmcl::MotorType::BLDC_MOTOR,
    /*pole_pairs      =*/vortex_bench_safety::kDefaultPolePairs,
    /*velocity_sensor =*/tmc9660::tmcl::VelocitySensorSelection::SAME_AS_COMMUTATION,
    /*encoder_cpr     =*/0u,
};
}  // namespace

extern "C" void app_main(void) {
    ESP_LOGW(TAG,
             "MOTION APP — SPI transport, %u pole pairs, I_open=%u mA, target=%.1f RPM, spin %lu ms",
             static_cast<unsigned>(vortex_bench_safety::kDefaultPolePairs),
             static_cast<unsigned>(vortex_bench_safety::kOpenLoopCurrentMa),
             vortex_bench_safety::kOpenLoopTargetRpm,
             static_cast<unsigned long>(vortex_bench_safety::kOpenLoopSpinDurationMs));

    Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Spi);

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.IsInitialized()) {
        ESP_LOGE(TAG, "Init failed (Vortex / MotorController)");
        return;
    }

    vortex_motor_bench_probe::dump_pcal95555_lines(TAG, "pre-config");

    auto& motors = VORTEX_API.motors;

    // 2026-05: bench rig has a 10:1 reducer between the BLDC rotor and the
    // output shaft. Configure the manager so `setLoadVelocity(80 RPM)` spins
    // the motor shaft at 800 RPM (the chip ramp + commutation are always in
    // motor frame). Direction not inverted.
    {
        MotorMechanics mech{};
        mech.gear_ratio_motor_to_output = 10.0;
        mech.invert_output              = false;
        if (!motors.setMotorMechanics(mech, MotorController::ONBOARD_TMC9660_INDEX)) {
            ESP_LOGE(TAG, "setMotorMechanics(10:1) failed — aborting");
            return;
        }
        ESP_LOGI(TAG, "  ✓ MotorMechanics: gear_ratio=10.0 (motor→load), invert=false");
    }

    bool ready = false;
    motors.visitDriver(
        [&ready](auto& d) {
            ready = vortex_motor_bench::configure_complete_bldc(
                d, TAG,
                vortex_bench_safety::kDefaultPolePairs,
                vortex_bench_safety::kPwmFrequencyHz,
                /*do_calibrate=*/true,
                /*enable_outputs=*/true,
                /*with_oc_vgs_protection=*/true,
                /*with_gate_current_limits=*/true,
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

    // 2026-05: switched from FOC_OPENLOOP_CURRENT_MODE → FOC_OPENLOOP_VOLTAGE_MODE
    // because the 30 W bench BLDC was slipping in current mode. In current mode
    // the chip injects pure d-axis flux current; pull-out torque is bounded by
    // K_t · Id at θ_lag = 90° elec. With cogging + bearing friction + accel·J,
    // the rotor falls behind, slips a pole pair, and "stutters" forward at
    // a fraction of the commanded ω. Voltage mode applies a fixed |U| at the
    // rotating phi_e angle: Iq emerges naturally as the rotor lags, giving
    // both d- and q-axis current and significantly higher low-speed pull-out
    // torque. This is the canonical TMC9660 unloaded open-loop test mode.
    bool motion_ok = false;
    motors.visitDriver(
        [&motion_ok](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            if (!vortex_motor_bench::configure_ramp_for_open_loop_voltage(d, TAG, kCtx)) {
                return;
            }
            if (!d.torqueFluxControl.setOpenloopVoltage(vortex_bench_safety::kOpenLoopVoltage)) {
                ESP_LOGE(TAG, "setOpenloopVoltage(%u) failed",
                         static_cast<unsigned>(vortex_bench_safety::kOpenLoopVoltage));
                return;
            }
            vortex_motor_bench::clear_fault_flags(d);
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE)) {
                ESP_LOGE(TAG, "setCommutationMode(FOC_OPENLOOP_VOLTAGE_MODE) failed");
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kSettleAfterModeChangeMs));

            uint32_t cm = 0xFFFFFFFFu;
            (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            if (cm != static_cast<uint32_t>(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE)) {
                ESP_LOGW(TAG, "COMMUTATION_MODE=%u (expected 3), retry after fault clear",
                         static_cast<unsigned>(cm));
                vortex_motor_bench::log_fault_flags(d, TAG, "after-revert");
                vortex_motor_bench::clear_fault_flags(d);
                (void)d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE);
                vTaskDelay(pdMS_TO_TICKS(50));
                (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            }
            motion_ok = (cm == static_cast<uint32_t>(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE));
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    if (!motion_ok) {
        ESP_LOGE(TAG, "Could not arm open-loop current mode — abort");
        motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d, TAG); },
                           MotorController::ONBOARD_TMC9660_INDEX);
        return;
    }

    // Drive the manual 4-step ramp via the manager's **load-frame** API
    // (`setLoadVelocity`). With the default 1:1 MotorMechanics this is
    // identical to commanding the motor frame, but exercises the gearbox
    // mapping path so changing `setMotorMechanics(...)` later is the only
    // edit needed for a geared bench rig.
    using ::tmc9660::units::VelocityUnit;
    const double load_target_rpm = vortex_bench_safety::kOpenLoopTargetRpm;
    const double load_floor_rpm  = vortex_bench_safety::kOpenLoopMinCommandRpm;
    for (unsigned step = 1; step <= kVelRampSteps; ++step) {
        double v_rpm = load_target_rpm * static_cast<double>(step) /
                       static_cast<double>(kVelRampSteps);
        if (v_rpm < load_floor_rpm) v_rpm = load_floor_rpm;
        if (!motors.setLoadVelocity(v_rpm, VelocityUnit::Rpm, kCtx,
                                    MotorController::ONBOARD_TMC9660_INDEX)) {
            ESP_LOGE(TAG, "setLoadVelocity(%.2f RPM) failed at ramp step %u/%u",
                     v_rpm, step, kVelRampSteps);
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
                     "post-velocity: COMMUTATION_MODE=%u (expect 3) TARGET_VELOCITY_rb=%ld "
                     "(Erratum 4 delay applied)",
                     static_cast<unsigned>(cm), static_cast<long>(static_cast<int32_t>(tv)));
            if (cm != static_cast<uint32_t>(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE)) {
                ESP_LOGW(TAG,
                         "Commutation mode reverted — VGS SAP writes may have failed; check "
                         "\"VGS UVW short protection\" log line");
            }
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    const uint32_t end_ms = vortex_bench_safety::kOpenLoopSpinDurationMs;
    const uint32_t step_ms = vortex_bench_safety::kTelemetryPollPeriodMs;
    // Diagnostic expectation: motor should be regulating at the commanded
    // RPM (positive, ±10 RPM tolerance once the ramp has had time to
    // complete), in FOC_OPENLOOP_VOLTAGE_MODE, with no gate-driver faults
    // and chip temperature under the safe ceiling. Velocity tolerance is
    // intentionally loose at the start so the diagnostic doesn't fail
    // during the ramp itself.
    vortex_motor_diagnostics::MotorExpectation exp_during_ramp{};
    exp_during_ramp.check_commutation_mode    = true;
    exp_during_ramp.expected_commutation_mode = tmc9660::tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE;
    exp_during_ramp.check_no_gate_driver_errors = true;
    exp_during_ramp.check_chip_temperature      = true;
    exp_during_ramp.chip_temp_max_c             = vortex_bench_safety::kChipShutdownTempC;
    exp_during_ramp.check_bus_voltage           = true;
    exp_during_ramp.vbus_min_v                  = 10.0f;
    exp_during_ramp.vbus_max_v                  = 30.0f;

    vortex_motor_diagnostics::MotorExpectation exp_at_cruise = exp_during_ramp;
    exp_at_cruise.check_velocity            = true;
    // The chip's `actual_velocity` is in **motor-frame** (under SAME_AS_COMMUTATION
    // it echoes the commanded phi_e ramper, which lives in the motor frame).
    // Our load-frame target × gear ratio = motor-frame target.
    exp_at_cruise.expected_velocity_rpm     = vortex_bench_safety::kOpenLoopTargetRpm * 10.0;
    exp_at_cruise.velocity_tolerance_rpm    = 20.0;
    exp_at_cruise.check_velocity_direction  = true;
    exp_at_cruise.check_current             = true;
    exp_at_cruise.max_motor_current_ma      = vortex_bench_safety::kMaxPhaseCurrentMa;

    // Ramp completes in ~(motor_target_rpm / kOpenLoopRampMaxRpmPerSec) seconds;
    // motor_target_rpm = load_target × gear_ratio (10). Give 50 % margin.
    const uint32_t cruise_threshold_ms = static_cast<uint32_t>(
        1500.0 * (vortex_bench_safety::kOpenLoopTargetRpm * 10.0) /
        vortex_bench_safety::kOpenLoopRampMaxRpmPerSec);

    for (uint32_t elapsed = 0; elapsed < end_ms; elapsed += step_ms) {
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        const auto& exp = (elapsed >= cruise_threshold_ms) ? exp_at_cruise : exp_during_ramp;
        motors.visitDriver(
            [elapsed, &exp](auto& d) {
                char label[24];
                snprintf(label, sizeof(label), "spin@%lu", static_cast<unsigned long>(elapsed));
                vortex_motor_bench::log_telemetry(d, TAG, label);
                vortex_motor_bench::log_open_loop_telemetry(d, TAG, label);
                (void)vortex_motor_diagnostics::diagnose_and_log(d, kCtx, exp, TAG, label);
            },
            MotorController::ONBOARD_TMC9660_INDEX);
    }

    motors.visitDriver(
        [](auto& d) {
            vortex_motor_bench::motor_stop_safe(d, TAG);
            vortex_motor_bench::log_fault_flags(d, TAG, "post-stop");
            // After stop, expect SYSTEM_OFF with zero velocity and no faults.
            vortex_motor_diagnostics::MotorExpectation exp_post_stop{};
            exp_post_stop.check_commutation_mode      = true;
            exp_post_stop.expected_commutation_mode   = tmc9660::tmcl::CommutationMode::SYSTEM_OFF;
            exp_post_stop.check_velocity              = true;
            exp_post_stop.expected_velocity_rpm       = 0.0;
            exp_post_stop.velocity_tolerance_rpm      = 5.0;
            exp_post_stop.check_velocity_direction    = false;
            exp_post_stop.check_no_gate_driver_errors = true;
            (void)vortex_motor_diagnostics::diagnose_and_log(d, kCtx, exp_post_stop, TAG, "post-stop");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "Open-loop profile finished OK");
}
