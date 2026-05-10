/**
 * @file vortex_bldc_probe.cpp
 * @brief Comprehensive read/write probe for the Vortex V1 + TMC9660 BLDC stack.
 *
 * Goal: tell us *exactly* why a motor command fails on this firmware image.
 *
 * Sequence:
 *   1. Vortex + MotorController bring-up.
 *   2. Snapshot PCAL95555 control lines BEFORE we configure the chip — we want
 *      to see RST_CTRL / DRV_EN / SPI_COMM / WAKE / FAULT / PWR_GOOD raw bits.
 *   3. Run `configure_complete_bldc()` (motor → gate driver → current sense →
 *      FOC PI → protection → ramp → brake → heartbeat → power → ADC offset).
 *   4. Snapshot PCAL95555 again so we can confirm DRV_EN bit is HIGH.
 *   5. Read back every relevant SAP parameter via GAP and decode flags.
 *   6. Run a static open-loop probe: FOC_OPENLOOP_VOLTAGE_MODE with
 *      OPENLOOP_VOLTAGE only (no TARGET_VELOCITY) — we should see ADC currents
 *      bias + a fixed phi_e if the gate driver actually switches.
 *   7. Run the TARGET_VELOCITY matrix in voltage mode (positive/negative,
 *      ramp on/off, DVM on/off) and log accept/reject per shot plus the
 *      readbacks of TARGET_VELOCITY/RAMP_VELOCITY/ACTUAL_VELOCITY/OPENLOOP_ANGLE.
 *   8. Repeat the matrix in current mode.
 *   9. Stop, dump post-stop status, deassert DRV_EN.
 *
 * Nothing here is guaranteed to spin the motor — the matrix is designed to
 * pinpoint *which* combination this firmware accepts so we can lock the bench
 * helpers to that combination next.
 */
#include "api/Vortex.h"
#include "managers/GpioManager.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"
#include "vortex_motor_bench_probe.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_bldc_probe";

extern "C" void app_main(void) {
    ESP_LOGW(TAG,
             "PROBE APP — read-only diagnostics + bounded TARGET_VELOCITY matrix on "
             "Vortex/TMC9660 (24V class, %u pole pairs)",
             static_cast<unsigned>(vortex_bench_safety::kDefaultPolePairs));

    // -----------------------------------------------------------------------
    // 1. Vortex + MotorController bring-up.
    // -----------------------------------------------------------------------
    if (!VORTEX_API.EnsureInitialized()) {
        ESP_LOGE(TAG, "Vortex EnsureInitialized() failed");
        return;
    }
    if (!VORTEX_API.motors.EnsureInitialized()) {
        ESP_LOGE(TAG, "MotorController EnsureInitialized() failed");
        return;
    }

    // -----------------------------------------------------------------------
    // 2. PCAL95555 snapshot BEFORE chip configuration (boot-default state).
    // -----------------------------------------------------------------------
    vortex_motor_bench_probe::dump_pcal95555_lines(TAG, "boot");

    auto& motors = VORTEX_API.motors;

    // -----------------------------------------------------------------------
    // 3. Run the full EVKit-style BLDC bring-up (no commutation mode change).
    //    `configure_complete_bldc` ends in SYSTEM_OFF with DRV_EN asserted.
    // -----------------------------------------------------------------------
    bool ready = false;
    motors.visitDriver(
        [&ready](auto& d) {
            ready = vortex_motor_bench::configure_complete_bldc(d, TAG);
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!ready) {
        ESP_LOGE(TAG, "configure_complete_bldc() failed — aborting probe");
        motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d, TAG); },
                           MotorController::ONBOARD_TMC9660_INDEX);
        return;
    }

    // -----------------------------------------------------------------------
    // 4. PCAL95555 snapshot AFTER bring-up — DRV_EN bit should now be 1.
    // -----------------------------------------------------------------------
    vortex_motor_bench_probe::dump_pcal95555_lines(TAG, "post-bringup");

    // -----------------------------------------------------------------------
    // 5. Full parameter dump + decoded fault flags (chip armed, SYSTEM_OFF).
    // -----------------------------------------------------------------------
    motors.visitDriver(
        [](auto& d) {
            vortex_motor_bench_probe::dump_params(d, TAG, "armed-systemoff");
            vortex_motor_bench_probe::dump_status_decoded(d, TAG, "armed-systemoff");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    // -----------------------------------------------------------------------
    // 6. Static open-loop probe in VOLTAGE mode — see if ADC currents change.
    // -----------------------------------------------------------------------
    motors.visitDriver(
        [](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            ESP_LOGI(TAG, "==== Static open-loop voltage probe (no TARGET_VELOCITY) ====");
            vortex_motor_bench::clear_fault_flags(d);
            // Re-enable ramp generator like the spin app does, even though we
            // won't ask for motion: the chip uses the ramper to advance phi_e.
            const ::tmc9660::units::MotorContext ctx{
                tmc9660::tmcl::MotorType::BLDC_MOTOR,
                vortex_bench_safety::kDefaultPolePairs,
                tmc9660::tmcl::VelocitySensorSelection::SAME_AS_COMMUTATION,
                0u};
            (void)vortex_motor_bench::configure_ramp_for_open_loop_voltage(d, TAG, ctx);

            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE)) {
                ESP_LOGE(TAG, "setCommutationMode(FOC_OPENLOOP_VOLTAGE_MODE) failed");
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kSettleAfterModeChangeMs));
            vortex_motor_bench_probe::probe_static_openloop(d, TAG,
                                                            vortex_bench_safety::kOpenLoopVoltage,
                                                            vortex_bench_safety::kOpenLoopCurrentMa,
                                                            /*hold_ms=*/600);
            vortex_motor_bench_probe::dump_status_decoded(d, TAG, "after-static-V");

            // ----------------------------------------------------------------
            // 6b. OPENLOOP_ANGLE writability + software commutator experiment.
            //     If this firmware accepts SAP 45 we can drive PHI_E directly
            //     and bypass the broken TARGET_VELOCITY path entirely.
            // ----------------------------------------------------------------
            const bool angle_writable =
                vortex_motor_bench_probe::probe_openloop_angle_writability(d, TAG);
            if (angle_writable) {
                ESP_LOGW(TAG, "OPENLOOP_ANGLE is WRITABLE on 051V100 — running 4 s software commutator spin");
                // step=256 / dt=5ms => 256/32768 of a turn per ms, so one electrical
                // revolution per ~640 ms; with 7 pole pairs that's ~13 mech rpm at
                // first, plenty slow to confirm rotor sync.
                vortex_motor_bench_probe::software_commutator_spin(
                    d, TAG, /*step=*/256, /*dt_ms=*/5, /*spin_ms=*/4000);
                vortex_motor_bench_probe::dump_status_decoded(d, TAG, "after-sw-commutator");
            } else {
                ESP_LOGW(TAG, "OPENLOOP_ANGLE not writable on this firmware — register mode is the only path");
            }
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    // -----------------------------------------------------------------------
    // 7. TARGET_VELOCITY matrix — VOLTAGE mode.
    // -----------------------------------------------------------------------
    motors.visitDriver(
        [](auto& d) {
            ESP_LOGI(TAG, "==== TARGET_VELOCITY matrix in FOC_OPENLOOP_VOLTAGE_MODE ====");
            vortex_motor_bench::clear_fault_flags(d);
            vortex_motor_bench_probe::exercise_target_velocity_matrix(d, TAG, "openloop-V");
            vortex_motor_bench_probe::dump_status_decoded(d, TAG, "after-matrix-V");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    // -----------------------------------------------------------------------
    // 8. TARGET_VELOCITY matrix — CURRENT mode.
    // -----------------------------------------------------------------------
    motors.visitDriver(
        [](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            ESP_LOGI(TAG, "==== Switching to FOC_OPENLOOP_CURRENT_MODE for current-mode matrix ====");
            (void)d.velocityControl.setTargetVelocityRaw(0);
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_CURRENT_MODE)) {
                ESP_LOGE(TAG, "setCommutationMode(FOC_OPENLOOP_CURRENT_MODE) failed");
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kSettleAfterModeChangeMs));
            (void)d.torqueFluxControl.setOpenloopCurrent(vortex_bench_safety::kOpenLoopCurrentMa);
            vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kErratum4TargetCoalesceMs));
            vortex_motor_bench::clear_fault_flags(d);
            vortex_motor_bench_probe::exercise_target_velocity_matrix(d, TAG, "openloop-I",
                                                                      /*prime_openloop_voltage=*/false);
            vortex_motor_bench_probe::dump_status_decoded(d, TAG, "after-matrix-I");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    // -----------------------------------------------------------------------
    // 9. Stop, post-stop dump, deassert DRV_EN.
    // -----------------------------------------------------------------------
    motors.visitDriver(
        [](auto& d) {
            ESP_LOGI(TAG, "==== Stop + final telemetry + status ====");
            vortex_motor_bench::motor_stop_safe(d, TAG);
            vortex_motor_bench_probe::dump_status_decoded(d, TAG, "post-stop");
            vortex_motor_bench::log_telemetry(d, TAG, "post-stop");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    vortex_motor_bench_probe::dump_pcal95555_lines(TAG, "post-stop");

    ESP_LOGI(TAG, "Probe complete. Use the matrix above to identify which prerequisites the "
                  "Vortex firmware accepts for TARGET_VELOCITY.");
}
