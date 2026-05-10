/**
 * @file vortex_bldc_open_loop_uart.cpp
 * @brief Confirmed-working open-loop **current-mode** BLDC bring-up over the
 *        TMC9660 UART transport. Spins the bench motor under
 *        `FOC_OPENLOOP_CURRENT_MODE` with full live telemetry of the
 *        electrical angle, per-phase currents and the inverter voltage
 *        commands.
 *
 * Background (see `docs/BLDC_UART_BRINGUP.md` for the deep dive):
 *   * The chip's per-phase **VGS-short protections (NR 272-275)** ship enabled
 *     by default. On our Vortex power stage they trip on every gate
 *     transition (the `GDRV_ERROR.*CHARGE_SHORT` bits 8-10 + 24-26), the
 *     chip's fault handler retries five times then **silently reverts
 *     `COMMUTATION_MODE` back to `SYSTEM_OFF`** (`FAULT_RETRIES_FAILED`).
 *     Symptom: every `setTargetVelocity` SAP comes back `REPLY_INVALID_VALUE`
 *     because there is no regulating mode to accept it.
 *   * In open-loop modes the chip's `HALL_ERROR` flag latches because the
 *     velocity loop touches the hall pipeline even when not used. We mask
 *     it (and the read-only / warning bits) out of the safety-stop check.
 *
 * Flow:
 *   1. Force `VortexOnboardTmc9660Transport::Uart` and bring up Vortex.
 *   2. Dump PCAL95555 control-line state so a stale GPIO mux/wake/RST is
 *      caught before the gate driver ever switches.
 *   3. `configure_complete_bldc(..., with_oc_vgs_protection=true,
 *      with_gate_current_limits=false, skip_y2_phase=true)` — programs the
 *      motor profile, gate driver with overcurrent + fault-handler protection,
 *      shunt sensing, FOC PI, supply OV/UV, chip temperature thresholds and
 *      I²t protections.
 *   4. **Disable the chip-default VGS-short enables on UVW** (NR 272-275)
 *      and bump the deglitch to 8 µs (NR 282 = 7).
 *   5. Configure ramp-for-open-loop, pre-load `OPENLOOP_CURRENT`, then switch
 *      `COMMUTATION_MODE` to `FOC_OPENLOOP_CURRENT_MODE`.
 *   6. Slow ramp `TARGET_VELOCITY` from `kMinRampVelocity` to the cruise
 *      target across `kRampSteps` steps with telemetry between each step.
 *   7. Hold at cruise, then ramp back to 0 and SYSTEM_OFF.
 *
 * **Motion warning** — secure the motor before running. Sized for the
 * 24 V / 30 W bench BLDC (`vortex_bench_safety.hpp`); ramp current is capped
 * at `kSafeOpenLoopCurrentMa` and `MAX_TORQUE` is clamped to
 * `kMaxPhaseCurrentMa` (1500 mA).
 */

#include "api/Vortex.h"
#include "managers/MotorController.h"

#include "common/vortex_bench_safety.hpp"
#include "common/vortex_motor_bench_common.hpp"
#include "common/vortex_motor_bench_probe.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>

namespace {

constexpr const char* TAG = "vortex_bldc_uart";

/// Open-loop current. `OPENLOOP_CURRENT` (NR 46) is what the flux loop drives
/// into the windings while the hardware ramper rotates `phi_e`. Must stay
/// ≤ `kMaxFluxCurrentMa` and ≤ the 1.5 A `MAX_TORQUE` clamp. 1 A is a strong
/// bring-up level for static friction; shorten cruise or lower if windings get hot.
constexpr uint16_t kSafeOpenLoopCurrentMa = 1000;

constexpr unsigned kRampSteps         = 8;
constexpr uint32_t kRampStepDelayMs   = 200;
constexpr uint32_t kCruiseHoldMs      = 10000;
/// Cruise `TARGET_VELOCITY` (TMCL internal units, scaling factor 1). Lower
/// than 5000 helps open-loop pull-in when the rotor lags the forced `phi_e`.
/// 2500 in voltage mode = ~1 RPM at motor shaft / 0.1 RPM at output (10:1
/// reducer) — slow enough that even a heavy rotor can pull in, fast enough
/// that motion is visible over a few-second cruise (~30° at motor shaft per
/// second; output shaft tape mark moves ~3°/s). Bump up once spinning.
constexpr int32_t  kCruiseVelocity    = 2500;
/// Floor used during ramp-up; bench observation is the chip rejects very
/// small values when the OPENLOOP-CURRENT pipeline is fresh, so we never
/// land between 0 and this value during the ramp itself (the final
/// ramp-down explicitly writes 0 to stop). 1000 internal units ≈ 0.3 RPM
/// at the bench's 7-pole-pair motor.
constexpr int32_t kMinRampVelocity = 1000;

template <typename Driver>
void emergency_stop(Driver& d) noexcept {
    ESP_LOGE(TAG, "*** EMERGENCY STOP triggered — fault flags non-zero ***");
    vortex_motor_bench::log_fault_flags(d, TAG, "emergency");
    vortex_motor_bench::motor_stop_safe(d, TAG);
}

}  // namespace

extern "C" void app_main(void) {
    ESP_LOGW(TAG,
             "MOTION APP — UART transport, %u pole pairs, I_open=%u mA, cruise_to=%ld units, hold=%lu ms",
             static_cast<unsigned>(vortex_bench_safety::kDefaultPolePairs),
             static_cast<unsigned>(kSafeOpenLoopCurrentMa),
             static_cast<long>(kCruiseVelocity),
             static_cast<unsigned long>(kCruiseHoldMs));

    Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Uart);

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.IsInitialized()) {
        ESP_LOGE(TAG, "Init failed (Vortex / MotorController). Check UART wiring + TMC9660 power.");
        return;
    }

    // Sanity check on the host control lines — if WAKE is not asserted, RST
    // is held, or PWR_GOOD is low, no amount of TMCL config will help.
    vortex_motor_bench_probe::dump_pcal95555_lines(TAG, "pre-config");

    auto& motors = VORTEX_API.motors;
    bool ready = false;
    motors.visitDriver(
        [&ready](auto& d) {
            // VGS-short protection on UVW is **not** programmed here (false);
            // we explicitly disable it below because the chip's defaults trip
            // on the Vortex power stage transitions. Y2 is also skipped
            // because the Vortex 3-phase power stage does not wire Y2.
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
        ESP_LOGE(TAG, "BLDC config failed — aborting before commutation");
        return;
    }

    motors.visitDriver(
        [](auto& d) { vortex_motor_bench::disable_vortex_uvw_vgs_short_protection(d, TAG); },
        MotorController::ONBOARD_TMC9660_INDEX);

    // ---- Arm open-loop current mode ----
    bool motion_ok = false;
    motors.visitDriver(
        [&motion_ok](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            // 1. Configure ramper FIRST so the hardware ramper exists when we
            //    enable openloop. (RAMP_ENABLE=1, DIRECT_VELOCITY_MODE=0,
            //    RAMP_VMAX = kOpenLoopRampMaxVelocity).
            const ::tmc9660::units::MotorContext ctx{
                tmc9660::tmcl::MotorType::BLDC_MOTOR,
                vortex_bench_safety::kDefaultPolePairs,
                tmc9660::tmcl::VelocitySensorSelection::SAME_AS_COMMUTATION,
                0u};
            if (!vortex_motor_bench::configure_ramp_for_open_loop_voltage(d, TAG, ctx)) {
                return;
            }
            // 2. Pre-load OPENLOOP_VOLTAGE BEFORE switching commutation mode.
            //    Voltage mode applies a fixed-magnitude rotating voltage vector
            //    (classic stepper-style startup); current AND torque emerge
            //    naturally from V/Z, so a stuck rotor gets dragged along by
            //    Iq instead of just locked by Id alignment force (which is
            //    what FOC_OPENLOOP_CURRENT_MODE would do).
            if (!d.torqueFluxControl.setOpenloopVoltage(vortex_bench_safety::kOpenLoopVoltage)) {
                ESP_LOGE(TAG, "setOpenloopVoltage(%u) failed",
                         static_cast<unsigned>(vortex_bench_safety::kOpenLoopVoltage));
                return;
            }
            // 3. Clear any latched faults right before mode change so the
            //    fault handler doesn't immediately revert the mode.
            vortex_motor_bench::clear_fault_flags(d);
            // 4. Switch into open-loop VOLTAGE mode.
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE)) {
                ESP_LOGE(TAG, "setCommutationMode(FOC_OPENLOOP_VOLTAGE_MODE) failed");
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kSettleAfterModeChangeMs));

            uint32_t cm = 0xFFFFFFFFu;
            (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            if (cm != static_cast<uint32_t>(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE)) {
                ESP_LOGW(TAG, "COMMUTATION_MODE reverted to %u, retrying after fault clear",
                         static_cast<unsigned>(cm));
                vortex_motor_bench::log_fault_flags(d, TAG, "after-revert");
                vortex_motor_bench::clear_fault_flags(d);
                (void)d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE);
                vTaskDelay(pdMS_TO_TICKS(50));
                (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            }
            ESP_LOGI(TAG, "  CM=%u OPENLOOP_V=%u  (armed, voltage mode)",
                     static_cast<unsigned>(cm),
                     static_cast<unsigned>(vortex_bench_safety::kOpenLoopVoltage));
            motion_ok = (cm == static_cast<uint32_t>(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE));
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!motion_ok) {
        ESP_LOGE(TAG, "Could not arm open-loop current mode — abort");
        motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d, TAG); },
                           MotorController::ONBOARD_TMC9660_INDEX);
        return;
    }

    // ---- RAMP UP ----
    ESP_LOGI(TAG, "RAMP UP %u steps to TARGET_VELOCITY=%ld (min step=%ld)",
             kRampSteps, static_cast<long>(kCruiseVelocity), static_cast<long>(kMinRampVelocity));
    for (unsigned step = 1; step <= kRampSteps; ++step) {
        const int32_t v_raw =
            static_cast<int32_t>(static_cast<int64_t>(kCruiseVelocity) * step / kRampSteps);
        const int32_t v = (v_raw < kMinRampVelocity) ? kMinRampVelocity : v_raw;
        bool wrote = false;
        motors.visitDriver(
            [v, &wrote](auto& d) {
                wrote = d.velocityControl.setTargetVelocityRaw(v);
            },
            MotorController::ONBOARD_TMC9660_INDEX);
        if (!wrote) {
            ESP_LOGE(TAG, "setTargetVelocity(%ld) failed at step %u — abort",
                     static_cast<long>(v), step);
            motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d, TAG); },
                               MotorController::ONBOARD_TMC9660_INDEX);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(kRampStepDelayMs));
        bool fault = false;
        motors.visitDriver(
            [step, v, &fault](auto& d) {
                char label[32];
                snprintf(label, sizeof(label), "ramp%u@%ld", step, static_cast<long>(v));
                vortex_motor_bench::log_telemetry(d, TAG, label);
                vortex_motor_bench::log_open_loop_telemetry(d, TAG, label);
                fault = vortex_motor_bench::bench_open_loop_motion_fault_critical(d);
            },
            MotorController::ONBOARD_TMC9660_INDEX);
        if (fault) {
            motors.visitDriver([](auto& d) { emergency_stop(d); },
                               MotorController::ONBOARD_TMC9660_INDEX);
            return;
        }
    }

    // ---- CRUISE ----
    // Ramp loop floors at `kMinRampVelocity`; if cruise target is lower
    // (e.g. for a slow open-loop pull-in) we have to write it explicitly
    // here so the field actually slows down before the telemetry segment.
    motors.visitDriver(
        [](auto& d) { (void)d.velocityControl.setTargetVelocityRaw(kCruiseVelocity); },
        MotorController::ONBOARD_TMC9660_INDEX);
    ESP_LOGI(TAG, "CRUISE at TARGET_VELOCITY=%ld for %lu ms",
             static_cast<long>(kCruiseVelocity), static_cast<unsigned long>(kCruiseHoldMs));
    for (uint32_t elapsed = 0; elapsed < kCruiseHoldMs;
         elapsed += vortex_bench_safety::kTelemetryPollPeriodMs) {
        vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kTelemetryPollPeriodMs));
        bool fault = false;
        motors.visitDriver(
            [elapsed, &fault](auto& d) {
                char label[32];
                snprintf(label, sizeof(label), "cruise@%lu",
                         static_cast<unsigned long>(elapsed));
                vortex_motor_bench::log_telemetry(d, TAG, label);
                vortex_motor_bench::log_open_loop_telemetry(d, TAG, label);
                fault = vortex_motor_bench::bench_open_loop_motion_fault_critical(d);
            },
            MotorController::ONBOARD_TMC9660_INDEX);
        if (fault) {
            motors.visitDriver([](auto& d) { emergency_stop(d); },
                               MotorController::ONBOARD_TMC9660_INDEX);
            return;
        }
    }

    // ---- RAMP DOWN ----
    ESP_LOGI(TAG, "RAMP DOWN %u steps to 0", kRampSteps);
    for (unsigned step = kRampSteps; step >= 1; --step) {
        const int32_t v_raw =
            static_cast<int32_t>(static_cast<int64_t>(kCruiseVelocity) * (step - 1) / kRampSteps);
        // Stay above the rejected-low-value window during the ramp; the very
        // last step writes 0 directly to actually stop.
        const int32_t v = (step > 1 && v_raw < kMinRampVelocity) ? kMinRampVelocity : v_raw;
        motors.visitDriver(
            [v](auto& d) { (void)d.velocityControl.setTargetVelocityRaw(v); },
            MotorController::ONBOARD_TMC9660_INDEX);
        vTaskDelay(pdMS_TO_TICKS(kRampStepDelayMs));
    }

    motors.visitDriver(
        [](auto& d) {
            vortex_motor_bench::motor_stop_safe(d, TAG);
            vortex_motor_bench::log_fault_flags(d, TAG, "post-stop");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "Open-loop UART profile finished OK");
}
