/**
 * @file vortex_bldc_dynamics.cpp
 * @brief Closed-loop hall-FOC dynamics test for a robot-dog-class BLDC actuator.
 *        Same multi-phase trajectory as before, but with `FOC_HALL_SENSOR`
 *        commutation so the chip can hold synchronism through the aggressive
 *        ramps that pole-slipped under open-loop voltage mode.
 *
 *        Phase 1 — **Staircase sweep up** (load 50→300 RPM, 50 RPM/step).
 *        Phase 2 — **Hold at peak** 300 load-RPM.
 *        Phase 3 — **Symmetric sweep down** 300→0 RPM.
 *        Phase 4 — **Trapezoidal swing-leg** ×3: 0→200→0 @ 300 load-RPM/s,
 *                  ≈1 Hz cadence (typical robot-dog hip/knee gait).
 *        Phase 5 — **Burst sprint** 0→300 @ 500 load-RPM/s — jump-launch /
 *                  impact-recovery class of motion.
 *        Phase 6 — **Reversal stress** ±100 load-RPM @ 150 load-RPM/s.
 *        Phase 7 — Wind-down, motor_stop_safe.
 *
 * **Bring-up sequence** (mirrors `vortex_bldc_velocity_foc_hall.cpp`):
 *   1. SPI transport + Vortex/MotorController init + 10:1 motor mechanics.
 *   2. `configure_complete_bldc()` (gate, current sense, FOC PI, protection,
 *      ramp, ADC cal, DRV_EN).
 *   3. `configure_hall(d)` — sector DEG_0, ideal 60° spacing. Requires the
 *      bootloader hall-pin mux on GPIO2/3/4 (EvKit default).
 *   4. `disable_vortex_uvw_vgs_short_protection()` workaround.
 *   5. Configure ramp ONCE at peak (3300 motor-RPM, 5500 motor-RPM/s) — host
 *      shapes per-tick trajectory; the chip ramper is left armed in Auto mode
 *      and never re-tuned mid-run (avoids the chip stop-event bug we hit on
 *      the open-loop variant).
 *   6. `setCommutationMode(FOC_HALL_SENSOR)` with retry-on-revert.
 *   7. Walk the profile at 50 ms tick (20 Hz host trajectory generator).
 *
 * **Why this should hold lock at speeds where open-loop slipped**: the chip
 * now reads phi_e from the halls every commutation cycle, so torque is
 * always applied along the q-axis regardless of rotor lag. The torque PI
 * automatically scales |U| to whatever overcomes back-EMF + load, capped
 * by `MAX_TORQUE` / I²t / hardware OC. No host-side voltage scaling
 * needed.
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

#include <algorithm>
#include <cmath>
#include <cstdint>

static const char* TAG = "vortex_bldc_dyn";

namespace {

constexpr double kGearRatioMotorToOutput = 10.0;

inline constexpr ::tmc9660::units::MotorContext kCtx{
    /*motor_type      =*/tmc9660::tmcl::MotorType::BLDC_MOTOR,
    /*pole_pairs      =*/vortex_bench_safety::kDefaultPolePairs,
    /*velocity_sensor =*/tmc9660::tmcl::VelocitySensorSelection::DIGITAL_HALL,
    /*encoder_cpr     =*/0u,
};

// ---- Profile descriptors ---------------------------------------------------
struct Segment {
    const char* name;
    double      load_rpm;            ///< Signed target velocity at the load shaft (RPM)
    double      ramp_load_rpm_per_s; ///< |Δv|/dt during the ramp portion (load frame)
    uint32_t    hold_ms;             ///< Plateau time after the ramp completes
};

const Segment kProfile[] = {
    // Closed-loop hall FOC: ramps can be aggressive — the chip's torque PI
    // applies whatever |U| the back-EMF + load demands, capped only by
    // MAX_TORQUE / I²t / hardware OC.
    // ---- Phase 1: staircase sweep up, find tracking-error point if any ----
    {"P1.50",   50.0,   50.0, 1500},
    {"P1.100", 100.0,   50.0, 1500},
    {"P1.150", 150.0,   50.0, 1500},
    {"P1.200", 200.0,   50.0, 1500},
    {"P1.250", 250.0,   50.0, 1500},
    {"P1.300", 300.0,   50.0, 1500},
    // ---- Phase 2: hold at peak ----
    {"P2.hold", 300.0,  50.0, 2500},
    // ---- Phase 3: sweep down ----
    {"P3.250", 250.0,   50.0, 1000},
    {"P3.150", 150.0,   50.0, 1000},
    {"P3.50",   50.0,   50.0, 1000},
    {"P3.0",     0.0,   50.0,  500},
    // ---- Phase 4: trapezoidal swing-leg, 3 cycles, ~1 Hz cadence ----
    {"P4.up.1",  200.0, 300.0,  400},
    {"P4.dn.1",    0.0, 300.0,  200},
    {"P4.up.2",  200.0, 300.0,  400},
    {"P4.dn.2",    0.0, 300.0,  200},
    {"P4.up.3",  200.0, 300.0,  400},
    {"P4.dn.3",    0.0, 300.0,  300},
    // ---- Phase 5: burst sprint to peak, hold, decel ----
    {"P5.burst", 300.0, 500.0, 1500},
    {"P5.brake",   0.0, 500.0,  500},
    // ---- Phase 6: reversal stress ----
    {"P6.fwd.1", 100.0, 150.0, 1000},
    {"P6.rev.1",-100.0, 150.0, 1000},
    {"P6.fwd.2", 100.0, 150.0, 1000},
    {"P6.rev.2",-100.0, 150.0, 1000},
    // ---- Phase 7: wind-down ----
    {"P7.stop",    0.0,  50.0, 1000},
};

inline vortex_motor_diagnostics::MotorExpectation make_expectation(
    double commanded_motor_rpm, bool check_velocity) noexcept {
    using namespace tmc9660::tmcl;
    vortex_motor_diagnostics::MotorExpectation exp{};
    exp.check_commutation_mode      = true;
    exp.expected_commutation_mode   = CommutationMode::FOC_HALL_SENSOR;
    exp.check_no_gate_driver_errors = true;
    exp.check_chip_temperature      = true;
    exp.chip_temp_max_c             = vortex_bench_safety::kChipShutdownTempC;
    exp.check_bus_voltage           = true;
    exp.vbus_min_v                  = 10.0f;
    exp.vbus_max_v                  = 30.0f;
    exp.check_velocity              = check_velocity;
    exp.expected_velocity_rpm       = commanded_motor_rpm;
    exp.velocity_tolerance_rpm      = 100.0;     // motor-frame, ±10 RPM at the load
    exp.check_velocity_direction    = false;     // signs flip during reversal phase
    exp.check_current               = true;
    exp.max_motor_current_ma        = vortex_bench_safety::kMaxPhaseCurrentMa;
    return exp;
}

}  // namespace

extern "C" void app_main(void) {
    ESP_LOGW(TAG,
             "MOTION APP — DYNAMICS hall-FOC, %u pole pairs, gear=%.1f:1",
             static_cast<unsigned>(vortex_bench_safety::kDefaultPolePairs),
             kGearRatioMotorToOutput);
    ESP_LOGW(TAG,
             "             max load=300 RPM, %zu segments, host trajectory shaping @ 20 Hz. "
             "Secure the rotor and gearbox output.",
             sizeof(kProfile) / sizeof(kProfile[0]));

    Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Spi);
    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.IsInitialized()) {
        ESP_LOGE(TAG, "Init failed (Vortex / MotorController)");
        return;
    }

    auto& motors = VORTEX_API.motors;

    {
        MotorMechanics mech{};
        mech.gear_ratio_motor_to_output = kGearRatioMotorToOutput;
        mech.invert_output              = false;
        if (!motors.setMotorMechanics(mech, MotorController::ONBOARD_TMC9660_INDEX)) {
            ESP_LOGE(TAG, "setMotorMechanics(%.1f:1) failed — aborting", kGearRatioMotorToOutput);
            return;
        }
        ESP_LOGI(TAG, "  ✓ MotorMechanics: gear_ratio=%.1f (motor→load), invert=false",
                 kGearRatioMotorToOutput);
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
        ESP_LOGE(TAG, "BLDC config failed — aborting before commutation engage");
        return;
    }

    motors.visitDriver(
        [](auto& d) { vortex_motor_bench::disable_vortex_uvw_vgs_short_protection(d, TAG); },
        MotorController::ONBOARD_TMC9660_INDEX);

    // Hall feedback configuration. Requires bootloader hall pin mux on
    // GPIO2/3/4 (EvKit default — Tmc9660Handler::kDefaultBootConfig).
    bool hall_ok = false;
    motors.visitDriver(
        [&hall_ok](auto& d) { hall_ok = vortex_motor_bench::configure_hall(d, TAG); },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!hall_ok) {
        ESP_LOGE(TAG, "Hall sensor config failed — aborting before commutation engage");
        return;
    }

    // Configure the chip ramper ONCE at peak limits — see file-header design note.
    constexpr double kPeakMotorRpm        = 3300.0;
    constexpr double kPeakMotorRpmPerSec  = 5500.0;
    bool armed = false;
    motors.visitDriver(
        [&armed](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            using ::tmc9660::units::VelocityUnit;
            using ::tmc9660::units::AccelerationUnit;
            using DriverT = std::decay_t<decltype(d)>;

            auto rc = DriverT::Ramp::buildRampConfig(
                kPeakMotorRpm,       VelocityUnit::Rpm,
                kPeakMotorRpmPerSec, AccelerationUnit::RpmPerSec,
                kCtx);
            rc.enableRamp               = true;
            rc.enableDirectVelocityMode = false;
            if (!d.ramp.configureAuto(rc)) {
                ESP_LOGE(TAG, "Initial ramp.configureAuto failed");
                return;
            }
            ESP_LOGI(TAG,
                     "  ✓ Ramp armed: VMAX=%.0f motor-RPM (%.0f load-RPM), "
                     "AMAX=%.0f motor-RPM/s (%.0f load-RPM/s) — fixed for run",
                     kPeakMotorRpm, kPeakMotorRpm / kGearRatioMotorToOutput,
                     kPeakMotorRpmPerSec, kPeakMotorRpmPerSec / kGearRatioMotorToOutput);

            vortex_motor_bench::clear_fault_flags(d);
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_HALL_SENSOR)) {
                ESP_LOGE(TAG, "setCommutationMode(FOC_HALL_SENSOR) failed");
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kSettleAfterModeChangeMs));

            uint32_t cm = 0xFFFFFFFFu;
            (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            if (cm != static_cast<uint32_t>(tmcl::CommutationMode::FOC_HALL_SENSOR)) {
                ESP_LOGW(TAG, "COMMUTATION_MODE=%u after arm (expected FOC_HALL=6) — retrying",
                         static_cast<unsigned>(cm));
                vortex_motor_bench::log_fault_flags(d, TAG, "after-revert");
                vortex_motor_bench::clear_fault_flags(d);
                (void)d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_HALL_SENSOR);
                vTaskDelay(pdMS_TO_TICKS(50));
                (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            }
            armed = (cm == static_cast<uint32_t>(tmcl::CommutationMode::FOC_HALL_SENSOR));
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    if (!armed) {
        ESP_LOGE(TAG, "Could not arm FOC_HALL_SENSOR — abort");
        motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d, TAG); },
                           MotorController::ONBOARD_TMC9660_INDEX);
        return;
    }

    // Walk the profile with host-side trajectory shaping.
    using ::tmc9660::units::VelocityUnit;
    constexpr uint32_t kTickMs = 50;          // 20 Hz host trajectory tick
    // Telemetry is expensive (~1 KB per snapshot @ 115200 baud ≈ 90 ms). Log
    // only every Nth tick so the host loop keeps up with the planned tick
    // rate; lightweight light_telemetry on the others gives a fast
    // single-line heartbeat.
    constexpr uint32_t kRampLogEvery = 5;     // ramp: full snapshot every 250 ms
    constexpr uint32_t kHoldLogEvery = 3;     // hold: full snapshot every 150 ms

    double prev_load_rpm = 0.0;

    for (const Segment& seg : kProfile) {
        const double dv_load    = seg.load_rpm - prev_load_rpm;
        const double rate_load  = std::max(1.0, std::fabs(seg.ramp_load_rpm_per_s));
        const double ramp_secs  = std::fabs(dv_load) / rate_load;
        const uint32_t ramp_ms  = static_cast<uint32_t>(std::ceil(ramp_secs * 1000.0));

        ESP_LOGI(TAG,
                 "[%s] cmd: prev=%+.1f -> tgt=%+.1f load-RPM (motor=%+.0f) "
                 "rate=%.0f load-RPM/s (%.0f motor-RPM/s) ramp=%lu ms hold=%lu ms",
                 seg.name,
                 prev_load_rpm, seg.load_rpm,
                 seg.load_rpm * kGearRatioMotorToOutput,
                 rate_load, rate_load * kGearRatioMotorToOutput,
                 static_cast<unsigned long>(ramp_ms),
                 static_cast<unsigned long>(seg.hold_ms));

        // ---- Ramp portion: linearly interpolate load-frame setpoint each tick.
        //                    Closed-loop FOC reads phi_e from halls every
        //                    commutation cycle, so no host-side voltage
        //                    shaping is needed — the torque PI scales |U|
        //                    automatically.
        bool segment_ok = true;
        if (ramp_ms > 0) {
            const uint32_t n_ticks = std::max<uint32_t>(1u, ramp_ms / kTickMs);
            for (uint32_t i = 1; i <= n_ticks; ++i) {
                const double frac = static_cast<double>(i) / static_cast<double>(n_ticks);
                const double v_load = prev_load_rpm + dv_load * frac;
                if (!motors.setLoadVelocity(v_load, VelocityUnit::Rpm, kCtx,
                                            MotorController::ONBOARD_TMC9660_INDEX)) {
                    ESP_LOGE(TAG, "[%s] setLoadVelocity(%.2f) failed", seg.name, v_load);
                    segment_ok = false;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(kTickMs));
                const bool full_log = (i % kRampLogEvery == 0) || (i == n_ticks);
                const auto exp = make_expectation(v_load * kGearRatioMotorToOutput,
                                                  /*check_velocity=*/false);
                motors.visitDriver(
                    [&seg, i, &exp, full_log](auto& d) {
                        char label[40];
                        snprintf(label, sizeof(label), "%s.r%lu", seg.name,
                                 static_cast<unsigned long>(i));
                        vortex_motor_bench::log_telemetry(d, TAG, label);
                        if (full_log) {
                            (void)vortex_motor_diagnostics::diagnose_and_log(
                                d, kCtx, exp, TAG, label);
                        }
                    },
                    MotorController::ONBOARD_TMC9660_INDEX);
            }
        } else {
            if (!motors.setLoadVelocity(seg.load_rpm, VelocityUnit::Rpm, kCtx,
                                        MotorController::ONBOARD_TMC9660_INDEX)) {
                ESP_LOGE(TAG, "[%s] setLoadVelocity step failed", seg.name);
                segment_ok = false;
            }
        }

        if (!segment_ok) break;

        // ---- Hold portion: enforce velocity check ----
        const auto exp_hold = make_expectation(seg.load_rpm * kGearRatioMotorToOutput,
                                               /*check_velocity=*/true);
        uint32_t hold_idx = 0;
        for (uint32_t t = 0; t < seg.hold_ms; t += kTickMs) {
            vTaskDelay(pdMS_TO_TICKS(kTickMs));
            const bool full_log = (hold_idx % kHoldLogEvery == 0);
            ++hold_idx;
            motors.visitDriver(
                [&seg, t, &exp_hold, full_log](auto& d) {
                    char label[40];
                    snprintf(label, sizeof(label), "%s.h%lu", seg.name,
                             static_cast<unsigned long>(t));
                    vortex_motor_bench::log_telemetry(d, TAG, label);
                    if (full_log) {
                        (void)vortex_motor_diagnostics::diagnose_and_log(
                            d, kCtx, exp_hold, TAG, label);
                    }
                },
                MotorController::ONBOARD_TMC9660_INDEX);
        }

        prev_load_rpm = seg.load_rpm;
    }

    motors.visitDriver(
        [](auto& d) {
            vortex_motor_bench::motor_stop_safe(d, TAG);
            vortex_motor_bench::log_fault_flags(d, TAG, "post-stop");
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

    ESP_LOGI(TAG, "Dynamics profile finished OK");
}
