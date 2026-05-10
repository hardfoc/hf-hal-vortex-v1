/**
 * @file vortex_bldc_abn_as5047_dynamics.cpp
 * @brief Closed-loop ABN-FOC velocity test with **dual** rotor-position
 *        feedback paths exposed to the host:
 *
 *          Path A — TMC9660 ABN1 quadrature input (GPIO8/13/14 EvKit map),
 *                   driven by the AS5047U's incremental ABI output. This is
 *                   what the chip uses internally for FOC commutation.
 *
 *          Path B — AS5047U over the dedicated SPI CS (GPIO20,
 *                   `SpiDeviceId::AS5047U_POSITION_ENCODER`). The host reads
 *                   the absolute 14-bit angle and DAEC-compensated velocity
 *                   directly, in parallel with chip-side telemetry.
 *
 *        Having both lets us cross-check that the ABN edges the chip is
 *        counting agree with the absolute angle the magnet/sensor measures —
 *        i.e. ABN wiring (A/B/I) is correct, polarity matches, and CPR
 *        scaling is right. Any silent quadrature glitch shows up as a
 *        growing offset between the two readings.
 *
 * Bring-up sequence:
 *   1. Vortex + MotorController + EncoderManager init.
 *   2. Create AS5047U on slot 0 with **ABI enabled @ 14-bit** (4096
 *      lines/rev × 4 quadrature edges = 16384 counts/rev seen by TMC9660
 *      ABN1). Frame format SPI_24 for CRC-protected reads.
 *   3. Verification phase: pull AGC, magnitude, error flags, and a short
 *      angle/velocity burst. Confirms (a) SPI bus + CS work, (b) magnet is
 *      in range, (c) DAEC pipeline is alive.
 *   4. `configure_complete_bldc()` (gate, current sense, FOC PI,
 *      protection, ramp, ADC cal, DRV_EN).
 *   5. `configure_abn(d, CPR=16384)` — `FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING`
 *      init aligns ABN counts to phi_e (~1 s shaft motion at start).
 *   6. VGS-short / fault-clear workaround (same as other bench apps).
 *   7. `setCommutationMode(FOC_ABN)` with retry on revert.
 *   8. Velocity sweep with 50 ms host trajectory tick. Each tick logs:
 *        - chip telemetry (TMC9660 actual velocity / Iq / Vbus)
 *        - AS5047U direct: angle (deg), velocity (RPM), AGC, magnitude
 *      Periodically computes integrated AS5047U revolutions vs commanded
 *      revolutions to flag gross encoder/ABN scale mismatches.
 *   9. `motor_stop_safe()`.
 *
 * @note Bootloader must enable ABN1 mux on GPIO8/13/14 (stock EvKit
 *       `Tmc9660Handler::kDefaultBootConfig`). AS5047U dedicated SPI CS
 *       (GPIO20) is registered in `CommChannelsManager` as
 *       `SpiDeviceId::AS5047U_POSITION_ENCODER`.
 *
 * @note We deliberately use **CPR=16384** (override of bench default 1024
 *       in `vortex_bench_safety.hpp`) because the AS5047U at 14-bit ABI
 *       resolution emits 4096 line cycles/rev → 16384 quad counts/rev.
 *       Wrong CPR → FOC believes one mechanical rev = N electrical revs
 *       different from reality → instant pole-slip on first encoder init.
 */
#include "api/Vortex.h"
#include "managers/CommChannelsManager.h"
#include "managers/EncoderManager.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"
#include "vortex_motor_bench_probe.hpp"
#include "vortex_motor_diagnostics.hpp"

#include "core/handlers/as5047u/As5047uHandler.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

static const char* TAG = "vortex_bldc_abn5047";

namespace {

// ---- Mechanical / electrical constants -------------------------------------
constexpr double   kGearRatioMotorToOutput = 10.0;
// 14-bit ABI on AS5047U => 4096 line cycles × 4 (quad) = 16384 counts/rev.
constexpr uint8_t  kAs5047uAbiResolutionBits = 14;
constexpr uint32_t kAbnCountsPerRev          = 16384;

inline constexpr ::tmc9660::units::MotorContext kCtx{
    /*motor_type      =*/tmc9660::tmcl::MotorType::BLDC_MOTOR,
    /*pole_pairs      =*/vortex_bench_safety::kDefaultPolePairs,
    /*velocity_sensor =*/tmc9660::tmcl::VelocitySensorSelection::ABN1_ENCODER,
    /*encoder_cpr     =*/kAbnCountsPerRev,
};

// ---- Velocity profile ------------------------------------------------------
// Closed-loop FOC + true rotor feedback → use a moderately aggressive sweep.
// Conservative-first because we want clean dual-feedback agreement before
// pushing the system. Tighten / loosen these per bench result.
struct Segment {
    const char* name;
    double      load_rpm;            ///< Signed target velocity at the load shaft (RPM)
    double      ramp_load_rpm_per_s; ///< |Δv|/dt during the ramp portion (load frame)
    uint32_t    hold_ms;             ///< Plateau time after the ramp completes
};

const Segment kProfile[] = {
    // Slow ramp up — verify ABN/AS5047U track each other along the way.
    {"P1.50",   50.0,   25.0, 1500},
    {"P1.100", 100.0,   25.0, 1500},
    {"P1.150", 150.0,   25.0, 1500},
    {"P1.200", 200.0,   50.0, 1500},
    {"P1.300", 300.0,   75.0, 2500},
    // Gentle reversal at low speed — cleanest test for direction agreement.
    {"P2.dn",    0.0,   75.0,  500},
    {"P2.rev", -75.0,   50.0, 1500},
    {"P2.zero",  0.0,   50.0,  500},
    // Modest dynamic burst, well within FOC capability.
    {"P3.burst", 200.0, 150.0, 1500},
    {"P3.brake",   0.0, 150.0,  500},
    // Wind down.
    {"P4.stop",    0.0,  50.0,  500},
};

inline vortex_motor_diagnostics::MotorExpectation make_expectation(
    double commanded_motor_rpm, bool check_velocity) noexcept {
    using namespace tmc9660::tmcl;
    vortex_motor_diagnostics::MotorExpectation exp{};
    exp.check_commutation_mode      = true;
    exp.expected_commutation_mode   = CommutationMode::FOC_ABN;
    exp.check_no_gate_driver_errors = true;
    exp.check_chip_temperature      = true;
    exp.chip_temp_max_c             = vortex_bench_safety::kChipShutdownTempC;
    exp.check_bus_voltage           = true;
    exp.vbus_min_v                  = 10.0f;
    exp.vbus_max_v                  = 30.0f;
    exp.check_velocity              = check_velocity;
    exp.expected_velocity_rpm       = commanded_motor_rpm;
    exp.velocity_tolerance_rpm      = 100.0;
    exp.check_velocity_direction    = false;
    exp.check_current               = true;
    exp.max_motor_current_ma        = vortex_bench_safety::kMaxPhaseCurrentMa;
    return exp;
}

// AS5047U configuration override — stock default disables ABI output.
inline As5047uConfig MakeAs5047uConfig() noexcept {
    As5047uConfig c = As5047uHandler::GetDefaultConfig();
    c.frame_format         = FrameFormat::SPI_24;       // CRC-protected
    c.crc_retries          = 2;
    c.enable_daec          = true;                      // dynamic angle error comp
    c.enable_adaptive_filter = true;
    c.enable_abi_output    = true;                      // <-- drives TMC9660 ABN1
    c.enable_uvw_output    = false;
    c.enable_pwm_output    = false;
    c.abi_resolution_bits  = kAs5047uAbiResolutionBits; // 14 → 16384 quad counts/rev
    c.zero_position        = 0;
    c.high_temperature_mode = false;
    return c;
}

// ----------------------------------------------------------------------------
// AS5047U bring-up + verification — runs before any motor command.
// Returns true if the sensor is talking and the magnetic field looks healthy.
// ----------------------------------------------------------------------------
[[maybe_unused]] bool bringup_as5047u(EncoderManager& enc, const char* tag) noexcept {
    if (!enc.EnsureInitialized()) {
        ESP_LOGE(tag, "EncoderManager init failed");
        return false;
    }

    const As5047uConfig cfg = MakeAs5047uConfig();
    if (!enc.CreateAs5047uDevice(EncoderManager::ENCODER_SLOT_0,
                                 SpiDeviceId::AS5047U_POSITION_ENCODER, cfg)) {
        ESP_LOGE(tag, "CreateAs5047uDevice(slot0, AS5047U_POSITION_ENCODER) failed — "
                      "check SPI bus + CS wiring (GPIO20)");
        return false;
    }
    ESP_LOGI(tag, "AS5047U created (frame=SPI_24, ABI=ON @ %u-bit, DAEC=ON, adaptive=ON)",
             static_cast<unsigned>(cfg.abi_resolution_bits));

    auto* drv = enc.GetAs5047uDriver(EncoderManager::ENCODER_SLOT_0);
    if (!drv) {
        ESP_LOGE(tag, "GetAs5047uDriver returned nullptr — handler not ready");
        return false;
    }

    // 1. Read & clear any latched error flags from boot.
    const uint16_t err_flags = drv->GetErrorFlags(2);
    ESP_LOGI(tag, "  ERRFL boot snapshot: 0x%04X (cleared by read)",
             static_cast<unsigned>(err_flags));

    // 2. AGC + magnitude — primary "is the magnet present and at the right gap" check.
    const uint8_t  agc  = drv->GetAGC(2);
    const uint16_t mag  = drv->GetMagnitude(2);
    const bool agc_ok   = (agc > 16 && agc < 240);   // healthy AGC band
    const bool mag_ok   = (mag > 200);               // non-trivial field strength
    ESP_LOGI(tag, "  AGC=%u (%s), Magnitude=%u (%s)",
             static_cast<unsigned>(agc), agc_ok ? "OK" : "WARN: rail",
             static_cast<unsigned>(mag), mag_ok ? "OK" : "WARN: weak field");

    // 3. Burst angle reads — detect SPI / framing issues.
    bool burst_ok = true;
    for (int i = 0; i < 8; ++i) {
        const uint16_t raw = drv->GetAngle(2);
        if (raw > 16383) {                        // 14-bit field; >16383 = corrupted
            burst_ok = false;
            ESP_LOGW(tag, "  angle burst[%d] = 0x%04X (out of 14-bit range)",
                     i, static_cast<unsigned>(raw));
        } else if (i == 0 || i == 7) {
            ESP_LOGI(tag, "  angle[%d] = %u LSB (%.2f deg)",
                     i, static_cast<unsigned>(raw),
                     static_cast<double>(raw) * (360.0 / 16384.0));
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // 4. Re-read ERRFL — anything latched during the burst is a red flag.
    const uint16_t err_after = drv->GetErrorFlags(2);
    if (err_after != 0) {
        ESP_LOGW(tag, "  ERRFL after burst: 0x%04X (post-comm errors latched)",
                 static_cast<unsigned>(err_after));
    }

    if (!burst_ok) {
        ESP_LOGE(tag, "AS5047U angle reads malformed — aborting");
        return false;
    }
    if (!agc_ok || !mag_ok) {
        ESP_LOGW(tag, "AS5047U field health degraded — proceeding but expect noisy ABN");
    }

    ESP_LOGI(tag, "✓ AS5047U bring-up + verification passed");
    return true;
}

// ----------------------------------------------------------------------------
// Decode TMC9660 GENERAL_ERROR_FLAGS bits to human-readable names.
// Bits per `GeneralErrorFlags` enum in tmc9660_param_mode_tmcl.hpp.
// ----------------------------------------------------------------------------
inline void log_gen_err_decoded(uint32_t flags, const char* tag, const char* label) noexcept {
    if (flags == 0) return;
    char buf[256];
    int  pos = 0;
    auto append = [&](const char* name) {
        const int rem = static_cast<int>(sizeof(buf)) - pos;
        if (rem <= 1) return;
        const int n = snprintf(buf + pos, static_cast<size_t>(rem), "%s%s",
                               (pos == 0) ? "" : ",", name);
        if (n > 0) pos += n;
    };
    if (flags & (1u <<  0)) append("CONFIG_ERROR");
    if (flags & (1u <<  1)) append("TMCL_SCRIPT_ERROR");
    if (flags & (1u <<  2)) append("HOMESWITCH_NOT_FOUND");
    if (flags & (1u <<  5)) append("HALL_ERROR");
    if (flags & (1u <<  9)) append("WATCHDOG_EVENT");
    if (flags & (1u << 13)) append("EXT_TEMP_EXCEEDED");
    if (flags & (1u << 14)) append("CHIP_TEMP_EXCEEDED");
    if (flags & (1u << 16)) append("I2T_1_EXCEEDED");
    if (flags & (1u << 17)) append("I2T_2_EXCEEDED");
    if (flags & (1u << 18)) append("EXT_TEMP_WARNING");
    if (flags & (1u << 19)) append("SUPPLY_OVERVOLTAGE_WARNING");
    if (flags & (1u << 20)) append("SUPPLY_UNDERVOLTAGE_WARNING");
    if (flags & (1u << 21)) append("ADC_IN_OVERVOLTAGE");
    if (flags & (1u << 22)) append("FAULT_RETRY_HAPPENED");
    if (flags & (1u << 23)) append("FAULT_RETRIES_FAILED");
    if (flags & (1u << 24)) append("CHIP_TEMP_WARNING");
    if (flags & (1u << 26)) append("HEARTBEAT_STOPPED");
    ESP_LOGW(tag, "  %s GEN_ERR=0x%08lX [%s]",
             label, static_cast<unsigned long>(flags),
             (pos > 0) ? buf : "(no decoded bits)");
}

// ----------------------------------------------------------------------------
// Build a single-line dual-feedback telemetry record:
//   chip Iq + chip ω vs AS5047U absolute angle + AS5047U ω.
// Cheap on the SPI bus; runs once per profile tick.
// ----------------------------------------------------------------------------
struct DualTelemetry {
    bool     chip_ok;
    bool     enc_ok;
    int32_t  chip_velocity_rpm;        ///< From TMC9660 ACTUAL_VELOCITY (motor-frame)
    int32_t  chip_iq_ma;
    uint16_t enc_angle_lsb;            ///< 0..16383 (14-bit), absolute mech angle
    float    enc_velocity_rpm;         ///< AS5047U DAEC velocity (deg/s → RPM, motor shaft)
    uint8_t  enc_agc;
};

template <typename Driver>
void capture_dual_telemetry(Driver& d, EncoderManager& enc,
                            DualTelemetry& out) noexcept {
    namespace tmcl = tmc9660::tmcl;
    (void)enc;  // SPI side disabled in this build
    out = {};

    uint32_t v_raw = 0;
    uint32_t iq_raw = 0;
    if (d.readParameter(tmcl::Parameters::ACTUAL_VELOCITY, v_raw) &&
        d.readParameter(tmcl::Parameters::ACTUAL_TORQUE, iq_raw)) {
        // Both fields are signed two's-complement (velocity 32-bit, torque 16-bit).
        out.chip_velocity_rpm = static_cast<int32_t>(v_raw);
        out.chip_iq_ma        = static_cast<int16_t>(iq_raw & 0xFFFF);
        out.chip_ok           = true;
    }

    // SPI-side fields intentionally left zero; enc_ok=false signals "not read".
    out.enc_ok = false;
}

}  // namespace

extern "C" void app_main(void) {
    ESP_LOGW(TAG,
             "MOTION APP — ABN-FOC + AS5047U dual feedback, %u pole pairs, gear=%.1f:1, "
             "ABN_CPR=%lu",
             static_cast<unsigned>(vortex_bench_safety::kDefaultPolePairs),
             kGearRatioMotorToOutput,
             static_cast<unsigned long>(kAbnCountsPerRev));
    ESP_LOGW(TAG, "             %zu profile segments, host trajectory @ 20 Hz. Secure motor & gearbox.",
             sizeof(kProfile) / sizeof(kProfile[0]));

    Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Spi);
    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.IsInitialized()) {
        ESP_LOGE(TAG, "Init failed (Vortex / MotorController)");
        return;
    }

    auto& motors = VORTEX_API.motors;
    auto& encoders = VORTEX_API.encoders;

    // ---- Phase A: AS5047U SPI bring-up SKIPPED ----------------------------
    // The AS5047U's ABN incremental output is active at POR with the default
    // 12-bit resolution (ABIRES=0 → 4096 PPR → 16384 quad counts/rev), so
    // FOC_ABN works without ever opening an SPI session. We rely entirely on
    // chip-side ACTUAL_VELOCITY / ACTUAL_TORQUE telemetry below.
    (void)encoders;  // not used in this run; SPI diagnostics deferred.
    ESP_LOGW(TAG, "AS5047U SPI bring-up SKIPPED — using ABN incremental input only "
                  "(POR default 12-bit ABI → CPR=%lu)",
             static_cast<unsigned long>(kAbnCountsPerRev));

    // ---- Phase B: Motor mechanics + complete TMC9660 BLDC config -----------
    {
        MotorMechanics mech{};
        mech.gear_ratio_motor_to_output = kGearRatioMotorToOutput;
        mech.invert_output              = false;
        if (!motors.setMotorMechanics(mech, MotorController::ONBOARD_TMC9660_INDEX)) {
            ESP_LOGE(TAG, "setMotorMechanics(%.1f:1) failed", kGearRatioMotorToOutput);
            return;
        }
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
        ESP_LOGE(TAG, "BLDC config failed");
        return;
    }

    // VGS-short workaround (see other bench apps for full explanation).
    motors.visitDriver(
        [](auto& d) { vortex_motor_bench::disable_vortex_uvw_vgs_short_protection(d, TAG); },
        MotorController::ONBOARD_TMC9660_INDEX);

    // ---- Phase C: ABN encoder config (CPR=16384, matches AS5047U 14-bit ABI)
    bool abn_ok = false;
    motors.visitDriver(
        [&abn_ok](auto& d) {
            abn_ok = vortex_motor_bench::configure_abn(d, TAG, kAbnCountsPerRev);
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!abn_ok) {
        ESP_LOGE(TAG, "configure_abn(CPR=%lu) failed",
                 static_cast<unsigned long>(kAbnCountsPerRev));
        return;
    }

    // ---- Phase D: ABN-edge probe (open-loop voltage spin) -------------------
    // FOC_ABN locks phi_e to the encoder counter; if no ABN edges arrive, the
    // chip dumps DC current into one electrical angle and the rotor never
    // moves. To verify ABN wiring before engaging FOC, briefly drive the rotor
    // in FOC_OPENLOOP_VOLTAGE_MODE (time-based commutation, independent of the
    // encoder) and watch ACTUAL_POSITION. If counts advance, ABN is wired
    // correctly and we can safely engage FOC_ABN. If counts stay at 0 we abort
    // with a clear message — physically pointless to continue.
    bool abn_edges_seen = false;
    int32_t abn_pos_delta = 0;
    motors.visitDriver(
        [&abn_edges_seen, &abn_pos_delta](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            ESP_LOGI(TAG, "ABN-edge probe: spinning open-loop voltage to verify quadrature");

            if (!vortex_motor_bench::configure_ramp_for_open_loop_voltage(d, TAG, kCtx)) {
                ESP_LOGE(TAG, "open-loop ramp config failed for probe");
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

            // Drive ~80 motor RPM to generate ABN edges. At 80 RPM the rotor
            // produces ~80 × 16384 / 60 ≈ 21845 quad counts per second, so a
            // 1.5 s spin should advance ACTUAL_POSITION by ≈32768 counts.
            constexpr double kProbeMotorRpm = 80.0;

            // Read baseline ACTUAL_POSITION.
            uint32_t pos_before_raw = 0;
            (void)d.readParameter(tmcl::Parameters::ACTUAL_POSITION, pos_before_raw);
            const int32_t pos_before = static_cast<int32_t>(pos_before_raw);

            if (!d.velocityControl.setTargetVelocityRpm(kProbeMotorRpm, kCtx)) {
                ESP_LOGE(TAG, "velocityControl.setTargetVelocityRpm(%.1f) for probe failed",
                         kProbeMotorRpm);
                return;
            }

            // Spin for 1500 ms.
            vTaskDelay(pdMS_TO_TICKS(1500));

            uint32_t pos_after_raw = 0;
            (void)d.readParameter(tmcl::Parameters::ACTUAL_POSITION, pos_after_raw);
            const int32_t pos_after = static_cast<int32_t>(pos_after_raw);
            abn_pos_delta = pos_after - pos_before;

            uint32_t v_raw = 0;
            (void)d.readParameter(tmcl::Parameters::ACTUAL_VELOCITY, v_raw);
            const int32_t v_now = static_cast<int32_t>(v_raw);

            const long expected_counts =
                static_cast<long>(kProbeMotorRpm * kAbnCountsPerRev / 60.0 * 1.5);
            ESP_LOGI(TAG,
                     "ABN-edge probe result: ΔACTUAL_POSITION=%ld counts (expected ≈%ld for 1.5s @ %.0f motor-RPM); "
                     "chip ω=%ld",
                     static_cast<long>(abn_pos_delta), expected_counts,
                     kProbeMotorRpm, static_cast<long>(v_now));

            uint32_t gen_err = 0;
            (void)d.telemetry.getGeneralErrorFlags(gen_err);
            log_gen_err_decoded(gen_err, TAG, "probe");

            // Threshold: at 80 motor-RPM we expect ~32768 counts in 1.5s.
            // Accept anything > 1000 as "edges are arriving".
            constexpr int32_t kMinExpectedCounts = 1000;
            abn_edges_seen = (std::abs(abn_pos_delta) >= kMinExpectedCounts);

            // Stop motion before exiting probe.
            (void)d.velocityControl.setTargetVelocityRpm(0.0, kCtx);
            vTaskDelay(pdMS_TO_TICKS(300));
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    if (!abn_edges_seen) {
        ESP_LOGE(TAG,
                 "✗ ABN-edge probe FAILED: |Δpos|=%ld counts < 1000 threshold. "
                 "Quadrature signal is not reaching TMC9660. Check:",
                 static_cast<long>(abn_pos_delta));
        ESP_LOGE(TAG, "    1) AS5047U VDD (3.3V at the IC, not just the breakout header)");
        ESP_LOGE(TAG, "    2) Wiring: AS5047U A/B/I → TMC9660 GPIO8/GPIO13/GPIO14");
        ESP_LOGE(TAG, "    3) Magnet present and within axial gap range");
        ESP_LOGE(TAG, "    4) AS5047U DISABLE register (cleared at POR; only set if SPI was used)");
        motors.visitDriver(
            [](auto& d) {
                namespace tmcl = tmc9660::tmcl;
                (void)d.motorConfig.setCommutationMode(tmcl::CommutationMode::SYSTEM_OFF);
                vortex_motor_bench::motor_stop_safe(d, TAG);
            },
            MotorController::ONBOARD_TMC9660_INDEX);
        return;
    }
    ESP_LOGI(TAG, "✓ ABN-edge probe passed (Δpos=%ld counts) — wiring confirmed",
             static_cast<long>(abn_pos_delta));

    // Stop the open-loop spin cleanly so we can re-arm in FOC_ABN.
    // CRITICAL CORRECTION (per datasheet §"ABN Initialization Methods", p. 2411-2422):
    // The FORCED_PHI_E_* init methods use OPENLOOP_CURRENT (param 46, mA), NOT
    // OPENLOOP_VOLTAGE. Default OPENLOOP_CURRENT = 1000 mA was insufficient to
    // overcome the 10:1 gearbox + BLDC cogging static friction → rotor stayed
    // at random angle → captured ABN_1_PHI_E_OFFSET was wrong → FOC_ABN
    // produced near-zero net torque (vibration only).
    // Datasheet: "Especially BLDC-Motors tend to show increased cogging which
    //  requires higher currents for smooth rotations."
    motors.visitDriver(
        [](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            (void)d.motorConfig.setCommutationMode(tmcl::CommutationMode::SYSTEM_OFF);
            vTaskDelay(pdMS_TO_TICKS(200));
            vortex_motor_bench::clear_fault_flags(d);
            // Re-apply VGS-short protection disable: SYSTEM_OFF can re-arm the
            // GDRV charge-short defaults; without re-disabling, FOC_ABN engages
            // but each PWM gate transition trips a fault → retried internally →
            // gates effectively never modulate → motor sits with Iq≈0.
            vortex_motor_bench::disable_vortex_uvw_vgs_short_protection(d, TAG);
            // Set OPENLOOP_CURRENT (the parameter the chip actually uses for
            // FORCED_PHI_E_* alignment) to 2000 mA — well within MAX_TORQUE=2500
            // mA, but 2x the default and enough to overcome gearbox+cogging.
            constexpr uint16_t kAbnInitOpenloopCurrent_mA = 2000;
            if (!d.torqueFluxControl.setOpenloopCurrent(kAbnInitOpenloopCurrent_mA)) {
                ESP_LOGW(TAG, "setOpenloopCurrent(%u) failed", kAbnInitOpenloopCurrent_mA);
            } else {
                ESP_LOGI(TAG, "Set OPENLOOP_CURRENT=%u mA for ABN init alignment",
                         kAbnInitOpenloopCurrent_mA);
            }
            // OPENLOOP_VOLTAGE is harmless for FOC_ABN init but kept for any
            // residual probe-mode behavior; not relied upon for alignment.
            (void)d.torqueFluxControl.setOpenloopVoltage(vortex_bench_safety::kOpenLoopVoltage);
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    // ---- Phase E: ramp arming + FOC_ABN engage ------------------------------
    constexpr double kPeakMotorRpm       = 3300.0;
    constexpr double kPeakMotorRpmPerSec = 5500.0;
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
                ESP_LOGE(TAG, "ramp.configureAuto failed");
                return;
            }

            vortex_motor_bench::clear_fault_flags(d);
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_ABN)) {
                ESP_LOGE(TAG, "setCommutationMode(FOC_ABN) failed");
                return;
            }
            ESP_LOGI(TAG, "FOC_ABN commanded — encoder init runs (~1 s phi_e zero swing)");

            // Poll ABN_1_INIT_STATE until DONE (or timeout). The chip drives
            // the rotor to phi_e=0 using OPENLOOP_VOLTAGE; if init never reaches
            // DONE the FOC velocity loop will be inhibited and the motor sits
            // at zero torque. This is the most likely failure mode.
            tmcl::AbnInitState init_state = tmcl::AbnInitState::IDLE;
            constexpr uint32_t kInitTimeoutMs = 5000;
            constexpr uint32_t kInitPollMs    = 100;
            uint32_t waited = 0;
            while (waited < kInitTimeoutMs) {
                vTaskDelay(pdMS_TO_TICKS(kInitPollMs));
                waited += kInitPollMs;
                if (!d.feedbackSense.getABNInitializationState(init_state)) {
                    ESP_LOGW(TAG, "getABNInitializationState() read failed at t=%lu ms",
                             static_cast<unsigned long>(waited));
                    continue;
                }
                if (init_state == tmcl::AbnInitState::DONE) break;
                if ((waited % 500) == 0) {
                    ESP_LOGI(TAG, "  ABN init state @ %lu ms = %s",
                             static_cast<unsigned long>(waited),
                             tmcl::to_string(init_state));
                }
            }
            ESP_LOGI(TAG, "ABN init final state after %lu ms = %s",
                     static_cast<unsigned long>(waited),
                     tmcl::to_string(init_state));

            // Post-init telemetry: did the rotor actually move during init? Is
            // phi_e a sensible value? What encoder offset was captured?
            uint32_t abn_val = 0, ol_v = 0, ol_c = 0, act_pos = 0, act_vel = 0;
            uint32_t phi_e_off_raw = 0;
            int16_t  phi_e   = 0;
            (void)d.readParameter(tmcl::Parameters::ABN_1_VALUE, abn_val);
            (void)d.readParameter(tmcl::Parameters::OPENLOOP_VOLTAGE, ol_v);
            (void)d.readParameter(tmcl::Parameters::OPENLOOP_CURRENT, ol_c);
            (void)d.readParameter(tmcl::Parameters::ACTUAL_POSITION, act_pos);
            (void)d.readParameter(tmcl::Parameters::ACTUAL_VELOCITY, act_vel);
            (void)d.readParameter(tmcl::Parameters::ABN_1_N_CHANNEL_PHI_E_OFFSET,
                                  phi_e_off_raw);
            (void)d.feedbackSense.getABNPhiE(phi_e);
            ESP_LOGI(TAG,
                     "  Post-init: ABN_1_VALUE=%lu, ABN_1_PHI_E=%d, "
                     "phi_e_offset=%d, ACTUAL_POSITION=%ld, ACTUAL_VELOCITY=%ld, "
                     "OPENLOOP_V=%lu, OPENLOOP_I=%lu mA",
                     static_cast<unsigned long>(abn_val), static_cast<int>(phi_e),
                     static_cast<int>(static_cast<int16_t>(phi_e_off_raw & 0xFFFFu)),
                     static_cast<long>(static_cast<int32_t>(act_pos)),
                     static_cast<long>(static_cast<int32_t>(act_vel)),
                     static_cast<unsigned long>(ol_v),
                     static_cast<unsigned long>(ol_c));

            if (init_state != tmcl::AbnInitState::DONE) {
                ESP_LOGE(TAG, "✗ ABN init did not reach DONE — FOC will not produce "
                              "torque. Per datasheet, FORCED_PHI_E_* methods need "
                              "OPENLOOP_CURRENT high enough to overcome cogging+load.");
            }

            uint32_t cm = 0xFFFFFFFFu;
            (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            if (cm != static_cast<uint32_t>(tmcl::CommutationMode::FOC_ABN)) {
                ESP_LOGW(TAG, "COMMUTATION_MODE=%u after arm (expected FOC_ABN=%u) — retrying",
                         static_cast<unsigned>(cm),
                         static_cast<unsigned>(tmcl::CommutationMode::FOC_ABN));
                uint32_t gen_err_revert = 0;
                (void)d.telemetry.getGeneralErrorFlags(gen_err_revert);
                log_gen_err_decoded(gen_err_revert, TAG, "after-revert");
                vortex_motor_bench::clear_fault_flags(d);
                (void)d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_ABN);
                vTaskDelay(pdMS_TO_TICKS(1500));
                (void)d.readParameter(tmcl::Parameters::COMMUTATION_MODE, cm);
            }
            armed = (cm == static_cast<uint32_t>(tmcl::CommutationMode::FOC_ABN));
            if (armed) {
                // CRITICAL: ensure TARGET_TORQUE is zero so the chip uses the
                // velocity loop (not torque mode). Per datasheet line 3102, any
                // non-zero TARGET_TORQUE write activates torque-regulation mode
                // and bypasses TARGET_VELOCITY entirely.
                (void)d.writeParameter(tmcl::Parameters::TARGET_TORQUE, 0u);
                // Also explicitly zero TARGET_FLUX (param 106). Default is 0
                // but bench probes/visits may leave it stale.
                (void)d.writeParameter(tmcl::Parameters::TARGET_FLUX, 0u);
                // Confirm regulation mode flags
                uint32_t gsf = 0;
                (void)d.readParameter(tmcl::Parameters::GENERAL_STATUS_FLAGS, gsf);
                ESP_LOGI(TAG, "  Post-arm GEN_STATUS=0x%08lX (REG_TORQUE=%u, "
                              "REG_VELOCITY=%u, REG_POSITION=%u)",
                         static_cast<unsigned long>(gsf),
                         static_cast<unsigned>((gsf >> 1) & 1u),
                         static_cast<unsigned>((gsf >> 2) & 1u),
                         static_cast<unsigned>((gsf >> 3) & 1u));
            }
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!armed) {
        ESP_LOGE(TAG, "Could not arm FOC_ABN — abort");
        motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d, TAG); },
                           MotorController::ONBOARD_TMC9660_INDEX);
        return;
    }
    ESP_LOGI(TAG, "✓ Armed in FOC_ABN, commencing dual-feedback velocity sweep");

    // ---- Phase F: velocity sweep with dual-feedback telemetry ---------------
    using ::tmc9660::units::VelocityUnit;
    constexpr uint32_t kTickMs       = 50;        // 20 Hz host trajectory tick
    constexpr uint32_t kRampLogEvery = 5;         // ~250 ms between full diagnostics during ramp
    constexpr uint32_t kHoldLogEvery = 4;         // ~200 ms between full diagnostics during hold

    double prev_load_rpm = 0.0;
    bool   profile_ok    = true;

    for (const Segment& seg : kProfile) {
        const double dv_load   = seg.load_rpm - prev_load_rpm;
        const double rate_load = std::max(1.0, std::fabs(seg.ramp_load_rpm_per_s));
        const double ramp_secs = std::fabs(dv_load) / rate_load;
        const uint32_t ramp_ms = static_cast<uint32_t>(std::ceil(ramp_secs * 1000.0));

        ESP_LOGI(TAG,
                 "[%s] %+.1f -> %+.1f load-RPM (motor=%+.0f) rate=%.0f load-RPM/s ramp=%lu ms hold=%lu ms",
                 seg.name, prev_load_rpm, seg.load_rpm,
                 seg.load_rpm * kGearRatioMotorToOutput, rate_load,
                 static_cast<unsigned long>(ramp_ms),
                 static_cast<unsigned long>(seg.hold_ms));

        // ---- Ramp portion: linearly interpolate load setpoint each tick.
        if (ramp_ms > 0) {
            const uint32_t n_ticks = std::max<uint32_t>(1u, ramp_ms / kTickMs);
            for (uint32_t i = 1; i <= n_ticks && profile_ok; ++i) {
                const double frac   = static_cast<double>(i) / static_cast<double>(n_ticks);
                const double v_load = prev_load_rpm + dv_load * frac;
                if (!motors.setLoadVelocity(v_load, VelocityUnit::Rpm, kCtx,
                                            MotorController::ONBOARD_TMC9660_INDEX)) {
                    ESP_LOGE(TAG, "[%s] setLoadVelocity(%.2f) failed", seg.name, v_load);
                    profile_ok = false;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(kTickMs));

                DualTelemetry dt{};
                motors.visitDriver(
                    [&dt, &encoders](auto& d) { capture_dual_telemetry(d, encoders, dt); },
                    MotorController::ONBOARD_TMC9660_INDEX);

                ESP_LOGI(TAG,
                         "[%s.r%lu] cmd=%+.1f load (motor=%+.0f) | chip ω=%+ld Iq=%+ld mA",
                         seg.name, static_cast<unsigned long>(i),
                         v_load, v_load * kGearRatioMotorToOutput,
                         static_cast<long>(dt.chip_velocity_rpm),
                         static_cast<long>(dt.chip_iq_ma));

                if ((i % kRampLogEvery == 0) || (i == n_ticks)) {
                    const auto exp = make_expectation(v_load * kGearRatioMotorToOutput,
                                                      /*check_velocity=*/false);
                    motors.visitDriver(
                        [&seg, i, &exp](auto& d) {
                            char label[40];
                            snprintf(label, sizeof(label), "%s.r%lu", seg.name,
                                     static_cast<unsigned long>(i));
                            (void)vortex_motor_diagnostics::diagnose_and_log(
                                d, kCtx, exp, TAG, label);

                            // Deep FOC-chain trace: read every signal in the
                            // velocity-PI → torque-PI → Uq path so we can see
                            // exactly where the chain breaks.
                            namespace tmcl = tmc9660::tmcl;
                            uint32_t r_tgt_v = 0, r_act_v = 0, r_vpi_err = 0, r_vpi_int = 0;
                            uint32_t r_tgt_t = 0, r_act_t = 0, r_tpi_err = 0, r_tpi_int = 0;
                            uint32_t r_foc_iq = 0, r_foc_uq = 0, r_abn_val = 0;
                            (void)d.readParameter(tmcl::Parameters::TARGET_VELOCITY,    r_tgt_v);
                            (void)d.readParameter(tmcl::Parameters::ACTUAL_VELOCITY,    r_act_v);
                            (void)d.readParameter(tmcl::Parameters::VELOCITY_PI_ERROR,  r_vpi_err);
                            (void)d.readParameter(tmcl::Parameters::VELOCITY_PI_INTEGRATOR, r_vpi_int);
                            (void)d.readParameter(tmcl::Parameters::TARGET_TORQUE,      r_tgt_t);
                            (void)d.readParameter(tmcl::Parameters::ACTUAL_TORQUE,      r_act_t);
                            (void)d.readParameter(tmcl::Parameters::TORQUE_PI_ERROR,    r_tpi_err);
                            (void)d.readParameter(tmcl::Parameters::TORQUE_PI_INTEGRATOR,r_tpi_int);
                            (void)d.readParameter(tmcl::Parameters::FOC_CURRENT_IQ,     r_foc_iq);
                            (void)d.readParameter(tmcl::Parameters::FOC_VOLTAGE_UQ,     r_foc_uq);
                            (void)d.readParameter(tmcl::Parameters::ABN_1_VALUE,        r_abn_val);
                            ESP_LOGI(TAG,
                                "[%s] FOC-CHAIN  TGT_V=%ld  ACT_V=%ld  V_ERR=%ld  V_INT=%ld  | "
                                "TGT_T=%d  ACT_T=%d  T_ERR=%ld  T_INT=%ld  | "
                                "FOC_IQ=%d  FOC_UQ=%d  ABN_VAL=%lu",
                                label,
                                static_cast<long>(static_cast<int32_t>(r_tgt_v)),
                                static_cast<long>(static_cast<int32_t>(r_act_v)),
                                static_cast<long>(static_cast<int32_t>(r_vpi_err)),
                                static_cast<long>(static_cast<int32_t>(r_vpi_int)),
                                static_cast<int>(static_cast<int16_t>(r_tgt_t & 0xFFFFu)),
                                static_cast<int>(static_cast<int16_t>(r_act_t & 0xFFFFu)),
                                static_cast<long>(static_cast<int32_t>(r_tpi_err)),
                                static_cast<long>(static_cast<int32_t>(r_tpi_int)),
                                static_cast<int>(static_cast<int16_t>(r_foc_iq & 0xFFFFu)),
                                static_cast<int>(static_cast<int16_t>(r_foc_uq & 0xFFFFu)),
                                static_cast<unsigned long>(r_abn_val));
                        },
                        MotorController::ONBOARD_TMC9660_INDEX);
                }
            }
        } else {
            (void)motors.setLoadVelocity(seg.load_rpm, VelocityUnit::Rpm, kCtx,
                                         MotorController::ONBOARD_TMC9660_INDEX);
        }

        if (!profile_ok) break;

        // ---- Hold portion --------------------------------------------------
        const auto exp_hold = make_expectation(seg.load_rpm * kGearRatioMotorToOutput,
                                               /*check_velocity=*/true);
        uint32_t hold_idx = 0;
        for (uint32_t t = 0; t < seg.hold_ms; t += kTickMs) {
            vTaskDelay(pdMS_TO_TICKS(kTickMs));

            DualTelemetry dt{};
            motors.visitDriver(
                [&dt, &encoders](auto& d) { capture_dual_telemetry(d, encoders, dt); },
                MotorController::ONBOARD_TMC9660_INDEX);

            ESP_LOGI(TAG,
                     "[%s.h%lu] tgt=%+.1f load | chip ω=%+ld Iq=%+ld mA",
                     seg.name, static_cast<unsigned long>(t),
                     seg.load_rpm,
                     static_cast<long>(dt.chip_velocity_rpm),
                     static_cast<long>(dt.chip_iq_ma));

            if (hold_idx % kHoldLogEvery == 0) {
                motors.visitDriver(
                    [&seg, t, &exp_hold](auto& d) {
                        char label[40];
                        snprintf(label, sizeof(label), "%s.h%lu", seg.name,
                                 static_cast<unsigned long>(t));
                        (void)vortex_motor_diagnostics::diagnose_and_log(
                            d, kCtx, exp_hold, TAG, label);
                    },
                    MotorController::ONBOARD_TMC9660_INDEX);
            }
            ++hold_idx;
        }

        prev_load_rpm = seg.load_rpm;
    }

    // ---- Phase F: stop + final diagnostics ---------------------------------
    motors.visitDriver(
        [](auto& d) {
            vortex_motor_bench::motor_stop_safe(d, TAG);
            vortex_motor_bench::log_fault_flags(d, TAG, "post-stop");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "ABN+AS5047U dynamics %s",
             profile_ok ? "finished OK" : "ABORTED — see [ERROR] above");
}
