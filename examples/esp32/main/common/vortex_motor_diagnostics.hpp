/**
 * @file vortex_motor_diagnostics.hpp
 * @brief Bench-side policy / expectation checker on top of the vendor
 *        `tmc9660::diagnostics::MotorSnapshot`.
 *
 * The vendor library now owns "what is the chip currently doing"
 * (`tmc9660::diagnostics::MotorSnapshot` populated by
 * `d.diagnostics.snapshot(snap, ctx)`). This header keeps **only** the
 * bench-bringup policy: a `MotorExpectation` describing what the operator
 * believes the motor should be doing, plus a pure `check()` that walks a
 * snapshot and emits a human-readable report. `diagnose_and_log()` is the
 * one-shot helper that grabs a snapshot, runs the check, and pretty-prints
 * everything.
 *
 * Read-only: never writes a parameter; safe to call mid-spin at any cadence.
 */
#pragma once

#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp"
#include "esp_log.h"

#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>

namespace vortex_motor_diagnostics {

namespace tmcl = tmc9660::tmcl;
using ::tmc9660::diagnostics::MotorSnapshot;
using ::tmc9660::units::MotorContext;

// ---------------------------------------------------------------------------
// Expectation
// ---------------------------------------------------------------------------

/**
 * @brief Bench-side description of "what should be happening". Each
 *        `check_*` flag enables one assertion. Disabled assertions never
 *        fire.
 */
struct MotorExpectation {
    // Commutation -----------------------------------------------------------
    bool                    check_commutation_mode = false;
    tmcl::CommutationMode   expected_commutation_mode = tmcl::CommutationMode::SYSTEM_OFF;

    // Velocity --------------------------------------------------------------
    bool   check_velocity            = false;
    double expected_velocity_rpm     = 0.0;
    double velocity_tolerance_rpm    = 5.0;
    /// If true, also require sign(actual_velocity) == sign(expected_velocity_rpm).
    bool   check_velocity_direction  = true;

    // Current / power -------------------------------------------------------
    bool     check_current      = false;
    uint16_t max_motor_current_ma = 2000;

    // Bus / thermal ---------------------------------------------------------
    bool   check_bus_voltage = false;
    float  vbus_min_v        = 10.0f;
    float  vbus_max_v        = 30.0f;

    bool   check_chip_temperature = false;
    float  chip_temp_max_c        = 90.0f;

    // Faults ----------------------------------------------------------------
    bool     check_no_gate_driver_errors = true;
    /// Bit-mask of GENERAL_ERROR_FLAGS bits to ignore (e.g. WATCHDOG / CONFIG
    /// bits that are sticky after boot but harmless during a spin).
    uint32_t general_error_ignore_mask   = 0;
};

// ---------------------------------------------------------------------------
// Report
// ---------------------------------------------------------------------------

/// One human-readable issue surfaced by `check()`.
struct DiagnosticIssue {
    enum class Severity : uint8_t { kInfo, kWarn, kError };
    Severity severity = Severity::kInfo;
    /// Short fixed buffer keeps this struct trivially copyable.
    char     message[128] = {};
};

/// Aggregate report from `check()`.
struct DiagnosticReport {
    static constexpr size_t kMaxIssues = 16;
    std::array<DiagnosticIssue, kMaxIssues> issues{};
    size_t  issue_count = 0;
    bool    pass        = true;

    void add(DiagnosticIssue::Severity sev, const char* fmt, ...) noexcept
        __attribute__((format(printf, 3, 4))) {
        if (issue_count >= kMaxIssues) return;
        DiagnosticIssue& it = issues[issue_count++];
        it.severity = sev;
        va_list ap; va_start(ap, fmt);
        vsnprintf(it.message, sizeof(it.message), fmt, ap);
        va_end(ap);
        if (sev == DiagnosticIssue::Severity::kError) pass = false;
    }
};

// ---------------------------------------------------------------------------
// check()
// ---------------------------------------------------------------------------

/**
 * @brief Compare the snapshot against `exp` and produce a `DiagnosticReport`.
 *        Pure function (no I/O).
 */
inline DiagnosticReport check(MotorSnapshot const& s,
                              MotorExpectation const& exp) noexcept {
    DiagnosticReport rep{};

    if (exp.check_commutation_mode) {
        if (!s.valid_commutation_mode) {
            rep.add(DiagnosticIssue::Severity::kError,
                    "COMMUTATION_MODE unreadable (chip not responding to NR 70 read)");
        } else if (s.commutation_mode != exp.expected_commutation_mode) {
            rep.add(DiagnosticIssue::Severity::kError,
                    "COMMUTATION_MODE=%u, expected %u",
                    static_cast<unsigned>(s.commutation_mode),
                    static_cast<unsigned>(exp.expected_commutation_mode));
        }
    }

    if (exp.check_velocity) {
        const double err = s.actual_velocity_rpm - exp.expected_velocity_rpm;
        if (std::fabs(err) > exp.velocity_tolerance_rpm) {
            rep.add(DiagnosticIssue::Severity::kError,
                    "Velocity off-target: actual=%.2f RPM, expected=%.2f \u00B1%.2f RPM (err=%.2f)",
                    s.actual_velocity_rpm, exp.expected_velocity_rpm,
                    exp.velocity_tolerance_rpm, err);
        }
        if (exp.check_velocity_direction && exp.expected_velocity_rpm != 0.0) {
            const bool same_sign = (s.actual_velocity_rpm * exp.expected_velocity_rpm) >= 0.0;
            if (!same_sign) {
                rep.add(DiagnosticIssue::Severity::kError,
                        "Velocity direction wrong: actual=%.2f RPM, expected sign matches %.2f RPM",
                        s.actual_velocity_rpm, exp.expected_velocity_rpm);
            }
        }
    }

    if (exp.check_current) {
        const int abs_i = std::abs(static_cast<int>(s.motor_current_ma));
        if (abs_i > static_cast<int>(exp.max_motor_current_ma)) {
            rep.add(DiagnosticIssue::Severity::kError,
                    "Motor current %d mA exceeds max %u mA",
                    abs_i, static_cast<unsigned>(exp.max_motor_current_ma));
        }
    }

    if (exp.check_bus_voltage) {
        if (s.vbus_volts < exp.vbus_min_v || s.vbus_volts > exp.vbus_max_v) {
            rep.add(DiagnosticIssue::Severity::kError,
                    "Vbus out of range: %.2f V (allowed %.2f \u2026 %.2f)",
                    static_cast<double>(s.vbus_volts),
                    static_cast<double>(exp.vbus_min_v),
                    static_cast<double>(exp.vbus_max_v));
        }
    }

    if (exp.check_chip_temperature && s.chip_temp_c > exp.chip_temp_max_c) {
        rep.add(DiagnosticIssue::Severity::kError,
                "Chip temperature %.1f \u00B0C exceeds %.1f \u00B0C",
                static_cast<double>(s.chip_temp_c),
                static_cast<double>(exp.chip_temp_max_c));
    }

    if (exp.check_no_gate_driver_errors && s.gate_driver_error_flags != 0) {
        rep.add(DiagnosticIssue::Severity::kError,
                "Gate driver error flags non-zero: 0x%08X",
                static_cast<unsigned>(s.gate_driver_error_flags));
    }

    const uint32_t masked_general_errors = s.general_error_flags & ~exp.general_error_ignore_mask;
    if (masked_general_errors != 0) {
        // Treat un-masked general errors as warnings — many of them are
        // sticky boot artifacts (config-loaded, watchdog) that the bench
        // intentionally ignores. Caller can promote with the mask.
        rep.add(DiagnosticIssue::Severity::kWarn,
                "GENERAL_ERROR_FLAGS=0x%08X (masked=0x%08X)",
                static_cast<unsigned>(s.general_error_flags),
                static_cast<unsigned>(masked_general_errors));
    }

    return rep;
}

// ---------------------------------------------------------------------------
// Pretty-print + one-shot helper
// ---------------------------------------------------------------------------

/**
 * @brief Take a snapshot via the vendor `Diagnostics` subsystem, run
 *        @ref check against `exp`, and pretty-print everything to ESP_LOG.
 *        Returns the report so callers can branch on `report.pass`.
 */
template <typename Comm>
inline DiagnosticReport diagnose_and_log(tmc9660::TMC9660<Comm>& d,
                                         MotorContext const& ctx,
                                         MotorExpectation const& exp,
                                         const char* tag,
                                         const char* label = "diag") noexcept {
    MotorSnapshot s{};
    (void)d.diagnostics.snapshot(s, ctx);
    const DiagnosticReport rep = check(s, exp);

    ESP_LOGI(tag, "[%s] === MOTOR SNAPSHOT (pole_pairs=%u, k_RPM=%.2f) ===",
             label, static_cast<unsigned>(ctx.pole_pairs), ctx.k_rpm());

    ESP_LOGI(tag, "[%s] Bus       Vbus=%.2f V    T_chip=%.1f \u00B0C",
             label, static_cast<double>(s.vbus_volts), static_cast<double>(s.chip_temp_c));

    ESP_LOGI(tag, "[%s] Mode      COMMUTATION_MODE=%u%s",
             label, static_cast<unsigned>(s.commutation_mode),
             s.valid_commutation_mode ? "" : " (READ FAILED)");

    ESP_LOGI(tag,
             "[%s] Velocity  target=%ld (%.2f RPM)  actual=%ld (%.2f RPM)  ramp=%ld (%.2f RPM)",
             label,
             static_cast<long>(s.target_velocity_internal), s.target_velocity_rpm,
             static_cast<long>(s.actual_velocity_internal), s.actual_velocity_rpm,
             static_cast<long>(s.ramp_velocity_internal),   s.ramp_velocity_rpm);

    ESP_LOGI(tag,
             "[%s] Position  pos=%ld counts (%.3f mech revs, %.1f \u00B0)  phi_e=%d (%.1f \u00B0e, %.1f \u00B0mech)",
             label,
             static_cast<long>(s.actual_position_counts), s.actual_position_revs,
             s.actual_position_deg_mech,
             static_cast<int>(s.phi_e_internal), s.phi_e_deg_elec, s.phi_e_deg_mech);

    ESP_LOGI(tag,
             "[%s] Currents  I_motor=%d mA  Iq=%d mA  Iuvw=(%d,%d,%d)  Uq=%d  Uuvw=(%d,%d,%d)",
             label,
             static_cast<int>(s.motor_current_ma), static_cast<int>(s.iq_ma),
             static_cast<int>(s.i_ux), static_cast<int>(s.i_v), static_cast<int>(s.i_wy),
             static_cast<int>(s.uq),
             static_cast<int>(s.u_ux), static_cast<int>(s.u_v), static_cast<int>(s.u_wy));

    ESP_LOGI(tag,
             "[%s] Flags     gen_status=0x%08X  gen_err=0x%08X  gd_err=0x%08X  adc_status=0x%08X",
             label,
             static_cast<unsigned>(s.general_status_flags),
             static_cast<unsigned>(s.general_error_flags),
             static_cast<unsigned>(s.gate_driver_error_flags),
             static_cast<unsigned>(s.adc_status_flags));

    ESP_LOGI(tag,
             "[%s] Regulating: %s%s%s",
             label,
             s.regulating_velocity ? "VELOCITY " : "",
             s.regulating_torque   ? "TORQUE "   : "",
             s.velocity_reached    ? "VEL_REACHED" : "");

    if (rep.issue_count == 0) {
        ESP_LOGI(tag, "[%s] CHECK PASS (no issues)", label);
    } else {
        for (size_t i = 0; i < rep.issue_count; ++i) {
            const DiagnosticIssue& it = rep.issues[i];
            switch (it.severity) {
                case DiagnosticIssue::Severity::kInfo:
                    ESP_LOGI(tag, "[%s]   info: %s", label, it.message); break;
                case DiagnosticIssue::Severity::kWarn:
                    ESP_LOGW(tag, "[%s]   warn: %s", label, it.message); break;
                case DiagnosticIssue::Severity::kError:
                    ESP_LOGE(tag, "[%s]   ERROR: %s", label, it.message); break;
            }
        }
        ESP_LOGI(tag, "[%s] CHECK %s (%u issue(s))",
                 label, rep.pass ? "PASS-with-warnings" : "FAIL",
                 static_cast<unsigned>(rep.issue_count));
    }
    return rep;
}

}  // namespace vortex_motor_diagnostics
