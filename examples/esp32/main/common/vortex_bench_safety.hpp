/**
 * @file vortex_bench_safety.hpp
 * @brief Single source of truth for Vortex V1 BLDC bench parameters.
 *
 * The on-hand reference motor is a small geared **24 V, 30 W** BLDC:
 *   - 14 rotor poles → **7 pole pairs**
 *   - Phase-to-phase R ≈ 1.4 Ω, L ≈ 0.78 mH
 *   - Continuous current ≈ 1.25 A, peak ~2.5–3 A (electrical, motor side)
 *   - 10:1 reducer, ≤ 400 rpm at the output shaft
 *
 * The Vortex V1 power stage closely mirrors the TMC9660-3PH-EVKIT (slightly
 * lower-Rds(on) MOSFETs, same 3 mΩ shunt class). Numbers below are the
 * starting point for parameter-mode bring-up; tweak per board / motor before
 * you turn on DRV_EN.
 *
 * Velocity targets are in **TMCL internal units** (driver auto-scales when
 * `velocityScalingFactor = 1` and a sensor with known CPR / poles is selected).
 * Verify against an external tach before trusting absolute RPM.
 */
#pragma once

#include <cstdint>

namespace vortex_bench_safety {

// ============================================================================
// Power stage / supply (24 V class bench)
// ============================================================================
/// Nominal DC bus (volts) — used for protection thresholds and gate-driver auto-config.
inline constexpr float kNominalSupplyVolts = 24.0f;
/// Overvoltage warning threshold (V) — picked above ramp-down headroom.
inline constexpr float kOvervoltageThresholdV = 28.0f;
/// Undervoltage warning threshold (V).
inline constexpr float kUndervoltageThresholdV = 20.0f;

/// MOSFET gate charge — Vortex Rds(on)/Qg are very close to the EvKit
/// BSC040N10NS5 reference; if your board uses a different FET, adjust here.
inline constexpr float kMosfetRdsOnMilliOhm = 4.0f;
inline constexpr float kMosfetGateChargeNc = 58.0f;

/// 3 mΩ bottom-shunt sense resistor (matches TMC9660-3PH-EVKIT class).
inline constexpr float kShuntResistanceMilliOhm = 3.0f;

/// Peak phase current we tell the gate driver / current sensing to plan for.
/// Sized for our 30 W motor (continuous ~1.25 A, peak ~2.5–3 A).
inline constexpr float kExpectedPeakCurrentA = 3.0f;

/// Approximate phase inductance (µH). 0.78 mH line-line ≈ 390 µH per phase.
inline constexpr float kMotorPhaseInductanceUh = 390.0f;

// ============================================================================
// Motor identity
// ============================================================================
/// Pole-pair count for commutation (14 rotor poles → 7 pairs).
inline constexpr uint8_t kDefaultPolePairs = 7;
/// Bench PWM frequency (Hz). 25 kHz mirrors the EvKit example and is well
/// above audible band; 20 kHz also acceptable.
inline constexpr uint32_t kPwmFrequencyHz = 25000;
/// FOC `OUTPUT_VOLTAGE_LIMIT` — keep at driver default for first runs.
inline constexpr uint16_t kOutputVoltageLimit = 8000;

// ============================================================================
// Current limits (parameter-mode `MAX_TORQUE` / `MAX_FLUX`)
// ============================================================================
/// Software clamp on torque current (mA). Conservative for a 30 W motor.
inline constexpr uint16_t kMaxPhaseCurrentMa = 1500;
/// Field weakening / flux current cap (mA). 0 unless you actively use field
/// weakening — small non-zero is harmless.
inline constexpr uint16_t kMaxFluxCurrentMa = 200;
/// Continuous I²t window 1 (matches `kMaxPhaseCurrentMa` thermal envelope).
inline constexpr float kI2tCurrent1A = 1.5f;
inline constexpr uint16_t kI2tWindow1Ms = 100;
/// Continuous I²t window 2 (longer-term).
inline constexpr float kI2tCurrent2A = 1.25f;
inline constexpr uint16_t kI2tWindow2Ms = 1000;

// ============================================================================
// Thermal protection
// ============================================================================
inline constexpr float kChipWarningTempC = 80.0f;
inline constexpr float kChipShutdownTempC = 100.0f;

// ============================================================================
// Velocity profile defaults (TMCL internal units)
// ============================================================================
/// Open-loop (FOC voltage mode) velocity command for first spin.
inline constexpr int32_t kOpenLoopTargetVelocity = 350;
/// Hall-FOC velocity command (still well below max RPM).
inline constexpr int32_t kHallFocTargetVelocity = 600;
/// ABN-FOC velocity command (assumes encoder + bootloader pin mux).
inline constexpr int32_t kAbnFocTargetVelocity = 500;
/// Torque command for FOC torque-mode demo (mA, signed).
inline constexpr int16_t kTorqueModeTargetMa = 250;

// ============================================================================
// Profile timings (ms)
// ============================================================================
/// Hard ceiling for any motion segment in a bench app.
inline constexpr uint32_t kMotorProfileMaxDurationMs = 5000;
/// Settle delay after mode change before commanding velocity.
inline constexpr uint32_t kSettleAfterModeChangeMs = 300;
/// Spin time for the open-loop bench (kept short).
inline constexpr uint32_t kOpenLoopSpinDurationMs = 1500;
/// Spin time for FOC bench profiles.
inline constexpr uint32_t kFocSpinDurationMs = 2000;
/// Telemetry poll period inside spin loops (ms).
inline constexpr uint32_t kTelemetryPollPeriodMs = 200;

// ============================================================================
// Encoder profile (used by ABN bench app)
// ============================================================================
/// Counts per revolution for the ABN encoder on the Vortex test bench.
inline constexpr uint32_t kAbnEncoderCountsPerRev = 1024;

}  // namespace vortex_bench_safety
