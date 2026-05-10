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
 * you turn on DRV_EN. See **`examples/esp32/docs/BLDC_BRINGUP.md`** for the
 * full bootloader vs runtime picture.
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
/// Software clamp on torque current (mA). Bumped 1500 → 2500 mA on 2026-05
/// to give `OPENLOOP_CURRENT` (2000 mA) headroom — see `kOpenLoopCurrentMa`.
inline constexpr uint16_t kMaxPhaseCurrentMa = 2500;
/// Field weakening / flux current cap (mA). In `FOC_OPENLOOP_CURRENT_MODE` the
/// commanded `OPENLOOP_CURRENT` is driven entirely along the d-axis (flux),
/// so this **also clamps the open-loop bench current**: keep it ≥ whatever
/// `kOpenLoopCurrentMa` / `kSafeOpenLoopCurrentMa` the bench apps command.
/// 2200 mA gives headroom for the 2 A open-loop command while staying
/// under the bumped 2.5 A `kI2tCurrent1A` window and the 3 A
/// `kExpectedPeakCurrentA` rail. Raised 1200 → 2200 mA on 2026-05.
inline constexpr uint16_t kMaxFluxCurrentMa = 2200;
/// Continuous I²t window 1 (short thermal envelope). The TMC9660 ROM rejects
/// `THERMAL_WINDING_TIME_CONSTANT_1 < ~1 s` with REPLY_INVALID_VALUE — keep
/// well above that floor. Defaults are 3 s / 6 s; we run a touch tighter to
/// trip earlier on a stalled rotor.
inline constexpr float kI2tCurrent1A = 2.5f;
inline constexpr uint16_t kI2tWindow1Ms = 2000;
/// Continuous I²t window 2 (longer-term).
inline constexpr float kI2tCurrent2A = 2.0f;
inline constexpr uint16_t kI2tWindow2Ms = 5000;

// ============================================================================
// Thermal protection
// ============================================================================
inline constexpr float kChipWarningTempC = 80.0f;
inline constexpr float kChipShutdownTempC = 100.0f;

// ============================================================================
// Velocity profile defaults — expressed in real-world units
// ============================================================================
//
// Bench code converts these into the chip's opaque internal units via
// `tmc9660::units::velocityToInternal(...)` /
// `tmc9660::units::accelerationToInternal(...)` using the active
// `tmc9660::units::MotorContext` (pole-pairs + sensor selection).
// See `tmc9660_units.hpp` (vendor library) for the full datasheet derivation
// (k_RPM = CPR × 2^24 / (40 MHz × 60), p. 69 of the TMC9660 Parameter Mode
// Reference Manual).
//
// The constants below are **the canonical bench defaults**: tweak these
// to change the spin profile. The internal-unit values are derived once,
// at use site, from the active MotorContext.

/// Open-loop spin target velocity at the **load (output)** shaft, RPM.
/// 50 load-RPM \u2192 500 motor-RPM through the 10:1 reducer. Smooth-rotation
/// regime for open-loop voltage mode at 1500/16383 modulation on a 24 V bus
/// (back-EMF at 500 motor-RPM \u2248 0.16 V \u2014 still well below the \u22482.2 V
/// applied fundamental, so plenty of torque margin against gearbox cogging).
inline constexpr double kOpenLoopTargetRpm = 50.0;
/// Floor below which `setTargetVelocity` writes are coalesced/rejected by
/// the chip while the FOC pipeline is still arming. Roughly 0.3 RPM
/// mechanical (1000 internal units at pp=7) — keep small but non-zero.
inline constexpr double kOpenLoopMinCommandRpm = 0.3;
/// Ramper VMAX in **motor-frame** RPM. Must be ≥ (load_target_rpm × gear_ratio).
/// With the 10:1 bench gearbox and `kOpenLoopTargetRpm = 80` (load), the motor
/// shaft cruises at 800 RPM — so 1000 RPM gives 25% headroom.
inline constexpr double kOpenLoopRampMaxRpm = 1000.0;
/// Ramper acceleration / deceleration in **motor-frame** RPM per second.
/// 125 motor-RPM/s ramps the motor 0 \u2192 500 RPM in 4 s (= load 0 \u2192 50 RPM in
/// 4 s through a 10:1 reducer), leaving ~6 s of cruise inside the 10 s
/// `kOpenLoopSpinDurationMs` window.
///
/// 2026-05 datasheet-math fix: prior driver versions wrote `rpm/s \u00d7 k_RPM`
/// directly to RAMP_AMAX, missing the chip's `2^17 / fCLK` factor (\u22480.00328).
/// 15 RPM/s on the old code was therefore actually executing at \u22484577 RPM/s
/// (instant jolt, slip-and-recover stutter). With the corrected math 125 RPM/s
/// is a smooth 4-second ramp.
inline constexpr double kOpenLoopRampMaxRpmPerSec = 125.0;

/// Hall-FOC velocity target (RPM, mechanical).
inline constexpr double kHallFocTargetRpm = 200.0;
/// ABN-FOC velocity target (RPM, mechanical).
inline constexpr double kAbnFocTargetRpm = 150.0;
/// Open-loop PWM magnitude (voltage mode): `OPENLOOP_VOLTAGE` 0..16383, where
/// 16383 ≈ 100% modulation on the bus. 1500 ≈ 9.2% modulation ≈ 2.2 V peak
/// fundamental on a 24 V supply — enough to maintain phase lock on a geared
/// rotor up to ~200 motor-RPM, where back-EMF is still small relative to
/// applied voltage. Bumped 800 → 1500 on 2026-05 because at 5% modulation
/// the geared rotor (10:1 reducer) only produced phase-locked torque at very
/// low speed and stalled into vibration once the ramp climbed.
/// Stays well under any current limit (line-line resistance ~1.6 Ω → stall
/// current at 2.2 V is ~1.4 A, sustainable continuously).
inline constexpr uint16_t kOpenLoopVoltage = 1500u;
/// Open-loop current (current mode): `OPENLOOP_CURRENT` in mA — must stay <=
/// kMaxFluxCurrentMa. Bumped 150 → 700 → 1000 → 2000 mA across 2026-05
/// bench iterations: at 1000 mA the 30 W bench motor's actual rotor only
/// followed ~0.5 mech rev while the chip's open-loop `phi_e` rotated 12+
/// revs (massive slip). 2000 mA is the motor's continuous rating; stays
/// under `kMaxFluxCurrentMa` (4000 mA), the I²t window (1.5 A / 2 s
/// ceiling for short bursts is overridden by the higher continuous limit),
/// and the `MAX_TORQUE` clamp. Raise only with thermal awareness (stalled
/// rotor at 2 A dissipates significant copper loss).
inline constexpr uint16_t kOpenLoopCurrentMa = 2000u;
/// Torque command for FOC torque-mode demo (mA, signed).
inline constexpr int16_t kTorqueModeTargetMa = 250;

// ============================================================================
// Profile timings (ms)
// ============================================================================
/// Hard ceiling for any motion segment in a bench app.
inline constexpr uint32_t kMotorProfileMaxDurationMs = 8000;
/// Settle delay after mode change before commanding velocity.
inline constexpr uint32_t kSettleAfterModeChangeMs = 300;
/// Erratum 4 (Parameter Mode): first target torque/flux/velocity/position
/// updates can be delayed up to ~1 ms inside a 1 kHz loop. Allow a few ms
/// after changing prerequisites so the first `TARGET_VELOCITY` SAP is not
/// racing the shadow register.
inline constexpr uint32_t kErratum4TargetCoalesceMs = 5;
/// Brief pause after `RAMP_ENABLE` / `DIRECT_VELOCITY_MODE` tweaks before a
/// velocity SAP (same 1 kHz scheduling class as Erratum 4 on some builds).
inline constexpr uint32_t kAfterRampDvmTweakMs = 5;
/// Spin time for the open-loop bench. Long enough to (a) finish the
/// hardware ramp 0 → kOpenLoopTargetVelocity at kOpenLoopRampMaxRpmPerSec
/// (~5.3 s for 80 RPM @ 15 RPM/s) and (b) leave ~5 s of visible spin
/// at the cruise velocity for the human observer at the bench.
inline constexpr uint32_t kOpenLoopSpinDurationMs = 10000;
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
