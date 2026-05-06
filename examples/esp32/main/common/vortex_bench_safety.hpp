/**
 * @file vortex_bench_safety.hpp
 * @brief Bench safety constants for Vortex motor bring-up (24 V bench, low current).
 *
 * Default numbers match the on-hand **24 V / 30 W** geared BLDC (10:1, 14 rotor poles →
 * 7 pole pairs). Tune `kOpenLoopTargetVelocity` after confirming TMCL velocity units vs tach.
 */
#pragma once

#include <cstdint>

namespace vortex_bench_safety {

/// Nominal DC bus for documentation (volts).
inline constexpr float kNominalSupplyVolts = 24.0f;

/// Phase / torque current ceiling for bring-up (~30 W @ 24 V class; stay below thermal stall).
inline constexpr uint16_t kMaxPhaseCurrentMa = 1200;

/// Rotor pole-pair count for BLDC commutation (14 magnetic poles on rotor → 7 pairs).
inline constexpr uint8_t kDefaultPolePairs = 7;

/// Open-loop / profile time limits (milliseconds) — hard ceiling for bench apps.
inline constexpr uint32_t kMotorProfileMaxDurationMs = 5000;

/// `setTargetVelocity()` argument for `vortex_bldc_open_loop` (TMCL internal scaling; start low).
inline constexpr int32_t kOpenLoopTargetVelocity = 350;

/// How long to hold velocity before `motor_stop_safe` in open-loop bench (ms).
inline constexpr uint32_t kOpenLoopSpinDurationMs = 1500;

/// Delay after motor init / mode change before commanding velocity (ms).
inline constexpr uint32_t kOpenLoopSettleBeforeSpinMs = 300;

}  // namespace vortex_bench_safety
