/**
 * @file vortex_bench_safety.hpp
 * @brief Bench safety constants for Vortex motor bring-up (24 V bench, low current).
 */
#pragma once

#include <cstdint>

namespace vortex_bench_safety {

/// Nominal DC bus for documentation (volts).
inline constexpr float kNominalSupplyVolts = 24.0f;

/// Phase / torque current ceiling for initial bring-up (milliamperes).
inline constexpr uint16_t kMaxPhaseCurrentMa = 1000;

/// Default pole-pair count for small BLDC on bench (override per motor).
inline constexpr uint8_t kDefaultPolePairs = 7;

/// Open-loop / profile time limits (milliseconds).
inline constexpr uint32_t kMotorProfileMaxDurationMs = 5000;

}  // namespace vortex_bench_safety
