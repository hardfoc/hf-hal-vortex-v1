/**
 * @file vortex_api_test_strict_hw.hpp
 * @brief Build-time gates for strict hardware assertions in vortex_api_test.
 *
 * Enable via `idf.py menuconfig` → *Component config* → *Main* → *Vortex API test*,
 * or add to sdkconfig / sdkconfig.defaults (see `main/Kconfig.projbuild`):
 *   CONFIG_VORTEX_API_TEST_STRICT_HW=y
 *   CONFIG_VORTEX_API_TEST_STRICT_SENSORS=y   (optional, depends on STRICT_HW)
 */
#pragma once

#include "sdkconfig.h"

#ifndef CONFIG_VORTEX_API_TEST_STRICT_HW
#define CONFIG_VORTEX_API_TEST_STRICT_HW 0
#endif
#ifndef CONFIG_VORTEX_API_TEST_STRICT_SENSORS
#define CONFIG_VORTEX_API_TEST_STRICT_SENSORS 0
#endif

namespace vortex_api_test_strict {

inline constexpr bool kHwChecks = (CONFIG_VORTEX_API_TEST_STRICT_HW != 0);
inline constexpr bool kSensorChecks = (CONFIG_VORTEX_API_TEST_STRICT_SENSORS != 0);

/// DC bus lower bound for strict VM read (volts); allows 12 V bench.
inline constexpr float kVmVoltsMin = 12.0f;
/// DC bus upper bound for strict VM read (volts).
inline constexpr float kVmVoltsMax = 32.0f;

}  // namespace vortex_api_test_strict
