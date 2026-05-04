/**
 * @file VortexVersion.h
 * @brief Version information for the Vortex HAL and API.
 */

#ifndef VORTEX_VERSION_H_
#define VORTEX_VERSION_H_

// HAL / component package (ESP-IDF Component Manager, manifest)
#define VORTEX_HAL_VERSION_MAJOR    1
#define VORTEX_HAL_VERSION_MINOR    1
#define VORTEX_HAL_VERSION_PATCH    0
#define VORTEX_HAL_VERSION_STRING   "1.1.0"
#define VORTEX_HAL_BOARD_NAME       "HardFOC Vortex V1"
#define VORTEX_HAL_MCU              "ESP32-C6"

// Application-facing API semver (may diverge from HAL package)
#define VORTEX_API_VERSION_MAJOR 2
#define VORTEX_API_VERSION_MINOR 0
#define VORTEX_API_VERSION_PATCH 0
#define VORTEX_API_VERSION_STRING "2.0.0"

#define VORTEX_API_BUILD_DATE __DATE__
#define VORTEX_API_BUILD_TIME __TIME__

#define VORTEX_API_FEATURE_DIAGNOSTICS 1
#define VORTEX_API_FEATURE_HEALTH_CHECK 1
#define VORTEX_API_FEATURE_STATISTICS 1

#endif  // VORTEX_VERSION_H_
