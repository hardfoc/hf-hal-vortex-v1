/**
 * @file VortexVersionManifest.cpp
 * @brief Static manifest of components the Vortex HAL links against.
 */

#include "VortexVersionManifest.h"
#include "VortexVersion.h"
#include "vortex_hal_git_manifest.h"

#if __has_include("ntc_thermistor_version.h")
#  include "ntc_thermistor_version.h"
#  define _HF_VS_NTC_THERMISTOR HF_NTC_THERMISTOR_VERSION_STRING
#else
#  define _HF_VS_NTC_THERMISTOR "n/a"
#endif

#if __has_include("ws2812_version.h")
#  include "ws2812_version.h"
#  define _HF_VS_WS2812_RMT HF_WS2812_RMT_VERSION_STRING
#else
#  define _HF_VS_WS2812_RMT "n/a"
#endif

#if __has_include("iiwrap_version.h")
#  include "iiwrap_version.h"
#  define _HF_VS_IIWRAP HF_IIWRAP_VERSION_STRING
#else
#  define _HF_VS_IIWRAP "n/a"
#endif

#if __has_include("pincfg_version.h")
#  include "pincfg_version.h"
#  define _HF_VS_PINCFG HF_PINCFG_VERSION_STRING
#else
#  define _HF_VS_PINCFG "n/a"
#endif

namespace vortex_version {

namespace {

constexpr Entry kHalManifest[] = {
    {"hf-hal-vortex-v1",   VORTEX_HAL_VERSION_STRING, HF_HAL_GIT_SHA_HAL},

    {"hf-core",            "n/a",                    HF_HAL_GIT_SHA_HF_CORE},
    {"hf-core-drivers",    "n/a",                    HF_HAL_GIT_SHA_HF_CORE_DRIVERS},
    {"hf-core-utils",      "n/a",                    HF_HAL_GIT_SHA_HF_CORE_UTILS},

    {"tmc9660",            "n/a",                    HF_HAL_GIT_SHA_TMC9660},
    {"pcal95555",          "n/a",                    HF_HAL_GIT_SHA_PCAL95555},
    {"as5047u",            "n/a",                    HF_HAL_GIT_SHA_AS5047U},
    {"bno08x",             "n/a",                    HF_HAL_GIT_SHA_BNO08X},
    {"ntc-thermistor",     _HF_VS_NTC_THERMISTOR,    HF_HAL_GIT_SHA_NTC_THERMISTOR},
    {"ws2812-rmt",         _HF_VS_WS2812_RMT,        HF_HAL_GIT_SHA_WS2812_RMT},

    {"hf-iiwrap",          _HF_VS_IIWRAP,            HF_HAL_GIT_SHA_IIWRAP},
    {"hf-pincfg",          _HF_VS_PINCFG,            HF_HAL_GIT_SHA_PINCFG},

    {"hf-utils-canopen",   "n/a",                    HF_HAL_GIT_SHA_UTIL_CANOPEN},
    {"hf-utils-general",   "n/a",                    HF_HAL_GIT_SHA_UTIL_GENERAL},
    {"hf-utils-rtos-wrap", "n/a",                    HF_HAL_GIT_SHA_UTIL_RTOS},
};

constexpr std::size_t kHalManifestCount =
    sizeof(kHalManifest) / sizeof(kHalManifest[0]);

}  // namespace

EntrySpan GetHalManifest() noexcept {
    return EntrySpan{kHalManifest, kHalManifestCount};
}

const char* GetHalBuildStamp() noexcept {
    return HF_HAL_BUILD_STAMP;
}

}  // namespace vortex_version
