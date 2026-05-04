/**
 * @file VortexVersionManifest.h
 * @brief HAL-side inventory of components the Vortex HAL links against.
 *
 * Git short-SHAs come from `vortex_hal_git_manifest.h`, written at HAL
 * configure time against the checked-out HEAD of each submodule.
 *
 * @author HardFOC Team
 * @date 2026
 */

#ifndef VORTEX_VERSION_MANIFEST_H_
#define VORTEX_VERSION_MANIFEST_H_

#include <cstddef>

namespace vortex_version {

struct Entry {
    const char* name;
    const char* version;
    const char* git_sha;
};

struct EntrySpan {
    const Entry* data;
    std::size_t  size;

    constexpr const Entry* begin() const noexcept { return data; }
    constexpr const Entry* end()   const noexcept { return data + size; }
};

EntrySpan GetHalManifest() noexcept;
const char* GetHalBuildStamp() noexcept;

}  // namespace vortex_version

#endif  // VORTEX_VERSION_MANIFEST_H_
