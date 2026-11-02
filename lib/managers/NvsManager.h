/**
 * @file NvsManager.h
 * @brief Non-volatile storage for Vortex V1 — ESP-IDF NVS behind `BaseNvs`.
 *
 * @details Exposes three fixed NVS namespaces (created by the board binding) for application data,
 *          calibration blobs, and device configuration. Upper layers should use
 *          `BaseNvs&` from this manager (not IDF NVS directly) so a future
 *          EEPROM-backed `BaseNvs` can be swapped in without changing callers.
 *
 * @author HardFOC Team
 * @date 2026
 */

#ifndef VORTEX_NVS_MANAGER_H_
#define VORTEX_NVS_MANAGER_H_

#include <atomic>
#include <cstdint>

#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseNvs.h"

/**
 * @class NvsManager
 * @brief Singleton giving access to the Vortex board persistent-storage namespaces.
 *
 * Initialization runs **before** communication managers so calibration or
 * board identity can be read early. Namespaces (≤15 chars per ESP-IDF):
 *   - `"vortex"`      — general application key-value
 *   - `"vortex_cal"` — calibration / factory tuning blobs
 *   - `"vortex_cfg"` — structured device configuration
 */
class NvsManager {
public:
    static NvsManager& GetInstance() noexcept;

    NvsManager(const NvsManager&) = delete;
    NvsManager& operator=(const NvsManager&) = delete;
    NvsManager(NvsManager&&) = delete;
    NvsManager& operator=(NvsManager&&) = delete;

    /** @brief Initialize NVS flash (once) and open all Vortex namespaces. */
    [[nodiscard]] bool EnsureInitialized() noexcept;

    [[nodiscard]] bool IsInitialized() const noexcept {
        return initialized_.load(std::memory_order_acquire);
    }

    /** @brief Close namespaces (call after other subsystems if they may persist on shutdown). */
    [[nodiscard]] bool Deinitialize() noexcept;

    /** @brief Last NVS error from the most recent failing operation (any region). */
    [[nodiscard]] hf_nvs_err_t GetLastError() const noexcept { return last_error_; }

    /** @brief General application storage (`vortex` namespace). */
    [[nodiscard]] BaseNvs& App() noexcept { return app_; }

    /** @brief Calibration and tuning blobs (`vortex_cal` namespace). */
    [[nodiscard]] BaseNvs& Calibration() noexcept { return calibration_; }

    /** @brief Device configuration (`vortex_cfg` namespace). */
    [[nodiscard]] BaseNvs& Config() noexcept { return config_; }

    void DumpStatistics() const noexcept;

private:
    NvsManager() noexcept;
    ~NvsManager() = default;

    void SetLastError(hf_nvs_err_t e) noexcept { last_error_ = e; }

    // Owned by the board binding (bsp_bind/VortexBoardBind.h); constructing them touches no hardware.
    BaseNvs& app_;
    BaseNvs& calibration_;
    BaseNvs& config_;

    std::atomic<bool> initialized_{false};
    hf_nvs_err_t last_error_{hf_nvs_err_t::NVS_SUCCESS};
};

#endif // VORTEX_NVS_MANAGER_H_
