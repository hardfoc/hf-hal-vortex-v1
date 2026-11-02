/**
 * @file VortexBootReport.h
 * @brief Fixed-size record of what failed and what was degraded during Vortex::EnsureInitialized().
 *
 * @details Holds string literals only (component names and warning texts with static storage), so it
 *          never allocates and never copies text. Adding the same pointer twice is a no-op, so a retry
 *          of EnsureInitialized() after a failure does not list a component twice. Entries beyond the
 *          capacity are counted in Dropped() rather than lost silently.
 *
 *          Ownership: value type, owned by Vortex. Thread safety: none; Vortex guards it with its
 *          lifecycle mutex. MCU-free.
 */

#ifndef VORTEX_BOOT_REPORT_H_
#define VORTEX_BOOT_REPORT_H_

#include <array>
#include <cstddef>

class VortexBootReport {
public:
    /** @brief Capacity of each list (one entry per manager is enough for failures). */
    static constexpr size_t kCapacity = 12;

    /** @brief Forget every entry (start of a new initialisation attempt). */
    void Clear() noexcept {
        failed_count_ = 0;
        warning_count_ = 0;
        dropped_ = 0;
    }

    /**
     * @brief Record a component that failed to initialise.
     * @param component String literal naming the component, e.g. "MotorController".
     * @return true if recorded or already present; false if the list is full (counted in Dropped()).
     */
    bool AddFailed(const char* component) noexcept { return Add(failed_, failed_count_, component); }

    /**
     * @brief Record a warning (a degraded subsystem or an unusual condition that did not fail init).
     * @param text String literal with static storage.
     * @return true if recorded or already present; false if the list is full (counted in Dropped()).
     */
    bool AddWarning(const char* text) noexcept { return Add(warnings_, warning_count_, text); }

    [[nodiscard]] size_t FailedCount() const noexcept { return failed_count_; }
    [[nodiscard]] size_t WarningCount() const noexcept { return warning_count_; }
    [[nodiscard]] size_t Dropped() const noexcept { return dropped_; }
    [[nodiscard]] const char* Failed(size_t i) const noexcept { return i < failed_count_ ? failed_[i] : nullptr; }
    [[nodiscard]] const char* Warning(size_t i) const noexcept { return i < warning_count_ ? warnings_[i] : nullptr; }

    /** @brief Copy up to @p max failed-component names into @p out. @return number copied. */
    size_t CopyFailed(const char* out[], size_t max) const noexcept { return Copy(failed_, failed_count_, out, max); }

    /** @brief Copy up to @p max warnings into @p out. @return number copied. */
    size_t CopyWarnings(const char* out[], size_t max) const noexcept {
        return Copy(warnings_, warning_count_, out, max);
    }

private:
    using List = std::array<const char*, kCapacity>;

    bool Add(List& list, size_t& count, const char* text) noexcept {
        if (text == nullptr) {
            return false;
        }
        for (size_t i = 0; i < count; ++i) {
            if (list[i] == text) {
                return true;
            }
        }
        if (count >= list.size()) {
            ++dropped_;
            return false;
        }
        list[count++] = text;
        return true;
    }

    static size_t Copy(const List& list, size_t count, const char* out[], size_t max) noexcept {
        if (out == nullptr) {
            return 0;
        }
        const size_t n = count < max ? count : max;
        for (size_t i = 0; i < n; ++i) {
            out[i] = list[i];
        }
        return n;
    }

    List failed_{};
    List warnings_{};
    size_t failed_count_{0};
    size_t warning_count_{0};
    size_t dropped_{0};
};

#endif  // VORTEX_BOOT_REPORT_H_
