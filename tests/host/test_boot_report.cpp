/**
 * @file test_boot_report.cpp
 * @brief VortexBootReport: what Vortex::GetFailedComponents() and GetSystemWarnings() return.
 */

#include "HostTest.h"
#include "VortexBootReport.h"

HF_TEST(BootReport_RecordsFailuresAndWarnings) {
    VortexBootReport r;
    CHECK(r.AddFailed("MotorController"));
    CHECK(r.AddWarning("IMU not ready"));
    CHECK(r.AddWarning("Encoders not ready"));
    CHECK_EQ(r.FailedCount(), 1u);
    CHECK_EQ(r.WarningCount(), 2u);

    const char* out[4] = {};
    CHECK_EQ(r.CopyWarnings(out, 4), 2u);
    CHECK(out[0] == r.Warning(0));
    CHECK(out[1] == r.Warning(1));
    CHECK_EQ(r.CopyFailed(out, 4), 1u);
    CHECK(out[0] == r.Failed(0));
}

HF_TEST(BootReport_SameEntryTwiceIsListedOnce) {
    // EnsureInitialized() may be retried after a failure; a component must not appear twice.
    static const char* const kName = "AdcManager";
    VortexBootReport r;
    CHECK(r.AddFailed(kName));
    CHECK(r.AddFailed(kName));
    CHECK_EQ(r.FailedCount(), 1u);
}

HF_TEST(BootReport_CopyHonoursTheCallerCapacity) {
    VortexBootReport r;
    (void)r.AddWarning("a");
    (void)r.AddWarning("b");
    (void)r.AddWarning("c");
    const char* out[2] = {};
    CHECK_EQ(r.CopyWarnings(out, 2), 2u);
    CHECK_EQ(r.CopyWarnings(nullptr, 2), 0u);
}

HF_TEST(BootReport_OverflowIsCountedNotLost) {
    static const char* const kTexts[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13"};
    VortexBootReport r;
    size_t accepted = 0;
    for (const char* t : kTexts) {
        accepted += r.AddWarning(t) ? 1 : 0;
    }
    CHECK_EQ(accepted, VortexBootReport::kCapacity);
    CHECK_EQ(r.WarningCount(), VortexBootReport::kCapacity);
    CHECK_EQ(r.Dropped(), (sizeof(kTexts) / sizeof(kTexts[0])) - VortexBootReport::kCapacity);
}

HF_TEST(BootReport_ClearStartsANewAttempt) {
    VortexBootReport r;
    (void)r.AddFailed("GpioManager");
    (void)r.AddWarning("x");
    r.Clear();
    CHECK_EQ(r.FailedCount(), 0u);
    CHECK_EQ(r.WarningCount(), 0u);
    CHECK(r.Failed(0) == nullptr);
    CHECK(!r.AddFailed(nullptr));
}
