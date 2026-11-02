/**
 * @file test_lock_discipline.cpp
 * @brief Driver access goes through the managers' visitors, which refuse absent or unready devices.
 *
 * The host RTOS layer has no scheduler (its mutex is a no-op), so these tests cover the contracts
 * a visitor gives a caller (runs only on a ready device, returns a default otherwise); the lock
 * order itself (manager lock, then handler lock) is reviewed and documented on the Threading page.
 */

#include "HostTest.h"
#include "VortexBoardBindHost.h"
#include "api/Vortex.h"
#include "managers/ImuManager.h"
#include "managers/MotorController.h"

namespace host = vortex::bsp::host;

HF_TEST(Imu_VisitSensorRefusesAnAbsentImu) {
    host::Reset();
    auto& v = Vortex::GetInstance();
    REQUIRE(v.EnsureInitialized());
    bool ran = false;
    CHECK(!ImuManager::GetInstance().VisitSensor(ImuManager::ONBOARD_IMU_INDEX,
                                                 [&](IBno08xDriverOps&) { ran = true; }));
    CHECK(!ran);
    CHECK(!ImuManager::GetInstance().VisitSensor(ImuManager::MAX_IMU_DEVICES, [&](IBno08xDriverOps&) { ran = true; }));
    CHECK(!ran);
    (void)v.Shutdown();
}

HF_TEST(Motor_VisitDriverReturnsDefaultWithoutADevice) {
    host::Reset();
    auto& motors = MotorController::GetInstance();
    bool ran = false;
    const int r = motors.visitDriver([&](auto&) {
        ran = true;
        return 42;
    });
    CHECK_EQ(r, 0);
    CHECK(!ran);
    tmc9660::TmclSpiStats stats{};
    CHECK(motors.GetTmclSpiStats(stats) == MotorError::DEVICE_NOT_FOUND);
}
