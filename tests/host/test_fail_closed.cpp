/**
 * @file test_fail_closed.cpp
 * @brief Fail-closed initialisation, aggregated diagnostics and static storage of the Vortex HAL.
 *
 * Each case starts from a reset host binding and ends with the facade shut down, because the
 * managers are process-wide singletons.
 */

#include "HostTest.h"
#include "VortexBoardBindHost.h"
#include "api/Vortex.h"
#include "managers/CommChannelsManager.h"
#include "managers/GpioManager.h"
#include "managers/LedManager.h"
#include "managers/NvsManager.h"

#include <atomic>
#include <cstdlib>
#include <new>

namespace host = vortex::bsp::host;

//------------------------------------------------------------------------------
// Allocation counter: global operator new, counted only while armed.
//------------------------------------------------------------------------------
namespace {
std::atomic<bool> g_count_allocations{false};
std::atomic<size_t> g_allocations{0};
}  // namespace

void* operator new(size_t size) {
    if (g_count_allocations.load(std::memory_order_relaxed)) {
        g_allocations.fetch_add(1, std::memory_order_relaxed);
    }
    if (void* p = std::malloc(size == 0 ? 1 : size)) {
        return p;
    }
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }

namespace {

Vortex& Fresh() {
    auto& v = Vortex::GetInstance();
    if (v.IsInitialized()) {
        (void)v.Shutdown();
    }
    host::Reset();
    return v;
}

void Done(Vortex& v) {
    if (v.IsInitialized()) {
        (void)v.Shutdown();
    }
    host::Reset();
}

}  // namespace

HF_TEST(FailClosed_NoBusShutsEverythingDownAgain) {
    Vortex& v = Fresh();
    host::Fail().spi = true;
    host::Fail().i2c = true;
    CHECK(!v.EnsureInitialized());
    CHECK(!v.IsInitialized());
    CHECK(v.GetLastError() == ManagerError::BusUnavailable);
    // NVS came up first and was rolled back; comms closed the UART and CAN it had opened.
    CHECK(!NvsManager::GetInstance().IsInitialized());
    CHECK(!CommChannelsManager::GetInstance().IsInitialized());
    CHECK(host::Uart() == nullptr);
    CHECK(host::Can() == nullptr);

    VortexBoardDiagnostics d{};
    CHECK(!v.CollectDiagnostics(d));
    CHECK(d.last_error == ManagerError::BusUnavailable);
    CHECK_EQ(d.managers_up, 0u);
    for (const auto& m : d.managers) {
        CHECK(!m.initialized);
        CHECK(m.error != ManagerError::None);
    }
    Done(v);
}

HF_TEST(FailClosed_RequiredLedFailureRollsBackGpioAndComms) {
    Vortex& v = Fresh();
    host::Fail().led = true;
    CHECK(!v.EnsureInitialized());
    CHECK(v.GetLastError() == ManagerError::InitFailed);
    CHECK(!GpioManager::GetInstance().IsInitialized());
    CHECK(host::Gpio(HfFunctionalGpioPin::EXT_GPIO_CS_1) == nullptr);  // MCU pins released
    CHECK(host::Uart() == nullptr);
    CHECK(!host::LedOpen());
    Done(v);
}

HF_TEST(FailClosed_RetryAfterTheFaultIsClearedSucceeds) {
    Vortex& v = Fresh();
    host::Fail().led = true;
    CHECK(!v.EnsureInitialized());
    host::Fail().led = false;
    CHECK(v.EnsureInitialized());
    CHECK(v.GetLastError() == ManagerError::None);
    const char* failed[VortexBootReport::kCapacity] = {};
    CHECK_EQ(v.GetFailedComponents(failed, VortexBootReport::kCapacity),
             v.GetFailedComponents(failed, VortexBootReport::kCapacity));
    Done(v);
}

HF_TEST(Diagnostics_AggregateAfterADegradedBoot) {
    Vortex& v = Fresh();
    REQUIRE(v.EnsureInitialized());
    VortexBoardDiagnostics d{};
    CHECK(v.CollectDiagnostics(d));
    CHECK(d.last_error == ManagerError::None);
    // Required managers, in init order.
    for (size_t i = 0; i < 4; ++i) {
        CHECK(d.managers[i].required);
        CHECK(d.managers[i].initialized);
        CHECK_EQ(d.managers[i].error, ManagerError::None);
        CHECK(d.managers[i].name != nullptr);
    }
    CHECK(d.managers_up >= 4u);
    CHECK(d.spi_ok);
    CHECK(d.i2c_ok);
    CHECK(d.uart_ok);
    CHECK(d.can_ok);
    CHECK(d.gpio_pins > 0u);
    CHECK_EQ(d.motor_devices_ready, 0u);  // no TMC9660 on the host
    CHECK(d.warnings >= 1u);
    CHECK(!d.managers[6].initialized);  // IMU: no BNO08x answers
    CHECK(d.managers[6].error != ManagerError::None);
    Done(v);
}

HF_TEST(Gpio_PointersAreStableAndClearedOnShutdown) {
    Vortex& v = Fresh();
    REQUIRE(v.EnsureInitialized());
    auto& gpio = GpioManager::GetInstance();
    BaseGpio* a = gpio.Get(HfFunctionalGpioPin::EXT_GPIO_CS_1);
    REQUIRE(a != nullptr);
    CHECK(gpio.Get("GPIO_EXT_GPIO_CS_1") == a);
    CHECK(a == static_cast<BaseGpio*>(host::Gpio(HfFunctionalGpioPin::EXT_GPIO_CS_1)));
    // No TMC9660 came up, so its bridge pins are not handed out (Tmc9660Handler::gpio() would
    // dereference wrappers that only exist after a successful bring-up).
    CHECK(gpio.Get(HfFunctionalGpioPin::TMC_GPIO17) == nullptr);
    Done(v);
    CHECK(gpio.Get(HfFunctionalGpioPin::EXT_GPIO_CS_1) == nullptr);
}

HF_TEST(StaticAllocation_SteadyStateCallsDoNotAllocate) {
    Vortex& v = Fresh();
    REQUIRE(v.EnsureInitialized());
    auto& leds = LedManager::GetInstance();
    auto& gpio = GpioManager::GetInstance();
    auto& comms = CommChannelsManager::GetInstance();
    // Warm up once (first-use statics inside the host mocks), then count.
    (void)leds.SetColor(LedColors::BLUE);

    g_allocations.store(0);
    g_count_allocations.store(true);
    for (int i = 0; i < 10; ++i) {
        (void)leds.SetColor(i % 2 ? LedColors::RED : LedColors::GREEN);
        (void)leds.SetBrightness(static_cast<uint8_t>(10 * i));
        bool active = false;
        (void)gpio.Read("GPIO_EXT_GPIO_CS_2", active);
        (void)gpio.Get(HfFunctionalGpioPin::EXT_GPIO_CS_1);
        (void)comms.GetSpiDevice(SpiDeviceId::AS5047U_POSITION_ENCODER);
        VortexBoardDiagnostics d{};
        (void)v.CollectDiagnostics(d);
        const char* w[4];
        (void)v.GetSystemWarnings(w, 4);
    }
    g_count_allocations.store(false);
    CHECK_EQ(g_allocations.load(), 0u);
    Done(v);
}
