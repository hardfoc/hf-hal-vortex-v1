/**
 * @file Vortex.cpp
 * @brief Implementation of the unified Vortex API singleton.
 * 
 * @details This file implements the Vortex API singleton with proper initialization
 *          order, dependency management, and comprehensive error handling.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 */

#include "Vortex.h"

#include <array>
#include <cstring>

// Include all component handler headers
#include "managers/NvsManager.h"
#include "managers/CommChannelsManager.h"
#include "managers/GpioManager.h"
#include "managers/AdcManager.h"
#include "managers/MotorController.h"
#include "managers/ImuManager.h"
#include "managers/EncoderManager.h"
#include "managers/LedManager.h"
#include "managers/TemperatureManager.h"

// Include OS abstraction for timing
#include "RtosMutex.h"

#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"

//==============================================================================
// SINGLETON IMPLEMENTATION
//==============================================================================

std::atomic<std::uint8_t> Vortex::onboard_tmc9660_transport_{
    static_cast<std::uint8_t>(VortexOnboardTmc9660Transport::Uart)};

Vortex& Vortex::GetInstance() noexcept {
    static Vortex instance;
    return instance;
}

void Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport transport) noexcept {
    if (GetInstance().initialized_.load(std::memory_order_acquire)) {
        Logger::GetInstance().Warn(
            "Vortex",
            "SetOnboardTmc9660Transport ignored — call before EnsureInitialized()");
        return;
    }
    onboard_tmc9660_transport_.store(static_cast<std::uint8_t>(transport), std::memory_order_relaxed);
}

VortexOnboardTmc9660Transport Vortex::GetOnboardTmc9660Transport() noexcept {
    return static_cast<VortexOnboardTmc9660Transport>(
        onboard_tmc9660_transport_.load(std::memory_order_relaxed));
}

//==============================================================================
// CONSTRUCTOR AND DESTRUCTOR
//==============================================================================

Vortex::Vortex() noexcept
    : comms(CommChannelsManager::GetInstance()),
      nvs(NvsManager::GetInstance()),
      gpio(GpioManager::GetInstance()),
      motors(MotorController::GetInstance()),
      adc(AdcManager::GetInstance()),
      imu(ImuManager::GetInstance()),
      encoders(EncoderManager::GetInstance()),
      leds(LedManager::GetInstance()),
      temp(TemperatureManager::GetInstance()),
      initialized_(false),
      initialization_start_time_(0),
      initialization_end_time_(0),
      nvs_initialized_(false),
      comms_initialized_(false),
      gpio_initialized_(false),
      motors_initialized_(false),
      adc_initialized_(false),
      imu_initialized_(false),
      encoders_initialized_(false),
      leds_initialized_(false),
      temp_initialized_(false),
      diagnostics_() {
    
    // Initialize Logger first (this is always available)
    Logger::GetInstance().Initialize();
    
    Logger::GetInstance().Info("Vortex", "Vortex API singleton created (%s)", VORTEX_HAL_VERSION_STRING);
}

//==============================================================================
// PUBLIC INITIALIZATION METHODS
//==============================================================================

bool Vortex::EnsureInitialized() noexcept {
    if (initialized_) {
        return true;
    }
    MutexLockGuard lock(lifecycle_mutex_);
    if (initialized_) {
        return true;  // another task finished initialisation while this one waited
    }
    
    Logger::GetInstance().Info("Vortex", "Starting Vortex API initialization");
    
    // Record initialization start time
    initialization_start_time_ = RtosTime::GetCurrentTimeUs() / 1000;
    
    // Initialize all components in proper order
    bool success = InitializeAllComponents();
    
    // Record initialization end time
    initialization_end_time_ = RtosTime::GetCurrentTimeUs() / 1000;
    
    if (success) {
        initialized_ = true;
        UpdateSystemDiagnostics();
        
        uint64_t init_time_ms = GetInitializationTimeMs();
        Logger::GetInstance().Info("Vortex", "Vortex API initialization completed successfully in %llu ms", init_time_ms);
        
        // Log component status
        Logger::GetInstance().Info("Vortex", "Component Status - NVS: %s, Comms: %s, GPIO: %s, Motors: %s, ADC: %s, IMU: %s, Encoders: %s, LEDs: %s, Temp: %s",
            nvs_initialized_ ? "OK" : "FAIL",
            comms_initialized_ ? "OK" : "FAIL",
            gpio_initialized_ ? "OK" : "FAIL",
            motors_initialized_ ? "OK" : "FAIL",
            adc_initialized_ ? "OK" : "FAIL",
            imu_initialized_ ? "OK" : "FAIL",
            encoders_initialized_ ? "OK" : "FAIL",
            leds_initialized_ ? "OK" : "FAIL",
            temp_initialized_ ? "OK" : "FAIL");
    } else {
        Logger::GetInstance().Error("Vortex", "Vortex API initialization failed");
        
        // Log failed components
        if (report_.FailedCount() > 0) {
            char failed_list[256] = {};
            size_t pos = 0;
            for (size_t i = 0; i < report_.FailedCount(); ++i) {
                if (pos > 0 && pos + 2 < sizeof(failed_list)) {
                    failed_list[pos++] = ',';
                    failed_list[pos++] = ' ';
                }
                const char* name = report_.Failed(i);
                size_t len = std::strlen(name);
                if (pos + len < sizeof(failed_list) - 1) {
                    std::memcpy(failed_list + pos, name, len);
                    pos += len;
                }
            }
            failed_list[pos] = '\0';
            Logger::GetInstance().Error("Vortex", "Failed components: %s", failed_list);
        }
    }

    return success;
}

bool Vortex::GetSystemDiagnostics(VortexSystemDiagnostics& diagnostics) const noexcept {
    MutexLockGuard lock(lifecycle_mutex_);
    if (!initialized_) {
        return false;
    }
    
    // Update diagnostics with current state
    const_cast<Vortex*>(this)->UpdateSystemDiagnostics();
    
    // Copy current diagnostics
    diagnostics = diagnostics_;
    return true;
}

std::array<bool, kVortexManagerCount> Vortex::GetComponentInitializationStatus() const noexcept {
    return {{
        nvs_initialized_.load(),
        comms_initialized_.load(),
        gpio_initialized_.load(),
        motors_initialized_.load(),
        adc_initialized_.load(),
        imu_initialized_.load(),
        encoders_initialized_.load(),
        leds_initialized_.load(),
        temp_initialized_.load()
    }};
}

size_t Vortex::GetFailedComponents(const char* out_names[], size_t max_entries) const noexcept {
    MutexLockGuard lock(lifecycle_mutex_);
    return report_.CopyFailed(out_names, max_entries);
}

size_t Vortex::GetSystemWarnings(const char* out_warnings[], size_t max_entries) const noexcept {
    MutexLockGuard lock(lifecycle_mutex_);
    return report_.CopyWarnings(out_warnings, max_entries);
}

//==============================================================================
// PUBLIC UTILITY METHODS
//==============================================================================

bool Vortex::CollectManagerHealth(ManagerHealthSnapshot& snapshot) noexcept {
    MutexLockGuard lock(lifecycle_mutex_);
    if (!initialized_) return false;

    snapshot = ManagerHealthSnapshot{};
    snapshot.count = kVortexManagerCount;
    snapshot.all_healthy = true;
    snapshot.managers_with_errors = 0;

    auto addEntry = [&](size_t idx, const char* name, bool init, uint8_t err) {
        snapshot.entries[idx].name = name;
        snapshot.entries[idx].initialized = init;
        snapshot.entries[idx].last_error_code = err;
        snapshot.entries[idx].has_error = (err != 0);
        if (err != 0) {
            snapshot.all_healthy = false;
            ++snapshot.managers_with_errors;
        }
        if (!init) {
            snapshot.all_healthy = false;
        }
    };

    addEntry(0, "NVS", nvs_initialized_.load(),
             static_cast<uint8_t>(nvs.GetLastError()));

    // 2. CommChannelsManager — direct GetLastError()
    addEntry(1, "CommChannels", comms_initialized_.load(),
             static_cast<uint8_t>(comms.GetLastError()));

    // 3. GpioManager — extract from diagnostics
    {
        GpioSystemDiagnostics gdiag{};
        gpio.GetSystemDiagnostics(gdiag);
        addEntry(2, "GPIO", gpio_initialized_.load(),
                 static_cast<uint8_t>(gdiag.last_error));
    }

    // 4. MotorController — direct GetLastError()
    addEntry(3, "Motors", motors_initialized_.load(),
             static_cast<uint8_t>(motors.GetLastError()));

    // 5. AdcManager — extract from diagnostics
    {
        AdcSystemDiagnostics adiag{};
        adc.GetSystemDiagnostics(adiag);
        addEntry(4, "ADC", adc_initialized_.load(),
                 static_cast<uint8_t>(adiag.last_error));
    }

    // 6. ImuManager — direct GetLastError()
    addEntry(5, "IMU", imu_initialized_.load(),
             static_cast<uint8_t>(imu.GetLastError()));

    // 7. EncoderManager — direct GetLastError()
    addEntry(6, "Encoders", encoders_initialized_.load(),
             static_cast<uint8_t>(encoders.GetLastError()));

    // 8. LedManager — typed GetLastError()
    addEntry(7, "LEDs", leds_initialized_.load(),
             static_cast<uint8_t>(leds.GetLastError()));

    // 9. TemperatureManager — extract from diagnostics
    {
        TempSystemDiagnostics tdiag{};
        temp.GetSystemDiagnostics(tdiag);
        addEntry(8, "Temperature", temp_initialized_.load(),
                 static_cast<uint8_t>(tdiag.last_error));
    }

    return true;
}

uint64_t Vortex::GetSystemUptimeMs() const noexcept {
    return RtosTime::GetCurrentTimeUs() / 1000;
}

uint64_t Vortex::GetInitializationTimeMs() const noexcept {
    if (initialization_start_time_ == 0 || initialization_end_time_ == 0) {
        return 0;
    }
    return (initialization_end_time_ - initialization_start_time_);
}

void Vortex::DumpSystemStatistics() const noexcept {
    if (!initialized_) {
        Logger::GetInstance().Warn("Vortex", "Cannot dump statistics - Vortex API not initialized");
        return;
    }
    
    Logger::GetInstance().Info("Vortex", "=== Vortex API System Statistics ===");
    Logger::GetInstance().Info("Vortex", "Version: %s  Board: %s  MCU: %s",
        VORTEX_HAL_VERSION_STRING, VORTEX_HAL_BOARD_NAME, VORTEX_HAL_MCU);
    Logger::GetInstance().Info("Vortex", "System Uptime: %llu ms", GetSystemUptimeMs());
    Logger::GetInstance().Info("Vortex", "Initialization Time: %llu ms", GetInitializationTimeMs());

    nvs.DumpStatistics();
    comms.DumpStatistics();
    gpio.DumpStatistics();
    motors.DumpStatistics();
    adc.DumpStatistics();
    imu.DumpStatistics();
    encoders.DumpStatistics();
    leds.DumpStatistics();
    temp.DumpStatistics();
    
    Logger::GetInstance().Info("Vortex", "=== End Vortex API System Statistics ===");
}

bool Vortex::PerformHealthCheck() noexcept {
    if (!initialized_) {
        Logger::GetInstance().Warn("Vortex", "Cannot perform health check - Vortex API not initialized");
        return false;
    }
    
    Logger::GetInstance().Info("Vortex", "Performing Vortex API health check");
    
    // Collect per-manager error health
    ManagerHealthSnapshot health{};
    CollectManagerHealth(health);
    
    // Log per-manager status with error codes
    for (size_t i = 0; i < health.count; ++i) {
        const auto& e = health.entries[i];
        if (!e.initialized) {
            Logger::GetInstance().Error("Vortex", "  %s: NOT INITIALIZED", e.name);
        } else if (e.has_error) {
            Logger::GetInstance().Warn("Vortex", "  %s: ERROR (code %u)", e.name,
                                      static_cast<unsigned>(e.last_error_code));
        } else {
            Logger::GetInstance().Info("Vortex", "  %s: OK", e.name);
        }
    }
    
    Logger::GetInstance().Info("Vortex", "Health Check: %s (%zu/%zu managers healthy)",
        health.all_healthy ? "HEALTHY" : "DEGRADED",
        health.count - health.managers_with_errors, health.count);
    
    return health.all_healthy;
}

const char* Vortex::GetSystemVersion() const noexcept {
    return "Vortex API v" VORTEX_HAL_VERSION_STRING " - " VORTEX_HAL_BOARD_NAME;
}

bool Vortex::Shutdown() noexcept {
    MutexLockGuard lock(lifecycle_mutex_);
    if (!initialized_) {
        Logger::GetInstance().Warn("Vortex", "Shutdown called but not initialized");
        return false;
    }

    Logger::GetInstance().Info("Vortex", "Shutting down Vortex API (reverse init order)");

    // Reverse initialization order: Temp → … → Comms → NVS
    if (temp_initialized_) {
        temp.Deinitialize();
        temp_initialized_ = false;
    }
    if (leds_initialized_) {
        leds.Deinitialize();
        leds_initialized_ = false;
    }
    if (encoders_initialized_) {
        encoders.Deinitialize();
        encoders_initialized_ = false;
    }
    if (imu_initialized_) {
        imu.Deinitialize();
        imu_initialized_ = false;
    }
    if (adc_initialized_) {
        adc.Deinitialize();
        adc_initialized_ = false;
    }
    if (motors_initialized_) {
        motors.Deinitialize();
        motors_initialized_ = false;
    }
    if (gpio_initialized_) {
        gpio.Deinitialize();
        gpio_initialized_ = false;
    }
    if (comms_initialized_) {
        comms.Deinitialize();
        comms_initialized_ = false;
    }
    if (nvs_initialized_) {
        nvs.Deinitialize();
        nvs_initialized_ = false;
    }

    initialized_ = false;
    report_.Clear();

    Logger::GetInstance().Info("Vortex", "Vortex API shutdown complete");
    return true;
}

//==============================================================================
// PRIVATE INITIALIZATION METHODS
//==============================================================================

bool Vortex::InitializeAllComponents() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing all Vortex components in dependency order");
    report_.Clear();

    if (!InitializeNvs()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize NVS");
        return false;
    }

    // Step 2: Initialize communication channels (foundation)
    if (!InitializeComms()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize communication channels");
        return false;
    }
    
    // Step 3: Initialize GPIO management (depends on CommChannelsManager)
    if (!InitializeGpio()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize GPIO management");
        return false;
    }

    // Step 4: LED / status (early — allows WS2812 bench when TMC9660 or sensors are absent)
    if (!InitializeLeds()) {
        Logger::GetInstance().Error("Vortex", "Failed to initialize LED management");
        return false;
    }

    // Remaining subsystems: best-effort for bench bring-up (log warnings, do not fail API init)
    if (!InitializeMotors()) {
        AddSystemWarning("Motor controllers not ready; running without the TMC9660");
    }
    if (!InitializeAdc()) {
        AddSystemWarning("ADC not ready (TMC9660-hosted channels unavailable)");
    }
    if (!InitializeImu()) {
        AddSystemWarning("IMU not ready");
    }
    if (!InitializeEncoders()) {
        AddSystemWarning("Encoders not ready");
    }
    if (!InitializeTemperature()) {
        AddSystemWarning("Temperature sensors not ready");
    }

    const bool core_ok =
        nvs_initialized_ && comms_initialized_ && gpio_initialized_ && leds_initialized_;
    if (!core_ok) {
        Logger::GetInstance().Error("Vortex", "Core subsystems incomplete (NVS+Comms+GPIO+LEDs required)");
        return false;
    }

    if (!motors_initialized_ || !adc_initialized_ || !imu_initialized_ || !encoders_initialized_ ||
        !temp_initialized_) {
        Logger::GetInstance().Warn(
            "Vortex", "Initialized in degraded mode — Motors:%s ADC:%s IMU:%s Enc:%s Temp:%s",
            motors_initialized_ ? "OK" : "OFF",
            adc_initialized_ ? "OK" : "OFF",
            imu_initialized_ ? "OK" : "OFF",
            encoders_initialized_ ? "OK" : "OFF",
            temp_initialized_ ? "OK" : "OFF");
    } else {
        Logger::GetInstance().Info("Vortex", "All Vortex components initialized successfully");
    }
    return true;
}

bool Vortex::InitializeNvs() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing NVS");

    const bool success = nvs.EnsureInitialized();
    nvs_initialized_ = success;

    if (success) {
        Logger::GetInstance().Info("Vortex", "NVS initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "NVS initialization failed");
        report_.AddFailed("NvsManager");
    }

    return success;
}

bool Vortex::InitializeComms() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing communication channels");
    
    bool success = comms.EnsureInitialized();
    comms_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "Communication channels initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "Communication channels initialization failed");
        report_.AddFailed("CommChannelsManager");
    }

    return success;
}

bool Vortex::InitializeGpio() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing GPIO management");
    
    bool success = gpio.EnsureInitialized();
    gpio_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "GPIO management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "GPIO management initialization failed");
        report_.AddFailed("GpioManager");
    }

    return success;
}

bool Vortex::InitializeMotors() noexcept {
    const VortexOnboardTmc9660Transport transport = GetOnboardTmc9660Transport();
    // Must match `Tmc9660Handler::kDefaultBootConfig` UART device_address (odd 1…255).
    constexpr uint8_t kOnboardTmc9660TmclAddress = 1;
    Logger::GetInstance().Info(
        "Vortex",
        "Initializing motor controllers (onboard TMC9660 transport: %s)",
        transport == VortexOnboardTmc9660Transport::Uart ? "UART" : "SPI");

    // Register onboard TMC9660 once GpioManager has created expander pins — mirrors
    // hf_functional_pin_config_vortex_v1.hpp. SPI path muxes host SPI to the IC via PCAL;
    // UART path keeps the mux released (MotorController applies gate from handler comm mode).
    if (motors.handler(MotorController::ONBOARD_TMC9660_INDEX) == nullptr) {
        auto rst = gpio.Get(HfFunctionalGpioPin::PCAL_TMC_RST_CTRL);
        auto drv = gpio.Get(HfFunctionalGpioPin::PCAL_TMC_DRV_EN);
        auto fault = gpio.Get(HfFunctionalGpioPin::PCAL_TMC_FAULT_STATUS);
        auto wake = gpio.Get(HfFunctionalGpioPin::PCAL_TMC_WAKE_CTRL);
        if (rst && drv && fault && wake) {
            Tmc9660ControlPins pins{*rst, *drv, *fault, *wake};
            if (transport == VortexOnboardTmc9660Transport::Uart) {
                BaseUart* uart = comms.GetUartBus();
                if (uart != nullptr) {
                    const MotorError reg = motors.CreateOnboardDevice(*uart, kOnboardTmc9660TmclAddress, pins, nullptr);
                    if (reg != MotorError::SUCCESS) {
                        Logger::GetInstance().Warn("Vortex", "CreateOnboardDevice(UART) returned %s",
                            MotorErrorToString(reg));
                        AddSystemWarning("Onboard TMC9660 (UART) could not be registered");
                    }
                } else {
                    AddSystemWarning("TMC9660 UART bus not registered; onboard motor skipped");
                }
            } else {
                BaseSpi* spi = comms.GetSpiDevice(SpiDeviceId::TMC9660_MOTOR_CONTROLLER);
                if (spi != nullptr) {
                    const MotorError reg = motors.CreateOnboardDevice(*spi, kOnboardTmc9660TmclAddress, pins, nullptr);
                    if (reg != MotorError::SUCCESS) {
                        Logger::GetInstance().Warn("Vortex", "CreateOnboardDevice(SPI) returned %s",
                            MotorErrorToString(reg));
                        AddSystemWarning("Onboard TMC9660 (SPI) could not be registered");
                    } else {
                        // Steer shared SPI to the TMC9660 before bridge GPIO registration or handler init.
                        // Otherwise the PCAL mux line can sit at its default until MotorController::Initialize(),
                        // so the first TMCL exchange sees open MISO (zeros) while the gate is still released.
                        (void)motors.ApplyOnboardTmc9660HostSpiGateMode(
                            Tmc9660HostSpiGateMode::EnabledForHostSpiMotor);
                    }
                } else {
                    AddSystemWarning("TMC9660 SPI device not registered; onboard motor skipped");
                }
            }
        } else {
            AddSystemWarning("PCAL95555 TMC9660 control GPIOs incomplete; onboard motor skipped");
        }
    }

    // GPIO table is built before motors; TMC9660Handler GPIO17/18 aliases register here.
    gpio.RegisterTmc9660BridgePinsIfNeeded();

    bool success = motors.EnsureInitialized();
    motors_initialized_ = success;

    if (success) {
        if (motors.GetDeviceCount() == 0) {
            // MotorController initialises with no device registered; say so instead of reporting OK.
            AddSystemWarning(transport == VortexOnboardTmc9660Transport::Uart
                                 ? "No motor device registered (UART transport): check UART wiring and "
                                   "PCAL RST/DRV_EN/FAULT/WAKE"
                                 : "No motor device registered (SPI transport): check PCAL95555 at 0x20 and "
                                   "the RST/DRV_EN/FAULT/WAKE GPIOs");
        }
    }

    if (success) {
        Logger::GetInstance().Info("Vortex", "Motor controllers initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "Motor controllers initialization failed");
        report_.AddFailed("MotorController");
    }

    return success;
}

bool Vortex::InitializeAdc() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing ADC management");
    
    bool success = adc.EnsureInitialized();
    adc_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "ADC management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "ADC management initialization failed");
        report_.AddFailed("AdcManager");
    }

    return success;
}

bool Vortex::InitializeImu() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing IMU management");
    
    bool success = imu.EnsureInitialized();
    imu_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "IMU management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "IMU management initialization failed");
        report_.AddFailed("ImuManager");
    }

    return success;
}

bool Vortex::InitializeEncoders() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing encoder management");
    
    bool success = encoders.EnsureInitialized();
    encoders_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "Encoder management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "Encoder management initialization failed");
        report_.AddFailed("EncoderManager");
    }

    return success;
}

bool Vortex::InitializeLeds() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing LED management");
    
    bool success = leds.EnsureInitialized();
    leds_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "LED management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "LED management initialization failed");
        report_.AddFailed("LedManager");
    }

    return success;
}

bool Vortex::InitializeTemperature() noexcept {
    Logger::GetInstance().Info("Vortex", "Initializing temperature management");
    
    bool success = temp.EnsureInitialized();
    temp_initialized_ = success;
    
    if (success) {
        Logger::GetInstance().Info("Vortex", "Temperature management initialized successfully");
    } else {
        Logger::GetInstance().Error("Vortex", "Temperature management initialization failed");
        report_.AddFailed("TemperatureManager");
    }

    return success;
}

//==============================================================================
// PRIVATE UTILITY METHODS
//==============================================================================

void Vortex::UpdateSystemDiagnostics() noexcept {
    // Update component status
    diagnostics_.nvs_initialized = nvs_initialized_;
    diagnostics_.comms_initialized = comms_initialized_;
    diagnostics_.gpio_initialized = gpio_initialized_;
    diagnostics_.motors_initialized = motors_initialized_;
    diagnostics_.adc_initialized = adc_initialized_;
    diagnostics_.imu_initialized = imu_initialized_;
    diagnostics_.encoders_initialized = encoders_initialized_;
    diagnostics_.leds_initialized = leds_initialized_;
    diagnostics_.temp_initialized = temp_initialized_;
    
    // Count initialized components
    diagnostics_.initialized_components = 0;
    if (nvs_initialized_) diagnostics_.initialized_components++;
    if (comms_initialized_) diagnostics_.initialized_components++;
    if (gpio_initialized_) diagnostics_.initialized_components++;
    if (motors_initialized_) diagnostics_.initialized_components++;
    if (adc_initialized_) diagnostics_.initialized_components++;
    if (imu_initialized_) diagnostics_.initialized_components++;
    if (encoders_initialized_) diagnostics_.initialized_components++;
    if (leds_initialized_) diagnostics_.initialized_components++;
    if (temp_initialized_) diagnostics_.initialized_components++;
    
    // Calculate failed components
    diagnostics_.failed_components = diagnostics_.total_components - diagnostics_.initialized_components;
    
    // Update timing information
    diagnostics_.initialization_time_ms = GetInitializationTimeMs();
    diagnostics_.system_uptime_ms = GetSystemUptimeMs();
    
    // Update system health
    diagnostics_.system_healthy = (diagnostics_.failed_components == 0) && initialized_;
    
    // Update failed components list
    diagnostics_.failed_components_list_count =
        report_.CopyFailed(diagnostics_.failed_components_list, VortexSystemDiagnostics::kMaxDiagEntries);

    // Update warnings
    diagnostics_.warnings_count =
        report_.CopyWarnings(diagnostics_.warnings, VortexSystemDiagnostics::kMaxDiagEntries);
}

void Vortex::AddSystemWarning(const char* warning) noexcept {
    if (!report_.AddWarning(warning)) {
        Logger::GetInstance().Warn("Vortex", "Warning list full; dropped: %s", warning);
        return;
    }
    Logger::GetInstance().Warn("Vortex", "Vortex API Warning: %s", warning);
} 