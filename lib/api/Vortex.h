/**
 * @file Vortex.h
 * @brief Unified API singleton for the HardFOC Vortex platform.
 * 
 * @details This class provides a beautiful, unified access point to all component handlers
 *          in the HardFOC Vortex system. It implements lazy initialization with proper
 *          dependency management and ensures all singletons are properly initialized
 *          in the correct order before exposing them to the user.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 * 
 * Key Features:
 * - True singleton pattern with lazy initialization
 * - Proper dependency management and initialization order
 * - Unified access to all component handlers
 * - Thread-safe initialization with comprehensive error handling
 * - Beautiful, intuitive API design
 * - Exception-free operation with proper error reporting
 * - Automatic resource management and cleanup
 * - Comprehensive system diagnostics and health monitoring
 * 
 * Architecture:
 * - Uses Meyer's singleton for thread-safe initialization
 * - Implements proper initialization order based on dependencies
 * - Provides reference-based access to all component handlers
 * - Follows RAII principles for resource management
 * - Uses unified Logger system for all logging
 * - Exception-free design with noexcept methods
 * 
 * Initialization Order:
 * 1. NvsManager (flash key-value — before buses so config/cal is available early)
 * 2. CommChannelsManager (foundation - SPI, I2C, UART, CAN)
 * 3. GpioManager (depends on CommChannelsManager)
 * 4. LedManager (WS2812 / status — early so LED-only benches work without TMC9660)
 * 5. MotorController (depends on CommChannelsManager; onboard **UART** vs SPI via
 *    `SetOnboardTmc9660Transport()` before `EnsureInitialized()` — default is UART)
 * 6. AdcManager (depends on MotorController)
 * 7. ImuManager (depends on CommChannelsManager, GpioManager)
 * 8. EncoderManager (depends on CommChannelsManager, GpioManager)
 * 9. TemperatureManager (depends on AdcManager, MotorController)
 *
 * @note Steps 5–9 are best-effort: `EnsureInitialized()` succeeds when NVS, comms, GPIO,
 *       and LEDs are up; motor / sensor failures produce warnings and degraded diagnostics.
 *
 * Usage Example:
 * @code
 * // Get the Vortex API instance
 * auto& vortex = Vortex::GetInstance();
 * 
 * // Ensure all systems are initialized
 * if (vortex.EnsureInitialized()) {
 *     // Access communication channels
 *     auto& comms = vortex.comms;
 *     
 *     // Access GPIO management
 *     auto& gpio = vortex.gpio;
 *     
 *     // Access motor controllers
 *     auto& motors = vortex.motors;
 *     
 *     // Access IMU sensors
 *     auto& imu = vortex.imu;
 *     
 *     // Access encoders
 *     auto& encoders = vortex.encoders;
 *     
 *     // Access LED management
 *     auto& leds = vortex.leds;
 *     
 *     // Access temperature sensors
 *     auto& temp = vortex.temp;
 *     
 *     // Access ADC channels
 *     auto& adc = vortex.adc;
 *     
 *     // Use any of the systems
 *     if (motors.EnsureInitialized()) {
 *         auto* handler = motors.handler(0);
 *         if (handler) {
 *             // Use motor controller
 *         }
 *     }
 * }
 * @endcode
 * 
 * @note This class is thread-safe and designed for concurrent access from multiple tasks.
 * @note All component handlers are accessed by reference for maximum performance.
 * @note The initialization order is carefully managed to handle dependencies correctly.
 */

#ifndef VORTEX_API_H_
#define VORTEX_API_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>

// Forward declarations for all component handlers
class CommChannelsManager;
class NvsManager;
class GpioManager;
class AdcManager;
class MotorController;
class ImuManager;
class EncoderManager;
class LedManager;
class TemperatureManager;

// Include the unified Logger
#include "handlers/logger/Logger.h"
#include "VortexVersion.h"

/** @brief Number of subsystem managers on `Vortex` (including NVS). */
inline constexpr size_t kVortexManagerCount = 9;

/**
 * @brief Host physical layer used for the onboard TMC9660 during `Vortex::InitializeMotors()`.
 *
 * @details Vortex always exposes a UART to the TMC9660. SPI to the motor IC is muxed
 *          (`PCAL_TMC_SPI_COMM_EN`); `MotorController` asserts it only for SPI handlers.
 *          Default is **UART** (bootloader/TMCL over UART, SPI mux released). Call
 *          `SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Spi)` before
 *          `EnsureInitialized()` if you need TMCL over SPI instead.
 */
enum class VortexOnboardTmc9660Transport : std::uint8_t {
    Spi = 0,
    Uart = 1,
};

//==============================================================================
// VORTEX SYSTEM DIAGNOSTICS
//==============================================================================

/**
 * @brief Comprehensive system diagnostics for the Vortex platform
 */
struct VortexSystemDiagnostics {
    bool system_healthy;                    ///< Overall system health
    bool nvs_initialized;                   ///< NvsManager ready
    bool comms_initialized;                 ///< Communication channels status
    bool gpio_initialized;                  ///< GPIO management status
    bool adc_initialized;                   ///< ADC management status
    bool motors_initialized;                ///< Motor controller status
    bool imu_initialized;                   ///< IMU management status
    bool encoders_initialized;              ///< Encoder management status
    bool leds_initialized;                  ///< LED management status
    bool temp_initialized;                  ///< Temperature management status
    
    uint32_t total_components;              ///< Total number of components
    uint32_t initialized_components;        ///< Number of initialized components
    uint32_t failed_components;             ///< Number of failed components
    
    uint64_t initialization_time_ms;        ///< Total initialization time
    uint64_t system_uptime_ms;              ///< System uptime
    
    static constexpr size_t kMaxDiagEntries = kVortexManagerCount;  ///< Max diagnostic entries

    const char* failed_components_list[kMaxDiagEntries]{};  ///< Failed component names
    size_t failed_components_list_count{0};                  ///< Number of failed entries
    const char* warnings[kMaxDiagEntries]{};                 ///< System warnings
    size_t warnings_count{0};                                ///< Number of warnings
    
    /**
     * @brief Constructor for VortexSystemDiagnostics
     */
    VortexSystemDiagnostics() noexcept
        : system_healthy(false), nvs_initialized(false), comms_initialized(false), gpio_initialized(false),
          adc_initialized(false), motors_initialized(false), imu_initialized(false),
          encoders_initialized(false), leds_initialized(false), temp_initialized(false),
          total_components(static_cast<uint32_t>(kVortexManagerCount)), initialized_components(0), failed_components(0),
          initialization_time_ms(0), system_uptime_ms(0) {}
};

//==============================================================================
// MANAGER HEALTH SNAPSHOT
//==============================================================================

/**
 * @brief Per-manager health entry for unified error aggregation
 */
struct ManagerHealthEntry {
    const char* name;           ///< Manager name (string literal)
    bool initialized;           ///< Whether manager is initialized
    bool has_error;             ///< Whether last error is non-zero
    uint8_t last_error_code;    ///< Last error code (0 = success, enum-specific otherwise)

    ManagerHealthEntry() noexcept
        : name(nullptr), initialized(false), has_error(false), last_error_code(0) {}
};

/**
 * @brief Aggregated health snapshot across all Vortex managers
 *
 * @details Provides a unified view of errors across all component managers.
 *          Error codes are cast to uint8_t; interpretation requires knowledge
 *          of the originating manager's error enum.
 *
 * Supported error sources per manager:
 *   NVS           — hf_nvs_err_t   via GetLastError()
 *   CommChannels  — CommError      via GetLastError()
 *   GPIO          — hf_gpio_err_t  via GetLastError()
 *   Motors        — MotorError     via GetLastError()
 *   ADC           — hf_adc_err_t   via GetLastError()
 *   IMU           — ImuError       via GetLastError()
 *   Encoders      — EncoderError   via GetLastError()
 *   LEDs          — LedError       via GetLastError()
 *   Temperature   — hf_temp_err_t  via GetLastError()
 */
struct ManagerHealthSnapshot {
    static constexpr size_t kMaxManagers = kVortexManagerCount;

    ManagerHealthEntry entries[kMaxManagers];    ///< Per-manager entries
    size_t count;                                ///< Number of populated entries
    size_t managers_with_errors;                 ///< Count of managers reporting errors
    bool all_healthy;                            ///< True if no errors and all initialized

    ManagerHealthSnapshot() noexcept
        : entries{}, count(0), managers_with_errors(0), all_healthy(true) {}
};

//==============================================================================
// VORTEX API CLASS
//==============================================================================

/**
 * @class Vortex
 * @brief Unified API singleton for the HardFOC Vortex platform
 */
class Vortex {
public:
    //**************************************************************************//
    //**                  SINGLETON ACCESS                                    **//
    //**************************************************************************//

    /**
     * @brief Get the singleton instance of Vortex API
     * @return Reference to the singleton Vortex instance
     * @note Uses Meyer's singleton for thread-safe initialization
     */
    static Vortex& GetInstance() noexcept;

    /**
     * @brief Select onboard TMC9660 transport before the first `EnsureInitialized()`.
     * @note If `EnsureInitialized()` has already completed, the call is ignored and a warning is logged.
     */
    static void SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport transport) noexcept;

    /** @brief Effective transport for `InitializeMotors()` (default: Uart). */
    [[nodiscard]] static VortexOnboardTmc9660Transport GetOnboardTmc9660Transport() noexcept;

    // Non-copyable, non-movable
    Vortex(const Vortex&) = delete;
    Vortex& operator=(const Vortex&) = delete;
    Vortex(Vortex&&) = delete;
    Vortex& operator=(Vortex&&) = delete;

    //**************************************************************************//
    //**                  INITIALIZATION METHODS                              **//
    //**************************************************************************//

    /**
     * @brief Ensure all Vortex systems are properly initialized
     * @return true if all systems initialized successfully, false otherwise
     * @note This method handles the complete initialization sequence in proper order
     */
    bool EnsureInitialized() noexcept;

    /**
     * @brief Check if all Vortex systems are initialized
     * @return true if all systems are initialized, false otherwise
     */
    [[nodiscard]] bool IsInitialized() const noexcept { return initialized_; }

    /**
     * @brief Shut down all subsystems in reverse initialization order.
     * @return true if all subsystems shut down cleanly
     */
    bool Shutdown() noexcept;

    /**
     * @brief Get comprehensive system diagnostics
     * @param diagnostics Reference to diagnostics structure to fill
     * @return true if diagnostics retrieved successfully, false otherwise
     */
    [[nodiscard]] bool GetSystemDiagnostics(VortexSystemDiagnostics& diagnostics) const noexcept;

    /**
     * @brief Get initialization status for each component
     * @return Array of initialization status (true/false) for each component
     */
    [[nodiscard]] std::array<bool, kVortexManagerCount> GetComponentInitializationStatus() const noexcept;

    /**
     * @brief Get list of failed components during initialization
     * @param out_names Array to receive failed component name pointers
     * @param max_entries Maximum entries the array can hold
     * @return Number of failed components written
     */
    [[nodiscard]] size_t GetFailedComponents(const char* out_names[], size_t max_entries) const noexcept;

    /**
     * @brief Get system warnings and issues
     * @param out_warnings Array to receive warning string pointers
     * @param max_entries Maximum entries the array can hold
     * @return Number of warnings written
     */
    [[nodiscard]] size_t GetSystemWarnings(const char* out_warnings[], size_t max_entries) const noexcept;

    /**
     * @brief Collect per-manager error health into a unified snapshot
     * @param snapshot Reference to snapshot structure to fill
     * @return true if snapshot collected successfully, false if not initialized
     * @note Error codes are manager-specific uint8_t values (0 = success)
     */
    [[nodiscard]] bool CollectManagerHealth(ManagerHealthSnapshot& snapshot) noexcept;

    //**************************************************************************//
    //**                  COMPONENT ACCESS                                    **//
    //**************************************************************************//

    /** @brief Communication channels (SPI, I²C, UART, CAN). After NVS. */
    CommChannelsManager& comms;

    /** @brief Non-volatile storage (`vortex` / `vortex_cal` / `vortex_cfg`). Init before comms. */
    NvsManager& nvs;

    /** @brief ESP32 + I/O-expander GPIO pins. Depends on comms. */
    GpioManager& gpio;

    /** @brief TMC9660 motor controllers. Depends on comms. */
    MotorController& motors;

    /** @brief ADC channels (Vortex: TMC9660-hosted path; see AdcManager). Depends on motors. */
    AdcManager& adc;

    /** @brief BNO08x IMU sensors. Depends on comms and gpio. */
    ImuManager& imu;

    /** @brief AS5047U rotary encoders. Depends on comms and gpio. */
    EncoderManager& encoders;

    /** @brief WS2812 status LED. Independent component. */
    LedManager& leds;

    /** @brief Temperature sensors (NTC, ESP32, TMC9660). Depends on ADC and motors. */
    TemperatureManager& temp;

    //**************************************************************************//
    //**                  UTILITY METHODS                                     **//
    //**************************************************************************//

    /**
     * @brief Get system uptime in milliseconds
     * @return System uptime in milliseconds
     */
    [[nodiscard]] uint64_t GetSystemUptimeMs() const noexcept;

    /**
     * @brief Get initialization time in milliseconds
     * @return Time taken for initialization in milliseconds
     */
    [[nodiscard]] uint64_t GetInitializationTimeMs() const noexcept;

    /**
     * @brief Dump comprehensive system statistics
     * @note This will dump statistics from all component handlers
     */
    void DumpSystemStatistics() const noexcept;

    /**
     * @brief Perform system health check
     * @return true if system is healthy, false otherwise
     */
    [[nodiscard]] bool PerformHealthCheck() noexcept;

    /**
     * @brief Get system version information
     * @return System version string
     */
    [[nodiscard]] const char* GetSystemVersion() const noexcept;

private:
    //**************************************************************************//
    //**                  PRIVATE CONSTRUCTOR                                 **//
    //**************************************************************************//

    /**
     * @brief Private constructor for singleton pattern
     */
    Vortex() noexcept;

    /**
     * @brief Destructor for cleanup
     */
    ~Vortex() = default;

    //**************************************************************************//
    //**                  PRIVATE INITIALIZATION METHODS                      **//
    //**************************************************************************//

    /**
     * @brief Initialize all component handlers in proper order
     * @return true if all components initialized successfully, false otherwise
     */
    bool InitializeAllComponents() noexcept;

    bool InitializeNvs() noexcept;

    /**
     * @brief Initialize communication channels (step 2)
     * @return true if successful, false otherwise
     */
    bool InitializeComms() noexcept;

    /**
     * @brief Initialize GPIO management (step 3)
     * @return true if successful, false otherwise
     */
    bool InitializeGpio() noexcept;

    /**
     * @brief Initialize motor controllers (step 5 — after GPIO + LEDs)
     * @return true if successful, false otherwise
     */
    bool InitializeMotors() noexcept;

    /**
     * @brief Initialize ADC management (step 6)
     * @return true if successful, false otherwise
     */
    bool InitializeAdc() noexcept;

    /**
     * @brief Initialize IMU management (step 7)
     * @return true if successful, false otherwise
     */
    bool InitializeImu() noexcept;

    /**
     * @brief Initialize encoder management (step 8)
     * @return true if successful, false otherwise
     */
    bool InitializeEncoders() noexcept;

    /**
     * @brief Initialize LED management (step 4 — early for LED-only benches)
     * @return true if successful, false otherwise
     */
    bool InitializeLeds() noexcept;

    /**
     * @brief Initialize temperature management (step 9)
     * @return true if successful, false otherwise
     */
    bool InitializeTemperature() noexcept;

    /**
     * @brief Update system diagnostics
     */
    void UpdateSystemDiagnostics() noexcept;

    /**
     * @brief Add warning message to system warnings
     * @param warning Warning message to add
     */
    void AddSystemWarning(const char* warning) noexcept;

    //**************************************************************************//
    //**                  PRIVATE MEMBER VARIABLES                            **//
    //**************************************************************************//

    std::atomic<bool> initialized_;                    ///< Overall initialization status
    std::atomic<uint64_t> initialization_start_time_;  ///< Initialization start timestamp
    std::atomic<uint64_t> initialization_end_time_;    ///< Initialization end timestamp
    
    // Component initialization status
    std::atomic<bool> nvs_initialized_;
    std::atomic<bool> comms_initialized_;
    std::atomic<bool> gpio_initialized_;
    std::atomic<bool> motors_initialized_;
    std::atomic<bool> adc_initialized_;
    std::atomic<bool> imu_initialized_;
    std::atomic<bool> encoders_initialized_;
    std::atomic<bool> leds_initialized_;
    std::atomic<bool> temp_initialized_;
    
    // System diagnostics (fixed-size, no heap allocation)
    mutable VortexSystemDiagnostics diagnostics_;

    static constexpr size_t kMaxWarnings = 8;
    static constexpr size_t kMaxFailedComponents = 8;
    mutable const char* system_warnings_[kMaxWarnings]{};
    mutable size_t system_warnings_count_{0};
    mutable const char* failed_components_[kMaxFailedComponents]{};
    mutable size_t failed_components_count_{0};

    /** 0 = Spi, 1 = Uart — must match `VortexOnboardTmc9660Transport` enumerators. */
    static std::atomic<std::uint8_t> onboard_tmc9660_transport_;

    // Component references (these are references to singletons, not owned)
    // NOTE: Removed duplicate _ref_ members - public refs serve both roles
};

//==============================================================================
// CONVENIENCE
//==============================================================================

/**
 * @brief Convenience accessor — equivalent to Vortex::GetInstance().
 * @return Reference to the singleton Vortex API.
 */
[[nodiscard]] inline Vortex& GetVortex() noexcept {
    return Vortex::GetInstance();
}

//==============================================================================
// GLOBAL ACCESS MACROS
//==============================================================================

/**
 * @brief Global access macro for Vortex API
 * @note This provides a convenient way to access the Vortex API from anywhere
 */
#define VORTEX_API Vortex::GetInstance()

/**
 * @brief Global access macro for Vortex API with initialization check
 * @note This ensures initialization before access
 */
#define VORTEX_API_INIT() (Vortex::GetInstance().EnsureInitialized(), Vortex::GetInstance())

#endif // VORTEX_API_H_ 