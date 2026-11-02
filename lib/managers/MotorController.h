#ifndef COMPONENT_HANDLER_MOTOR_CONTROLLER_H_
#define COMPONENT_HANDLER_MOTOR_CONTROLLER_H_

#include "handlers/tmc9660/Tmc9660Handler.h"
#include "CommChannelsManager.h"
#include "RtosMutex.h"
#include "api_helpers/Tmc9660DriveObservability.h"
#include <span>
#include <array>
#include <atomic>
#include <optional>

/**
 * @file MotorController.h
 * @brief Singleton class for managing multiple TMC9660 motor controllers and their interfaces.
 *
 * This class provides a global singleton for managing TMC9660 motor controllers, including:
 * - Always creates and manages the onboard TMC9660 device (device index 0)
 * - Allows dynamic creation/deletion of external TMC9660 devices (indices 2-3)
 * - Array-based access to Tmc9660Handler instances by device index
 * - Access to individual TMC9660 drivers, GPIO, and ADC by device index
 * - Thread-safe device registration and access
 * - Board-aware device management with predefined CS pin assignments
 *
 * @note Since the TMC9660 driver update, handler construction requires host-side GPIO
 *       control pins (RST, DRV_EN, FAULTN, WAKE). These must be provided when creating
 *       TMC9660 devices via the Tmc9660ControlPins struct.
 */

/**
 * @brief Bundle of host-side GPIO references for TMC9660 control pins.
 *
 * These are the GPIO pins on the host MCU (e.g., ESP32) that connect to the
 * TMC9660's control signals. They are required by the TMC9660 bootloader
 * initialization sequence for hardware reset and fault monitoring.
 *
 * @note The BaseGpio instances must be pre-configured (pin number, direction)
 *       before being passed to the handler.
 */
struct Tmc9660ControlPins {
    BaseGpio& rst;      ///< RST pin (output) - hardware reset
    BaseGpio& drv_en;   ///< DRV_EN pin (output) - driver enable
    BaseGpio& faultn;   ///< FAULTN pin (input) - fault status
    BaseGpio& wake;     ///< WAKE pin (output) - wake from hibernation
};

//==============================================================================
// MOTOR MECHANICAL FRAME (gearbox / output shaft mapping)
//==============================================================================

/**
 * @brief Mechanical mapping between the motor (rotor) frame and the load
 *        (output-shaft) frame.
 *
 * Vortex talks to the silicon in the motor frame: every set/get on
 * `velocityControl`, `positionControl`, the Diagnostics snapshot, and the
 * commutation engine uses motor-shaft units. When the motor drives a
 * gearbox, the user typically wants to think in the output-shaft frame.
 *
 * `gear_ratio_motor_to_output` is the number of motor-shaft revolutions per
 * one output-shaft revolution — e.g. a 10:1 reducer has gear_ratio = 10.
 *
 *     motor_value = load_value * gear_ratio_motor_to_output
 *     load_value  = motor_value / gear_ratio_motor_to_output
 *
 * Defaults to `1.0` (direct drive) so motor-frame and load-frame APIs are
 * interchangeable until the user configures a gearbox.
 */
struct MotorMechanics {
    /**
     * Motor-shaft revolutions per one output-shaft revolution.
     * Must be > 0. 1.0 = direct drive. Reducers (typical) are > 1.0;
     * step-up gearing is < 1.0 (e.g. 0.5 for 1:2 step-up).
     */
    double gear_ratio_motor_to_output{1.0};

    /**
     * If true, sign of any load-frame command/feedback is inverted before
     * being mapped to/from the motor frame. Useful when the gearbox or
     * coupling reverses rotation direction.
     */
    bool invert_output{false};
};

//==============================================================================
// DRIVE OBSERVABILITY TYPES (see api_helpers/Tmc9660DriveObservability.h)
//==============================================================================

using MotorParameterAccess = vortex::motor::ParamAccess;          ///< One element of a batch read / write
using MotorParameterWrite = vortex::motor::ParamWrite;            ///< One entry of a parameter image
using MotorTelemetrySet = vortex::motor::TelemetrySet;            ///< Which signals a telemetry read fetches
using MotorTelemetryField = vortex::motor::TelemetryField;
using MotorTelemetrySnapshot = vortex::motor::MotorTelemetrySnapshot;
using MotorDriveDiagnostics = vortex::motor::MotorDriveDiagnostics;
using MotorApplyOptions = vortex::motor::ApplyOptions;
using MotorApplyResult = vortex::motor::ApplyResult;

/** @brief TMCL access accounting per device (every batch MotorController sends on the device's behalf). */
struct MotorAccessStats {
    uint32_t batches{0};            ///< Batches sent (one per telemetry / diagnostics / parameter call)
    uint32_t accesses{0};           ///< TMCL accesses in those batches
    uint32_t failed_accesses{0};    ///< Accesses not answered REPLY_OK
    uint32_t last_batch_us{0};      ///< Duration of the latest batch
    uint32_t max_batch_us{0};       ///< Longest batch since the last reset
};

//==============================================================================
// MOTOR CONTROLLER ERROR CODES
//==============================================================================

/**
 * @brief Motor controller manager error codes for consistent error reporting.
 *
 * Manager-layer errors for TMC9660 device lifecycle operations (create, delete,
 * initialize). Driver-level TMC9660 errors are handled internally by the handler.
 */
enum class MotorError : uint8_t {
    SUCCESS = 0,              ///< Operation completed successfully
    NOT_INITIALIZED,          ///< Manager not yet initialised
    INITIALIZATION_FAILED,    ///< Device initialisation failed
    DEVICE_ALREADY_EXISTS,    ///< A device already occupies this slot
    DEVICE_NOT_FOUND,         ///< No active device at the given index
    INVALID_DEVICE_INDEX,     ///< Device index out of valid range
    CANNOT_DELETE_ONBOARD,    ///< Onboard device (index 0) cannot be deleted
    DEPENDENCY_NOT_READY,     ///< Required dependency not initialised
    COMMUNICATION_FAILED,     ///< A bus-level transfer failed
    INVALID_PARAMETER,        ///< Null pointer or out-of-range argument
    HANDLER_CREATION_FAILED,  ///< Heap allocation or handler init failed
    MUTEX_LOCK_FAILED         ///< RTOS mutex acquire timed out
};

/**
 * @brief Vortex-style host SPI steering to the onboard TMC9660 (PCAL95555 outputs).
 *
 * @details On Vortex, `PCAL_TMC_SPI_COMM_EN` (TMC_COMM_ENn, active-low) steers the shared SPI
 *          segment: asserted (`SetActive`) connects the ESP32 SPI master to the TMC9660;
 *          deasserted releases that path (e.g. host UART to TMC, or TMC-local SPI) per schematic.
 */
enum class Tmc9660HostSpiGateMode : uint8_t {
    Disabled = 0,                 ///< SPI mux released (UART motor path / idle).
    EnabledForHostSpiMotor = 1    ///< Host SPI to TMC9660 (`PCAL_TMC_SPI_COMM_EN` asserted).
};

/**
 * @brief Convert MotorError to a human-readable string.
 * @param error Error code to convert.
 * @return Null-terminated string representation.
 */
constexpr const char* MotorErrorToString(MotorError error) noexcept {
    switch (error) {
        case MotorError::SUCCESS:                 return "Success";
        case MotorError::NOT_INITIALIZED:         return "Not initialized";
        case MotorError::INITIALIZATION_FAILED:   return "Initialization failed";
        case MotorError::DEVICE_ALREADY_EXISTS:   return "Device already exists";
        case MotorError::DEVICE_NOT_FOUND:        return "Device not found";
        case MotorError::INVALID_DEVICE_INDEX:    return "Invalid device index";
        case MotorError::CANNOT_DELETE_ONBOARD:   return "Cannot delete onboard device";
        case MotorError::DEPENDENCY_NOT_READY:    return "Dependency not ready";
        case MotorError::COMMUNICATION_FAILED:    return "Communication failed";
        case MotorError::INVALID_PARAMETER:       return "Invalid parameter";
        case MotorError::HANDLER_CREATION_FAILED: return "Handler creation failed";
        case MotorError::MUTEX_LOCK_FAILED:       return "Mutex lock failed";
        default:                                  return "Unknown error";
    }
}

//==============================================================================
// MOTOR CONTROLLER DIAGNOSTICS
//==============================================================================

/**
 * @brief System-level diagnostics snapshot for motor controller health monitoring.
 *
 * Aggregates per-device and system-wide status in a single copyable struct,
 * matching the pattern used by LedSystemDiagnostics / GpioSystemDiagnostics.
 */
struct MotorSystemDiagnostics {
    bool system_healthy;               ///< true when all active devices are initialized
    bool system_initialized;           ///< MotorController::IsInitialized()
    uint8_t active_device_count;       ///< Number of active device slots
    uint8_t initialized_device_count;  ///< Number of successfully initialized devices
    MotorError last_error;             ///< Most recent error code

    /// Per-device snapshot (indexed 0..MAX_TMC9660_DEVICES-1)
    struct DeviceSnapshot {
        bool active;
        bool initialized;
        uint32_t error_count;          ///< Cumulative errors for this slot
    };
    static constexpr uint8_t kMaxDevices = 4;
    DeviceSnapshot devices[kMaxDevices]{};
};

/**
 * @class MotorController
 * @brief Singleton managing multiple TMC9660 motor controllers on Vortex V1.
 *
 * @details Creates the onboard TMC9660 (index 0) and allows dynamic creation
 *          of external TMC9660 devices (indices 2–3). Provides array-based
 *          access to Tmc9660Handler instances, thread-safe device registration,
 *          and board-aware CS-pin management.
 *
 * @note Call CreateOnboardDevice() before EnsureInitialized(). Host-side GPIO
 *       control pins (RST, DRV_EN, FAULTN, WAKE) are required for the TMC9660
 *       bootloader sequence.
 */
class MotorController {
public:
    static constexpr uint8_t MAX_TMC9660_DEVICES = 4;   ///< Maximum supported TMC9660 devices (board + external)
    static constexpr uint8_t ONBOARD_TMC9660_INDEX = 0; ///< Onboard TMC9660 device index (SPI2_CS_TMC9660)
    static constexpr uint8_t EXTERNAL_DEVICE_1_INDEX = 2; ///< External device 1 index (EXT_GPIO_CS_1)  
    static constexpr uint8_t EXTERNAL_DEVICE_2_INDEX = 3; ///< External device 2 index (EXT_GPIO_CS_2)
public:
    /**
     * @brief Get the singleton instance.
     * @return Reference to the singleton MotorController.
     */
    static MotorController& GetInstance() noexcept;

    //**************************************************************************//
    //**                  DEVICE MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Initialise all registered motor controllers if not already done.
     * @return true on success or if already initialised.
     */
    bool EnsureInitialized() noexcept {
        if (!initialized_.load(std::memory_order_acquire)) {
            RtosUniqueLock<RtosMutex> lock(deviceMutex_);
            if (!initialized_.load(std::memory_order_relaxed)) {
                initialized_.store(Initialize(), std::memory_order_release);
            }
        }
        return initialized_.load(std::memory_order_acquire);
    }

    /**
     * @brief Check whether the motor controller subsystem has been initialised.
     * @return true if initialised.
     */
    inline bool IsInitialized() const noexcept { return initialized_.load(std::memory_order_acquire); }

    /**
     * @brief Shut down all motor controller devices and release resources.
     * @return true on success
     */
    bool Deinitialize() noexcept;

    /**
     * @brief Get the most recent error code.
     * @return Last MotorError set by any API call
     */
    [[nodiscard]] MotorError GetLastError() const noexcept { return last_error_.load(std::memory_order_acquire); }

    /**
     * @brief Fill a diagnostics snapshot with current system state.
     * @param diagnostics Output structure to populate
     * @return MotorError::SUCCESS on success, MotorError::NOT_INITIALIZED if not init
     */
    [[nodiscard]] MotorError GetSystemDiagnostics(MotorSystemDiagnostics& diagnostics) const noexcept;

    /**
     * @brief Drive the onboard TMC9660 host SPI mux (`PCAL_TMC_SPI_COMM_EN`) on Vortex.
     *
     * @param mode `Disabled` for UART motor comms or to free the exported SPI segment;
     *             `EnabledForHostSpiMotor` before TMCL/bootloader over SPI.
     * @return MotorError::SUCCESS after applying drive levels (warnings only if a line is missing).
     * @note Thread-safe. Call from application code when switching between SPI and UART transports
     *       without tearing down the handler (requires coordinated handler re-creation today).
     */
    [[nodiscard]] MotorError ApplyOnboardTmc9660HostSpiGateMode(Tmc9660HostSpiGateMode mode) noexcept;

    /**
     * @brief Physical comm mode for the onboard handler (slot 0), if present.
     * @return `CommMode::SPI` or `::UART` from the active `Tmc9660Handler`, or nullopt when no device.
     */
    [[nodiscard]] std::optional<tmc9660::CommMode> TryGetOnboardCommMode() const noexcept;

    //**************************************************************************//
    //**                  TMCL SPI TIMING (BENCH TUNING)                      **//
    //**************************************************************************//

    /**
     * @brief Set the SPI TMCL handshake timing for one TMC9660.
     *
     * @details The driver polls the TMC9660's NOT_READY status instead of waiting a fixed
     *          gap (see `tmc9660::TmclSpiTiming`). Every device starts with the driver
     *          defaults (`TmclSpiTiming{}`: no frame gap, 20 to 200 us back-off, 60 polls).
     *          The value is stored per slot and applied to the driver now if it is up, and
     *          again after every (re)initialization of that slot, so it can be set before
     *          `Vortex` brings the motors up. Measure with the TMC9660 driver's `tmcl_timing`
     *          example before changing it; `tmc9660::TmclSpiTiming::legacy()` restores the
     *          1000 us gap used up to driver 1.0.0.
     *
     *          The poll budget runs on the calling thread. Lower `maxPolls` for slots that
     *          are accessed from a high-priority control task.
     *
     * @param timing      New timing.
     * @param deviceIndex Device index (0=onboard, 2-3=external).
     * @return SUCCESS, INVALID_DEVICE_INDEX, or INVALID_PARAMETER when the active device
     *         on that slot talks UART (the setting only affects SPI).
     */
    [[nodiscard]] MotorError SetTmclSpiTiming(const tmc9660::TmclSpiTiming& timing,
                                              uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /**
     * @brief SPI TMCL timing stored for a slot (the driver defaults unless changed).
     * @return SUCCESS or INVALID_DEVICE_INDEX.
     */
    [[nodiscard]] MotorError GetTmclSpiTiming(tmc9660::TmclSpiTiming& timing,
                                              uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) const noexcept;

    /**
     * @brief SPI TMCL transport counters (frames, NOT_READY re-sends, stale replies, failures).
     * @return SUCCESS, INVALID_DEVICE_INDEX, DEVICE_NOT_FOUND (no device or driver not up),
     *         or INVALID_PARAMETER for a UART device.
     */
    [[nodiscard]] MotorError GetTmclSpiStats(tmc9660::TmclSpiStats& stats,
                                             uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /**
     * @brief Reset the SPI TMCL transport counters.
     * @return Same codes as GetTmclSpiStats().
     */
    [[nodiscard]] MotorError ResetTmclSpiStats(uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    //**************************************************************************//
    //**        DRIVE OBSERVABILITY: BATCH ACCESS, TELEMETRY, DIAGNOSTICS    **//
    //**************************************************************************//
    // Every call below is one (or, for an image, a few) pipelined TMCL batch, sent through the
    // handler's visitDriver() while this manager's device lock is held. A device that is not
    // active returns DEVICE_NOT_FOUND; one whose bring-up did not succeed returns NOT_INITIALIZED
    // (no call here starts a bootloader bring-up). COMMUNICATION_FAILED means at least one access
    // was not answered REPLY_OK; per-element results are in the items / snapshot / result.

    /**
     * @brief Read several axis parameters in pipelined batches (GAP).
     * @param items In: ids (and axis). Out: values and per-element TMCL status.
     */
    [[nodiscard]] MotorError ReadParameters(std::span<MotorParameterAccess> items,
                                            uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /** @brief Write several axis parameters in pipelined batches (SAP); per-element status in @p items. */
    [[nodiscard]] MotorError WriteParameters(std::span<MotorParameterAccess> items,
                                             uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /**
     * @brief Refresh a caller-owned telemetry snapshot from one batch.
     * @details MotorTelemetrySet::Cyclic is the drive-observability design's fixed slots (position,
     *          velocity, packed Iq/Id, status, and the error words alternating); Status adds power, thermal
     *          and I²t words; Full reads every field. Each field keeps the sequence number of its last good
     *          read, so a consumer can judge its age (MotorTelemetrySnapshot::Age()).
     */
    [[nodiscard]] MotorError ReadTelemetry(MotorTelemetrySet set, MotorTelemetrySnapshot& snapshot,
                                           uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /**
     * @brief Read the four status words (289/299/300/301) in one batch and decode them.
     * @details Includes the chip-reset indication (watchdog bit: re-apply the parameter image), heartbeat
     *          stop, gate-driver faults per phase and side, I²t and thermal conditions, and a severity.
     *          On SPI the TMCL link counters are copied into @p out as well.
     */
    [[nodiscard]] MotorError ReadDriveDiagnostics(MotorDriveDiagnostics& out,
                                                  uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /** @brief Clear latched (write-1-to-clear) bits of the general error, gate-driver and ADC words. */
    [[nodiscard]] MotorError ClearLatchedFaults(uint32_t general_errors, uint32_t gate_driver_errors,
                                                uint32_t adc_status,
                                                uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /**
     * @brief Apply a parameter image (MOTOR_TYPE first), optionally only where the chip differs and with
     *        read-back verification. The TMC9660's own NVM is not used (design §4.7).
     * @param[out] result Counts of written, skipped, rejected and mismatched entries, and the first bad one.
     * @return SUCCESS when result.ok(); COMMUNICATION_FAILED otherwise.
     */
    [[nodiscard]] MotorError ApplyParameterImage(std::span<const MotorParameterWrite> image,
                                                 const MotorApplyOptions& options, MotorApplyResult& result,
                                                 uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /** @brief TMCL access accounting of one device (batches, accesses, failures, batch latency). */
    [[nodiscard]] MotorError GetAccessStats(MotorAccessStats& stats,
                                            uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) const noexcept;

    /** @brief Zero the access accounting of one device. */
    void ResetAccessStats(uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    //**************************************************************************//
    //**                  HANDLER AND DRIVER MANAGEMENT                       **//
    //**************************************************************************//

    /**
     * @brief Get access to Tmc9660Handler by device index.
     * @param deviceIndex Device index (0=onboard, 2-3=external)
     * @return Pointer to Tmc9660Handler if valid and active, nullptr otherwise
     * @note Returns nullptr if deviceIndex is invalid, device not active, or not initialized
     */
    Tmc9660Handler* handler(uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /**
     * @brief Visit the underlying typed TMC9660 driver by device index.
     *
     * Since the TMC9660 driver is now template-based, direct pointer access is
     * replaced by a visitor pattern. The provided function receives a reference
     * to the typed driver (either TMC9660<HalSpiTmc9660Comm> or TMC9660<HalUartTmc9660Comm>).
     *
     * @tparam Func Callable type accepting auto& (the typed driver reference)
     * @param func The visitor function to execute on the driver
     * @param deviceIndex Device index (0=onboard, 2-3=external)
     * @return The return value of func, or default-constructed if driver not available
     *
     * @code
     * motorController.visitDriver([](auto& driver) {
     *     driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC, 7);
     * });
     * @endcode
     */
    template <typename Func>
    auto visitDriver(Func&& func, uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept {
        // Hold the device lock for the whole visit, so the slot cannot be deleted underneath it;
        // the handler takes its own lock inside (order: deviceMutex_ -> handler mutex).
        MutexLockGuard lock(deviceMutex_);
        Tmc9660Handler* h = (deviceIndex < MAX_TMC9660_DEVICES && deviceActive_[deviceIndex] &&
                             tmcHandlers_[deviceIndex])
                                ? &*tmcHandlers_[deviceIndex]
                                : nullptr;
        if (h) {
            return h->visitDriver(std::forward<Func>(func));
        }
        // Driver not available - return default
        using ReturnType = decltype(h->visitDriver(std::forward<Func>(func)));
        if constexpr (std::is_void_v<ReturnType>) {
            return;
        } else {
            return ReturnType{};
        }
    }

    //**************************************************************************//
    //**            MECHANICAL FRAME / GEARBOX (Motor <-> Load)               **//
    //**************************************************************************//

    /**
     * @brief Set the mechanical mapping (gearbox + direction) for a device.
     *
     * The mapping is per-device, lives in the manager, and applies to every
     * subsequent setLoadXxx / getLoadXxx call. Default mapping is direct-drive
     * (gear_ratio=1.0, invert=false), in which load-frame == motor-frame.
     *
     * @param mech         Mechanical configuration
     * @param deviceIndex  Device slot (defaults to onboard)
     * @return true if accepted, false if @p deviceIndex is invalid or
     *         @p mech.gear_ratio_motor_to_output is non-positive / non-finite.
     */
    bool setMotorMechanics(const MotorMechanics& mech,
                           uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /**
     * @brief Read back the currently configured mechanical mapping.
     * @param deviceIndex  Device slot (defaults to onboard)
     */
    [[nodiscard]] MotorMechanics getMotorMechanics(
        uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) const noexcept;

    //**************************************************************************//
    //**            MOTOR-FRAME COMMANDS (rotor-side, raw chip frame)         **//
    //**************************************************************************//

    /**
     * @brief Command target velocity in the **motor (rotor) frame**.
     *
     * This is a thin convenience wrapper around the vendor unit-aware API:
     * the value is forwarded straight to `velocityControl.setTargetVelocity`
     * with no gearbox math applied. Use this when you want to think in the
     * frame the silicon thinks in.
     */
    bool setMotorVelocity(double value, ::tmc9660::units::VelocityUnit unit,
                          ::tmc9660::units::MotorContext const& ctx,
                          uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /** @brief Read actual velocity in motor frame. */
    bool getMotorVelocity(double& out_value, ::tmc9660::units::VelocityUnit unit,
                          ::tmc9660::units::MotorContext const& ctx,
                          uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /** @brief Command target position in motor frame. */
    bool setMotorPosition(double value, ::tmc9660::units::PositionUnit unit,
                          ::tmc9660::units::MotorContext const& ctx,
                          uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /** @brief Read actual position in motor frame. */
    bool getMotorPosition(double& out_value, ::tmc9660::units::PositionUnit unit,
                          ::tmc9660::units::MotorContext const& ctx,
                          uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    //**************************************************************************//
    //**            LOAD-FRAME COMMANDS (output-shaft frame, gearbox-aware)   **//
    //**************************************************************************//

    /**
     * @brief Command target velocity in the **load (output-shaft) frame**.
     *
     * The value is multiplied by `gear_ratio_motor_to_output` (and sign-flipped
     * if `invert_output` is set) before being forwarded to the silicon.
     * With the default 1:1 mechanics this is identical to
     * setMotorVelocity().
     *
     * Convention example: a 10:1 reducer with `gear_ratio_motor_to_output =
     * 10` and `setLoadVelocity(60.0, VelocityUnit::Rpm, ctx)` will spin the
     * **rotor** at 600 RPM so the **output** turns at 60 RPM.
     */
    bool setLoadVelocity(double value, ::tmc9660::units::VelocityUnit unit,
                         ::tmc9660::units::MotorContext const& ctx,
                         uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /** @brief Read actual velocity expressed in the load frame. */
    bool getLoadVelocity(double& out_value, ::tmc9660::units::VelocityUnit unit,
                         ::tmc9660::units::MotorContext const& ctx,
                         uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /** @brief Command target position in the load frame (gearbox-aware). */
    bool setLoadPosition(double value, ::tmc9660::units::PositionUnit unit,
                         ::tmc9660::units::MotorContext const& ctx,
                         uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    /** @brief Read actual position expressed in the load frame. */
    bool getLoadPosition(double& out_value, ::tmc9660::units::PositionUnit unit,
                         ::tmc9660::units::MotorContext const& ctx,
                         uint8_t deviceIndex = ONBOARD_TMC9660_INDEX) noexcept;

    //**************************************************************************//
    //**                  DEVICES MANAGEMENT METHODS                           **//
    //**************************************************************************//

    /**
     * @brief Create the onboard TMC9660 device using an SPI interface.
     * @param spiInterface Reference to BaseSpi implementation
     * @param address TMC9660 device address
     * @param pins Host-side GPIO control pin references for the TMC9660
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @return MotorError::SUCCESS if device created, typed error otherwise
     * @note Must be called before Initialize() so the onboard device is registered.
     *       The GPIO pins must already be configured (pin number, direction).
     */
    [[nodiscard]] MotorError CreateOnboardDevice(BaseSpi& spiInterface, 
                            uint8_t address,
                            const Tmc9660ControlPins& pins,
                            const tmc9660::BootloaderConfig* bootCfg = nullptr) noexcept;

    /**
     * @brief Create the onboard TMC9660 device using a UART interface.
     * @param uartInterface Reference to BaseUart implementation
     * @param address TMC9660 device address
     * @param pins Host-side GPIO control pin references for the TMC9660
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @return MotorError::SUCCESS if device created, typed error otherwise
     * @note Must be called before Initialize() so the onboard device is registered.
     *       The GPIO pins must already be configured (pin number, direction).
     */
    [[nodiscard]] MotorError CreateOnboardDevice(BaseUart& uartInterface,
                            uint8_t address,
                            const Tmc9660ControlPins& pins,
                            const tmc9660::BootloaderConfig* bootCfg = nullptr) noexcept;

    /**
     * @brief Create an external TMC9660 device on specified CS line (SPI).
     * @param csDeviceIndex External device CS index (2 or 3 only)
     * @param spiDeviceId SPI device ID for communication (EXTERNAL_DEVICE_1 or EXTERNAL_DEVICE_2)
     * @param address TMC9660 device address
     * @param pins Host-side GPIO control pin references for the TMC9660
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @return MotorError::SUCCESS if device created, typed error otherwise
     * @note Only EXTERNAL_DEVICE_1_INDEX (2) and EXTERNAL_DEVICE_2_INDEX (3) are allowed
     */
    [[nodiscard]] MotorError CreateExternalDevice(uint8_t csDeviceIndex, 
                            SpiDeviceId spiDeviceId, 
                            uint8_t address,
                            const Tmc9660ControlPins& pins,
                            const tmc9660::BootloaderConfig* bootCfg = nullptr) noexcept;

    /**
     * @brief Create an external TMC9660 device on a UART interface.
     * @param csDeviceIndex External device slot index (2 or 3 only)
     * @param uartInterface Reference to BaseUart implementation
     * @param address TMC9660 device address
     * @param pins Host-side GPIO control pin references for the TMC9660
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @return MotorError::SUCCESS if device created, typed error otherwise
     * @note Only EXTERNAL_DEVICE_1_INDEX (2) and EXTERNAL_DEVICE_2_INDEX (3) are allowed
     */
    [[nodiscard]] MotorError CreateExternalDevice(uint8_t csDeviceIndex,
                            BaseUart& uartInterface,
                            uint8_t address,
                            const Tmc9660ControlPins& pins,
                            const tmc9660::BootloaderConfig* bootCfg = nullptr) noexcept;

    /**
     * @brief Delete an external TMC9660 device.
     * @param csDeviceIndex External device CS index (2 or 3 only)
     * @return MotorError::SUCCESS if device deleted, typed error otherwise
     * @note Cannot delete onboard device (index 0). Only external devices can be deleted.
     */
    [[nodiscard]] MotorError DeleteExternalDevice(uint8_t csDeviceIndex) noexcept;

    /**
     * @brief Get the number of active TMC9660 devices.
     * @return Number of active devices (1 to MAX_TMC9660_DEVICES)
     * @note Always includes onboard device, plus any active external devices
     */
    uint8_t GetDeviceCount() const noexcept;

    /**
     * @brief Check if a device index is valid and has an active device.
     * @param deviceIndex Device index to check
     * @return true if device exists and is active, false otherwise
     */
    bool IsDeviceValid(uint8_t deviceIndex) const noexcept;

    /**
     * @brief Check if an external device slot is available for creation.
     * @param csDeviceIndex External device CS index (2 or 3 only)
     * @return true if slot is available, false if occupied or invalid index
     */
    bool IsExternalSlotAvailable(uint8_t csDeviceIndex) const noexcept;

    /**
     * @brief Get indices of all active devices.
     * @param out    Fixed-size output array
     * @param count  Output: number of entries written
     */
    void GetActiveDeviceIndices(std::array<uint8_t, MAX_TMC9660_DEVICES>& out, size_t& count) const noexcept;

    /**
     * @brief Initialize all active devices.
     * @param results  Output: per-slot initialization result
     * @return Number of devices successfully initialized
     */
    size_t InitializeAllDevices(std::array<bool, MAX_TMC9660_DEVICES>& results) noexcept;

    /**
     * @brief Get initialization status of all device slots.
     * @param status  Output: per-slot status
     */
    void GetInitializationStatus(std::array<bool, MAX_TMC9660_DEVICES>& status) const noexcept;
    
    /**
     * @brief Dump comprehensive system statistics to log as INFO level.
     * Logs all device statistics, handler status, and system health information.
     */
    void DumpStatistics() const noexcept;

private:
    MotorController();
    ~MotorController() = default;
    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;

    /**
     * @brief Initialize all registered motor controller devices.
     *
     * Calls Tmc9660Handler::Initialize() on every active device. Devices must
     * be created first via CreateOnboardDevice() / CreateExternalDevice() which
     * supply the required host-side GPIO control pins.
     *
     * @return true if all active devices initialized successfully, false otherwise
     */
    bool Initialize();

    /**
     * @brief Validate if CS device index is for external device.
     * @param csDeviceIndex Device index to validate
     * @return true if index is for external device (2 or 3), false otherwise
     */
    bool IsExternalDeviceIndex(uint8_t csDeviceIndex) const noexcept;

    /**
     * @brief Store a new error code and update per-device counter.
     * @param error The error to record
     * @param deviceIndex Optional device index to bump its error counter
     */
    void UpdateLastError(MotorError error, int deviceIndex = -1) noexcept;

    /** @pre `deviceMutex_` held. Applies PCAL SPI mux for slot @p device_index before handler init. */
    void configureOnboardHostBusBeforeHandlerInitLocked(uint8_t device_index) noexcept;

    /** @pre `deviceMutex_` held. */
    [[nodiscard]] MotorError applyOnboardTmc9660HostSpiGateModeLocked(Tmc9660HostSpiGateMode mode) noexcept;

    /** @pre `deviceMutex_` held. Pushes the stored SPI TMCL timing of @p device_index to its driver. */
    void applyTmclSpiTimingLocked(uint8_t device_index) noexcept;

    /** @pre `deviceMutex_` held. SPI driver of a slot whose handler is initialized, or nullptr. */
    /**
     * @brief Run @p fn on the SPI driver of a ready device, through the handler (handler lock held).
     * @return true if @p fn ran (device active, driver up, transport is SPI). Caller holds deviceMutex_.
     */
    /**
     * @brief Run @p fn(driver) on a ready device through the handler, SPI or UART (handler lock held).
     * @return SUCCESS, or why it did not run (DEVICE_NOT_FOUND / NOT_INITIALIZED). Caller holds deviceMutex_.
     */
    template <typename Fn>
    MotorError withReadyDriverLocked(uint8_t device_index, Fn&& fn) noexcept {
        if (device_index >= MAX_TMC9660_DEVICES || !deviceActive_[device_index] || !tmcHandlers_[device_index]) {
            return MotorError::DEVICE_NOT_FOUND;
        }
        // Never start a bootloader bring-up from an observability call: only drivers that are up.
        if (!tmcHandlers_[device_index]->IsDriverReady()) {
            return MotorError::NOT_INITIALIZED;
        }
        bool ran = false;
        tmcHandlers_[device_index]->visitDriver([&](auto& driver) {
            ran = true;
            fn(driver);
        });
        return ran ? MotorError::SUCCESS : MotorError::NOT_INITIALIZED;
    }

    /** @brief Account one batch in accessStats_. Caller holds deviceMutex_. */
    void recordBatchLocked(uint8_t device_index, size_t accesses, size_t failed, uint32_t duration_us) noexcept;

    template <typename Fn>
    bool withSpiDriverLocked(uint8_t device_index, Fn&& fn) noexcept {
        // A handler that is not up would run its full bootloader init inside visitDriver(); that
        // belongs to the init paths (PCAL SPI mux first), so only ready drivers are touched here.
        if (device_index >= MAX_TMC9660_DEVICES || !deviceActive_[device_index] || !tmcHandlers_[device_index] ||
            !tmcHandlers_[device_index]->IsDriverReady()) {
            return false;
        }
        bool ran = false;
        tmcHandlers_[device_index]->visitDriver([&](auto& driver) {
            if constexpr (std::is_same_v<std::decay_t<decltype(driver)>, Tmc9660Handler::SpiDriver>) {
                ran = true;
                fn(driver);
            }
        });
        return ran;
    }

    /// Handlers in place (no heap): a slot is emplaced by Create*Device() and reset on delete/deinit.
    std::array<std::optional<Tmc9660Handler>, MAX_TMC9660_DEVICES> tmcHandlers_;
    std::array<bool, MAX_TMC9660_DEVICES> deviceInitialized_;
    std::array<bool, MAX_TMC9660_DEVICES> deviceActive_;    ///< Track which devices are active
    std::array<std::atomic<uint32_t>, MAX_TMC9660_DEVICES> deviceErrorCounts_{}; ///< Per-device error counters
    std::array<MotorMechanics, MAX_TMC9660_DEVICES> mechanics_{}; ///< Per-device mechanical mapping (default 1:1)
    std::array<tmc9660::TmclSpiTiming, MAX_TMC9660_DEVICES> tmclSpiTiming_{}; ///< Per-device SPI TMCL timing
    std::array<MotorAccessStats, MAX_TMC9660_DEVICES> accessStats_{};          ///< Per-device TMCL access accounting
    
    bool onboardDeviceCreated_;         ///< Track if onboard device has been created
    std::atomic<bool> initialized_;      ///< Track if system has been initialized
    std::atomic<MotorError> last_error_{MotorError::SUCCESS}; ///< Most recent error code
    mutable RtosMutex deviceMutex_;     ///< RTOS mutex for thread-safe device access
};

//==============================================================================
// CONVENIENCE
//==============================================================================

/**
 * @brief Convenience accessor — equivalent to MotorController::GetInstance().
 * @return Reference to the singleton MotorController.
 */
[[nodiscard]] inline MotorController& GetMotorController() noexcept {
    return MotorController::GetInstance();
}

#endif // COMPONENT_HANDLER_MOTOR_CONTROLLER_H_