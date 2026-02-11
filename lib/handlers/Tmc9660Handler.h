/**
 * @file Tmc9660Handler.h
 * @brief Unified handler for TMC9660 motor controller with GPIO, ADC, and temperature integration.
 *
 * @details
 * This file provides the complete HAL-level integration for a single TMC9660 motor driver
 * device. It bridges the HardFOC base interfaces (BaseSpi, BaseUart, BaseGpio, BaseAdc,
 * BaseTemperature) with the templated tmc9660::TMC9660<CommType> driver from the
 * hf-tmc9660-driver library.
 *
 * ## Architecture Overview
 *
 * The file contains three layers:
 *
 * 1. **CRTP Communication Adapters** (HalSpiTmc9660Comm, HalUartTmc9660Comm):
 *    Bridge HardFOC BaseSpi/BaseUart/BaseGpio to the TMC9660 driver's CRTP-based
 *    communication interfaces (tmc9660::SpiCommInterface, tmc9660::UartCommInterface).
 *    These also manage the four host-side GPIO control pins (RST, DRV_EN, FAULTN, WAKE)
 *    required by the TMC9660 bootloader initialization sequence.
 *
 * 2. **Tmc9660Handler** (main class):
 *    Non-templated facade that owns one typed driver instance (SpiDriver or UartDriver).
 *    Uses the visitDriver() template to route calls to the active driver, keeping
 *    the public API free of template parameters. Provides:
 *    - Bootloader initialization with configurable reset and verification options
 *    - TMCL parameter read/write and raw command access
 *    - Convenience methods for motor control (velocity, position, torque)
 *    - Telemetry (voltage, temperature, current, flags)
 *    - visitDriver() for advanced users needing direct subsystem access
 *
 * 3. **Peripheral Wrappers** (Gpio, Adc, Temperature inner classes):
 *    Implement BaseGpio, BaseAdc, and BaseTemperature respectively, delegating to the
 *    TMC9660 driver's subsystems (gpio, telemetry, etc.) via the handler's visitDriver().
 *
 * ## Ownership Model
 *
 * The handler owns all its internal resources:
 * - One HalSpiTmc9660Comm or HalUartTmc9660Comm (communication adapter)
 * - One SpiDriver or UartDriver (typed TMC9660 driver)
 * - Two Gpio wrappers (for GPIO17, GPIO18)
 * - One Adc wrapper (multi-channel TMC9660 ADC)
 * - One Temperature wrapper (chip temperature sensor)
 *
 * External managers (AdcManager, TemperatureManager) that need to own a BaseAdc* or
 * BaseTemperature* should use the thin delegation wrappers Tmc9660AdcWrapper and
 * Tmc9660TemperatureWrapper (defined in their own files), which delegate to the
 * handler's inner class instances.
 *
 * ## Initialization Sequence
 *
 * @code
 * // 1. Obtain SPI/UART interface and four GPIO control pin instances
 * // 2. Construct the handler
 * Tmc9660Handler handler(spi, rst_gpio, drv_en_gpio, faultn_gpio, wake_gpio, 0x01);
 *
 * // 3. Initialize (runs bootloader config, enters parameter mode)
 * if (!handler.Initialize()) { return; }
 *
 * // 4. Use motor control, telemetry, GPIO, ADC, etc.
 * handler.SetTargetVelocity(1000);
 * float voltage = handler.GetSupplyVoltage();
 * @endcode
 *
 * ## Thread Safety
 *
 * Individual handler methods are NOT thread-safe by themselves. If multiple threads
 * access the same handler, external synchronization is required. The Adc and Temperature
 * inner classes use RtosMutex for their internal statistics tracking.
 *
 * @see Tmc9660AdcWrapper       Thin adapter for AdcManager ownership
 * @see Tmc9660TemperatureWrapper  Thin adapter for TemperatureManager ownership
 * @see MotorController         Higher-level manager that owns Tmc9660Handler instances
 *
 * @author HardFOC Team
 * @date 2025
 */

#ifndef COMPONENT_HANDLER_TMC9660_HANDLER_H_
#define COMPONENT_HANDLER_TMC9660_HANDLER_H_

#include <cstdint>
#include <memory>
#include <array>
#include <atomic>
#include <type_traits>
#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp"
#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660_comm_interface.hpp"
#include "base/BaseGpio.h"
#include "base/BaseAdc.h"
#include "base/BaseTemperature.h"
#include "base/BaseSpi.h"
#include "base/BaseUart.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils/RtosMutex.h"

///////////////////////////////////////////////////////////////////////////////
/// @defgroup TMC9660_HAL_CommAdapters HAL Communication Adapters
/// @brief CRTP communication classes bridging HardFOC interfaces to TMC9660.
/// @{
///////////////////////////////////////////////////////////////////////////////

/**
 * @struct Tmc9660CtrlPins
 * @brief Shared helper holding the four TMC9660 host-side control pin references.
 *
 * @details
 * Both HalSpiTmc9660Comm and HalUartTmc9660Comm require the same four GPIO
 * control pins (RST, DRV_EN, FAULTN, WAKE) and share identical pin-resolution
 * logic. This struct consolidates the four GPIO references and pin lookup into
 * a single definition, eliminating code duplication across communication adapters.
 *
 * @note None of the GPIO references are owned by this struct.
 */
struct Tmc9660CtrlPins {
    BaseGpio& rst;      ///< RST control pin GPIO (not owned).
    BaseGpio& drv_en;   ///< DRV_EN control pin GPIO (not owned).
    BaseGpio& faultn;   ///< FAULTN status pin GPIO (not owned).
    BaseGpio& wake;     ///< WAKE control pin GPIO (not owned).

    /** @brief Resolve a TMC9660CtrlPin enum to the corresponding BaseGpio reference. */
    BaseGpio& get(tmc9660::TMC9660CtrlPin pin) noexcept {
        switch (pin) {
            case tmc9660::TMC9660CtrlPin::RST:    return rst;
            case tmc9660::TMC9660CtrlPin::DRV_EN: return drv_en;
            case tmc9660::TMC9660CtrlPin::FAULTN: return faultn;
            case tmc9660::TMC9660CtrlPin::WAKE:   return wake;
            default:                               return rst;
        }
    }

    /** @brief Resolve a TMC9660CtrlPin enum (const). */
    const BaseGpio& get(tmc9660::TMC9660CtrlPin pin) const noexcept {
        switch (pin) {
            case tmc9660::TMC9660CtrlPin::RST:    return rst;
            case tmc9660::TMC9660CtrlPin::DRV_EN: return drv_en;
            case tmc9660::TMC9660CtrlPin::FAULTN: return faultn;
            case tmc9660::TMC9660CtrlPin::WAKE:   return wake;
            default:                               return rst;
        }
    }
};

/**
 * @class HalSpiTmc9660Comm
 * @brief Concrete SPI communication adapter for TMC9660 using BaseSpi and BaseGpio.
 *
 * @details
 * Implements all methods required by tmc9660::SpiCommInterface<HalSpiTmc9660Comm>
 * through the CRTP (Curiously Recurring Template Pattern). This class acts as the
 * bridge between the HardFOC BaseSpi abstraction and the TMC9660 driver's SPI
 * protocol, handling:
 *
 * - **TMCL transfers**: 8-byte full-duplex SPI exchanges for parameter mode
 * - **Bootloader transfers**: 5-byte full-duplex SPI exchanges for bootloader mode
 * - **GPIO control**: Maps TMC9660 control pin identifiers (RST, DRV_EN, FAULTN, WAKE)
 *   to physical BaseGpio instances with configurable active-level polarity
 * - **Timing**: Platform-aware delays (millisecond via RTOS, microsecond via
 *   esp_rom_delay_us or busy-wait fallback)
 * - **Logging**: Routes TMC9660 driver debug output to the HardFOC Logger
 *
 * ## Control Pin Polarity
 *
 * Each control pin can be configured as active-high or active-low to match the
 * board's electrical design (e.g., inverting buffers). The constructor parameters
 * set the initial polarity; the base class provides set_pin_active_level() for
 * runtime reconfiguration if needed.
 *
 * @note This class does NOT own the BaseSpi or BaseGpio instances; they must
 *       remain valid for the lifetime of this object.
 *
 * @see tmc9660::SpiCommInterface  CRTP base class from the TMC9660 driver
 * @see HalUartTmc9660Comm         UART equivalent of this class
 */
class HalSpiTmc9660Comm : public tmc9660::SpiCommInterface<HalSpiTmc9660Comm> {
public:
    /**
     * @brief Construct the SPI communication adapter.
     *
     * @param spi              Reference to a pre-configured BaseSpi implementation
     *                         (SPI mode 3, 8-bit frames, chip select managed externally).
     * @param rst              BaseGpio connected to TMC9660 RST (pin 22). Must be
     *                         pre-configured as output.
     * @param drv_en           BaseGpio connected to TMC9660 DRV_EN (pin 21). Must be
     *                         pre-configured as output.
     * @param faultn           BaseGpio connected to TMC9660 FAULTN (pin 20). Must be
     *                         pre-configured as input.
     * @param wake             BaseGpio connected to TMC9660 WAKE (pin 19). Must be
     *                         pre-configured as output.
     * @param rst_active_high  Physical GPIO level when RST is ACTIVE (default: true = HIGH).
     * @param drv_en_active_high Physical GPIO level when DRV_EN is ACTIVE (default: true = HIGH).
     * @param faultn_active_low  Physical GPIO level when FAULTN is ACTIVE (default: false = HIGH).
     *                           Set to true if your board inverts the FAULTN signal.
     * @param wake_active_low    Physical GPIO level when WAKE is ACTIVE (default: false = HIGH).
     *                           Set to true if your board inverts the WAKE signal.
     */
    HalSpiTmc9660Comm(BaseSpi& spi, BaseGpio& rst, BaseGpio& drv_en,
                       BaseGpio& faultn, BaseGpio& wake,
                       bool rst_active_high = true, bool drv_en_active_high = true,
                       bool faultn_active_low = false, bool wake_active_low = false) noexcept;

    /// @name CRTP-Required Methods
    /// These are called by tmc9660::SpiCommInterface<HalSpiTmc9660Comm> via static dispatch.
    /// @{

    /**
     * @brief Perform an 8-byte full-duplex SPI transfer for TMCL parameter mode.
     * @param[in,out] tx 8-byte transmit buffer (TMCL command frame).
     * @param[out]    rx 8-byte receive buffer (TMCL reply frame).
     * @return true if BaseSpi::Transfer() succeeded.
     */
    bool spiTransferTMCL(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept;

    /**
     * @brief Perform a 5-byte full-duplex SPI transfer for bootloader mode.
     * @param[in,out] tx 5-byte transmit buffer (bootloader command).
     * @param[out]    rx 5-byte receive buffer (bootloader reply).
     * @return true if BaseSpi::Transfer() succeeded.
     */
    bool spiTransferBootloader(std::array<uint8_t, 5>& tx, std::array<uint8_t, 5>& rx) noexcept;

    /**
     * @brief Set a TMC9660 control pin to a given logical signal state.
     *
     * Translates the logical ACTIVE/INACTIVE signal to a physical HIGH/LOW level
     * based on the pin's configured active-level polarity, then writes via BaseGpio.
     *
     * @param pin    Which control pin to set (RST, DRV_EN, FAULTN, WAKE).
     * @param signal Desired logical signal state.
     * @return true if the GPIO write succeeded.
     */
    bool gpioSet(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal signal) noexcept;

    /**
     * @brief Read a TMC9660 control pin's current logical signal state.
     *
     * Reads the physical GPIO level via BaseGpio, then translates to ACTIVE/INACTIVE
     * based on the pin's configured active-level polarity.
     *
     * @param pin       Which control pin to read (typically FAULTN for fault status).
     * @param[out] signal  Current logical signal state.
     * @return true if the GPIO read succeeded.
     */
    bool gpioRead(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal& signal) noexcept;

    /**
     * @brief Route TMC9660 driver debug output to the HardFOC Logger.
     * @param level  Log level (0=Error, 1=Warning, 2=Info, 3=Debug).
     * @param tag    Log tag string for categorization.
     * @param format printf-style format string.
     * @param args   Variable argument list.
     */
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept;

    /**
     * @brief Delay execution for the specified number of milliseconds.
     * @param ms Milliseconds to delay (delegates to os_delay_msec).
     */
    void delayMs(uint32_t ms) noexcept;

    /**
     * @brief Delay execution for the specified number of microseconds.
     *
     * On ESP32, uses esp_rom_delay_us() for accurate sub-millisecond timing.
     * On other platforms, uses a busy-wait loop based on the processor cycle counter.
     *
     * @param us Microseconds to delay.
     */
    void delayUs(uint32_t us) noexcept;

    /// @}

private:
    BaseSpi&  spi_;             ///< SPI bus interface (not owned).
    Tmc9660CtrlPins ctrl_pins_; ///< Host-side control pin references.
};

/**
 * @class HalUartTmc9660Comm
 * @brief Concrete UART communication adapter for TMC9660 using BaseUart and BaseGpio.
 *
 * @details
 * Implements all methods required by tmc9660::UartCommInterface<HalUartTmc9660Comm>
 * through the CRTP pattern. Handles:
 *
 * - **TMCL transfers**: 9-byte send/receive for parameter mode
 * - **Bootloader transfers**: 8-byte send/receive for bootloader mode
 * - **GPIO control**: Same four control pins as HalSpiTmc9660Comm
 * - **Timing and logging**: Same platform-aware implementations
 *
 * @note This class does NOT own the BaseUart or BaseGpio instances; they must
 *       remain valid for the lifetime of this object.
 *
 * @see tmc9660::UartCommInterface  CRTP base class from the TMC9660 driver
 * @see HalSpiTmc9660Comm           SPI equivalent of this class
 */
class HalUartTmc9660Comm : public tmc9660::UartCommInterface<HalUartTmc9660Comm> {
public:
    /**
     * @brief Construct the UART communication adapter.
     *
     * @param uart             Reference to a pre-configured BaseUart implementation.
     * @param rst              BaseGpio connected to TMC9660 RST (pin 22, output).
     * @param drv_en           BaseGpio connected to TMC9660 DRV_EN (pin 21, output).
     * @param faultn           BaseGpio connected to TMC9660 FAULTN (pin 20, input).
     * @param wake             BaseGpio connected to TMC9660 WAKE (pin 19, output).
     * @param rst_active_high  Physical level for RST ACTIVE state (default: true).
     * @param drv_en_active_high Physical level for DRV_EN ACTIVE state (default: true).
     * @param faultn_active_low  Physical level for FAULTN ACTIVE state (default: false).
     * @param wake_active_low    Physical level for WAKE ACTIVE state (default: false).
     */
    HalUartTmc9660Comm(BaseUart& uart, BaseGpio& rst, BaseGpio& drv_en,
                        BaseGpio& faultn, BaseGpio& wake,
                        bool rst_active_high = true, bool drv_en_active_high = true,
                        bool faultn_active_low = false, bool wake_active_low = false) noexcept;

    /// @name CRTP-Required Methods
    /// @{

    /**
     * @brief Send a 9-byte TMCL datagram over UART.
     * @param data 9-byte array: [sync+addr, opcode, type, motor, value(4), checksum].
     * @return true if BaseUart::Write() succeeded.
     */
    bool uartSendTMCL(const std::array<uint8_t, 9>& data) noexcept;

    /**
     * @brief Receive a 9-byte TMCL reply datagram over UART.
     * @param[out] data 9-byte array to store the received reply.
     * @return true if BaseUart::Read() succeeded within timeout.
     */
    bool uartReceiveTMCL(std::array<uint8_t, 9>& data) noexcept;

    /**
     * @brief Perform an 8-byte UART bootloader transfer (send + receive).
     * @param tx 8-byte transmit buffer (bootloader command).
     * @param[out] rx 8-byte receive buffer (bootloader reply).
     * @return true if both write and read succeeded.
     */
    bool uartTransferBootloader(const std::array<uint8_t, 8>& tx,
                                 std::array<uint8_t, 8>& rx) noexcept;

    /// @copydoc HalSpiTmc9660Comm::gpioSet
    bool gpioSet(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal signal) noexcept;

    /// @copydoc HalSpiTmc9660Comm::gpioRead
    bool gpioRead(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal& signal) noexcept;

    /// @copydoc HalSpiTmc9660Comm::debugLog
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept;

    /// @copydoc HalSpiTmc9660Comm::delayMs
    void delayMs(uint32_t ms) noexcept;

    /// @copydoc HalSpiTmc9660Comm::delayUs
    void delayUs(uint32_t us) noexcept;

    /// @}

private:
    BaseUart& uart_;            ///< UART bus interface (not owned).
    Tmc9660CtrlPins ctrl_pins_; ///< Host-side control pin references.
};

/// @} // end of TMC9660_HAL_CommAdapters

///////////////////////////////////////////////////////////////////////////////
/// @defgroup TMC9660_HAL_Handler TMC9660 Handler
/// @brief Non-templated facade for TMC9660 motor controller integration.
/// @{
///////////////////////////////////////////////////////////////////////////////

/**
 * @class Tmc9660Handler
 * @brief Unified, non-templated handler for a single TMC9660 motor controller device.
 *
 * @details
 * Tmc9660Handler is the primary interface that application and manager code uses to
 * interact with a TMC9660 motor driver. It hides the template complexity of
 * tmc9660::TMC9660<CommType> behind a clean, polymorphism-free API.
 *
 * ## Design Decisions
 *
 * - **Non-templated**: The handler uses visitDriver() to route calls to the
 *   correct typed driver (SPI or UART). This prevents template propagation
 *   through the codebase and allows MotorController to store handlers in a
 *   plain std::unique_ptr<Tmc9660Handler>.
 *
 * - **Control pins required**: The TMC9660 bootloader initialization sequence
 *   requires host-side GPIO control of four pins (RST, DRV_EN, FAULTN, WAKE).
 *   These must be provided at construction time.
 *
 * - **Lazy driver creation**: The typed TMC9660 driver instance is created during
 *   Initialize(), not in the constructor. This saves memory until initialization
 *   is actually needed.
 *
 * - **Peripheral wrappers**: Inner classes Gpio, Adc, and Temperature implement
 *   the HardFOC base interfaces, making TMC9660 peripherals available to the
 *   manager layer (GpioManager, AdcManager, TemperatureManager) through the
 *   standard abstractions.
 *
 * ## Usage Example
 *
 * @code
 * // Construction (SPI mode, address 1, default boot config)
 * Tmc9660Handler handler(spi_bus, rst_pin, drv_en_pin, faultn_pin, wake_pin, 0x01);
 *
 * // Initialize (performs hardware reset and bootloader configuration)
 * if (!handler.Initialize()) {
 *     Logger::GetInstance().Error("App", "TMC9660 init failed!");
 *     return;
 * }
 *
 * // Motor control
 * handler.SetMotorType(tmc9660::tmcl::MotorType::BLDC, 7);
 * handler.SetCommutationMode(tmc9660::tmcl::CommutationMode::FOC);
 * handler.EnableMotor();
 * handler.SetTargetVelocity(1000);
 *
 * // Telemetry
 * float supply_v = handler.GetSupplyVoltage();
 * float chip_temp = handler.GetChipTemperature();
 *
 * // GPIO (TMC9660 internal GPIO17)
 * handler.gpio(17).SetPinLevel(hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
 *
 * // ADC (read AIN channel 0)
 * float voltage;
 * handler.adc().ReadChannelV(0, voltage);
 *
 * // Advanced: direct driver access via visitor
 * handler.visitDriver([](auto& driver) {
 *     driver.feedbackSense.configureHallSensor(1, 2, 3);
 *     driver.protection.setOvertemperatureLimit(120.0f);
 * });
 * @endcode
 *
 * @see HalSpiTmc9660Comm    SPI communication adapter
 * @see HalUartTmc9660Comm   UART communication adapter
 * @see MotorController       Multi-device manager that owns Tmc9660Handler instances
 * @see Tmc9660AdcWrapper     Thin BaseAdc adapter for AdcManager ownership
 */
class Tmc9660Handler {
public:
    //==========================================================================
    /// @name Type Aliases
    /// @{
    //==========================================================================

    /** @brief TMC9660 driver instantiated with SPI communication. */
    using SpiDriver  = tmc9660::TMC9660<HalSpiTmc9660Comm>;

    /** @brief TMC9660 driver instantiated with UART communication. */
    using UartDriver = tmc9660::TMC9660<HalUartTmc9660Comm>;

    /// @}

    //==========================================================================
    /// @name Default Configuration
    /// @{
    //==========================================================================

    /**
     * @brief Default bootloader configuration based on TMC9660-3PH-EVAL board settings.
     *
     * @details Configures:
     * - LDO: VEXT1=5.0V, VEXT2=3.3V, 3ms slope
     * - Boot mode: Parameter mode with motor control start
     * - UART: Auto16x baud rate, GPIO6/7, address 1
     * - External clock: 16MHz crystal with PLL (40MHz system)
     * - SPI Flash: Enabled on SPI0 (GPIO11/12) at 10MHz
     * - GPIO: GPIO5 analog, GPIO17/18 digital pull-down
     * - Hall, ABN encoders, step/dir, etc.: disabled (safe defaults)
     *
     * Override by passing a custom tmc9660::BootloaderConfig* to the constructor.
     */
    static const tmc9660::BootloaderConfig kDefaultBootConfig;

    /// @}

    //==========================================================================
    /// @name Construction and Destruction
    /// @{
    //==========================================================================

    /**
     * @brief Construct a handler for a TMC9660 connected via SPI.
     *
     * The communication adapter (HalSpiTmc9660Comm) is created immediately, but the
     * typed TMC9660 driver instance is deferred until Initialize() is called.
     *
     * @param spi      Reference to an initialized BaseSpi bus connected to the TMC9660.
     *                 Must use SPI Mode 3 (CPOL=1, CPHA=1), MSB-first.
     * @param rst      BaseGpio for the TMC9660 RST control pin (host-side output).
     *                 Must be pre-configured with correct pin number and output direction.
     * @param drv_en   BaseGpio for the TMC9660 DRV_EN control pin (host-side output).
     * @param faultn   BaseGpio for the TMC9660 FAULTN status pin (host-side input).
     * @param wake     BaseGpio for the TMC9660 WAKE control pin (host-side output).
     * @param address  7-bit device address used in TMCL communication (default: 0).
     * @param bootCfg  Pointer to bootloader configuration. The pointed-to struct must
     *                 remain valid for the handler's lifetime. Pass nullptr or omit
     *                 to use kDefaultBootConfig.
     *
     * @warning All GPIO and SPI references must outlive this handler.
     */
    Tmc9660Handler(BaseSpi& spi, BaseGpio& rst, BaseGpio& drv_en,
                   BaseGpio& faultn, BaseGpio& wake,
                   uint8_t address = 0,
                   const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig);

    /**
     * @brief Construct a handler for a TMC9660 connected via UART.
     *
     * @param uart     Reference to an initialized BaseUart interface connected to the TMC9660.
     * @param rst      BaseGpio for the TMC9660 RST control pin (host-side output).
     * @param drv_en   BaseGpio for the TMC9660 DRV_EN control pin (host-side output).
     * @param faultn   BaseGpio for the TMC9660 FAULTN status pin (host-side input).
     * @param wake     BaseGpio for the TMC9660 WAKE control pin (host-side output).
     * @param address  7-bit device address used in TMCL communication (default: 0).
     * @param bootCfg  Pointer to bootloader configuration (default: kDefaultBootConfig).
     *
     * @warning All GPIO and UART references must outlive this handler.
     */
    Tmc9660Handler(BaseUart& uart, BaseGpio& rst, BaseGpio& drv_en,
                   BaseGpio& faultn, BaseGpio& wake,
                   uint8_t address = 0,
                   const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig);

    /**
     * @brief Destructor. Releases the driver, communication adapter, and all wrappers.
     */
    ~Tmc9660Handler();

    /// Non-copyable.
    Tmc9660Handler(const Tmc9660Handler&) = delete;
    /// Non-copyable.
    Tmc9660Handler& operator=(const Tmc9660Handler&) = delete;

    /// @}

    //==========================================================================
    /// @name Initialization
    /// @{
    //==========================================================================

    /**
     * @brief Initialize the TMC9660 device.
     *
     * @details Performs the following sequence:
     * 1. Creates the typed TMC9660 driver instance (SpiDriver or UartDriver).
     * 2. Executes the bootloader initialization sequence via the driver's
     *    bootloaderInit() method, which:
     *    - Optionally performs a hardware reset via the RST pin
     *    - Writes the bootloader configuration words
     *    - Optionally verifies each configuration word readback
     *    - Enters parameter mode for TMCL operation
     * 3. Creates the GPIO, ADC, and Temperature peripheral wrappers.
     *
     * @param performReset           If true, assert RST pin to perform a hardware
     *                               reset before configuration (default: true).
     * @param retrieveBootloaderInfo If true, read the bootloader firmware version
     *                               and log it (default: true).
     * @param failOnVerifyError      If true, return false if any bootloader config
     *                               word fails readback verification. If false,
     *                               log a warning but continue (default: false).
     * @return true if initialization completed successfully, false on any failure.
     *
     * @note Can be called multiple times; subsequent calls skip driver creation
     *       if already initialized.
     * @warning This method must be called before any motor control, telemetry,
     *          or peripheral access methods.
     */
    bool Initialize(bool performReset = true, bool retrieveBootloaderInfo = true,
                    bool failOnVerifyError = false);

    /**
     * @brief Check if the TMC9660 driver has been initialized and is ready.
     *
     * @return true if Initialize() has completed successfully and the driver
     *         instance exists, false otherwise.
     */
    bool IsDriverReady() const noexcept;

    /// @}

    //==========================================================================
    /// @name Core Parameter Access
    /// @brief Low-level TMCL parameter read/write and raw command interface.
    /// @{
    //==========================================================================

    /**
     * @brief Write a value to a TMCL parameter.
     *
     * @param id         The TMCL parameter identifier (from tmc9660::tmcl::Parameters enum).
     * @param value      The 32-bit value to write.
     * @param motorIndex Motor/bank index for multi-motor parameters (default: 0).
     * @return true if the parameter write was acknowledged successfully.
     *
     * @pre IsDriverReady() == true
     */
    bool WriteParameter(tmc9660::tmcl::Parameters id, uint32_t value,
                        uint8_t motorIndex = 0) noexcept;

    /**
     * @brief Read a value from a TMCL parameter.
     *
     * @param id              The TMCL parameter identifier.
     * @param[out] value      Reference to store the 32-bit read value.
     * @param motorIndex      Motor/bank index (default: 0).
     * @return true if the parameter was read successfully.
     *
     * @pre IsDriverReady() == true
     */
    bool ReadParameter(tmc9660::tmcl::Parameters id, uint32_t& value,
                       uint8_t motorIndex = 0) noexcept;

    /**
     * @brief Send a raw TMCL command and optionally receive the reply value.
     *
     * Use this for operations not covered by the convenience methods (e.g.,
     * custom opcodes, firmware update commands).
     *
     * @param opcode TMCL operation code (from tmc9660::tmcl::Op enum).
     * @param type   Parameter/command type field (12-bit).
     * @param motor  Motor/bank index field (4-bit).
     * @param value  32-bit command value.
     * @param[out] reply Optional pointer to store the 32-bit reply value.
     *                   Pass nullptr to discard the reply.
     * @return true if the command was sent and a valid reply was received.
     *
     * @pre IsDriverReady() == true
     */
    bool SendCommand(tmc9660::tmcl::Op opcode, uint16_t type, uint8_t motor,
                     uint32_t value, uint32_t* reply = nullptr) noexcept;

    /// @}

    //==========================================================================
    /// @name Motor Control Convenience Methods
    /// @brief High-level methods that map to TMC9660 motor configuration and
    ///        control subsystems. All require IsDriverReady() == true.
    /// @{
    //==========================================================================

    /**
     * @brief Configure the motor type and number of pole pairs.
     * @param type      Motor type (e.g., BLDC, DC, Stepper).
     * @param polePairs Number of pole pairs (typically 1-14 for BLDC).
     * @return true if the parameters were written successfully.
     */
    bool SetMotorType(tmc9660::tmcl::MotorType type, uint8_t polePairs) noexcept;

    /**
     * @brief Set the PWM switching frequency.
     * @param freq_hz Desired PWM frequency in Hz (e.g., 25000 for 25kHz).
     * @return true if the parameter was written successfully.
     */
    bool SetPWMFrequency(uint32_t freq_hz) noexcept;

    /**
     * @brief Set the commutation mode.
     * @param mode Commutation mode (e.g., FOC, block commutation, open-loop).
     * @return true if the parameter was written successfully.
     */
    bool SetCommutationMode(tmc9660::tmcl::CommutationMode mode) noexcept;

    /**
     * @brief Enable the motor driver output stage.
     * @return true if the enable command was acknowledged.
     */
    bool EnableMotor() noexcept;

    /**
     * @brief Disable the motor driver output stage (coast/brake behavior
     *        depends on gate driver configuration).
     * @return true if the disable command was acknowledged.
     */
    bool DisableMotor() noexcept;

    /**
     * @brief Set the target velocity for velocity control mode.
     * @param velocity Target velocity in internal units (sign indicates direction).
     * @return true if the parameter was written successfully.
     */
    bool SetTargetVelocity(int32_t velocity) noexcept;

    /**
     * @brief Set the target position for position control mode.
     * @param position Target position in encoder counts.
     * @return true if the parameter was written successfully.
     */
    bool SetTargetPosition(int32_t position) noexcept;

    /**
     * @brief Set the target torque for torque control mode.
     * @param torque_ma Target torque current in milliamps (sign indicates direction).
     * @return true if the parameter was written successfully.
     */
    bool SetTargetTorque(int16_t torque_ma) noexcept;

    //--------------------------------------------------------------------------
    // DRV_EN (Driver Enable) Pin Control
    //--------------------------------------------------------------------------

    /**
     * @brief Assert the DRV_EN control pin (enable the TMC9660 power stage hardware).
     *
     * @details
     * This controls the physical DRV_EN GPIO pin connected to the TMC9660, which
     * gates the power stage at the hardware level. This is distinct from
     * EnableMotor()/DisableMotor(), which issue TMCL software commands.
     *
     * Typical startup sequence:
     *   1. EnableDriverOutput()    -- hardware gate on
     *   2. CalibrateCurrentSensing() -- ADC offset calibration
     *   3. EnableMotor()           -- software enable
     *   4. SetTargetVelocity(...)  -- begin control
     *
     * @return true if the GPIO was set successfully.
     * @pre IsDriverReady() == true
     */
    bool EnableDriverOutput() noexcept;

    /**
     * @brief De-assert the DRV_EN control pin (disable the TMC9660 power stage hardware).
     * @return true if the GPIO was set successfully.
     * @pre IsDriverReady() == true
     */
    bool DisableDriverOutput() noexcept;

    //--------------------------------------------------------------------------
    // Feedback Sensor Configuration
    //--------------------------------------------------------------------------

    /**
     * @brief Configure Hall sensor feedback with common parameters.
     *
     * @param sector_offset Hall sector offset angle (default: 0 degrees).
     * @param inverted      Direction inversion for Hall signals.
     * @param enable_extrapolation Enable angle extrapolation between Hall transitions
     *                              for smoother commutation.
     * @param filter_length Digital filter length for Hall input debouncing (0 = none).
     * @return true if all parameters were written successfully.
     * @pre IsDriverReady() == true
     */
    bool ConfigureHallSensor(
        tmc9660::tmcl::HallSectorOffset sector_offset = tmc9660::tmcl::HallSectorOffset::DEG_0,
        tmc9660::tmcl::Direction inverted = tmc9660::tmcl::Direction::NOT_INVERTED,
        tmc9660::tmcl::EnableDisable enable_extrapolation = tmc9660::tmcl::EnableDisable::DISABLED,
        uint8_t filter_length = 0) noexcept;

    /**
     * @brief Configure ABN (incremental) encoder feedback.
     *
     * @param counts_per_rev Encoder counts per mechanical revolution (PPR * 4 for quadrature).
     * @param inverted       Direction inversion for encoder signals.
     * @param n_channel_inverted Invert the N (index) channel polarity.
     * @return true if all parameters were written successfully.
     * @pre IsDriverReady() == true
     */
    bool ConfigureABNEncoder(
        uint32_t counts_per_rev,
        tmc9660::tmcl::Direction inverted = tmc9660::tmcl::Direction::NOT_INVERTED,
        tmc9660::tmcl::EnableDisable n_channel_inverted =
            tmc9660::tmcl::EnableDisable::DISABLED) noexcept;

    //--------------------------------------------------------------------------
    // Current Sensing Calibration
    //--------------------------------------------------------------------------

    /**
     * @brief Run ADC offset calibration for current sensing.
     *
     * @details
     * Must be called after the power stage is enabled (EnableDriverOutput())
     * but before any motor motion. The motor must be at standstill during
     * calibration so that the zero-current baseline is measured accurately.
     *
     * @param wait_for_completion If true, blocks until calibration finishes or times out.
     * @param timeout_ms          Maximum wait time in milliseconds (only used if waiting).
     * @return true if calibration completed (or started, if not waiting) successfully.
     * @pre IsDriverReady() == true
     */
    bool CalibrateCurrentSensing(bool wait_for_completion = true,
                                  uint32_t timeout_ms = 1000) noexcept;

    //--------------------------------------------------------------------------
    // PID Loop Gains
    //--------------------------------------------------------------------------

    /**
     * @brief Set current (torque/flux) PI loop gains.
     *
     * @param p         Proportional gain for both torque and flux (or torque only if separate=true).
     * @param i         Integral gain for both torque and flux (or torque only if separate=true).
     * @param separate  If true, use separate gains for torque and flux loops.
     * @param flux_p    Flux-loop P gain (only used when separate=true).
     * @param flux_i    Flux-loop I gain (only used when separate=true).
     * @return true if all parameters were written successfully.
     * @pre IsDriverReady() == true
     */
    bool SetCurrentLoopGains(uint16_t p, uint16_t i,
                              bool separate = false,
                              uint16_t flux_p = 0, uint16_t flux_i = 0) noexcept;

    /**
     * @brief Set velocity PI loop gains.
     * @param p Proportional gain.
     * @param i Integral gain.
     * @return true if all parameters were written successfully.
     * @pre IsDriverReady() == true
     */
    bool SetVelocityLoopGains(uint16_t p, uint16_t i) noexcept;

    /**
     * @brief Set position PI loop gains.
     * @param p Proportional gain.
     * @param i Integral gain.
     * @return true if all parameters were written successfully.
     * @pre IsDriverReady() == true
     */
    bool SetPositionLoopGains(uint16_t p, uint16_t i) noexcept;

    /// @}

    //==========================================================================
    /// @name Telemetry
    /// @brief Real-time monitoring of voltages, temperatures, currents, and flags.
    ///        All require IsDriverReady() == true.
    /// @{
    //==========================================================================

    /**
     * @brief Read the motor supply voltage.
     * @return Supply voltage in volts, or NaN if the driver is not ready.
     */
    float GetSupplyVoltage() noexcept;

    /**
     * @brief Read the TMC9660 internal chip temperature.
     * @return Chip temperature in degrees Celsius, or NaN if not ready.
     */
    float GetChipTemperature() noexcept;

    /**
     * @brief Read the instantaneous motor phase current.
     * @return Motor current in milliamps, or 0 if not ready.
     */
    int16_t GetMotorCurrent() noexcept;

    /**
     * @brief Read the actual (measured) motor velocity.
     * @return Actual velocity in internal units, or 0 if not ready.
     */
    int32_t GetActualVelocity() noexcept;

    /**
     * @brief Read the actual (measured) motor position.
     * @return Actual position in encoder counts, or 0 if not ready.
     */
    int32_t GetActualPosition() noexcept;

    /**
     * @brief Read the external temperature sensor raw value.
     *
     * The TMC9660 supports an external NTC thermistor on AIN3.
     * The raw value must be converted to temperature using the NTC calibration.
     *
     * @return Raw ADC value from the external temperature sensor, or 0 if not ready.
     */
    uint16_t GetExternalTemperature() noexcept;

    /**
     * @brief Read the TMC9660 general status flags register.
     * @param[out] flags 32-bit status flags (bit meanings defined in TMC9660 datasheet).
     * @return true if the flags were read successfully.
     */
    bool GetStatusFlags(uint32_t& flags) noexcept;

    /**
     * @brief Read the TMC9660 general error flags register.
     * @param[out] flags 32-bit error flags.
     * @return true if the flags were read successfully.
     */
    bool GetErrorFlags(uint32_t& flags) noexcept;

    /**
     * @brief Clear general error flags.
     * @param mask Bitmask of error flags to clear (default: 0xFFFFFFFF = clear all).
     * @return true if the clear command was acknowledged.
     */
    bool ClearErrorFlags(uint32_t mask = 0xFFFFFFFF) noexcept;

    /**
     * @brief Read the gate driver error flags register.
     * @param[out] flags 32-bit gate driver error flags.
     * @return true if the flags were read successfully.
     */
    bool GetGateDriverErrorFlags(uint32_t& flags) noexcept;

    /**
     * @brief Clear gate driver error flags.
     * @param mask Bitmask of flags to clear (default: 0xFFFFFFFF = clear all).
     * @return true if the clear command was acknowledged.
     */
    bool ClearGateDriverErrorFlags(uint32_t mask = 0xFFFFFFFF) noexcept;

    /// @}

    //==========================================================================
    /// @name Bootloader Access
    /// @{
    //==========================================================================

    /**
     * @brief Command the TMC9660 to leave parameter mode and enter bootloader mode.
     *
     * @warning After calling this, TMCL parameter commands will no longer work.
     *          A full hardware reset and re-initialization is required to return
     *          to parameter mode.
     *
     * @return true if the command was sent successfully.
     */
    bool EnterBootloaderMode() noexcept;

    /// @}

    //==========================================================================
    /// @name Peripheral Wrappers
    /// @brief Inner classes that adapt TMC9660 peripherals to HardFOC base interfaces.
    /// @{
    //==========================================================================

    /**
     * @class Gpio
     * @brief BaseGpio adapter for a single TMC9660 internal GPIO channel.
     *
     * @details
     * Wraps one of the TMC9660's internal GPIO pins (e.g., GPIO17, GPIO18) as a
     * HardFOC BaseGpio instance. Supports digital read/write via the TMC9660 driver's
     * gpio subsystem.
     *
     * The TMC9660 GPIO pins are configured as push-pull outputs by default. Pull
     * mode is limited to floating (the TMC9660 configures pull via bootloader config,
     * not at runtime through TMCL).
     *
     * @note Only GPIO17 and GPIO18 are currently exposed. Additional pins can be
     *       added by expanding the gpioWrappers_ array.
     */
    class Gpio : public BaseGpio {
    public:
        /**
         * @brief Construct a GPIO wrapper for a specific TMC9660 internal pin.
         * @param parent     Reference to the owning Tmc9660Handler.
         * @param gpioNumber TMC9660 internal GPIO number (e.g., 17 or 18).
         */
        Gpio(Tmc9660Handler& parent, uint8_t gpioNumber);

        /** @brief Default destructor. */
        ~Gpio() override = default;

        /// @name BaseGpio Interface Implementation
        /// @{

        /** @brief Initialize the GPIO pin (configures as output by default on the TMC9660). */
        bool Initialize() noexcept override;

        /** @brief Deinitialize (no-op; TMC9660 GPIO persists until device reset). */
        bool Deinitialize() noexcept override;

        /** @brief Check if this GPIO number is a valid TMC9660 pin (17 or 18). */
        bool IsPinAvailable() const noexcept override;

        /** @brief Returns 2 (GPIO17 and GPIO18 are exposed). */
        hf_u8_t GetMaxPins() const noexcept override;

        /** @brief Returns a description string like "TMC9660 GPIO17". */
        const char* GetDescription() const noexcept override;

        /// @}

    protected:
        /// @name BaseGpio Protected Implementation
        /// @{

        /** @brief Set pin direction (INPUT or OUTPUT supported). */
        hf_gpio_err_t SetDirectionImpl(hf_gpio_direction_t direction) noexcept override;

        /** @brief Set output mode. Only PUSH_PULL is supported. */
        hf_gpio_err_t SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept override;

        /** @brief Set pull mode. Only FLOATING is supported. */
        hf_gpio_err_t SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept override;

        /** @brief Write HIGH or LOW to the TMC9660 GPIO pin via the driver. */
        hf_gpio_err_t SetPinLevelImpl(hf_gpio_level_t level) noexcept override;

        /** @brief Read the current level of the TMC9660 GPIO pin via the driver. */
        hf_gpio_err_t GetPinLevelImpl(hf_gpio_level_t& level) noexcept override;

        /** @brief Returns FLOATING (TMC9660 pull is set via bootloader config). */
        hf_gpio_pull_mode_t GetPullModeImpl() const noexcept override;

        /** @brief Returns the current configured direction (INPUT or OUTPUT). */
        hf_gpio_err_t GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept override;

        /** @brief Returns PUSH_PULL. */
        hf_gpio_err_t GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept override;

        /// @}

    private:
        Tmc9660Handler& parent_;        ///< Owning handler instance.
        uint8_t gpioNumber_;            ///< TMC9660 internal GPIO pin number.
        char description_[32];          ///< Human-readable description (e.g., "TMC9660 GPIO17").
        hf_gpio_direction_t direction_ = hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT; ///< Current direction.
    };

    /**
     * @class Adc
     * @brief BaseAdc adapter for all TMC9660 ADC channels.
     *
     * @details
     * Provides a unified BaseAdc interface for reading the TMC9660's various analog
     * and telemetry data channels. Channels are identified by a numeric ID scheme:
     *
     * | Channel ID Range | Type               | Description                     |
     * |:----------------:|:-------------------|:--------------------------------|
     * | 0 - 3            | AIN channels       | External analog inputs (GPIO5)  |
     * | 10 - 13           | Current sense      | Phase current ADC (I0-I3)       |
     * | 20 - 21           | Voltage monitor    | 20=supply, 21=driver voltage    |
     * | 30 - 31           | Temperature        | 30=chip, 31=external NTC        |
     * | 40 - 42           | Motor data         | 40=current, 41=velocity, 42=pos |
     *
     * The wrapper includes thread-safe statistics and diagnostics tracking via
     * RtosMutex. Voltage conversions use a default 3.3V / 16-bit scale; for
     * temperature and motor data channels, the "voltage" field contains the
     * physical value directly (degrees Celsius, mA, etc.).
     *
     * @note This inner class is owned by the handler. For manager-layer ownership,
     *       use Tmc9660AdcWrapper which delegates to this instance.
     *
     * @see Tmc9660AdcWrapper  Thin delegation wrapper for AdcManager ownership
     */
    class Adc : public BaseAdc {
    public:
        /**
         * @brief Construct the ADC wrapper.
         * @param parent Reference to the owning Tmc9660Handler.
         */
        Adc(Tmc9660Handler& parent);

        /** @brief Default destructor. */
        ~Adc() override = default;

        /// @name BaseAdc Interface Implementation
        /// @{

        /** @brief Initialize ADC wrapper (no-op; TMC9660 ADC is always available). */
        bool Initialize() noexcept override;

        /** @brief Deinitialize ADC wrapper (no-op). */
        bool Deinitialize() noexcept override;

        /**
         * @brief Returns the total number of readable channels (15).
         *
         * @details
         * The TMC9660 uses a non-contiguous channel ID scheme (0-3, 10-13, 20-21,
         * 30-31, 40-42). Since IDs are sparse, callers should use
         * IsChannelAvailable(channel_id) to validate specific channel IDs rather
         * than iterating `0..GetMaxChannels()-1`.
         *
         * @return 15 (the count of all valid channel IDs across all ranges).
         */
        hf_u8_t GetMaxChannels() const noexcept override;

        /**
         * @brief Check if a channel ID is valid per the TMC9660 channel scheme.
         * @param channel_id Channel ID to validate.
         * @return true if the channel ID is within any valid range.
         */
        bool IsChannelAvailable(hf_channel_id_t channel_id) const noexcept override;

        /**
         * @brief Read a channel and return the voltage.
         * @param channel_id          Channel ID per the scheme above.
         * @param[out] channel_reading_v Voltage reading (or physical value for temp/motor channels).
         * @param numOfSamplesToAvg   Number of samples to average (currently unused).
         * @param timeBetweenSamples  Delay between samples in ms (currently unused).
         * @return ADC_SUCCESS or an appropriate error code.
         */
        hf_adc_err_t ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                                 hf_u8_t numOfSamplesToAvg = 1,
                                 hf_time_t timeBetweenSamples = 0) noexcept override;

        /**
         * @brief Read a channel and return the raw count.
         * @param channel_id                Channel ID.
         * @param[out] channel_reading_count Raw ADC count or integer value.
         * @param numOfSamplesToAvg          Number of samples to average.
         * @param timeBetweenSamples         Delay between samples in ms.
         * @return ADC_SUCCESS or error code.
         */
        hf_adc_err_t ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                     hf_u8_t numOfSamplesToAvg = 1,
                                     hf_time_t timeBetweenSamples = 0) noexcept override;

        /**
         * @brief Read a channel and return both raw count and voltage.
         * @param channel_id                Channel ID.
         * @param[out] channel_reading_count Raw count.
         * @param[out] channel_reading_v     Voltage / physical value.
         * @param numOfSamplesToAvg          Number of samples to average.
         * @param timeBetweenSamples         Delay between samples in ms.
         * @return ADC_SUCCESS or error code.
         */
        hf_adc_err_t ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                               float& channel_reading_v, hf_u8_t numOfSamplesToAvg = 1,
                               hf_time_t timeBetweenSamples = 0) noexcept override;

        /**
         * @brief Read multiple channels in a batch.
         * @param channel_ids Array of channel IDs to read.
         * @param num_channels Number of channels.
         * @param[out] readings Array to store raw counts.
         * @param[out] voltages Array to store voltages.
         * @return ADC_SUCCESS if all channels read successfully.
         */
        hf_adc_err_t ReadMultipleChannels(const hf_channel_id_t* channel_ids, hf_u8_t num_channels,
                                         hf_u32_t* readings, float* voltages) noexcept override;

        /** @brief Get accumulated ADC statistics (conversion counts, timing). */
        hf_adc_err_t GetStatistics(hf_adc_statistics_t& statistics) noexcept override;

        /** @brief Get ADC health diagnostics (error counts, health status). */
        hf_adc_err_t GetDiagnostics(hf_adc_diagnostics_t& diagnostics) noexcept override;

        /** @brief Reset all accumulated statistics to zero. */
        hf_adc_err_t ResetStatistics() noexcept override;

        /** @brief Reset diagnostics counters and health flags. */
        hf_adc_err_t ResetDiagnostics() noexcept override;

        /// @}

        /// @name TMC9660-Specific Channel Methods
        /// @brief Direct accessors for specific TMC9660 ADC channel types.
        /// @{

        /**
         * @brief Read an external analog input (AIN) channel.
         * @param ain_channel AIN channel number (0-3).
         * @param[out] raw_value Raw 16-bit ADC count.
         * @param[out] voltage Converted voltage (3.3V / 65535 scale).
         * @return ADC_SUCCESS or ADC_ERR_CHANNEL_READ_ERR.
         */
        hf_adc_err_t ReadAinChannel(uint8_t ain_channel, hf_u32_t& raw_value, float& voltage) noexcept;

        /**
         * @brief Read a phase current sense channel.
         * @param current_channel Current channel number (0=I0, 1=I1, 2=I2, 3=I3).
         * @param[out] raw_value Raw ADC count.
         * @param[out] voltage Converted voltage.
         * @return ADC_SUCCESS or error code.
         */
        hf_adc_err_t ReadCurrentSenseChannel(uint8_t current_channel, hf_u32_t& raw_value, float& voltage) noexcept;

        /**
         * @brief Read a voltage monitoring channel.
         * @param voltage_channel 0=supply voltage, 1=driver voltage.
         * @param[out] raw_value Raw value (voltage * 1000 as integer mV).
         * @param[out] voltage Voltage in volts.
         * @return ADC_SUCCESS or ADC_ERR_INVALID_CHANNEL.
         */
        hf_adc_err_t ReadVoltageChannel(uint8_t voltage_channel, hf_u32_t& raw_value, float& voltage) noexcept;

        /**
         * @brief Read a temperature channel.
         * @param temp_channel 0=chip temperature (Celsius), 1=external NTC raw.
         * @param[out] raw_value Raw value (temp*100 for chip, raw ADC for external).
         * @param[out] voltage Temperature in Celsius (chip) or voltage (external).
         * @return ADC_SUCCESS or ADC_ERR_INVALID_CHANNEL.
         */
        hf_adc_err_t ReadTemperatureChannel(uint8_t temp_channel, hf_u32_t& raw_value, float& voltage) noexcept;

        /**
         * @brief Read a motor data channel.
         * @param motor_channel 0=current (mA), 1=velocity (internal), 2=position (counts).
         * @param[out] raw_value Raw integer value.
         * @param[out] voltage Float representation of the same value.
         * @return ADC_SUCCESS or ADC_ERR_INVALID_CHANNEL.
         */
        hf_adc_err_t ReadMotorDataChannel(uint8_t motor_channel, hf_u32_t& raw_value, float& voltage) noexcept;

        /// @}

    private:
        Tmc9660Handler& parent_;                   ///< Owning handler.
        mutable RtosMutex mutex_;                  ///< Thread-safety for statistics.
        mutable hf_adc_statistics_t statistics_;    ///< Accumulated conversion statistics.
        mutable hf_adc_diagnostics_t diagnostics_; ///< Health and error diagnostics.
        std::atomic<hf_adc_err_t> last_error_;     ///< Most recent error code.

        /** @brief Validate a channel ID against all valid ranges. */
        hf_adc_err_t ValidateChannelId(hf_channel_id_t channel_id) const noexcept;

        /**
         * @brief Internal channel read that returns both raw count and correct voltage.
         *
         * Dispatches to the appropriate sub-reader (AIN, current, voltage, temperature,
         * motor data) and returns both the raw count and the correctly-converted voltage.
         * Caller must hold mutex_.
         *
         * @param channel_id  Channel ID per the TMC9660 channel scheme.
         * @param[out] raw    Raw ADC count or integer value.
         * @param[out] voltage Correctly-converted voltage or physical value.
         * @return ADC_SUCCESS or error code.
         */
        hf_adc_err_t ReadChannelLocked(hf_channel_id_t channel_id,
                                       hf_u32_t& raw, float& voltage) noexcept;

        /** @brief Update conversion timing statistics. */
        hf_adc_err_t UpdateStatistics(hf_adc_err_t result, uint64_t start_time_us) noexcept;

        /** @brief Get current time in microseconds from RTOS tick counter. */
        uint64_t GetCurrentTimeUs() const noexcept;

        /** @brief Update error diagnostics (consecutive error tracking, health flag). */
        void UpdateDiagnostics(hf_adc_err_t error) noexcept;

        /** @brief Get a human-readable string for a channel type (for logging). */
        const char* GetChannelTypeString(hf_channel_id_t channel_id) const noexcept;
    };

    /**
     * @class Temperature
     * @brief BaseTemperature adapter for the TMC9660 internal chip temperature sensor.
     *
     * @details
     * Reads the TMC9660's built-in chip temperature via the telemetry subsystem and
     * presents it through the standard HardFOC BaseTemperature interface. Includes
     * thread-safe statistics and diagnostics tracking.
     *
     * Sensor specifications:
     * - Range: -40°C to +150°C
     * - Resolution: 0.1°C
     * - Accuracy: ±2°C typical
     * - Response time: ~100ms
     *
     * @note This inner class is owned by the handler. For manager-layer ownership,
     *       use Tmc9660TemperatureWrapper (defined in TemperatureManager.h).
     */
    class Temperature : public BaseTemperature {
    public:
        /**
         * @brief Construct the temperature wrapper.
         * @param parent Reference to the owning Tmc9660Handler.
         */
        Temperature(Tmc9660Handler& parent);

        /** @brief Default destructor. */
        ~Temperature() override = default;

        /// @name BaseTemperature Interface Implementation
        /// @{

        /** @brief Initialize (verifies parent driver is ready). */
        bool Initialize() noexcept override;

        /** @brief Deinitialize (no-op). */
        bool Deinitialize() noexcept override;

        /**
         * @brief Read the chip temperature in degrees Celsius.
         * @param[out] temperature_celsius Pointer to store the temperature reading.
         * @return TEMP_SUCCESS, TEMP_ERR_READ_FAILED, or TEMP_ERR_OUT_OF_RANGE.
         */
        hf_temp_err_t ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept override;

        /**
         * @brief Get sensor information (type, range, resolution, accuracy).
         * @param[out] info Pointer to store the sensor info struct.
         * @return TEMP_SUCCESS or TEMP_ERR_NULL_POINTER.
         */
        hf_temp_err_t GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept override;

        /**
         * @brief Get sensor capabilities flags.
         * @return Bitmask of HF_TEMP_CAP_HIGH_PRECISION | HF_TEMP_CAP_FAST_RESPONSE.
         */
        hf_u32_t GetCapabilities() const noexcept override;

        /// @}

    private:
        Tmc9660Handler& parent_;                    ///< Owning handler.
        mutable RtosMutex mutex_;                   ///< Thread-safety for statistics.
        mutable hf_temp_statistics_t statistics_;    ///< Accumulated operation statistics.
        mutable hf_temp_diagnostics_t diagnostics_; ///< Health and error diagnostics.
        std::atomic<hf_temp_err_t> last_error_;     ///< Most recent error code.

        /** @brief Update operation timing statistics. */
        hf_temp_err_t UpdateStatistics(hf_temp_err_t result, uint64_t start_time_us) noexcept;

        /** @brief Get current time in microseconds. */
        uint64_t GetCurrentTimeUs() const noexcept;

        /** @brief Update error diagnostics. */
        void UpdateDiagnostics(hf_temp_err_t error) noexcept;
    };

    /// @}

    //==========================================================================
    /// @name Peripheral Accessors
    /// @brief Retrieve references to the handler's internal peripheral wrappers.
    /// @{
    //==========================================================================

    /**
     * @brief Get a reference to the GPIO wrapper for a TMC9660 internal GPIO pin.
     *
     * @param gpioNumber TMC9660 internal GPIO pin number.
     *                   Currently supported: 17, 18.
     * @return Reference to the corresponding Gpio wrapper instance.
     *
     * @note If an unsupported GPIO number is given, falls back to GPIO17's wrapper.
     * @warning The returned reference is valid only as long as this handler exists.
     */
    Gpio& gpio(uint8_t gpioNumber);

    /**
     * @brief Get a reference to the ADC wrapper.
     * @return Reference to the internal Adc instance.
     * @warning The returned reference is valid only as long as this handler exists.
     */
    Adc& adc();

    /**
     * @brief Get a reference to the Temperature wrapper.
     * @return Reference to the internal Temperature instance.
     * @warning The returned reference is valid only as long as this handler exists.
     */
    Temperature& temperature();

    /// @}

    //==========================================================================
    /// @name Communication Info
    /// @{
    //==========================================================================

    /**
     * @brief Get the communication mode in use (SPI or UART).
     * @return tmc9660::CommMode::SPI or tmc9660::CommMode::UART.
     */
    tmc9660::CommMode GetCommMode() const noexcept;

    /**
     * @brief Get the bootloader configuration in use.
     * @return Const reference to the BootloaderConfig struct.
     */
    const tmc9660::BootloaderConfig& bootConfig() const noexcept { return *bootCfg_; }

    /// @}

    //==========================================================================
    /// @name Diagnostics
    /// @{
    //==========================================================================

    /**
     * @brief Dump comprehensive diagnostics to the system log.
     *
     * Logs device status, communication mode, supply voltage, chip temperature,
     * status/error flags, GPIO wrapper count, ADC status, and bootloader config
     * summary at INFO level.
     *
     * @note Non-const because telemetry reads require I2C transactions.
     */
    void DumpDiagnostics() noexcept;

    /// @}

    //==========================================================================
    /// @name Typed Driver Access
    /// @brief Direct access to the underlying typed TMC9660 driver instance.
    /// @{
    //==========================================================================

    /**
     * @brief Get the SPI-mode driver instance (nullptr if constructed with UART).
     *
     * @details
     * Returns a direct pointer to the typed TMC9660<HalSpiTmc9660Comm> driver,
     * giving full access to all 22+ subsystems (motorConfig, feedbackSense,
     * velocityControl, protection, gateDriver, etc.) without lambda wrappers.
     *
     * The user always knows which comm mode they chose at construction time,
     * so they can safely call the matching accessor.
     *
     * @return Pointer to the SPI driver, or nullptr if using UART mode or
     *         Initialize() has not been called.
     *
     * @code
     * // Direct subsystem access after Initialize()
     * auto* drv = handler->spiDriver();
     * drv->feedbackSense.configureHall();
     * drv->motorConfig.setType(tmcl::MotorType::BLDC_MOTOR, 7);
     * drv->velocityControl.setTargetVelocity(1000);
     * drv->protection.configureVoltage(48000, 10000);
     * @endcode
     */
    SpiDriver* spiDriver() noexcept { return spi_driver_.get(); }

    /** @brief Const version of spiDriver(). */
    const SpiDriver* spiDriver() const noexcept { return spi_driver_.get(); }

    /**
     * @brief Get the UART-mode driver instance (nullptr if constructed with SPI).
     *
     * @return Pointer to the UART driver, or nullptr if using SPI mode or
     *         Initialize() has not been called.
     *
     * @code
     * auto* drv = handler->uartDriver();
     * drv->feedbackSense.configureABNEncoder(1024);
     * @endcode
     */
    UartDriver* uartDriver() noexcept { return uart_driver_.get(); }

    /** @brief Const version of uartDriver(). */
    const UartDriver* uartDriver() const noexcept { return uart_driver_.get(); }

    /// @}

    //==========================================================================
    /// @name Driver Visitor Pattern
    /// @brief Generic access when comm mode is not known at the call site.
    /// @{
    //==========================================================================

    /**
     * @brief Execute a function on the underlying typed TMC9660 driver.
     *
     * @details
     * For most use cases, prefer spiDriver() or uartDriver() for direct typed
     * access. Use visitDriver() only when writing generic code that must work
     * with either communication mode (e.g., in manager-level code that doesn't
     * know the comm type).
     *
     * @tparam Func Callable type with signature `ReturnType(auto& driver)`.
     * @param func The visitor function to execute.
     * @return The return value of @p func, or a default-constructed value if the
     *         driver is not ready.
     */
    template <typename Func>
    auto visitDriver(Func&& func) {
        if (use_spi_) {
            if (spi_driver_) return func(*spi_driver_);
        } else {
            if (uart_driver_) return func(*uart_driver_);
        }
        using ReturnType = decltype(func(*spi_driver_));
        if constexpr (std::is_void_v<ReturnType>) {
            return;
        } else {
            return ReturnType{};
        }
    }

    /** @brief Const overload of visitDriver(). */
    template <typename Func>
    auto visitDriver(Func&& func) const {
        if (use_spi_) {
            if (spi_driver_) return func(*spi_driver_);
        } else {
            if (uart_driver_) return func(*uart_driver_);
        }
        using ReturnType = decltype(func(std::as_const(*spi_driver_)));
        if constexpr (std::is_void_v<ReturnType>) {
            return;
        } else {
            return ReturnType{};
        }
    }

    /// @}

private:
    //==========================================================================
    // Private Members
    //==========================================================================

    bool use_spi_;  ///< true = SPI communication active, false = UART.

    /// @name Communication Adapters
    /// @brief One of these is created at construction time; the other remains null.
    /// @{
    std::unique_ptr<HalSpiTmc9660Comm>  spi_comm_;   ///< SPI communication adapter (or nullptr).
    std::unique_ptr<HalUartTmc9660Comm> uart_comm_;   ///< UART communication adapter (or nullptr).
    /// @}

    /// @name Typed Driver Instances
    /// @brief Created during Initialize(); one is active, the other remains null.
    /// @{
    std::unique_ptr<SpiDriver>  spi_driver_;   ///< TMC9660 driver for SPI mode (or nullptr).
    std::unique_ptr<UartDriver> uart_driver_;   ///< TMC9660 driver for UART mode (or nullptr).
    /// @}

    /// @name Peripheral Wrappers
    /// @brief Created during Initialize().
    /// @{
    std::array<std::unique_ptr<Gpio>, 2> gpioWrappers_;   ///< GPIO17 [0] and GPIO18 [1].
    std::unique_ptr<Adc>         adcWrapper_;              ///< Multi-channel ADC wrapper.
    std::unique_ptr<Temperature> temperatureWrapper_;      ///< Chip temperature wrapper.
    /// @}

    /// @name Configuration
    /// @{
    const tmc9660::BootloaderConfig* bootCfg_;   ///< Bootloader config (not owned; must outlive handler).
    uint8_t device_address_;                      ///< 7-bit TMCL device address.
    /// @}
};

/// @} // end of TMC9660_HAL_Handler

#endif // COMPONENT_HANDLER_TMC9660_HANDLER_H_
