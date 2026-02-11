/**
 * @file Pcal95555Handler.h
 * @brief Unified handler for the PCA9555 / PCAL9555A 16-bit I2C GPIO expander.
 *
 * @details
 * This file provides the complete HAL-level integration for a single PCA9555 /
 * PCAL9555A GPIO expander device. It bridges the HardFOC base interfaces
 * (BaseI2c, BaseGpio) with the templated pcal95555::PCAL95555<I2cType> driver
 * from the hf-pcal95555-driver library.
 *
 * ## Architecture Overview
 *
 * The file contains three layers:
 *
 * 1. **CRTP I2C Communication Adapter** (HalI2cPcal95555Comm):
 *    Bridges the HardFOC BaseI2c device-centric interface (where the I2C address
 *    is pre-configured on the device object) with the PCAL95555 driver's
 *    CRTP-based pcal95555::I2cInterface (which passes the address as a parameter).
 *    Includes address validation, thread-safe I2C operations, and optional
 *    interrupt handler registration.
 *
 * 2. **Pcal95555Handler** (main class):
 *    Non-templated facade that owns a typed driver instance. Provides:
 *    - Lazy initialization with automatic chip variant detection
 *    - Per-pin GPIO operations (direction, read, write, toggle, pull mode)
 *    - Batch pin operations via 16-bit masks
 *    - Interrupt management (hardware INT pin + per-pin callbacks)
 *    - PCAL9555A "Agile I/O" features (drive strength, input latch, output mode)
 *    - Error flag access and diagnostics
 *    - Pin factory for creating BaseGpio-compatible wrappers
 *
 * 3. **Pcal95555GpioPin** (per-pin wrapper):
 *    Implements BaseGpio for a single expander pin, delegating all operations
 *    to the parent handler. Supports direction, level, pull mode, output mode,
 *    polarity inversion, and interrupt configuration.
 *
 * ## Ownership Model
 *
 * The handler owns all its internal resources:
 * - One HalI2cPcal95555Comm (I2C communication adapter)
 * - One typed pcal95555::PCAL95555<HalI2cPcal95555Comm> driver instance
 * - Up to 16 Pcal95555GpioPin wrappers (created on demand via pin registry)
 *
 * External managers (GpioManager) obtain shared_ptr<BaseGpio> references to
 * pins via CreateGpioPin(). The handler and pin registry co-own the pin objects.
 *
 * ## Initialization Sequence
 *
 * @code
 * // 1. Obtain a BaseI2c device reference (address pre-configured)
 * // 2. Construct the handler (optionally with an interrupt pin)
 * Pcal95555Handler handler(i2c_device, &interrupt_gpio);
 *
 * // 3. Initialize (lazy -- happens automatically on first use, or explicitly)
 * if (!handler.EnsureInitialized()) { return; }
 *
 * // 4. Create pin wrappers and use them
 * auto pin0 = handler.CreateGpioPin(0, HF_GPIO_DIRECTION_OUTPUT);
 * pin0->SetPinLevel(HF_GPIO_LEVEL_HIGH);
 *
 * // 5. Use handler-level operations
 * handler.SetDirection(5, HF_GPIO_DIRECTION_INPUT);
 * handler.SetPullMode(5, HF_GPIO_PULL_MODE_UP);
 * @endcode
 *
 * ## Thread Safety
 *
 * The handler uses RtosMutex for thread-safe access to:
 * - I2C communication (in the adapter's i2c_mutex_)
 * - All handler-level operations including pin registry and interrupts (handler_mutex_)
 *
 * A single handler_mutex_ protects all state because all hardware access is
 * serialized through I2C anyway, making finer-grained locking unnecessary overhead.
 *
 * @see GpioManager          Manager that creates and owns handler + pins
 * @see pcal95555::PCAL95555 Templated driver from hf-pcal95555-driver
 *
 * @author HardFOC Team
 * @date 2025
 */

#ifndef COMPONENT_HANDLER_PCAL95555_HANDLER_H_
#define COMPONENT_HANDLER_PCAL95555_HANDLER_H_

#include <cstdint>
#include <memory>
#include <array>
#include <functional>
#include "base/BaseGpio.h"
#include "base/BaseI2c.h"
#include "core/hf-core-drivers/external/hf-pcal95555-driver/inc/pcal95555.hpp"
#include "utils/RtosMutex.h"

// Forward declarations
class Pcal95555Handler;

///////////////////////////////////////////////////////////////////////////////
/// @defgroup PCAL95555_HAL_I2CAdapter HAL I2C Communication Adapter
/// @brief CRTP I2C adapter bridging BaseI2c to the PCAL95555 driver.
/// @{
///////////////////////////////////////////////////////////////////////////////

/**
 * @class HalI2cPcal95555Comm
 * @brief Concrete I2C communication adapter for PCAL95555 using BaseI2c.
 *
 * @details
 * Implements all methods required by pcal95555::I2cInterface<HalI2cPcal95555Comm>
 * through the CRTP pattern. This class bridges the HardFOC BaseI2c device interface
 * (where the I2C address is pre-configured on the device) with the PCAL95555 driver's
 * I2C interface (which passes the address as a parameter).
 *
 * Key behaviors:
 * - **Address validation**: Every write/read validates that the driver's address
 *   matches the BaseI2c device's configured address, preventing cross-device errors.
 * - **Register framing**: Writes are framed as [register, data...] per I2C convention.
 * - **Thread safety**: All I2C operations are mutex-protected.
 * - **Lazy initialization**: EnsureInitialized() delegates to BaseI2c.
 *
 * @note This class does NOT own the BaseI2c device; it must remain valid for
 *       the lifetime of this adapter.
 *
 * @see pcal95555::I2cInterface  CRTP base class from the PCAL95555 driver
 */
class HalI2cPcal95555Comm : public pcal95555::I2cInterface<HalI2cPcal95555Comm> {
public:
    /**
     * @brief Construct the I2C communication adapter.
     * @param i2c_device Reference to a BaseI2c device with pre-configured address.
     * @warning The BaseI2c device must outlive this adapter.
     */
    explicit HalI2cPcal95555Comm(BaseI2c& i2c_device) noexcept;

    /// @name CRTP-Required Methods
    /// Called by pcal95555::I2cInterface<HalI2cPcal95555Comm> via static dispatch.
    /// @{

    /**
     * @brief Write data to a device register.
     *
     * Frames the write as [register_address, data...] and sends via BaseI2c::Write().
     * The addr parameter is validated against the device's configured address.
     *
     * @param addr 7-bit I2C address (validated against BaseI2c device).
     * @param reg  Register address to write to.
     * @param data Pointer to data buffer.
     * @param len  Number of bytes to write.
     * @return true if write succeeded, false on address mismatch or I2C error.
     */
    bool write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) noexcept;

    /**
     * @brief Read data from a device register.
     *
     * Sends the register address via BaseI2c::WriteRead() and reads back data.
     *
     * @param addr 7-bit I2C address (validated against BaseI2c device).
     * @param reg  Register address to read from.
     * @param data Buffer to store read data.
     * @param len  Number of bytes to read.
     * @return true if read succeeded, false on address mismatch or I2C error.
     */
    bool read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) noexcept;

    /**
     * @brief Ensure the I2C bus is initialized and ready.
     * @return true if the BaseI2c device is initialized or was initialized successfully.
     */
    bool EnsureInitialized() noexcept;

    /**
     * @brief Register an interrupt handler to be called when the INT pin fires.
     *
     * The PCAL95555 driver calls this to register its HandleInterrupt() method.
     * The handler stores the function and can be triggered externally.
     *
     * @param handler Function to call when interrupt occurs.
     * @return true if handler was stored (always returns true; actual GPIO setup
     *         is done by Pcal95555Handler::ConfigureHardwareInterrupt).
     */
    bool RegisterInterruptHandler(std::function<void()> handler) noexcept;

    /// @}

    /**
     * @brief Get the stored interrupt handler (if any).
     * @return Reference to the stored interrupt handler function.
     */
    const std::function<void()>& GetInterruptHandler() const noexcept { return interrupt_handler_; }

private:
    BaseI2c& i2c_device_;                  ///< I2C device interface (not owned).
    mutable RtosMutex i2c_mutex_;          ///< Thread safety for I2C operations.
    std::function<void()> interrupt_handler_; ///< Stored interrupt handler from driver.
};

/// @} // end of PCAL95555_HAL_I2CAdapter

///////////////////////////////////////////////////////////////////////////////
/// @defgroup PCAL95555_HAL_GpioPin Per-Pin GPIO Wrapper
/// @brief BaseGpio adapter for individual PCAL95555 expander pins.
/// @{
///////////////////////////////////////////////////////////////////////////////

/**
 * @class Pcal95555GpioPin
 * @brief BaseGpio adapter for a single PCAL95555 GPIO expander pin.
 *
 * @details
 * Wraps one of the 16 GPIO expander pins (0-15) as a HardFOC BaseGpio instance.
 * All hardware operations delegate to the parent Pcal95555Handler, which in turn
 * calls the typed PCAL95555 driver.
 *
 * Features:
 * - Direction (input/output)
 * - Level read/write
 * - Pull mode (floating, pull-up, pull-down) -- PCAL9555A only
 * - Output mode (push-pull / open-drain) -- PCAL9555A only, per-port granularity
 * - Polarity inversion
 * - Interrupt configuration (delegates to handler for centralized management)
 *
 * @note Pin instances are created via Pcal95555Handler::CreateGpioPin() and
 *       stored in the handler's pin registry as shared_ptr.
 */
class Pcal95555GpioPin : public BaseGpio {
public:
    /**
     * @brief Construct a PCAL95555 GPIO pin wrapper.
     * @param pin            Pin number (0-15).
     * @param parent_handler Pointer to the owning handler (must not be null).
     * @param direction      Initial direction (default: input).
     * @param active_state   Active polarity (default: active high).
     * @param output_mode    Output mode (default: push-pull).
     * @param pull_mode      Pull resistor mode (default: floating).
     */
    Pcal95555GpioPin(hf_pin_num_t pin,
                     Pcal95555Handler* parent_handler,
                     hf_gpio_direction_t direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
                     hf_gpio_active_state_t active_state = hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
                     hf_gpio_output_mode_t output_mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
                     hf_gpio_pull_mode_t pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING) noexcept;

    /** @brief Default destructor. */
    ~Pcal95555GpioPin() override = default;

    /// @name BaseGpio Interface Implementation
    /// @{

    /** @brief Initialize the pin (configures direction and pull mode on the expander). */
    bool Initialize() noexcept override;

    /** @brief Deinitialize (marks pin as uninitialized; expander state persists). */
    bool Deinitialize() noexcept override;

    /** @brief Check if this pin number is valid (0-15). */
    bool IsPinAvailable() const noexcept override;

    /** @brief Returns 16 (total pins on the expander). */
    hf_u8_t GetMaxPins() const noexcept override { return 16; }

    /** @brief Returns a description string like "PCAL95555_PIN_5". */
    const char* GetDescription() const noexcept override;

    /** @brief Returns true (PCAL9555A supports interrupts). */
    bool SupportsInterrupts() const noexcept override { return true; }

    /**
     * @brief Configure interrupt for this pin.
     *
     * Delegates to the parent handler's RegisterPinInterrupt() for centralized
     * interrupt management.
     *
     * @param trigger  Trigger type (rising, falling, both, none).
     * @param callback Function to call when interrupt fires.
     * @param user_data User data passed to callback.
     * @return GPIO_SUCCESS or error code.
     */
    hf_gpio_err_t ConfigureInterrupt(hf_gpio_interrupt_trigger_t trigger,
                                     InterruptCallback callback = nullptr,
                                     void* user_data = nullptr) noexcept override;

    /// @}

    /// @name PCAL9555A Advanced Features
    /// @brief These features require PCAL9555A. On PCA9555, they return failure.
    /// @{

    /**
     * @brief Set input polarity inversion for this pin.
     * @param invert true to invert, false for normal polarity.
     * @return GPIO_SUCCESS or GPIO_ERR_FAILURE.
     */
    hf_gpio_err_t SetPolarityInversion(bool invert) noexcept;

    /**
     * @brief Set the interrupt mask for this pin.
     * @param mask true to mask (disable) interrupt, false to unmask (enable).
     * @return GPIO_SUCCESS or GPIO_ERR_FAILURE.
     */
    hf_gpio_err_t SetInterruptMask(bool mask) noexcept;

    /**
     * @brief Get the interrupt status for this pin.
     * @param[out] status true if this pin has a pending interrupt.
     * @return GPIO_SUCCESS or GPIO_ERR_FAILURE.
     */
    hf_gpio_err_t GetInterruptStatus(bool& status) noexcept;

    /// @}

protected:
    /// @name BaseGpio Protected Implementation
    /// @{

    /** @brief Set pin direction via the PCAL95555 driver. */
    hf_gpio_err_t SetDirectionImpl(hf_gpio_direction_t direction) noexcept override;

    /** @brief Returns GPIO_ERR_UNSUPPORTED_OPERATION (use Pcal95555Handler::SetOutputMode for per-port control). */
    hf_gpio_err_t SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept override;

    /** @brief Set pull mode (floating/up/down). PCAL9555A only. */
    hf_gpio_err_t SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept override;

    /** @brief Get current pull mode (tracked internally). */
    hf_gpio_pull_mode_t GetPullModeImpl() const noexcept override;

    /** @brief Write pin level via the PCAL95555 driver. */
    hf_gpio_err_t SetPinLevelImpl(hf_gpio_level_t level) noexcept override;

    /** @brief Read pin level via the PCAL95555 driver. */
    hf_gpio_err_t GetPinLevelImpl(hf_gpio_level_t& level) noexcept override;

    /** @brief Get pin direction from the PCAL95555 driver. */
    hf_gpio_err_t GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept override;

    /** @brief Get output mode (tracked internally). */
    hf_gpio_err_t GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept override;

    /// @}

    /// Allow handler to access pin interrupt data directly.
    friend class Pcal95555Handler;

private:
    hf_pin_num_t pin_;                     ///< Pin number (0-15).
    Pcal95555Handler* parent_handler_;     ///< Owning handler (not owned).
    char description_[32] = {};            ///< Human-readable description.

    /// @name Per-Pin Interrupt State
    /// @{
    InterruptCallback interrupt_callback_ = nullptr;
    void* interrupt_user_data_ = nullptr;
    hf_gpio_interrupt_trigger_t interrupt_trigger_ =
        hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_NONE;
    bool interrupt_enabled_ = false;
    /// @}
};

/// @} // end of PCAL95555_HAL_GpioPin

///////////////////////////////////////////////////////////////////////////////
/// @defgroup PCAL95555_HAL_Handler PCAL95555 Handler
/// @brief Non-templated facade for PCA9555 / PCAL9555A GPIO expander integration.
/// @{
///////////////////////////////////////////////////////////////////////////////

/**
 * @class Pcal95555Handler
 * @brief Unified, non-templated handler for a single PCA9555 / PCAL9555A device.
 *
 * @details
 * Pcal95555Handler is the primary interface that application and manager code uses
 * to interact with a PCAL95555 GPIO expander. It hides the template complexity of
 * pcal95555::PCAL95555<I2cType> behind a clean public API.
 *
 * ## Design Decisions
 *
 * - **Non-templated**: The handler owns a concrete typed driver instance
 *   (pcal95555::PCAL95555<HalI2cPcal95555Comm>) internally. Since only one I2C
 *   communication type is used per handler, no dispatch mechanism is needed
 *   (unlike the TMC9660Handler which supports SPI or UART).
 *
 * - **Lazy initialization**: The driver and adapter are created in Initialize(),
 *   not in the constructor. The driver itself also uses lazy initialization
 *   (auto-detecting chip variant on first I2C access).
 *
 * - **Pin factory pattern**: Pins are created on demand via CreateGpioPin() and
 *   stored in a shared_ptr registry. Multiple calls for the same pin return the
 *   same instance.
 *
 * - **Chip variant awareness**: After initialization, HasAgileIO() reports
 *   whether PCAL9555A features (pull resistors, drive strength, interrupts,
 *   input latch, output mode) are available. Methods requiring Agile I/O
 *   gracefully fail on standard PCA9555.
 *
 * @see HalI2cPcal95555Comm  I2C communication adapter
 * @see Pcal95555GpioPin     Per-pin BaseGpio wrapper
 * @see GpioManager          Manager that creates and owns handler instances
 */
class Pcal95555Handler {
public:
    //==========================================================================
    /// @name Type Aliases
    /// @{
    //==========================================================================

    /** @brief Typed PCAL95555 driver using our I2C adapter. */
    using Pcal95555Driver = pcal95555::PCAL95555<HalI2cPcal95555Comm>;

    /// @}

    //==========================================================================
    /// @name Construction and Destruction
    /// @{
    //==========================================================================

    /**
     * @brief Construct a handler for a PCAL95555 device.
     *
     * The I2C adapter and driver are not created until EnsureInitialized() or
     * the first operation that requires them.
     *
     * @param i2c_device    Reference to a BaseI2c device with address pre-configured.
     *                      Must outlive the handler.
     * @param interrupt_pin Optional BaseGpio pin connected to the expander's INT output.
     *                      Pass nullptr for polling mode (no hardware interrupts).
     */
    explicit Pcal95555Handler(BaseI2c& i2c_device, BaseGpio* interrupt_pin = nullptr) noexcept;

    /** @brief Destructor. Releases driver, adapter, and all pin wrappers. */
    ~Pcal95555Handler() = default;

    /// Non-copyable.
    Pcal95555Handler(const Pcal95555Handler&) = delete;
    /// Non-copyable.
    Pcal95555Handler& operator=(const Pcal95555Handler&) = delete;

    /// @}

    //==========================================================================
    /// @name Initialization and Lifecycle
    /// @{
    //==========================================================================

    /**
     * @brief Ensure the handler is initialized (lazy initialization).
     *
     * On first call, creates the I2C adapter and driver, runs the driver's
     * EnsureInitialized() (which auto-detects chip variant), and optionally
     * configures the hardware interrupt pin.
     *
     * @return true if already initialized or initialization succeeded.
     */
    bool EnsureInitialized() noexcept;

    /**
     * @brief Deinitialize the handler and release resources.
     *
     * Disables hardware interrupt, clears the pin registry, and releases
     * the driver and I2C adapter.
     *
     * @return true if already deinitialized or deinitialization succeeded.
     */
    bool EnsureDeinitialized() noexcept;

    /**
     * @brief Check if the handler has been initialized.
     * @return true if initialized, false otherwise.
     */
    bool IsInitialized() const noexcept { return initialized_; }

    /// @}

    //==========================================================================
    /// @name Basic GPIO Operations
    /// @brief Per-pin operations using pin numbers (0-15).
    /// @{
    //==========================================================================

    /**
     * @brief Set the direction of a single pin.
     * @param pin       Pin number (0-15).
     * @param direction Input or output.
     * @return GPIO_SUCCESS or error code.
     */
    hf_gpio_err_t SetDirection(uint8_t pin, hf_gpio_direction_t direction) noexcept;

    /**
     * @brief Write a logical level to an output pin.
     * @param pin    Pin number (0-15).
     * @param active true for HIGH, false for LOW.
     * @return GPIO_SUCCESS or error code.
     */
    hf_gpio_err_t SetOutput(uint8_t pin, bool active) noexcept;

    /**
     * @brief Read the logical level of an input pin.
     * @param pin         Pin number (0-15).
     * @param[out] active true if HIGH, false if LOW.
     * @return GPIO_SUCCESS or error code (checks driver error flags).
     */
    hf_gpio_err_t ReadInput(uint8_t pin, bool& active) noexcept;

    /**
     * @brief Toggle the output state of a pin.
     * @param pin Pin number (0-15).
     * @return GPIO_SUCCESS or error code.
     */
    hf_gpio_err_t Toggle(uint8_t pin) noexcept;

    /**
     * @brief Set the pull resistor mode for a single pin.
     *
     * Maps HardFOC pull modes to PCAL9555A SetPullEnable/SetPullDirection:
     * - FLOATING: disable pull
     * - UP: enable pull, direction = up
     * - DOWN: enable pull, direction = down
     * - UP_DOWN: not supported, defaults to pull-up
     *
     * @param pin       Pin number (0-15).
     * @param pull_mode Desired pull mode.
     * @return GPIO_SUCCESS or error code.
     * @note Requires PCAL9555A. Returns GPIO_ERR_UNSUPPORTED_OPERATION on PCA9555.
     */
    hf_gpio_err_t SetPullMode(uint8_t pin, hf_gpio_pull_mode_t pull_mode) noexcept;

    /**
     * @brief Get the pull resistor mode for a single pin (tracked internally).
     * @param pin            Pin number (0-15).
     * @param[out] pull_mode Current pull mode.
     * @return GPIO_SUCCESS or error code.
     */
    hf_gpio_err_t GetPullMode(uint8_t pin, hf_gpio_pull_mode_t& pull_mode) noexcept;

    /// @}

    //==========================================================================
    /// @name Batch GPIO Operations
    /// @brief Operations on multiple pins using 16-bit masks.
    /// @{
    //==========================================================================

    /**
     * @brief Set direction for multiple pins at once.
     * @param pin_mask  16-bit mask (bit N = pin N).
     * @param direction Common direction for all selected pins.
     * @return GPIO_SUCCESS or error code.
     */
    hf_gpio_err_t SetDirections(uint16_t pin_mask, hf_gpio_direction_t direction) noexcept;

    /**
     * @brief Write output level for multiple pins at once.
     * @param pin_mask 16-bit mask (bit N = pin N).
     * @param active   Common level for all selected pins.
     * @return GPIO_SUCCESS or error code.
     */
    hf_gpio_err_t SetOutputs(uint16_t pin_mask, bool active) noexcept;

    /**
     * @brief Set pull mode for multiple pins at once.
     * @param pin_mask  16-bit mask (bit N = pin N).
     * @param pull_mode Common pull mode for all selected pins.
     * @return GPIO_SUCCESS or error code.
     * @note Requires PCAL9555A.
     */
    hf_gpio_err_t SetPullModes(uint16_t pin_mask, hf_gpio_pull_mode_t pull_mode) noexcept;

    /// @}

    //==========================================================================
    /// @name Interrupt Management
    /// @{
    //==========================================================================

    /**
     * @brief Check if hardware interrupt support is available.
     * @return true if an interrupt pin was provided at construction.
     */
    bool HasInterruptSupport() const noexcept { return interrupt_pin_ != nullptr; }

    /**
     * @brief Check if hardware interrupt is configured and enabled.
     * @return true if interrupt pin is actively monitoring.
     */
    bool IsInterruptConfigured() const noexcept { return interrupt_configured_; }

    /**
     * @brief Get the interrupt mask for all 16 pins.
     * @param[out] mask 16-bit mask (0=enabled, 1=masked per PCAL9555A convention).
     * @return GPIO_SUCCESS or error code.
     * @note Requires PCAL9555A.
     */
    hf_gpio_err_t GetAllInterruptMasks(uint16_t& mask) noexcept;

    /**
     * @brief Get the interrupt status for all 16 pins.
     *
     * Reading the status clears the interrupt condition on the device.
     *
     * @param[out] status 16-bit mask (bit N set = pin N has pending interrupt).
     * @return GPIO_SUCCESS or error code.
     * @note Requires PCAL9555A.
     */
    hf_gpio_err_t GetAllInterruptStatus(uint16_t& status) noexcept;

    /// @}

    //==========================================================================
    /// @name Pin Factory
    /// @brief Create and manage BaseGpio-compatible pin wrapper instances.
    /// @{
    //==========================================================================

    /** @brief Total number of GPIO pins on the expander. */
    static constexpr uint8_t PinCount() noexcept { return 16; }

    /**
     * @brief Get the I2C address of the underlying device.
     * @return 7-bit I2C address (0x20-0x27), or 0 if driver not created.
     */
    uint8_t GetI2cAddress() const noexcept;

    /**
     * @brief Create or retrieve an existing BaseGpio pin wrapper.
     *
     * If a wrapper for the given pin already exists in the registry and
     * allow_existing is true, the existing instance is returned. Otherwise
     * a new Pcal95555GpioPin is created, initialized, and registered.
     *
     * @param pin            Pin number (0-15).
     * @param direction      Initial direction (ignored if pin exists).
     * @param active_state   Active polarity (ignored if pin exists).
     * @param output_mode    Output mode (ignored if pin exists).
     * @param pull_mode      Pull resistor mode (ignored if pin exists).
     * @param allow_existing If true, returns existing pin; if false, fails if exists.
     * @return shared_ptr<BaseGpio> or nullptr on failure.
     */
    std::shared_ptr<BaseGpio> CreateGpioPin(
        hf_pin_num_t pin,
        hf_gpio_direction_t direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
        hf_gpio_active_state_t active_state = hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
        hf_gpio_output_mode_t output_mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
        hf_gpio_pull_mode_t pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING,
        bool allow_existing = true) noexcept;

    /**
     * @brief Get an existing GPIO pin wrapper by number.
     * @param pin Pin number (0-15).
     * @return shared_ptr<BaseGpio> or nullptr if pin not created.
     */
    std::shared_ptr<BaseGpio> GetGpioPin(hf_pin_num_t pin) noexcept;

    /**
     * @brief Check if a pin wrapper has been created.
     * @param pin Pin number (0-15).
     * @return true if the pin exists in the registry.
     */
    bool IsPinCreated(hf_pin_num_t pin) const noexcept;

    /**
     * @brief Get a bitmask of all created pin numbers.
     * @return 16-bit mask where bit N is set if pin N has been created.
     */
    uint16_t GetCreatedPinMask() const noexcept;

    /// @}

    //==========================================================================
    /// @name PCAL9555A Advanced Features (Agile I/O)
    /// @brief These features require a PCAL9555A chip. They return false on PCA9555.
    /// @{
    //==========================================================================

    /**
     * @brief Check if the detected chip supports Agile I/O features.
     * @return true if PCAL9555A detected, false if PCA9555 or not yet initialized.
     * @note Call after EnsureInitialized() for accurate results.
     */
    bool HasAgileIO() const noexcept;

    /**
     * @brief Get the detected chip variant.
     * @return ChipVariant enum (Unknown, PCA9555, or PCAL9555A).
     */
    pcal95555::ChipVariant GetChipVariant() const noexcept;

    /**
     * @brief Set input polarity inversion for a pin.
     * @param pin    Pin number (0-15).
     * @param invert true to invert input polarity, false for normal.
     * @return true on success, false on failure.
     */
    bool SetPolarityInversion(hf_pin_num_t pin, bool invert) noexcept;

    /**
     * @brief Set the per-pin interrupt mask.
     * @param pin  Pin number (0-15).
     * @param mask true to mask (disable) interrupt, false to unmask (enable).
     * @return true on success, false on failure.
     * @note Requires PCAL9555A.
     */
    bool SetInterruptMask(hf_pin_num_t pin, bool mask) noexcept;

    /**
     * @brief Get interrupt status for a single pin.
     * @param pin          Pin number (0-15).
     * @param[out] status  true if pin has pending interrupt.
     * @return true on success, false on failure.
     * @note Reading status clears the interrupt. Requires PCAL9555A.
     */
    bool GetInterruptStatus(hf_pin_num_t pin, bool& status) noexcept;

    /**
     * @brief Set output drive strength for a pin.
     * @param pin   Pin number (0-15).
     * @param level Drive strength (Level0=25%, Level1=50%, Level2=75%, Level3=100%).
     * @return true on success, false on failure.
     * @note Requires PCAL9555A.
     */
    bool SetDriveStrength(hf_pin_num_t pin, DriveStrength level) noexcept;

    /**
     * @brief Enable or disable input latch for a pin.
     * @param pin    Pin number (0-15).
     * @param enable true to enable input latch, false to disable.
     * @return true on success, false on failure.
     * @note Requires PCAL9555A.
     */
    bool EnableInputLatch(hf_pin_num_t pin, bool enable) noexcept;

    /**
     * @brief Configure per-port output mode (push-pull or open-drain).
     * @param port0_open_drain true for open-drain on port 0 (pins 0-7).
     * @param port1_open_drain true for open-drain on port 1 (pins 8-15).
     * @return true on success, false on failure.
     * @note Requires PCAL9555A.
     */
    bool SetOutputMode(bool port0_open_drain, bool port1_open_drain) noexcept;

    /**
     * @brief Reset all registers to power-on default state.
     * @return true on success, false on failure.
     */
    bool ResetToDefault() noexcept;

    /// @}

    //==========================================================================
    /// @name Error Management
    /// @{
    //==========================================================================

    /**
     * @brief Get the driver's error flags.
     * @return 16-bit error flag bitmask (see pcal95555 Error enum).
     */
    uint16_t GetErrorFlags() const noexcept;

    /**
     * @brief Clear driver error flags.
     * @param mask Bitmask of flags to clear (default: all).
     */
    void ClearErrorFlags(uint16_t mask = 0xFFFF) noexcept;

    /// @}

    //==========================================================================
    /// @name Diagnostics
    /// @{
    //==========================================================================

    /**
     * @brief Dump comprehensive diagnostics to the system log.
     *
     * Logs initialization status, I2C address, chip variant, pin registry,
     * interrupt configuration, and error flags at INFO level.
     */
    void DumpDiagnostics() const noexcept;

    /// @}

    /// Allow Pcal95555GpioPin to access private interrupt methods.
    friend class Pcal95555GpioPin;

private:
    //==========================================================================
    // Private Methods
    //==========================================================================

    /**
     * @brief Check init state under already-held handler_mutex_.
     * @return true if initialized or initialization succeeds.
     */
    bool EnsureInitializedLocked() noexcept;

    /**
     * @brief Internal initialization (called by EnsureInitialized under mutex).
     * @return GPIO error code.
     */
    hf_gpio_err_t Initialize() noexcept;

    /**
     * @brief Internal deinitialization.
     * @return GPIO error code.
     */
    hf_gpio_err_t Deinitialize() noexcept;

    /** @brief Validate pin number is in range 0-15. */
    bool ValidatePin(uint8_t pin) const noexcept { return pin < 16; }

    /**
     * @brief Register a per-pin interrupt callback (called by Pcal95555GpioPin).
     * @param pin      Pin number (0-15).
     * @param trigger  Trigger type.
     * @param callback Callback function.
     * @param user_data User data for callback.
     * @return GPIO error code.
     */
    hf_gpio_err_t RegisterPinInterrupt(hf_pin_num_t pin,
                                       hf_gpio_interrupt_trigger_t trigger,
                                       InterruptCallback callback,
                                       void* user_data) noexcept;

    /**
     * @brief Unregister a per-pin interrupt callback.
     * @param pin Pin number (0-15).
     * @return GPIO error code.
     */
    hf_gpio_err_t UnregisterPinInterrupt(hf_pin_num_t pin) noexcept;

    /**
     * @brief Configure the hardware interrupt pin (falling edge, active-low).
     * @return GPIO error code.
     */
    hf_gpio_err_t ConfigureHardwareInterrupt() noexcept;

    /**
     * @brief Static ISR callback for the hardware interrupt pin.
     * @param gpio      The GPIO pin that triggered.
     * @param trigger   Trigger type.
     * @param user_data Pointer to Pcal95555Handler instance.
     */
    static void HardwareInterruptCallback(BaseGpio* gpio,
                                          hf_gpio_interrupt_trigger_t trigger,
                                          void* user_data) noexcept;

    /**
     * @brief Process pending interrupts by reading status and dispatching callbacks.
     */
    void ProcessInterrupts() noexcept;

    //==========================================================================
    // Private Members
    //==========================================================================

    /// @name Core Components
    /// @{
    BaseI2c& i2c_device_;                              ///< I2C device reference (not owned).
    std::unique_ptr<HalI2cPcal95555Comm> i2c_adapter_; ///< I2C adapter (created in Initialize).
    std::unique_ptr<Pcal95555Driver> pcal95555_driver_; ///< Typed driver (created in Initialize).
    bool initialized_ = false;                          ///< Initialization state.
    mutable RtosMutex handler_mutex_;                   ///< Thread safety for handler operations.
    /// @}

    /// @name Pin Registry
    /// @{
    std::array<std::shared_ptr<Pcal95555GpioPin>, 16> pin_registry_; ///< Created pin wrappers.
    /// @}

    /// @name Interrupt Management
    /// @{
    BaseGpio* interrupt_pin_;       ///< Hardware INT pin (optional, not owned).
    bool interrupt_configured_;     ///< Whether hardware interrupt is active.
    /// @}

    /// @name Internal Pull Mode Tracking
    /// @brief Tracks pull mode per-pin since the driver doesn't provide readback.
    /// @{
    std::array<hf_gpio_pull_mode_t, 16> pull_mode_cache_;
    /// @}

    /// @name Edge Detection State
    /// @brief Tracks previous input state for rising/falling edge filtering.
    /// @{
    uint16_t prev_input_state_ = 0; ///< Last-read pin input levels (bitmask).
    /// @}
};

/// @} // end of PCAL95555_HAL_Handler

#endif // COMPONENT_HANDLER_PCAL95555_HANDLER_H_
