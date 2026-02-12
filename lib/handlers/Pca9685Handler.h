/**
 * @file Pca9685Handler.h
 * @brief Unified handler for the PCA9685 16-channel 12-bit PWM controller.
 *
 * @details
 * This file provides the complete HAL-level integration for a single PCA9685
 * PWM controller device. It bridges the HardFOC base interfaces (BaseI2c,
 * BasePwm, BaseGpio) with the templated pca9685::PCA9685<I2cType> driver
 * from the hf-pca9685-driver library.
 *
 * ## Architecture Overview
 *
 * The file contains four layers:
 *
 * 1. **CRTP I2C Communication Adapter** (HalI2cPca9685Comm):
 *    Bridges the HardFOC BaseI2c device-centric interface (where the I2C address
 *    is pre-configured on the device object) with the PCA9685 driver's CRTP-based
 *    pca9685::I2cInterface (which passes the address as a parameter).
 *    Includes address validation and thread-safe I2C operations.
 *
 * 2. **Pca9685Handler** (main class):
 *    Non-templated facade that owns a typed driver instance. Provides:
 *    - Lazy initialization with automatic device reset
 *    - Per-channel PWM control (duty cycle, raw tick values)
 *    - Global frequency control (24-1526 Hz)
 *    - Power management (sleep/wake)
 *    - Output configuration (invert, open-drain/totem-pole)
 *    - Error flag access and diagnostics
 *    - Factory methods for BasePwm and BaseGpio wrappers
 *
 * 3. **Pca9685PwmAdapter** (multi-channel BasePwm):
 *    Implements BasePwm for all 16 PWM channels, delegating to the parent handler.
 *    Supports duty cycle, frequency, phase shift (via on-time offset), and
 *    per-channel enable/disable (via full-on/full-off bits).
 *
 * 4. **Pca9685GpioPin** (per-pin BaseGpio wrapper):
 *    Implements BaseGpio for a single PCA9685 channel used as a digital output.
 *    HIGH = full-on, LOW = full-off. Output only (PCA9685 has no input capability).
 *
 * ## Ownership Model
 *
 * The handler owns all its internal resources:
 * - One HalI2cPca9685Comm (I2C communication adapter)
 * - One typed pca9685::PCA9685<HalI2cPca9685Comm> driver instance
 * - One Pca9685PwmAdapter (created on demand)
 * - Up to 16 Pca9685GpioPin wrappers (created on demand via pin registry)
 *
 * External managers (PwmManager, GpioManager) obtain shared_ptr references via
 * GetPwmAdapter() and CreateGpioPin().
 *
 * ## Initialization Sequence
 *
 * @code
 * // 1. Obtain a BaseI2c device reference (address pre-configured, e.g. 0x40)
 * // 2. Construct the handler
 * Pca9685Handler handler(i2c_device);
 *
 * // 3. Initialize (lazy -- happens automatically on first use, or explicitly)
 * if (!handler.EnsureInitialized()) { return; }
 *
 * // 4. Set PWM frequency (affects all channels)
 * handler.SetFrequency(50);  // 50 Hz for servos
 *
 * // 5a. Use as PWM via the adapter
 * auto pwm = handler.GetPwmAdapter();
 * pwm->SetDutyCycle(0, 0.5f);  // 50% duty on channel 0
 *
 * // 5b. Or use as digital GPIO
 * auto pin = handler.CreateGpioPin(15);
 * pin->SetState(HF_GPIO_STATE_ACTIVE);  // Channel 15 fully on
 * @endcode
 *
 * ## Thread Safety
 *
 * The handler uses RtosMutex for thread-safe access to:
 * - I2C communication (in the adapter's i2c_mutex_)
 * - All handler-level operations (handler_mutex_)
 *
 * @see GpioManager           Manager that can own GPIO pin wrappers
 * @see pca9685::PCA9685      Templated driver from hf-pca9685-driver
 *
 * @author HardFOC Team
 * @date 2025
 */

#ifndef COMPONENT_HANDLER_PCA9685_HANDLER_H_
#define COMPONENT_HANDLER_PCA9685_HANDLER_H_

#include <cstdint>
#include <memory>
#include <array>
#include <cmath>
#include "base/BaseGpio.h"
#include "base/BasePwm.h"
#include "base/BaseI2c.h"
#include "core/hf-core-drivers/external/hf-pca9685-driver/inc/pca9685.hpp"
#include "utils/RtosMutex.h"

// Forward declarations
class Pca9685Handler;

///////////////////////////////////////////////////////////////////////////////
/// @defgroup PCA9685_HAL_I2CAdapter HAL I2C Communication Adapter
/// @brief CRTP I2C adapter bridging BaseI2c to the PCA9685 driver.
/// @{
///////////////////////////////////////////////////////////////////////////////

/**
 * @class HalI2cPca9685Comm
 * @brief Concrete I2C communication adapter for PCA9685 using BaseI2c.
 *
 * @details
 * Implements all methods required by pca9685::I2cInterface<HalI2cPca9685Comm>
 * through the CRTP pattern. This class bridges the HardFOC BaseI2c device interface
 * (where the I2C address is pre-configured on the device) with the PCA9685 driver's
 * I2C interface (which passes the address as a parameter).
 *
 * Key behaviors:
 * - **Address validation**: Every write/read validates that the driver's address
 *   matches the BaseI2c device's configured address.
 * - **Register framing**: Writes are framed as [register, data...] per I2C convention.
 * - **Thread safety**: All I2C operations are mutex-protected.
 * - **Lazy initialization**: EnsureInitialized() delegates to BaseI2c.
 *
 * @note This class does NOT own the BaseI2c device; it must remain valid for
 *       the lifetime of this adapter.
 *
 * @see pca9685::I2cInterface  CRTP base class from the PCA9685 driver
 */
class HalI2cPca9685Comm : public pca9685::I2cInterface<HalI2cPca9685Comm> {
public:
    /**
     * @brief Construct the I2C communication adapter.
     * @param i2c_device Reference to a BaseI2c device with pre-configured address.
     * @warning The BaseI2c device must outlive this adapter.
     */
    explicit HalI2cPca9685Comm(BaseI2c& i2c_device) noexcept;

    /// @name CRTP-Required Methods
    /// Called by pca9685::I2cInterface<HalI2cPca9685Comm> via static dispatch.
    /// @{

    /**
     * @brief Write data to a device register.
     *
     * Frames the write as [register_address, data...] and sends via BaseI2c::Write().
     *
     * @param addr 7-bit I2C address (validated against BaseI2c device).
     * @param reg  Register address to write to.
     * @param data Pointer to data buffer.
     * @param len  Number of bytes to write.
     * @return true if write succeeded, false on address mismatch or I2C error.
     */
    bool Write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) noexcept;

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
    bool Read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) noexcept;

    /**
     * @brief Ensure the I2C bus is initialized and ready.
     * @return true if the BaseI2c device is initialized or was initialized successfully.
     */
    bool EnsureInitialized() noexcept;

    /// @}

private:
    BaseI2c& i2c_device_;              ///< I2C device interface (not owned).
    mutable RtosMutex i2c_mutex_;      ///< Thread safety for I2C operations.
};

/// @} // end of PCA9685_HAL_I2CAdapter

///////////////////////////////////////////////////////////////////////////////
/// @defgroup PCA9685_HAL_PwmAdapter Multi-Channel PWM Adapter
/// @brief BasePwm adapter for all 16 PCA9685 PWM channels.
/// @{
///////////////////////////////////////////////////////////////////////////////

/**
 * @class Pca9685PwmAdapter
 * @brief BasePwm adapter for all 16 PCA9685 PWM channels.
 *
 * @details
 * Wraps the PCA9685's 16 PWM channels as a HardFOC BasePwm instance.
 * All hardware operations delegate to the parent Pca9685Handler, which in turn
 * calls the typed PCA9685 driver.
 *
 * ## Hardware Constraints
 *
 * - **Global frequency**: The PCA9685 has a single oscillator with one prescaler.
 *   SetFrequency() on any channel changes the frequency for ALL channels.
 *   The adapter tracks the current frequency and returns it for any channel.
 *
 * - **12-bit resolution**: PWM values are 0-4095 (12-bit).
 *
 * - **Phase shift**: Supported via the PCA9685's on-time offset. Each channel
 *   has a programmable on-time (0-4095) which creates a phase offset.
 *
 * - **No complementary outputs**: SetComplementaryOutput() is unsupported.
 *
 * @note Created via Pca9685Handler::GetPwmAdapter(). The handler owns this instance.
 */
class Pca9685PwmAdapter : public BasePwm {
public:
    /** @brief Number of PWM channels on the PCA9685. */
    static constexpr uint8_t kMaxChannels = 16;

    /** @brief PWM resolution in bits. */
    static constexpr uint8_t kResolutionBits = 12;

    /** @brief Maximum raw PWM value (2^12 - 1 = 4095). */
    static constexpr uint16_t kMaxRawValue = 4095;

    /** @brief Minimum PWM frequency in Hz. */
    static constexpr uint32_t kMinFrequencyHz = 24;

    /** @brief Maximum PWM frequency in Hz. */
    static constexpr uint32_t kMaxFrequencyHz = 1526;

    /**
     * @brief Construct the PWM adapter.
     * @param parent_handler Pointer to the owning handler (must not be null).
     */
    explicit Pca9685PwmAdapter(Pca9685Handler* parent_handler) noexcept;

    /** @brief Default destructor. */
    ~Pca9685PwmAdapter() override = default;

    /// @name BasePwm Interface Implementation
    /// @{

    /** @brief Initialize the PWM adapter (ensures parent handler is initialized). */
    hf_pwm_err_t Initialize() noexcept override;

    /** @brief Deinitialize (marks adapter as uninitialized). */
    hf_pwm_err_t Deinitialize() noexcept override;

    /**
     * @brief Enable a PWM channel (restore its duty cycle from cache).
     * @param channel_id Channel number (0-15).
     * @return PWM_SUCCESS or error code.
     */
    hf_pwm_err_t EnableChannel(hf_channel_id_t channel_id) noexcept override;

    /**
     * @brief Disable a PWM channel (set to full-off).
     * @param channel_id Channel number (0-15).
     * @return PWM_SUCCESS or error code.
     */
    hf_pwm_err_t DisableChannel(hf_channel_id_t channel_id) noexcept override;

    /**
     * @brief Check if a channel is enabled.
     * @param channel_id Channel number (0-15).
     * @return true if the channel is active.
     */
    bool IsChannelEnabled(hf_channel_id_t channel_id) const noexcept override;

    /**
     * @brief Set duty cycle for a channel (0.0-1.0).
     * @param channel_id Channel number (0-15).
     * @param duty_cycle  Duty cycle (0.0 = off, 1.0 = full on).
     * @return PWM_SUCCESS or error code.
     */
    hf_pwm_err_t SetDutyCycle(hf_channel_id_t channel_id,
                              float duty_cycle) noexcept override;

    /**
     * @brief Set raw PWM tick value for a channel.
     * @param channel_id Channel number (0-15).
     * @param raw_value  Off-time tick count (0-4095).
     * @return PWM_SUCCESS or error code.
     */
    hf_pwm_err_t SetDutyCycleRaw(hf_channel_id_t channel_id,
                                 hf_u32_t raw_value) noexcept override;

    /**
     * @brief Set PWM frequency (global -- affects ALL channels).
     *
     * The PCA9685 has a single prescaler so frequency is shared across all 16
     * channels. The channel_id parameter is accepted for API conformance but
     * the frequency change applies globally.
     *
     * @param channel_id Ignored (frequency is global).
     * @param frequency_hz Frequency in Hz (24-1526).
     * @return PWM_SUCCESS or error code.
     */
    hf_pwm_err_t SetFrequency(hf_channel_id_t channel_id,
                              hf_frequency_hz_t frequency_hz) noexcept override;

    /**
     * @brief Set phase shift for a channel (via PCA9685 on-time offset).
     *
     * The PCA9685 supports staggered outputs by programming different on-time
     * values per channel. This maps to phase shift:
     *   on_time = (phase_degrees / 360.0) * 4096
     *
     * @param channel_id Channel number (0-15).
     * @param phase_shift_degrees Phase offset in degrees (0-360).
     * @return PWM_SUCCESS or error code.
     */
    hf_pwm_err_t SetPhaseShift(hf_channel_id_t channel_id,
                               float phase_shift_degrees) noexcept override;

    /**
     * @brief Wake the PCA9685 from sleep (start all channels).
     * @return PWM_SUCCESS or error code.
     */
    hf_pwm_err_t StartAll() noexcept override;

    /**
     * @brief Put the PCA9685 into sleep mode (stop all outputs).
     * @return PWM_SUCCESS or error code.
     */
    hf_pwm_err_t StopAll() noexcept override;

    /**
     * @brief No-op (PCA9685 updates immediately on register write).
     * @return PWM_SUCCESS.
     */
    hf_pwm_err_t UpdateAll() noexcept override;

    /**
     * @brief Not supported by PCA9685.
     * @return PWM_ERR_UNSUPPORTED_OPERATION.
     */
    hf_pwm_err_t SetComplementaryOutput(hf_channel_id_t primary_channel,
                                        hf_channel_id_t complementary_channel,
                                        hf_u32_t deadtime_ns) noexcept override;

    /**
     * @brief Get the cached duty cycle for a channel.
     * @param channel_id Channel number (0-15).
     * @return Duty cycle (0.0-1.0), or -1.0 on error.
     */
    float GetDutyCycle(hf_channel_id_t channel_id) const noexcept override;

    /**
     * @brief Get the current PWM frequency (global).
     * @param channel_id Ignored (frequency is global).
     * @return Frequency in Hz, or 0 on error.
     */
    hf_frequency_hz_t GetFrequency(hf_channel_id_t channel_id) const noexcept override;

    /// @}

    /// Allow handler to access internals.
    friend class Pca9685Handler;

private:
    Pca9685Handler* parent_handler_;                    ///< Owning handler (not owned).

    /// @name Per-Channel State Cache
    /// @{
    std::array<float, kMaxChannels> duty_cache_;        ///< Cached duty cycles (0.0-1.0).
    std::array<uint16_t, kMaxChannels> on_time_cache_;  ///< Cached on-time values (phase offset).
    std::array<bool, kMaxChannels> channel_enabled_;    ///< Channel enable state.
    /// @}

    hf_frequency_hz_t current_frequency_hz_ = 200;     ///< Current global frequency.

    /** @brief Validate channel ID is in range 0-15. */
    bool validateChannel(hf_channel_id_t channel_id) const noexcept {
        return channel_id < kMaxChannels;
    }
};

/// @} // end of PCA9685_HAL_PwmAdapter

///////////////////////////////////////////////////////////////////////////////
/// @defgroup PCA9685_HAL_GpioPin Per-Pin GPIO Wrapper
/// @brief BaseGpio adapter for using PCA9685 channels as digital outputs.
/// @{
///////////////////////////////////////////////////////////////////////////////

/**
 * @class Pca9685GpioPin
 * @brief BaseGpio adapter for a single PCA9685 channel used as a digital output.
 *
 * @details
 * Wraps one of the 16 PCA9685 PWM channels (0-15) as a HardFOC BaseGpio instance.
 * Since the PCA9685 is a PWM output controller (not a GPIO expander), this wrapper
 * operates in output-only mode:
 *
 * - **HIGH**: Sets the channel to full-on (bit 12 of ON register)
 * - **LOW**: Sets the channel to full-off (bit 12 of OFF register)
 * - **Input**: Not supported (PCA9685 cannot read pin states)
 * - **Pull mode**: Not supported
 * - **Interrupts**: Not supported
 *
 * This is useful for controlling LEDs, relays, or enable pins where simple
 * on/off control is sufficient without PWM.
 *
 * @note Pin instances are created via Pca9685Handler::CreateGpioPin() and stored
 *       in the handler's pin registry as shared_ptr.
 */
class Pca9685GpioPin : public BaseGpio {
public:
    /**
     * @brief Construct a PCA9685 GPIO pin wrapper.
     * @param pin            Channel number (0-15, used as pin number).
     * @param parent_handler Pointer to the owning handler (must not be null).
     * @param active_state   Active polarity (default: active high).
     */
    Pca9685GpioPin(hf_pin_num_t pin,
                   Pca9685Handler* parent_handler,
                   hf_gpio_active_state_t active_state =
                       hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH) noexcept;

    /** @brief Default destructor. */
    ~Pca9685GpioPin() override = default;

    /// @name BaseGpio Interface Implementation
    /// @{

    /** @brief Initialize the GPIO pin (sets channel to full-off by default). */
    bool Initialize() noexcept override;

    /** @brief Deinitialize (marks as uninitialized; channel state persists). */
    bool Deinitialize() noexcept override;

    /** @brief Check if this channel number is valid (0-15). */
    bool IsPinAvailable() const noexcept override;

    /** @brief Returns 16 (total channels on the PCA9685). */
    hf_u8_t GetMaxPins() const noexcept override { return 16; }

    /** @brief Returns a description string like "PCA9685_CH_5". */
    const char* GetDescription() const noexcept override;

    /// @}

protected:
    /// @name BaseGpio Protected Implementation
    /// @{

    /**
     * @brief Set pin direction. Only OUTPUT is supported.
     * @return GPIO_SUCCESS for OUTPUT, GPIO_ERR_UNSUPPORTED_OPERATION for INPUT.
     */
    hf_gpio_err_t SetDirectionImpl(hf_gpio_direction_t direction) noexcept override;

    /** @brief Get direction (always OUTPUT). */
    hf_gpio_err_t GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept override;

    /** @brief Set output mode. Only push-pull is natively supported (global setting). */
    hf_gpio_err_t SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept override;

    /** @brief Get output mode (tracked internally). */
    hf_gpio_err_t GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept override;

    /** @brief Not supported -- returns GPIO_ERR_UNSUPPORTED_OPERATION. */
    hf_gpio_err_t SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept override;

    /** @brief Returns FLOATING (no pull resistors on PCA9685). */
    hf_gpio_pull_mode_t GetPullModeImpl() const noexcept override;

    /**
     * @brief Write pin level via the PCA9685 driver.
     *
     * HIGH = SetChannelFullOn, LOW = SetChannelFullOff.
     */
    hf_gpio_err_t SetPinLevelImpl(hf_gpio_level_t level) noexcept override;

    /**
     * @brief Read pin level (returns cached state since PCA9685 is output-only).
     */
    hf_gpio_err_t GetPinLevelImpl(hf_gpio_level_t& level) noexcept override;

    /// @}

    friend class Pca9685Handler;

private:
    hf_pin_num_t pin_;                     ///< Channel number (0-15).
    Pca9685Handler* parent_handler_;       ///< Owning handler (not owned).
    char description_[32] = {};            ///< Human-readable description.
    hf_gpio_level_t cached_level_ =
        hf_gpio_level_t::HF_GPIO_LEVEL_LOW; ///< Cached output level.
};

/// @} // end of PCA9685_HAL_GpioPin

///////////////////////////////////////////////////////////////////////////////
/// @defgroup PCA9685_HAL_Handler PCA9685 Handler
/// @brief Non-templated facade for PCA9685 PWM controller integration.
/// @{
///////////////////////////////////////////////////////////////////////////////

/**
 * @class Pca9685Handler
 * @brief Unified, non-templated handler for a single PCA9685 PWM controller.
 *
 * @details
 * Pca9685Handler is the primary interface that application and manager code uses
 * to interact with a PCA9685 PWM controller. It hides the template complexity of
 * pca9685::PCA9685<I2cType> behind a clean public API.
 *
 * ## Design Decisions
 *
 * - **Non-templated**: The handler owns a concrete typed driver instance
 *   (pca9685::PCA9685<HalI2cPca9685Comm>) internally. Since only one I2C
 *   communication type is used per handler, no dispatch mechanism is needed.
 *
 * - **Lazy initialization**: The driver and adapter are created in Initialize(),
 *   not in the constructor.
 *
 * - **Dual-purpose channels**: Each channel can be used as either:
 *   - A PWM output (via the Pca9685PwmAdapter / BasePwm interface)
 *   - A digital output (via Pca9685GpioPin / BaseGpio interface)
 *   Using both simultaneously for the same channel is allowed but the last
 *   write wins.
 *
 * @see HalI2cPca9685Comm    I2C communication adapter
 * @see Pca9685PwmAdapter    Multi-channel BasePwm wrapper
 * @see Pca9685GpioPin       Per-channel BaseGpio wrapper
 */
class Pca9685Handler {
public:
    //==========================================================================
    /// @name Type Aliases
    /// @{
    //==========================================================================

    /** @brief Typed PCA9685 driver using our I2C adapter. */
    using Pca9685Driver = pca9685::PCA9685<HalI2cPca9685Comm>;

    /// @}

    //==========================================================================
    /// @name Construction and Destruction
    /// @{
    //==========================================================================

    /**
     * @brief Construct a handler for a PCA9685 device.
     *
     * The I2C adapter and driver are not created until EnsureInitialized() or
     * the first operation that requires them.
     *
     * @param i2c_device Reference to a BaseI2c device with address pre-configured
     *                   (typically 0x40-0x7F). Must outlive the handler.
     */
    explicit Pca9685Handler(BaseI2c& i2c_device) noexcept;

    /** @brief Destructor. Releases driver, adapter, and all wrappers. */
    ~Pca9685Handler() = default;

    /// Non-copyable.
    Pca9685Handler(const Pca9685Handler&) = delete;
    /// Non-copyable.
    Pca9685Handler& operator=(const Pca9685Handler&) = delete;

    /// @}

    //==========================================================================
    /// @name Initialization and Lifecycle
    /// @{
    //==========================================================================

    /**
     * @brief Ensure the handler is initialized (lazy initialization).
     *
     * On first call, creates the I2C adapter and driver, runs the driver's
     * EnsureInitialized() (which resets the device).
     *
     * @return true if already initialized or initialization succeeded.
     */
    bool EnsureInitialized() noexcept;

    /**
     * @brief Deinitialize the handler and release resources.
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
    /// @name PWM Control
    /// @brief Per-channel and global PWM operations.
    /// @{
    //==========================================================================

    /**
     * @brief Set the PWM frequency (global -- all channels).
     * @param freq_hz Desired frequency in Hz (24-1526).
     * @return true on success, false on failure.
     */
    bool SetFrequency(float freq_hz) noexcept;

    /**
     * @brief Set duty cycle for a channel (0.0-1.0).
     * @param channel Channel number (0-15).
     * @param duty    Duty cycle (0.0 = off, 1.0 = fully on).
     * @return true on success, false on failure.
     */
    bool SetDuty(uint8_t channel, float duty) noexcept;

    /**
     * @brief Set raw on/off tick values for a channel.
     * @param channel  Channel number (0-15).
     * @param on_time  Tick count when signal turns ON (0-4095).
     * @param off_time Tick count when signal turns OFF (0-4095).
     * @return true on success, false on failure.
     */
    bool SetPwm(uint8_t channel, uint16_t on_time, uint16_t off_time) noexcept;

    /**
     * @brief Set all channels to the same PWM value.
     * @param on_time  Tick count when signal turns ON (0-4095).
     * @param off_time Tick count when signal turns OFF (0-4095).
     * @return true on success, false on failure.
     */
    bool SetAllPwm(uint16_t on_time, uint16_t off_time) noexcept;

    /**
     * @brief Set a channel to fully ON (100% duty, no PWM).
     * @param channel Channel number (0-15).
     * @return true on success, false on failure.
     */
    bool SetChannelFullOn(uint8_t channel) noexcept;

    /**
     * @brief Set a channel to fully OFF (0% duty).
     * @param channel Channel number (0-15).
     * @return true on success, false on failure.
     */
    bool SetChannelFullOff(uint8_t channel) noexcept;

    /// @}

    //==========================================================================
    /// @name Power Management
    /// @{
    //==========================================================================

    /**
     * @brief Put PCA9685 into low-power sleep mode.
     * @return true on success, false on failure.
     */
    bool Sleep() noexcept;

    /**
     * @brief Wake PCA9685 from sleep mode.
     * @return true on success, false on failure.
     */
    bool Wake() noexcept;

    /// @}

    //==========================================================================
    /// @name Output Configuration
    /// @{
    //==========================================================================

    /**
     * @brief Set output polarity inversion (global).
     * @param invert true to invert all outputs, false for normal.
     * @return true on success, false on failure.
     */
    bool SetOutputInvert(bool invert) noexcept;

    /**
     * @brief Set output driver mode (global).
     * @param totem_pole true for totem-pole (default), false for open-drain.
     * @return true on success, false on failure.
     */
    bool SetOutputDriverMode(bool totem_pole) noexcept;

    /// @}

    //==========================================================================
    /// @name Wrapper Factories
    /// @brief Create BasePwm and BaseGpio wrappers for manager-layer usage.
    /// @{
    //==========================================================================

    /** @brief Total number of PWM channels. */
    static constexpr uint8_t ChannelCount() noexcept { return 16; }

    /**
     * @brief Get the I2C address of the underlying device.
     * @return 7-bit I2C address, or 0 if driver not created.
     */
    uint8_t GetI2cAddress() const noexcept;

    /**
     * @brief Get or create the BasePwm adapter for all 16 channels.
     *
     * The adapter is created on first call and cached. Subsequent calls
     * return the same shared_ptr.
     *
     * @return shared_ptr<BasePwm> or nullptr on failure.
     */
    std::shared_ptr<BasePwm> GetPwmAdapter() noexcept;

    /**
     * @brief Create or retrieve a BaseGpio pin wrapper for a channel.
     *
     * Uses the channel as a digital output (fully on / fully off).
     * If a wrapper for the channel already exists and allow_existing is true,
     * the existing instance is returned.
     *
     * @param channel        Channel number (0-15).
     * @param active_state   Active polarity (default: active high).
     * @param allow_existing If true, returns existing wrapper; if false, fails if exists.
     * @return shared_ptr<BaseGpio> or nullptr on failure.
     */
    std::shared_ptr<BaseGpio> CreateGpioPin(
        hf_pin_num_t channel,
        hf_gpio_active_state_t active_state =
            hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
        bool allow_existing = true) noexcept;

    /**
     * @brief Get an existing GPIO pin wrapper by channel number.
     * @param channel Channel number (0-15).
     * @return shared_ptr<BaseGpio> or nullptr if not created.
     */
    std::shared_ptr<BaseGpio> GetGpioPin(hf_pin_num_t channel) noexcept;

    /**
     * @brief Check if a GPIO pin wrapper has been created for a channel.
     * @param channel Channel number (0-15).
     * @return true if the wrapper exists in the registry.
     */
    bool IsPinCreated(hf_pin_num_t channel) const noexcept;

    /// @}

    //==========================================================================
    /// @name Error Management
    /// @{
    //==========================================================================

    /**
     * @brief Get the driver's error flags.
     * @return 16-bit error flag bitmask (see pca9685::PCA9685::Error enum).
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
     */
    void DumpDiagnostics() const noexcept;

    /// @}

    /// Allow wrapper classes to access private driver.
    friend class Pca9685PwmAdapter;
    friend class Pca9685GpioPin;

private:
    //==========================================================================
    // Private Methods
    //==========================================================================

    /** @brief Check init under already-held handler_mutex_. */
    bool ensureInitializedLocked() noexcept;

    /** @brief Internal initialization (called under mutex). */
    hf_pwm_err_t initializeInternal() noexcept;

    /** @brief Internal deinitialization. */
    hf_pwm_err_t deinitializeInternal() noexcept;

    /** @brief Validate channel number (0-15). */
    bool validateChannel(uint8_t channel) const noexcept { return channel < 16; }

    //==========================================================================
    // Private Members
    //==========================================================================

    /// @name Core Components
    /// @{
    BaseI2c& i2c_device_;                              ///< I2C device reference (not owned).
    std::unique_ptr<HalI2cPca9685Comm> i2c_adapter_;   ///< I2C adapter (created in init).
    std::unique_ptr<Pca9685Driver> pca9685_driver_;    ///< Typed driver (created in init).
    bool initialized_ = false;                          ///< Initialization state.
    mutable RtosMutex handler_mutex_;                   ///< Thread safety for all operations.
    /// @}

    /// @name Wrapper Registry
    /// @{
    std::shared_ptr<Pca9685PwmAdapter> pwm_adapter_;                 ///< PWM adapter (lazy).
    std::array<std::shared_ptr<Pca9685GpioPin>, 16> gpio_registry_;  ///< GPIO pin registry.
    /// @}
};

/// @} // end of PCA9685_HAL_Handler

#endif // COMPONENT_HANDLER_PCA9685_HANDLER_H_
