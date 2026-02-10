/**
 * @file Pcal95555Handler.h
 * @brief Minimal PCAL95555 I2C GPIO expander handler for direct pin control.
 *
 * This class provides a focused, platform-agnostic handler for the PCAL95555 16-bit I2C GPIO expander.
 * It exposes all pin operations by pin number (0-15), with no platform mapping or diagnostics.
 *
 * Features:
 * - Direct pin control (input/output, direction, pull, toggle, etc.)
 * - Thread-safe I2C communication
 * - Minimal, modern C++ interface
 * - No platform mapping, diagnostics, or functional pin abstractions
 *
 * @author HardFOC Team
 * @version 1.0
 * @date 2025
 * @copyright HardFOC
 */

#ifndef COMPONENT_HANDLER_PCAL95555_HANDLER_H_
#define COMPONENT_HANDLER_PCAL95555_HANDLER_H_

#include "base/BaseGpio.h"
#include "base/BaseI2c.h"
#include "utils-and-drivers/hf-core-drivers/external/hf-pcal95555-driver/src/pcal95555.hpp"
#include "utils/RtosMutex.h"
#include <memory>
#include <cstdint>
#include <array>
#include <functional>
#include <vector>

// Forward declarations
class Pcal95555Handler;

/**
 * @brief Adapter to connect BaseI2c device to PCAL95555::i2cBus interface.
 * 
 * This adapter bridges the device-centric BaseI2c interface (where device address 
 * is pre-configured) with the PCAL95555 driver's i2cBus interface (which expects 
 * address parameters). The adapter validates that operations target the correct 
 * device and strips the address parameter before calling BaseI2c methods.
 */
class Pcal95555I2cAdapter : public PCAL95555::i2cBus {
public:
    /**
     * @brief Construct adapter with BaseI2c device reference.
     * @param i2c_device Reference to BaseI2c device (not bus)
     * @note The device address is already configured in the BaseI2c device
     */
    explicit Pcal95555I2cAdapter(BaseI2c& i2c_device) noexcept 
        : i2c_device_(i2c_device) {}

    /**
     * @brief Write data to device register via BaseI2c device.
     * @param addr I2C device address (validated against BaseI2c device address)
     * @param reg Register address
     * @param data Data buffer to write
     * @param len Number of bytes to write
     * @return true if successful, false on error
     * @note Address validation ensures type safety - prevents accidental cross-device communication
     */
    bool write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) override;

    /**
     * @brief Read data from device register via BaseI2c device.
     * @param addr I2C device address (validated against BaseI2c device address)
     * @param reg Register address
     * @param data Buffer to store read data
     * @param len Number of bytes to read
     * @return true if successful, false on error
     * @note Address validation ensures type safety - prevents accidental cross-device communication
     */
    bool read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) override;

private:
    BaseI2c& i2c_device_;           ///< Reference to I2C device (not bus)
    mutable RtosMutex i2c_mutex_;   ///< Thread safety for I2C operations
};

// ===================== Per-Pin BaseGpio Wrapper ===================== //
class Pcal95555GpioPin : public BaseGpio {
public:
    /**
     * @brief Construct a PCAL95555 GPIO pin wrapper.
     * @param pin Pin number (0-15)
     * @param driver Shared PCAL95555 driver instance
     * @param parent_handler Parent handler for interrupt management
     * @param directions Initial direction
     * @param active_state Active polarity
     * @param output_mode Output mode
     * @param pull_mode Pull resistor mode
     */
    Pcal95555GpioPin(hf_pin_num_t pin,
                     std::shared_ptr<PCAL95555> driver,
                     Pcal95555Handler* parent_handler,
                     hf_gpio_direction_t direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
                     hf_gpio_active_state_t active_state = hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
                     hf_gpio_output_mode_t output_mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
                     hf_gpio_pull_mode_t pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING) noexcept;
    ~Pcal95555GpioPin() override = default;

    hf_bool_t Initialize() noexcept override;
    hf_bool_t Deinitialize() noexcept override;
    hf_bool_t IsPinAvailable() const noexcept override;
    hf_u8_t GetMaxPins() const noexcept override { return 16; }
    const char* GetDescription() const noexcept override;
    hf_bool_t SupportsInterrupts() const noexcept override { return true; }
    hf_gpio_err_t ConfigureInterrupt(hf_gpio_interrupt_trigger_t trigger,
                                     InterruptCallback callback = nullptr,
                                     void* user_data = nullptr) noexcept override;
    // Advanced PCAL95555A features
    hf_gpio_err_t SetPolarityInversion(hf_bool_t invert) noexcept;
    hf_gpio_err_t GetPolarityInversion(hf_bool_t& invert) noexcept;
    hf_gpio_err_t SetInterruptMask(hf_bool_t mask) noexcept;
    hf_gpio_err_t GetInterruptMask(hf_bool_t& mask) noexcept;
    hf_gpio_err_t GetInterruptStatus(hf_bool_t& status) noexcept;

protected:
    hf_gpio_err_t SetDirectionImpl(hf_gpio_direction_t direction) noexcept override;
    hf_gpio_err_t SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept override;
    hf_gpio_err_t SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept override;
    hf_gpio_pull_mode_t GetPullModeImpl() const noexcept override;
    hf_gpio_err_t SetPinLevelImpl(hf_gpio_level_t level) noexcept override;
    hf_gpio_err_t GetPinLevelImpl(hf_gpio_level_t& level) noexcept override;
    hf_gpio_err_t GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept override;
    hf_gpio_err_t GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept override;

    // Allow handler to access pin interrupt data directly
    friend class Pcal95555Handler;

private:
    hf_pin_num_t pin_;
    std::shared_ptr<PCAL95555> driver_;
    Pcal95555Handler* parent_handler_;  ///< Reference to parent handler for interrupt management
    mutable RtosMutex pin_mutex_;
    char description_[32] = {};
    
    // Pin-owned interrupt data
    InterruptCallback interrupt_callback_ = nullptr;   ///< This pin's interrupt callback
    void* interrupt_user_data_ = nullptr;             ///< This pin's user data
    hf_gpio_interrupt_trigger_t interrupt_trigger_ = hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_NONE; ///< This pin's trigger type
    bool interrupt_enabled_ = false;                  ///< Whether this pin has interrupt enabled
};

// ===================== Handler Factory & Advanced Features ===================== //
class Pcal95555Handler {
public:
    /**
     * @brief Construct a new handler for a PCAL95555 chip with optional interrupt support.
     * @param i2c_device Reference to BaseI2c device (not bus)
     * @param interrupt_pin Optional hardware interrupt pin (can be nullptr for polling mode)
     * @note The device address should already be configured in the BaseI2c device
     */
    explicit Pcal95555Handler(BaseI2c& i2c_device, BaseGpio* interrupt_pin = nullptr) noexcept;
    ~Pcal95555Handler() = default;

    // No copy
    Pcal95555Handler(const Pcal95555Handler&) = delete;
    Pcal95555Handler& operator=(const Pcal95555Handler&) = delete;
    // Allow move
    Pcal95555Handler(Pcal95555Handler&&) = default;
    Pcal95555Handler& operator=(Pcal95555Handler&&) = default;

    //**************************************************************************//
    //**                    INITIALIZATION & LIFECYCLE                        **//
    //**************************************************************************//

    /**
     * @brief Ensure the handler is initialized (lazy initialization pattern).
     * @return true if already initialized or initialization successful, false otherwise
     * @note This is the preferred method for initialization following the pattern used by MotorController and ImuManager
     */
    bool EnsureInitialized() noexcept;

    /**
     * @brief Ensure the handler is deinitialized (lazy deinitialization pattern).
     * @return true if already deinitialized or deinitialization successful, false otherwise
     * @note This is the preferred method for deinitialization following the pattern used by MotorController and ImuManager
     */
    bool EnsureDeinitialized() noexcept;

    /**
     * @brief Check if the handler is initialized.
     * @return true if initialized, false otherwise
     */
    bool IsInitialized() const noexcept { return initialized_; }

    //**************************************************************************//
    //**                       BASIC GPIO OPERATIONS                          **//
    //**************************************************************************//

    // Single pin operations (return hf_gpio_err_t for consistency)
    hf_gpio_err_t SetDirection(hf_u8_t pin, hf_gpio_direction_t direction) noexcept;
    hf_gpio_err_t SetOutput(hf_u8_t pin, hf_bool_t active) noexcept;
    hf_gpio_err_t ReadInput(hf_u8_t pin, hf_bool_t& active) noexcept;
    hf_gpio_err_t Toggle(hf_u8_t pin) noexcept;
    hf_gpio_err_t SetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t pull_mode) noexcept;
    hf_gpio_err_t GetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t& pull_mode) noexcept;

    // Batch operations
    hf_gpio_err_t SetDirections(uint16_t pin_mask, hf_gpio_direction_t direction) noexcept;
    hf_gpio_err_t SetOutputs(uint16_t pin_mask, hf_bool_t active) noexcept;
    hf_gpio_err_t SetPullModes(uint16_t pin_mask, hf_gpio_pull_mode_t pull_mode) noexcept;

    //**************************************************************************//
    //**                      INTERRUPT MANAGEMENT                            **//
    //**************************************************************************//

    /**
     * @brief Check if hardware interrupt support is available.
     * @return true if interrupt pin is configured, false otherwise
     */
    bool HasInterruptSupport() const noexcept { return interrupt_pin_ != nullptr; }

    /**
     * @brief Get interrupt configuration status.
     * @return true if hardware interrupt is configured and enabled, false otherwise
     */
    bool IsInterruptConfigured() const noexcept { return interrupt_configured_; }

    // Get all interrupt masks and status
    hf_gpio_err_t GetAllInterruptMasks(uint16_t& mask) noexcept;
    hf_gpio_err_t GetAllInterruptStatus(uint16_t& status) noexcept;

    //**************************************************************************//
    //**                       FACTORY METHODS                                **//
    //**************************************************************************//

    /**
     * @brief Get the number of pins (always 16).
     */
    static constexpr hf_u8_t PinCount() noexcept { return 16; }

    /**
     * @brief Get the I2C address.
     */
    hf_u8_t GetI2cAddress() const noexcept;

    /**
     * @brief Create or get existing BaseGpio-compatible pin wrapper for a PCAL95555 pin.
     * @param pin Pin number (0-15)
     * @param direction Initial direction (ignored if pin already exists)
     * @param active_state Active polarity (ignored if pin already exists)
     * @param output_mode Output mode (ignored if pin already exists)
     * @param pull_mode Pull resistor mode (ignored if pin already exists)
     * @param allow_existing If true, returns existing pin; if false, fails if pin exists
     * @return std::shared_ptr<BaseGpio> or nullptr if invalid or already exists (when allow_existing=false)
     * @note Multiple calls with same pin number return the same shared instance
     */
    std::shared_ptr<BaseGpio> CreateGpioPin(
        hf_pin_num_t pin,
        hf_gpio_direction_t direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
        hf_gpio_active_state_t active_state = hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH,
        hf_gpio_output_mode_t output_mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
        hf_gpio_pull_mode_t pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING,
        bool allow_existing = true) noexcept;

    /**
     * @brief Get existing GPIO pin by number.
     * @param pin Pin number (0-15)
     * @return std::shared_ptr<BaseGpio> or nullptr if pin doesn't exist
     */
    std::shared_ptr<BaseGpio> GetGpioPin(hf_pin_num_t pin) noexcept;

    /**
     * @brief Check if a pin is already created.
     * @param pin Pin number (0-15)
     * @return true if pin exists, false otherwise
     */
    bool IsPinCreated(hf_pin_num_t pin) const noexcept;

    /**
     * @brief Get list of all created pin numbers.
     * @return Vector of pin numbers that have been created
     */
    std::vector<hf_pin_num_t> GetCreatedPins() const noexcept;

    //**************************************************************************//
    //**                      ADVANCED PCAL95555A FEATURES                    **//
    //**************************************************************************//

    // Advanced PCAL95555A features
    hf_bool_t SetPolarityInversion(hf_pin_num_t pin, hf_bool_t invert) noexcept;
    hf_bool_t GetPolarityInversion(hf_pin_num_t pin, hf_bool_t& invert) noexcept;
    hf_bool_t SetInterruptMask(hf_pin_num_t pin, hf_bool_t mask) noexcept;
    hf_bool_t GetInterruptMask(hf_pin_num_t pin, hf_bool_t& mask) noexcept;
    hf_bool_t GetInterruptStatus(hf_pin_num_t pin, hf_bool_t& status) noexcept;
    hf_bool_t SoftwareReset() noexcept;
    hf_bool_t PowerDown() noexcept;

    //**************************************************************************//
    //**                   FRIEND ACCESS FOR GPIO PINS                        **//
    //**************************************************************************//

    // Allow Pcal95555GpioPin to access private interrupt methods
    friend class Pcal95555GpioPin;
    
private:
    //**************************************************************************//
    //**                      PRIVATE MEMBERS                                 **//
    //**************************************************************************//

    // Core driver components (lazy initialization)
    BaseI2c& i2c_device_;                                   ///< Reference to BaseI2c device
    std::unique_ptr<Pcal95555I2cAdapter> i2c_adapter_;      ///< I2C adapter (created during initialization)
    std::shared_ptr<PCAL95555> pcal95555_driver_;           ///< PCAL95555 driver (created during initialization)
    hf_bool_t initialized_ = false;                         ///< Initialization status
    mutable RtosMutex handler_mutex_;                       ///< Thread safety mutex

    // Pin registry management
    std::array<std::shared_ptr<Pcal95555GpioPin>, 16> pin_registry_;  ///< Registry of created GPIO pins
    mutable RtosMutex pin_registry_mutex_;                  ///< Thread safety for pin registry

    // Interrupt management
    BaseGpio* interrupt_pin_;                               ///< Hardware interrupt pin (optional)
    bool interrupt_configured_;                             ///< Hardware interrupt configured flag
    mutable RtosMutex interrupt_mutex_;                     ///< Thread safety for interrupt operations

    //**************************************************************************//
    //**                      PRIVATE METHODS                                 **//
    //**************************************************************************//

    /**
     * @brief Initialize the PCAL95555 handler and configure hardware interrupt if available.
     * @return GPIO error code
     * @note Internal method called by EnsureInitialized()
     */
    hf_gpio_err_t Initialize() noexcept;
    
    /**
     * @brief Deinitialize the handler and clean up interrupt resources.
     * @return GPIO error code
     */
    hf_gpio_err_t Deinitialize() noexcept;

    /**
     * @brief Validate pin number (0-15).
     * @param pin Pin number to validate
     * @return true if valid, false otherwise
     */
    hf_bool_t ValidatePin(hf_u8_t pin) const noexcept { return pin < 16; }

    /**
     * @brief Register a pin's interrupt callback (called by Pcal95555GpioPin).
     * @param pin Pin number (0-15)
     * @param trigger Interrupt trigger type
     * @param callback Interrupt callback function
     * @param user_data User data to pass to callback
     * @return GPIO error code
     */
    hf_gpio_err_t RegisterPinInterrupt(hf_pin_num_t pin, 
                                       hf_gpio_interrupt_trigger_t trigger,
                                       InterruptCallback callback,
                                       void* user_data) noexcept;

    /**
     * @brief Unregister a pin's interrupt callback (called by Pcal95555GpioPin).
     * @param pin Pin number (0-15)  
     * @return GPIO error code
     */
    hf_gpio_err_t UnregisterPinInterrupt(hf_pin_num_t pin) noexcept;

    /**
     * @brief Configure the hardware interrupt pin.
     * @return GPIO error code
     */
    hf_gpio_err_t ConfigureHardwareInterrupt() noexcept;

    /**
     * @brief Static hardware interrupt callback (ISR context).
     * @param gpio GPIO pin that triggered the interrupt
     * @param trigger Trigger type that caused the interrupt
     * @param user_data Pointer to Pcal95555Handler instance
     */
    static void HardwareInterruptCallback(BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) noexcept;

    /**
     * @brief Process pending interrupts by reading status and dispatching callbacks.
     * Called from hardware interrupt handler.
     */
    void ProcessInterrupts() noexcept;
    
    /**
     * @brief Dump comprehensive diagnostics and statistics to log as INFO level.
     * Logs PCAL95555 GPIO expander status, pin configurations, and communication health.
     */
    void DumpDiagnostics() const noexcept;

    /**
     * @brief Architecture Note: Address Handling
     * 
     * This design implements device-centric I2C communication:
     * 
     * 1. **BaseI2c Device**: Pre-configured with device address, no address params needed
     * 2. **PCAL95555 Driver**: Stores address internally for validation and state management  
     * 3. **Pcal95555I2cAdapter**: Validates address consistency between the two layers
     * 
     * The adapter serves as a bridge between:
     * - PCAL95555 driver API (expects address parameters)  
     * - BaseI2c device API (address pre-configured, no parameters)
     * 
     * This provides both type safety (can't use wrong device) and compatibility
     * with existing driver interfaces without requiring modification.
     */
};

#endif // COMPONENT_HANDLER_PCAL95555_HANDLER_H_ 