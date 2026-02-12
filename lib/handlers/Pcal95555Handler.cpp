/**
 * @file Pcal95555Handler.cpp
 * @brief Implementation of the PCAL95555 GPIO expander handler.
 *
 * @details
 * Implements all three layers defined in Pcal95555Handler.h:
 * 1. HalI2cPcal95555Comm -- CRTP I2C communication adapter
 * 2. Pcal95555Handler    -- Main handler (init, GPIO ops, interrupts, factory)
 * 3. Pcal95555GpioPin    -- Per-pin BaseGpio wrapper
 *
 * All driver calls use the PascalCase API of the updated hf-pcal95555-driver
 * (pcal95555::PCAL95555<I2cType>). Error handling follows the driver's error-flag
 * model: individual methods return bool, with accumulated error flags available
 * via GetErrorFlags().
 *
 * @see Pcal95555Handler.h  for architectural overview and Doxygen documentation.
 *
 * @author HardFOC Team
 * @date 2025
 */

#include "Pcal95555Handler.h"
#include "handlers/Logger.h"
#include <cstring>

// =====================================================================
// HalI2cPcal95555Comm Implementation
// =====================================================================

/// @brief Construct the CRTP I2C adapter.
HalI2cPcal95555Comm::HalI2cPcal95555Comm(BaseI2c& i2c_device) noexcept
    : i2c_device_(i2c_device) {}

bool HalI2cPcal95555Comm::Write(uint8_t addr, uint8_t reg,
                                const uint8_t* data, size_t len) noexcept {
    MutexLockGuard lock(i2c_mutex_);

    // Validate that the driver's address matches the BaseI2c device address.
    if (addr != i2c_device_.GetDeviceAddress()) {
        return false;
    }

    // Frame the I2C register write: [register, data...]
    // PCAL9555 register writes are at most 2 data bytes; 4-byte buffer is sufficient.
    constexpr size_t kMaxBuf = 4;
    if (len + 1 > kMaxBuf) {
        return false;  // Unexpected oversized write for this device.
    }

    uint8_t command[kMaxBuf];
    command[0] = reg;
    std::memcpy(&command[1], data, len);
    return i2c_device_.Write(command, len + 1) == hf_i2c_err_t::I2C_SUCCESS;
}

bool HalI2cPcal95555Comm::Read(uint8_t addr, uint8_t reg,
                               uint8_t* data, size_t len) noexcept {
    MutexLockGuard lock(i2c_mutex_);

    if (addr != i2c_device_.GetDeviceAddress()) {
        return false;
    }

    return i2c_device_.WriteRead(&reg, 1, data, len) == hf_i2c_err_t::I2C_SUCCESS;
}

bool HalI2cPcal95555Comm::EnsureInitialized() noexcept {
    // BaseI2c device is expected to be initialized before the handler uses it.
    return true;
}

bool HalI2cPcal95555Comm::RegisterInterruptHandler(
    std::function<void()> handler) noexcept {
    interrupt_handler_ = std::move(handler);
    return true;  // Actual GPIO interrupt setup is done by the handler.
}

// =====================================================================
// Pcal95555Handler -- Construction & Lifecycle
// =====================================================================

Pcal95555Handler::Pcal95555Handler(BaseI2c& i2c_device,
                                   BaseGpio* interrupt_pin) noexcept
    : i2c_device_(i2c_device),
      i2c_adapter_(nullptr),
      pcal95555_driver_(nullptr),
      initialized_(false),
      interrupt_pin_(interrupt_pin),
      interrupt_configured_(false) {
    pin_registry_.fill(nullptr);
    pull_mode_cache_.fill(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING);
}

bool Pcal95555Handler::EnsureInitialized() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (initialized_) {
        return true;
    }
    return Initialize() == hf_gpio_err_t::GPIO_SUCCESS;
}

bool Pcal95555Handler::EnsureDeinitialized() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) {
        return true;
    }
    return Deinitialize() == hf_gpio_err_t::GPIO_SUCCESS;
}

/// @brief Check initialization under an already-held handler_mutex_.
inline bool Pcal95555Handler::EnsureInitializedLocked() noexcept {
    if (initialized_) return true;
    return Initialize() == hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::Initialize() noexcept {
    // Note: caller must hold handler_mutex_.
    if (initialized_) {
        return hf_gpio_err_t::GPIO_SUCCESS;
    }

    // 1. Create the CRTP I2C adapter.
    if (!i2c_adapter_) {
        i2c_adapter_ = std::make_unique<HalI2cPcal95555Comm>(i2c_device_);
        if (!i2c_adapter_) {
            return hf_gpio_err_t::GPIO_ERR_OUT_OF_MEMORY;
        }
    }

    // 2. Create the typed PCAL95555 driver (address-based constructor).
    if (!pcal95555_driver_) {
        pcal95555_driver_ = std::make_unique<Pcal95555Driver>(
            i2c_adapter_.get(), i2c_device_.GetDeviceAddress());
        if (!pcal95555_driver_) {
            return hf_gpio_err_t::GPIO_ERR_OUT_OF_MEMORY;
        }
    }

    // 3. Initialize the driver (lazy init, auto-detects chip variant).
    if (!pcal95555_driver_->EnsureInitialized()) {
        return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    }

    // 4. Register the driver's interrupt handler with the I2C adapter
    //    so that HandleInterrupt() can be triggered by the adapter.
    pcal95555_driver_->RegisterInterruptHandler();

    // 5. Configure hardware interrupt pin if available.
    if (interrupt_pin_ != nullptr) {
        auto result = ConfigureHardwareInterrupt();
        if (result != hf_gpio_err_t::GPIO_SUCCESS) {
            // Non-fatal: polling mode still works.
        }
    }

    // Seed previous input state for edge detection on first interrupt.
    prev_input_state_ = pcal95555_driver_->ReadAllInputs();

    // Seed pull_mode_cache_ from hardware registers via driver API (PCAL9555A only).
    if (pcal95555_driver_->HasAgileIO()) {
        uint16_t enable_mask = 0;
        uint16_t direction_mask = 0;

        if (pcal95555_driver_->GetPullConfiguration(enable_mask, direction_mask)) {
            for (uint8_t pin = 0; pin < 16; ++pin) {
                bool enabled = (enable_mask >> pin) & 1U;
                bool is_up   = (direction_mask >> pin) & 1U;

                if (!enabled) {
                    pull_mode_cache_[pin] = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
                } else if (is_up) {
                    pull_mode_cache_[pin] = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_PULL_UP;
                } else {
                    pull_mode_cache_[pin] = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_PULL_DOWN;
                }
            }
        }
        // If read fails, cache stays at default (FLOATING) -- non-fatal.
    }

    initialized_ = true;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::Deinitialize() noexcept {
    // Note: caller must hold handler_mutex_.
    if (!initialized_) {
        return hf_gpio_err_t::GPIO_SUCCESS;
    }

    // Disable hardware interrupt.
    if (interrupt_configured_ && interrupt_pin_) {
        interrupt_pin_->ConfigureInterrupt(
            hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_NONE);
        interrupt_configured_ = false;
    }

    // Clear pin registry (handler_mutex_ already held by caller).
    pin_registry_.fill(nullptr);

    // Release driver and adapter.
    pcal95555_driver_.reset();
    i2c_adapter_.reset();
    initialized_ = false;
    pull_mode_cache_.fill(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING);
    return hf_gpio_err_t::GPIO_SUCCESS;
}

// =====================================================================
// Pcal95555Handler -- Basic GPIO Operations
// =====================================================================

hf_gpio_err_t Pcal95555Handler::SetDirection(uint8_t pin,
                                             hf_gpio_direction_t direction) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    GPIODir dir = (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT)
                      ? GPIODir::Output
                      : GPIODir::Input;
    return pcal95555_driver_->SetPinDirection(pin, dir)
               ? hf_gpio_err_t::GPIO_SUCCESS
               : hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetOutput(uint8_t pin, bool active) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    return pcal95555_driver_->WritePin(pin, active)
               ? hf_gpio_err_t::GPIO_SUCCESS
               : hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::ReadInput(uint8_t pin, bool& active) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    active = pcal95555_driver_->ReadPin(pin);

    // ReadPin returns the level directly; check error flags for I2C failures.
    uint16_t error_flags = pcal95555_driver_->GetErrorFlags();
    if (error_flags != 0) {
        return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::Toggle(uint8_t pin) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    return pcal95555_driver_->TogglePin(pin)
               ? hf_gpio_err_t::GPIO_SUCCESS
               : hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetPullMode(uint8_t pin,
                                            hf_gpio_pull_mode_t pull_mode) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    // Pull resistors require PCAL9555A (Agile I/O).
    if (!pcal95555_driver_->HasAgileIO()) {
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    }

    bool success = true;
    switch (pull_mode) {
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
            success = pcal95555_driver_->SetPullEnable(pin, false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
            success = pcal95555_driver_->SetPullEnable(pin, true) &&
                      pcal95555_driver_->SetPullDirection(pin, true);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
            success = pcal95555_driver_->SetPullEnable(pin, true) &&
                      pcal95555_driver_->SetPullDirection(pin, false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
            // Not directly supported by hardware -- default to pull-up.
            success = pcal95555_driver_->SetPullEnable(pin, true) &&
                      pcal95555_driver_->SetPullDirection(pin, true);
            break;
        default:
            return hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER;
    }

    if (success) {
        pull_mode_cache_[pin] = pull_mode;
    }
    return success ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::GetPullMode(uint8_t pin,
                                            hf_gpio_pull_mode_t& pull_mode) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    pull_mode = pull_mode_cache_[pin];
    return hf_gpio_err_t::GPIO_SUCCESS;
}

// =====================================================================
// Pcal95555Handler -- Batch GPIO Operations
// =====================================================================

hf_gpio_err_t Pcal95555Handler::SetDirections(uint16_t pin_mask,
                                              hf_gpio_direction_t direction) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    GPIODir dir = (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT)
                      ? GPIODir::Input
                      : GPIODir::Output;
    return pcal95555_driver_->SetMultipleDirections(pin_mask, dir)
               ? hf_gpio_err_t::GPIO_SUCCESS
               : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetOutputs(uint16_t pin_mask, bool active) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    if (pin_mask == 0) return hf_gpio_err_t::GPIO_SUCCESS;

    // Use the driver's batch mask-based output write (read-modify-write both
    // output port registers in one operation, instead of per-pin WritePin calls).
    if (!pcal95555_driver_->SetMultipleOutputs(pin_mask, active)) {
        return hf_gpio_err_t::GPIO_ERR_FAILURE;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::SetPullModes(uint16_t pin_mask,
                                             hf_gpio_pull_mode_t pull_mode) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    if (!pcal95555_driver_->HasAgileIO()) {
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    }

    // Delegate to the single-pin pull mode logic for each selected pin.
    // SetPullMode() would re-lock handler_mutex_, so we inline the core logic here.
    bool ok = true;
    for (uint8_t pin = 0; pin < 16; ++pin) {
        if (!(pin_mask & (1U << pin))) continue;

        bool pin_ok = true;
        switch (pull_mode) {
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
                pin_ok = pcal95555_driver_->SetPullEnable(pin, false);
                break;
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
                pin_ok = pcal95555_driver_->SetPullEnable(pin, true) &&
                         pcal95555_driver_->SetPullDirection(pin, true);
                break;
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
                pin_ok = pcal95555_driver_->SetPullEnable(pin, true) &&
                         pcal95555_driver_->SetPullDirection(pin, false);
                break;
            default:
                pin_ok = false;
                break;
        }

        if (pin_ok) {
            pull_mode_cache_[pin] = pull_mode;
        }
        ok &= pin_ok;
    }
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

// =====================================================================
// Pcal95555Handler -- Interrupt Management
// =====================================================================

hf_gpio_err_t Pcal95555Handler::GetAllInterruptMasks(uint16_t& mask) noexcept {
    if (!EnsureInitialized()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    MutexLockGuard lock(handler_mutex_);

    if (!pcal95555_driver_->HasAgileIO()) {
        mask = 0xFFFF;  // All masked on PCA9555.
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    }

    // The driver doesn't expose a GetInterruptMask() getter.
    // Default: report all masked. Handler-level callers should track
    // which pins have been enabled via ConfigureInterrupt().
    mask = 0xFFFF;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::GetAllInterruptStatus(uint16_t& status) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    status = pcal95555_driver_->GetInterruptStatus();

    uint16_t error_flags = pcal95555_driver_->GetErrorFlags();
    if (error_flags != 0) {
        return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::RegisterPinInterrupt(
    hf_pin_num_t pin,
    hf_gpio_interrupt_trigger_t trigger,
    InterruptCallback callback,
    void* user_data) noexcept {

    if (pin >= 16) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;

    MutexLockGuard lock(handler_mutex_);

    auto& gpio_pin = pin_registry_[pin];
    if (!gpio_pin) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;

    // Store interrupt data in the pin object.
    gpio_pin->interrupt_callback_ = callback;
    gpio_pin->interrupt_user_data_ = user_data;
    gpio_pin->interrupt_trigger_ = trigger;
    gpio_pin->interrupt_enabled_ = true;

    // Set up hardware interrupt on first use.
    if (interrupt_pin_ && !interrupt_configured_) {
        auto result = ConfigureHardwareInterrupt();
        if (result != hf_gpio_err_t::GPIO_SUCCESS) {
            // Rollback.
            gpio_pin->interrupt_callback_ = nullptr;
            gpio_pin->interrupt_user_data_ = nullptr;
            gpio_pin->interrupt_enabled_ = false;
            return result;
        }
    }

    // Enable interrupt for this pin in the driver (unmask it).
    if (pcal95555_driver_) {
        pcal95555_driver_->ConfigureInterrupt(pin, InterruptState::Enabled);
    }

    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::UnregisterPinInterrupt(hf_pin_num_t pin) noexcept {
    if (pin >= 16) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;

    MutexLockGuard lock(handler_mutex_);

    auto& gpio_pin = pin_registry_[pin];
    if (!gpio_pin) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;

    // Clear interrupt data.
    gpio_pin->interrupt_callback_ = nullptr;
    gpio_pin->interrupt_user_data_ = nullptr;
    gpio_pin->interrupt_trigger_ =
        hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_NONE;
    gpio_pin->interrupt_enabled_ = false;

    // Mask interrupt for this pin in the driver.
    if (pcal95555_driver_) {
        pcal95555_driver_->ConfigureInterrupt(pin, InterruptState::Disabled);
    }

    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::ConfigureHardwareInterrupt() noexcept {
    if (!interrupt_pin_) return hf_gpio_err_t::GPIO_ERR_NULL_POINTER;

    // PCAL95555 INT output is active-low, open-drain -- trigger on falling edge.
    auto result = interrupt_pin_->ConfigureInterrupt(
        hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE,
        HardwareInterruptCallback,
        this);

    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        interrupt_configured_ = true;
    }
    return result;
}

void Pcal95555Handler::HardwareInterruptCallback(
    BaseGpio* /*gpio*/,
    hf_gpio_interrupt_trigger_t /*trigger*/,
    void* user_data) noexcept {
    // Called in ISR context -- keep minimal.
    auto* handler = static_cast<Pcal95555Handler*>(user_data);
    if (handler) {
        handler->ProcessInterrupts();
    }
}

void Pcal95555Handler::ProcessInterrupts() noexcept {
    if (!pcal95555_driver_) return;

    // Read interrupt status (this clears the interrupt condition on the chip).
    uint16_t status = pcal95555_driver_->GetInterruptStatus();
    if (status == 0) return;

    // Read current pin input levels for edge detection.
    uint16_t current_state = pcal95555_driver_->ReadAllInputs();

    // Determine which pins transitioned high (rising) and low (falling).
    uint16_t rising  = current_state & ~prev_input_state_;  // was 0, now 1
    uint16_t falling = ~current_state & prev_input_state_;  // was 1, now 0

    // Update stored state for next interrupt.
    prev_input_state_ = current_state;

    // Dispatch to per-pin callbacks, filtering by requested trigger type.
    for (int pin = 0; pin < 16; ++pin) {
        if (!(status & (1U << pin))) continue;

        auto& gpio_pin = pin_registry_[pin];
        if (!gpio_pin || !gpio_pin->interrupt_enabled_ ||
            !gpio_pin->interrupt_callback_) {
            continue;
        }

        const uint16_t mask = static_cast<uint16_t>(1U << pin);
        const auto trigger = gpio_pin->interrupt_trigger_;
        bool fire = false;

        switch (trigger) {
            case hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE:
                fire = (rising & mask) != 0;
                break;
            case hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE:
                fire = (falling & mask) != 0;
                break;
            case hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_BOTH_EDGES:
                fire = ((rising | falling) & mask) != 0;
                break;
            default:
                break;  // NONE or unknown -- skip
        }

        if (fire) {
            // Report the actual observed trigger, not just the configured one.
            hf_gpio_interrupt_trigger_t actual =
                (rising & mask)
                    ? hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE
                    : hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE;
            gpio_pin->interrupt_callback_(
                gpio_pin.get(), actual, gpio_pin->interrupt_user_data_);
        }
    }
}

// =====================================================================
// Pcal95555Handler -- Pin Factory
// =====================================================================

uint8_t Pcal95555Handler::GetI2cAddress() const noexcept {
    return pcal95555_driver_ ? pcal95555_driver_->GetAddress() : 0;
}

std::shared_ptr<BaseGpio> Pcal95555Handler::CreateGpioPin(
    hf_pin_num_t pin,
    hf_gpio_direction_t direction,
    hf_gpio_active_state_t active_state,
    hf_gpio_output_mode_t output_mode,
    hf_gpio_pull_mode_t pull_mode,
    bool allow_existing) noexcept {

    if (pin >= 16) return nullptr;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return nullptr;

    // Check if pin already exists.
    if (pin_registry_[pin] != nullptr) {
        return allow_existing
                   ? std::static_pointer_cast<BaseGpio>(pin_registry_[pin])
                   : nullptr;
    }

    // Create a new Pcal95555GpioPin with handler reference (not driver pointer).
    auto new_pin = std::make_shared<Pcal95555GpioPin>(
        pin, this, direction, active_state, output_mode, pull_mode);

    if (new_pin && new_pin->Initialize()) {
        pin_registry_[pin] = new_pin;
        return new_pin;
    }

    return nullptr;
}

std::shared_ptr<BaseGpio> Pcal95555Handler::GetGpioPin(hf_pin_num_t pin) noexcept {
    if (pin >= 16) return nullptr;
    MutexLockGuard lock(handler_mutex_);
    return pin_registry_[pin];
}

bool Pcal95555Handler::IsPinCreated(hf_pin_num_t pin) const noexcept {
    if (pin >= 16) return false;
    MutexLockGuard lock(handler_mutex_);
    return pin_registry_[pin] != nullptr;
}

uint16_t Pcal95555Handler::GetCreatedPinMask() const noexcept {
    MutexLockGuard lock(handler_mutex_);
    uint16_t mask = 0;
    for (hf_pin_num_t i = 0; i < 16; ++i) {
        if (pin_registry_[i] != nullptr) {
            mask |= (1U << i);
        }
    }
    return mask;
}

// =====================================================================
// Pcal95555Handler -- PCAL9555A Advanced Features (Agile I/O)
// =====================================================================

bool Pcal95555Handler::HasAgileIO() const noexcept {
    return pcal95555_driver_ && pcal95555_driver_->HasAgileIO();
}

pcal95555::ChipVariant Pcal95555Handler::GetChipVariant() const noexcept {
    return pcal95555_driver_
               ? pcal95555_driver_->GetChipVariant()
               : pcal95555::ChipVariant::Unknown;
}

bool Pcal95555Handler::SetPolarityInversion(hf_pin_num_t pin, bool invert) noexcept {
    if (!ValidatePin(static_cast<uint8_t>(pin))) return false;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return false;

    Polarity pol = invert ? Polarity::Inverted : Polarity::Normal;
    return pcal95555_driver_->SetPinPolarity(pin, pol);
}

bool Pcal95555Handler::SetInterruptMask(hf_pin_num_t pin, bool mask) noexcept {
    if (!ValidatePin(static_cast<uint8_t>(pin))) return false;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return false;

    InterruptState state = mask ? InterruptState::Disabled : InterruptState::Enabled;
    return pcal95555_driver_->ConfigureInterrupt(pin, state);
}

bool Pcal95555Handler::GetInterruptStatus(hf_pin_num_t pin, bool& status) noexcept {
    if (!ValidatePin(static_cast<uint8_t>(pin))) return false;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return false;

    uint16_t global_status = pcal95555_driver_->GetInterruptStatus();
    status = (global_status & (1U << pin)) != 0;
    return true;
}

bool Pcal95555Handler::SetDriveStrength(hf_pin_num_t pin,
                                        DriveStrength level) noexcept {
    if (!ValidatePin(static_cast<uint8_t>(pin))) return false;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return false;

    return pcal95555_driver_->SetDriveStrength(pin, level);
}

bool Pcal95555Handler::EnableInputLatch(hf_pin_num_t pin, bool enable) noexcept {
    if (!ValidatePin(static_cast<uint8_t>(pin))) return false;
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return false;

    return pcal95555_driver_->EnableInputLatch(pin, enable);
}

bool Pcal95555Handler::SetOutputMode(bool port0_open_drain,
                                     bool port1_open_drain) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return false;

    return pcal95555_driver_->SetOutputMode(port0_open_drain, port1_open_drain);
}

bool Pcal95555Handler::ResetToDefault() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!EnsureInitializedLocked()) return false;

    pcal95555_driver_->ResetToDefault();  // void return
    pull_mode_cache_.fill(hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING);
    return true;
}

// =====================================================================
// Pcal95555Handler -- Error Management
// =====================================================================

uint16_t Pcal95555Handler::GetErrorFlags() const noexcept {
    return pcal95555_driver_ ? pcal95555_driver_->GetErrorFlags() : 0;
}

void Pcal95555Handler::ClearErrorFlags(uint16_t mask) noexcept {
    if (pcal95555_driver_) {
        pcal95555_driver_->ClearErrorFlags(mask);
    }
}

// =====================================================================
// Pcal95555Handler -- Diagnostics
// =====================================================================

void Pcal95555Handler::DumpDiagnostics() const noexcept {
    static constexpr const char* TAG = "Pcal95555Handler";

    Logger::GetInstance().Info(TAG, "=== PCAL95555 HANDLER DIAGNOSTICS ===");

    MutexLockGuard lock(handler_mutex_);

    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s",
                              initialized_ ? "YES" : "NO");

    // I2C Interface
    Logger::GetInstance().Info(TAG, "I2C Interface:");
    if (i2c_adapter_) {
        Logger::GetInstance().Info(TAG, "  I2C Adapter: ACTIVE (CRTP-based)");
        Logger::GetInstance().Info(TAG, "  Device Address: 0x%02X",
                                  i2c_device_.GetDeviceAddress());
    } else {
        Logger::GetInstance().Info(TAG, "  I2C Adapter: NOT_INITIALIZED");
    }

    // Driver Status
    Logger::GetInstance().Info(TAG, "PCAL95555 Driver:");
    if (pcal95555_driver_) {
        const char* variant_str = "Unknown";
        auto variant = pcal95555_driver_->GetChipVariant();
        if (variant == pcal95555::ChipVariant::PCAL9555A) {
            variant_str = "PCAL9555A (Agile I/O)";
        } else if (variant == pcal95555::ChipVariant::PCA9555) {
            variant_str = "PCA9555 (Standard)";
        }

        Logger::GetInstance().Info(TAG, "  Driver Instance: ACTIVE");
        Logger::GetInstance().Info(TAG, "  Chip Variant: %s", variant_str);
        Logger::GetInstance().Info(TAG, "  I2C Address: 0x%02X",
                                  pcal95555_driver_->GetAddress());
        Logger::GetInstance().Info(TAG, "  Error Flags: 0x%04X",
                                  pcal95555_driver_->GetErrorFlags());
    } else {
        Logger::GetInstance().Info(TAG, "  Driver Instance: NOT_INITIALIZED");
    }

    // Pin Registry Status
    Logger::GetInstance().Info(TAG, "Pin Registry:");
    int active_pins = 0;
    int interrupt_pins = 0;
    for (size_t i = 0; i < pin_registry_.size(); ++i) {
        if (pin_registry_[i] != nullptr) {
            active_pins++;
            if (pin_registry_[i]->interrupt_enabled_) {
                interrupt_pins++;
            }
        }
    }
    Logger::GetInstance().Info(TAG, "  Active Pin Objects: %d/16", active_pins);
    Logger::GetInstance().Info(TAG, "  Pins with Interrupts: %d", interrupt_pins);

    // Interrupt Configuration
    Logger::GetInstance().Info(TAG, "Interrupt Configuration:");
    Logger::GetInstance().Info(TAG, "  Hardware Interrupt Pin: %s",
                              interrupt_pin_ ? "CONFIGURED" : "NOT_CONFIGURED");
    Logger::GetInstance().Info(TAG, "  Interrupt System: %s",
                              interrupt_configured_ ? "ENABLED" : "DISABLED");

    // Active Pin Details
    Logger::GetInstance().Info(TAG, "Active Pin Details:");
    int shown = 0;
    for (size_t i = 0; i < pin_registry_.size(); ++i) {
        if (!pin_registry_[i]) continue;
        ++shown;

        const char* trigger_str = "NONE";
        auto t = pin_registry_[i]->interrupt_trigger_;
        if (t == hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE) {
            trigger_str = "RISING";
        } else if (t == hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE) {
            trigger_str = "FALLING";
        } else if (t == hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_BOTH_EDGES) {
            trigger_str = "BOTH";
        }

        Logger::GetInstance().Info(
            TAG, "  Pin %d: Int=%s Trigger=%s",
            static_cast<int>(i),
            pin_registry_[i]->interrupt_enabled_ ? "ON" : "OFF",
            trigger_str);
    }
    if (shown == 0) {
        Logger::GetInstance().Info(TAG, "  No active pins");
    }

    // Overall Status
    bool healthy = initialized_ && pcal95555_driver_ && i2c_adapter_;
    Logger::GetInstance().Info(TAG, "System Status: %s",
                              healthy ? "HEALTHY" : "DEGRADED");

    Logger::GetInstance().Info(TAG, "=== END PCAL95555 HANDLER DIAGNOSTICS ===");
}

// =====================================================================
// Pcal95555GpioPin Implementation
// =====================================================================

Pcal95555GpioPin::Pcal95555GpioPin(
    hf_pin_num_t pin,
    Pcal95555Handler* parent_handler,
    hf_gpio_direction_t direction,
    hf_gpio_active_state_t active_state,
    hf_gpio_output_mode_t output_mode,
    hf_gpio_pull_mode_t pull_mode) noexcept
    : BaseGpio(pin, direction, active_state, output_mode, pull_mode),
      pin_(pin),
      parent_handler_(parent_handler) {
    snprintf(description_, sizeof(description_), "PCAL95555_PIN_%d",
             static_cast<int>(pin_));
}

bool Pcal95555GpioPin::Initialize() noexcept {
    if (!parent_handler_) return false;

    // Configure direction via the handler (which routes to the driver).
    auto dir_result = parent_handler_->SetDirection(
        static_cast<uint8_t>(pin_), current_direction_);
    if (dir_result != hf_gpio_err_t::GPIO_SUCCESS) {
        return false;
    }

    // Configure pull mode via handler (only for PCAL9555A; non-fatal on PCA9555).
    if (parent_handler_->HasAgileIO() &&
        pull_mode_ != hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING) {
        parent_handler_->SetPullMode(static_cast<uint8_t>(pin_), pull_mode_);
        // Pull mode failure is non-fatal.
    }

    initialized_ = true;
    return true;
}

bool Pcal95555GpioPin::Deinitialize() noexcept {
    initialized_ = false;
    return true;
}

bool Pcal95555GpioPin::IsPinAvailable() const noexcept {
    return parent_handler_ && pin_ < 16;
}

const char* Pcal95555GpioPin::GetDescription() const noexcept {
    return description_;
}

hf_gpio_err_t Pcal95555GpioPin::ConfigureInterrupt(
    hf_gpio_interrupt_trigger_t trigger,
    InterruptCallback callback,
    void* user_data) noexcept {
    if (!parent_handler_) return hf_gpio_err_t::GPIO_ERR_NULL_POINTER;
    return parent_handler_->RegisterPinInterrupt(pin_, trigger, callback, user_data);
}

// --- PCAL9555A Advanced Features ---

hf_gpio_err_t Pcal95555GpioPin::SetPolarityInversion(bool invert) noexcept {
    if (!parent_handler_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    return parent_handler_->SetPolarityInversion(pin_, invert)
               ? hf_gpio_err_t::GPIO_SUCCESS
               : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetInterruptMask(bool mask) noexcept {
    if (!parent_handler_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    return parent_handler_->SetInterruptMask(pin_, mask)
               ? hf_gpio_err_t::GPIO_SUCCESS
               : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetInterruptStatus(bool& status) noexcept {
    if (!parent_handler_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    return parent_handler_->GetInterruptStatus(pin_, status)
               ? hf_gpio_err_t::GPIO_SUCCESS
               : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

// --- BaseGpio Protected Implementation ---

hf_gpio_err_t Pcal95555GpioPin::SetDirectionImpl(
    hf_gpio_direction_t direction) noexcept {
    if (!parent_handler_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    return parent_handler_->SetDirection(static_cast<uint8_t>(pin_), direction);
}

hf_gpio_err_t Pcal95555GpioPin::SetOutputModeImpl(
    hf_gpio_output_mode_t /*mode*/) noexcept {
    // PCAL9555A only supports output mode at per-port granularity (pins 0-7
    // share one mode, pins 8-15 share another). Per-pin changes are not
    // supported; use Pcal95555Handler::SetOutputMode() for port-level control.
    return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
}

hf_gpio_err_t Pcal95555GpioPin::SetPullModeImpl(
    hf_gpio_pull_mode_t mode) noexcept {
    if (!parent_handler_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    return parent_handler_->SetPullMode(static_cast<uint8_t>(pin_), mode);
}

hf_gpio_pull_mode_t Pcal95555GpioPin::GetPullModeImpl() const noexcept {
    if (!parent_handler_) {
        return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
    }
    hf_gpio_pull_mode_t mode;
    if (parent_handler_->GetPullMode(static_cast<uint8_t>(pin_), mode) ==
        hf_gpio_err_t::GPIO_SUCCESS) {
        return mode;
    }
    return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
}

hf_gpio_err_t Pcal95555GpioPin::SetPinLevelImpl(
    hf_gpio_level_t level) noexcept {
    if (!parent_handler_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool hw_level = (level == hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
    return parent_handler_->SetOutput(static_cast<uint8_t>(pin_), hw_level);
}

hf_gpio_err_t Pcal95555GpioPin::GetPinLevelImpl(
    hf_gpio_level_t& level) noexcept {
    if (!parent_handler_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool active = false;
    auto result = parent_handler_->ReadInput(static_cast<uint8_t>(pin_), active);
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        level = active ? hf_gpio_level_t::HF_GPIO_LEVEL_HIGH
                       : hf_gpio_level_t::HF_GPIO_LEVEL_LOW;
    }
    return result;
}

hf_gpio_err_t Pcal95555GpioPin::GetDirectionImpl(
    hf_gpio_direction_t& direction) const noexcept {
    // Direction is tracked in the BaseGpio base class after SetDirection().
    direction = current_direction_;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555GpioPin::GetOutputModeImpl(
    hf_gpio_output_mode_t& mode) const noexcept {
    // Output mode is tracked in the BaseGpio base class.
    mode = output_mode_;
    return hf_gpio_err_t::GPIO_SUCCESS;
}
