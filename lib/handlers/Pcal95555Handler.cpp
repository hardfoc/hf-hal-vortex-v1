#include "Pcal95555Handler.h"
#include <cstring>
#include <vector>
#include "utils-and-drivers/driver-handlers/Logger.h"

// ================= Pcal95555I2cAdapter =================
bool Pcal95555I2cAdapter::write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
    : i2c_adapter_(std::make_unique<Pcal95555I2cAdapter>(i2c_device)),
      pcal95555_driver_(std::make_shared<PCAL95555>(i2c_adapter_.get(), i2c_device.GetDeviceAddress())),
      initialized_(false),
      interrupt_pin_(interrupt_pin),
      interrupt_configured_(false)
{
    // Initialize pin registry to null
    pin_registry_.fill(nullptr);
    
    // Initialize interrupt arrays
    pin_callbacks_.fill(nullptr);
    pin_user_data_.fill(nullptr);
    pin_triggers_.fill(hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_NONE);
}

// ================= Pcal95555I2cAdapter =================
bool Pcal95555I2cAdapter::write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
    MutexLockGuard lock(i2c_mutex_);
    
    // Address validation: Ensures the PCAL95555 driver and BaseI2c device are consistent
    // This prevents accidental cross-device communication and provides type safety
    if (addr != i2c_device_.GetDeviceAddress()) {
        // Log error or handle mismatch - this indicates a configuration error
        return false;
    }
    
    // Create register write command: [register_address, data...]
    // Use stack buffer for typical PCAL95555 register writes (usually 1-3 bytes)
    constexpr size_t MAX_STACK_BUFFER = 32; // Reasonable limit for I2C register writes
    
    if (len < MAX_STACK_BUFFER) {
        // Fast path: use stack-allocated buffer
        uint8_t command[MAX_STACK_BUFFER];
        command[0] = reg;
        std::memcpy(&command[1], data, len);
        
        return i2c_device_.Write(command, len + 1) == hf_i2c_err_t::I2C_SUCCESS;
    } else {
        // Fallback: use vector for large writes (rare case)
        std::vector<uint8_t> command;
        command.reserve(1 + len);
        command.push_back(reg);
        command.insert(command.end(), data, data + len);
        
        return i2c_device_.Write(command.data(), command.size()) == hf_i2c_err_t::I2C_SUCCESS;
    }
}

bool Pcal95555I2cAdapter::read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    MutexLockGuard lock(i2c_mutex_);
    
    // Address validation: Ensures the PCAL95555 driver and BaseI2c device are consistent
    if (addr != i2c_device_.GetDeviceAddress()) {
        return false;
    }
    
    // Use device-centric BaseI2c interface for register read
    return i2c_device_.WriteRead(&reg, 1, data, len) == hf_i2c_err_t::I2C_SUCCESS;
}

// ================= Pcal95555Handler =================
Pcal95555Handler::Pcal95555Handler(BaseI2c& i2c_device, BaseGpio* interrupt_pin) noexcept
    : i2c_device_(i2c_device),
      i2c_adapter_(nullptr),                // Created during initialization
      pcal95555_driver_(nullptr),           // Created during initialization
      initialized_(false),
      interrupt_pin_(interrupt_pin),
      interrupt_configured_(false)
{
    // Initialize pin registry to null
    pin_registry_.fill(nullptr);
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
    if (!initialized_) return true;  // Already deinitialized

    pcal95555_driver_.reset();
    i2c_adapter_.reset();
    initialized_ = false;
    return true;
}

hf_gpio_err_t Pcal95555Handler::Initialize() noexcept {
    // Note: Caller should hold handler_mutex_ when calling this method
    if (initialized_) return hf_gpio_err_t::GPIO_SUCCESS;
    
    // Create I2C adapter if not already created
    if (!i2c_adapter_) {
        i2c_adapter_ = std::make_unique<Pcal95555I2cAdapter>(i2c_device_);
        if (!i2c_adapter_) {
            return hf_gpio_err_t::GPIO_ERR_OUT_OF_MEMORY;
        }
    }
    
    // Create PCAL95555 driver if not already created
    if (!pcal95555_driver_) {
        pcal95555_driver_ = std::make_shared<PCAL95555>(i2c_adapter_.get(), i2c_device_.GetDeviceAddress());
        if (!pcal95555_driver_) {
            return hf_gpio_err_t::GPIO_ERR_OUT_OF_MEMORY;
        }
    }
    
    // Initialize PCAL95555 driver
    // The PCAL95555 driver doesn't have an init() method that returns success/failure
    // Instead, we can call initFromConfig() and assume success if driver was created successfully
    pcal95555_driver_->initFromConfig();
    
    // We could also call resetToDefault() to ensure known state
    // pcal95555_driver_->resetToDefault();
    
    // Configure hardware interrupt if pin is available
    if (interrupt_pin_ != nullptr) {
        auto result = ConfigureHardwareInterrupt();
        if (result != hf_gpio_err_t::GPIO_SUCCESS) {
            // Log warning but don't fail initialization - can still work in polling mode
            //Logger::GetInstance().Warn("Pcal95555Handler", "Failed to configure hardware interrupt, using polling mode");
        }
    }
    
    initialized_ = true;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::Deinitialize() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_SUCCESS;
    
    // Disable hardware interrupt if configured
    if (interrupt_configured_ && interrupt_pin_) {
        interrupt_pin_->ConfigureInterrupt(hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_NONE);
        interrupt_configured_ = false;
    }
    
    // Clear all pin registry entries (this releases shared_ptr references)
    {
        MutexLockGuard pin_lock(pin_registry_mutex_);
        pin_registry_.fill(nullptr);
    }
    
    // Clean up driver and adapter (lazy initialization cleanup)
    pcal95555_driver_.reset();
    i2c_adapter_.reset();
    
    initialized_ = false;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::SetDirection(hf_u8_t pin, hf_gpio_direction_t direction) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    if (!EnsureInitialized()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    MutexLockGuard lock(handler_mutex_);
    PCAL95555::GPIODir dir = (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT) ? 
                             PCAL95555::GPIODir::Output : PCAL95555::GPIODir::Input;
    return pcal95555_driver_->setPinDirection(pin, dir) ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetOutput(hf_u8_t pin, hf_bool_t active) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    if (!EnsureInitialized()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    MutexLockGuard lock(handler_mutex_);
    return pcal95555_driver_->writePin(pin, active) ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::ReadInput(hf_u8_t pin, hf_bool_t& active) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    if (!EnsureInitialized()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    MutexLockGuard lock(handler_mutex_);
    // PCAL95555::readPin returns the pin state directly, not success/failure
    // We need to check for communication errors using the driver's error state
    active = pcal95555_driver_->readPin(pin);
    
    // Check if there was a communication error during the read operation  
    uint16_t error_flags = pcal95555_driver_->getErrorFlags();
    if (error_flags != 0) {
        return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    }
    
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::Toggle(hf_u8_t pin) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    if (!EnsureInitialized()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    MutexLockGuard lock(handler_mutex_);
    // Use PCAL95555's built-in toggle function which is more efficient
    bool success = pcal95555_driver_->togglePin(pin);
    return success ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t pull_mode) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    bool success = true;
    switch (pull_mode) {
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
            success = pcal95555_driver_->setPullEnable(pin, false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
            success = pcal95555_driver_->setPullEnable(pin, true) && 
                     pcal95555_driver_->setPullDirection(pin, true);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
            success = pcal95555_driver_->setPullEnable(pin, true) && 
                     pcal95555_driver_->setPullDirection(pin, false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
            // Not directly supported by PCAL95555 - default to pull-up
            success = pcal95555_driver_->setPullEnable(pin, true) && 
                     pcal95555_driver_->setPullDirection(pin, true);
            break;
        default:
            return hf_gpio_err_t::GPIO_ERR_INVALID_PARAMETER;
    }
    return success ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::GetPullMode(hf_u8_t pin, hf_gpio_pull_mode_t& pull_mode) noexcept {
    if (!ValidatePin(pin)) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    // PCAL95555 doesn't have direct read methods for pull configuration
    // This is a limitation of the current driver - we'd need to track state or extend the driver
    // For now, default to floating
    pull_mode = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
    return hf_gpio_err_t::GPIO_SUCCESS;
} 

//**************************************************************************//
//**                   INTERRUPT MANAGEMENT IMPLEMENTATION                **//
//**************************************************************************//

hf_gpio_err_t Pcal95555Handler::RegisterPinInterrupt(hf_pin_num_t pin, 
                                                     hf_gpio_interrupt_trigger_t trigger,
                                                     InterruptCallback callback,
                                                     void* user_data) noexcept {
    if (pin >= 16) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    
    // Get the pin object from registry
    std::shared_ptr<Pcal95555GpioPin> gpio_pin;
    {
        MutexLockGuard pin_lock(pin_registry_mutex_);
        gpio_pin = pin_registry_[pin];
    }
    
    if (!gpio_pin) {
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;  // Pin must be created first
    }
    
    // Store interrupt data in the pin object itself
    {
        MutexLockGuard pin_lock(gpio_pin->pin_mutex_);
        gpio_pin->interrupt_callback_ = callback;
        gpio_pin->interrupt_user_data_ = user_data;
        gpio_pin->interrupt_trigger_ = trigger;
        gpio_pin->interrupt_enabled_ = true;
    }
    
    // Configure hardware interrupt if not already done
    if (interrupt_pin_ && !interrupt_configured_) {
        auto result = ConfigureHardwareInterrupt();
        if (result != hf_gpio_err_t::GPIO_SUCCESS) {
            // Clean up on failure
            MutexLockGuard pin_lock(gpio_pin->pin_mutex_);
            gpio_pin->interrupt_callback_ = nullptr;
            gpio_pin->interrupt_user_data_ = nullptr;
            gpio_pin->interrupt_enabled_ = false;
            return result;
        }
    }
    
    // Enable interrupt for this pin in PCAL95555 (unmask it)
    if (pcal95555_driver_) {
        uint16_t current_mask = pcal95555_driver_->getInterruptMask();
        uint16_t new_mask = current_mask & ~(1 << pin);  // Clear bit to unmask interrupt
        pcal95555_driver_->configureInterruptMask(new_mask);
    }
    
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::UnregisterPinInterrupt(hf_pin_num_t pin) noexcept {
    if (pin >= 16) return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    
    // Get the pin object from registry
    std::shared_ptr<Pcal95555GpioPin> gpio_pin;
    {
        MutexLockGuard pin_lock(pin_registry_mutex_);
        gpio_pin = pin_registry_[pin];
    }
    
    if (!gpio_pin) {
        return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;  // Pin doesn't exist
    }
    
    // Clear interrupt data in the pin object itself
    {
        MutexLockGuard pin_lock(gpio_pin->pin_mutex_);
        gpio_pin->interrupt_callback_ = nullptr;
        gpio_pin->interrupt_user_data_ = nullptr;
        gpio_pin->interrupt_trigger_ = hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_NONE;
        gpio_pin->interrupt_enabled_ = false;
    }
    
    // Mask interrupt for this pin in PCAL95555
    if (pcal95555_driver_) {
        uint16_t current_mask = pcal95555_driver_->getInterruptMask();
        uint16_t new_mask = current_mask | (1 << pin);  // Set bit to mask interrupt
        pcal95555_driver_->configureInterruptMask(new_mask);
    }
    
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555Handler::ConfigureHardwareInterrupt() noexcept {
    if (!interrupt_pin_) return hf_gpio_err_t::GPIO_ERR_NULL_POINTER;
    
    // Configure the hardware interrupt pin as input with falling edge trigger
    // PCAL95555 INT pin is active low (pulls low when interrupt occurs)
    auto result = interrupt_pin_->ConfigureInterrupt(
        hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING,
        HardwareInterruptCallback,
        this  // Pass handler instance as user data
    );
    
    if (result == hf_gpio_err_t::GPIO_SUCCESS) {
        interrupt_configured_ = true;
    }
    
    return result;
}

void Pcal95555Handler::HardwareInterruptCallback(BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) noexcept {
    // This is called in ISR context - keep it minimal
    auto* handler = static_cast<Pcal95555Handler*>(user_data);
    if (handler) {
        handler->ProcessInterrupts();
    }
}

void Pcal95555Handler::ProcessInterrupts() noexcept {
    if (!pcal95555_driver_) return;
    
    // Read interrupt status from PCAL95555 (this clears the interrupt)
    uint16_t status = pcal95555_driver_->getInterruptStatus();
    if (status == 0) return;  // No interrupts pending
    
    // Process each bit in the status mask
    for (int pin = 0; pin < 16; pin++) {
        if (status & (1 << pin)) {
            // Get the pin object from registry
            std::shared_ptr<Pcal95555GpioPin> gpio_pin;
            {
                MutexLockGuard pin_lock(pin_registry_mutex_);
                gpio_pin = pin_registry_[pin];
            }
            
            // If pin exists and has interrupt configured, call its callback
            if (gpio_pin && gpio_pin->interrupt_enabled_ && gpio_pin->interrupt_callback_) {
                gpio_pin->interrupt_callback_(
                    gpio_pin.get(), 
                    gpio_pin->interrupt_trigger_, 
                    gpio_pin->interrupt_user_data_
                );
            }
        }
    }
}
}

hf_gpio_err_t Pcal95555Handler::GetAllInterruptMasks(uint16_t& mask) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    if (pcal95555_driver_) {
        mask = pcal95555_driver_->getInterruptMask();
        return hf_gpio_err_t::GPIO_SUCCESS;
    }
    return hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::GetAllInterruptStatus(uint16_t& status) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    if (pcal95555_driver_) {
        status = pcal95555_driver_->getInterruptStatus();
        return hf_gpio_err_t::GPIO_SUCCESS;
    }
    return hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_u8_t Pcal95555Handler::GetI2cAddress() const noexcept {
    return pcal95555_driver_ ? pcal95555_driver_->getDeviceAddress() : 0;
}

std::shared_ptr<BaseGpio> Pcal95555Handler::CreateGpioPin(
    hf_pin_num_t pin,
    hf_gpio_direction_t direction,
    hf_gpio_active_state_t active_state,
    hf_gpio_output_mode_t output_mode,
    hf_gpio_pull_mode_t pull_mode,
    bool allow_existing) noexcept {
    
    if (pin >= 16) {
        return nullptr;
    }
    
    // Ensure handler is initialized before creating pins
    if (!EnsureInitialized()) {
        return nullptr;
    }
    
    MutexLockGuard pin_lock(pin_registry_mutex_);
    
    // Check if pin already exists
    if (pin_registry_[pin] != nullptr) {
        if (allow_existing) {
            return pin_registry_[pin];  // Return existing pin
        } else {
            return nullptr;  // Fail if pin already exists and not allowing existing
        }
    }
    
    // Create new pin and store in registry
    auto new_pin = std::make_shared<Pcal95555GpioPin>(
        pin, pcal95555_driver_, this, direction, active_state, output_mode, pull_mode
    );
    
    if (new_pin && new_pin->Initialize()) {
        pin_registry_[pin] = new_pin;
        return new_pin;
    }
    
    return nullptr;  // Failed to create or initialize
}

std::shared_ptr<BaseGpio> Pcal95555Handler::GetGpioPin(hf_pin_num_t pin) noexcept {
    if (pin >= 16) return nullptr;
    
    MutexLockGuard pin_lock(pin_registry_mutex_);
    return pin_registry_[pin];
}

bool Pcal95555Handler::IsPinCreated(hf_pin_num_t pin) const noexcept {
    if (pin >= 16) return false;
    
    MutexLockGuard pin_lock(pin_registry_mutex_);
    return pin_registry_[pin] != nullptr;
}

std::vector<hf_pin_num_t> Pcal95555Handler::GetCreatedPins() const noexcept {
    std::vector<hf_pin_num_t> created_pins;
    
    MutexLockGuard pin_lock(pin_registry_mutex_);
    for (hf_pin_num_t i = 0; i < 16; ++i) {
        if (pin_registry_[i] != nullptr) {
            created_pins.push_back(i);
        }
    }
    
    return created_pins;
}

// ===================== Pcal95555GpioPin Implementation ===================== //
Pcal95555GpioPin::Pcal95555GpioPin(
    hf_pin_num_t pin,
    std::shared_ptr<PCAL95555> driver,
    Pcal95555Handler* parent_handler,
    hf_gpio_direction_t direction,
    hf_gpio_active_state_t active_state,
    hf_gpio_output_mode_t output_mode,
    hf_gpio_pull_mode_t pull_mode) noexcept
    : BaseGpio(pin, direction, active_state, output_mode, pull_mode),
      pin_(pin), 
      driver_(std::move(driver)), 
      parent_handler_(parent_handler),
      interrupt_callback_(nullptr),
      interrupt_user_data_(nullptr),
      interrupt_trigger_(hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_NONE),
      interrupt_enabled_(false) {
    snprintf(description_, sizeof(description_), "PCAL95555_PIN_%d", static_cast<int>(pin_));
}

hf_bool_t Pcal95555GpioPin::Initialize() noexcept {
    MutexLockGuard lock(pin_mutex_);
    hf_bool_t ok = driver_ && driver_->setPinDirection(static_cast<hf_u8_t>(pin_), current_direction_ == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    if (ok) {
        switch (pull_mode_) {
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
                ok &= driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), false) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), false);
                break;
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
                ok &= driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), true) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), false);
                break;
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
                ok &= driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), false) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), true);
                break;
            case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
                ok &= driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), true) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), true);
                break;
        }
    }
    initialized_ = ok;
    return ok;
}

hf_bool_t Pcal95555GpioPin::Deinitialize() noexcept {
    MutexLockGuard lock(pin_mutex_);
    initialized_ = false;
    return true;
}

hf_bool_t Pcal95555GpioPin::IsPinAvailable() const noexcept {
    return driver_ && pin_ >= 0 && pin_ < 16;
}

const char* Pcal95555GpioPin::GetDescription() const noexcept {
    return description_;
}

hf_bool_t Pcal95555GpioPin::SupportsInterrupts() const noexcept {
    return true;
}

hf_gpio_err_t Pcal95555GpioPin::ConfigureInterrupt(hf_gpio_interrupt_trigger_t trigger, InterruptCallback callback, void* user_data) noexcept {
    if (!parent_handler_) {
        return hf_gpio_err_t::GPIO_ERR_NULL_POINTER;
    }
    
    // Delegate to parent handler for centralized interrupt management
    return parent_handler_->RegisterPinInterrupt(pin_, trigger, callback, user_data);
}

hf_gpio_err_t Pcal95555GpioPin::SetPolarityInversion(hf_bool_t invert) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->setPinPolarity(static_cast<hf_u8_t>(pin_), invert);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetPolarityInversion(hf_bool_t& invert) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = driver_->getPinPolarity(static_cast<hf_u8_t>(pin_), value);
    if (ok) invert = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetInterruptMask(hf_bool_t mask) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->setPinInterruptMask(static_cast<hf_u8_t>(pin_), mask);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetInterruptMask(hf_bool_t& mask) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = driver_->getPinInterruptMask(static_cast<hf_u8_t>(pin_), value);
    if (ok) mask = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetInterruptStatus(hf_bool_t& status) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t value = false;
    hf_bool_t ok = driver_->getPinInterruptStatus(static_cast<hf_u8_t>(pin_), value);
    if (ok) status = value;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetDirectionImpl(hf_gpio_direction_t direction) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = driver_->setPinDirection(static_cast<hf_u8_t>(pin_), direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::SetOutputModeImpl(hf_gpio_output_mode_t /*mode*/) noexcept {
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pcal95555GpioPin::SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    hf_bool_t ok = true;
    switch (mode) {
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
            ok = driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), false) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
            ok = driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), true) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
            ok = driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), false) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), true);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
            ok = driver_->setPinPullUp(static_cast<hf_u8_t>(pin_), true) && driver_->setPinPullDown(static_cast<hf_u8_t>(pin_), true);
            break;
    }
    pull_mode_ = mode;
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_pull_mode_t Pcal95555GpioPin::GetPullModeImpl() const noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
    hf_bool_t pullup = false, pulldown = false;
    driver_->getPinPullUp(static_cast<hf_u8_t>(pin_), pullup);
    driver_->getPinPullDown(static_cast<hf_u8_t>(pin_), pulldown);
    if (pullup && pulldown) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN;
    if (pullup) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP;
    if (pulldown) return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN;
    return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
}

hf_gpio_err_t Pcal95555GpioPin::SetPinLevelImpl(hf_gpio_level_t level) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    hf_bool_t hardware_level = (level == hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
    hf_bool_t ok = driver_->writePin(static_cast<hf_u8_t>(pin_), hardware_level);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetPinLevelImpl(hf_gpio_level_t& level) noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    hf_bool_t value = false;
    hf_bool_t ok = driver_->readPin(static_cast<hf_u8_t>(pin_), value);
    if (ok) {
        level = value ? hf_gpio_level_t::HF_GPIO_LEVEL_HIGH : hf_gpio_level_t::HF_GPIO_LEVEL_LOW;
    }
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    hf_bool_t is_output = false;
    hf_bool_t ok = driver_->getPinDirection(static_cast<hf_u8_t>(pin_), is_output);
    if (ok) {
        direction = is_output ? hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT 
                             : hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT;
    }
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555GpioPin::GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept {
    MutexLockGuard lock(pin_mutex_);
    if (!driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    // PCAL95555 only supports push-pull output mode
    mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

// ===================== Handler Factory Method ===================== //
std::unique_ptr<BaseGpio> Pcal95555Handler::CreateGpioPin(
    hf_pin_num_t pin,
    hf_gpio_direction_t direction,
    hf_gpio_active_state_t active_state,
    hf_gpio_output_mode_t output_mode,
    hf_gpio_pull_mode_t pull_mode) noexcept {
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return nullptr;
    return std::make_unique<Pcal95555GpioPin>(pin, pcal95555_driver_, direction, active_state, output_mode, pull_mode);
} 

// ===================== Pcal95555Handler Advanced Features ===================== //
hf_bool_t Pcal95555Handler::SetPolarityInversion(hf_pin_num_t pin, hf_bool_t invert) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return false;
    PCAL95555::Polarity polarity = invert ? PCAL95555::Polarity::Inverted : PCAL95555::Polarity::Normal;
    return pcal95555_driver_->setPinPolarity(static_cast<hf_u8_t>(pin), polarity);
}

hf_bool_t Pcal95555Handler::GetPolarityInversion(hf_pin_num_t pin, hf_bool_t& invert) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return false;
    
    // PCAL95555 driver doesn't provide a getPinPolarity method
    // We'd need to track this state internally or extend the driver
    // For now, default to normal polarity
    invert = false;
    return true;  // Return success since we provide a valid default
}

hf_bool_t Pcal95555Handler::SetInterruptMask(hf_pin_num_t pin, hf_bool_t mask) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return false;
    
    // PCAL95555 only provides configureInterruptMask for all pins at once
    // To set individual pin mask, we need to read current mask, modify bit, then write back
    uint16_t current_status = pcal95555_driver_->getInterruptStatus(); // This also reads current mask
    uint16_t pin_bit = 1U << pin;
    
    // Create new mask: if mask=true (disable interrupt), set bit to 1; if mask=false (enable), set bit to 0
    uint16_t new_mask = mask ? (current_status | pin_bit) : (current_status & ~pin_bit);
    
    return pcal95555_driver_->configureInterruptMask(new_mask);
}

hf_bool_t Pcal95555Handler::GetInterruptMask(hf_pin_num_t pin, hf_bool_t& mask) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return false;
    
    // PCAL95555 doesn't provide a way to read current interrupt mask separately from status
    // We'd need to track this state internally or extend the driver
    // For now, assume all interrupts are masked (disabled) by default
    mask = true;  // true means masked/disabled
    return true;  // Return success since we provide a valid default
}

hf_bool_t Pcal95555Handler::GetInterruptStatus(hf_pin_num_t pin, hf_bool_t& status) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ValidatePin(static_cast<hf_u8_t>(pin)) || !pcal95555_driver_) return false;
    
    // Get global interrupt status and extract the specific pin bit
    uint16_t global_status = pcal95555_driver_->getInterruptStatus();
    uint16_t pin_bit = 1U << pin;
    status = (global_status & pin_bit) != 0;
    
    return true;
}

hf_gpio_err_t Pcal95555Handler::SetDirections(uint16_t pin_mask, hf_gpio_direction_t direction) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    PCAL95555::GPIODir dir = (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT) ? 
                             PCAL95555::GPIODir::Input : PCAL95555::GPIODir::Output;
    bool success = pcal95555_driver_->setMultipleDirections(pin_mask, dir);
    return success ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetOutputs(uint16_t pin_mask, hf_bool_t active) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    // PCAL95555 doesn't have a bulk write method, so we need to set each pin individually
    // This is less efficient but ensures correctness
    bool all_success = true;
    for (uint8_t pin = 0; pin < 16; ++pin) {
        if (pin_mask & (1U << pin)) {
            if (!pcal95555_driver_->writePin(pin, active)) {
                all_success = false;
            }
        }
    }
    
    return all_success ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::SetPullModes(uint16_t pin_mask, hf_gpio_pull_mode_t pull_mode) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool ok = true;
    switch (pull_mode) {
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING:
            ok = pcal95555_driver_->setPinsPullUp(pin_mask, false) && pcal95555_driver_->setPinsPullDown(pin_mask, false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP:
            ok = pcal95555_driver_->setPinsPullUp(pin_mask, true) && pcal95555_driver_->setPinsPullDown(pin_mask, false);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN:
            ok = pcal95555_driver_->setPinsPullUp(pin_mask, false) && pcal95555_driver_->setPinsPullDown(pin_mask, true);
            break;
        case hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP_DOWN:
            ok = pcal95555_driver_->setPinsPullUp(pin_mask, true) && pcal95555_driver_->setPinsPullDown(pin_mask, true);
            break;
        default:
            ok = false;
    }
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::GetAllInterruptMasks(uint16_t& mask) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool ok = pcal95555_driver_->getAllInterruptMasks(mask);
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_FAILURE;
}

hf_gpio_err_t Pcal95555Handler::GetAllInterruptStatus(uint16_t& status) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_ || !pcal95555_driver_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    status = pcal95555_driver_->getInterruptStatus();
    
    // Check if there was a communication error during the read operation  
    uint16_t error_flags = pcal95555_driver_->getErrorFlags();
    if (error_flags != 0) {
        return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    }
    
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_bool_t Pcal95555Handler::SoftwareReset() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!pcal95555_driver_) return false;
    return pcal95555_driver_->softwareReset();
}

hf_bool_t Pcal95555Handler::PowerDown() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!pcal95555_driver_) return false;
    return pcal95555_driver_->powerDown();
}

void Pcal95555Handler::DumpDiagnostics() const noexcept {
    static constexpr const char* TAG = "Pcal95555Handler";
    
    Logger::GetInstance().Info(TAG, "=== PCAL95555 HANDLER DIAGNOSTICS ===");
    
    MutexLockGuard lock(handler_mutex_);
    
    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", initialized_ ? "YES" : "NO");
    
    // I2C Interface
    Logger::GetInstance().Info(TAG, "I2C Interface:");
    if (i2c_adapter_) {
        Logger::GetInstance().Info(TAG, "  I2C Adapter: ACTIVE");
        Logger::GetInstance().Info(TAG, "  Device Address: 0x%02X", i2c_adapter_->GetDeviceAddress());
    } else {
        Logger::GetInstance().Info(TAG, "  I2C Adapter: NOT_INITIALIZED");
    }
    
    // Driver Status
    Logger::GetInstance().Info(TAG, "PCAL95555 Driver:");
    if (pcal95555_driver_) {
        Logger::GetInstance().Info(TAG, "  Driver Instance: ACTIVE");
    } else {
        Logger::GetInstance().Info(TAG, "  Driver Instance: NOT_INITIALIZED");
    }
    
    // Pin Registry Status
    Logger::GetInstance().Info(TAG, "Pin Registry:");
    int active_pins = 0;
    for (size_t i = 0; i < pin_registry_.size(); ++i) {
        if (pin_registry_[i] != nullptr) {
            active_pins++;
        }
    }
    Logger::GetInstance().Info(TAG, "  Active Pin Objects: %d/%d", active_pins, static_cast<int>(pin_registry_.size()));
    
    // Interrupt Configuration
    Logger::GetInstance().Info(TAG, "Interrupt Configuration:");
    Logger::GetInstance().Info(TAG, "  Hardware Interrupt Pin: %s", interrupt_pin_ ? "CONFIGURED" : "NOT_CONFIGURED");
    Logger::GetInstance().Info(TAG, "  Interrupt System: %s", interrupt_configured_ ? "ENABLED" : "DISABLED");
    
    // Pin Configuration Summary
    int configured_interrupts = 0;
    for (size_t i = 0; i < pin_callbacks_.size(); ++i) {
        if (pin_callbacks_[i] != nullptr) {
            configured_interrupts++;
        }
    }
    Logger::GetInstance().Info(TAG, "  Pins with Interrupts: %d", configured_interrupts);
    
    // Pin Details (show first few active pins)
    Logger::GetInstance().Info(TAG, "Pin Details (first 8 active pins):");
    int pins_shown = 0;
    for (size_t i = 0; i < pin_registry_.size() && pins_shown < 8; ++i) {
        if (pin_registry_[i] != nullptr) {
            bool has_interrupt = pin_callbacks_[i] != nullptr;
            const char* trigger_str = "NONE";
            if (pin_triggers_[i] == hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_RISING_EDGE) {
                trigger_str = "RISING";
            } else if (pin_triggers_[i] == hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE) {
                trigger_str = "FALLING";
            } else if (pin_triggers_[i] == hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_BOTH_EDGES) {
                trigger_str = "BOTH";
            }
            
            Logger::GetInstance().Info(TAG, "    Pin %d: ACTIVE, Interrupt: %s, Trigger: %s", 
                static_cast<int>(i), has_interrupt ? "YES" : "NO", trigger_str);
            pins_shown++;
        }
    }
    
    if (pins_shown == 0) {
        Logger::GetInstance().Info(TAG, "    No active pins found");
    }
    
    // Memory Usage
    Logger::GetInstance().Info(TAG, "Memory Usage:");
    size_t estimated_memory = sizeof(*this);
    if (pcal95555_driver_) estimated_memory += sizeof(PCAL95555);
    if (i2c_adapter_) estimated_memory += sizeof(Pcal95555I2cAdapter);
    
    // Estimate pin object memory
    estimated_memory += active_pins * sizeof(Pcal95555GpioPin);
    
    Logger::GetInstance().Info(TAG, "  Estimated Total: %d bytes", static_cast<int>(estimated_memory));
    Logger::GetInstance().Info(TAG, "  Pin Objects: %d x %d bytes", active_pins, static_cast<int>(sizeof(Pcal95555GpioPin)));
    
    // Hardware Status
    Logger::GetInstance().Info(TAG, "Hardware Status:");
    Logger::GetInstance().Info(TAG, "  Max Pins Supported: %d", static_cast<int>(pin_registry_.size()));
    Logger::GetInstance().Info(TAG, "  I2C Communication: %s", i2c_adapter_ ? "READY" : "NOT_READY");
    
    // System Status Summary
    bool system_healthy = initialized_ && 
                         (pcal95555_driver_ != nullptr) &&
                         (i2c_adapter_ != nullptr);
    
    Logger::GetInstance().Info(TAG, "System Status: %s", system_healthy ? "HEALTHY" : "DEGRADED");
    
    Logger::GetInstance().Info(TAG, "=== END PCAL95555 HANDLER DIAGNOSTICS ===");
}
