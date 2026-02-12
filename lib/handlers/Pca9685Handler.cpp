/**
 * @file Pca9685Handler.cpp
 * @brief Implementation of the PCA9685 PWM controller handler.
 *
 * @details
 * Implements all four layers defined in Pca9685Handler.h:
 * 1. HalI2cPca9685Comm   -- CRTP I2C communication adapter
 * 2. Pca9685Handler       -- Main handler (init, PWM ops, config, factory)
 * 3. Pca9685PwmAdapter    -- Multi-channel BasePwm adapter
 * 4. Pca9685GpioPin       -- Per-channel BaseGpio wrapper
 *
 * All driver calls use the PascalCase API of the updated hf-pca9685-driver
 * (pca9685::PCA9685<I2cType>). Error handling follows the driver's error-flag
 * model: individual methods return bool, with accumulated error flags available
 * via GetErrorFlags().
 *
 * @see Pca9685Handler.h  for architectural overview and Doxygen documentation.
 *
 * @author HardFOC Team
 * @date 2025
 */

#include "Pca9685Handler.h"
#include "handlers/Logger.h"
#include <cstring>
#include <cmath>

// =====================================================================
// HalI2cPca9685Comm Implementation
// =====================================================================

HalI2cPca9685Comm::HalI2cPca9685Comm(BaseI2c& i2c_device) noexcept
    : i2c_device_(i2c_device) {}

bool HalI2cPca9685Comm::Write(uint8_t addr, uint8_t reg,
                               const uint8_t* data, size_t len) noexcept {
    MutexLockGuard lock(i2c_mutex_);

    // Validate that the driver's address matches the BaseI2c device address.
    if (addr != i2c_device_.GetDeviceAddress()) {
        return false;
    }

    // Frame the I2C register write: [register, data...]
    // PCA9685 register writes are at most 4 data bytes; 8-byte buffer is sufficient.
    constexpr size_t kMaxBuf = 8;
    if (len + 1 > kMaxBuf) {
        return false;
    }

    uint8_t command[kMaxBuf];
    command[0] = reg;
    std::memcpy(&command[1], data, len);
    return i2c_device_.Write(command, len + 1) == hf_i2c_err_t::I2C_SUCCESS;
}

bool HalI2cPca9685Comm::Read(uint8_t addr, uint8_t reg,
                              uint8_t* data, size_t len) noexcept {
    MutexLockGuard lock(i2c_mutex_);

    if (addr != i2c_device_.GetDeviceAddress()) {
        return false;
    }

    return i2c_device_.WriteRead(&reg, 1, data, len) == hf_i2c_err_t::I2C_SUCCESS;
}

bool HalI2cPca9685Comm::EnsureInitialized() noexcept {
    // BaseI2c device is expected to be initialized before the handler uses it.
    return true;
}

// =====================================================================
// Pca9685Handler -- Construction & Lifecycle
// =====================================================================

Pca9685Handler::Pca9685Handler(BaseI2c& i2c_device) noexcept
    : i2c_device_(i2c_device),
      i2c_adapter_(nullptr),
      pca9685_driver_(nullptr),
      initialized_(false),
      pwm_adapter_(nullptr) {
    gpio_registry_.fill(nullptr);
}

bool Pca9685Handler::EnsureInitialized() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (initialized_) {
        return true;
    }
    return initializeInternal() == hf_pwm_err_t::PWM_SUCCESS;
}

bool Pca9685Handler::EnsureDeinitialized() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!initialized_) {
        return true;
    }
    return deinitializeInternal() == hf_pwm_err_t::PWM_SUCCESS;
}

bool Pca9685Handler::ensureInitializedLocked() noexcept {
    if (initialized_) return true;
    return initializeInternal() == hf_pwm_err_t::PWM_SUCCESS;
}

hf_pwm_err_t Pca9685Handler::initializeInternal() noexcept {
    if (initialized_) {
        return hf_pwm_err_t::PWM_SUCCESS;
    }

    // 1. Create the CRTP I2C adapter.
    if (!i2c_adapter_) {
        i2c_adapter_ = std::make_unique<HalI2cPca9685Comm>(i2c_device_);
        if (!i2c_adapter_) {
            return hf_pwm_err_t::PWM_ERR_FAILURE;
        }
    }

    // 2. Create the typed PCA9685 driver.
    if (!pca9685_driver_) {
        pca9685_driver_ = std::make_unique<Pca9685Driver>(
            i2c_adapter_.get(),
            static_cast<uint8_t>(i2c_device_.GetDeviceAddress()));
        if (!pca9685_driver_) {
            return hf_pwm_err_t::PWM_ERR_FAILURE;
        }
    }

    // 3. Initialize the driver (resets device, verifies communication).
    if (!pca9685_driver_->EnsureInitialized()) {
        return hf_pwm_err_t::PWM_ERR_DEVICE_NOT_RESPONDING;
    }

    initialized_ = true;
    return hf_pwm_err_t::PWM_SUCCESS;
}

hf_pwm_err_t Pca9685Handler::deinitializeInternal() noexcept {
    if (!initialized_) {
        return hf_pwm_err_t::PWM_SUCCESS;
    }

    // Clear GPIO registry.
    gpio_registry_.fill(nullptr);

    // Clear PWM adapter.
    pwm_adapter_.reset();

    // Release driver and adapter.
    pca9685_driver_.reset();
    i2c_adapter_.reset();
    initialized_ = false;
    return hf_pwm_err_t::PWM_SUCCESS;
}

// =====================================================================
// Pca9685Handler -- PWM Control
// =====================================================================

bool Pca9685Handler::SetFrequency(float freq_hz) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return false;
    return pca9685_driver_->SetPwmFreq(freq_hz);
}

bool Pca9685Handler::SetDuty(uint8_t channel, float duty) noexcept {
    if (!validateChannel(channel)) return false;
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return false;
    return pca9685_driver_->SetDuty(channel, duty);
}

bool Pca9685Handler::SetPwm(uint8_t channel, uint16_t on_time,
                             uint16_t off_time) noexcept {
    if (!validateChannel(channel)) return false;
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return false;
    return pca9685_driver_->SetPwm(channel, on_time, off_time);
}

bool Pca9685Handler::SetAllPwm(uint16_t on_time, uint16_t off_time) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return false;
    return pca9685_driver_->SetAllPwm(on_time, off_time);
}

bool Pca9685Handler::SetChannelFullOn(uint8_t channel) noexcept {
    if (!validateChannel(channel)) return false;
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return false;
    return pca9685_driver_->SetChannelFullOn(channel);
}

bool Pca9685Handler::SetChannelFullOff(uint8_t channel) noexcept {
    if (!validateChannel(channel)) return false;
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return false;
    return pca9685_driver_->SetChannelFullOff(channel);
}

// =====================================================================
// Pca9685Handler -- Power Management
// =====================================================================

bool Pca9685Handler::Sleep() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return false;
    return pca9685_driver_->Sleep();
}

bool Pca9685Handler::Wake() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return false;
    return pca9685_driver_->Wake();
}

// =====================================================================
// Pca9685Handler -- Output Configuration
// =====================================================================

bool Pca9685Handler::SetOutputInvert(bool invert) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return false;
    return pca9685_driver_->SetOutputInvert(invert);
}

bool Pca9685Handler::SetOutputDriverMode(bool totem_pole) noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return false;
    return pca9685_driver_->SetOutputDriverMode(totem_pole);
}

// =====================================================================
// Pca9685Handler -- Wrapper Factories
// =====================================================================

uint8_t Pca9685Handler::GetI2cAddress() const noexcept {
    // PCA9685 driver stores address but has no public getter.
    // Return the BaseI2c device address which is always in sync.
    return static_cast<uint8_t>(i2c_device_.GetDeviceAddress());
}

std::shared_ptr<BasePwm> Pca9685Handler::GetPwmAdapter() noexcept {
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return nullptr;

    if (!pwm_adapter_) {
        pwm_adapter_ = std::make_shared<Pca9685PwmAdapter>(this);
        if (pwm_adapter_) {
            pwm_adapter_->Initialize();
        }
    }
    return pwm_adapter_;
}

std::shared_ptr<BaseGpio> Pca9685Handler::CreateGpioPin(
    hf_pin_num_t channel,
    hf_gpio_active_state_t active_state,
    bool allow_existing) noexcept {

    if (channel < 0 || channel >= 16) return nullptr;
    MutexLockGuard lock(handler_mutex_);
    if (!ensureInitializedLocked()) return nullptr;

    // Check if pin already exists.
    if (gpio_registry_[channel] != nullptr) {
        return allow_existing
                   ? std::static_pointer_cast<BaseGpio>(gpio_registry_[channel])
                   : nullptr;
    }

    auto new_pin = std::make_shared<Pca9685GpioPin>(channel, this, active_state);
    if (new_pin && new_pin->Initialize()) {
        gpio_registry_[channel] = new_pin;
        return new_pin;
    }
    return nullptr;
}

std::shared_ptr<BaseGpio> Pca9685Handler::GetGpioPin(hf_pin_num_t channel) noexcept {
    if (channel < 0 || channel >= 16) return nullptr;
    MutexLockGuard lock(handler_mutex_);
    return gpio_registry_[channel];
}

bool Pca9685Handler::IsPinCreated(hf_pin_num_t channel) const noexcept {
    if (channel < 0 || channel >= 16) return false;
    MutexLockGuard lock(handler_mutex_);
    return gpio_registry_[channel] != nullptr;
}

// =====================================================================
// Pca9685Handler -- Error Management
// =====================================================================

uint16_t Pca9685Handler::GetErrorFlags() const noexcept {
    return pca9685_driver_ ? pca9685_driver_->GetErrorFlags() : 0;
}

void Pca9685Handler::ClearErrorFlags(uint16_t mask) noexcept {
    if (pca9685_driver_) {
        pca9685_driver_->ClearErrorFlags(mask);
    }
}

// =====================================================================
// Pca9685Handler -- Diagnostics
// =====================================================================

void Pca9685Handler::DumpDiagnostics() const noexcept {
    static constexpr const char* TAG = "Pca9685Handler";

    Logger::GetInstance().Info(TAG, "=== PCA9685 HANDLER DIAGNOSTICS ===");

    MutexLockGuard lock(handler_mutex_);

    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s",
                              initialized_ ? "YES" : "NO");

    Logger::GetInstance().Info(TAG, "I2C Interface:");
    if (i2c_adapter_) {
        Logger::GetInstance().Info(TAG, "  I2C Adapter: ACTIVE (CRTP-based)");
        Logger::GetInstance().Info(TAG, "  Device Address: 0x%02X",
                                  i2c_device_.GetDeviceAddress());
    } else {
        Logger::GetInstance().Info(TAG, "  I2C Adapter: NOT_INITIALIZED");
    }

    Logger::GetInstance().Info(TAG, "PCA9685 Driver:");
    if (pca9685_driver_) {
        Logger::GetInstance().Info(TAG, "  Driver Instance: ACTIVE");
        Logger::GetInstance().Info(TAG, "  Error Flags: 0x%04X",
                                  pca9685_driver_->GetErrorFlags());
    } else {
        Logger::GetInstance().Info(TAG, "  Driver Instance: NOT_INITIALIZED");
    }

    // PWM Adapter Status
    Logger::GetInstance().Info(TAG, "PWM Adapter:");
    if (pwm_adapter_) {
        int enabled = 0;
        for (int i = 0; i < 16; ++i) {
            if (pwm_adapter_->channel_enabled_[i]) ++enabled;
        }
        Logger::GetInstance().Info(TAG, "  Status: ACTIVE");
        Logger::GetInstance().Info(TAG, "  Enabled Channels: %d/16", enabled);
        Logger::GetInstance().Info(TAG, "  Frequency: %lu Hz",
                                  static_cast<unsigned long>(pwm_adapter_->current_frequency_hz_));
    } else {
        Logger::GetInstance().Info(TAG, "  Status: NOT_CREATED");
    }

    // GPIO Pin Registry
    int gpio_count = 0;
    for (int i = 0; i < 16; ++i) {
        if (gpio_registry_[i] != nullptr) ++gpio_count;
    }
    Logger::GetInstance().Info(TAG, "GPIO Registry:");
    Logger::GetInstance().Info(TAG, "  Active GPIO Pins: %d/16", gpio_count);

    bool healthy = initialized_ && pca9685_driver_ && i2c_adapter_;
    Logger::GetInstance().Info(TAG, "System Status: %s",
                              healthy ? "HEALTHY" : "DEGRADED");

    Logger::GetInstance().Info(TAG, "=== END PCA9685 HANDLER DIAGNOSTICS ===");
}

// =====================================================================
// Pca9685PwmAdapter Implementation
// =====================================================================

Pca9685PwmAdapter::Pca9685PwmAdapter(Pca9685Handler* parent_handler) noexcept
    : parent_handler_(parent_handler) {
    duty_cache_.fill(0.0f);
    on_time_cache_.fill(0);
    channel_enabled_.fill(false);
}

hf_pwm_err_t Pca9685PwmAdapter::Initialize() noexcept {
    if (!parent_handler_) return hf_pwm_err_t::PWM_ERR_NULL_POINTER;
    if (!parent_handler_->IsInitialized()) {
        if (!parent_handler_->EnsureInitialized()) {
            return hf_pwm_err_t::PWM_ERR_NOT_INITIALIZED;
        }
    }
    initialized_ = true;
    return hf_pwm_err_t::PWM_SUCCESS;
}

hf_pwm_err_t Pca9685PwmAdapter::Deinitialize() noexcept {
    initialized_ = false;
    channel_enabled_.fill(false);
    duty_cache_.fill(0.0f);
    on_time_cache_.fill(0);
    return hf_pwm_err_t::PWM_SUCCESS;
}

hf_pwm_err_t Pca9685PwmAdapter::EnableChannel(hf_channel_id_t channel_id) noexcept {
    if (!validateChannel(channel_id)) return hf_pwm_err_t::PWM_ERR_INVALID_CHANNEL;
    if (!parent_handler_) return hf_pwm_err_t::PWM_ERR_NULL_POINTER;

    // Restore the cached duty cycle when enabling.
    float duty = duty_cache_[channel_id];
    if (duty > 0.0f) {
        if (!parent_handler_->SetDuty(static_cast<uint8_t>(channel_id), duty)) {
            return hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
        }
    } else {
        // Duty is 0 -- set full-off to be explicit.
        if (!parent_handler_->SetChannelFullOff(static_cast<uint8_t>(channel_id))) {
            return hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
        }
    }

    channel_enabled_[channel_id] = true;
    return hf_pwm_err_t::PWM_SUCCESS;
}

hf_pwm_err_t Pca9685PwmAdapter::DisableChannel(hf_channel_id_t channel_id) noexcept {
    if (!validateChannel(channel_id)) return hf_pwm_err_t::PWM_ERR_INVALID_CHANNEL;
    if (!parent_handler_) return hf_pwm_err_t::PWM_ERR_NULL_POINTER;

    // Set channel to full-off (disable output).
    if (!parent_handler_->SetChannelFullOff(static_cast<uint8_t>(channel_id))) {
        return hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
    }

    channel_enabled_[channel_id] = false;
    return hf_pwm_err_t::PWM_SUCCESS;
}

bool Pca9685PwmAdapter::IsChannelEnabled(hf_channel_id_t channel_id) const noexcept {
    if (!validateChannel(channel_id)) return false;
    return channel_enabled_[channel_id];
}

hf_pwm_err_t Pca9685PwmAdapter::SetDutyCycle(hf_channel_id_t channel_id,
                                              float duty_cycle) noexcept {
    if (!validateChannel(channel_id)) return hf_pwm_err_t::PWM_ERR_INVALID_CHANNEL;
    if (!parent_handler_) return hf_pwm_err_t::PWM_ERR_NULL_POINTER;

    duty_cycle = ClampDutyCycle(duty_cycle);

    // Handle edge cases: full-on and full-off.
    if (duty_cycle >= 1.0f) {
        if (!parent_handler_->SetChannelFullOn(static_cast<uint8_t>(channel_id))) {
            return hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
        }
    } else if (duty_cycle <= 0.0f) {
        if (!parent_handler_->SetChannelFullOff(static_cast<uint8_t>(channel_id))) {
            return hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
        }
    } else {
        // Use on-time offset for phase shift, off-time for duty.
        uint16_t on_time = on_time_cache_[channel_id];
        auto off_tick = static_cast<uint16_t>(lroundf(duty_cycle * kMaxRawValue));
        uint16_t off_time = (on_time + off_tick) & kMaxRawValue;
        if (!parent_handler_->SetPwm(static_cast<uint8_t>(channel_id), on_time, off_time)) {
            return hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
        }
    }

    duty_cache_[channel_id] = duty_cycle;
    channel_enabled_[channel_id] = true;
    return hf_pwm_err_t::PWM_SUCCESS;
}

hf_pwm_err_t Pca9685PwmAdapter::SetDutyCycleRaw(hf_channel_id_t channel_id,
                                                  hf_u32_t raw_value) noexcept {
    if (!validateChannel(channel_id)) return hf_pwm_err_t::PWM_ERR_INVALID_CHANNEL;
    if (raw_value > kMaxRawValue) return hf_pwm_err_t::PWM_ERR_DUTY_OUT_OF_RANGE;
    if (!parent_handler_) return hf_pwm_err_t::PWM_ERR_NULL_POINTER;

    uint16_t on_time = on_time_cache_[channel_id];
    auto off_time = static_cast<uint16_t>(raw_value);

    if (!parent_handler_->SetPwm(static_cast<uint8_t>(channel_id), on_time, off_time)) {
        return hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
    }

    duty_cache_[channel_id] = static_cast<float>(raw_value) / static_cast<float>(kMaxRawValue);
    channel_enabled_[channel_id] = true;
    return hf_pwm_err_t::PWM_SUCCESS;
}

hf_pwm_err_t Pca9685PwmAdapter::SetFrequency(hf_channel_id_t /*channel_id*/,
                                               hf_frequency_hz_t frequency_hz) noexcept {
    if (!parent_handler_) return hf_pwm_err_t::PWM_ERR_NULL_POINTER;

    if (frequency_hz < kMinFrequencyHz || frequency_hz > kMaxFrequencyHz) {
        return hf_pwm_err_t::PWM_ERR_INVALID_FREQUENCY;
    }

    if (!parent_handler_->SetFrequency(static_cast<float>(frequency_hz))) {
        return hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
    }

    current_frequency_hz_ = frequency_hz;
    return hf_pwm_err_t::PWM_SUCCESS;
}

hf_pwm_err_t Pca9685PwmAdapter::SetPhaseShift(hf_channel_id_t channel_id,
                                                float phase_shift_degrees) noexcept {
    if (!validateChannel(channel_id)) return hf_pwm_err_t::PWM_ERR_INVALID_CHANNEL;
    if (!parent_handler_) return hf_pwm_err_t::PWM_ERR_NULL_POINTER;

    // Clamp to [0, 360).
    while (phase_shift_degrees < 0.0f) phase_shift_degrees += 360.0f;
    while (phase_shift_degrees >= 360.0f) phase_shift_degrees -= 360.0f;

    // Map degrees to PCA9685 on-time ticks (0-4095).
    auto on_time = static_cast<uint16_t>(
        lroundf((phase_shift_degrees / 360.0f) * static_cast<float>(kMaxRawValue + 1)));
    if (on_time > kMaxRawValue) on_time = kMaxRawValue;

    on_time_cache_[channel_id] = on_time;

    // If channel is active, re-apply duty cycle with new phase offset.
    if (channel_enabled_[channel_id] && duty_cache_[channel_id] > 0.0f) {
        auto off_tick = static_cast<uint16_t>(
            lroundf(duty_cache_[channel_id] * kMaxRawValue));
        uint16_t off_time = (on_time + off_tick) & kMaxRawValue;
        if (!parent_handler_->SetPwm(static_cast<uint8_t>(channel_id), on_time, off_time)) {
            return hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
        }
    }

    return hf_pwm_err_t::PWM_SUCCESS;
}

hf_pwm_err_t Pca9685PwmAdapter::StartAll() noexcept {
    if (!parent_handler_) return hf_pwm_err_t::PWM_ERR_NULL_POINTER;
    return parent_handler_->Wake()
               ? hf_pwm_err_t::PWM_SUCCESS
               : hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
}

hf_pwm_err_t Pca9685PwmAdapter::StopAll() noexcept {
    if (!parent_handler_) return hf_pwm_err_t::PWM_ERR_NULL_POINTER;
    return parent_handler_->Sleep()
               ? hf_pwm_err_t::PWM_SUCCESS
               : hf_pwm_err_t::PWM_ERR_COMMUNICATION_FAILURE;
}

hf_pwm_err_t Pca9685PwmAdapter::UpdateAll() noexcept {
    // PCA9685 updates immediately on register write -- no buffered output mode.
    return hf_pwm_err_t::PWM_SUCCESS;
}

hf_pwm_err_t Pca9685PwmAdapter::SetComplementaryOutput(
    hf_channel_id_t /*primary_channel*/,
    hf_channel_id_t /*complementary_channel*/,
    hf_u32_t /*deadtime_ns*/) noexcept {
    return hf_pwm_err_t::PWM_ERR_UNSUPPORTED_OPERATION;
}

float Pca9685PwmAdapter::GetDutyCycle(hf_channel_id_t channel_id) const noexcept {
    if (!validateChannel(channel_id)) return -1.0f;
    return duty_cache_[channel_id];
}

hf_frequency_hz_t Pca9685PwmAdapter::GetFrequency(
    hf_channel_id_t /*channel_id*/) const noexcept {
    return current_frequency_hz_;
}

// =====================================================================
// Pca9685GpioPin Implementation
// =====================================================================

Pca9685GpioPin::Pca9685GpioPin(
    hf_pin_num_t pin,
    Pca9685Handler* parent_handler,
    hf_gpio_active_state_t active_state) noexcept
    : BaseGpio(pin,
               hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT,
               active_state,
               hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL,
               hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING),
      pin_(pin),
      parent_handler_(parent_handler) {
    snprintf(description_, sizeof(description_), "PCA9685_CH_%d",
             static_cast<int>(pin_));
}

bool Pca9685GpioPin::Initialize() noexcept {
    if (!parent_handler_) return false;
    if (!parent_handler_->EnsureInitialized()) return false;

    // Start with channel off.
    if (!parent_handler_->SetChannelFullOff(static_cast<uint8_t>(pin_))) {
        return false;
    }
    cached_level_ = hf_gpio_level_t::HF_GPIO_LEVEL_LOW;
    initialized_ = true;
    return true;
}

bool Pca9685GpioPin::Deinitialize() noexcept {
    initialized_ = false;
    return true;
}

bool Pca9685GpioPin::IsPinAvailable() const noexcept {
    return parent_handler_ && pin_ >= 0 && pin_ < 16;
}

const char* Pca9685GpioPin::GetDescription() const noexcept {
    return description_;
}

// --- BaseGpio Protected Implementation ---

hf_gpio_err_t Pca9685GpioPin::SetDirectionImpl(
    hf_gpio_direction_t direction) noexcept {
    // PCA9685 is output-only.
    if (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT) {
        return hf_gpio_err_t::GPIO_SUCCESS;
    }
    return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
}

hf_gpio_err_t Pca9685GpioPin::GetDirectionImpl(
    hf_gpio_direction_t& direction) const noexcept {
    direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pca9685GpioPin::SetOutputModeImpl(
    hf_gpio_output_mode_t /*mode*/) noexcept {
    // PCA9685 output mode is configured globally via SetOutputDriverMode().
    // Per-channel changes are not supported at the GPIO wrapper level.
    return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
}

hf_gpio_err_t Pca9685GpioPin::GetOutputModeImpl(
    hf_gpio_output_mode_t& mode) const noexcept {
    mode = output_mode_;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Pca9685GpioPin::SetPullModeImpl(
    hf_gpio_pull_mode_t /*mode*/) noexcept {
    // PCA9685 has no pull resistors.
    return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
}

hf_gpio_pull_mode_t Pca9685GpioPin::GetPullModeImpl() const noexcept {
    return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
}

hf_gpio_err_t Pca9685GpioPin::SetPinLevelImpl(
    hf_gpio_level_t level) noexcept {
    if (!parent_handler_) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    bool success = false;
    if (level == hf_gpio_level_t::HF_GPIO_LEVEL_HIGH) {
        success = parent_handler_->SetChannelFullOn(static_cast<uint8_t>(pin_));
    } else {
        success = parent_handler_->SetChannelFullOff(static_cast<uint8_t>(pin_));
    }

    if (success) {
        cached_level_ = level;
        return hf_gpio_err_t::GPIO_SUCCESS;
    }
    return hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
}

hf_gpio_err_t Pca9685GpioPin::GetPinLevelImpl(
    hf_gpio_level_t& level) noexcept {
    // PCA9685 is output-only; return the cached level.
    level = cached_level_;
    return hf_gpio_err_t::GPIO_SUCCESS;
}
