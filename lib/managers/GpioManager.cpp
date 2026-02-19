/**
 * @file GpioManager.cpp
 * @brief Implementation of the Vortex V1 GPIO manager.
 *
 * @details Multi-chip GPIO initialization:
 *          - ESP32 internal: make_shared<EspGpio>(...)
 *          - PCAL95555 expander: Pcal95555Handler::CreateGpioPin(...)
 *          - TMC9660 controller: non-owning alias to handler->gpio(pin_id)
 *
 * @version 2.0
 */

#include "GpioManager.h"
#include "CommChannelsManager.h"
#include "MotorController.h"
#include "handlers/pcal95555/Pcal95555Handler.h"
#include "handlers/tmc9660/Tmc9660Handler.h"
#include "mcu/esp32/EspGpio.h"
#include "handlers/logger/Logger.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"

static constexpr const char* TAG = "VortexGpio";

//==============================================================================
// SINGLETON
//==============================================================================

GpioManager& GpioManager::GetInstance() noexcept {
    static GpioManager instance;
    return instance;
}

//==============================================================================
// INITIALIZATION
//==============================================================================

bool GpioManager::EnsureInitialized() noexcept {
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    MutexLockGuard lock(mutex_);
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    bool ok = Initialize();
    initialized_.store(ok, std::memory_order_release);
    return ok;
}

bool GpioManager::Initialize() noexcept {
    Logger::GetInstance().Info(TAG, "Initializing Vortex GPIO manager");

    init_time_ms_ = os_get_elapsed_time_msec();
    registered_count_ = 0;

    // Obtain CommChannelsManager (needed for PCAL95555 I2C access)
    auto& comm = CommChannelsManager::GetInstance();
    if (!comm.IsInitialized()) {
        Logger::GetInstance().Error(TAG, "CommChannelsManager not initialized");
        return false;
    }

    // Obtain MotorController for TMC9660 GPIO bridge
    motor_controller_ = &MotorController::GetInstance();
    if (!motor_controller_->EnsureInitialized()) {
        Logger::GetInstance().Warn(TAG, "MotorController init failed — TMC9660 GPIO will be unavailable");
    }

    // Iterate all pins defined in the platform pin config
    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE; ++i) {
        const auto& m = HF_GPIO_MAPPING[i];
        auto category = static_cast<HfPinCategory>(m.category);

        // Only register GPIO / USER category pins
        if (!should_register_as_gpio(category)) {
            continue;
        }

        // Only register unit-0, bank-0 pins (single instance of each chip on board)
        if (m.chip_unit != 0 || m.gpio_bank != 0) {
            continue;
        }

        auto chip = static_cast<HfGpioChipType>(m.chip_type);
        std::shared_ptr<BaseGpio> gpio;

        switch (chip) {
            case HfGpioChipType::ESP32_INTERNAL: {
                // Derive configuration from pincfg primitives
                hf_gpio_pull_mode_t pull = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
                if (m.has_pull) {
                    pull = m.pull_is_up ? hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP
                                       : hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN;
                }
                hf_gpio_output_mode_t out = m.is_push_pull
                    ? hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL
                    : hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_OPEN_DRAIN;
                hf_gpio_active_state_t active = m.is_inverted
                    ? hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW
                    : hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH;

                auto esp = std::make_shared<EspGpio>(
                    static_cast<hf_pin_num_t>(m.physical_pin),
                    hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT,
                    active, out, pull);

                if (esp->EnsureInitialized()) {
                    gpio = std::move(esp);
                }
                break;
            }

            case HfGpioChipType::PCAL95555_EXPANDER: {
                if (!EnsurePcal95555Handler()) {
                    Logger::GetInstance().Error(TAG, "PCAL95555 handler unavailable for %.*s",
                                               static_cast<int>(m.name.size()), m.name.data());
                    break;
                }

                hf_gpio_pull_mode_t pull = hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
                if (m.has_pull) {
                    pull = m.pull_is_up ? hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP
                                       : hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN;
                }
                hf_gpio_output_mode_t out = m.is_push_pull
                    ? hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL
                    : hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_OPEN_DRAIN;
                hf_gpio_active_state_t active = m.is_inverted
                    ? hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW
                    : hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH;

                gpio = pcal95555_handler_->CreateGpioPin(
                    static_cast<hf_pin_num_t>(m.physical_pin),
                    hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
                    active, out, pull);
                break;
            }

            case HfGpioChipType::TMC9660_CONTROLLER: {
                Tmc9660Handler* handler = GetTmc9660Handler(m.chip_unit);
                if (!handler) {
                    Logger::GetInstance().Error(TAG, "TMC9660 handler unavailable for %.*s",
                                               static_cast<int>(m.name.size()), m.name.data());
                    break;
                }
                if (m.physical_pin != 17 && m.physical_pin != 18) {
                    Logger::GetInstance().Error(TAG, "TMC9660 only supports GPIO17/18, got %u", m.physical_pin);
                    break;
                }

                // Non-owning aliasing shared_ptr — handler retains ownership
                BaseGpio& ref = handler->gpio(m.physical_pin);
                gpio = std::shared_ptr<BaseGpio>(std::shared_ptr<BaseGpio>{}, &ref);

                if (m.is_inverted) {
                    gpio->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW);
                }
                break;
            }

            default:
                continue;
        }

        if (gpio) {
            auto& entry = entries_[i];
            entry.driver = std::move(gpio);
            entry.functional_pin = static_cast<HfFunctionalGpioPin>(i);
            entry.registered = true;
            ++registered_count_;
            Logger::GetInstance().Info(TAG, "Registered GPIO: %.*s (chip=%u, pin=%u)",
                                      static_cast<int>(m.name.size()), m.name.data(),
                                      m.chip_type, m.physical_pin);
        } else {
            Logger::GetInstance().Error(TAG, "Failed to create GPIO: %.*s",
                                       static_cast<int>(m.name.size()), m.name.data());
        }
    }

    Logger::GetInstance().Info(TAG, "GPIO manager initialized: %zu pins registered", registered_count_);
    return registered_count_ > 0;
}

//==============================================================================
// HARDWARE HANDLER HELPERS
//==============================================================================

bool GpioManager::EnsurePcal95555Handler() noexcept {
    if (pcal95555_handler_) {
        return true;
    }

    auto& comm = CommChannelsManager::GetInstance();
    if (!comm.IsInitialized()) {
        return false;
    }

    auto i2c_device = comm.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    if (!i2c_device) {
        Logger::GetInstance().Error(TAG, "PCAL95555 I2C device not available");
        return false;
    }

    pcal95555_handler_ = std::make_unique<Pcal95555Handler>(*i2c_device);
    if (!pcal95555_handler_->EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "PCAL95555 handler initialization failed");
        pcal95555_handler_.reset();
        return false;
    }

    return true;
}

Tmc9660Handler* GpioManager::GetTmc9660Handler(uint8_t device_index) noexcept {
    if (!motor_controller_) {
        motor_controller_ = &MotorController::GetInstance();
    }
    if (!motor_controller_->EnsureInitialized()) {
        return nullptr;
    }
    return motor_controller_->handler(device_index);
}

//==============================================================================
// LOOKUP
//==============================================================================

size_t GpioManager::FindByName(std::string_view name) const noexcept {
    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE; ++i) {
        if (entries_[i].registered && HF_GPIO_MAPPING[i].name == name) {
            return i;
        }
    }
    return kInvalidIndex;
}

std::shared_ptr<BaseGpio> GpioManager::Get(std::string_view name) noexcept {
    if (!initialized_.load(std::memory_order_acquire)) return nullptr;
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return nullptr;
    return entries_[idx].driver;
}

std::shared_ptr<BaseGpio> GpioManager::Get(HfFunctionalGpioPin pin) noexcept {
    if (!initialized_.load(std::memory_order_acquire)) return nullptr;
    size_t idx = static_cast<size_t>(pin);
    if (idx >= kMaxEntries || !entries_[idx].registered) return nullptr;
    return entries_[idx].driver;
}

bool GpioManager::Contains(std::string_view name) const noexcept {
    return FindByName(name) != kInvalidIndex;
}

size_t GpioManager::Size() const noexcept {
    return registered_count_;
}

//==============================================================================
// BASIC GPIO OPERATIONS
//==============================================================================

hf_gpio_err_t GpioManager::Set(std::string_view name, bool value) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) { ++failed_ops_; return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND; }
    auto& e = entries_[idx];
    hf_gpio_err_t r = value ? e.driver->SetActive() : e.driver->SetInactive();
    ++total_ops_;
    if (r == hf_gpio_err_t::GPIO_SUCCESS) { ++successful_ops_; ++e.access_count; }
    else { ++failed_ops_; ++e.error_count; last_error_ = r; }
    return r;
}

hf_gpio_err_t GpioManager::SetActive(std::string_view name) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) { ++failed_ops_; return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND; }
    auto& e = entries_[idx];
    hf_gpio_err_t r = e.driver->SetActive();
    ++total_ops_;
    if (r == hf_gpio_err_t::GPIO_SUCCESS) { ++successful_ops_; ++e.access_count; }
    else { ++failed_ops_; ++e.error_count; last_error_ = r; }
    return r;
}

hf_gpio_err_t GpioManager::SetInactive(std::string_view name) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) { ++failed_ops_; return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND; }
    auto& e = entries_[idx];
    hf_gpio_err_t r = e.driver->SetInactive();
    ++total_ops_;
    if (r == hf_gpio_err_t::GPIO_SUCCESS) { ++successful_ops_; ++e.access_count; }
    else { ++failed_ops_; ++e.error_count; last_error_ = r; }
    return r;
}

hf_gpio_err_t GpioManager::Read(std::string_view name, bool& state) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) { ++failed_ops_; return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND; }
    auto& e = entries_[idx];
    hf_gpio_err_t r = e.driver->IsActive(state);
    ++total_ops_;
    if (r == hf_gpio_err_t::GPIO_SUCCESS) { ++successful_ops_; ++e.access_count; }
    else { ++failed_ops_; ++e.error_count; last_error_ = r; }
    return r;
}

hf_gpio_err_t GpioManager::Toggle(std::string_view name) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) { ++failed_ops_; return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND; }
    auto& e = entries_[idx];
    hf_gpio_err_t r = e.driver->Toggle();
    ++total_ops_;
    if (r == hf_gpio_err_t::GPIO_SUCCESS) { ++successful_ops_; ++e.access_count; }
    else { ++failed_ops_; ++e.error_count; last_error_ = r; }
    return r;
}

hf_gpio_err_t GpioManager::IsActive(std::string_view name, bool& active) noexcept {
    return Read(name, active);
}

//==============================================================================
// PIN CONFIGURATION
//==============================================================================

hf_gpio_err_t GpioManager::SetDirection(std::string_view name, hf_gpio_direction_t direction) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->SetDirection(direction);
}

hf_gpio_err_t GpioManager::SetPullMode(std::string_view name, hf_gpio_pull_mode_t pull_mode) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->SetPullMode(pull_mode);
}

hf_gpio_err_t GpioManager::SetOutputMode(std::string_view name, hf_gpio_output_mode_t mode) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->SetOutputMode(mode);
}

//==============================================================================
// INTERRUPT SUPPORT
//==============================================================================

hf_gpio_err_t GpioManager::ConfigureInterrupt(std::string_view name,
                                              hf_gpio_interrupt_trigger_t trigger,
                                              InterruptCallback callback,
                                              void* user_data) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->ConfigureInterrupt(trigger, callback, user_data);
}

hf_gpio_err_t GpioManager::EnableInterrupt(std::string_view name) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->EnableInterrupt();
}

hf_gpio_err_t GpioManager::DisableInterrupt(std::string_view name) noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->DisableInterrupt();
}

bool GpioManager::SupportsInterrupts(std::string_view name) const noexcept {
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return false;
    return entries_[idx].driver->SupportsInterrupts() == hf_gpio_err_t::GPIO_SUCCESS;
}

//==============================================================================
// DIAGNOSTICS
//==============================================================================

hf_gpio_err_t GpioManager::GetSystemDiagnostics(GpioSystemDiagnostics& d) const noexcept {
    if (!initialized_.load(std::memory_order_acquire)) {
        return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    }

    MutexLockGuard lock(mutex_);

    d.total_pins_registered = static_cast<uint32_t>(registered_count_);
    d.total_operations = total_ops_;
    d.successful_operations = successful_ops_;
    d.failed_operations = failed_ops_;
    d.last_error = last_error_;
    d.system_uptime_ms = (init_time_ms_ > 0) ? (os_get_elapsed_time_msec() - init_time_ms_) : 0;

    // Count pins by chip type and category
    std::fill(std::begin(d.pins_by_chip), std::end(d.pins_by_chip), 0u);
    std::fill(std::begin(d.pins_by_category), std::end(d.pins_by_category), 0u);

    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE; ++i) {
        if (!entries_[i].registered) continue;
        const auto& m = HF_GPIO_MAPPING[i];
        uint8_t chip_idx = m.chip_type;
        if (chip_idx < std::size(d.pins_by_chip)) {
            d.pins_by_chip[chip_idx]++;
        }
        uint8_t cat_idx = static_cast<uint8_t>(m.category);
        if (cat_idx < std::size(d.pins_by_category)) {
            d.pins_by_category[cat_idx]++;
        }
    }

    d.system_healthy = (failed_ops_ == 0);
    return hf_gpio_err_t::GPIO_SUCCESS;
}

void GpioManager::DumpStatistics() const noexcept {
    auto& log = Logger::GetInstance();
    log.Info(TAG, "=== Vortex GPIO Manager Statistics ===");
    log.Info(TAG, "  Registered pins: %zu", registered_count_);
    log.Info(TAG, "  Total ops: %u  success: %u  failed: %u",
             total_ops_, successful_ops_, failed_ops_);

    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE; ++i) {
        if (entries_[i].registered) {
            const auto& m = HF_GPIO_MAPPING[i];
            log.Info(TAG, "  [%zu] %.*s -> chip=%u pin=%u (access=%u, err=%u)",
                     i,
                     static_cast<int>(m.name.size()), m.name.data(),
                     m.chip_type, m.physical_pin,
                     entries_[i].access_count,
                     entries_[i].error_count);
        }
    }
    log.Info(TAG, "=== End Vortex GPIO Statistics ===");
}
