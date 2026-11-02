/**
 * @file GpioManager.cpp
 * @brief Implementation of the Vortex V1 GPIO manager.
 *
 * @details Multi-chip GPIO initialization:
 *          - ESP32 internal: created by the board binding (vortex::bsp::McuGpioOpen)
 *          - PCAL95555 expander: Pcal95555Handler::CreateGpioPin(...) — includes host→TMC control
 *            outputs (`PCAL_TMC_DRV_EN`, `PCAL_TMC_RST_CTRL`, `PCAL_TMC_SPI_COMM_EN`,
 *            `PCAL_TMC_WAKE_CTRL`) and inputs (`PCAL_TMC_FAULT_STATUS`,
 *            `PCAL_PWR_GOOD`, `PCAL_IMU_INT`), plus TMC GPIO17/18 on PCAL P1–P2 as `PCAL_TMC_GPIO17_EXP_IN` /
 *            `PCAL_TMC_GPIO18_EXP_IN`.
 *          - TMC9660 controller: non-owning alias to handler->gpio(17|18), registered after
 *            MotorController::CreateOnboardDevice() via RegisterTmc9660BridgePinsIfNeeded()
 *
 * @version 2.0
 */

#include "GpioManager.h"
#include "CommChannelsManager.h"
#include "MotorController.h"
#include "handlers/pcal95555/Pcal95555Handler.h"
#include "handlers/tmc9660/Tmc9660Handler.h"
#include "bsp_bind/VortexBoardBind.h"
#include "handlers/logger/Logger.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"

static constexpr const char* TAG = "VortexGpio";

/** PCAL95555 lines that are inputs on Vortex (fault, power-good, IMU INT, TMC GPIO17/18 feeds). */
static hf_gpio_direction_t pcal_pin_direction(size_t map_index) noexcept {
    const auto pin = static_cast<HfFunctionalGpioPin>(map_index);
    switch (pin) {
        case HfFunctionalGpioPin::PCAL_TMC_FAULT_STATUS:
        case HfFunctionalGpioPin::PCAL_PWR_GOOD:
        case HfFunctionalGpioPin::PCAL_IMU_INT:
        case HfFunctionalGpioPin::PCAL_TMC_GPIO17_EXP_IN:
        case HfFunctionalGpioPin::PCAL_TMC_GPIO18_EXP_IN:
            return hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT;
        default:
            return hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT;
    }
}

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

bool GpioManager::Deinitialize() noexcept {
    if (!initialized_.load(std::memory_order_acquire)) return true;
    MutexLockGuard lock(mutex_);
    Logger::GetInstance().Info(TAG, "Deinitializing Vortex GPIO manager");

    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE; ++i) {
        entries_[i].driver.reset();
        entries_[i].registered = false;
        entries_[i].access_count = 0;
        entries_[i].error_count = 0;
    }
    registered_count_ = 0;
    total_ops_ = 0;
    successful_ops_ = 0;
    failed_ops_ = 0;
    last_error_ = hf_gpio_err_t::GPIO_SUCCESS;
    if (pcal95555_handler_) {
        (void)pcal95555_handler_->EnsureDeinitialized();
    }
    pcal95555_handler_.reset();
    pcal_host_int_gpio_ = nullptr;
    vortex::bsp::McuGpioCloseAll();
    initialized_.store(false, std::memory_order_release);
    return true;
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

    // Bring up PCAL95555 on I²C before MotorController so the first expander traffic is
    // not interleaved with long SPI/TMCL bring-up (matches hf-pcal95555-driver examples).
    if (!EnsurePcal95555Handler()) {
        Logger::GetInstance().Warn(
            TAG,
            "PCAL95555 early init failed — expander GPIO will be unavailable until bus is fixed");
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

                // The binding owns the pin object (static storage); the entry holds a
                // non-owning alias of it.
                BaseGpio* const mcu = vortex::bsp::McuGpioOpen(
                    static_cast<HfFunctionalGpioPin>(i),
                    hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT, active, out, pull);
                if (mcu != nullptr) {
                    gpio = std::shared_ptr<BaseGpio>(std::shared_ptr<BaseGpio>{}, mcu);
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
                    pcal_pin_direction(i),
                    active, out, pull);
                break;
            }

            case HfGpioChipType::TMC9660_CONTROLLER:
                // The TMC9660 bridge pins (GPIO17/18) belong to the MotorController handler, which
                // does not exist yet: Vortex brings GPIO up before motors. They are registered by
                // RegisterTmc9660BridgePinsIfNeeded() once the onboard handler exists.
                continue;

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

    // Host GPIO for expander /INT (open-drain, active low). Vortex pincfg: GPIO23.
    if (!pcal_host_int_gpio_) {
        const HfGpioMapping* m = GetGpioMapping(HfFunctionalGpioPin::I2C_PCAL95555_INT);
        if (m != nullptr) {
            const hf_gpio_pull_mode_t pull =
                m->has_pull ? (m->pull_is_up ? hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_UP
                                             : hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_DOWN)
                            : hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
            const hf_gpio_active_state_t active =
                m->is_inverted ? hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW
                               : hf_gpio_active_state_t::HF_GPIO_ACTIVE_HIGH;
            const hf_gpio_output_mode_t out_mode =
                m->is_push_pull ? hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL
                                : hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_OPEN_DRAIN;
            BaseGpio* const mcu = vortex::bsp::McuGpioOpen(HfFunctionalGpioPin::I2C_PCAL95555_INT,
                                                          hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT,
                                                          active, out_mode, pull);
            if (mcu != nullptr) {
                pcal_host_int_gpio_ = mcu;
                Logger::GetInstance().Info(
                    TAG,
                    "PCAL95555 host INT: GPIO%u (COMM_I2C_PCAL95555_INT); A0–A2 tied GND → 7-bit 0x20",
                    static_cast<unsigned>(m->physical_pin));
            } else {
                Logger::GetInstance().Warn(
                    TAG,
                    "PCAL95555 host INT GPIO%u init failed — handler uses polling for expander IRQ",
                    static_cast<unsigned>(m->physical_pin));
            }
        }
    }

    os_delay_msec(1);

    BaseGpio* const int_pin = pcal_host_int_gpio_;
    pcal95555_handler_ = std::make_unique<Pcal95555Handler>(*i2c_device, int_pin);
    if (!pcal95555_handler_->EnsureInitialized()) {
        Logger::GetInstance().Error(
            TAG,
            "PCAL95555 handler initialization failed (I2C 0x20, pull-ups, INT GPIO23, chip power)");
        pcal95555_handler_.reset();
        return false;
    }

    return true;
}

hf_gpio_err_t GpioManager::DebugReadPcal95555Pin(uint8_t expander_pin, bool& out_level) noexcept {
    if (expander_pin >= 16) {
        return hf_gpio_err_t::GPIO_ERR_INVALID_PIN;
    }
    if (!EnsurePcal95555Handler()) {
        return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    }
    if (!pcal95555_handler_->EnsureInitialized()) {
        return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    }
    return pcal95555_handler_->ReadInput(expander_pin, out_level);
}

hf_gpio_err_t GpioManager::DebugReadPcal95555InputSnapshot(uint16_t& out_bits) noexcept {
    out_bits = 0;
    if (!EnsurePcal95555Handler()) {
        return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    }
    if (!pcal95555_handler_->EnsureInitialized()) {
        return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    }
    return pcal95555_handler_->ReadAllInputLevels(out_bits);
}

void GpioManager::RegisterTmc9660BridgePinsIfNeeded() noexcept {
    // Lock order is MotorController -> GpioManager (MotorController drives PCAL_TMC_SPI_COMM_EN
    // through this manager while it holds its own lock), so look the handler up before taking ours.
    Tmc9660Handler* const h = MotorController::GetInstance().handler(0);
    if (!h) {
        return;
    }
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) {
        return;
    }

    for (size_t i = 0; i < HF_GPIO_MAPPING_SIZE; ++i) {
        const auto& m = HF_GPIO_MAPPING[i];
        const auto category = static_cast<HfPinCategory>(m.category);
        if (!should_register_as_gpio(category)) {
            continue;
        }
        if (m.chip_unit != 0 || m.gpio_bank != 0) {
            continue;
        }
        if (static_cast<HfGpioChipType>(m.chip_type) != HfGpioChipType::TMC9660_CONTROLLER) {
            continue;
        }
        if (m.physical_pin != 17 && m.physical_pin != 18) {
            continue;
        }
        if (entries_[i].registered) {
            continue;
        }

        BaseGpio& ref = h->gpio(static_cast<uint8_t>(m.physical_pin));
        std::shared_ptr<BaseGpio> gpio = std::shared_ptr<BaseGpio>(std::shared_ptr<BaseGpio>{}, &ref);
        if (m.is_inverted) {
            gpio->SetActiveState(hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW);
        }

        auto& entry = entries_[i];
        entry.driver = std::move(gpio);
        entry.functional_pin = static_cast<HfFunctionalGpioPin>(i);
        entry.registered = true;
        ++registered_count_;
        Logger::GetInstance().Info(TAG, "Registered GPIO: %.*s (chip=%u, pin=%u) (TMC bridge, deferred)",
                                  static_cast<int>(m.name.size()), m.name.data(), m.chip_type,
                                  m.physical_pin);
    }
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
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return nullptr;
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return nullptr;
    return entries_[idx].driver;
}

std::shared_ptr<BaseGpio> GpioManager::Get(HfFunctionalGpioPin pin) noexcept {
    const size_t idx = static_cast<size_t>(pin);
    if (idx >= kMaxEntries || !initialized_.load(std::memory_order_acquire)) return nullptr;
    if (pin == HfFunctionalGpioPin::TMC_GPIO17 || pin == HfFunctionalGpioPin::TMC_GPIO18) {
        // The TMC9660 bridge pins appear once MotorController has its onboard handler.
        // Registering takes the MotorController lock before ours (see RegisterTmc9660BridgePinsIfNeeded).
        RegisterTmc9660BridgePinsIfNeeded();
    }
    MutexLockGuard lock(mutex_);
    if (!entries_[idx].registered) return nullptr;
    return entries_[idx].driver;
}

bool GpioManager::Contains(std::string_view name) const noexcept {
    MutexLockGuard lock(mutex_);
    return FindByName(name) != kInvalidIndex;
}

size_t GpioManager::Size() const noexcept {
    MutexLockGuard lock(mutex_);
    return registered_count_;
}

//==============================================================================
// BASIC GPIO OPERATIONS
//==============================================================================

hf_gpio_err_t GpioManager::Set(std::string_view name, bool value) noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
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
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
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
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
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
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
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
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
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
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->SetDirection(direction);
}

hf_gpio_err_t GpioManager::SetPullMode(std::string_view name, hf_gpio_pull_mode_t pull_mode) noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->SetPullMode(pull_mode);
}

hf_gpio_err_t GpioManager::SetOutputMode(std::string_view name, hf_gpio_output_mode_t mode) noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
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
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->ConfigureInterrupt(trigger, callback, user_data);
}

hf_gpio_err_t GpioManager::EnableInterrupt(std::string_view name) noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->EnableInterrupt();
}

hf_gpio_err_t GpioManager::DisableInterrupt(std::string_view name) noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    size_t idx = FindByName(name);
    if (idx == kInvalidIndex) return hf_gpio_err_t::GPIO_ERR_PIN_NOT_FOUND;
    return entries_[idx].driver->DisableInterrupt();
}

bool GpioManager::SupportsInterrupts(std::string_view name) const noexcept {
    MutexLockGuard lock(mutex_);
    if (!initialized_.load(std::memory_order_acquire)) return false;
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
