/**
 * @file GpioManager.h
 * @brief GPIO management for the HardFOC Vortex V1 platform.
 *
 * @details The Vortex board spans three GPIO chip types:
 *          - ESP32-C6 internal GPIOs
 *          - PCAL95555 I²C GPIO expander
 *          - TMC9660 motor-controller GPIO bridge (GPIO17, GPIO18)
 *
 *          Uses a fixed-size std::array indexed by the platform mapping index
 *          (all 32 pins known at compile time) instead of a heap-allocated
 *          unordered_map. String lookups iterate the constexpr mapping table.
 *
 *          Each entry stores a shared_ptr<BaseGpio> to accommodate the three
 *          different ownership models:
 *          - ESP32 pins: make_shared<EspGpio> (GpioManager owns)
 *          - PCAL95555 pins: shared_ptr from Pcal95555Handler::CreateGpioPin()
 *          - TMC9660 pins: non-owning aliasing shared_ptr (handler owns)
 *
 * @version 2.0
 */

#ifndef VORTEX_GPIO_MANAGER_H_
#define VORTEX_GPIO_MANAGER_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string_view>

#include "base/BaseGpio.h"
#include "RtosMutex.h"
#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class EspGpio;
class Pcal95555Handler;
class Tmc9660Handler;
class MotorController;

//==============================================================================
// GPIO ENTRY — one slot per pin in the fixed-size array
//==============================================================================

/**
 * @brief A single GPIO entry in the fixed-size registry.
 *
 * Stores a shared_ptr<BaseGpio> for polymorphic ownership across chip types.
 * The GpioEntry structs live in a fixed-size std::array inside the singleton.
 */
struct GpioEntry {
    std::shared_ptr<BaseGpio> driver;          ///< Polymorphic GPIO driver
    HfFunctionalGpioPin functional_pin;        ///< Functional pin enum
    bool registered{false};                    ///< Whether this slot is active
    uint32_t access_count{0};                  ///< Operation counter
    uint32_t error_count{0};                   ///< Error counter

    GpioEntry() noexcept
        : driver(nullptr),
          functional_pin(HfFunctionalGpioPin::HF_FUNCTIONAL_GPIO_COUNT) {}
};

//==============================================================================
// DIAGNOSTICS STRUCTURE
//==============================================================================

/**
 * @brief System-level GPIO diagnostics.
 */
struct GpioSystemDiagnostics {
    bool system_healthy;
    uint32_t total_pins_registered;
    uint32_t pins_by_chip[static_cast<uint8_t>(HfGpioChipType::TMC9660_CONTROLLER) + 1];
    uint32_t pins_by_category[4];
    uint32_t total_operations;
    uint32_t successful_operations;
    uint32_t failed_operations;
    uint64_t system_uptime_ms;
    hf_gpio_err_t last_error;
};

//==============================================================================
// GPIO MANAGER
//==============================================================================

class GpioManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================

    static GpioManager& GetInstance() noexcept;

    GpioManager(const GpioManager&) = delete;
    GpioManager& operator=(const GpioManager&) = delete;
    GpioManager(GpioManager&&) = delete;
    GpioManager& operator=(GpioManager&&) = delete;

    [[nodiscard]] bool EnsureInitialized() noexcept;
    [[nodiscard]] bool IsInitialized() const noexcept { return initialized_.load(std::memory_order_acquire); }

    //==========================================================================
    // GPIO ACCESS — by string name or functional enum
    //==========================================================================

    /** @brief Get the BaseGpio driver for a named pin. Returns nullptr if not found. */
    [[nodiscard]] std::shared_ptr<BaseGpio> Get(std::string_view name) noexcept;

    /** @brief Get the BaseGpio driver for a functional pin enum. Returns nullptr if not found. */
    [[nodiscard]] std::shared_ptr<BaseGpio> Get(HfFunctionalGpioPin pin) noexcept;

    /** @brief Check if a named pin exists in the registry. */
    [[nodiscard]] bool Contains(std::string_view name) const noexcept;

    /** @brief Number of registered GPIO pins. */
    [[nodiscard]] size_t Size() const noexcept;

    //==========================================================================
    // BASIC GPIO OPERATIONS — string-based routing
    //==========================================================================

    [[nodiscard]] hf_gpio_err_t Set(std::string_view name, bool value) noexcept;
    [[nodiscard]] hf_gpio_err_t SetActive(std::string_view name) noexcept;
    [[nodiscard]] hf_gpio_err_t SetInactive(std::string_view name) noexcept;
    [[nodiscard]] hf_gpio_err_t Read(std::string_view name, bool& state) noexcept;
    [[nodiscard]] hf_gpio_err_t Toggle(std::string_view name) noexcept;
    [[nodiscard]] hf_gpio_err_t IsActive(std::string_view name, bool& active) noexcept;

    //==========================================================================
    // PIN CONFIGURATION
    //==========================================================================

    [[nodiscard]] hf_gpio_err_t SetDirection(std::string_view name, hf_gpio_direction_t direction) noexcept;
    [[nodiscard]] hf_gpio_err_t SetPullMode(std::string_view name, hf_gpio_pull_mode_t pull_mode) noexcept;
    [[nodiscard]] hf_gpio_err_t SetOutputMode(std::string_view name, hf_gpio_output_mode_t mode) noexcept;

    //==========================================================================
    // INTERRUPT SUPPORT — forwarded to underlying BaseGpio driver
    //==========================================================================

    [[nodiscard]] hf_gpio_err_t ConfigureInterrupt(std::string_view name,
                                                   hf_gpio_interrupt_trigger_t trigger,
                                                   InterruptCallback callback,
                                                   void* user_data = nullptr) noexcept;
    [[nodiscard]] hf_gpio_err_t EnableInterrupt(std::string_view name) noexcept;
    [[nodiscard]] hf_gpio_err_t DisableInterrupt(std::string_view name) noexcept;
    [[nodiscard]] bool SupportsInterrupts(std::string_view name) const noexcept;

    //==========================================================================
    // DIAGNOSTICS
    //==========================================================================

    [[nodiscard]] hf_gpio_err_t GetSystemDiagnostics(GpioSystemDiagnostics& diagnostics) const noexcept;
    void DumpStatistics() const noexcept;

private:
    GpioManager() noexcept = default;
    ~GpioManager() = default;

    [[nodiscard]] bool Initialize() noexcept;

    /** @brief Find entry index by string name. Returns kInvalidIndex if not found. */
    [[nodiscard]] size_t FindByName(std::string_view name) const noexcept;

    /** @brief Lazy-init the PCAL95555 handler (needs I²C from CommChannelsManager). */
    [[nodiscard]] bool EnsurePcal95555Handler() noexcept;

    /** @brief Get TMC9660 handler from MotorController. */
    [[nodiscard]] Tmc9660Handler* GetTmc9660Handler(uint8_t device_index = 0) noexcept;

    //--------------------------------------------------------------------------
    // Storage
    //--------------------------------------------------------------------------

    static constexpr size_t kMaxEntries =
        static_cast<size_t>(HfFunctionalGpioPin::HF_FUNCTIONAL_GPIO_COUNT);
    static constexpr size_t kInvalidIndex = SIZE_MAX;

    std::array<GpioEntry, kMaxEntries> entries_{};
    size_t registered_count_{0};
    uint64_t init_time_ms_{0};

    uint32_t total_ops_{0};
    uint32_t successful_ops_{0};
    uint32_t failed_ops_{0};
    hf_gpio_err_t last_error_{hf_gpio_err_t::GPIO_SUCCESS};

    std::atomic<bool> initialized_{false};
    mutable RtosMutex mutex_;

    //--------------------------------------------------------------------------
    // Hardware handlers
    //--------------------------------------------------------------------------

    std::unique_ptr<Pcal95555Handler> pcal95555_handler_;
    MotorController* motor_controller_{nullptr};
};

//==============================================================================
// CONVENIENCE
//==============================================================================

[[nodiscard]] inline GpioManager& GetGpioManager() noexcept {
    return GpioManager::GetInstance();
}

#endif // VORTEX_GPIO_MANAGER_H_
