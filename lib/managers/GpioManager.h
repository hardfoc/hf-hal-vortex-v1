/**
 * @file GpioManager.h
 * @brief Advanced GPIO management system for the HardFOC platform.
 * 
 * @details This class provides a comprehensive GPIO management system that integrates
 *          with the platform mapping system to automatically manage GPIOs from multiple
 *          hardware sources (ESP32-C6, PCAL95555, TMC9660) based on functional pin
 *          identifiers and hardware chip mappings.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 * 
 * Key Features:
 * - Platform mapping integration for automatic pin discovery
 * - Multi-chip GPIO management (ESP32, PCAL95555, TMC9660)
 * - Functional pin abstraction with hardware-agnostic API
 * - Thread-safe operation with comprehensive error handling
 * - Automatic pin registration based on platform configuration
 * - Advanced diagnostics and health monitoring
 * - Batch operations for performance optimization
 * - Hardware resource validation and conflict detection
 * - String-based pin identification for extensibility
 * - Smart pin categorization (CORE, COMM, GPIO)
 * - Complete BaseGpio function coverage through string-based routing
 * 
 * Architecture:
 * - Uses string_view for pin identification (extensible)
 * - Integrates with platform mapping for hardware mapping
 * - Supports all HardwareChip types defined in platform mapping
 * - Provides unified BaseGpio interface for all pin operations
 * - Handler-based GPIO creation for proper ownership
 * - Routes all BaseGpio functions through string-based API
 * 
 * @note This class is thread-safe and designed for concurrent access from multiple tasks.
 * @note All pin operations use string_view identifiers for maximum flexibility.
 */

#ifndef COMPONENT_HANDLER_GPIO_MANAGER_H_
#define COMPONENT_HANDLER_GPIO_MANAGER_H_

#include "CommonIDs.h"
#include "ThingsToString.h"
#include "base/BaseGpio.h"
#include "SfI2cBus.h"
#include "Tmc9660MotorController.h"
#include "Pcal95555Handler.h"
#include "MotorController.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspGpio.h"

#include <array>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <string_view>
#include <unordered_map>
#include <optional>

//==============================================================================
// FORWARD DECLARATIONS
//==============================================================================

class Pcal95555Handler;
class Tmc9660Handler;

//==============================================================================
// GPIO INFORMATION STRUCTURES
//==============================================================================

/**
 * @brief Structure containing comprehensive GPIO pin information with platform mapping.
 */
struct GpioInfo {
    std::string_view name;                      ///< Human-readable name (string_view to static data)
    std::shared_ptr<BaseGpio> gpio_driver;      ///< GPIO driver instance (shared ownership)
    HfFunctionalGpioPin functional_pin;         ///< Functional pin identifier
    HfPinCategory category;                     ///< Pin category (CORE, COMM, GPIO, USER)
    HfGpioChipType hardware_chip;               ///< Hardware chip identifier
    uint8_t hardware_pin_id;                    ///< Hardware pin ID within the chip
    bool is_registered;                         ///< Registration status
    bool is_input;                              ///< Pin direction (true = input, false = output)
    bool current_state;                         ///< Last known pin state
    uint32_t access_count;                      ///< Number of times accessed
    uint32_t error_count;                       ///< Number of errors encountered
    uint64_t last_access_time;                  ///< Timestamp of last access
    
    /**
     * @brief Constructor for GpioInfo.
     */
    GpioInfo(std::string_view n, std::shared_ptr<BaseGpio> driver, 
             HfFunctionalGpioPin fp, HfPinCategory cat, HfGpioChipType chip, uint8_t pin_id) noexcept
        : name(n), gpio_driver(std::move(driver)), functional_pin(fp), category(cat),
          hardware_chip(chip), hardware_pin_id(pin_id), is_registered(true),
          is_input(false), current_state(false), access_count(0), 
          error_count(0), last_access_time(0) {}
    
    // Disable copy operations due to shared_ptr
    GpioInfo(const GpioInfo&) = delete;
    GpioInfo& operator=(const GpioInfo&) = delete;
    
    // Enable move operations
    GpioInfo(GpioInfo&&) = default;
    GpioInfo& operator=(GpioInfo&&) = default;
};

/**
 * @brief Structure for GPIO batch operation specifications.
 */
struct GpioBatchOperation {
    std::vector<std::string_view> pin_names;    ///< Pin names to operate on
    std::vector<bool> states;                   ///< Desired states (for write operations)
    bool is_write_operation;                    ///< true for write, false for read
    
    /**
     * @brief Constructor for write operations.
     */
    GpioBatchOperation(std::vector<std::string_view> names, std::vector<bool> s) noexcept
        : pin_names(std::move(names)), states(std::move(s)), is_write_operation(true) {}
    
    /**
     * @brief Constructor for read operations.
     */
    explicit GpioBatchOperation(std::vector<std::string_view> names) noexcept
        : pin_names(std::move(names)), is_write_operation(false) {}
};

/**
 * @brief Structure for GPIO batch operation results.
 */
struct GpioBatchResult {
    std::vector<std::string_view> pin_names;    ///< Pin names operated on
    std::vector<bool> states;                   ///< Resulting states
    std::vector<hf_gpio_err_t> results;         ///< Individual operation results
    hf_gpio_err_t overall_result;               ///< Overall operation result
    
    /**
     * @brief Check if all operations were successful.
     */
    [[nodiscard]] bool AllSuccessful() const noexcept {
        return overall_result == hf_gpio_err_t::GPIO_SUCCESS;
    }
};

/**
 * @brief Structure for GPIO system diagnostics.
 */
struct GpioSystemDiagnostics {
    bool system_healthy;                           ///< Overall system health
    uint32_t total_pins_registered;                ///< Total pins registered
    uint32_t pins_by_chip[static_cast<uint8_t>(HfGpioChipType::TMC9660_CONTROLLER) + 1]; ///< Pins per chip
    uint32_t pins_by_category[4];                  ///< Pins per category (CORE, COMM, GPIO, USER)
    uint32_t total_operations;                     ///< Total operations performed
    uint32_t successful_operations;                ///< Successful operations
    uint32_t failed_operations;                    ///< Failed operations
    uint32_t communication_errors;                 ///< Communication errors
    uint32_t hardware_errors;                      ///< Hardware errors
    uint64_t system_uptime_ms;                     ///< System uptime
    hf_gpio_err_t last_error;                      ///< Last error encountered
};

//==============================================================================
// MAIN GPIO MANAGER CLASS
//==============================================================================

/**
 * @class GpioManager
 * @brief Advanced GPIO management system for the HardFOC platform.
 * 
 * This class provides a comprehensive GPIO management system that integrates
 * with the platform mapping system to automatically manage GPIOs from multiple
 * hardware sources. It uses string_view identifiers and hardware chip
 * mappings to provide a unified, hardware-agnostic API that routes all
 * BaseGpio functions through string-based pin identification.
 * 
 * Thread Safety:
 * - All public methods are thread-safe
 * - Uses internal mutex for protection
 * - Atomic operations where appropriate
 * 
 * Error Handling:
 * - Core operations return simple bool values for embedded system efficiency
 * - Configuration operations return hf_gpio_err_t for detailed error codes
 * - Comprehensive error codes via ResultCode enum
 * - Detailed error descriptions and diagnostics
 * 
 * Performance:
 * - Optimized for common operations
 * - Batch operations for multiple pins
 * - Cached state information
 * - Lazy initialization of hardware resources
 * 
 * Platform Integration:
 * - Automatic pin discovery via platform mapping
 * - Hardware resource validation
 * - Conflict detection and resolution
 * - Multi-chip coordination
 * - Smart pin categorization (CORE, COMM, GPIO)
 * 
 * Function Coverage:
 * - Complete BaseGpio function coverage through string-based routing
 * - All GPIO operations available via string_view pin names
 * - Consistent API design with proper camelCase naming
 */
class GpioManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================
    
    /**
     * @brief Get the singleton instance.
     * @return Reference to the GPIO manager instance
     */
    static GpioManager& GetInstance() noexcept;
    
    /**
     * @brief Ensure the GPIO manager system is initialized.
     * @return true if initialization successful, false otherwise
     */
    [[nodiscard]] bool EnsureInitialized() noexcept;
    
    /**
     * @brief Shutdown the GPIO manager system.
     * @return true if shutdown successful, false otherwise
     */
    [[nodiscard]] bool Shutdown() noexcept;
    
    /**
     * @brief Check if the GPIO system is initialized.
     * @return true if initialized, false otherwise
     */
    [[nodiscard]] bool IsInitialized() const noexcept;
    
    /**
     * @brief Get system diagnostics and health information.
     * @param diagnostics Reference to store system diagnostics
     * @return hf_gpio_err_t::GPIO_SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] hf_gpio_err_t GetSystemDiagnostics(GpioSystemDiagnostics& diagnostics) const noexcept;
    
    //==========================================================================
    // GPIO REGISTRATION AND MANAGEMENT
    //==========================================================================
    
    /**
     * @brief Register a GPIO pin with the system using string_view identifier.
     * @param name Pin name (must be static string or outlive the manager)
     * @param gpio Shared pointer to GPIO driver
     * @return hf_gpio_err_t::GPIO_SUCCESS if successful, error code otherwise
     * @note Pin names must be static strings (string literals or static arrays)
     * @note Reserved prefixes (CORE_, COMM_, SYS_, INTERNAL_) are not allowed
     */
    [[nodiscard]] hf_gpio_err_t RegisterGpio(std::string_view name, std::shared_ptr<BaseGpio> gpio) noexcept;
    
    /**
     * @brief Get a GPIO pin by name.
     * @param name Pin name
     * @return Shared pointer to GPIO or nullptr if not found
     */
    [[nodiscard]] std::shared_ptr<BaseGpio> Get(std::string_view name) noexcept;
    
    /**
     * @brief Check if a GPIO pin is registered.
     * @param name Pin name
     * @return true if registered, false otherwise
     */
    [[nodiscard]] bool Contains(std::string_view name) const noexcept;
    
    /**
     * @brief Get count of registered pins.
     * @return Number of registered pins
     */
    [[nodiscard]] size_t Size() const noexcept;
    
    /**
     * @brief Log all registered GPIO pins for debugging.
     */
    void LogAllRegisteredGpios() const noexcept;
    
    //==========================================================================
    // BASIC GPIO OPERATIONS (Complete BaseGpio Coverage)
    //==========================================================================
    
    /**
     * @brief Set a GPIO pin to a specific state.
     * @param name Pin name
     * @param value Desired state (true = active, false = inactive)
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t Set(std::string_view name, bool value) noexcept;
    
    /**
     * @brief Set a GPIO pin to active state.
     * @param name Pin name
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t SetActive(std::string_view name) noexcept;
    
    /**
     * @brief Set a GPIO pin to inactive state.
     * @param name Pin name
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t SetInactive(std::string_view name) noexcept;
    
    /**
     * @brief Read the current state of a GPIO pin.
     * @param name Pin name
     * @param state Reference to store the read state
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t Read(std::string_view name, bool& state) noexcept;
    
    /**
     * @brief Toggle a GPIO pin state.
     * @param name Pin name
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t Toggle(std::string_view name) noexcept;
    
    /**
     * @brief Check if a GPIO pin is in active state.
     * @param name Pin name
     * @param active Reference to store the active state
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t IsActive(std::string_view name, bool& active) noexcept;
    
    //==========================================================================
    // PIN CONFIGURATION (Complete BaseGpio Coverage)
    //==========================================================================
    
    /**
     * @brief Configure pin direction.
     * @param name Pin name
     * @param direction Pin direction
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t SetDirection(std::string_view name, hf_gpio_direction_t direction) noexcept;
    
    /**
     * @brief Configure pin pull-up/pull-down.
     * @param name Pin name
     * @param pull_mode Pull mode configuration
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t SetPullMode(std::string_view name, hf_gpio_pull_mode_t pull_mode) noexcept;
    
    /**
     * @brief Configure pin output mode.
     * @param name Pin name
     * @param output_mode Output mode configuration
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t SetOutputMode(std::string_view name, hf_gpio_output_mode_t output_mode) noexcept;
    
    /**
     * @brief Get pin direction.
     * @param name Pin name
     * @param direction Reference to store direction
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t GetDirection(std::string_view name, hf_gpio_direction_t& direction) const noexcept;
    
    /**
     * @brief Get pin pull mode.
     * @param name Pin name
     * @param pull_mode Reference to store pull mode
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t GetPullMode(std::string_view name, hf_gpio_pull_mode_t& pull_mode) const noexcept;
    
    /**
     * @brief Get pin output mode.
     * @param name Pin name
     * @param output_mode Reference to store output mode
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t GetOutputMode(std::string_view name, hf_gpio_output_mode_t& output_mode) const noexcept;
    
    //==========================================================================
    // INTERRUPT SUPPORT (Complete BaseGpio Coverage)
    //==========================================================================
    
    /**
     * @brief Configure interrupt for a pin.
     * @param name Pin name
     * @param trigger Interrupt trigger type
     * @param callback Interrupt callback function
     * @param user_data User data for callback
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t ConfigureInterrupt(std::string_view name,
                                                   hf_gpio_interrupt_trigger_t trigger,
                                                   BaseGpio::InterruptCallback callback,
                                                   void* user_data = nullptr) noexcept;
    
    /**
     * @brief Enable interrupt for a pin.
     * @param name Pin name
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t EnableInterrupt(std::string_view name) noexcept;
    
    /**
     * @brief Disable interrupt for a pin.
     * @param name Pin name
     * @return hf_gpio_err_t error code
     */
    [[nodiscard]] hf_gpio_err_t DisableInterrupt(std::string_view name) noexcept;
    
    /**
     * @brief Check if pin supports interrupts.
     * @param name Pin name
     * @return true if interrupts are supported, false otherwise
     */
    [[nodiscard]] bool SupportsInterrupts(std::string_view name) const noexcept;
    
    //==========================================================================
    // STATISTICS AND DIAGNOSTICS (Complete BaseGpio Coverage)
    //==========================================================================
    
    /**
     * @brief Get pin statistics.
     * @param name Pin name
     * @param statistics Reference to store pin statistics
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t GetStatistics(std::string_view name, BaseGpio::PinStatistics& statistics) const noexcept;
    
    /**
     * @brief Reset pin statistics.
     * @param name Pin name
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t ResetStatistics(std::string_view name) noexcept;
    
    /**
     * @brief Dump comprehensive system statistics to log as INFO level.
     * Logs all pin statistics, system health, and operation counters.
     */
    void DumpStatistics() const noexcept;

private:
    /**
     * @brief Get count of active pins (helper for statistics).
     * @return Number of pins with valid GPIO instances
     */
    int GetActivePinCount() const noexcept;

public:
    
    //==========================================================================
    // BATCH OPERATIONS
    //==========================================================================
    
    /**
     * @brief Perform batch write operations on multiple pins.
     * @param operation Batch operation specification
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] GpioBatchResult BatchWrite(const GpioBatchOperation& operation) noexcept;
    
    /**
     * @brief Perform batch read operations on multiple pins.
     * @param pin_names Vector of pin names to read
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] GpioBatchResult BatchRead(const std::vector<std::string_view>& pin_names) noexcept;
    
    /**
     * @brief Set multiple pins to active state.
     * @param pin_names Vector of pin names
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] GpioBatchResult SetMultipleActive(const std::vector<std::string_view>& pin_names) noexcept;
    
    /**
     * @brief Set multiple pins to inactive state.
     * @param pin_names Vector of pin names
     * @return Batch operation results with individual and overall status
     */
    [[nodiscard]] GpioBatchResult SetMultipleInactive(const std::vector<std::string_view>& pin_names) noexcept;
    
    //==========================================================================
    // SYSTEM INFORMATION
    //==========================================================================
    
    /**
     * @brief Reset all output pins to inactive state.
     * @return hf_gpio_err_t operation result
     */
    [[nodiscard]] hf_gpio_err_t ResetAllPins() noexcept;

private:
    //==========================================================================
    // PRIVATE MEMBERS
    //==========================================================================
    
    /**
     * @brief Initialize the GPIO manager system with platform pin registration.
     * 
     * This private method performs the actual initialization including:
     * - CommChannelsManager dependency validation
     * - Smart pin categorization and registration
     * - Lazy initialization of hardware handlers
     * - Thread-safe initialization with RtosMutex
     * 
     * @return true if initialization successful, false otherwise
     */
    [[nodiscard]] bool Initialize() noexcept;
    
    // ===============================
    // SYSTEM STATE
    // ===============================
    
    /**
     * @brief System initialization state (atomic for thread safety).
     */
    std::atomic<bool> is_initialized_{false};
    
    /**
     * @brief Main system mutex for thread-safe operations.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex mutex_;
    
    // ===============================
    // GPIO STORAGE
    // ===============================
    
    /**
     * @brief String-based GPIO registry with shared_ptr ownership.
     * Uses string_view keys pointing to static strings for embedded efficiency.
     * Protected by RtosMutex for thread-safe access.
     */
    std::unordered_map<std::string_view, std::shared_ptr<BaseGpio>> gpio_registry_;
    mutable RtosMutex registry_mutex_;  ///< RtosMutex for registry access
    
    // ===============================
    // HARDWARE HANDLERS (LAZY INITIALIZATION)
    // ===============================
    
    /**
     * @brief PCAL95555 GPIO expander handler (lazy initialized when first accessed).
     * Single instance manages all PCAL95555 pins to prevent duplicate controllers.
     * Uses CommChannelsManager for I2C access.
     */
    std::unique_ptr<Pcal95555Handler> pcal95555_handler_;
    mutable RtosMutex pcal_handler_mutex_;  ///< Mutex for PCAL95555 handler initialization
    
    /**
     * @brief Reference to MotorController for TMC9660 handler access.
     * MotorController owns all Tmc9660Handler instances - GpioManager gets access through it.
     * This ensures proper ownership hierarchy and prevents duplicate handlers.
     * 
     * Ownership Model:
     * - MotorController: Sole owner of Tmc9660Handler instances (unique_ptr)
     * - GpioManager: Consumer that gets access via raw pointers
     * - Handler lifetime: Tied to MotorController lifecycle
     */
    MotorController* motor_controller_ = nullptr;
    
    // ===============================
    // SYSTEM STATISTICS
    // ===============================
    
    /**
     * @brief Total operations performed (atomic for thread safety).
     */
    std::atomic<uint32_t> total_operations_{0};
    
    /**
     * @brief Successful operations count (atomic for thread safety).
     */
    std::atomic<uint32_t> successful_operations_{0};
    
    /**
     * @brief Failed operations count (atomic for thread safety).
     */
    std::atomic<uint32_t> failed_operations_{0};
    
    /**
     * @brief Communication errors count (atomic for thread safety).
     */
    std::atomic<uint32_t> communication_errors_{0};
    
    /**
     * @brief Hardware errors count (atomic for thread safety).
     */
    std::atomic<uint32_t> hardware_errors_{0};
    
    /**
     * @brief System start time for uptime calculations (atomic for thread safety).
     */
    std::atomic<uint64_t> 0{0};
    
    // ===============================
    // ERROR TRACKING
    // ===============================
    
    /**
     * @brief Thread-safe access to error message storage.
     * Uses RtosMutex for embedded RTOS compatibility.
     */
    mutable RtosMutex error_mutex_;
    
    /**
     * @brief Last error encountered (for diagnostics only).
     * Single error code instead of error message storage.
     */
    std::atomic<hf_gpio_err_t> last_error_{hf_gpio_err_t::GPIO_SUCCESS};
    
    //==========================================================================
    // PRIVATE METHODS
    //==========================================================================
    
    /**
     * @brief Private constructor for singleton pattern.
     */
    GpioManager() = default;
    
    /**
     * @brief Private destructor.
     */
    ~GpioManager() = default;
    
    // Disable copy and move operations
    GpioManager(const GpioManager&) = delete;
    GpioManager& operator=(const GpioManager&) = delete;
    GpioManager(GpioManager&&) = delete;
    GpioManager& operator=(GpioManager&&) = delete;
    
    /**
     * @brief Ensure PCAL95555 handler is initialized (lazy initialization).
     * @return true if successful, false otherwise
     * @note Thread-safe with RtosMutex protection
     */
    [[nodiscard]] bool EnsurePcal95555Handler() noexcept;
    
    /**
     * @brief Get TMC9660 handler from MotorController.
     * @param device_index Device index (defaults to onboard device)
     * @return Pointer to Tmc9660Handler if available, nullptr otherwise
     * @note Returns raw pointer - MotorController retains ownership
     * @note Handler lifetime is tied to MotorController lifecycle
     */
    [[nodiscard]] Tmc9660Handler* GetTmc9660Handler(uint8_t device_index = 0) noexcept;
    
    /**
     * @brief Update operation statistics.
     * @param success Whether the operation was successful
     */
    void UpdateStatistics(bool success) noexcept;
    
    /**
     * @brief Update last error for diagnostics.
     * @param error_code Error code to record
     */
    void UpdateLastError(hf_gpio_err_t error_code) noexcept;
    
    /**
     * @brief Validate pin name using hf_gpio_err_t error codes.
     * @param name Pin name to validate
     * @return hf_gpio_err_t::GPIO_SUCCESS if valid, specific error otherwise
     */
    [[nodiscard]] static hf_gpio_err_t ValidatePinName(std::string_view name) noexcept;
    
    //==========================================================================
    // GPIO CREATION METHODS (PRIVATE)
    //==========================================================================
    
    /**
     * @brief Create ESP32 GPIO pin with proper configuration.
     * @param pin_id Physical pin number
     * @param is_inverted Whether pin logic is inverted
     * @param has_pull Whether pin has pull resistor
     * @param pull_is_up If has_pull=true: true=pull-up, false=pull-down
     * @param is_push_pull Output mode: true=push-pull, false=open-drain
     * @param max_current_ma Maximum current in milliamps
     * @return Shared pointer to BaseGpio or nullptr if creation failed
     */
    [[nodiscard]] std::shared_ptr<BaseGpio> CreateEsp32GpioPin(hf_u8_t pin_id, bool is_inverted, 
                                                             bool has_pull, bool pull_is_up, bool is_push_pull,
                                                             hf_u32_t max_current_ma) noexcept;
    
    /**
     * @brief Create PCAL95555 GPIO pin with proper configuration.
     * @param pin_id Physical pin number
     * @param unit_number PCAL95555 unit/device number (0 for first unit, 1 for second, etc.)
     * @param is_inverted Whether pin logic is inverted
     * @param has_pull Whether pin has pull resistor
     * @param pull_is_up If has_pull=true: true=pull-up, false=pull-down
     * @param is_push_pull Output mode: true=push-pull, false=open-drain
     * @param max_current_ma Maximum current in milliamps
     * @return Shared pointer to BaseGpio or nullptr if creation failed
     */
    [[nodiscard]] std::shared_ptr<BaseGpio> CreatePcal95555GpioPin(hf_u8_t pin_id, hf_u8_t unit_number, bool is_inverted, 
                                                                 bool has_pull, bool pull_is_up, bool is_push_pull,
                                                                 hf_u32_t max_current_ma) noexcept;
    
    /**
     * @brief Create TMC9660 GPIO pin with proper configuration.
     * @param pin_id Physical pin number
     * @param device_index TMC9660 device index (0=onboard, 2-3=external devices)
     * @param is_inverted Whether pin logic is inverted
     * @param has_pull Whether pin has pull resistor
     * @param pull_is_up If has_pull=true: true=pull-up, false=pull-down
     * @param is_push_pull Output mode: true=push-pull, false=open-drain
     * @param max_current_ma Maximum current in milliamps
     * @return Shared pointer to BaseGpio or nullptr if creation failed
     */
    [[nodiscard]] std::shared_ptr<BaseGpio> CreateTmc9660GpioPin(hf_u8_t pin_id, hf_u8_t device_index, bool is_inverted, 
                                                               bool has_pull, bool pull_is_up, bool is_push_pull,
                                                               hf_u32_t max_current_ma) noexcept;
};

//==============================================================================
// GLOBAL HELPER FUNCTIONS
//==============================================================================

/**
 * @brief Get the GPIO manager instance.
 * @return Reference to the GPIO manager instance
 */
[[nodiscard]] inline GpioManager& GetGpioManager() noexcept {
    return GpioManager::GetInstance();
}

#endif // COMPONENT_HANDLER_GPIO_MANAGER_H_ 