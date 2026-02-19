/**
 * @file LedManager.h
 * @brief Simple LED management system for a single WS2812 LED with animation support.
 * 
 * @details This class provides a simple LED management system for a single WS2812 LED
 *          on the ESP32-C6 platform. It uses the HF-WS2812-RMT driver for hardware
 *          control and provides basic animation capabilities for status indication.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 * 
 * Key Features:
 * - Single WS2812 LED management with default RMT channel allocation
 * - Basic animation effects (solid, blink, breath, rainbow)
 * - Thread-safe operation with comprehensive error handling
 * - Integration with unified Logger system
 * - Simple color management and brightness control
 * - Animation timing control and state management
 * - System status indication patterns
 * 
 * Architecture:
 * - Uses WS2812 C++ wrapper for hardware control
 * - Follows singleton pattern like other managers
 * - Thread-safe with RtosMutex protection
 * - Exception-free design with noexcept methods
 * - Uses Logger::GetInstance() for all logging
 * 
 * @note This class is designed for a single LED system with simple animation needs.
 * @note Uses default RMT channel allocation (channel 0) as specified.
 */

#ifndef COMPONENT_HANDLER_LED_MANAGER_H_
#define COMPONENT_HANDLER_LED_MANAGER_H_

#include "core/hf-core-drivers/external/hf-ws2812-rmt-driver/inc/ws2812_cpp.hpp"
#include "core/hf-core-drivers/external/hf-ws2812-rmt-driver/inc/ws2812_effects.hpp"
#include "handlers/logger/Logger.h"
#include "utils/RtosMutex.h"
#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"

#include <cstdint>
#include <atomic>
#include <memory>
#include <string>

//==============================================================================
// LED ERROR CODES
//==============================================================================

/**
 * @brief LED manager error codes for consistent error reporting
 */
enum class LedError : uint8_t {
    SUCCESS = 0,
    NOT_INITIALIZED,
    INITIALIZATION_FAILED,
    INVALID_PARAMETER,
    HARDWARE_ERROR,
    ANIMATION_FAILED,
    MUTEX_LOCK_FAILED,
    TIMEOUT,
    INVALID_COLOR,
    INVALID_BRIGHTNESS
};

/**
 * @brief Convert LedError to string for debugging
 */
constexpr const char* LedErrorToString(LedError error) noexcept {
    switch (error) {
        case LedError::SUCCESS: return "Success";
        case LedError::NOT_INITIALIZED: return "Not initialized";
        case LedError::INITIALIZATION_FAILED: return "Initialization failed";
        case LedError::INVALID_PARAMETER: return "Invalid parameter";
        case LedError::HARDWARE_ERROR: return "Hardware error";
        case LedError::ANIMATION_FAILED: return "Animation failed";
        case LedError::MUTEX_LOCK_FAILED: return "Mutex lock failed";
        case LedError::TIMEOUT: return "Timeout";
        case LedError::INVALID_COLOR: return "Invalid color";
        case LedError::INVALID_BRIGHTNESS: return "Invalid brightness";
        default: return "Unknown error";
    }
}

//==============================================================================
// LED ANIMATION TYPES
//==============================================================================

/**
 * @brief Available LED animation effects
 */
enum class LedAnimation : uint8_t {
    OFF = 0,           ///< LED off
    SOLID,             ///< Solid color
    BLINK,             ///< Simple blink
    BREATH,            ///< Breathing effect
    RAINBOW,           ///< Rainbow cycle
    STATUS_OK,         ///< Green solid (system OK)
    STATUS_WARN,       ///< Yellow blink (warning)
    STATUS_ERROR,      ///< Red blink (error)
    STATUS_BOOT,       ///< Blue breath (booting)
    STATUS_CALIBRATE   ///< Purple blink (calibrating)
};

/**
 * @brief Convert LedAnimation to string for debugging
 */
constexpr const char* LedAnimationToString(LedAnimation animation) noexcept {
    switch (animation) {
        case LedAnimation::OFF: return "Off";
        case LedAnimation::SOLID: return "Solid";
        case LedAnimation::BLINK: return "Blink";
        case LedAnimation::BREATH: return "Breath";
        case LedAnimation::RAINBOW: return "Rainbow";
        case LedAnimation::STATUS_OK: return "Status OK";
        case LedAnimation::STATUS_WARN: return "Status Warning";
        case LedAnimation::STATUS_ERROR: return "Status Error";
        case LedAnimation::STATUS_BOOT: return "Status Boot";
        case LedAnimation::STATUS_CALIBRATE: return "Status Calibrate";
        default: return "Unknown";
    }
}

//==============================================================================
// LED INFORMATION STRUCTURES
//==============================================================================

/**
 * @brief Structure containing LED system diagnostics
 */
struct LedSystemDiagnostics {
    bool system_healthy;                    ///< Overall system health
    bool led_initialized;                   ///< LED initialization status
    bool animation_active;                  ///< Current animation status
    LedAnimation current_animation;         ///< Current animation type
    uint32_t total_operations;              ///< Total operations performed
    uint32_t successful_operations;         ///< Successful operations
    uint32_t failed_operations;             ///< Failed operations
    uint32_t animation_cycles;              ///< Animation cycles completed
    uint64_t system_uptime_ms;              ///< System uptime
    LedError last_error;                    ///< Last error encountered
    uint32_t current_brightness;            ///< Current brightness level
    uint32_t current_color;                 ///< Current color value
};

/**
 * @brief Structure for LED color definition
 */
struct LedColor {
    uint8_t red;       ///< Red component (0-255)
    uint8_t green;     ///< Green component (0-255)
    uint8_t blue;      ///< Blue component (0-255)
    
    /**
     * @brief Default constructor (black / off)
     */
    constexpr LedColor() noexcept : red(0), green(0), blue(0) {}
    
    /**
     * @brief Constructor for RGB color
     */
    constexpr LedColor(uint8_t r, uint8_t g, uint8_t b) noexcept 
        : red(r), green(g), blue(b) {}
    
    /**
     * @brief Convert to packed RGB value
     */
    [[nodiscard]] constexpr uint32_t ToRgb() const noexcept {
        return (static_cast<uint32_t>(red) << 16) | 
               (static_cast<uint32_t>(green) << 8) | 
               static_cast<uint32_t>(blue);
    }
    
    /**
     * @brief Create from packed RGB value
     */
    static constexpr LedColor FromRgb(uint32_t rgb) noexcept {
        return LedColor(
            static_cast<uint8_t>((rgb >> 16) & 0xFF),
            static_cast<uint8_t>((rgb >> 8) & 0xFF),
            static_cast<uint8_t>(rgb & 0xFF)
        );
    }
};

// Predefined colors
namespace LedColors {
    static constexpr LedColor BLACK(0, 0, 0);
    static constexpr LedColor RED(255, 0, 0);
    static constexpr LedColor GREEN(0, 255, 0);
    static constexpr LedColor BLUE(0, 0, 255);
    static constexpr LedColor YELLOW(255, 255, 0);
    static constexpr LedColor CYAN(0, 255, 255);
    static constexpr LedColor MAGENTA(255, 0, 255);
    static constexpr LedColor WHITE(255, 255, 255);
    static constexpr LedColor ORANGE(255, 165, 0);
    static constexpr LedColor PURPLE(128, 0, 128);
}

//==============================================================================
// MAIN LED MANAGER CLASS
//==============================================================================

/**
 * @class LedManager
 * @brief Simple LED management system for a single WS2812 LED.
 * 
 * This class provides a simple LED management system for a single WS2812 LED
 * on the ESP32-C6 platform. It uses the HF-WS2812-RMT driver for hardware
 * control and provides basic animation capabilities for status indication.
 * 
 * Thread Safety:
 * - All public methods are thread-safe
 * - Uses internal mutex for protection
 * - Atomic operations where appropriate
 * 
 * Error Handling:
 * - Core operations return LedError for detailed error codes
 * - Comprehensive error codes and diagnostics
 * - Detailed error descriptions via Logger
 * 
 * Performance:
 * - Optimized for single LED operation
 * - Efficient animation timing
 * - Minimal memory footprint
 * 
 * Integration:
 * - Uses unified Logger system
 * - Follows established project patterns
 * - Exception-free design
 */
class LedManager {
public:
    //==========================================================================
    // SINGLETON AND LIFECYCLE
    //==========================================================================
    
    /**
     * @brief Get the singleton instance.
     * @return Reference to the LED manager instance
     */
    static LedManager& GetInstance() noexcept;
    
    /**
     * @brief Ensure the LED manager system is initialized.
     * @return true if initialization successful, false otherwise
     */
    [[nodiscard]] bool EnsureInitialized() noexcept;
    
    /**
     * @brief Shutdown the LED manager system.
     * @return true if shutdown successful, false otherwise
     */
    [[nodiscard]] bool Shutdown() noexcept;
    
    /**
     * @brief Check if the LED system is initialized.
     * @return true if initialized, false otherwise
     */
    [[nodiscard]] bool IsInitialized() const noexcept;
    
    /**
     * @brief Get system diagnostics and health information.
     * @param diagnostics Reference to store system diagnostics
     * @return LedError::SUCCESS if successful, error code otherwise
     */
    [[nodiscard]] LedError GetSystemDiagnostics(LedSystemDiagnostics& diagnostics) const noexcept;
    
    //==========================================================================
    // BASIC LED OPERATIONS
    //==========================================================================
    
    /**
     * @brief Set LED to a specific color.
     * @param color LED color to set
     * @param led_index LED index (0 for first LED, default 0)
     * @return LedError operation result
     */
    [[nodiscard]] LedError SetColor(const LedColor& color, uint32_t led_index = 0) noexcept;
    
    /**
     * @brief Set LED to a specific RGB value.
     * @param rgb Packed RGB value (0xRRGGBB)
     * @param led_index LED index (0 for first LED, default 0)
     * @return LedError operation result
     */
    [[nodiscard]] LedError SetColor(uint32_t rgb, uint32_t led_index = 0) noexcept;
    
    /**
     * @brief Turn LED off.
     * @return LedError operation result
     */
    [[nodiscard]] LedError TurnOff() noexcept;
    
    /**
     * @brief Get current LED color.
     * @param color Reference to store current color
     * @param led_index LED index (0 for first LED, default 0)
     * @return LedError operation result
     */
    [[nodiscard]] LedError GetCurrentColor(LedColor& color, uint32_t led_index = 0) const noexcept;
    
    /**
     * @brief Set LED brightness as percentage (0-100%).
     * @param brightness_percent Brightness percentage (0-100)
     * @return LedError operation result
     */
    [[nodiscard]] LedError SetBrightnessPercent(uint8_t brightness_percent) noexcept;
    
    /**
     * @brief Set LED brightness as raw value (0-255).
     * @param brightness_raw Brightness level (0-255)
     * @return LedError operation result
     */
    [[nodiscard]] LedError SetBrightnessRaw(uint8_t brightness_raw) noexcept;
    
    /**
     * @brief Get current LED brightness as percentage (0-100%).
     * @param brightness_percent Reference to store current brightness percentage
     * @return LedError operation result
     */
    [[nodiscard]] LedError GetCurrentBrightnessPercent(uint8_t& brightness_percent) const noexcept;
    
    /**
     * @brief Get current LED brightness as raw value (0-255).
     * @param brightness_raw Reference to store current brightness raw value
     * @return LedError operation result
     */
    [[nodiscard]] LedError GetCurrentBrightnessRaw(uint8_t& brightness_raw) const noexcept;
    
    /**
     * @brief Set global maximum brightness (0-255).
     * This affects what 100% brightness represents.
     * @param max_brightness Maximum brightness level (0-255)
     * @return LedError operation result
     */
    [[nodiscard]] LedError SetMaxBrightness(uint8_t max_brightness) noexcept;
    
    /**
     * @brief Get global maximum brightness (0-255).
     * @param max_brightness Reference to store maximum brightness
     * @return LedError operation result
     */
    [[nodiscard]] LedError GetMaxBrightness(uint8_t& max_brightness) const noexcept;
    
    //==========================================================================
    // BACKWARD COMPATIBILITY METHODS
    //==========================================================================
    
    /**
     * @brief Set LED brightness (backward compatibility).
     * @param brightness Brightness level (0-255)
     * @return LedError operation result
     * @deprecated Use SetBrightnessPercent() or SetBrightnessRaw() instead
     */
    [[nodiscard]] LedError SetBrightness(uint8_t brightness) noexcept;
    
    /**
     * @brief Get current LED brightness (backward compatibility).
     * @param brightness Reference to store current brightness
     * @return LedError operation result
     * @deprecated Use GetCurrentBrightnessPercent() or GetCurrentBrightnessRaw() instead
     */
    [[nodiscard]] LedError GetCurrentBrightness(uint8_t& brightness) const noexcept;
    
    //==========================================================================
    // ANIMATION OPERATIONS
    //==========================================================================
    
    /**
     * @brief Start a specific animation.
     * @param animation Animation type to start
     * @param color Base color for the animation (optional)
     * @return LedError operation result
     */
    [[nodiscard]] LedError StartAnimation(LedAnimation animation, const LedColor& color = LedColors::WHITE) noexcept;
    
    /**
     * @brief Stop current animation and turn LED off.
     * @return LedError operation result
     */
    [[nodiscard]] LedError StopAnimation() noexcept;
    
    /**
     * @brief Get current animation type.
     * @param animation Reference to store current animation
     * @return LedError operation result
     */
    [[nodiscard]] LedError GetCurrentAnimation(LedAnimation& animation) const noexcept;
    
    /**
     * @brief Check if animation is currently active.
     * @return true if animation is active, false otherwise
     */
    [[nodiscard]] bool IsAnimationActive() const noexcept;
    
    /**
     * @brief Update animation (call periodically from main loop).
     * @return LedError operation result
     */
    [[nodiscard]] LedError UpdateAnimation() noexcept;
    
    //==========================================================================
    // STATUS INDICATION METHODS
    //==========================================================================
    
    /**
     * @brief Set system status indication.
     * @param animation Status animation type
     * @return LedError operation result
     */
    [[nodiscard]] LedError SetStatus(LedAnimation animation) noexcept;
    
    /**
     * @brief Indicate system boot status.
     * @return LedError operation result
     */
    [[nodiscard]] LedError IndicateBoot() noexcept;
    
    /**
     * @brief Indicate system ready status.
     * @return LedError operation result
     */
    [[nodiscard]] LedError IndicateReady() noexcept;
    
    /**
     * @brief Indicate system warning status.
     * @return LedError operation result
     */
    [[nodiscard]] LedError IndicateWarning() noexcept;
    
    /**
     * @brief Indicate system error status.
     * @return LedError operation result
     */
    [[nodiscard]] LedError IndicateError() noexcept;
    
    /**
     * @brief Indicate calibration status.
     * @return LedError operation result
     */
    [[nodiscard]] LedError IndicateCalibration() noexcept;
    
    //==========================================================================
    // UTILITY METHODS
    //==========================================================================
    
    /**
     * @brief Generate color wheel value.
     * @param position Position on color wheel (0-255)
     * @return Packed RGB color value
     */
    [[nodiscard]] static uint32_t ColorWheel(uint8_t position) noexcept;
    
    /**
     * @brief Log current LED state for debugging.
     */
    void LogCurrentState() const noexcept;
    
    /**
     * @brief Dump comprehensive system statistics to log as INFO level.
     * Logs all diagnostics, statistics, and system health information.
     */
    void DumpStatistics() const noexcept;
    
    /**
     * @brief Get current LED GPIO pin information.
     * @return GPIO pin number currently being used
     */
    [[nodiscard]] gpio_num_t GetCurrentGpioPin() const noexcept;
    
    /**
     * @brief Get number of LEDs in the strip.
     * @return Number of LEDs configured
     */
    [[nodiscard]] uint32_t GetLedCount() const noexcept;
    
    /**
     * @brief Set all LEDs to the same color.
     * @param color LED color to set for all LEDs
     * @return LedError operation result
     */
    [[nodiscard]] LedError SetAllLeds(const LedColor& color) noexcept;
    
    /**
     * @brief Set all LEDs to the same RGB value.
     * @param rgb Packed RGB value (0xRRGGBB) for all LEDs
     * @return LedError operation result
     */
    [[nodiscard]] LedError SetAllLeds(uint32_t rgb) noexcept;

private:
    //==========================================================================
    // PRIVATE MEMBERS
    //==========================================================================
    
    /**
     * @brief Initialize the LED manager system.
     * @return true if initialization successful, false otherwise
     */
    [[nodiscard]] bool Initialize() noexcept;
    
    /**
     * @brief Update animation step for current animation type.
     * @return LedError operation result
     */
    [[nodiscard]] LedError UpdateAnimationStep() noexcept;
    
    /**
     * @brief Update system statistics.
     * @param success Whether the operation was successful
     */
    void UpdateStatistics(bool success) noexcept;
    
    /**
     * @brief Update last error encountered.
     * @param error_code Error code to set
     */
    void UpdateLastError(LedError error_code) noexcept;
    
    /**
     * @brief Get current system uptime in milliseconds.
     * @return System uptime in milliseconds
     */
    [[nodiscard]] uint64_t GetSystemUptimeMs() const noexcept;
    
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
    // HARDWARE COMPONENTS
    // ===============================
    
    /**
     * @brief WS2812 strip object for hardware control.
     * Single LED strip with default RMT channel allocation.
     */
    std::unique_ptr<WS2812Strip> led_strip_;
    
    /**
     * @brief WS2812 animator for animation effects.
     * Provides animation capabilities for the LED strip.
     */
    std::unique_ptr<WS2812Animator> led_animator_;
    
    // ===============================
    // ANIMATION STATE
    // ===============================
    
    /**
     * @brief Current animation type.
     */
    LedAnimation current_animation_{LedAnimation::OFF};
    
    /**
     * @brief Current LED color.
     */
    LedColor current_color_{LedColors::BLACK};
    
    /**
     * @brief Current brightness level (raw value 0-255).
     */
    uint8_t current_brightness_{255};
    
    /**
     * @brief Global maximum brightness (raw value 0-255).
     * This represents what 100% brightness equals.
     */
    uint8_t max_brightness_{DEFAULT_MAX_BRIGHTNESS};
    
    /**
     * @brief Animation step counter.
     */
    uint16_t animation_step_{0};
    
    /**
     * @brief Animation direction (for breathing effect).
     */
    int8_t animation_direction_{1};
    
    /**
     * @brief Last animation update time.
     */
    uint64_t last_animation_update_{0};
    
    // ===============================
    // SYSTEM STATISTICS
    // ===============================
    
    /**
     * @brief Total operations performed (atomic for thread safety).
     */
    std::atomic<uint32_t> total_operations_{0};
    
    /**
     * @brief Successful operations performed (atomic for thread safety).
     */
    std::atomic<uint32_t> successful_operations_{0};
    
    /**
     * @brief Failed operations performed (atomic for thread safety).
     */
    std::atomic<uint32_t> failed_operations_{0};
    
    /**
     * @brief Animation cycles completed (atomic for thread safety).
     */
    std::atomic<uint32_t> animation_cycles_{0};
    
    /**
     * @brief Last error encountered.
     */
    std::atomic<LedError> last_error_{LedError::SUCCESS};
    
    /**
     * @brief System start time for uptime calculation.
     */
    uint64_t system_start_time_{0};
    
    /**
     * @brief Current LED GPIO pin being used.
     */
    gpio_num_t current_led_gpio_{GPIO_NUM_3};
    
    // ===============================
    // CONSTANTS
    // ===============================
    
    /**
     * @brief Get LED GPIO pin from platform mapping.
     * @return GPIO pin number for WS2812 LED
     */
    [[nodiscard]] static gpio_num_t GetLedGpioPin() noexcept {
        const auto* mapping = GetGpioMapping(HfFunctionalGpioPin::WS2812_LED_DAT);
        if (mapping && mapping->chip_type == static_cast<uint8_t>(HfGpioChipType::ESP32_INTERNAL)) {
            return static_cast<gpio_num_t>(mapping->physical_pin);
        }
        // Fallback to GPIO 3 if mapping not found
        return GPIO_NUM_3;
    }
    
    /**
     * @brief Convert percentage brightness to raw brightness.
     * @param percent Brightness percentage (0-100)
     * @return Raw brightness value (0-255)
     */
    [[nodiscard]] uint8_t PercentToRaw(uint8_t percent) const noexcept {
        if (percent > MAX_BRIGHTNESS_PERCENT) {
            percent = MAX_BRIGHTNESS_PERCENT;
        }
        return static_cast<uint8_t>((static_cast<uint32_t>(percent) * max_brightness_) / MAX_BRIGHTNESS_PERCENT);
    }
    
    /**
     * @brief Convert raw brightness to percentage.
     * @param raw Raw brightness value (0-255)
     * @return Brightness percentage (0-100)
     */
    [[nodiscard]] uint8_t RawToPercent(uint8_t raw) const noexcept {
        if (max_brightness_ == 0) {
            return 0;
        }
        return static_cast<uint8_t>((static_cast<uint32_t>(raw) * MAX_BRIGHTNESS_PERCENT) / max_brightness_);
    }
    
    /**
     * @brief Default RMT channel for LED.
     */
    static constexpr int DEFAULT_RMT_CHANNEL = 0;
    
    /**
     * @brief Number of LEDs in the strip (configurable).
     */
    static constexpr uint32_t kLedCount = 1;  // Can be extended for multiple LEDs
    
    /**
     * @brief Default global maximum brightness (0-255).
     * This represents 100% brightness and can be configured.
     */
    static constexpr uint8_t DEFAULT_MAX_BRIGHTNESS = 255;
    
    /**
     * @brief Minimum brightness percentage (0%).
     */
    static constexpr uint8_t MIN_BRIGHTNESS_PERCENT = 0;
    
    /**
     * @brief Maximum brightness percentage (100%).
     */
    static constexpr uint8_t MAX_BRIGHTNESS_PERCENT = 100;
    
    /**
     * @brief Animation update interval in milliseconds.
     */
    static constexpr uint32_t ANIMATION_UPDATE_INTERVAL_MS = 50;
    
    /**
     * @brief Breathing animation speed.
     */
    static constexpr uint32_t BREATH_SPEED = 5;
    
    /**
     * @brief Blink animation speed.
     */
    static constexpr uint32_t BLINK_SPEED = 500;
};

//==============================================================================
// CONVENIENCE FUNCTIONS
//==============================================================================

/**
 * @brief Get LED manager instance (convenience function).
 * @return Reference to LED manager instance
 */
[[nodiscard]] inline LedManager& GetLedManager() noexcept {
    return LedManager::GetInstance();
}

#endif // COMPONENT_HANDLER_LED_MANAGER_H_ 