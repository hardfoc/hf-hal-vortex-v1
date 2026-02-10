/**
 * @file LedManager.cpp
 * @brief Implementation of the LED management system for a single WS2812 LED.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 1.0
 * @copyright HardFOC
 */

#include "LedManager.h"
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_cpp.hpp"
#include "utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver/include/ws2812_effects.hpp"
#include "utils-and-drivers/driver-handlers/Logger.h"
#include "utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/include/RtosMutex.h"
#include "utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/include/OsAbstraction.h"

#include <cstring>
#include <cmath>

//==============================================================================
// STATIC MEMBERS
//==============================================================================

static constexpr const char* TAG = "LedManager";

//==============================================================================
// SINGLETON IMPLEMENTATION
//==============================================================================

LedManager& LedManager::GetInstance() noexcept {
    static LedManager instance;
    return instance;
}

//==============================================================================
// LIFECYCLE METHODS
//==============================================================================

bool LedManager::EnsureInitialized() noexcept {
    if (is_initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    std::lock_guard<RtosMutex> lock(mutex_);
    
    // Double-check after acquiring lock
    if (is_initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    return Initialize();
}

bool LedManager::Shutdown() noexcept {
    std::lock_guard<RtosMutex> lock(mutex_);
    
    if (!is_initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    // Stop any active animation
    StopAnimation();
    
    // Turn off LED
    TurnOff();
    
    // Clear hardware components
    led_animator_.reset();
    led_strip_.reset();
    
    // Update state
    is_initialized_.store(false, std::memory_order_release);
    
    Logger::GetInstance().Info(TAG, "LED manager shutdown completed");
    return true;
}

bool LedManager::IsInitialized() const noexcept {
    return is_initialized_.load(std::memory_order_acquire);
}

LedError LedManager::GetSystemDiagnostics(LedSystemDiagnostics& diagnostics) const noexcept {
    std::lock_guard<RtosMutex> lock(mutex_);
    
    diagnostics.system_healthy = is_initialized_.load(std::memory_order_acquire) && 
                                (led_strip_ != nullptr) && (led_animator_ != nullptr);
    diagnostics.led_initialized = is_initialized_.load(std::memory_order_acquire);
    diagnostics.animation_active = IsAnimationActive();
    diagnostics.current_animation = current_animation_;
    diagnostics.total_operations = total_operations_.load(std::memory_order_acquire);
    diagnostics.successful_operations = successful_operations_.load(std::memory_order_acquire);
    diagnostics.failed_operations = failed_operations_.load(std::memory_order_acquire);
    diagnostics.animation_cycles = animation_cycles_.load(std::memory_order_acquire);
    diagnostics.system_uptime_ms = GetSystemUptimeMs();
    diagnostics.last_error = last_error_.load(std::memory_order_acquire);
    diagnostics.current_brightness = current_brightness_;
    diagnostics.current_color = current_color_.ToRgb();
    
    return LedError::SUCCESS;
}

//==============================================================================
// BASIC LED OPERATIONS
//==============================================================================

LedError LedManager::SetColor(const LedColor& color, uint32_t led_index) noexcept {
    if (!EnsureInitialized()) {
        UpdateLastError(LedError::NOT_INITIALIZED);
        return LedError::NOT_INITIALIZED;
    }
    
    if (led_index >= NUM_LEDS) {
        UpdateLastError(LedError::INVALID_PARAMETER);
        Logger::GetInstance().Error(TAG, "Invalid LED index: %d (max: %d)", led_index, NUM_LEDS - 1);
        return LedError::INVALID_PARAMETER;
    }
    
    std::lock_guard<RtosMutex> lock(mutex_);
    
    // Stop any active animation
    if (current_animation_ != LedAnimation::OFF) {
        StopAnimation();
    }
    
    // Set the color for specific LED
    led_strip_->setPixel(led_index, color.ToRgb());
    esp_err_t result = led_strip_->show();
    
    if (result != ESP_OK) {
        UpdateLastError(LedError::HARDWARE_ERROR);
        UpdateStatistics(false);
        Logger::GetInstance().Error(TAG, "Failed to set LED %d color: %s", led_index, esp_err_to_name(result));
        return LedError::HARDWARE_ERROR;
    }
    
    // Update state (for single LED, this represents the main LED)
    if (led_index == 0) {
        current_color_ = color;
        current_animation_ = LedAnimation::SOLID;
    }
    
    UpdateStatistics(true);
    Logger::GetInstance().Debug(TAG, "LED %d color set to RGB(%d, %d, %d)", 
                               led_index, color.red, color.green, color.blue);
    
    return LedError::SUCCESS;
}

LedError LedManager::SetColor(uint32_t rgb, uint32_t led_index) noexcept {
    return SetColor(LedColor::FromRgb(rgb), led_index);
}

LedError LedManager::TurnOff() noexcept {
    if (!EnsureInitialized()) {
        UpdateLastError(LedError::NOT_INITIALIZED);
        return LedError::NOT_INITIALIZED;
    }
    
    std::lock_guard<RtosMutex> lock(mutex_);
    
    // Stop any active animation
    if (current_animation_ != LedAnimation::OFF) {
        StopAnimation();
    }
    
    // Turn off LED
    led_strip_->setPixel(0, 0);
    esp_err_t result = led_strip_->show();
    
    if (result != ESP_OK) {
        UpdateLastError(LedError::HARDWARE_ERROR);
        UpdateStatistics(false);
        Logger::GetInstance().Error(TAG, "Failed to turn off LED: %s", esp_err_to_name(result));
        return LedError::HARDWARE_ERROR;
    }
    
    // Update state
    current_color_ = LedColors::BLACK;
    current_animation_ = LedAnimation::OFF;
    
    UpdateStatistics(true);
    Logger::GetInstance().Debug(TAG, "LED turned off");
    
    return LedError::SUCCESS;
}

LedError LedManager::GetCurrentColor(LedColor& color, uint32_t led_index) const noexcept {
    if (led_index >= NUM_LEDS) {
        return LedError::INVALID_PARAMETER;
    }
    
    std::lock_guard<RtosMutex> lock(mutex_);
    
    // For now, return the main LED color (LED 0)
    // In a full multi-LED implementation, we'd store colors per LED
    if (led_index == 0) {
        color = current_color_;
        return LedError::SUCCESS;
    }
    
    // For other LEDs, return black (not implemented yet)
    color = LedColors::BLACK;
    return LedError::SUCCESS;
}

LedError LedManager::SetBrightnessPercent(uint8_t brightness_percent) noexcept {
    if (!EnsureInitialized()) {
        UpdateLastError(LedError::NOT_INITIALIZED);
        return LedError::NOT_INITIALIZED;
    }
    
    if (brightness_percent > MAX_BRIGHTNESS_PERCENT) {
        UpdateLastError(LedError::INVALID_BRIGHTNESS);
        Logger::GetInstance().Error(TAG, "Invalid brightness percentage: %d (max: %d)", 
                                   brightness_percent, MAX_BRIGHTNESS_PERCENT);
        return LedError::INVALID_BRIGHTNESS;
    }
    
    std::lock_guard<RtosMutex> lock(mutex_);
    
    // Convert percentage to raw value
    uint8_t raw_brightness = PercentToRaw(brightness_percent);
    
    // Set brightness
    led_strip_->setBrightness(raw_brightness);
    current_brightness_ = raw_brightness;
    
    // Update LED display
    esp_err_t result = led_strip_->show();
    if (result != ESP_OK) {
        UpdateLastError(LedError::HARDWARE_ERROR);
        UpdateStatistics(false);
        Logger::GetInstance().Error(TAG, "Failed to set brightness %d%% (raw: %d): %s", 
                                   brightness_percent, raw_brightness, esp_err_to_name(result));
        return LedError::HARDWARE_ERROR;
    }
    
    UpdateStatistics(true);
    Logger::GetInstance().Debug(TAG, "LED brightness set to %d%% (raw: %d)", brightness_percent, raw_brightness);
    
    return LedError::SUCCESS;
}

LedError LedManager::SetBrightnessRaw(uint8_t brightness_raw) noexcept {
    if (!EnsureInitialized()) {
        UpdateLastError(LedError::NOT_INITIALIZED);
        return LedError::NOT_INITIALIZED;
    }
    
    std::lock_guard<RtosMutex> lock(mutex_);
    
    // Set brightness
    led_strip_->setBrightness(brightness_raw);
    current_brightness_ = brightness_raw;
    
    // Update LED display
    esp_err_t result = led_strip_->show();
    if (result != ESP_OK) {
        UpdateLastError(LedError::HARDWARE_ERROR);
        UpdateStatistics(false);
        Logger::GetInstance().Error(TAG, "Failed to set brightness raw %d: %s", brightness_raw, esp_err_to_name(result));
        return LedError::HARDWARE_ERROR;
    }
    
    UpdateStatistics(true);
    Logger::GetInstance().Debug(TAG, "LED brightness set to raw %d (%d%%)", brightness_raw, RawToPercent(brightness_raw));
    
    return LedError::SUCCESS;
}

LedError LedManager::GetCurrentBrightnessPercent(uint8_t& brightness_percent) const noexcept {
    std::lock_guard<RtosMutex> lock(mutex_);
    brightness_percent = RawToPercent(current_brightness_);
    return LedError::SUCCESS;
}

LedError LedManager::GetCurrentBrightnessRaw(uint8_t& brightness_raw) const noexcept {
    std::lock_guard<RtosMutex> lock(mutex_);
    brightness_raw = current_brightness_;
    return LedError::SUCCESS;
}

LedError LedManager::SetMaxBrightness(uint8_t max_brightness) noexcept {
    if (!EnsureInitialized()) {
        UpdateLastError(LedError::NOT_INITIALIZED);
        return LedError::NOT_INITIALIZED;
    }
    
    std::lock_guard<RtosMutex> lock(mutex_);
    
    // Store old max brightness for logging
    uint8_t old_max = max_brightness_;
    
    // Set new max brightness
    max_brightness_ = max_brightness;
    
    // Recalculate current brightness to maintain same percentage
    uint8_t current_percent = RawToPercent(current_brightness_);
    uint8_t new_raw_brightness = PercentToRaw(current_percent);
    
    // Apply new brightness
    led_strip_->setBrightness(new_raw_brightness);
    current_brightness_ = new_raw_brightness;
    
    // Update LED display
    esp_err_t result = led_strip_->show();
    if (result != ESP_OK) {
        UpdateLastError(LedError::HARDWARE_ERROR);
        UpdateStatistics(false);
        Logger::GetInstance().Error(TAG, "Failed to apply max brightness change: %s", esp_err_to_name(result));
        return LedError::HARDWARE_ERROR;
    }
    
    UpdateStatistics(true);
    Logger::GetInstance().Info(TAG, "Max brightness changed from %d to %d (current: %d%% -> %d raw)", 
                               old_max, max_brightness, current_percent, new_raw_brightness);
    
    return LedError::SUCCESS;
}

LedError LedManager::GetMaxBrightness(uint8_t& max_brightness) const noexcept {
    std::lock_guard<RtosMutex> lock(mutex_);
    max_brightness = max_brightness_;
    return LedError::SUCCESS;
}

//==============================================================================
// BACKWARD COMPATIBILITY METHODS
//==============================================================================

LedError LedManager::SetBrightness(uint8_t brightness) noexcept {
    // Backward compatibility - treat as raw brightness
    return SetBrightnessRaw(brightness);
}

LedError LedManager::GetCurrentBrightness(uint8_t& brightness) const noexcept {
    // Backward compatibility - return raw brightness
    return GetCurrentBrightnessRaw(brightness);
}

//==============================================================================
// ANIMATION OPERATIONS
//==============================================================================

LedError LedManager::StartAnimation(LedAnimation animation, const LedColor& color) noexcept {
    if (!EnsureInitialized()) {
        UpdateLastError(LedError::NOT_INITIALIZED);
        return LedError::NOT_INITIALIZED;
    }
    
    std::lock_guard<RtosMutex> lock(mutex_);
    
    // Stop current animation if any
    if (current_animation_ != LedAnimation::OFF) {
        StopAnimation();
    }
    
    // Set up new animation
    current_animation_ = animation;
    current_color_ = color;
    animation_step_ = 0;
    animation_direction_ = 1;
    last_animation_update_ = GetSystemUptimeMs();
    
    // Handle status animations
    switch (animation) {
        case LedAnimation::STATUS_OK:
            current_color_ = LedColors::GREEN;
            break;
        case LedAnimation::STATUS_WARN:
            current_color_ = LedColors::YELLOW;
            break;
        case LedAnimation::STATUS_ERROR:
            current_color_ = LedColors::RED;
            break;
        case LedAnimation::STATUS_BOOT:
            current_color_ = LedColors::BLUE;
            break;
        case LedAnimation::STATUS_CALIBRATE:
            current_color_ = LedColors::PURPLE;
            break;
        default:
            break;
    }
    
    UpdateStatistics(true);
    Logger::GetInstance().Info(TAG, "Started animation: %s", LedAnimationToString(animation));
    
    return LedError::SUCCESS;
}

LedError LedManager::StopAnimation() noexcept {
    std::lock_guard<RtosMutex> lock(mutex_);
    
    if (current_animation_ == LedAnimation::OFF) {
        return LedError::SUCCESS;
    }
    
    // Turn off LED
    led_strip_->setPixel(0, 0);
    esp_err_t result = led_strip_->show();
    
    if (result != ESP_OK) {
        UpdateLastError(LedError::HARDWARE_ERROR);
        UpdateStatistics(false);
        Logger::GetInstance().Error(TAG, "Failed to stop animation: %s", esp_err_to_name(result));
        return LedError::HARDWARE_ERROR;
    }
    
    // Update state
    current_animation_ = LedAnimation::OFF;
    current_color_ = LedColors::BLACK;
    animation_step_ = 0;
    
    UpdateStatistics(true);
    Logger::GetInstance().Debug(TAG, "Animation stopped");
    
    return LedError::SUCCESS;
}

LedError LedManager::GetCurrentAnimation(LedAnimation& animation) const noexcept {
    std::lock_guard<RtosMutex> lock(mutex_);
    animation = current_animation_;
    return LedError::SUCCESS;
}

bool LedManager::IsAnimationActive() const noexcept {
    return current_animation_ != LedAnimation::OFF;
}

LedError LedManager::UpdateAnimation() noexcept {
    if (!EnsureInitialized()) {
        return LedError::NOT_INITIALIZED;
    }
    
    std::lock_guard<RtosMutex> lock(mutex_);
    
    if (current_animation_ == LedAnimation::OFF) {
        return LedError::SUCCESS;
    }
    
    uint64_t current_time = GetSystemUptimeMs();
    if (current_time - last_animation_update_ < ANIMATION_UPDATE_INTERVAL_MS) {
        return LedError::SUCCESS;
    }
    
    last_animation_update_ = current_time;
    
    LedError result = UpdateAnimationStep();
    if (result == LedError::SUCCESS) {
        animation_cycles_.fetch_add(1, std::memory_order_relaxed);
    }
    return result;
}

//==============================================================================
// STATUS INDICATION METHODS
//==============================================================================

LedError LedManager::SetStatus(LedAnimation animation) noexcept {
    return StartAnimation(animation);
}

LedError LedManager::IndicateBoot() noexcept {
    return StartAnimation(LedAnimation::STATUS_BOOT);
}

LedError LedManager::IndicateReady() noexcept {
    return StartAnimation(LedAnimation::STATUS_OK);
}

LedError LedManager::IndicateWarning() noexcept {
    return StartAnimation(LedAnimation::STATUS_WARN);
}

LedError LedManager::IndicateError() noexcept {
    return StartAnimation(LedAnimation::STATUS_ERROR);
}

LedError LedManager::IndicateCalibration() noexcept {
    return StartAnimation(LedAnimation::STATUS_CALIBRATE);
}

//==============================================================================
// UTILITY METHODS
//==============================================================================

uint32_t LedManager::ColorWheel(uint8_t position) noexcept {
    position = 255 - position;
    
    if (position < 85) {
        return (255 - position * 3) << 16 | (position * 3) << 8;
    } else if (position < 170) {
        position -= 85;
        return (position * 3) << 16 | (255 - position * 3) << 8;
    } else {
        position -= 170;
        return (255 - position * 3) << 8 | (position * 3);
    }
}

void LedManager::LogCurrentState() const noexcept {
    std::lock_guard<RtosMutex> lock(mutex_);
    
    Logger::GetInstance().Info(TAG, "LED State:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", is_initialized_.load() ? "Yes" : "No");
    Logger::GetInstance().Info(TAG, "  Animation: %s", LedAnimationToString(current_animation_));
    Logger::GetInstance().Info(TAG, "  Color: RGB(%d, %d, %d)", 
                               current_color_.red, current_color_.green, current_color_.blue);
    Logger::GetInstance().Info(TAG, "  Brightness: %d", current_brightness_);
    Logger::GetInstance().Info(TAG, "  Animation Step: %d", animation_step_);
    Logger::GetInstance().Info(TAG, "  Total Operations: %d", total_operations_.load());
    Logger::GetInstance().Info(TAG, "  Successful Operations: %d", successful_operations_.load());
    Logger::GetInstance().Info(TAG, "  Failed Operations: %d", failed_operations_.load());
    Logger::GetInstance().Info(TAG, "  Animation Cycles: %d", animation_cycles_.load());
    Logger::GetInstance().Info(TAG, "  Last Error: %s", LedErrorToString(last_error_.load()));
    Logger::GetInstance().Info(TAG, "  GPIO Pin: %d", current_led_gpio_);
}

void LedManager::DumpStatistics() const noexcept {
    Logger::GetInstance().Info(TAG, "=== LED MANAGER STATISTICS ===");
    
    // Get comprehensive diagnostics
    LedSystemDiagnostics diagnostics;
    if (GetSystemDiagnostics(diagnostics) == LedError::SUCCESS) {
        Logger::GetInstance().Info(TAG, "System Health:");
        Logger::GetInstance().Info(TAG, "  Overall Health: %s", diagnostics.system_healthy ? "HEALTHY" : "DEGRADED");
        Logger::GetInstance().Info(TAG, "  Initialized: %s", diagnostics.led_initialized ? "YES" : "NO");
        Logger::GetInstance().Info(TAG, "  System Uptime: %llu ms (%.2f hours)", 
                                   diagnostics.system_uptime_ms, 
                                   diagnostics.system_uptime_ms / 3600000.0);
        
        Logger::GetInstance().Info(TAG, "Current Status:");
        Logger::GetInstance().Info(TAG, "  Animation Active: %s", diagnostics.animation_active ? "YES" : "NO");
        Logger::GetInstance().Info(TAG, "  Current Animation: %s", LedAnimationToString(diagnostics.current_animation));
        Logger::GetInstance().Info(TAG, "  Current Color: 0x%06X", diagnostics.current_color);
        Logger::GetInstance().Info(TAG, "  Current Brightness: %d/255 (%d%%)", 
                                   diagnostics.current_brightness,
                                   (diagnostics.current_brightness * 100) / 255);
        
        Logger::GetInstance().Info(TAG, "Operation Statistics:");
        Logger::GetInstance().Info(TAG, "  Total Operations: %d", diagnostics.total_operations);
        Logger::GetInstance().Info(TAG, "  Successful Operations: %d", diagnostics.successful_operations);
        Logger::GetInstance().Info(TAG, "  Failed Operations: %d", diagnostics.failed_operations);
        
        if (diagnostics.total_operations > 0) {
            float success_rate = (float)diagnostics.successful_operations / diagnostics.total_operations * 100.0f;
            Logger::GetInstance().Info(TAG, "  Success Rate: %.2f%%", success_rate);
        }
        
        Logger::GetInstance().Info(TAG, "Animation Statistics:");
        Logger::GetInstance().Info(TAG, "  Animation Cycles: %d", diagnostics.animation_cycles);
        Logger::GetInstance().Info(TAG, "  Last Error: %s", LedErrorToString(diagnostics.last_error));
        
        Logger::GetInstance().Info(TAG, "Hardware Configuration:");
        Logger::GetInstance().Info(TAG, "  GPIO Pin: %d", current_led_gpio_);
        Logger::GetInstance().Info(TAG, "  LED Count: %d", NUM_LEDS);
        Logger::GetInstance().Info(TAG, "  RMT Channel: %d", DEFAULT_RMT_CHANNEL);
        Logger::GetInstance().Info(TAG, "  Max Brightness: %d/255", max_brightness_);
    } else {
        Logger::GetInstance().Error(TAG, "Failed to retrieve system diagnostics for statistics dump");
    }
    
    Logger::GetInstance().Info(TAG, "=== END LED MANAGER STATISTICS ===");
}

//==============================================================================
// PRIVATE METHODS
//==============================================================================

bool LedManager::Initialize() noexcept {
    Logger::GetInstance().Info(TAG, "Initializing LED manager...");
    
            // Get LED GPIO pin from platform mapping
        gpio_num_t led_gpio = GetLedGpioPin();
        current_led_gpio_ = led_gpio;
        
        // Create WS2812 strip with proper timing parameters
        led_strip_ = std::make_unique<WS2812Strip>(
            led_gpio,              // GPIO pin from platform mapping
            DEFAULT_RMT_CHANNEL,   // RMT channel
            NUM_LEDS,              // Number of LEDs
            LedType::RGB,          // LED type
            14, 52, 52, 52,        // Timing (T0H, T1H, T0L, T1L) - WS2812 standard
            255                    // Default brightness
        );
    
    if (!led_strip_) {
        Logger::GetInstance().Error(TAG, "Failed to create WS2812 strip");
        UpdateLastError(LedError::INITIALIZATION_FAILED);
        return false;
    }
    
    // Initialize the strip
    esp_err_t result = led_strip_->begin();
    if (result != ESP_OK) {
        Logger::GetInstance().Error(TAG, "Failed to initialize WS2812 strip: %s", esp_err_to_name(result));
        UpdateLastError(LedError::INITIALIZATION_FAILED);
        return false;
    }
    
    // Create animator
    led_animator_ = std::make_unique<WS2812Animator>(*led_strip_, NUM_LEDS);
    
    if (!led_animator_) {
        Logger::GetInstance().Error(TAG, "Failed to create WS2812 animator");
        UpdateLastError(LedError::INITIALIZATION_FAILED);
        return false;
    }
    
    // Initialize state
    current_animation_ = LedAnimation::OFF;
    current_color_ = LedColors::BLACK;
    current_brightness_ = 255;
    animation_step_ = 0;
    animation_direction_ = 1;
    system_start_time_ = GetSystemUptimeMs();
    
    // Turn off LED initially
    led_strip_->setPixel(0, 0);
    result = led_strip_->show();
    if (result != ESP_OK) {
        Logger::GetInstance().Error(TAG, "Failed to initialize LED state: %s", esp_err_to_name(result));
        UpdateLastError(LedError::INITIALIZATION_FAILED);
        return false;
    }
    
    // Update state
    is_initialized_.store(true, std::memory_order_release);
    
            Logger::GetInstance().Info(TAG, "LED manager initialized successfully on GPIO %d", led_gpio);
        return true;
}

LedError LedManager::UpdateAnimationStep() noexcept {
    if (!led_strip_ || !led_animator_) {
        return LedError::NOT_INITIALIZED;
    }
    
    switch (current_animation_) {
        case LedAnimation::BLINK:
        case LedAnimation::STATUS_WARN:
        case LedAnimation::STATUS_ERROR:
        case LedAnimation::STATUS_CALIBRATE: {
            // Simple blink animation
            bool is_on = (animation_step_ / (BLINK_SPEED / ANIMATION_UPDATE_INTERVAL_MS)) % 2 == 0;
            uint32_t color = is_on ? current_color_.ToRgb() : 0;
            led_strip_->setPixel(0, color);
            esp_err_t result = led_strip_->show();
            if (result != ESP_OK) {
                return LedError::ANIMATION_FAILED;
            }
            break;
        }
        
        case LedAnimation::BREATH:
        case LedAnimation::STATUS_BOOT: {
            // Breathing animation - calculate percentage and convert to raw
            uint8_t brightness_percent = static_cast<uint8_t>(
                50 + 50 * std::sin(animation_step_ * BREATH_SPEED * 0.01f)
            );
            uint8_t raw_brightness = PercentToRaw(brightness_percent);
            led_strip_->setBrightness(raw_brightness);
            led_strip_->setPixel(0, current_color_.ToRgb());
            esp_err_t result = led_strip_->show();
            if (result != ESP_OK) {
                return LedError::ANIMATION_FAILED;
            }
            break;
        }
        
        case LedAnimation::RAINBOW: {
            // Rainbow animation
            uint32_t color = ColorWheel(animation_step_ % 256);
            led_strip_->setPixel(0, color);
            esp_err_t result = led_strip_->show();
            if (result != ESP_OK) {
                return LedError::ANIMATION_FAILED;
            }
            break;
        }
        
        case LedAnimation::SOLID:
        case LedAnimation::STATUS_OK: {
            // Solid color - no animation needed
            break;
        }
        
        case LedAnimation::OFF:
        default:
            return LedError::SUCCESS;
    }
    
    animation_step_++;
    return LedError::SUCCESS;
}

void LedManager::UpdateStatistics(bool success) noexcept {
    total_operations_.fetch_add(1, std::memory_order_relaxed);
    
    if (success) {
        successful_operations_.fetch_add(1, std::memory_order_relaxed);
    } else {
        failed_operations_.fetch_add(1, std::memory_order_relaxed);
    }
}

void LedManager::UpdateLastError(LedError error_code) noexcept {
    last_error_.store(error_code, std::memory_order_release);
}

uint64_t LedManager::GetSystemUptimeMs() const noexcept {
    return OsAbstraction::GetTimeMs();
}

gpio_num_t LedManager::GetCurrentGpioPin() const noexcept {
    return current_led_gpio_;
}

uint32_t LedManager::GetLedCount() const noexcept {
    return NUM_LEDS;
}

LedError LedManager::SetAllLeds(const LedColor& color) noexcept {
    if (!EnsureInitialized()) {
        UpdateLastError(LedError::NOT_INITIALIZED);
        return LedError::NOT_INITIALIZED;
    }
    
    std::lock_guard<RtosMutex> lock(mutex_);
    
    // Stop any active animation
    if (current_animation_ != LedAnimation::OFF) {
        StopAnimation();
    }
    
    // Set all LEDs to the same color
    for (uint32_t i = 0; i < NUM_LEDS; ++i) {
        led_strip_->setPixel(i, color.ToRgb());
    }
    
    esp_err_t result = led_strip_->show();
    if (result != ESP_OK) {
        UpdateLastError(LedError::HARDWARE_ERROR);
        UpdateStatistics(false);
        Logger::GetInstance().Error(TAG, "Failed to set all LEDs color: %s", esp_err_to_name(result));
        return LedError::HARDWARE_ERROR;
    }
    
    // Update state
    current_color_ = color;
    current_animation_ = LedAnimation::SOLID;
    
    UpdateStatistics(true);
    Logger::GetInstance().Debug(TAG, "All %d LEDs set to RGB(%d, %d, %d)", 
                               NUM_LEDS, color.red, color.green, color.blue);
    
    return LedError::SUCCESS;
}

LedError LedManager::SetAllLeds(uint32_t rgb) noexcept {
    return SetAllLeds(LedColor::FromRgb(rgb));
} 