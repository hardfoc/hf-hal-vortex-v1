/**
 * @file LedManager.cpp
 * @brief Implementation of the Vortex V1 LED manager (single WS2812 LED).
 *
 * @author HardFOC Team
 * @date 2026
 * @version 2.0
 */

#include "LedManager.h"

#include "bsp_bind/VortexBoardBind.h"

#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"

#include <cmath>

static constexpr const char* TAG = "VortexLed";

//==============================================================================
// SINGLETON
//==============================================================================

LedManager& LedManager::GetInstance() noexcept {
    static LedManager instance;
    return instance;
}

//==============================================================================
// LIFECYCLE
//==============================================================================

bool LedManager::EnsureInitialized() noexcept {
    if (is_initialized_.load(std::memory_order_acquire)) return true;
    MutexLockGuard lock(mutex_);
    if (is_initialized_.load(std::memory_order_acquire)) return true;
    return Initialize();
}

bool LedManager::Shutdown() noexcept {
    MutexLockGuard lock(mutex_);
    if (!is_initialized_.load(std::memory_order_acquire)) return true;

    (void)StopAnimation();
    (void)ShowPixel(0, 0);
    vortex::bsp::StatusLedClose();
    current_animation_ = LedAnimation::OFF;
    current_color_ = LedColors::BLACK;
    is_initialized_.store(false, std::memory_order_release);

    Logger::GetInstance().Info(TAG, "LED manager shutdown");
    return true;
}

LedError LedManager::GetSystemDiagnostics(LedSystemDiagnostics& d) const noexcept {
    MutexLockGuard lock(mutex_);
    d.system_healthy      = is_initialized_.load() && last_error_.load() == LedError::SUCCESS;
    d.led_initialized     = is_initialized_.load();
    d.animation_active    = IsAnimationActive();
    d.current_animation   = current_animation_;
    d.total_operations    = total_operations_.load();
    d.successful_operations = successful_operations_.load();
    d.failed_operations   = failed_operations_.load();
    d.animation_cycles    = animation_cycles_.load();
    d.current_brightness  = current_brightness_;
    d.current_color       = current_color_.ToRgb();
    return LedError::SUCCESS;
}

//==============================================================================
// BASIC OPERATIONS
//==============================================================================

LedError LedManager::SetColor(const LedColor& color, uint32_t led_index) noexcept {
    if (!EnsureInitialized()) { UpdateLastError(LedError::NOT_INITIALIZED); return LedError::NOT_INITIALIZED; }
    if (led_index >= kNumLeds) { UpdateLastError(LedError::INVALID_PARAMETER); return LedError::INVALID_PARAMETER; }

    MutexLockGuard lock(mutex_);

    if (current_animation_ != LedAnimation::OFF) (void)StopAnimation();

    if (!ShowPixel(color.ToRgb(), current_brightness_)) {
        UpdateLastError(LedError::HARDWARE_ERROR);
        UpdateStatistics(false);
        return LedError::HARDWARE_ERROR;
    }

    if (led_index == 0) {
        current_color_ = color;
        current_animation_ = LedAnimation::SOLID;
    }
    UpdateStatistics(true);
    return LedError::SUCCESS;
}

LedError LedManager::SetColor(uint32_t rgb, uint32_t led_index) noexcept {
    return SetColor(LedColor::FromRgb(rgb), led_index);
}

LedError LedManager::TurnOff() noexcept {
    if (!EnsureInitialized()) { UpdateLastError(LedError::NOT_INITIALIZED); return LedError::NOT_INITIALIZED; }

    MutexLockGuard lock(mutex_);
    if (current_animation_ != LedAnimation::OFF) (void)StopAnimation();

    if (!ShowPixel(0, current_brightness_)) {
        UpdateLastError(LedError::HARDWARE_ERROR);
        UpdateStatistics(false);
        return LedError::HARDWARE_ERROR;
    }

    current_color_ = LedColors::BLACK;
    current_animation_ = LedAnimation::OFF;
    UpdateStatistics(true);
    return LedError::SUCCESS;
}

LedError LedManager::GetCurrentColor(LedColor& color) const noexcept {
    MutexLockGuard lock(mutex_);
    color = current_color_;
    return LedError::SUCCESS;
}

//==============================================================================
// BRIGHTNESS
//==============================================================================

LedError LedManager::SetBrightnessPercent(uint8_t brightness_percent) noexcept {
    if (!EnsureInitialized()) return LedError::NOT_INITIALIZED;
    if (brightness_percent > 100) return LedError::INVALID_BRIGHTNESS;

    MutexLockGuard lock(mutex_);
    current_brightness_ = PercentToRaw(brightness_percent);
    const bool ok = ShowPixel(current_color_.ToRgb(), current_brightness_);
    UpdateStatistics(ok);
    return ok ? LedError::SUCCESS : LedError::HARDWARE_ERROR;
}

LedError LedManager::SetBrightnessRaw(uint8_t brightness_raw) noexcept {
    if (!EnsureInitialized()) return LedError::NOT_INITIALIZED;

    MutexLockGuard lock(mutex_);
    current_brightness_ = brightness_raw;
    const bool ok = ShowPixel(current_color_.ToRgb(), current_brightness_);
    UpdateStatistics(ok);
    return ok ? LedError::SUCCESS : LedError::HARDWARE_ERROR;
}

LedError LedManager::SetMaxBrightness(uint8_t max_brightness) noexcept {
    if (!EnsureInitialized()) return LedError::NOT_INITIALIZED;
    MutexLockGuard lock(mutex_);
    max_brightness_ = max_brightness;
    return LedError::SUCCESS;
}

//==============================================================================
// ANIMATION
//==============================================================================

LedError LedManager::StartAnimation(LedAnimation animation, const LedColor& color) noexcept {
    if (!EnsureInitialized()) return LedError::NOT_INITIALIZED;

    MutexLockGuard lock(mutex_);
    if (current_animation_ != LedAnimation::OFF) (void)StopAnimation();

    current_animation_ = animation;
    current_color_ = color;
    animation_step_ = 0;
    animation_direction_ = 1;
    last_animation_update_ = GetSystemUptimeMs();

    switch (animation) {
        case LedAnimation::STATUS_OK:        current_color_ = LedColors::GREEN;  break;
        case LedAnimation::STATUS_WARN:      current_color_ = LedColors::YELLOW; break;
        case LedAnimation::STATUS_ERROR:     current_color_ = LedColors::RED;    break;
        case LedAnimation::STATUS_BOOT:      current_color_ = LedColors::BLUE;   break;
        case LedAnimation::STATUS_CALIBRATE: current_color_ = LedColors::PURPLE; break;
        default: break;
    }

    // First frame: UpdateAnimation() rate-limits to ANIMATION_UPDATE_INTERVAL, so without
    // this STATUS_OK / SOLID never hit the strip and the previous test colour could stick.
    const LedError first = UpdateAnimationStep();
    last_animation_update_ = GetSystemUptimeMs();
    UpdateStatistics(first == LedError::SUCCESS);
    return first;
}

LedError LedManager::StopAnimation() noexcept {
    MutexLockGuard lock(mutex_);
    if (current_animation_ == LedAnimation::OFF) return LedError::SUCCESS;
    if (!is_initialized_.load(std::memory_order_acquire)) return LedError::NOT_INITIALIZED;
    (void)ShowPixel(0, current_brightness_);
    current_animation_ = LedAnimation::OFF;
    current_color_ = LedColors::BLACK;
    animation_step_ = 0;
    return LedError::SUCCESS;
}

LedError LedManager::UpdateAnimation() noexcept {
    if (!EnsureInitialized()) return LedError::NOT_INITIALIZED;
    MutexLockGuard lock(mutex_);
    if (current_animation_ == LedAnimation::OFF) return LedError::SUCCESS;

    uint64_t now = GetSystemUptimeMs();
    if (now - last_animation_update_ < ANIMATION_UPDATE_INTERVAL) return LedError::SUCCESS;
    last_animation_update_ = now;

    LedError err = UpdateAnimationStep();
    if (err == LedError::SUCCESS) animation_cycles_.fetch_add(1, std::memory_order_relaxed);
    return err;
}

//==============================================================================
// UTILITY
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

void LedManager::DumpStatistics() const noexcept {
    auto& log = Logger::GetInstance();
    log.Info(TAG, "=== Vortex LED Manager Statistics ===");
    log.Info(TAG, "  Init: %s  GPIO: %d  LEDs: %u",
             is_initialized_.load() ? "YES" : "NO", current_led_gpio_, kNumLeds);
    log.Info(TAG, "  Animation: %s  Color: 0x%06X  Brightness: %u",
             LedAnimationToString(current_animation_), current_color_.ToRgb(), current_brightness_);
    log.Info(TAG, "  Ops total/ok/fail: %u/%u/%u  Cycles: %u",
             total_operations_.load(), successful_operations_.load(),
             failed_operations_.load(), animation_cycles_.load());
    log.Info(TAG, "=== End Vortex LED Manager Statistics ===");
}

//==============================================================================
// PRIVATE
//==============================================================================

bool LedManager::Initialize() noexcept {
    Logger::GetInstance().Info(TAG, "Initializing Vortex LED manager");

    const int led_gpio = GetLedGpioPin();
    current_led_gpio_ = led_gpio;
    if (led_gpio < 0) {
        Logger::GetInstance().Error(TAG, "WS2812_LED_DAT is not an MCU pin in the pin map");
        UpdateLastError(LedError::INITIALIZATION_FAILED);
        return false;
    }

    if (!vortex::bsp::StatusLedOpen(led_gpio, kNumLeds)) {
        Logger::GetInstance().Error(TAG, "Failed to open the WS2812 status LED on GPIO %d", led_gpio);
        UpdateLastError(LedError::INITIALIZATION_FAILED);
        return false;
    }

    // Start with LED off
    (void)ShowPixel(0, current_brightness_);

    system_start_time_ = GetSystemUptimeMs();
    is_initialized_.store(true, std::memory_order_release);
    Logger::GetInstance().Info(TAG, "LED manager initialized on GPIO %d", led_gpio);
    return true;
}

bool LedManager::ShowPixel(uint32_t rgb, uint8_t brightness) noexcept {
    return vortex::bsp::StatusLedShow(&rgb, 1, brightness);
}

LedError LedManager::UpdateAnimationStep() noexcept {
    if (!is_initialized_.load(std::memory_order_acquire)) return LedError::NOT_INITIALIZED;

    bool shown = true;
    switch (current_animation_) {
        case LedAnimation::BLINK:
        case LedAnimation::STATUS_WARN:
        case LedAnimation::STATUS_ERROR:
        case LedAnimation::STATUS_CALIBRATE: {
            const bool on = (animation_step_ / (BLINK_SPEED / ANIMATION_UPDATE_INTERVAL)) % 2 == 0;
            shown = ShowPixel(on ? current_color_.ToRgb() : 0, current_brightness_);
            break;
        }
        case LedAnimation::BREATH:
        case LedAnimation::STATUS_BOOT: {
            // The breathing brightness applies to this frame only; the set brightness is kept.
            const uint8_t pct = static_cast<uint8_t>(50 + 50 * std::sin(animation_step_ * BREATH_SPEED * 0.01f));
            shown = ShowPixel(current_color_.ToRgb(), PercentToRaw(pct));
            break;
        }
        case LedAnimation::RAINBOW:
            shown = ShowPixel(ColorWheel(static_cast<uint8_t>(animation_step_ % 256)), current_brightness_);
            break;
        case LedAnimation::SOLID:
        case LedAnimation::STATUS_OK:
            shown = ShowPixel(current_color_.ToRgb(), current_brightness_);
            break;
        default:
            break;
    }
    if (!shown) return LedError::ANIMATION_FAILED;

    animation_step_++;
    return LedError::SUCCESS;
}

void LedManager::UpdateStatistics(bool success) noexcept {
    total_operations_.fetch_add(1, std::memory_order_relaxed);
    if (success) successful_operations_.fetch_add(1, std::memory_order_relaxed);
    else         failed_operations_.fetch_add(1, std::memory_order_relaxed);
}

void LedManager::UpdateLastError(LedError error_code) noexcept {
    last_error_.store(error_code, std::memory_order_release);
}

uint64_t LedManager::GetSystemUptimeMs() const noexcept {
    return static_cast<uint64_t>(os_get_elapsed_time_msec());
}
