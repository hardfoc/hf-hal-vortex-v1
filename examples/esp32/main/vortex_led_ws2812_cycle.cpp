/**
 * @file vortex_led_ws2812_cycle.cpp
 * @brief Continuous WS2812 exercise: colours, brightness, rainbow, breath, blink (infinite loop).
 *
 * Use for bench bring-up when you want to stress `LedManager` + RMT over a long time.
 * Flash: `./scripts/build_app.sh vortex_led_ws2812_cycle Release` then
 * `./scripts/flash_app.sh flash_monitor vortex_led_ws2812_cycle Release`
 */
#include "api/Vortex.h"
#include "common/vortex_board_pins.hpp"
#include "managers/LedManager.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_led_cycle";

static void delay_ms(uint32_t ms) noexcept { vTaskDelay(pdMS_TO_TICKS(ms)); }

static void run_animation_ticks(LedManager& leds, int ticks, uint32_t step_ms) noexcept {
  for (int i = 0; i < ticks; ++i) {
    (void)leds.UpdateAnimation();
    delay_ms(step_ms);
  }
}

/** Ramp global strip brightness while holding a solid colour. */
static void brightness_ramp_on_color(LedManager& leds, const LedColor& c,
                                     uint8_t pct_lo, uint8_t pct_hi, uint8_t step) noexcept {
  (void)leds.StopAnimation();
  (void)leds.SetColor(c);
  uint16_t p = pct_lo;
  for (;;) {
    const uint8_t bp = static_cast<uint8_t>(p > 100 ? 100 : p);
    (void)leds.SetBrightnessPercent(bp);
    delay_ms(90);
    if (p >= pct_hi) {
      break;
    }
    const uint16_t next = static_cast<uint16_t>(p + step);
    p = next > pct_hi ? pct_hi : next;
  }
}

/** Step around the colour wheel (single pixel). */
static void colour_wheel_sweep(LedManager& leds, uint32_t dwell_ms) noexcept {
  (void)leds.StopAnimation();
  (void)leds.SetMaxBrightness(255);
  (void)leds.SetBrightnessPercent(55);
  for (uint16_t pos = 0; pos < 256; pos += 6) {
    const uint32_t rgb = LedManager::ColorWheel(static_cast<uint8_t>(pos));
    (void)leds.SetColor(LedColor::FromRgb(rgb));
    delay_ms(dwell_ms);
  }
}

/** One full visual cycle (returns so outer loop can log + repeat). */
static void run_demo_cycle(LedManager& leds, unsigned cycle_index) noexcept {
  ESP_LOGI(TAG, "=== Cycle %u start ===", cycle_index);

  // 1) Hue sweep (wheel) at mid brightness
  colour_wheel_sweep(leds, 55);

  // 2) Brightness ramps on saturated primaries / secondaries
  static constexpr LedColor kRampColors[] = {
      LedColors::RED, LedColors::GREEN, LedColors::BLUE, LedColors::CYAN,
      LedColor(255, 128, 0), LedColor(200, 0, 255), LedColors::WHITE,
  };
  for (const auto& c : kRampColors) {
    brightness_ramp_on_color(leds, c, 6, 100, 8);
    delay_ms(120);
  }

  // 3) Short solid “palette” at staggered brightness levels
  struct {
    LedColor color;
    uint8_t pct;
  } const kHolds[] = {
      {LedColors::RED, 22},
      {LedColors::GREEN, 40},
      {LedColors::BLUE, 18},
      {LedColor(255, 200, 0), 70},
      {LedColor(0, 200, 180), 33},
      {LedColor(255, 0, 128), 88},
  };
  for (const auto& h : kHolds) {
    (void)leds.StopAnimation();
    (void)leds.SetBrightnessPercent(h.pct);
    (void)leds.SetColor(h.color);
    delay_ms(380);
  }

  // 4) Built-in animations (time-bounded segments)
  (void)leds.SetBrightnessPercent(100);
  (void)leds.StartAnimation(LedAnimation::RAINBOW);
  run_animation_ticks(leds, 96, 32);
  (void)leds.StopAnimation();
  delay_ms(120);

  (void)leds.SetBrightnessPercent(85);
  (void)leds.StartAnimation(LedAnimation::BREATH, LedColor(255, 90, 0));
  run_animation_ticks(leds, 72, 35);
  (void)leds.StopAnimation();
  delay_ms(120);

  (void)leds.SetBrightnessPercent(100);
  (void)leds.StartAnimation(LedAnimation::BLINK, LedColor(120, 0, 255));
  run_animation_ticks(leds, 48, 35);
  (void)leds.StopAnimation();
  delay_ms(120);

  // 5) Status presets (each drives its own colour / timing)
  (void)leds.SetBrightnessPercent(90);
  (void)leds.SetStatus(LedAnimation::STATUS_BOOT);
  run_animation_ticks(leds, 28, 40);
  (void)leds.SetStatus(LedAnimation::STATUS_WARN);
  run_animation_ticks(leds, 20, 40);
  (void)leds.SetStatus(LedAnimation::STATUS_ERROR);
  run_animation_ticks(leds, 20, 40);
  (void)leds.SetStatus(LedAnimation::STATUS_CALIBRATE);
  run_animation_ticks(leds, 36, 40);
  (void)leds.StopAnimation();

  // 6) Micro “sparkle”: quick low-high-low on white (exercises fast Show path)
  (void)leds.SetColor(LedColors::WHITE);
  for (uint8_t p : {12U, 35U, 75U, 100U, 30U, 10U}) {
    (void)leds.SetBrightnessPercent(p);
    delay_ms(70);
  }

  ESP_LOGI(TAG, "=== Cycle %u end (repeating forever) ===", cycle_index);
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "WS2812 continuous demo — GPIO %d (see vortex_board_pins.hpp)", vortex_board_pins::kWs2812DataGpio);

  if (!VORTEX_API.EnsureInitialized()) {
    ESP_LOGE(TAG, "Vortex EnsureInitialized failed — see earlier logs");
    return;
  }

  auto& leds = VORTEX_API.leds;
  if (!leds.EnsureInitialized()) {
    ESP_LOGE(TAG, "LedManager init failed");
    return;
  }

  (void)leds.SetMaxBrightness(255);
  unsigned cycle = 0;
  for (;;) {
    run_demo_cycle(leds, cycle);
    ++cycle;
    delay_ms(400);
  }
}
