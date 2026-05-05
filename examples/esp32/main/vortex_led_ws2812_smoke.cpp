/**
 * @file vortex_led_ws2812_smoke.cpp
 * @brief WS2812 smoke + visual demo on Vortex V1 (LedManager, pincfg `WS2812_LED_DAT` / GPIO3).
 *
 * Flash: `./scripts/build_app.sh vortex_led_ws2812_smoke Release` then
 * `./scripts/flash_app.sh flash_monitor vortex_led_ws2812_smoke Release`
 */
#include "api/Vortex.h"
#include "common/vortex_board_pins.hpp"
#include "managers/LedManager.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_led_ws2812_smoke";

static void delay_ms(uint32_t ms) noexcept { vTaskDelay(pdMS_TO_TICKS(ms)); }

static void run_animation_ticks(LedManager& leds, int ticks, uint32_t step_ms) noexcept {
  for (int i = 0; i < ticks; ++i) {
    (void)leds.UpdateAnimation();
    delay_ms(step_ms);
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "WS2812 demo — Vortex V1 (data GPIO %d per vortex_board_pins.hpp)",
           vortex_board_pins::kWs2812DataGpio);

  if (!VORTEX_API.EnsureInitialized()) {
    ESP_LOGE(TAG, "Vortex EnsureInitialized failed — check serial for component warnings");
    return;
  }

  auto& leds = VORTEX_API.leds;
  if (!leds.EnsureInitialized()) {
    ESP_LOGE(TAG, "LedManager init failed");
    return;
  }

  // ── 1) Boot-style pulse (blue blink status) ─────────────────────────────
  ESP_LOGI(TAG, "Phase 1: boot indicator");
  [[maybe_unused]] auto e_boot = leds.IndicateBoot();
  run_animation_ticks(leds, 24, 45);
  (void)leds.StopAnimation();
  delay_ms(200);

  // ── 2) Solid colour sweep ───────────────────────────────────────────────
  ESP_LOGI(TAG, "Phase 2: RGB + orange sweep");
  (void)leds.SetColor(LedColors::RED);
  delay_ms(450);
  (void)leds.SetColor(LedColors::GREEN);
  delay_ms(450);
  (void)leds.SetColor(LedColors::BLUE);
  delay_ms(450);
  (void)leds.SetColor(LedColor(255, 140, 0));
  delay_ms(450);
  (void)leds.SetBrightnessPercent(100);

  // ── 3) Rainbow (colour wheel — best “cool” effect on one pixel) ─────────
  ESP_LOGI(TAG, "Phase 3: rainbow (~3.2 s)");
  (void)leds.StartAnimation(LedAnimation::RAINBOW);
  run_animation_ticks(leds, 80, 40);
  (void)leds.StopAnimation();
  delay_ms(150);

  // ── 4) Breathing cyan ───────────────────────────────────────────────────
  ESP_LOGI(TAG, "Phase 4: breath cyan (~2.4 s)");
  (void)leds.StartAnimation(LedAnimation::BREATH, LedColors::CYAN);
  run_animation_ticks(leds, 60, 40);
  (void)leds.StopAnimation();
  delay_ms(150);

  // ── 5) Magenta blink ─────────────────────────────────────────────────────
  ESP_LOGI(TAG, "Phase 5: blink magenta (~1.6 s)");
  (void)leds.StartAnimation(LedAnimation::BLINK, LedColor(255, 0, 200));
  run_animation_ticks(leds, 40, 40);
  (void)leds.StopAnimation();
  delay_ms(150);

  // ── 6) Short brightness ramp on white ────────────────────────────────────
  ESP_LOGI(TAG, "Phase 6: brightness ramp");
  (void)leds.SetColor(LedColors::WHITE);
  for (unsigned p = 15; p <= 100; p += 17) {
    (void)leds.SetBrightnessPercent(static_cast<uint8_t>(p));
    delay_ms(120);
  }
  (void)leds.SetBrightnessPercent(85);

  // ── 7) Status colours (HAL presets) ──────────────────────────────────────
  ESP_LOGI(TAG, "Phase 7: status presets");
  (void)leds.SetStatus(LedAnimation::STATUS_WARN);
  run_animation_ticks(leds, 14, 45);
  (void)leds.SetStatus(LedAnimation::STATUS_ERROR);
  run_animation_ticks(leds, 14, 45);
  (void)leds.SetStatus(LedAnimation::STATUS_OK);
  run_animation_ticks(leds, 20, 45);

  // ── 8) Land on “ready” green ─────────────────────────────────────────────
  ESP_LOGI(TAG, "Phase 8: ready — solid green (hold ~3 s, then dim)");
  (void)leds.IndicateReady();
  run_animation_ticks(leds, 8, 50);
  (void)leds.StopAnimation();
  (void)leds.SetColor(LedColors::GREEN);
  (void)leds.SetBrightnessPercent(70);
  delay_ms(3000);
  (void)leds.SetBrightnessPercent(35);
  delay_ms(400);

  ESP_LOGI(TAG, "WS2812 demo complete — LED left dim green; reset or flash another app to change");
}
