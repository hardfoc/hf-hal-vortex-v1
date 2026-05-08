/**
 * @file vortex_tmcl_param_scan.cpp
 * @brief Exhaustive TMCL read probe: **GAP** on **motor 0** (Table 57 NRs 0…334, gaps skipped) +
 *        **GGP** (banks 0, 2, 3) over numeric TYPE ranges.
 *
 * Run after normal MotorController init (parameter mode). Logs every non-OK read with TMCL status
 * and ends with per-opcode histograms. Use to inventory what this ROM accepts vs driver tables.
 *
 * Build twice from `app_config.yml`:
 *   - `vortex_tmcl_param_scan` — **UART** (explicit).
 *   - `vortex_tmcl_param_scan_spi` — **SPI** (same source; `EXAMPLE_TYPE_*` selects transport).
 *
 * Typical wall time to `Scan complete` is on the order of 40s; serial capture can use e.g. `timeout 90`.
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_tmcl_read_probe.hpp"

#include "esp_log.h"

static const char* TAG = "vortex_tmcl_scan";

#if defined(EXAMPLE_TYPE_vortex_tmcl_param_scan_spi) && EXAMPLE_TYPE_vortex_tmcl_param_scan_spi
constexpr bool kUseSpiTransport = true;
#else
constexpr bool kUseSpiTransport = false;
#endif

extern "C" void app_main(void) {
  if constexpr (kUseSpiTransport) {
    ESP_LOGI(TAG, "TMCL parameter read scan (GAP motor0 + GGP banks 0/2/3) over **SPI**");
    // SPI wire trace uses the same driver log macros; keep UART quiet if both are compiled in.
    esp_log_level_set("Tmc9660Uart", ESP_LOG_WARN);
    esp_log_level_set("TMC9660", ESP_LOG_WARN);
    Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Spi);
  } else {
    ESP_LOGI(TAG, "TMCL parameter read scan (GAP motor0 + GGP banks 0/2/3) over **UART**");
    // Wire-trace from HalUartTmc9660Comm is too noisy for a 600+ frame scan.
    esp_log_level_set("Tmc9660Uart", ESP_LOG_WARN);
    esp_log_level_set("TMC9660", ESP_LOG_WARN);
    Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Uart);
  }

  if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
    ESP_LOGE(TAG, "Init failed");
    return;
  }

  VORTEX_API.motors.visitDriver(
      [](auto& d) { vortex_tmcl_read_probe::scan_all_reads(d, TAG); },
      MotorController::ONBOARD_TMC9660_INDEX);

  ESP_LOGI(TAG, "Scan complete");
}
