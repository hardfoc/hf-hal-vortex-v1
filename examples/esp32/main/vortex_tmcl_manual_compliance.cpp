/**
 * @file vortex_tmcl_manual_compliance.cpp
 * @brief ESP32 harness: representative TMCL operations vs *TMC9660 Parameter Mode Reference Manual*
 *        (Analog Devices Rev 0, 02/25).
 *
 * Flash: `./scripts/flash_app.sh --port /dev/ttyACM0 flash vortex_tmcl_manual_compliance Release`
 * Monitor ~5s for PASS/FAIL lines.
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_tmcl_manual_compliance.hpp"

#include "esp_log.h"

static const char* TAG = "tmcl_manual";

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "TMCL manual compliance (reference manual representative checks)");

  if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
    ESP_LOGE(TAG, "Init failed");
    return;
  }

  VORTEX_API.motors.visitDriver(
      [](auto& d) { vortex_tmcl_manual_compliance::run_checks(d, TAG); },
      MotorController::ONBOARD_TMC9660_INDEX);
}
