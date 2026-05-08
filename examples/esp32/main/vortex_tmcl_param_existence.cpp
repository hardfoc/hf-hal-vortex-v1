/**
 * @file vortex_tmcl_param_existence.cpp
 * @brief ESP32 harness: per-NR read path inventory (GAP vs GGP) for TMC9660 parameter mode ROM.
 *
 * Flash: `./scripts/flash_app.sh --port /dev/ttyACM0 flash vortex_tmcl_param_existence Release`
 * Capture: `timeout 180 ... monitor 2>&1 | tee tmcl_existence.log` then `grep INV tmcl_existence.log`.
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_tmcl_param_existence.hpp"

#include "esp_log.h"

static const char* TAG = "tmcl_exist";

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "TMCL parameter existence matrix");

  if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
    ESP_LOGE(TAG, "Init failed");
    return;
  }

  VORTEX_API.motors.visitDriver(
      [](auto& d) { vortex_tmcl_param_existence::run_existence_matrix(d, TAG); },
      MotorController::ONBOARD_TMC9660_INDEX);
}
