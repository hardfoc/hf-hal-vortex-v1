/**
 * @file vortex_tmcl_gap_smoke.cpp
 * @brief Minimal TMCL-over-UART check: init onboard TMC9660 on UART, one GAP read (type 0, motor 0).
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "common/vortex_motor_bench_probe.hpp"

#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/parameter_mode/tmc9660_param_mode_tmcl.hpp"
#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp"

#include "esp_log.h"

static const char* TAG = "tmcl_gap_smoke";

extern "C" void app_main(void) {
  esp_log_level_set("Tmc9660Uart", ESP_LOG_DEBUG);

  Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Uart);

  if (!VORTEX_API.EnsureInitialized()) {
    ESP_LOGE(TAG, "Vortex init failed");
    return;
  }
  // After Vortex bring-up, dump the raw PCAL95555 levels for the TMC9660 control
  // nets. This tells us — from the host side — whether WAKE is driven LOW (chip
  // awake), DRV_EN is HIGH, RST is LOW (released), FAULTN is HIGH (no fault),
  // PWR_GOOD is HIGH, and that the SPI mux is released for UART. If those lines
  // disagree with what the bootloader sequence expects, no UART reply will come
  // even though the host is happily transmitting.
  vortex_motor_bench_probe::dump_pcal95555_lines(TAG, "post-Vortex-init");
  // Vortex already ran MotorController::EnsureInitialized once; do not call it again (would repeat
  // full bootloader/TMC bring-up). Require a fully initialized onboard handler for TMCL.
  if (!VORTEX_API.motors.IsInitialized()) {
    ESP_LOGE(TAG, "MotorController not initialized after Vortex (bootloader / transport)");
    return;
  }

  VORTEX_API.motors.visitDriver(
      [](auto& driver) {
        uint32_t val = 0;
        tmc9660::tmcl::ReplyCode st = tmc9660::tmcl::ReplyCode::REPLY_OK;
        const bool xfer =
            driver.sendCommand(tmc9660::tmcl::Op::GAP, 0, 0, 0, &val, &st);
        ESP_LOGI(TAG,
                 "GAP type=0 motor=0: xfer_ok=%d TMCL=%s (0x%02X) value=0x%08lX",
                 static_cast<int>(xfer), tmc9660::tmcl::to_string(st),
                 static_cast<unsigned>(static_cast<std::uint8_t>(st)),
                 static_cast<unsigned long>(val));
      },
      MotorController::ONBOARD_TMC9660_INDEX);

  ESP_LOGI(TAG, "done");
}
