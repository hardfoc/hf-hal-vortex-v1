/**
 * @file vortex_tmcl_param_scan_post_config.cpp
 * @brief **Post-config** TMCL read probe — same sweep as `vortex_tmcl_param_scan` but runs
 *        AFTER `configure_complete_bldc()` so the chip is in `MOTOR_TYPE = BLDC_MOTOR` with
 *        DRV_EN asserted before any GAP is issued.
 *
 * Purpose: empirically determine which axis parameters (notably NR 256–283 — the gate-driver
 * OC + VGS-short protection block, all marked **RWE** in the datasheet) become visible to GAP
 * only after a motor type is bound to motor 0. The "pre-config" baseline (`vortex_tmcl_param_scan`)
 * runs with `MOTOR_TYPE = NO_MOTOR`, where these NRs return `REPLY_WRONG_TYPE`. If our hypothesis
 * is right, the post-config scan should show those NRs returning `REPLY_OK`.
 *
 * Build twice from `app_config.yml`:
 *   - `vortex_tmcl_param_scan_post_config`     — **UART** (default).
 *   - `vortex_tmcl_param_scan_post_config_spi` — **SPI** (same source; `EXAMPLE_TYPE_*` gates).
 *
 * Safety: the scan does **not** start motion. `configure_complete_bldc` arms the chip but leaves
 * `COMMUTATION_MODE = SYSTEM_OFF` and `TARGET_VELOCITY = 0`, so the gate driver is enabled but
 * idle while the GAP/GGP sweep runs. Wall time ~40 s.
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "common/vortex_motor_bench_common.hpp"
#include "common/vortex_bench_safety.hpp"
#include "vortex_tmcl_read_probe.hpp"

#include "esp_log.h"

static const char* TAG = "vortex_tmcl_scan_post";

#if defined(EXAMPLE_TYPE_vortex_tmcl_param_scan_post_config_spi) && EXAMPLE_TYPE_vortex_tmcl_param_scan_post_config_spi
constexpr bool kUseSpiTransport = true;
#else
constexpr bool kUseSpiTransport = false;
#endif

extern "C" void app_main(void) {
    if constexpr (kUseSpiTransport) {
        ESP_LOGI(TAG, "Post-config TMCL read scan over **SPI**");
        esp_log_level_set("Tmc9660Uart", ESP_LOG_WARN);
        esp_log_level_set("TMC9660", ESP_LOG_WARN);
        Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Spi);
    } else {
        ESP_LOGI(TAG, "Post-config TMCL read scan over **UART**");
        esp_log_level_set("Tmc9660Uart", ESP_LOG_WARN);
        esp_log_level_set("TMC9660", ESP_LOG_WARN);
        Vortex::SetOnboardTmc9660Transport(VortexOnboardTmc9660Transport::Uart);
    }

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
        ESP_LOGE(TAG, "Init failed");
        return;
    }

    // Configure MOTOR_TYPE=BLDC + full bring-up (gate driver, current sensing, FOC, protection,
    // ramp, brake, heartbeat, power, DRV_EN). Same flag combo as `vortex_bldc_open_loop` but no
    // commutation-mode change and no setTargetVelocity — the chip stays armed-but-idle.
    bool armed = false;
    VORTEX_API.motors.visitDriver(
        [&](auto& d) {
            armed = vortex_motor_bench::configure_complete_bldc(
                d, TAG,
                vortex_bench_safety::kDefaultPolePairs,
                vortex_bench_safety::kPwmFrequencyHz,
                /*do_calibrate=*/true,
                /*enable_outputs=*/true,
                /*with_oc_vgs_protection=*/true,
                /*with_gate_current_limits=*/true,
                /*skip_y2_phase=*/true);
        },
        MotorController::ONBOARD_TMC9660_INDEX);
    if (!armed) {
        ESP_LOGE(TAG, "configure_complete_bldc failed — aborting scan");
        return;
    }

    ESP_LOGI(TAG, "BLDC armed (SYSTEM_OFF, DRV_EN active). Starting GAP/GGP sweep...");

    VORTEX_API.motors.visitDriver(
        [](auto& d) { vortex_tmcl_read_probe::scan_all_reads(d, TAG); },
        MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "Scan complete (post-config baseline). Compare WARN counts vs pre-config "
                  "snapshot in TMCL_PARAM_SCAN_FAILURES.md to identify motor-state-gated NRs.");
}
