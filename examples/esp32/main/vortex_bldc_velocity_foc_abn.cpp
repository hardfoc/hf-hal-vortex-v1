/**
 * @file vortex_bldc_velocity_foc_abn.cpp
 * @brief FOC velocity with ABN encoder commutation — bench with incremental encoder on shaft.
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_bldc_vel_abn";

extern "C" void app_main(void) {
    ESP_LOGW(TAG, "MOTION APP — ABN FOC; torque cap %u mA; set CPR to your encoder",
             static_cast<unsigned>(vortex_bench_safety::kMaxPhaseCurrentMa));

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
        ESP_LOGE(TAG, "Init failed");
        return;
    }

    constexpr uint32_t kEncoderCpr = 1024;

    VORTEX_API.motors.visitDriver(
        [kEncoderCpr](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            if (!vortex_motor_bench::configure_basic_bldc(d, vortex_bench_safety::kDefaultPolePairs)) {
                return;
            }
            if (!d.feedbackSense.configureABNEncoder(kEncoderCpr)) {
                ESP_LOGE(TAG, "configureABNEncoder failed");
                return;
            }
            if (!vortex_motor_bench::configure_torque_velocity_loops(d)) {
                return;
            }
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_ABN)) {
                ESP_LOGE(TAG, "FOC_ABN mode failed");
                return;
            }
            (void)d.velocityControl.setTargetVelocity(500);
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    vTaskDelay(pdMS_TO_TICKS(1200));
    VORTEX_API.motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d); },
                                  MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "ABN velocity bench segment done (CPR=%lu)", static_cast<unsigned long>(kEncoderCpr));
}
