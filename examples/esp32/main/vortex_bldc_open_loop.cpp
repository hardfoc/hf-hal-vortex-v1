/**
 * @file vortex_bldc_open_loop.cpp
 * @brief Short open-loop voltage spin — strict current cap, time-bounded (bench only).
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_bldc_open_loop";

extern "C" void app_main(void) {
    ESP_LOGW(TAG, "MOTION APP — secure motor, 24 V max, torque cap %u mA, max profile %lu ms",
             static_cast<unsigned>(vortex_bench_safety::kMaxPhaseCurrentMa),
             static_cast<unsigned long>(vortex_bench_safety::kMotorProfileMaxDurationMs));

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
        ESP_LOGE(TAG, "Init failed");
        return;
    }

    auto& motors = VORTEX_API.motors;
    motors.visitDriver(
        [](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            if (!vortex_motor_bench::configure_basic_bldc(d, vortex_bench_safety::kDefaultPolePairs)) {
                ESP_LOGE(TAG, "configure_basic_bldc failed");
                return;
            }
            if (!vortex_motor_bench::configure_torque_velocity_loops(d)) {
                ESP_LOGE(TAG, "torque/velocity configure failed");
                return;
            }
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_OPENLOOP_VOLTAGE_MODE)) {
                ESP_LOGE(TAG, "open-loop voltage mode failed");
                return;
            }
            (void)d.velocityControl.setTargetVelocity(400);
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    vTaskDelay(pdMS_TO_TICKS(800));
    motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d); },
                       MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "Open-loop profile finished");
}
