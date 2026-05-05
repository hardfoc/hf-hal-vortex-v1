/**
 * @file vortex_bldc_velocity_foc_hall.cpp
 * @brief FOC velocity with Hall commutation — bench with motor + halls only.
 */
#include "api/Vortex.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_bldc_vel_hall";

extern "C" void app_main(void) {
    ESP_LOGW(TAG, "MOTION APP — Hall FOC; cap %u mA; verify halls + phase order on bench",
             static_cast<unsigned>(vortex_bench_safety::kMaxPhaseCurrentMa));

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
        ESP_LOGE(TAG, "Init failed");
        return;
    }

    VORTEX_API.motors.visitDriver(
        [](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            if (!vortex_motor_bench::configure_basic_bldc(d, vortex_bench_safety::kDefaultPolePairs)) {
                return;
            }
            if (!d.feedbackSense.configureHall()) {
                ESP_LOGE(TAG, "configureHall failed");
                return;
            }
            if (!vortex_motor_bench::configure_torque_velocity_loops(d)) {
                return;
            }
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_HALL_SENSOR)) {
                ESP_LOGE(TAG, "FOC_HALL_SENSOR mode failed");
                return;
            }
            (void)d.velocityControl.setTargetVelocity(600);
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    vTaskDelay(pdMS_TO_TICKS(1200));
    VORTEX_API.motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d); },
                                  MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "Hall velocity bench segment done");
}
