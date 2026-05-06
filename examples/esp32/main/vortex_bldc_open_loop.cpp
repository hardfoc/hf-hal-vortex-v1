/**
 * @file vortex_bldc_open_loop.cpp
 * @brief Open-loop voltage-mode BLDC spin — tuned for Vortex bench + 24 V / 30 W geared motor.
 *
 * Uses `vortex_bench_safety.hpp` for pole pairs, current cap, velocity target, and spin time.
 * **Motion:** secure the shaft, stay within supply rating, use an e-stop path.
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
    ESP_LOGW(TAG,
             "MOTION APP — 24 V class | phase current cap %u mA | pole_pairs=%u | "
             "open-loop velocity=%ld (internal units) | spin %lu ms",
             static_cast<unsigned>(vortex_bench_safety::kMaxPhaseCurrentMa),
             static_cast<unsigned>(vortex_bench_safety::kDefaultPolePairs),
             static_cast<long>(vortex_bench_safety::kOpenLoopTargetVelocity),
             static_cast<unsigned long>(vortex_bench_safety::kOpenLoopSpinDurationMs));

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
        ESP_LOGE(TAG, "Init failed (Vortex or MotorController)");
        return;
    }

    auto& motors = VORTEX_API.motors;

    motors.visitDriver(
        [](auto& d) { vortex_motor_bench::log_motor_comms(TAG, d); },
        MotorController::ONBOARD_TMC9660_INDEX);

    bool configured = false;
    motors.visitDriver(
        [&configured](auto& d) {
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
                ESP_LOGE(TAG, "FOC_OPENLOOP_VOLTAGE_MODE failed");
                return;
            }
            configured = true;
            ESP_LOGI(TAG, "Motor configured: BLDC, open-loop voltage mode");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    if (!configured) {
        ESP_LOGE(TAG, "Aborting (configuration failed)");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kOpenLoopSettleBeforeSpinMs));

    motors.visitDriver(
        [](auto& d) {
            if (!d.velocityControl.setTargetVelocity(vortex_bench_safety::kOpenLoopTargetVelocity)) {
                ESP_LOGE(TAG, "setTargetVelocity failed");
                return;
            }
            ESP_LOGI(TAG, "Target velocity set — spinning...");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    motors.visitDriver(
        [](auto& d) { vortex_motor_bench::log_motor_comms(TAG, d); },
        MotorController::ONBOARD_TMC9660_INDEX);

    vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kOpenLoopSpinDurationMs));

    motors.visitDriver(
        [](auto& d) {
            vortex_motor_bench::motor_stop_safe(d);
            ESP_LOGI(TAG, "motor_stop_safe (torque stop + SYSTEM_OFF)");
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "Open-loop profile finished OK");
}
