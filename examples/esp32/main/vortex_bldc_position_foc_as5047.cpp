/**
 * @file vortex_bldc_position_foc_as5047.cpp
 * @brief Position loop on ABN-style feedback — AS5047U angle via EncoderManager; TMC uses ABN CPR.
 *
 * For true FOC_SPI_ENC on the TMC9660, configure `FeedbackSense::configureAuto(SpiEncoderConfig)`
 * to match AS5047 framing (see driver examples). This app uses ABN commutation + position PI as a
 * documented stepping stone; gearbox 10:1 should be reflected in mechanical limits in product code.
 */
#include "api/Vortex.h"
#include "managers/EncoderManager.h"
#include "managers/MotorController.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <type_traits>

static const char* TAG = "vortex_bldc_pos";

extern "C" void app_main(void) {
    ESP_LOGW(TAG, "MOTION APP — position segment; torque cap %u mA",
             static_cast<unsigned>(vortex_bench_safety::kMaxPhaseCurrentMa));

    if (!VORTEX_API.EnsureInitialized() || !VORTEX_API.motors.EnsureInitialized()) {
        ESP_LOGE(TAG, "Init failed");
        return;
    }

    if (VORTEX_API.encoders.EnsureInitialized()) {
        uint16_t ang = 0;
        (void)VORTEX_API.encoders.ReadAngle(0, ang);
        ESP_LOGI(TAG, "AS5047 path (EncoderManager slot 0) raw angle sample: %u", static_cast<unsigned>(ang));
    }

    constexpr uint32_t kShaftCpr = 16384;

    VORTEX_API.motors.visitDriver(
        [kShaftCpr](auto& d) {
            namespace tmcl = tmc9660::tmcl;
            if (!vortex_motor_bench::configure_basic_bldc(d, vortex_bench_safety::kDefaultPolePairs)) {
                return;
            }
            if (!d.feedbackSense.configureABNEncoder(kShaftCpr)) {
                ESP_LOGE(TAG, "configureABNEncoder failed");
                return;
            }
            if (!vortex_motor_bench::configure_torque_velocity_loops(d)) {
                return;
            }
            if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_ABN)) {
                return;
            }

            typename std::remove_reference_t<decltype(d)>::PositionControl::PositionConfig pc{};
            pc.sensorSelection = tmcl::PositionSensorSelection::SAME_AS_COMMUTATION;
            pc.encoderCountsPerRev = kShaftCpr;
            if (!d.positionControl.configureAuto(pc)) {
                ESP_LOGE(TAG, "positionControl.configureAuto failed");
                return;
            }

            (void)d.positionControl.setTargetPositionRaw(200);
        },
        MotorController::ONBOARD_TMC9660_INDEX);

    vTaskDelay(pdMS_TO_TICKS(400));
    VORTEX_API.motors.visitDriver([](auto& d) { (void)d.positionControl.setTargetPositionRaw(0); },
                                  MotorController::ONBOARD_TMC9660_INDEX);
    vTaskDelay(pdMS_TO_TICKS(400));
    VORTEX_API.motors.visitDriver([](auto& d) { vortex_motor_bench::motor_stop_safe(d); },
                                  MotorController::ONBOARD_TMC9660_INDEX);

    ESP_LOGI(TAG, "Position profile segment done (CPR=%lu placeholder for shaft encoder)",
             static_cast<unsigned long>(kShaftCpr));
}
