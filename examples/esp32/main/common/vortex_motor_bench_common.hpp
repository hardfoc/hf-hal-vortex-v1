/**
 * @file vortex_motor_bench_common.hpp
 * @brief Shared TMC9660 bring-up helpers for Vortex examples (via MotorController::visitDriver).
 */
#pragma once

#include "vortex_bench_safety.hpp"
#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp"
#include "esp_log.h"

namespace vortex_motor_bench {

namespace tmcl = tmc9660::tmcl;

template <typename Comm>
bool configure_basic_bldc(tmc9660::TMC9660<Comm>& d, uint8_t pole_pairs) {
    if (!d.motorConfig.setType(tmcl::MotorType::BLDC_MOTOR, pole_pairs)) {
        return false;
    }
    if (!d.motorConfig.setPWMFrequency(20000)) {
        return false;
    }
    if (!d.motorConfig.setMaxTorqueCurrent(vortex_bench_safety::kMaxPhaseCurrentMa)) {
        return false;
    }
    if (!d.motorConfig.setMaxFluxCurrent(150)) {
        return false;
    }
    return true;
}

template <typename Comm>
bool configure_torque_velocity_loops(tmc9660::TMC9660<Comm>& d) {
    typename tmc9660::TMC9660<Comm>::TorqueFluxControl::TorqueFluxConfig tf{};
    tf.torqueP = 50;
    tf.torqueI = 100;
    if (!d.torqueFluxControl.configureAuto(tf)) {
        return false;
    }
    typename tmc9660::TMC9660<Comm>::VelocityControl::VelocityConfig vc{};
    vc.sensorSelection = tmcl::VelocitySensorSelection::SAME_AS_COMMUTATION;
    vc.velocityP = 1000;
    vc.velocityI = 2;
    vc.velocityScalingFactor = 1;
    vc.velocityOffset = 0;
    return d.velocityControl.configureAuto(vc);
}

template <typename Comm>
void motor_stop_safe(tmc9660::TMC9660<Comm>& d) {
    d.torqueFluxControl.stop();
    (void)d.motorConfig.setCommutationMode(tmcl::CommutationMode::SYSTEM_OFF);
}

template <typename Comm>
void log_motor_comms(const char* tag, tmc9660::TMC9660<Comm>& d) {
    const float v = d.telemetry.getSupplyVoltage();
    const float t = d.telemetry.getChipTemperature();
    ESP_LOGI(tag, "TMC9660 telemetry: supply=%.2f V, chip_temp=%.1f C", static_cast<double>(v),
             static_cast<double>(t));
}

}  // namespace vortex_motor_bench
