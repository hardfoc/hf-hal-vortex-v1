/**
 * @file vortex_motor_bench_common.hpp
 * @brief Shared TMC9660 BLDC bring-up helpers for Vortex parameter-mode bench apps.
 *
 * Wraps the TMC9660 driver's `configureAuto(...)` surfaces (motor, gate driver,
 * current sensing, FOC, protection, ramp, brake) with values keyed off
 * `vortex_bench_safety.hpp`. Step ordering mirrors the EvKit
 * `bldc_comprehensive_test.cpp::configureCompleteBLDCMotor()`:
 *
 *   1. SYSTEM_OFF
 *   2. Gate driver (must precede DRV_EN)
 *   3. Current sensing
 *   4. Motor parameters (type, pole pairs, PWM, voltage limit, idle PWM)
 *   5. FOC torque/flux + velocity loop gains
 *   6. Protection (OV/UV, OT, OC, I²t)
 *   7. Ramp (direct velocity mode by default; ramp gen disabled)
 *   8. Brake (disabled by default — enable per board)
 *   9. Stop events / heartbeat / power (best-effort, optional)
 *  10. Current sense ADC offset calibration (motor stationary, SYSTEM_OFF)
 *  11. DRV_EN → ACTIVE
 *
 * After step 11 the chip is **armed but still in `SYSTEM_OFF`**. The caller
 * sets the desired commutation mode (`FOC_OPENLOOP_VOLTAGE_MODE`,
 * `FOC_HALL_SENSOR`, `FOC_ABN`, etc.) and writes `setTargetVelocity(...)` /
 * `setTargetTorque(...)`.
 *
 * **Sensor bootloader prerequisites:** Hall and ABN feedback also require
 * the bootloader to mux the right GPIOs (Hall on GPIO2/3/4; ABN1 on
 * GPIO8/13/14 in the EvKit map). Vortex's `Tmc9660Handler::kDefaultBootConfig`
 * leaves Hall/ABN disabled — pass a custom `BootloaderConfig*` with
 * `cfg.hall.enable = true` (and/or `cfg.abn1.enable = true`) when calling
 * `MotorController::CreateOnboardDevice(...)` if you need those.
 */
#pragma once

#include "vortex_bench_safety.hpp"
#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp"
#include "esp_log.h"

#include <cmath>
#include <cstdint>

namespace vortex_motor_bench {

namespace tmcl = tmc9660::tmcl;

// ---------------------------------------------------------------------------
// Per-step helpers (each returns true on success; logs on failure with `tag`).
// ---------------------------------------------------------------------------

template <typename Comm>
inline bool set_system_off(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::SYSTEM_OFF)) {
        ESP_LOGE(tag, "setCommutationMode(SYSTEM_OFF) failed");
        return false;
    }
    return true;
}

template <typename Comm>
inline bool configure_gate_driver(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    typename tmc9660::TMC9660<Comm>::GateDriver::PowerStageProfile p{};
    p.mosfet_RdsOn_mOhm     = vortex_bench_safety::kMosfetRdsOnMilliOhm;
    p.mosfet_gateCharge_nC  = vortex_bench_safety::kMosfetGateChargeNc;
    p.shuntResistance_mOhm  = vortex_bench_safety::kShuntResistanceMilliOhm;
    p.busVoltage_V          = vortex_bench_safety::kNominalSupplyVolts;
    p.pwmFrequency_Hz       = static_cast<float>(vortex_bench_safety::kPwmFrequencyHz);
    p.expectedPeakCurrent_A = vortex_bench_safety::kExpectedPeakCurrentA;
    p.motorInductance_uH    = vortex_bench_safety::kMotorPhaseInductanceUh;
    p.overcurrentMargin     = 1.5f;
    p.blankingMargin        = 1.2f;
    p.pwmLowPolarity        = tmcl::PwmOutputPolarity::ACTIVE_HIGH;
    p.pwmHighPolarity       = tmcl::PwmOutputPolarity::ACTIVE_HIGH;
    p.supplyLevel           = tmcl::UndervoltageLevel::LEVEL_10;
    p.retryBehaviour        = tmcl::GdrvRetryBehaviour::OPEN_CIRCUIT;
    p.faultBehaviour        = tmcl::DriveFaultBehaviour::OPEN_CIRCUIT;
    p.faultHandlerRetries   = 5;

    if (!d.gateDriver.configurePowerStageProtection(p)) {
        ESP_LOGE(tag, "gateDriver.configurePowerStageProtection failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Gate driver: Rds=%.1fmΩ, Qg=%.0fnC, Vbus=%.0fV, fPWM=%.0fHz, Ipk=%.1fA",
             static_cast<double>(p.mosfet_RdsOn_mOhm),
             static_cast<double>(p.mosfet_gateCharge_nC),
             static_cast<double>(p.busVoltage_V),
             static_cast<double>(p.pwmFrequency_Hz),
             static_cast<double>(p.expectedPeakCurrent_A));
    return true;
}

template <typename Comm>
inline bool configure_current_sensing(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    typename tmc9660::TMC9660<Comm>::CurrentSensing::AutoConfig c{};
    c.shuntResistance_mOhm  = vortex_bench_safety::kShuntResistanceMilliOhm;
    c.expectedPeakCurrent_A = vortex_bench_safety::kExpectedPeakCurrentA;
    c.motorType             = tmcl::MotorType::BLDC_MOTOR;
    c.usePeakScaling        = true;
    c.shuntType             = tmcl::AdcShuntType::BOTTOM_SHUNTS;
    c.csaFilter             = tmcl::CsaFilter::T_1_0_MICROSEC;
    // Default ADC mapping (U→I0, V→I1, W→I2, Y2→I3) and motor-type-driven inversion.
    c.autoCalibrate         = false;  // we run calibrateOffsets() explicitly later
    if (!d.currentSensing.configureAuto(c)) {
        ESP_LOGE(tag, "currentSensing.configureAuto failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Current sensing: shunt=%.2fmΩ, peak=%.2fA, BOTTOM_SHUNTS, 1µs filter",
             static_cast<double>(c.shuntResistance_mOhm),
             static_cast<double>(c.expectedPeakCurrent_A));
    return true;
}

template <typename Comm>
inline bool configure_motor(tmc9660::TMC9660<Comm>& d, const char* tag,
                            uint8_t pole_pairs = vortex_bench_safety::kDefaultPolePairs,
                            uint32_t pwm_hz = vortex_bench_safety::kPwmFrequencyHz) noexcept {
    typename tmc9660::TMC9660<Comm>::MotorConfig::MotorProfile mp{};
    mp.motorType         = tmcl::MotorType::BLDC_MOTOR;
    mp.polePairs         = pole_pairs;
    mp.pwmFrequency_Hz   = pwm_hz;
    mp.maxPhaseCurrent_A = vortex_bench_safety::kExpectedPeakCurrentA;
    mp.direction         = tmcl::MotorDirection::FORWARD;
    mp.pwmSwitchingScheme = tmcl::PwmSwitchingScheme::SVPWM;
    mp.maxFluxCurrent_A   = vortex_bench_safety::kMaxFluxCurrentMa / 1000.0f;
    mp.outputVoltageLimit = vortex_bench_safety::kOutputVoltageLimit;
    mp.idlePwmBehavior    = tmcl::IdleMotorPwmBehavior::PWM_OFF_WHEN_MOTOR_IDLE;

    if (!d.motorConfig.configureAuto(mp)) {
        ESP_LOGE(tag, "motorConfig.configureAuto failed");
        return false;
    }
    // configureAuto does not touch MAX_TORQUE explicitly — set the bench cap here so
    // any later torque/velocity command is hard-clamped in mA units after current scaling.
    if (!d.motorConfig.setMaxTorqueCurrent(vortex_bench_safety::kMaxPhaseCurrentMa)) {
        ESP_LOGW(tag, "setMaxTorqueCurrent(%u mA) failed", vortex_bench_safety::kMaxPhaseCurrentMa);
    }
    ESP_LOGI(tag, "  ✓ Motor: BLDC, pole_pairs=%u, fPWM=%lu Hz, max_torque_clamp=%u mA",
             static_cast<unsigned>(pole_pairs),
             static_cast<unsigned long>(pwm_hz),
             static_cast<unsigned>(vortex_bench_safety::kMaxPhaseCurrentMa));
    return true;
}

template <typename Comm>
inline bool configure_torque_flux_loop(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    typename tmc9660::TMC9660<Comm>::TorqueFluxControl::TorqueFluxConfig tf{};
    tf.torqueP = 50;
    tf.torqueI = 100;
    tf.torqueOffset_mA = 0;
    tf.fluxOffset_mA = 0;
    if (!d.torqueFluxControl.configureAuto(tf)) {
        ESP_LOGE(tag, "torqueFluxControl.configureAuto failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Torque/flux PI: P=50, I=100 (combined loops, no field weakening)");
    return true;
}

template <typename Comm>
inline bool configure_velocity_loop(tmc9660::TMC9660<Comm>& d, const char* tag,
                                    tmcl::VelocitySensorSelection sensor =
                                        tmcl::VelocitySensorSelection::SAME_AS_COMMUTATION) noexcept {
    typename tmc9660::TMC9660<Comm>::VelocityControl::VelocityConfig vc{};
    vc.sensorSelection      = sensor;
    vc.velocityP            = 1000;
    vc.velocityI            = 2;
    vc.velocityScalingFactor = 1;
    vc.velocityOffset       = 0;
    if (!d.velocityControl.configureAuto(vc)) {
        ESP_LOGE(tag, "velocityControl.configureAuto failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Velocity PI: P=1000, I=2, sensor=%d (1=SAME_AS_COMMUTATION)",
             static_cast<int>(sensor));
    return true;
}

template <typename Comm>
inline bool configure_protection(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    typename tmc9660::TMC9660<Comm>::Protection::ProtectionConfig pc{};
    pc.overvoltageThreshold_V    = vortex_bench_safety::kOvervoltageThresholdV;
    pc.undervoltageThreshold_V   = vortex_bench_safety::kUndervoltageThresholdV;
    pc.temperatureWarning_C      = vortex_bench_safety::kChipWarningTempC;
    pc.temperatureShutdown_C     = vortex_bench_safety::kChipShutdownTempC;
    pc.enableOvercurrent         = true;
    pc.i2tTimeConstant1_ms       = vortex_bench_safety::kI2tWindow1Ms;
    pc.i2tContinuousCurrent1_A   = vortex_bench_safety::kI2tCurrent1A;
    pc.i2tTimeConstant2_ms       = vortex_bench_safety::kI2tWindow2Ms;
    pc.i2tContinuousCurrent2_A   = vortex_bench_safety::kI2tCurrent2A;
    if (!d.protection.configureAuto(pc)) {
        ESP_LOGE(tag, "protection.configureAuto failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Protection: OV=%.1fV, UV=%.1fV, T_warn=%.0f°C, T_shut=%.0f°C, OC=on",
             static_cast<double>(*pc.overvoltageThreshold_V),
             static_cast<double>(*pc.undervoltageThreshold_V),
             static_cast<double>(*pc.temperatureWarning_C),
             static_cast<double>(*pc.temperatureShutdown_C));
    ESP_LOGI(tag, "    I²t W1=%ums@%.2fA, W2=%ums@%.2fA",
             static_cast<unsigned>(*pc.i2tTimeConstant1_ms),
             static_cast<double>(*pc.i2tContinuousCurrent1_A),
             static_cast<unsigned>(*pc.i2tTimeConstant2_ms),
             static_cast<double>(*pc.i2tContinuousCurrent2_A));
    return true;
}

template <typename Comm>
inline bool configure_ramp_passthrough(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    // Direct velocity mode (default) — ramp generator OFF; setTargetVelocity is applied
    // immediately to the velocity controller (or open-loop angle if no velocity loop).
    typename tmc9660::TMC9660<Comm>::Ramp::RampConfig rc{};
    rc.enableRamp = false;
    rc.enableDirectVelocityMode = true;
    if (!d.ramp.configureAuto(rc)) {
        ESP_LOGE(tag, "ramp.configureAuto failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Ramp: direct velocity mode ON, ramp generator OFF");
    return true;
}

template <typename Comm>
inline bool configure_brake_disabled(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    typename tmc9660::TMC9660<Comm>::Brake::BrakeConfig bc{};
    // Defaults: chopper off, mech brake off. Bootloader on Vortex does not enable a
    // mech brake output today; if your board adds one, configure here per schematic.
    if (!d.brake.configureAuto(bc)) {
        ESP_LOGE(tag, "brake.configureAuto failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Brake: chopper OFF, mech brake OFF (defaults)");
    return true;
}

template <typename Comm>
inline bool configure_stop_events_disabled(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    typename tmc9660::TMC9660<Comm>::StopEvents::StopEventsConfig sc{};
    if (!d.stopEvents.configureAuto(sc)) {
        ESP_LOGW(tag, "stopEvents.configureAuto failed (non-fatal)");
        return false;
    }
    return true;
}

template <typename Comm>
inline bool configure_hall(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    typename tmc9660::TMC9660<Comm>::FeedbackSense::HallConfig hc{};
    hc.sectorOffset = tmcl::HallSectorOffset::DEG_0;
    hc.direction = tmcl::Direction::NOT_INVERTED;
    hc.extrapolation = tmcl::EnableDisable::DISABLED;
    hc.filterLength = 0;
    hc.offset0Deg = 0.0f;   hc.offset60Deg  = 60.0f;
    hc.offset120Deg = 120.0f; hc.offset180Deg = 180.0f;
    hc.offset240Deg = 240.0f; hc.offset300Deg = 300.0f;
    hc.globalOffsetDeg = 0.0f;
    if (!d.feedbackSense.configureAuto(hc)) {
        ESP_LOGE(tag, "feedbackSense.configureAuto(Hall) failed (bootloader hall pins enabled?)");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Hall: DEG_0 sector, ideal 60° spacing, no filter/extrapolation");
    return true;
}

template <typename Comm>
inline bool configure_abn(tmc9660::TMC9660<Comm>& d, const char* tag,
                          uint32_t counts_per_rev =
                              vortex_bench_safety::kAbnEncoderCountsPerRev) noexcept {
    typename tmc9660::TMC9660<Comm>::FeedbackSense::AbnConfig ac{};
    ac.countsPerRev = counts_per_rev;
    ac.direction = tmcl::Direction::NOT_INVERTED;
    ac.nChannelInverted = tmcl::EnableDisable::DISABLED;
    ac.initMethod = tmcl::AbnInitMethod::FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING;
    ac.initDelay = 1000;
    ac.initVelocity = 5;
    ac.nChannelOffset = 0;
    ac.nChannelFiltering = tmcl::AbnNChannelFiltering::FILTERING_OFF;
    ac.clearOnNextNull = tmcl::EnableDisable::DISABLED;
    if (!d.feedbackSense.configureAuto(ac)) {
        ESP_LOGE(tag, "feedbackSense.configureAuto(ABN) failed (bootloader ABN pins enabled?)");
        return false;
    }
    ESP_LOGI(tag, "  ✓ ABN: %lu CPR, FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING init, no N-filter",
             static_cast<unsigned long>(counts_per_rev));
    return true;
}

template <typename Comm>
inline bool calibrate_current_offsets(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    if (!d.currentSensing.calibrateOffsets(true, 2000)) {
        ESP_LOGW(tag, "currentSensing.calibrateOffsets failed/timeout (continuing)");
        return false;
    }
    ESP_LOGI(tag, "  ✓ ADC offset calibration done");
    return true;
}

template <typename Comm>
inline bool enable_drv_en(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    if (!d.GpioSetActive(tmc9660::TMC9660CtrlPin::DRV_EN)) {
        ESP_LOGE(tag, "GpioSetActive(DRV_EN) failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ DRV_EN active — gate driver outputs are LIVE");
    return true;
}

template <typename Comm>
inline void disable_drv_en(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    if (!d.GpioSetInactive(tmc9660::TMC9660CtrlPin::DRV_EN)) {
        ESP_LOGW(tag, "GpioSetInactive(DRV_EN) returned false");
    } else {
        ESP_LOGI(tag, "  ✓ DRV_EN inactive — gate driver outputs disabled");
    }
}

template <typename Comm>
inline void motor_stop_safe(tmc9660::TMC9660<Comm>& d, const char* tag = "vortex_motor") noexcept {
    d.torqueFluxControl.stop();
    if (!d.motorConfig.setCommutationMode(tmcl::CommutationMode::SYSTEM_OFF)) {
        ESP_LOGW(tag, "setCommutationMode(SYSTEM_OFF) failed during stop");
    }
    disable_drv_en(d, tag);
}

// ---------------------------------------------------------------------------
// Telemetry / diagnostics
// ---------------------------------------------------------------------------

template <typename Comm>
inline void log_telemetry(tmc9660::TMC9660<Comm>& d, const char* tag, const char* label = "telemetry") noexcept {
    const float v = d.telemetry.getSupplyVoltage();
    const float t = d.telemetry.getChipTemperature();
    const int32_t vel = d.telemetry.getActualVelocity();
    const int32_t pos = d.telemetry.getActualPosition();
    const int16_t i = d.telemetry.getMotorCurrent();
    int16_t iq = 0;
    (void)d.torqueFluxControl.getFocCurrentIq(iq);
    ESP_LOGI(tag, "[%s] Vbus=%.2fV T_chip=%.1f°C vel=%ld pos=%ld I_motor=%d mA Iq=%d mA",
             label, static_cast<double>(v), static_cast<double>(t),
             static_cast<long>(vel), static_cast<long>(pos),
             static_cast<int>(i), static_cast<int>(iq));
}

template <typename Comm>
inline void log_fault_flags(tmc9660::TMC9660<Comm>& d, const char* tag, const char* label = "faults") noexcept {
    uint32_t gen_status = 0, gen_err = 0, gd_err = 0, adc_status = 0;
    (void)d.telemetry.getGeneralStatusFlags(gen_status);
    (void)d.telemetry.getGeneralErrorFlags(gen_err);
    (void)d.telemetry.getGateDriverErrorFlags(gd_err);
    (void)d.telemetry.getADCStatusFlags(adc_status);
    ESP_LOGI(tag, "[%s] gen_status=0x%08X gen_err=0x%08X gd_err=0x%08X adc_status=0x%08X",
             label,
             static_cast<unsigned>(gen_status),
             static_cast<unsigned>(gen_err),
             static_cast<unsigned>(gd_err),
             static_cast<unsigned>(adc_status));
}

template <typename Comm>
inline void clear_fault_flags(tmc9660::TMC9660<Comm>& d) noexcept {
    (void)d.telemetry.clearGeneralErrorFlags(0xFFFFFFFFu);
    (void)d.telemetry.clearGateDriverErrorFlags(0xFFFFFFFFu);
    (void)d.telemetry.clearADCStatusFlags(0xFFFFFFFFu);
}

// ---------------------------------------------------------------------------
// One-call full bring-up
// ---------------------------------------------------------------------------

/**
 * @brief Configure the gate driver, current sensing, motor parameters, FOC PI
 *        gains, protections, ramp, brake, stop events, calibrate ADC offsets,
 *        and assert DRV_EN. After this returns true, the chip is **armed in
 *        SYSTEM_OFF**; caller picks the commutation mode and target.
 *
 * @param tag         ESP_LOG tag.
 * @param pole_pairs  Motor pole pair count.
 * @param pwm_hz      Switching frequency.
 * @param do_calibrate Run ADC offset calibration before DRV_EN (recommended).
 * @return true if every required step succeeded.
 */
template <typename Comm>
inline bool configure_complete_bldc(tmc9660::TMC9660<Comm>& d,
                                    const char* tag,
                                    uint8_t pole_pairs = vortex_bench_safety::kDefaultPolePairs,
                                    uint32_t pwm_hz = vortex_bench_safety::kPwmFrequencyHz,
                                    bool do_calibrate = true,
                                    bool enable_outputs = true) noexcept {
    ESP_LOGI(tag, "==== Vortex BLDC bring-up start (24V class, %u pole pairs, %lu Hz PWM) ====",
             static_cast<unsigned>(pole_pairs), static_cast<unsigned long>(pwm_hz));
    if (!set_system_off(d, tag))                             return false;
    if (!configure_gate_driver(d, tag))                      return false;
    if (!configure_current_sensing(d, tag))                  return false;
    if (!configure_motor(d, tag, pole_pairs, pwm_hz))        return false;
    if (!configure_torque_flux_loop(d, tag))                 return false;
    if (!configure_velocity_loop(d, tag))                    return false;
    if (!configure_protection(d, tag))                       return false;
    if (!configure_ramp_passthrough(d, tag))                 return false;
    (void)configure_brake_disabled(d, tag);                  // non-fatal
    (void)configure_stop_events_disabled(d, tag);            // non-fatal
    if (do_calibrate) (void)calibrate_current_offsets(d, tag);
    if (enable_outputs) {
        if (!enable_drv_en(d, tag)) return false;
    }
    log_telemetry(d, tag, "ready");
    log_fault_flags(d, tag, "ready");
    ESP_LOGI(tag, "==== Vortex BLDC bring-up complete (chip armed, SYSTEM_OFF) ====");
    return true;
}

}  // namespace vortex_motor_bench
