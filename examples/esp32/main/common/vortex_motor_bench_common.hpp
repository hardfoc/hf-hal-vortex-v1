/**
 * @file vortex_motor_bench_common.hpp
 * @brief Shared TMC9660 BLDC bring-up helpers for Vortex parameter-mode bench apps.
 *
 * Wraps the TMC9660 driver's `configureAuto(...)` surfaces (motor, gate driver,
 * current sensing, FOC, protection, ramp, brake, stop events, heartbeat, power)
 * with values keyed off
 * `vortex_bench_safety.hpp`. Step ordering mirrors the EvKit
 * `bldc_comprehensive_test.cpp::configureCompleteBLDCMotor()`:
 *
 *   1. SYSTEM_OFF
 *   2. Motor parameters (BLDC type + pole pairs first — required before gate-driver SAPs that
 *      touch the Y2 phase on 3-phase builds)
 *   3. Gate driver (must precede final DRV_EN for switching setup)
 *   4. Current sensing
 *   5. FOC torque/flux + velocity loop gains
 *   6. Protection (OV/UV, OT, OC, I²t)
 *   7. Ramp (direct velocity mode by default; ramp gen disabled)
 *   8. Brake (disabled by default — enable per board)
 *   9. Stop events (defaults; non-fatal if configureAuto fails)
 *  10. Heartbeat (defaults / watchdog off — matches EvKit `configureHeartbeatForBLDC`)
 *  11. Power management (defaults — matches EvKit `configurePowerForBLDC`)
 *  12. Current sense ADC offset calibration (motor stationary, SYSTEM_OFF)
 *  13. DRV_EN → ACTIVE
 *
 * After step 13 the chip is **armed but still in `SYSTEM_OFF`**. The caller
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
 *
 * Narrative: `examples/esp32/docs/BLDC_BRINGUP.md`.
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

/**
 * @brief Configure the TMC9660 power-stage / gate-driver protection block.
 *
 * Three independently togglable groups (chip-side acceptance varies by
 * variant / firmware revision):
 *
 *   - **Y2 phase SAPs** (`!skip_y2`): the Vortex 3-phase BLDC stage does not
 *     wire Y2 — leave skipped.
 *   - **UVW gate current limits** (NR 245 `UVW_SINK_CURRENT` and NR 246
 *     `UVW_SOURCE_CURRENT`): the silicon revision on Vortex rejects the
 *     enum codes the auto-config picks for our 58 nC FETs (REPLY_INVALID_VALUE
 *     for both 245/246, regardless of which enum index we send). The chip
 *     keeps reset defaults instead, which still produce safe switching since
 *     `DRIVE_TIME_*_UVW` (241/242) and `USE_ADAPTIVE_DRIVE_TIME_UVW` (239)
 *     accept normally — leave these **off** until silicon support is
 *     verified.
 *   - **OC + VGS short protection block** (NRs 254/255 enables, 258/259
 *     thresholds, 262/263 blanking, 266/267 deglitch, 270 use_VDS, 272–275
 *     VGS short enables, 280 VGS blanking, 282 VGS deglitch, 286/287/288
 *     fault-handling): all accept on UART once the
 *     `TMCLFrame::toUart` 12-bit type packing is correct (see
 *     `TMCL_PARAM_SCAN_FAILURES_UART.md`). Leave **on** for any real motor
 *     spin so a hardware short trips the gate driver and not the FETs.
 */
template <typename Comm>
inline bool configure_gate_driver(tmc9660::TMC9660<Comm>& d, const char* tag,
                                  bool with_oc_vgs_protection = false,
                                  bool with_gate_current_limits = false,
                                  bool skip_y2 = true) noexcept {
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
    p.configure_y2_phase                 = !skip_y2;
    p.program_gate_current_limits        = with_gate_current_limits;
    p.configure_gate_oc_vgs_protection   = with_oc_vgs_protection;

    if (!d.gateDriver.configurePowerStageProtection(p)) {
        ESP_LOGE(tag, "gateDriver.configurePowerStageProtection failed");
        return false;
    }
    ESP_LOGI(tag,
             "  ✓ Gate driver: Rds=%.1fmΩ, Qg=%.0fnC, Vbus=%.0fV, fPWM=%.0fHz, Ipk=%.1fA "
             "[Y2:%s, gate_I_limits:%s, OC_Vgs:%s]",
             static_cast<double>(p.mosfet_RdsOn_mOhm),
             static_cast<double>(p.mosfet_gateCharge_nC),
             static_cast<double>(p.busVoltage_V),
             static_cast<double>(p.pwmFrequency_Hz),
             static_cast<double>(p.expectedPeakCurrent_A),
             p.configure_y2_phase ? "on" : "skip",
             p.program_gate_current_limits ? "on" : "off",
             p.configure_gate_oc_vgs_protection ? "on" : "off");
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
    ESP_LOGI(tag, "  ✓ Velocity PI: P=1000, I=2, sensorSelection=%d",
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
    // All `program_*` flags default to true in `ProtectionConfig`: supply OV/UV thresholds,
    // chip temperature warn/shutdown SAPs, gate-driver OC enables, and I²t limits are applied
    // via SAP only (see `Protection::configureAuto` / `configureVoltage` in hf-tmc9660-driver).
    // GAP/GGP reads of live SUPPLY_VOLTAGE / CHIP_TEMPERATURE are separate and may still fail
    // on some ROMs; threshold programming here does not use those reads.
    if (!d.protection.configureAuto(pc)) {
        ESP_LOGE(tag, "protection.configureAuto failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Protection: OV=%.1fV, UV=%.1fV, T_warn=%.0f°C, T_shut=%.0f°C",
             static_cast<double>(*pc.overvoltageThreshold_V),
             static_cast<double>(*pc.undervoltageThreshold_V),
             static_cast<double>(*pc.temperatureWarning_C),
             static_cast<double>(*pc.temperatureShutdown_C));
    ESP_LOGI(tag, "    SAP blocks: supply_warn, chip_temp_warn, gate_OC_enable, I²t — "
                  "OC enable falls back to UVW-only if Y2 SAPs are unsupported");
    ESP_LOGI(tag, "    OC=on, I²t W1=%ums@%.2fA, W2=%ums@%.2fA",
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

/** Ramp generator ON, **DIRECT_VELOCITY_MODE off** — used for BLDC open-loop
 *  **current** mode so the hardware ramper actually advances phi_e from the
 *  written `TARGET_VELOCITY` rate (TMCL commutation docs).
 *
 *  In direct-velocity mode the ramper is bypassed and `TARGET_VELOCITY` lands
 *  on the velocity-PI input directly; in open-loop modes that breaks the
 *  ramper-driven phi_e and the chip rejects the SAP with REPLY_INVALID_VALUE
 *  for any value that the un-ramped pipeline cannot serve.
 */
template <typename Comm>
inline bool configure_ramp_for_open_loop_voltage(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    typename tmc9660::TMC9660<Comm>::Ramp::RampConfig rc{};
    rc.maxVelocity              = vortex_bench_safety::kOpenLoopRampMaxVelocity;
    rc.enableRamp               = true;
    rc.enableDirectVelocityMode = false;
    if (!d.ramp.configureAuto(rc)) {
        ESP_LOGE(tag, "ramp.configureAuto (open-loop spin) failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Ramp: generator ON, direct velocity OFF, max_vel=%lu (ramper drives phi_e)",
             static_cast<unsigned long>(rc.maxVelocity));
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
inline bool configure_heartbeat_defaults(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    typename tmc9660::TMC9660<Comm>::Heartbeat::HeartbeatConfig hc{};
    if (!d.heartbeat.configureAuto(hc)) {
        ESP_LOGE(tag, "heartbeat.configureAuto failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Heartbeat: defaults (watchdog off unless you enable in config)");
    return true;
}

template <typename Comm>
inline bool configure_power_defaults(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    typename tmc9660::TMC9660<Comm>::Power::PowerConfig pc{};
    if (!d.power.configureAuto(pc)) {
        ESP_LOGE(tag, "power.configureAuto failed");
        return false;
    }
    ESP_LOGI(tag, "  ✓ Power: defaults (wake / power-down timeout off)");
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

/**
 * @brief Extra telemetry slice useful while spinning open-loop: live electrical
 *        angle from the ramper (OPENLOOP_ANGLE, NR 45) plus the per-phase FOC
 *        currents and `Uq` to confirm the inverter is actually modulating.
 *
 * @details Does **not** subsume `log_telemetry`; call both for a full picture.
 *          All reads are best-effort (read-only params, may return WRONG_TYPE
 *          on some ROMs in some commutation modes — logged but not fatal).
 */
template <typename Comm>
inline void log_open_loop_telemetry(tmc9660::TMC9660<Comm>& d, const char* tag,
                                     const char* label = "openloop") noexcept {
    int16_t phi_e = 0;
    int16_t i_ux = 0, i_v = 0, i_wy = 0, i_q = 0;
    int16_t u_q = 0, u_ux = 0, u_v = 0, u_wy = 0;
    (void)d.torqueFluxControl.getOpenloopAngle(phi_e);
    (void)d.torqueFluxControl.getFocCurrentUx(i_ux);
    (void)d.torqueFluxControl.getFocCurrentV(i_v);
    (void)d.torqueFluxControl.getFocCurrentWy(i_wy);
    (void)d.torqueFluxControl.getFocCurrentIq(i_q);
    (void)d.torqueFluxControl.getFocVoltageUq(u_q);
    (void)d.torqueFluxControl.getFocVoltageUx(u_ux);
    (void)d.torqueFluxControl.getFocVoltageV(u_v);
    (void)d.torqueFluxControl.getFocVoltageWy(u_wy);
    ESP_LOGI(tag,
             "[%s] phi_e=%6d  Iuvw=(%5d,%5d,%5d) Iq=%5d  Uuvw=(%5d,%5d,%5d) Uq=%5d",
             label, static_cast<int>(phi_e),
             static_cast<int>(i_ux), static_cast<int>(i_v), static_cast<int>(i_wy),
             static_cast<int>(i_q),
             static_cast<int>(u_ux), static_cast<int>(u_v), static_cast<int>(u_wy),
             static_cast<int>(u_q));
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

/**
 * @brief Probe SAP/GAP for UVW + Y2 gate current limits (NR 245/246/247/248).
 *
 * Used by **`vortex_bldc_telemetry_sweep`** (flip the constexpr there). Helps separate:
 *   - **Transport / endian**: TMCL value is **32-bit big-endian on the wire** for both UART and
 *     SPI (Trinamic `TMC9660.c::sendRequestUART` bytes 4–7; `TMCLFrame::toSpi` / `toUart` in
 *     `hf-tmc9660-driver`). The probes below try `enum` (correct), `0x07000000` (mis-byte-swapped),
 *     `0x00000700` (mis-shift) — **only `enum` should be accepted** if framing matches the manual.
 *   - **Chip policy**: `REPLY_INVALID_VALUE` with enum `0…15` in the low byte means **the opcode
 *     and axis parameter index are accepted** but the **value** is rejected. The
 *     `enum 0..15` sweep tells us *which specific enums* the silicon/FW will take — the datasheet
 *     defaults (NR 245→4, NR 246→4) **must** be accepted on a healthy chip.
 *   - **Order / state dependency**: pass a `phase` label so the same probe can run **before**
 *     `configure_complete_bldc` (chip near reset) and **after** (motor type=BLDC, all other gate
 *     driver SAPs already programmed). If results differ, gate-current acceptance is
 *     state-gated by another SAP we wrote in between.
 *
 * Logs the bench Qg and the same rough mA targets as `configurePowerStageProtection`
 * (200 ns / 135 ns edge targets) so you can correlate with the enum sweep.
 */
template <typename Comm>
inline void diagnose_uvw_gate_current_tmcl(tmc9660::TMC9660<Comm>& d, const char* tag,
                                           const char* phase = "post-config") noexcept {
    namespace tmcl = tmc9660::tmcl;

    const float qg_nc = vortex_bench_safety::kMosfetGateChargeNc;
    constexpr float kTargetTurnOn_ns = 200.0f;
    constexpr float kTargetTurnOff_ns = 135.0f;
    const float req_src_ma = (qg_nc / kTargetTurnOn_ns) * 1000.0f;
    const float req_sink_ma = (qg_nc / kTargetTurnOff_ns) * 1000.0f;
    ESP_LOGI(tag, "[gate-diag/%s] -------- BEGIN --------", phase);
    ESP_LOGI(tag,
             "[gate-diag/%s] Bench Qg=%.1f nC @ %.0f/%.0f ns edge targets → ~%.0f mA source / ~%.0f mA sink",
             phase, static_cast<double>(qg_nc), static_cast<double>(kTargetTurnOn_ns),
             static_cast<double>(kTargetTurnOff_ns), static_cast<double>(req_src_ma),
             static_cast<double>(req_sink_ma));

    auto gap_log = [&](const char* name, tmcl::Parameters p) -> uint32_t {
        uint32_t v = 0;
        const bool ok = d.readParameter(p, v, 0);
        ESP_LOGI(tag, "[gate-diag/%s] GAP NR%u %-20s %s raw=0x%08lX",
                 phase, static_cast<unsigned>(p), name, ok ? "OK" : "GAP_FAIL",
                 static_cast<unsigned long>(v));
        return v;
    };

    const uint32_t initial_245 = gap_log("UVW_SINK_CURRENT", tmcl::Parameters::UVW_SINK_CURRENT);
    const uint32_t initial_246 = gap_log("UVW_SOURCE_CURRENT", tmcl::Parameters::UVW_SOURCE_CURRENT);
    (void)gap_log("Y2_SINK_CURRENT",  tmcl::Parameters::Y2_SINK_CURRENT);
    (void)gap_log("Y2_SOURCE_CURRENT", tmcl::Parameters::Y2_SOURCE_CURRENT);

    auto log_sap = [&](const char* label_sap, tmcl::Parameters p, uint32_t val) {
        tmcl::ReplyCode st{};
        const bool xfer_ok = d.sendCommand(tmcl::Op::SAP, static_cast<uint16_t>(p), 0u, val, nullptr, &st);
        const unsigned st_u = static_cast<unsigned>(static_cast<uint8_t>(st));
        ESP_LOGI(tag,
                 "[gate-diag/%s] SAP %s NR%u value=0x%08lX → xfer_ok=%d tmcl=%s (0x%02X)",
                 phase, label_sap, static_cast<unsigned>(p), static_cast<unsigned long>(val),
                 xfer_ok ? 1 : 0, tmcl::to_string(st), st_u);
    };

    ESP_LOGI(tag, "[gate-diag/%s] CONTROL TEST: SAP NR241 DRIVE_TIME_SINK_UVW (must succeed)", phase);
    {
        uint32_t before = 0, after = 0;
        (void)d.readParameter(tmcl::Parameters::DRIVE_TIME_SINK_UVW, before, 0);
        log_sap("DRIVE_TIME_SINK_UVW =100", tmcl::Parameters::DRIVE_TIME_SINK_UVW, 100u);
        (void)d.readParameter(tmcl::Parameters::DRIVE_TIME_SINK_UVW, after, 0);
        ESP_LOGI(tag, "[gate-diag/%s]   NR241 before=0x%08lX after=0x%08lX (expect after=100)", phase,
                 static_cast<unsigned long>(before), static_cast<unsigned long>(after));
        log_sap("DRIVE_TIME_SINK_UVW restore", tmcl::Parameters::DRIVE_TIME_SINK_UVW, before);
    }

    ESP_LOGI(tag, "[gate-diag/%s] WRITE-BACK-ECHO: re-write the value the chip just reported", phase);
    log_sap("UVW_SINK_CURRENT  echo",   tmcl::Parameters::UVW_SINK_CURRENT,   initial_245);
    log_sap("UVW_SOURCE_CURRENT echo", tmcl::Parameters::UVW_SOURCE_CURRENT, initial_246);

    ESP_LOGI(tag, "[gate-diag/%s] DATASHEET DEFAULTS: NR245=4 (CUR_270_MA), NR246=4 (CUR_135_MA)", phase);
    log_sap("UVW_SINK_CURRENT  default(4)", tmcl::Parameters::UVW_SINK_CURRENT,   4u);
    log_sap("UVW_SOURCE_CURRENT default(4)", tmcl::Parameters::UVW_SOURCE_CURRENT, 4u);

    ESP_LOGI(tag, "[gate-diag/%s] ENDIAN PROBE: low-byte enum 7 vs 0x07000000 vs 0x00000700", phase);
    log_sap("UVW_SINK_CURRENT  enum 7", tmcl::Parameters::UVW_SINK_CURRENT, 7u);
    log_sap("UVW_SINK_CURRENT  byte-swap 0x07000000", tmcl::Parameters::UVW_SINK_CURRENT, 0x07000000u);
    log_sap("UVW_SINK_CURRENT  shifted   0x00000700", tmcl::Parameters::UVW_SINK_CURRENT, 0x00000700u);

    ESP_LOGI(tag, "[gate-diag/%s] PER-PHASE PACKED CANDIDATES (in case fw expects U|V|W nibbles)",
             phase);
    log_sap("UVW_SINK_CURRENT 0x00000444",       tmcl::Parameters::UVW_SINK_CURRENT, 0x00000444u);
    log_sap("UVW_SINK_CURRENT 0x00444444",       tmcl::Parameters::UVW_SINK_CURRENT, 0x00444444u);
    log_sap("UVW_SINK_CURRENT 0x44444444",       tmcl::Parameters::UVW_SINK_CURRENT, 0x44444444u);
    log_sap("UVW_SINK_CURRENT 0x80000004 (sign)",tmcl::Parameters::UVW_SINK_CURRENT, 0x80000004u);

    ESP_LOGI(tag, "[gate-diag/%s] GAP NR245 right after packed-pattern attempts:", phase);
    (void)gap_log("UVW_SINK_CURRENT post-packed", tmcl::Parameters::UVW_SINK_CURRENT);

    // Bisect the high-nibble-of-LSByte rule: the prior probe showed that 0x444 etc are accepted.
    // We hypothesise the FW051V100 firmware is checking that **bits[7:4] of byte 7 (LSByte of the
    // 32-bit value)** are non-zero, and stores only `value & 0xF`. Confirm by sweeping the upper
    // nibble of the LSByte while varying the lower nibble, and GAP-reading after each.
    auto sap_then_gap = [&](const char* lbl, uint32_t v) {
        tmcl::ReplyCode st{};
        const bool xfer_ok = d.sendCommand(tmcl::Op::SAP,
                                           static_cast<uint16_t>(tmcl::Parameters::UVW_SINK_CURRENT),
                                           0u, v, nullptr, &st);
        uint32_t r = 0;
        (void)d.readParameter(tmcl::Parameters::UVW_SINK_CURRENT, r, 0);
        ESP_LOGI(tag, "[gate-diag/%s]   SAP 0x%08lX %-20s → tmcl=%s (xfer=%d), GAP=0x%08lX", phase,
                 static_cast<unsigned long>(v), lbl, tmcl::to_string(st), xfer_ok ? 1 : 0,
                 static_cast<unsigned long>(r));
    };

    ESP_LOGI(tag, "[gate-diag/%s] HIGH-NIBBLE-LOCK PROBE (vary bits[7:4] with enum=0)", phase);
    sap_then_gap("hi=0 lo=0", 0x00u);
    sap_then_gap("hi=1 lo=0", 0x10u);
    sap_then_gap("hi=2 lo=0", 0x20u);
    sap_then_gap("hi=4 lo=0", 0x40u);
    sap_then_gap("hi=8 lo=0", 0x80u);
    sap_then_gap("hi=F lo=0", 0xF0u);

    ESP_LOGI(tag, "[gate-diag/%s] HIGH-NIBBLE-LOCK PROBE (vary lo with hi=4)", phase);
    sap_then_gap("hi=4 lo=0",  0x40u);
    sap_then_gap("hi=4 lo=4",  0x44u);
    sap_then_gap("hi=4 lo=7",  0x47u);
    sap_then_gap("hi=4 lo=A",  0x4Au);
    sap_then_gap("hi=4 lo=F",  0x4Fu);

    ESP_LOGI(tag, "[gate-diag/%s] HIGH-NIBBLE-LOCK PROBE (vary lo with hi=F)", phase);
    sap_then_gap("hi=F lo=0",  0xF0u);
    sap_then_gap("hi=F lo=8",  0xF8u);
    sap_then_gap("hi=F lo=F",  0xFFu);

    ESP_LOGI(tag, "[gate-diag/%s] FULL ENUM 0..15 with hi-nibble 0x40 OR'd in", phase);
    for (uint32_t e = 0; e <= 15; ++e) {
        char lbl[32];
        snprintf(lbl, sizeof(lbl), "0x40|%lu", static_cast<unsigned long>(e));
        sap_then_gap(lbl, 0x40u | e);
    }

    // After several probe rounds: only writes where BOTH byte 6 low nibble AND byte 7 low nibble
    // equal E produced GAP=E (e.g. 0x444 → GAP=4). All other guarded writes produced GAP=0. Try
    // `0x111 * E` to confirm — should give GAP=E for E in 0..15.
    ESP_LOGI(tag, "[gate-diag/%s] BIT-REPLICATED CANDIDATE: 0x111 * E", phase);
    for (uint32_t e = 0; e <= 15; ++e) {
        char lbl[32];
        snprintf(lbl, sizeof(lbl), "0x111*E=%lu", static_cast<unsigned long>(e));
        sap_then_gap(lbl, e * 0x111u);
    }
    // Also try (E<<8)|(E<<4)|E in two-nibble form: just (E<<4)|E (= 0x11 * E) to see if byte 7 alone matters
    ESP_LOGI(tag, "[gate-diag/%s] TWO-NIBBLE CANDIDATE: 0x11 * E (byte7 only)", phase);
    for (uint32_t e = 0; e <= 15; ++e) {
        char lbl[32];
        snprintf(lbl, sizeof(lbl), "0x11*E=%lu", static_cast<unsigned long>(e));
        sap_then_gap(lbl, e * 0x11u);
    }
    // Also try (E<<8)|0x4E (constant 4 in byte 6, enum in byte 7 low) to isolate byte 7's role
    ESP_LOGI(tag, "[gate-diag/%s] BYTE6=0x04 + BYTE7=(0x40|E)", phase);
    for (uint32_t e = 0; e <= 15; ++e) {
        char lbl[32];
        snprintf(lbl, sizeof(lbl), "0x4(0x40|E=%lu)", static_cast<unsigned long>(e));
        sap_then_gap(lbl, 0x400u | 0x40u | e);  // 0x440 + E
    }

    ESP_LOGI(tag, "[gate-diag/%s] FULL ENUM SWEEP 0..15 NR245 (sink current)", phase);
    for (uint32_t e = 0; e <= 15; ++e) {
        tmcl::ReplyCode st{};
        (void)d.sendCommand(tmcl::Op::SAP, static_cast<uint16_t>(tmcl::Parameters::UVW_SINK_CURRENT),
                            0u, e, nullptr, &st);
        ESP_LOGI(tag, "[gate-diag/%s]   NR245 enum=%2lu → %s", phase,
                 static_cast<unsigned long>(e), tmcl::to_string(st));
    }
    (void)gap_log("UVW_SINK_CURRENT post-sweep", tmcl::Parameters::UVW_SINK_CURRENT);

    ESP_LOGI(tag, "[gate-diag/%s] FULL ENUM SWEEP 0..15 NR246 (source current)", phase);
    for (uint32_t e = 0; e <= 15; ++e) {
        tmcl::ReplyCode st{};
        (void)d.sendCommand(tmcl::Op::SAP, static_cast<uint16_t>(tmcl::Parameters::UVW_SOURCE_CURRENT),
                            0u, e, nullptr, &st);
        ESP_LOGI(tag, "[gate-diag/%s]   NR246 enum=%2lu → %s", phase,
                 static_cast<unsigned long>(e), tmcl::to_string(st));
    }
    (void)gap_log("UVW_SOURCE_CURRENT post-sweep", tmcl::Parameters::UVW_SOURCE_CURRENT);

    ESP_LOGI(tag, "[gate-diag/%s] -------- END --------", phase);
}

/**
 * @brief Probe NR 245/246 with `MOTOR_TYPE = NO_MOTOR` (chip default after reset).
 *
 * EvKit's `configureCompleteBLDCMotor` programs the gate driver **before** setting
 * `MOTOR_TYPE = BLDC_MOTOR`. Our Vortex bring-up sets `MOTOR_TYPE = BLDC` first, then the gate
 * driver. If the FW051V100 firmware only allows NR 245/246 SAPs while the motor type is still
 * `NO_MOTOR`, this probe will succeed where the post-config probe fails. Run it as the FIRST
 * thing after `EnsureInitialized` (motor type still defaults to `NO_MOTOR`).
 *
 * Returns `true` if the chip accepted the datasheet default (NR 245 = 4) with `REPLY_OK`.
 */
template <typename Comm>
inline bool probe_gate_current_with_no_motor_type(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
    namespace tmcl = tmc9660::tmcl;

    uint32_t mt = 0xFFFFFFFFu;
    (void)d.readParameter(tmcl::Parameters::MOTOR_TYPE, mt, 0);
    ESP_LOGI(tag, "[gate-diag/no-motor] starting with MOTOR_TYPE=%u (0=NO_MOTOR, 3=BLDC)",
             static_cast<unsigned>(mt));

    tmcl::ReplyCode st1{}, st2{};
    const bool ok1 = d.sendCommand(tmcl::Op::SAP,
                                   static_cast<uint16_t>(tmcl::Parameters::UVW_SINK_CURRENT), 0u,
                                   4u, nullptr, &st1);
    const bool ok2 = d.sendCommand(tmcl::Op::SAP,
                                   static_cast<uint16_t>(tmcl::Parameters::UVW_SOURCE_CURRENT), 0u,
                                   4u, nullptr, &st2);
    ESP_LOGI(tag, "[gate-diag/no-motor] SAP NR245=4 → tmcl=%s xfer=%d", tmcl::to_string(st1), ok1 ? 1 : 0);
    ESP_LOGI(tag, "[gate-diag/no-motor] SAP NR246=4 → tmcl=%s xfer=%d", tmcl::to_string(st2), ok2 ? 1 : 0);
    return st1 == tmcl::ReplyCode::REPLY_OK && st2 == tmcl::ReplyCode::REPLY_OK;
}

/** Snapshot for smoke / bench apps: supply, temp, motion, faults (used by `vortex_motor_comms_smoke`). */
template <typename Comm>
inline void log_motor_comms(const char* tag, tmc9660::TMC9660<Comm>& d) noexcept {
    log_telemetry(d, tag, "comms");
    log_fault_flags(d, tag, "comms");
}

/**
 * @brief Clear latched fault/status flags using **documented write-1-to-clear**
 *        semantics (ADI Rev 0 Parameter Mode manual, Flags section).
 *
 * We **read** each aggregate first, then SAP only the **bits currently set**
 * (never `0xFFFFFFFF`), so we do not attempt to "clear" read-only bits such as
 * `CONFIG_ERROR` / `TMCL_SCRIPT_ERROR` / `HOMESWITCH_NOT_FOUND` on
 * `GENERAL_ERROR_FLAGS` (299).  Skips a register entirely if GAP read fails.
 */
template <typename Comm>
inline void clear_fault_flags(tmc9660::TMC9660<Comm>& d) noexcept {
    // GENERAL_ERROR_FLAGS: Table 59 — low bits 0x1/0x2/0x4 are R (not RWC).
    constexpr uint32_t kGenErrReadOnlyLow = 0x1u | 0x2u | 0x4u;

    uint32_t gen_err = 0, gd_err = 0, adc_st = 0;
    if (d.telemetry.getGeneralErrorFlags(gen_err)) {
        const uint32_t clear_mask = gen_err & ~kGenErrReadOnlyLow;
        if (clear_mask != 0) {
            (void)d.telemetry.clearGeneralErrorFlags(clear_mask);
        }
    }
    if (d.telemetry.getGateDriverErrorFlags(gd_err) && gd_err != 0) {
        (void)d.telemetry.clearGateDriverErrorFlags(gd_err);
    }
    if (d.telemetry.getADCStatusFlags(adc_st) && adc_st != 0) {
        (void)d.telemetry.clearADCStatusFlags(adc_st);
    }
}

/**
 * @brief `GENERAL_ERROR_FLAGS` bits that must **not** trip motion safety during
 *        open-loop bench work (read-only status, warnings, or pipeline noise).
 *
 * See `docs/BLDC_UART_BRINGUP.md` — the chip latches `HALL_ERROR` in open-loop
 * even without hall wiring; `CONFIG_ERROR` is read-only NVS state; several
 * warning bits are not exceedance faults.
 */
inline constexpr uint32_t kBenchOpenLoopIgnoredGeneralErrors =
    (1u << 0) |   // CONFIG_ERROR (R/O)
    (1u << 1) |   // TMCL_SCRIPT_ERROR (R/O)
    (1u << 2) |   // HOMESWITCH_NOT_FOUND (R/O)
    (1u << 5) |   // HALL_ERROR — open-loop does not use halls
    (1u << 18) |  // EXT_TEMP_WARNING
    (1u << 19) |  // SUPPLY_OVERVOLTAGE_WARNING
    (1u << 20) |  // SUPPLY_UNDERVOLTAGE_WARNING
    (1u << 21) |  // ADC_IN_OVERVOLTAGE — unused AINx
    (1u << 24);   // CHIP_TEMP_WARNING

template <typename Comm>
inline bool bench_open_loop_motion_fault_critical(tmc9660::TMC9660<Comm>& d) noexcept {
    uint32_t gen_err = 0, gd_err = 0;
    (void)d.telemetry.getGeneralErrorFlags(gen_err);
    (void)d.telemetry.getGateDriverErrorFlags(gd_err);
    return ((gen_err & ~kBenchOpenLoopIgnoredGeneralErrors) != 0) || (gd_err != 0);
}

/**
 * @brief Disable UVW **VGS-short** protection enables (NR 272–275) and set UVW
 *        VGS deglitch to 8 µs (NR 282). Then clear latched fault flags.
 *
 * On the Vortex 3-phase power stage the chip defaults (protection ON) trip
 * spurious `GDRV_ERROR` charge-short bits on every transition; the fault
 * handler then reverts `COMMUTATION_MODE` to `SYSTEM_OFF` and all
 * `TARGET_VELOCITY` SAPs return `REPLY_INVALID_VALUE`. **Re-tune** blanking /
 * deglitch for your FETs before re-enabling these bits in production.
 */
template <typename Comm>
inline void disable_vortex_uvw_vgs_short_protection(tmc9660::TMC9660<Comm>& d,
                                                    const char* tag) noexcept {
    namespace tmcl = tmc9660::tmcl;
    const uint32_t off = static_cast<uint32_t>(tmcl::VgsShortEnable::DISABLED);
    const uint32_t deglitch_max = static_cast<uint32_t>(tmcl::VgsDeglitchTime::T_8_MICROSEC);
    bool ok = true;
    ok &= d.writeParameter(tmcl::Parameters::UVW_LOW_SIDE_ON_ENABLE, off);
    ok &= d.writeParameter(tmcl::Parameters::UVW_LOW_SIDE_OFF_ENABLE, off);
    ok &= d.writeParameter(tmcl::Parameters::UVW_HIGH_SIDE_ON_ENABLE, off);
    ok &= d.writeParameter(tmcl::Parameters::UVW_HIGH_SIDE_OFF_ENABLE, off);
    ok &= d.writeParameter(tmcl::Parameters::UVW_DEGLITCH, deglitch_max);
    ESP_LOGI(tag, "  VGS UVW short protection: disabled + max deglitch (%s)",
             ok ? "all SAPs OK" : "one or more SAPs failed — check TMCL log");
    clear_fault_flags(d);
}

// ---------------------------------------------------------------------------
// One-call full bring-up
// ---------------------------------------------------------------------------

/**
 * @brief Configure the gate driver, current sensing, motor parameters, FOC PI
 *        gains, protections, ramp, brake, stop events, heartbeat, power,
 *        calibrate ADC offsets, and assert DRV_EN. After this returns true,
 *        the chip is **armed in SYSTEM_OFF**; caller picks the commutation
 *        mode and target.
 *
 * @note Defaults are tuned for the **Vortex 3-phase power stage with TMC9660
 *       firmware 051V100**: Y2 phase is skipped, gate-current SAPs (NR 245/246)
 *       are skipped (`REPLY_INVALID_VALUE` for the auto-selected enums on this
 *       bench — use `diagnose_uvw_gate_current_tmcl()` from a telemetry build to
 *       confirm per silicon), and the full EvKit-style OC + VGS SAP block is
 *       skipped because this firmware/board combination rejects parts of that
 *       stack (`REPLY_INVALID_CMD` / `REPLY_INVALID_VALUE` — see
 *       `docs/BLDC_UART_BRINGUP.md`). Motor spin still needs
 *       `disable_vortex_uvw_vgs_short_protection()` when VGS-short defaults
 *       false-trip on UVW — see `vortex_bldc_open_loop` / `vortex_bldc_open_loop_uart`.
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
                                    bool enable_outputs = true,
                                    bool with_oc_vgs_protection = false,
                                    bool with_gate_current_limits = false,
                                    bool skip_y2_phase = true) noexcept {
    ESP_LOGI(tag, "==== Vortex BLDC bring-up start (24V class, %u pole pairs, %lu Hz PWM) ====",
             static_cast<unsigned>(pole_pairs), static_cast<unsigned long>(pwm_hz));
    if (!set_system_off(d, tag))                                                   return false;
    // Set BLDC + pole pairs before gate-driver TMCL: configurePowerStageProtection programs
    // Y2 as well as UVW; on 3-phase BLDC builds the IC rejects Y2 axis SAPs until motor type is BLDC.
    if (!configure_motor(d, tag, pole_pairs, pwm_hz))                              return false;
    if (!configure_gate_driver(d, tag, with_oc_vgs_protection,
                                       with_gate_current_limits,
                                       skip_y2_phase))                             return false;
    if (!configure_current_sensing(d, tag))                  return false;
    if (!configure_torque_flux_loop(d, tag))                 return false;
    if (!configure_velocity_loop(d, tag))                    return false;
    if (!configure_protection(d, tag))                       return false;
    if (!configure_ramp_passthrough(d, tag))                 return false;
    (void)configure_brake_disabled(d, tag);                  // non-fatal
    (void)configure_stop_events_disabled(d, tag);            // non-fatal
    if (!configure_heartbeat_defaults(d, tag))               return false;
    if (!configure_power_defaults(d, tag))                    return false;
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
