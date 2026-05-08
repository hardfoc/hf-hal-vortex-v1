/**
 * @file vortex_motor_bench_probe.hpp
 * @brief Read-only probes for the Vortex BLDC bench: PCAL95555 control-line
 *        snapshot, TMC9660 parameter dump, fault/status flags, and a
 *        TARGET_VELOCITY rejection matrix.
 *
 * These helpers ride alongside `vortex_motor_bench_common.hpp` and are split
 * out so motion bench apps stay small, but a `vortex_bldc_probe` app can pull
 * the kitchen sink in one include.
 *
 * Nothing in here mutates safe state — `setCommutationMode(SYSTEM_OFF)` and
 * `disable_drv_en()` should be the bookends in caller scope.
 */
#pragma once

#include "managers/GpioManager.h"
#include "vortex_bench_safety.hpp"
#include "vortex_motor_bench_common.hpp"

#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <utility>

namespace vortex_motor_bench_probe {

namespace tmcl = tmc9660::tmcl;

// ===========================================================================
// PCAL95555 control-line snapshot
// ===========================================================================

/**
 * @brief Read the full 16-bit PCAL95555 input port snapshot and log the
 *        electrical level of each TMC9660-control net.
 *
 * Logical-vs-physical: the PCAL pins are wired with the polarities defined in
 * `hf_functional_pin_config_vortex_v1.hpp` (PCAL_TMC_FAULT_STATUS / SPI_COMM_EN
 * / WAKE_CTRL are inverted; DRV_EN and RST_CTRL are active-high).  This dump
 * shows the *raw electrical* port bit so it's unambiguous on a scope.
 *
 * @param tag   ESP_LOG tag.
 * @param label Short label (e.g. `"pre-init"`, `"after DRV_EN"`).
 */
inline void dump_pcal95555_lines(const char* tag, const char* label) noexcept {
    auto& gpio = GpioManager::GetInstance();
    if (!gpio.EnsureInitialized()) {
        ESP_LOGW(tag, "[%s] GpioManager not initialised — skipping PCAL dump", label);
        return;
    }

    uint16_t bits = 0;
    auto err = gpio.DebugReadPcal95555InputSnapshot(bits);
    if (err != hf_gpio_err_t::GPIO_SUCCESS) {
        ESP_LOGW(tag, "[%s] PCAL95555 input snapshot failed (err=%d)", label,
                 static_cast<int>(err));
        return;
    }

    auto bit = [bits](uint8_t pin) -> int { return static_cast<int>((bits >> pin) & 0x1u); };

    ESP_LOGI(tag, "[%s] PCAL95555 raw inputs = 0x%04X (port1<<8 | port0)", label,
             static_cast<unsigned>(bits));
    ESP_LOGI(tag,
             "    P0.3 FAULT_STATUS=%d (active-low)   P0.4 DRV_EN=%d   P0.5 RST_CTRL=%d   "
             "P0.6 PWR_GOOD=%d",
             bit(3), bit(4), bit(5), bit(6));
    ESP_LOGI(tag, "    P1.5 SPI_COMM_EN=%d (active-low)   P1.6 WAKE_CTRL=%d (active-low)",
             bit(13), bit(14));
    ESP_LOGI(tag, "    P0.1 TMC_GPIO17_EXP_IN=%d   P0.2 TMC_GPIO18_EXP_IN=%d", bit(1), bit(2));

    const bool drv_en_high = bit(4) != 0;
    const bool fault_asserted = bit(3) == 0;     // active-low
    const bool comm_enabled  = bit(13) == 0;     // active-low
    const bool wake_high     = bit(14) == 0;     // active-low (driven low to "wake")
    ESP_LOGI(tag,
             "    interpretation: DRV_EN %s | FAULT %s | SPI_COMM %s | WAKE %s",
             drv_en_high ? "ACTIVE (gate driver allowed to switch)" : "INACTIVE",
             fault_asserted ? "ASSERTED (TMC9660 reports FAULTN low)" : "released",
             comm_enabled ? "enabled (SPI mux open)" : "disabled",
             wake_high ? "asserted (chip kept awake)" : "deasserted");
}

// ===========================================================================
// Parameter dump
// ===========================================================================

/// Simple `(id, name)` table entry; we keep it private to this header.
struct ParamEntry {
    tmcl::Parameters id;
    const char* name;
    bool is_signed;
};

/// Curated list of parameters relevant to BLDC open-loop / FOC bring-up.
/// Order mirrors the bring-up sequence so the log reads like a story.
inline constexpr std::array<ParamEntry, 33> kBldcParamTable = {{
    {tmcl::Parameters::MOTOR_TYPE,                 "MOTOR_TYPE (0)",                 false},
    {tmcl::Parameters::MOTOR_POLE_PAIRS,           "MOTOR_POLE_PAIRS (1)",           false},
    {tmcl::Parameters::MOTOR_DIRECTION,            "MOTOR_DIRECTION (2)",            false},
    {tmcl::Parameters::MOTOR_PWM_FREQUENCY,        "MOTOR_PWM_FREQUENCY (3)",        false},
    {tmcl::Parameters::COMMUTATION_MODE,           "COMMUTATION_MODE (4)",           false},
    {tmcl::Parameters::OUTPUT_VOLTAGE_LIMIT,       "OUTPUT_VOLTAGE_LIMIT (5)",       false},
    {tmcl::Parameters::MAX_TORQUE,                 "MAX_TORQUE (6)",                 false},
    {tmcl::Parameters::MAX_FLUX,                   "MAX_FLUX (7)",                   false},
    {tmcl::Parameters::PWM_SWITCHING_SCHEME,       "PWM_SWITCHING_SCHEME (8)",       false},
    {tmcl::Parameters::IDLE_MOTOR_PWM_BEHAVIOR,    "IDLE_MOTOR_PWM_BEHAVIOR (9)",    false},
    {tmcl::Parameters::OPENLOOP_ANGLE,             "OPENLOOP_ANGLE (45)",            true},
    {tmcl::Parameters::OPENLOOP_CURRENT,           "OPENLOOP_CURRENT (46) [mA]",     false},
    {tmcl::Parameters::OPENLOOP_VOLTAGE,           "OPENLOOP_VOLTAGE (47) 0..16383", false},
    {tmcl::Parameters::RAMP_ENABLE,                "RAMP_ENABLE (52)",               false},
    {tmcl::Parameters::DIRECT_VELOCITY_MODE,       "DIRECT_VELOCITY_MODE (53)",      false},
    {tmcl::Parameters::RAMP_AMAX,                  "RAMP_AMAX (54)",                 false},
    {tmcl::Parameters::RAMP_VMAX,                  "RAMP_VMAX (60)",                 false},
    {tmcl::Parameters::RAMP_V1,                    "RAMP_V1 (61)",                   false},
    {tmcl::Parameters::RAMP_V2,                    "RAMP_V2 (62)",                   false},
    {tmcl::Parameters::RAMP_VSTART,                "RAMP_VSTART (63)",               false},
    {tmcl::Parameters::RAMP_VSTOP,                 "RAMP_VSTOP (64)",                false},
    {tmcl::Parameters::RAMP_VELOCITY,              "RAMP_VELOCITY (69)",             true},
    {tmcl::Parameters::RAMP_POSITION,              "RAMP_POSITION (70)",             true},
    {tmcl::Parameters::TARGET_TORQUE,              "TARGET_TORQUE (104) [mA]",       true},
    {tmcl::Parameters::VELOCITY_SENSOR_SELECTION,  "VELOCITY_SENSOR_SELECTION (123)",false},
    {tmcl::Parameters::TARGET_VELOCITY,            "TARGET_VELOCITY (124)",          true},
    {tmcl::Parameters::ACTUAL_VELOCITY,            "ACTUAL_VELOCITY (125)",          true},
    {tmcl::Parameters::VELOCITY_SCALING_FACTOR,    "VELOCITY_SCALING_FACTOR (133)",  false},
    {tmcl::Parameters::VELOCITY_LOOP_DOWNSAMPLING, "VELOCITY_LOOP_DOWNSAMPLING (135)", false},
    {tmcl::Parameters::SUPPLY_VOLTAGE,             "SUPPLY_VOLTAGE (290) [0.1V]",    false},
    {tmcl::Parameters::CHIP_TEMPERATURE,           "CHIP_TEMPERATURE (296)",         false},
    {tmcl::Parameters::FOC_VOLTAGE_UQ,             "FOC_VOLTAGE_UQ (314)",           true},
    {tmcl::Parameters::FOC_CURRENT_IQ,             "FOC_CURRENT_IQ (315) [mA]",      true},
}};

template <typename Comm>
inline void dump_params(tmc9660::TMC9660<Comm>& d, const char* tag, const char* label) noexcept {
    ESP_LOGI(tag, "[%s] -- TMC9660 parameter snapshot --", label);
    for (const auto& e : kBldcParamTable) {
        uint32_t value = 0;
        if (!d.readParameter(e.id, value)) {
            ESP_LOGW(tag, "    %-40s = <GAP failed>", e.name);
            continue;
        }
        if (e.is_signed) {
            ESP_LOGI(tag, "    %-40s = %ld (0x%08X)", e.name,
                     static_cast<long>(static_cast<int32_t>(value)),
                     static_cast<unsigned>(value));
        } else {
            ESP_LOGI(tag, "    %-40s = %lu (0x%08X)", e.name,
                     static_cast<unsigned long>(value), static_cast<unsigned>(value));
        }
    }
}

// ===========================================================================
// Status / fault flag dump (decoded)
// ===========================================================================

template <typename Comm>
inline void dump_status_decoded(tmc9660::TMC9660<Comm>& d, const char* tag,
                                const char* label) noexcept {
    uint32_t gen_status = 0, gen_err = 0, gd_err = 0, adc_status = 0;
    (void)d.telemetry.getGeneralStatusFlags(gen_status);
    (void)d.telemetry.getGeneralErrorFlags(gen_err);
    (void)d.telemetry.getGateDriverErrorFlags(gd_err);
    (void)d.telemetry.getADCStatusFlags(adc_status);
    ESP_LOGI(tag, "[%s] gen_status=0x%08X gen_err=0x%08X gd_err=0x%08X adc_status=0x%08X", label,
             static_cast<unsigned>(gen_status), static_cast<unsigned>(gen_err),
             static_cast<unsigned>(gd_err), static_cast<unsigned>(adc_status));

    auto bit = [](uint32_t v, uint8_t b) { return (v >> b) & 0x1u; };

    ESP_LOGI(tag,
             "  gen_status: STOPPED=%u TORQUE=%u VELOCITY=%u POSITION=%u  "
             "ADC_OFF_CAL=%u RAMPER_LATCHED=%u POS_REACHED=%u VEL_REACHED=%u",
             bit(gen_status, 0), bit(gen_status, 1), bit(gen_status, 2), bit(gen_status, 3),
             bit(gen_status, 11), bit(gen_status, 12), bit(gen_status, 9), bit(gen_status, 10));
    ESP_LOGI(tag,
             "  gen_status feedback bits: ABN1=%u ABN2=%u HALL=%u STEPDIR=%u",
             bit(gen_status, 29), bit(gen_status, 27), bit(gen_status, 28), bit(gen_status, 23));
    ESP_LOGI(tag,
             "  gen_err: HALL=%u WDOG=%u CHIP_OT_EXC=%u IIT1=%u IIT2=%u OV_WARN=%u UV_WARN=%u "
             "ADC_OV=%u FAULT_RETRY=%u FAULT_FAIL=%u CHIP_OT_WARN=%u HEARTBEAT_STOP=%u",
             bit(gen_err, 5), bit(gen_err, 9), bit(gen_err, 14), bit(gen_err, 16), bit(gen_err, 17),
             bit(gen_err, 19), bit(gen_err, 20), bit(gen_err, 21), bit(gen_err, 22),
             bit(gen_err, 23), bit(gen_err, 24), bit(gen_err, 26));
}

// ===========================================================================
// TARGET_VELOCITY / open-loop motion test matrix
// ===========================================================================

struct VelocityShot {
    int32_t value;
    bool ramp_enable;
    bool direct_velocity_mode;
    const char* note;
};

template <typename Comm>
inline bool write_int_param(tmc9660::TMC9660<Comm>& d, tmcl::Parameters id,
                            int32_t v) noexcept {
    return d.writeParameter(id, static_cast<uint32_t>(v));
}

template <typename Comm>
inline bool write_uint_param(tmc9660::TMC9660<Comm>& d, tmcl::Parameters id,
                             uint32_t v) noexcept {
    return d.writeParameter(id, v);
}

/**
 * @brief Run a matrix of TARGET_VELOCITY writes through different ramp/dvm
 *        combinations to identify exactly which prerequisite state our chip
 *        firmware accepts.
 *
 * Caller is responsible for selecting commutation mode and open-loop magnitude
 * (`OPENLOOP_VOLTAGE` in voltage mode, `OPENLOOP_CURRENT` in current mode)
 * **before** calling. This helper optionally **re-applies** open-loop voltage
 * (datasheet order: magnitude then `TARGET_VELOCITY`) and inserts short delays
 * for **Erratum 4** (first target updates quantized to ~1 ms) after prerequisite
 * changes and ramp/DVM tweaks.
 *
 * Each shot reports SAP success and reads back TARGET_VELOCITY,
 * RAMP_VELOCITY, ACTUAL_VELOCITY, and OPENLOOP_ANGLE so we can see whether the
 * ramper actually advances.
 */
template <typename Comm>
inline void exercise_target_velocity_matrix(tmc9660::TMC9660<Comm>& d, const char* tag,
                                            const char* label,
                                            bool prime_openloop_voltage = true) noexcept {
    if (prime_openloop_voltage) {
        (void)d.torqueFluxControl.setOpenloopVoltage(vortex_bench_safety::kOpenLoopVoltage);
        vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kErratum4TargetCoalesceMs));
    } else {
        vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kErratum4TargetCoalesceMs));
    }

    constexpr std::array<VelocityShot, 8> shots = {{
        {0,        false, true,  "baseline zero, ramp OFF / DVM ON"},
        {0,        true,  true,  "baseline zero, ramp ON  / DVM ON"},
        {1,        true,  true,  "tiny +1,    ramp ON / DVM ON"},
        {350,      true,  true,  "small +350, ramp ON / DVM ON"},
        {-350,     true,  true,  "small -350, ramp ON / DVM ON"},
        {350,      false, true,  "small +350, ramp OFF / DVM ON"},
        {350,      true,  false, "small +350, ramp ON / DVM OFF (position-based)"},
        {350,      false, false, "small +350, ramp OFF / DVM OFF"},
    }};

    for (const auto& s : shots) {
        // Apply prerequisites (best-effort; record whether each succeeded).
        bool ramp_ok = d.ramp.enable(s.ramp_enable);
        bool dvm_ok  = d.ramp.setDirectVelocityMode(s.direct_velocity_mode);
        vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kAfterRampDvmTweakMs));

        bool tv_ok = d.velocityControl.setTargetVelocity(s.value);

        // Read back the four interesting registers (always — even on failure).
        uint32_t tv_rb = 0, rv_rb = 0, av_rb = 0, ang_rb = 0, gen_status = 0;
        bool tv_rb_ok  = d.readParameter(tmcl::Parameters::TARGET_VELOCITY, tv_rb);
        bool rv_rb_ok  = d.readParameter(tmcl::Parameters::RAMP_VELOCITY, rv_rb);
        bool av_rb_ok  = d.readParameter(tmcl::Parameters::ACTUAL_VELOCITY, av_rb);
        bool ang_rb_ok = d.readParameter(tmcl::Parameters::OPENLOOP_ANGLE, ang_rb);
        (void)d.telemetry.getGeneralStatusFlags(gen_status);

        ESP_LOGI(tag,
                 "[%s] shot %s  -> SAP_TV=%s SAP_RAMP=%s SAP_DVM=%s  "
                 "TV_rb=%ld RAMP_V_rb=%ld ACTUAL_V_rb=%ld OPENLOOP_ANGLE_rb=%ld  "
                 "REG_VEL_bit=%u REG_TORQUE_bit=%u",
                 label, s.note,
                 tv_ok ? "OK" : "FAIL", ramp_ok ? "OK" : "FAIL", dvm_ok ? "OK" : "FAIL",
                 tv_rb_ok  ? static_cast<long>(static_cast<int32_t>(tv_rb))  : -1L,
                 rv_rb_ok  ? static_cast<long>(static_cast<int32_t>(rv_rb))  : -1L,
                 av_rb_ok  ? static_cast<long>(static_cast<int32_t>(av_rb))  : -1L,
                 ang_rb_ok ? static_cast<long>(static_cast<int32_t>(ang_rb)) : -1L,
                 static_cast<unsigned>((gen_status >> 2) & 1u),
                 static_cast<unsigned>((gen_status >> 1) & 1u));

        // Settle a moment so OPENLOOP_ANGLE has a chance to advance if the ramper engaged.
        vTaskDelay(pdMS_TO_TICKS(150));

        // Tear motion back to zero before next shot, ignoring failures.
        (void)d.velocityControl.setTargetVelocity(0);
        vTaskDelay(pdMS_TO_TICKS(vortex_bench_safety::kErratum4TargetCoalesceMs));
    }
}

// ===========================================================================
// Open-loop "is the gate driver actually switching?" probe
// ===========================================================================

/**
 * @brief Hold OPENLOOP_VOLTAGE on, **without** any TARGET_VELOCITY motion, and
 *        sample phase ADCs / FOC voltages so we can see whether the gate driver
 *        actually applies anything to the motor.
 *
 * The TMC9660 docs say `OPENLOOP_VOLTAGE` only "starts" once a TARGET_VELOCITY
 * is written, but the FOC current loop / ADC pipeline runs continuously after
 * COMMUTATION_MODE != SYSTEM_OFF — so phase currents should still settle to
 * something measurable if DRV_EN is genuinely live.
 */
template <typename Comm>
inline void probe_static_openloop(tmc9660::TMC9660<Comm>& d, const char* tag,
                                  uint16_t openloop_voltage = 800,
                                  uint16_t openloop_current_mA = 100,
                                  uint32_t hold_ms = 600) noexcept {
    ESP_LOGI(tag, "Static open-loop probe: V=%u C=%u mA hold=%lu ms (no TARGET_VELOCITY)",
             static_cast<unsigned>(openloop_voltage), static_cast<unsigned>(openloop_current_mA),
             static_cast<unsigned long>(hold_ms));

    bool v_ok = d.torqueFluxControl.setOpenloopVoltage(openloop_voltage);
    bool c_ok = d.torqueFluxControl.setOpenloopCurrent(openloop_current_mA);
    ESP_LOGI(tag, "  setOpenloopVoltage=%s, setOpenloopCurrent=%s",
             v_ok ? "OK" : "FAIL", c_ok ? "OK" : "FAIL");

    const uint32_t step = 100;  // ms
    for (uint32_t t = 0; t < hold_ms; t += step) {
        vTaskDelay(pdMS_TO_TICKS(step));
        uint32_t ux = 0, vph = 0, wy = 0, ang = 0;
        (void)d.readParameter(tmcl::Parameters::FOC_CURRENT_UX, ux);
        (void)d.readParameter(tmcl::Parameters::FOC_CURRENT_V, vph);
        (void)d.readParameter(tmcl::Parameters::FOC_CURRENT_WY, wy);
        (void)d.readParameter(tmcl::Parameters::OPENLOOP_ANGLE, ang);
        const float vbus = d.telemetry.getSupplyVoltage();
        const float tchip = d.telemetry.getChipTemperature();
        ESP_LOGI(tag, "  t=%lums Vbus=%.2fV T=%.1fC  I_UX=%d I_V=%d I_WY=%d  ANGLE=%d",
                 static_cast<unsigned long>(t), static_cast<double>(vbus),
                 static_cast<double>(tchip),
                 static_cast<int>(static_cast<int16_t>(ux)),
                 static_cast<int>(static_cast<int16_t>(vph)),
                 static_cast<int>(static_cast<int16_t>(wy)),
                 static_cast<int>(static_cast<int16_t>(ang)));
    }
}

// ===========================================================================
// OPENLOOP_ANGLE writability probe + software commutator
// ===========================================================================
//
// In the latest documented Parameter Mode firmware OPENLOOP_ANGLE (id 45) is
// `R` (read-only) — it's calculated by the chip's internal ramper from
// TARGET_VELOCITY.  But on the chip ROM we have (`051V100`), TARGET_VELOCITY
// writes are rejected with REPLY_INVALID_VALUE, so the ramper never advances.
//
// Some Trinamic firmwares accept SAP writes to "R" parameters anyway.  This
// helper does two things:
//
//   1. Tries SAP 45 = {0, 1024, 4096, -4096, 0} and reports REPLY codes +
//      readbacks.  If every SAP returns OK and the readback matches, the
//      firmware is effectively letting us drive PHI_E from software.
//
//   2. If the writability check passes, runs a `spin_ms` software commutator:
//      writes OPENLOOP_ANGLE = (ang0 + step*tick) mod 32768 every `dt_ms` ms
//      while sampling ACTUAL_TOTAL_MOTOR_CURRENT.  A motor with the gate
//      driver alive will drag synchronously with that vector — we'll see the
//      current dance and (if mechanical load allows) the rotor catch up.
//
// Caller responsibilities (must be set BEFORE calling):
//   • COMMUTATION_MODE = FOC_OPENLOOP_VOLTAGE_MODE (3)
//   • OPENLOOP_VOLTAGE > 0  (e.g. 600..1200 for a 24 V bus)
//   • DRV_EN asserted
//   • RAMP_ENABLE = 0 (so the chip's ramper doesn't fight us)

template <typename Comm>
inline bool probe_openloop_angle_writability(tmc9660::TMC9660<Comm>& d,
                                             const char* tag) noexcept {
    ESP_LOGI(tag, "==== OPENLOOP_ANGLE (45) writability probe ====");
    constexpr std::array<int16_t, 5> shots = {0, 1024, 4096, -4096, 0};
    bool all_ok = true;
    for (auto target : shots) {
        const bool wr = d.writeParameter(tmcl::Parameters::OPENLOOP_ANGLE,
                                         static_cast<uint32_t>(target));
        uint32_t rb = 0;
        const bool rd = d.readParameter(tmcl::Parameters::OPENLOOP_ANGLE, rb);
        const int16_t rb_signed = static_cast<int16_t>(rb);
        ESP_LOGI(tag, "  SAP OPENLOOP_ANGLE=%d -> SAP=%s  GAP_rb=%d (%s)  match=%s",
                 static_cast<int>(target),
                 wr ? "OK" : "FAIL",
                 static_cast<int>(rb_signed),
                 rd ? "OK" : "FAIL",
                 (wr && rd && rb_signed == target) ? "YES" : "no");
        if (!wr || !rd || rb_signed != target) all_ok = false;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    ESP_LOGI(tag, "  -> OPENLOOP_ANGLE is %sWRITABLE on this firmware",
             all_ok ? "" : "NOT ");
    return all_ok;
}

template <typename Comm>
inline void software_commutator_spin(tmc9660::TMC9660<Comm>& d, const char* tag,
                                     int16_t step_per_tick = 256,
                                     uint32_t dt_ms = 5,
                                     uint32_t spin_ms = 4000) noexcept {
    ESP_LOGI(tag, "==== Software commutator spin: step=%d / %lums for %lums ====",
             static_cast<int>(step_per_tick),
             static_cast<unsigned long>(dt_ms),
             static_cast<unsigned long>(spin_ms));
    int32_t ang = 0;
    const uint32_t ticks = (spin_ms + dt_ms - 1) / dt_ms;
    const uint32_t log_every = (ticks > 50) ? (ticks / 50) : 1;
    for (uint32_t i = 0; i < ticks; ++i) {
        ang = static_cast<int16_t>(ang + step_per_tick);  // wrap on int16
        (void)d.writeParameter(tmcl::Parameters::OPENLOOP_ANGLE,
                               static_cast<uint32_t>(static_cast<int16_t>(ang)));
        if ((i % log_every) == 0) {
            uint32_t imot = 0, ang_rb = 0;
            (void)d.readParameter(tmcl::Parameters::ACTUAL_TOTAL_MOTOR_CURRENT, imot);
            (void)d.readParameter(tmcl::Parameters::OPENLOOP_ANGLE, ang_rb);
            ESP_LOGI(tag, "  t=%lums  ang_cmd=%d  ang_rb=%d  I_motor=%d mA",
                     static_cast<unsigned long>(i * dt_ms),
                     static_cast<int>(static_cast<int16_t>(ang)),
                     static_cast<int>(static_cast<int16_t>(ang_rb)),
                     static_cast<int>(imot));
        }
        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
    ESP_LOGI(tag, "  -> commutator loop complete");
}

}  // namespace vortex_motor_bench_probe
