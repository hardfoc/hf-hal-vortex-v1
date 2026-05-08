/**
 * @file vortex_tmcl_manual_compliance.hpp
 * @brief Representative TMCL checks aligned with *TMC9660 Parameter Mode Reference Manual*
 *        (Rev 0, 02/25): Table 18 operations, Table 57 axis parameters, Tables 61–63 globals,
 *        RamDebug Table 16, RFS command types, flag parameters (289–301).
 *
 * Intentionally avoids destructive or mode-exiting ops: no FactoryDefault, Boot→bootloader,
 * STAP/NVM store, SAP writes, RFS START, scripting download/run.
 */
#pragma once

#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp"
#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/parameter_mode/tmc9660_param_mode_tmcl.hpp"

#include "esp_log.h"

#include <cstdint>

namespace vortex_tmcl_manual_compliance {

namespace tmcl = tmc9660::tmcl;

template <typename D>
inline void log_probe(const char* tag, const char* case_name, D& d, tmcl::Op op, uint16_t type,
                      uint8_t motor_bank, uint32_t value) noexcept {
  uint32_t reply = 0;
  tmcl::ReplyCode st = tmcl::ReplyCode::REPLY_OK;
  const bool xfer = d.sendCommand(op, type, motor_bank, value, &reply, &st);
  const bool ok = xfer && (st == tmcl::ReplyCode::REPLY_OK || st == tmcl::ReplyCode::REPLY_CMD_LOADED);
  ESP_LOGI(tag,
           "%s  %s  op=%s type=%u bank/motor=%u value=0x%08lX  reply=0x%08lX  tmcl=%s  %s",
           ok ? "PASS" : "FAIL", case_name, tmcl::to_string(op), static_cast<unsigned>(type),
           static_cast<unsigned>(motor_bank), static_cast<unsigned long>(value),
           static_cast<unsigned long>(reply), tmcl::to_string(st), xfer ? "xfer" : "no-xfer");
}

/**
 * Run a fixed battery of checks. Logs PASS/FAIL per line; does not abort on first failure.
 */
template <typename D>
inline void run_checks(D& d, const char* tag) noexcept {
  ESP_LOGI(tag, "TMC9660 Parameter Mode manual compliance (representative reads, Rev 0 Table 18+)");

  // Table 18 — GetInfo (157)
  log_probe(tag, "GetInfo ID (type=0)", d, tmcl::Op::GetInfo, 0, 0, 0);
  log_probe(tag, "GetInfo Version (type=1)", d, tmcl::Op::GetInfo, 1, 0, 0);
  log_probe(tag, "GetVersion (op 136)", d, tmcl::Op::GetVersion, 0, 0, 0);

  // Table 57 — GAP axis parameters (manual: motor number must be zero for SAP/GAP in doc text)
  constexpr uint8_t kMotor = 0;
  log_probe(tag, "GAP MOTOR_TYPE (NR 0)", d, tmcl::Op::GAP,
            static_cast<uint16_t>(tmcl::Parameters::MOTOR_TYPE), kMotor, 0);
  log_probe(tag, "GAP COMMUTATION_MODE (NR 4)", d, tmcl::Op::GAP,
            static_cast<uint16_t>(tmcl::Parameters::COMMUTATION_MODE), kMotor, 0);
  log_probe(tag, "GAP MOTOR_PWM_FREQUENCY (NR 3)", d, tmcl::Op::GAP,
            static_cast<uint16_t>(tmcl::Parameters::MOTOR_PWM_FREQUENCY), kMotor, 0);
  log_probe(tag, "GAP GENERAL_STATUS_FLAGS (289)", d, tmcl::Op::GAP,
            static_cast<uint16_t>(tmcl::Parameters::GENERAL_STATUS_FLAGS), kMotor, 0);
  log_probe(tag, "GAP GENERAL_ERROR_FLAGS (299)", d, tmcl::Op::GAP,
            static_cast<uint16_t>(tmcl::Parameters::GENERAL_ERROR_FLAGS), kMotor, 0);
  log_probe(tag, "GAP GDRV_ERROR_FLAGS (300)", d, tmcl::Op::GAP,
            static_cast<uint16_t>(tmcl::Parameters::GDRV_ERROR_FLAGS), kMotor, 0);
  log_probe(tag, "GAP SUPPLY_VOLTAGE (290)", d, tmcl::Op::GAP,
            static_cast<uint16_t>(tmcl::Parameters::SUPPLY_VOLTAGE), kMotor, 0);

  // Tables 61–63 — GGP globals (banks 0, 2, 3)
  log_probe(tag, "GGP bank0 SERIAL_ADDRESS (NR 1)", d, tmcl::Op::GGP,
            static_cast<uint16_t>(tmcl::GlobalParamBank0::SERIAL_ADDRESS), 0, 0);
  log_probe(tag, "GGP bank0 HEARTBEAT_MONITORING_CONFIG (3)", d, tmcl::Op::GGP,
            static_cast<uint16_t>(tmcl::GlobalParamBank0::HEARTBEAT_MONITORING_CONFIG), 0, 0);
  log_probe(tag, "GGP bank0 MAIN_LOOPS (12)", d, tmcl::Op::GGP,
            static_cast<uint16_t>(tmcl::GlobalParamBank0::MAIN_LOOPS), 0, 0);
  log_probe(tag, "GGP bank2 USER_VARIABLE_0", d, tmcl::Op::GGP,
            static_cast<uint16_t>(tmcl::GlobalParamBank2::USER_VARIABLE_0), 2, 0);
  log_probe(tag, "GGP bank3 TIMER_0_PERIOD", d, tmcl::Op::GGP,
            static_cast<uint16_t>(tmcl::GlobalParamBank3::TIMER_0_PERIOD), 3, 0);

  // Table 18 — MST (3): stop movement (safe when already idle)
  log_probe(tag, "MST stop motor", d, tmcl::Op::MST, 0, 0, 0);

  // Table 18 / Table 46 — RFS STATUS only (does not start reference search)
  log_probe(tag, "RFS STATUS (type=2)", d, tmcl::Op::RFS,
            static_cast<uint16_t>(tmcl::ReferenceSearchCommand::STATUS), 0, 0);

  // Table 16 — RamDebug: reset then idle state read (no capture started)
  log_probe(tag, "RamDebug INITIALISE_RESET (type 0)", d, tmcl::Op::RamDebug,
            static_cast<uint16_t>(tmcl::RamDebugType::INITIALISE_RESET), 0, 0);
  log_probe(tag, "RamDebug GET_STATE (type 8)", d, tmcl::Op::RamDebug,
            static_cast<uint16_t>(tmcl::RamDebugType::GET_STATE), 0, 0);
  log_probe(tag, "RamDebug GET_INFO max channels (type 10, value=0)", d, tmcl::Op::RamDebug,
            static_cast<uint16_t>(tmcl::RamDebugType::GET_INFO), 0, 0);

  // Table 18 — script status query (does not run a program)
  log_probe(tag, "GetStatusScript (op 135)", d, tmcl::Op::GetStatusScript, 0, 0, 0);

  // Table 18 — Breakpoint type 3 = max breakpoints supported
  log_probe(tag, "Breakpoint get max (type=3)", d, tmcl::Op::Breakpoint, 3, 0, 0);

  // Table 18 — GIO read digital port 0 (GPIO must exist in boot config for meaningful value)
  log_probe(tag, "GIO digital port 0", d, tmcl::Op::GIO, 0, 0, 0);

  ESP_LOGI(tag, "Manual compliance sweep done (see PASS/FAIL above).");
}

}  // namespace vortex_tmcl_manual_compliance
