/**
 * @file vortex_tmcl_param_existence.hpp
 * @brief For each axis parameter *number* (NR), determine whether the running TMC9660 ROM accepts a
 *        read via **GAP (motor 0)** and/or **GGP (banks 0, 2, 3)** with the same TYPE field.
 *
 * Analog Devices *TMC9660 Parameter Mode Reference Manual* Rev 0 (02/25) **Table 57** lists axis
 * parameters NR **0…334** for SAP/GAP (motor/bank 0 per manual). That table is not the same as
 * **Table 53** (script interrupt indices) or **Tables 54–55** (CALC\* operation types).
 *
 * Globals remain **SGP/GGP** with banks 0, 2, 3 (Tables 61–63). This harness still probes **GGP
 * bank 0** with the same numeric TYPE as GAP for each axis NR (ROM discovery); that does **not**
 * imply those NRs are real Table 61 globals — true bank 0 parameters use the Table 61 IDs (see
 * `GlobalParamBank0` in the driver). Production code reads axis diagnostics via **GAP/SAP** only.
 *
 * Log format (grep-friendly):
 *   `INV nr=<u> sym=<name> win=<GAP0|GGP0|GGP2|GGP3|SKIP|NONE> st=<tmcl_u8> v=0x........`
 */
#pragma once

#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp"
#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/parameter_mode/tmc9660_param_mode_tmcl.hpp"
#include "vortex_tmcl_read_probe.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>

namespace vortex_tmcl_param_existence {

namespace tmcl = tmc9660::tmcl;

inline constexpr bool tmcl_ok(tmcl::ReplyCode st) noexcept {
  return st == tmcl::ReplyCode::REPLY_OK || st == tmcl::ReplyCode::REPLY_CMD_LOADED;
}

/// Highest axis NR in Rev 0 Table 57 (`INTEGRATED_ACTUAL_VELOCITY_VALUE` = 334). Matches `Parameters` top.
inline constexpr uint16_t kNrMaxInclusive = 334;

template <typename D>
inline void run_existence_matrix(D& d, const char* tag) noexcept {
  ESP_LOGI(tag,
           "TMC9660 param existence matrix (GAP motor0, Table57 gaps skipped, then GGP b0/b2/b3; "
           "NR 0..%u). Capture serial.",
           static_cast<unsigned>(kNrMaxInclusive));

  unsigned n = 0;
  for (uint16_t nr = 0; nr <= kNrMaxInclusive; ++nr, ++n) {
    const char* sym = tmcl::to_string(static_cast<tmcl::Parameters>(nr));
    const char* win = "NONE";
    tmcl::ReplyCode st_last = tmcl::ReplyCode::REPLY_CHKERR;
    uint32_t v_last = 0;
    bool xfer_last = false;

    if (vortex_tmcl_read_probe::gap_axis_nr_is_table57_gap(nr)) {
      ESP_LOGI(tag,
               "INV nr=%4u sym=%-28s win=%-5s st=%3u v=0x%08lX xfer=%d", static_cast<unsigned>(nr),
               sym, "SKIP", 0u, 0ul, 0);
      if ((n & 0xFu) == 0)
        vTaskDelay(1);
      continue;
    }

    auto try_one = [&](tmcl::Op op, uint8_t motor_or_bank, const char* wlabel) -> bool {
      uint32_t v = 0;
      tmcl::ReplyCode st = tmcl::ReplyCode::REPLY_OK;
      const bool xfer = d.sendCommand(op, nr, motor_or_bank, 0, &v, &st);
      st_last = st;
      v_last = v;
      xfer_last = xfer;
      if (xfer && tmcl_ok(st)) {
        win = wlabel;
        return true;
      }
      return false;
    };

    if (try_one(tmcl::Op::GAP, 0, "GAP0")) {
    } else if (try_one(tmcl::Op::GGP, 0, "GGP0")) {
    } else if (try_one(tmcl::Op::GGP, 2, "GGP2")) {
    } else {
      (void)try_one(tmcl::Op::GGP, 3, "GGP3");
    }

    const unsigned st_u8 =
        xfer_last ? static_cast<unsigned>(static_cast<std::uint8_t>(st_last)) : 255u;
    ESP_LOGI(tag, "INV nr=%4u sym=%-28s win=%-5s st=%3u v=0x%08lX xfer=%d", static_cast<unsigned>(nr),
             sym, win, st_u8, static_cast<unsigned long>(v_last), xfer_last ? 1 : 0);

    if ((n & 0xFu) == 0)
      vTaskDelay(1);
  }

  ESP_LOGI(tag, "Existence matrix done (filter INV lines; win=NONE => no path returned OK).");
}

}  // namespace vortex_tmcl_param_existence
