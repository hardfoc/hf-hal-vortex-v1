/**
 * @file vortex_tmcl_read_probe.hpp
 * @brief Systematic TMCL read probe: **GAP** on **motor 0 only** for Rev 0 **Table 57** axis NRs
 *        **0…334** excluding NR gaps (no row in that table); then **GGP** on global banks 0/2/3.
 *
 * Uses `TMC9660::sendCommand` with optional `ReplyCode` so every attempt is classified:
 * transport failure (`REPLY_CHKERR` sentinel), TMCL NACK (wrong type, invalid value, …), or OK.
 *
 * Numbers above 334 are not Table 57 axis parameters. Global bank 0 is swept 0…127; banks 2 and 3
 * use small NR spaces (unchanged dense sweep).
 */
#pragma once

#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp"
#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/parameter_mode/tmc9660_param_mode_tmcl.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>

namespace vortex_tmcl_read_probe {

namespace tmcl = tmc9660::tmcl;

/// Highest axis parameter NR in Rev 0 Table 57 / `Parameters` enum (334).
inline constexpr uint16_t kGapAxisTypeMaxInclusive = 334;

/// NRs with **no** entry in Rev 0 Table 57 (jumps between listed rows). Not sent on GAP.
inline constexpr uint16_t kTable57GapAxisNrs[] = {
    10,  11,  42,  43,  44,  48,  49,  71,  72,  73,  101, 102, 103, 121, 122, 140, 141,
    171, 172, 173, 202, 203, 204, 210, 211, 220, 222, 223, 232, 284, 285, 302, 303, 309, 316, 317,
};

inline bool gap_axis_nr_is_table57_gap(uint16_t id) noexcept {
  for (uint16_t g : kTable57GapAxisNrs) {
    if (g == id)
      return true;
  }
  return false;
}

/// Global bank 0: sweep low NR space (documented entries are sparse; unknowns show as NACK).
inline constexpr uint16_t kGgpBank0TypeMaxInclusive = 127;
inline constexpr uint16_t kGgpBank2TypeMaxInclusive = 31;
inline constexpr uint16_t kGgpBank3TypeMaxInclusive = 35;

inline void hist_inc(uint32_t hist[256], tmcl::ReplyCode c) noexcept {
  const auto u = static_cast<unsigned>(static_cast<std::uint8_t>(c));
  hist[u < 256 ? u : 0]++;
}

template <typename Comm>
inline void scan_all_reads(tmc9660::TMC9660<Comm>& d, const char* tag) noexcept {
  // BSS — avoid ~4 KiB on the main task stack (default stack is small; scan was resetting the chip).
  static uint32_t hist_gap[256];
  static uint32_t hist_ggp0[256];
  static uint32_t hist_ggp2[256];
  static uint32_t hist_ggp3[256];
  for (auto& h : hist_gap)
    h = 0;
  for (auto& h : hist_ggp0)
    h = 0;
  for (auto& h : hist_ggp2)
    h = 0;
  for (auto& h : hist_ggp3)
    h = 0;

  unsigned gap_ok = 0, ggp0_ok = 0, ggp2_ok = 0, ggp3_ok = 0;
  unsigned n = 0;

  ESP_LOGI(tag, "Starting sweep (many WARN lines for non-OK reads are expected)...");

  auto log_fail_gap = [&](uint16_t type, tmcl::ReplyCode st, uint32_t val) {
    ESP_LOGW(tag,
             "GAP type=%u (%s) motor=0  TMCL=%s (0x%02X)  value=0x%08lX",
             static_cast<unsigned>(type), tmcl::to_string(static_cast<tmcl::Parameters>(type)),
             tmcl::to_string(st), static_cast<unsigned>(static_cast<std::uint8_t>(st)),
             static_cast<unsigned long>(val));
  };
  auto log_fail_ggp = [&](uint8_t bank, uint16_t type, tmcl::ReplyCode st, uint32_t val) {
    ESP_LOGW(tag,
             "GGP bank=%u type=%u  TMCL=%s (0x%02X)  value=0x%08lX", static_cast<unsigned>(bank),
             static_cast<unsigned>(type), tmcl::to_string(st),
             static_cast<unsigned>(static_cast<std::uint8_t>(st)),
             static_cast<unsigned long>(val));
  };

  constexpr uint8_t kGapMotor = 0;
  unsigned gap_probes = 0;
  for (uint16_t id = 0; id <= kGapAxisTypeMaxInclusive; ++id) {
    if (gap_axis_nr_is_table57_gap(id))
      continue;
    uint32_t val = 0;
    tmcl::ReplyCode st = tmcl::ReplyCode::REPLY_OK;
    const bool xfer = d.sendCommand(tmcl::Op::GAP, id, kGapMotor, 0, &val, &st);
    (void)xfer;
    ++n;
    hist_inc(hist_gap, st);
    if (st == tmcl::ReplyCode::REPLY_OK || st == tmcl::ReplyCode::REPLY_CMD_LOADED) {
      gap_ok++;
    } else {
      log_fail_gap(id, st, val);
    }
    if ((gap_probes++ & 0xFu) == 0)
      vTaskDelay(1);
  }

  n = 0;
  for (uint8_t bank : {static_cast<uint8_t>(0), static_cast<uint8_t>(2), static_cast<uint8_t>(3)}) {
    uint16_t tmax = (bank == 0) ? kGgpBank0TypeMaxInclusive
                     : (bank == 2) ? kGgpBank2TypeMaxInclusive
                                   : kGgpBank3TypeMaxInclusive;
    uint32_t* hist = (bank == 0) ? hist_ggp0 : (bank == 2) ? hist_ggp2 : hist_ggp3;
    unsigned* okc = (bank == 0) ? &ggp0_ok : (bank == 2) ? &ggp2_ok : &ggp3_ok;

    for (uint16_t id = 0; id <= tmax; ++id, ++n) {
      uint32_t val = 0;
      tmcl::ReplyCode st = tmcl::ReplyCode::REPLY_OK;
      (void)d.sendCommand(tmcl::Op::GGP, id, bank, 0, &val, &st);
      hist_inc(hist, st);
      if (st == tmcl::ReplyCode::REPLY_OK || st == tmcl::ReplyCode::REPLY_CMD_LOADED) {
        (*okc)++;
      } else {
        log_fail_ggp(bank, id, st, val);
      }
      if ((n & 0xFu) == 0)
        vTaskDelay(1);
    }
  }

  auto dump_hist = [&](const char* name, const uint32_t h[256]) {
    char line[160];
    size_t pos = 0;
    pos += static_cast<size_t>(
        snprintf(line + pos, sizeof(line) - pos, "%s TMCL status counts:", name));
    for (unsigned i = 0; i < 256; ++i) {
      if (h[i] == 0)
        continue;
      const int left = static_cast<int>(sizeof(line) - pos);
      if (left < 24)
        break;
      pos += static_cast<size_t>(snprintf(line + pos, static_cast<size_t>(left), " [%u]=%lu", i,
                                          static_cast<unsigned long>(h[i])));
    }
    ESP_LOGI(tag, "%s", line);
  };

  ESP_LOGI(tag,
           "TMCL read scan summary: GAP ok=%u (motor0, Table57 NR 0…%u minus %u gaps), GGP bank0 "
           "ok=%u, bank2 ok=%u, bank3 ok=%u",
           gap_ok, static_cast<unsigned>(kGapAxisTypeMaxInclusive),
           static_cast<unsigned>(sizeof(kTable57GapAxisNrs) / sizeof(kTable57GapAxisNrs[0])), ggp0_ok,
           ggp2_ok, ggp3_ok);
  dump_hist("GAP", hist_gap);
  dump_hist("GGP b0", hist_ggp0);
  dump_hist("GGP b2", hist_ggp2);
  dump_hist("GGP b3", hist_ggp3);
}

}  // namespace vortex_tmcl_read_probe
