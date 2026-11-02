/**
 * @file Tmc9660DriveObservability.h
 * @brief TMC9660 telemetry snapshots, fault decoding and parameter-image apply for MotorController.
 *
 * @details The hooks the drive-observability design (hardfoc-vortex-v1 docs/design/drive-observability.md)
 *          needs from the HAL, independent of any protocol stack:
 *
 *          - **Batch access.** Every read and write goes through the driver's pipelined TMCL batch API
 *            (`readParameters()` / `writeParameters()`): on SPI, N accesses cost N + 1 datagrams instead of
 *            2N (design §1.8).
 *          - **Telemetry snapshots** (§1.3, §2.2, §2.3): a caller-owned MotorTelemetrySnapshot is refreshed in
 *            place from one batch. Every field remembers the sequence number of its last good read, so a
 *            consumer can tell its age; a failed access leaves the old value and does not move its stamp.
 *            The cyclic set is the design's fixed slots: position and velocity actual, packed Iq/Id (331),
 *            status (289), and the general / gate-driver error words alternating (299 / 300).
 *          - **Fault decoding** (§1.5): the four status words become named conditions and a severity.
 *          - **Configuration** (§4.3–§4.5): a parameter image is applied with MOTOR_TYPE first (the chip
 *            rejects most SAPs until it is set), optionally only where the chip differs (differential apply),
 *            and optionally read back and compared (verify).
 *
 *          Everything here is templated on the driver type (`Tmc9660Handler::SpiDriver` or `UartDriver`) and
 *          is called by MotorController inside `Tmc9660Handler::visitDriver()`, so the handler mutex is held.
 *          Units are the chip's raw units; the SI conversions given are those the design marks as solid.
 *          The packed 331/332 values are unscaled until calibrated (§1.4).
 *
 *          Ownership: value types and free functions. Thread safety: none of its own (callers hold the
 *          handler lock). No allocation: batches are chunked through a fixed stack buffer. MCU-free.
 */

#ifndef VORTEX_TMC9660_DRIVE_OBSERVABILITY_H_
#define VORTEX_TMC9660_DRIVE_OBSERVABILITY_H_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "RtosMutex.h"  // RtosTime::GetCurrentTimeUs()
#include "core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp"

namespace vortex::motor {

using Param = tmc9660::tmcl::Parameters;
using ReplyCode = tmc9660::tmcl::ReplyCode;

//==============================================================================
// Batch access
//==============================================================================

/** @brief One axis-parameter access in a batch (MCU- and transport-neutral copy of the driver's type). */
struct ParamAccess {
    Param id{};                                   ///< Axis parameter
    uint32_t value{0};                            ///< Write: value to set. Read: value read back
    uint8_t motor{0};                             ///< Axis (0 on the TMC9660)
    ReplyCode status{ReplyCode::REPLY_CHKERR};    ///< TMCL status of this element
    [[nodiscard]] bool ok() const noexcept { return status == ReplyCode::REPLY_OK; }
};

/** @brief Elements per pipelined chunk (bounds the stack buffer; the driver chunks again internally). */
inline constexpr size_t kBatchChunk = 16;

namespace detail {

template <typename Driver, bool kWrite>
size_t RunBatch(Driver& driver, std::span<ParamAccess> items) noexcept {
    using DriverAccess = typename Driver::ParameterAccess;
    std::array<DriverAccess, kBatchChunk> buf{};
    size_t ok = 0;
    for (size_t base = 0; base < items.size(); base += kBatchChunk) {
        const size_t n = std::min(kBatchChunk, items.size() - base);
        for (size_t i = 0; i < n; ++i) {
            buf[i] = DriverAccess{};
            buf[i].id = items[base + i].id;
            buf[i].value = items[base + i].value;
            buf[i].motor = items[base + i].motor;
        }
        const std::span<DriverAccess> chunk(buf.data(), n);
        if constexpr (kWrite) {
            (void)driver.writeParameters(chunk);
        } else {
            (void)driver.readParameters(chunk);
        }
        for (size_t i = 0; i < n; ++i) {
            ParamAccess& out = items[base + i];
            out.status = buf[i].status;
            if constexpr (!kWrite) {
                if (buf[i].ok()) {
                    out.value = buf[i].value;
                }
            }
            ok += out.ok() ? 1 : 0;
        }
    }
    return ok;
}

}  // namespace detail

/** @brief Read a batch (GAP, pipelined). @return number of elements answered REPLY_OK. */
template <typename Driver>
size_t BatchRead(Driver& driver, std::span<ParamAccess> items) noexcept {
    return detail::RunBatch<Driver, false>(driver, items);
}

/** @brief Write a batch (SAP, pipelined; SAP is idempotent). @return number of elements answered REPLY_OK. */
template <typename Driver>
size_t BatchWrite(Driver& driver, std::span<ParamAccess> items) noexcept {
    return detail::RunBatch<Driver, true>(driver, items);
}

//==============================================================================
// Telemetry snapshot
//==============================================================================

/** @brief The telemetry signals a snapshot carries (design §1.3, tiers C/F/S). */
enum class TelemetryField : uint8_t {
    ActualPosition = 0,     ///< A:144, counts
    ActualVelocity,         ///< A:125, chip velocity units
    TorqueFluxActual,       ///< A:331, packed Iq (high 16) / Id (low 16), unscaled
    GeneralStatus,          ///< A:289
    GeneralErrors,          ///< A:299 (write-1-to-clear latches)
    GateDriverErrors,       ///< A:300
    AdcStatus,              ///< A:301
    SupplyVoltage,          ///< A:290, 0.1 V
    ChipTemperature,        ///< A:296, raw (°C = v·0.01615 − 268.15)
    ExternalTemperature,    ///< A:293, raw AIN3 (NTC curve on the host)
    TotalMotorCurrent,      ///< A:231, mA
    RampVelocity,           ///< A:69
    RampPosition,           ///< A:70
    VoltageDqActual,        ///< A:332, packed Ud/Uq, unscaled
    Iit1Sum,                ///< A:226, A²·ms
    Iit2Sum,                ///< A:229, A²·ms
    Count
};

inline constexpr size_t kTelemetryFieldCount = static_cast<size_t>(TelemetryField::Count);

/** @brief TMC9660 parameter behind each telemetry field. */
inline constexpr std::array<Param, kTelemetryFieldCount> kTelemetryParam{
    Param::ACTUAL_POSITION,     Param::ACTUAL_VELOCITY,
    Param::TORQUE_FLUX_COMBINED_ACTUAL_VALUES,
    Param::GENERAL_STATUS_FLAGS, Param::GENERAL_ERROR_FLAGS, Param::GDRV_ERROR_FLAGS, Param::ADC_STATUS_FLAGS,
    Param::SUPPLY_VOLTAGE,      Param::CHIP_TEMPERATURE,    Param::EXTERNAL_TEMPERATURE,
    Param::ACTUAL_TOTAL_MOTOR_CURRENT,
    Param::RAMP_VELOCITY,       Param::RAMP_POSITION,
    Param::VOLTAGE_D_Q_COMBINED_ACTUAL_VALUES,
    Param::IIT_SUM_1,           Param::IIT_SUM_2,
};

/** @brief Which signals one ReadTelemetry() call fetches. */
enum class TelemetrySet : uint8_t {
    /// The cyclic fixed slots (§2.2): 144, 125, 331, 289, and 299 / 300 on alternate calls (5 accesses).
    Cyclic = 0,
    /// Status, error and power/thermal words: 289, 299, 300, 301, 290, 296, 293, 231, 226, 229.
    Status,
    /// Every field.
    Full,
};

/**
 * @brief Caller-owned snapshot, refreshed in place by MotorController::ReadTelemetry().
 * @details Values are raw chip units. `field_seq[f]` is the value of `sequence` when field f was last read
 *          successfully (0 = never), so `Age(f)` is how many reads ago it was.
 */
struct MotorTelemetrySnapshot {
    std::array<uint32_t, kTelemetryFieldCount> raw{};        ///< Last good value per field
    std::array<uint32_t, kTelemetryFieldCount> field_seq{};  ///< Sequence of the last good read (0 = never)
    uint32_t sequence{0};            ///< Incremented by every ReadTelemetry() call
    uint32_t valid_mask{0};          ///< Bit f set if field f was read OK in the latest call
    uint32_t requested_mask{0};      ///< Bit f set if the latest call asked for field f
    uint64_t timestamp_us{0};        ///< Monotonic time at the end of the latest batch
    uint32_t batch_duration_us{0};   ///< Time the latest batch took (all accesses)
    uint8_t accesses{0};             ///< TMCL accesses in the latest batch
    uint8_t failed{0};               ///< ... of which were not answered REPLY_OK

    [[nodiscard]] uint32_t Raw(TelemetryField f) const noexcept { return raw[static_cast<size_t>(f)]; }
    [[nodiscard]] bool Valid(TelemetryField f) const noexcept {
        return (valid_mask >> static_cast<unsigned>(f)) & 1U;
    }
    /** @brief Reads since field @p f was last good (0 = in the latest call); UINT32_MAX if never read. */
    [[nodiscard]] uint32_t Age(TelemetryField f) const noexcept {
        const uint32_t s = field_seq[static_cast<size_t>(f)];
        return s == 0 ? UINT32_MAX : sequence - s;
    }

    [[nodiscard]] int32_t ActualPosition() const noexcept {
        return static_cast<int32_t>(Raw(TelemetryField::ActualPosition));
    }
    [[nodiscard]] int32_t ActualVelocity() const noexcept {
        return static_cast<int32_t>(Raw(TelemetryField::ActualVelocity));
    }
    /** @brief Iq from the packed 331 word (high 16 bits), unscaled until calibrated (design §1.4). */
    [[nodiscard]] int16_t TorqueCurrentRaw() const noexcept {
        return static_cast<int16_t>(Raw(TelemetryField::TorqueFluxActual) >> 16);
    }
    /** @brief Id from the packed 331 word (low 16 bits), unscaled until calibrated. */
    [[nodiscard]] int16_t FluxCurrentRaw() const noexcept {
        return static_cast<int16_t>(Raw(TelemetryField::TorqueFluxActual) & 0xFFFFU);
    }
    [[nodiscard]] float SupplyVoltageV() const noexcept {
        return static_cast<float>(Raw(TelemetryField::SupplyVoltage)) * 0.1f;
    }
    [[nodiscard]] float ChipTemperatureC() const noexcept {
        return static_cast<float>(Raw(TelemetryField::ChipTemperature)) * 0.01615f - 268.15f;
    }
    [[nodiscard]] int32_t TotalMotorCurrentMa() const noexcept {
        return static_cast<int32_t>(Raw(TelemetryField::TotalMotorCurrent));
    }
};

/** @brief Fields a set asks for, given the snapshot's next sequence number (Cyclic alternates 299/300). */
[[nodiscard]] constexpr uint32_t TelemetryRequestMask(TelemetrySet set, uint32_t next_sequence) noexcept {
    auto bit = [](TelemetryField f) constexpr { return 1U << static_cast<unsigned>(f); };
    switch (set) {
        case TelemetrySet::Cyclic:
            return bit(TelemetryField::ActualPosition) | bit(TelemetryField::ActualVelocity) |
                   bit(TelemetryField::TorqueFluxActual) | bit(TelemetryField::GeneralStatus) |
                   ((next_sequence & 1U) ? bit(TelemetryField::GeneralErrors)
                                         : bit(TelemetryField::GateDriverErrors));
        case TelemetrySet::Status:
            return bit(TelemetryField::GeneralStatus) | bit(TelemetryField::GeneralErrors) |
                   bit(TelemetryField::GateDriverErrors) | bit(TelemetryField::AdcStatus) |
                   bit(TelemetryField::SupplyVoltage) | bit(TelemetryField::ChipTemperature) |
                   bit(TelemetryField::ExternalTemperature) | bit(TelemetryField::TotalMotorCurrent) |
                   bit(TelemetryField::Iit1Sum) | bit(TelemetryField::Iit2Sum);
        default:
            return (1U << kTelemetryFieldCount) - 1U;
    }
}

/**
 * @brief Refresh @p snap from one pipelined batch.
 * @return Number of fields read OK.
 */
template <typename Driver>
size_t ReadTelemetry(Driver& driver, TelemetrySet set, MotorTelemetrySnapshot& snap) noexcept {
    const uint32_t seq = snap.sequence + 1;
    const uint32_t mask = TelemetryRequestMask(set, seq);
    std::array<ParamAccess, kTelemetryFieldCount> items{};
    std::array<uint8_t, kTelemetryFieldCount> field_of{};
    size_t n = 0;
    for (size_t f = 0; f < kTelemetryFieldCount; ++f) {
        if ((mask >> f) & 1U) {
            items[n].id = kTelemetryParam[f];
            field_of[n] = static_cast<uint8_t>(f);
            ++n;
        }
    }
    const uint64_t t0 = RtosTime::GetCurrentTimeUs();
    const size_t ok = BatchRead(driver, std::span<ParamAccess>(items.data(), n));
    const uint64_t t1 = RtosTime::GetCurrentTimeUs();

    snap.sequence = seq;
    snap.requested_mask = mask;
    snap.valid_mask = 0;
    for (size_t i = 0; i < n; ++i) {
        if (items[i].ok()) {
            const size_t f = field_of[i];
            snap.raw[f] = items[i].value;
            snap.field_seq[f] = seq;
            snap.valid_mask |= 1U << f;
        }
    }
    snap.timestamp_us = t1;
    snap.batch_duration_us = static_cast<uint32_t>(t1 - t0);
    snap.accesses = static_cast<uint8_t>(n);
    snap.failed = static_cast<uint8_t>(n - ok);
    return ok;
}

//==============================================================================
// Fault decoding
//==============================================================================

/** @brief How serious the decoded status words are. */
enum class DriveSeverity : uint8_t {
    Ok = 0,      ///< Nothing set that needs attention
    Warning,     ///< Thresholds crossed or events latched; the bridge may still run
    Fault,       ///< The chip stopped (or must stop) driving: gate-driver fault, retries failed, overtemperature
};

/** @brief The four TMC9660 status words, decoded (design §1.5). Raw words are kept for anything not named. */
struct MotorDriveDiagnostics {
    uint32_t general_status{0};       ///< A:289
    uint32_t general_errors{0};       ///< A:299
    uint32_t gate_driver_errors{0};   ///< A:300
    uint32_t adc_status{0};           ///< A:301
    uint8_t words_valid{0};           ///< Bit 0..3 = 289/299/300/301 read OK

    // General status (289)
    bool regulation_stopped{false};   ///< b0: not regulating (the fault handler reverts to SYSTEM_OFF)
    bool adc_offset_calibrated{false};///< b11
    bool iit1_active{false};          ///< b18
    bool iit2_active{false};          ///< b19
    bool config_loaded{false};        ///< b5: config loaded from TMC NVM (not used on Vortex; unexpected)

    // General errors (299)
    bool chip_reset_detected{false};  ///< b9 watchdog event: the TMC9660 reset; re-apply the parameter image
    bool hall_invalid{false};         ///< b5
    bool ext_overtemperature{false};  ///< b13
    bool chip_overtemperature{false}; ///< b14
    bool iit1_exceeded{false};        ///< b16
    bool iit2_exceeded{false};        ///< b17
    bool ext_temperature_warning{false};   ///< b18
    bool supply_overvoltage_warning{false};///< b19
    bool supply_undervoltage_warning{false};///< b20
    bool adc_input_overvoltage{false};     ///< b21
    bool fault_retry_happened{false};      ///< b22
    bool fault_retries_failed{false};      ///< b23
    bool chip_temperature_warning{false};  ///< b24
    bool heartbeat_stopped{false};         ///< b26: the independent stop path acted

    // Gate driver errors (300), grouped by kind; bit order per kind is U, V, W, Y2
    uint8_t overcurrent_low_side{0};   ///< b0..3
    uint8_t overcurrent_high_side{0};  ///< b16..19
    uint8_t vgs_short_low_side{0};     ///< b4..11 (discharge b4..7 | charge b8..11), folded per phase
    uint8_t vgs_short_high_side{0};    ///< b20..27, folded per phase
    uint8_t bootstrap_undervoltage{0}; ///< b12..15
    bool gate_driver_undervoltage{false}; ///< b29..31 (UV, low voltage, supply UV)

    // ADC (301)
    uint16_t adc_clipped{0};          ///< b0..13: I0..I3, U0..U3, AIN0..AIN3, VM, TEMP

    DriveSeverity severity{DriveSeverity::Ok};

    // Link
    uint32_t batch_duration_us{0};    ///< Time the status batch took
    bool spi_stats_valid{false};      ///< spi_stats filled (SPI transport only)
    tmc9660::TmclSpiStats spi_stats{};///< TMCL SPI adapter counters (retries, NOT_READY, transport errors)
};

/** @brief Decode the four status words into named conditions and a severity. */
[[nodiscard]] inline MotorDriveDiagnostics DecodeDriveStatus(uint32_t status, uint32_t gen_err, uint32_t gdrv,
                                                             uint32_t adc) noexcept {
    using tmc9660::tmcl::GeneralErrorFlags;
    using tmc9660::tmcl::GeneralStatusFlags;
    auto s = [status](GeneralStatusFlags f) { return (status & static_cast<uint32_t>(f)) != 0; };
    auto e = [gen_err](GeneralErrorFlags f) { return (gen_err & static_cast<uint32_t>(f)) != 0; };
    auto nib = [](uint32_t w, unsigned shift) { return static_cast<uint8_t>((w >> shift) & 0xFU); };

    MotorDriveDiagnostics d{};
    d.general_status = status;
    d.general_errors = gen_err;
    d.gate_driver_errors = gdrv;
    d.adc_status = adc;

    d.regulation_stopped = s(GeneralStatusFlags::REGULATION_STOPPED);
    d.adc_offset_calibrated = s(GeneralStatusFlags::ADC_OFFSET_CALIBRATED);
    d.iit1_active = s(GeneralStatusFlags::IIT_1_ACTIVE);
    d.iit2_active = s(GeneralStatusFlags::IIT_2_ACTIVE);
    d.config_loaded = s(GeneralStatusFlags::CONFIG_LOADED);

    d.chip_reset_detected = e(GeneralErrorFlags::WATCHDOG_EVENT);
    d.hall_invalid = e(GeneralErrorFlags::HALL_ERROR);
    d.ext_overtemperature = e(GeneralErrorFlags::EXT_TEMP_EXCEEDED);
    d.chip_overtemperature = e(GeneralErrorFlags::CHIP_TEMP_EXCEEDED);
    d.iit1_exceeded = e(GeneralErrorFlags::I2T_1_EXCEEDED);
    d.iit2_exceeded = e(GeneralErrorFlags::I2T_2_EXCEEDED);
    d.ext_temperature_warning = e(GeneralErrorFlags::EXT_TEMP_WARNING);
    d.supply_overvoltage_warning = e(GeneralErrorFlags::SUPPLY_OVERVOLTAGE_WARNING);
    d.supply_undervoltage_warning = e(GeneralErrorFlags::SUPPLY_UNDERVOLTAGE_WARNING);
    d.adc_input_overvoltage = e(GeneralErrorFlags::ADC_IN_OVERVOLTAGE);
    d.fault_retry_happened = e(GeneralErrorFlags::FAULT_RETRY_HAPPENED);
    d.fault_retries_failed = e(GeneralErrorFlags::FAULT_RETRIES_FAILED);
    d.chip_temperature_warning = e(GeneralErrorFlags::CHIP_TEMP_WARNING);
    d.heartbeat_stopped = e(GeneralErrorFlags::HEARTBEAT_STOPPED);

    d.overcurrent_low_side = nib(gdrv, 0);
    d.vgs_short_low_side = static_cast<uint8_t>(nib(gdrv, 4) | nib(gdrv, 8));
    d.bootstrap_undervoltage = nib(gdrv, 12);
    d.overcurrent_high_side = nib(gdrv, 16);
    d.vgs_short_high_side = static_cast<uint8_t>(nib(gdrv, 20) | nib(gdrv, 24));
    d.gate_driver_undervoltage = (gdrv & 0xE0000000U) != 0;

    d.adc_clipped = static_cast<uint16_t>(adc & 0x3FFFU);

    const bool fault = gdrv != 0 || d.fault_retries_failed || d.ext_overtemperature || d.chip_overtemperature ||
                       d.heartbeat_stopped || d.iit1_exceeded || d.iit2_exceeded;
    const bool warning = d.chip_reset_detected || d.hall_invalid || d.ext_temperature_warning ||
                         d.supply_overvoltage_warning || d.supply_undervoltage_warning || d.adc_input_overvoltage ||
                         d.fault_retry_happened || d.chip_temperature_warning || d.config_loaded ||
                         d.adc_clipped != 0;
    d.severity = fault ? DriveSeverity::Fault : (warning ? DriveSeverity::Warning : DriveSeverity::Ok);
    return d;
}

/** @brief Read the four status words in one batch and decode them. @return false if a word failed. */
template <typename Driver>
bool ReadDriveDiagnostics(Driver& driver, MotorDriveDiagnostics& out) noexcept {
    std::array<ParamAccess, 4> items{};
    items[0].id = Param::GENERAL_STATUS_FLAGS;
    items[1].id = Param::GENERAL_ERROR_FLAGS;
    items[2].id = Param::GDRV_ERROR_FLAGS;
    items[3].id = Param::ADC_STATUS_FLAGS;
    const uint64_t t0 = RtosTime::GetCurrentTimeUs();
    (void)BatchRead(driver, std::span<ParamAccess>(items));
    const uint64_t t1 = RtosTime::GetCurrentTimeUs();
    out = DecodeDriveStatus(items[0].value, items[1].value, items[2].value, items[3].value);
    out.words_valid = 0;
    for (size_t i = 0; i < items.size(); ++i) {
        out.words_valid = static_cast<uint8_t>(out.words_valid | (items[i].ok() ? (1U << i) : 0U));
    }
    out.batch_duration_us = static_cast<uint32_t>(t1 - t0);
    return out.words_valid == 0xFU;
}

/**
 * @brief Clear latched (write-1-to-clear) bits of the error words in one batch.
 * @param general_errors Bits of A:299 to clear (0 = leave the word alone)
 * @param gate_driver_errors Bits of A:300 to clear
 * @param adc_status Bits of A:301 to clear
 * @return true if every write was accepted.
 */
template <typename Driver>
bool ClearLatchedFlags(Driver& driver, uint32_t general_errors, uint32_t gate_driver_errors,
                       uint32_t adc_status) noexcept {
    std::array<ParamAccess, 3> items{};
    size_t n = 0;
    if (general_errors != 0) items[n++] = ParamAccess{Param::GENERAL_ERROR_FLAGS, general_errors, 0, {}};
    if (gate_driver_errors != 0) items[n++] = ParamAccess{Param::GDRV_ERROR_FLAGS, gate_driver_errors, 0, {}};
    if (adc_status != 0) items[n++] = ParamAccess{Param::ADC_STATUS_FLAGS, adc_status, 0, {}};
    return BatchWrite(driver, std::span<ParamAccess>(items.data(), n)) == n;
}

//==============================================================================
// Configuration: parameter image apply
//==============================================================================

/** @brief One entry of a parameter image (design §4.1, the TMC image). */
struct ParamWrite {
    Param id{};
    uint32_t value{0};
};

/** @brief How ApplyParameterImage() works. */
struct ApplyOptions {
    bool differential{true};  ///< Read the chip first and write only entries whose value differs (§4.5)
    bool verify{true};        ///< Read every entry back after writing and compare (§4.4)
};

/** @brief What ApplyParameterImage() did. */
struct ApplyResult {
    uint16_t entries{0};          ///< Entries in the image
    uint16_t written{0};          ///< SAPs sent (after the differential skip)
    uint16_t skipped{0};          ///< Already equal on the chip
    uint16_t rejected{0};         ///< SAPs the chip refused (status != REPLY_OK)
    uint16_t mismatches{0};       ///< Read back different from the image
    Param first_bad{};            ///< First rejected or mismatched parameter
    ReplyCode first_bad_status{ReplyCode::REPLY_OK};
    uint32_t first_bad_readback{0};
    [[nodiscard]] bool ok() const noexcept { return rejected == 0 && mismatches == 0; }
};

namespace detail {

/**
 * @brief Apply the image entries selected by @p take, in image order, kBatchChunk at a time.
 * @details Per chunk: optional differential read, one pipelined write of the entries that differ, optional
 *          read-back. Stack use is three chunk buffers whatever the image size.
 */
template <typename Driver, typename Pred>
void ApplyPhase(Driver& driver, std::span<const ParamWrite> image, ApplyOptions opt, Pred take,
                ApplyResult& r) noexcept {
    auto note_bad = [&r](Param id, ReplyCode status, uint32_t readback) {
        if (r.rejected + r.mismatches == 0) {
            r.first_bad = id;
            r.first_bad_status = status;
            r.first_bad_readback = readback;
        }
    };
    std::array<ParamAccess, kBatchChunk> chunk{};
    size_t i = 0;
    while (i < image.size()) {
        // Gather the next chunk of selected entries.
        size_t n = 0;
        while (i < image.size() && n < kBatchChunk) {
            if (take(image[i])) {
                chunk[n++] = ParamAccess{image[i].id, image[i].value, 0, ReplyCode::REPLY_CHKERR};
            }
            ++i;
        }
        if (n == 0) {
            continue;
        }
        // Differential: drop entries the chip already holds.
        std::array<ParamAccess, kBatchChunk> to_write{};
        size_t w = 0;
        if (opt.differential) {
            std::array<ParamAccess, kBatchChunk> current{};
            for (size_t j = 0; j < n; ++j) current[j].id = chunk[j].id;
            (void)BatchRead(driver, std::span<ParamAccess>(current.data(), n));
            for (size_t j = 0; j < n; ++j) {
                if (current[j].ok() && current[j].value == chunk[j].value) {
                    ++r.skipped;
                } else {
                    to_write[w++] = chunk[j];
                }
            }
        } else {
            for (size_t j = 0; j < n; ++j) to_write[w++] = chunk[j];
        }
        (void)BatchWrite(driver, std::span<ParamAccess>(to_write.data(), w));
        r.written = static_cast<uint16_t>(r.written + w);
        for (size_t j = 0; j < w; ++j) {
            if (!to_write[j].ok()) {
                note_bad(to_write[j].id, to_write[j].status, 0);
                ++r.rejected;
            }
        }
        // Verify: read the whole chunk back.
        if (opt.verify) {
            for (size_t j = 0; j < n; ++j) {
                to_write[j] = ParamAccess{chunk[j].id, 0, 0, ReplyCode::REPLY_CHKERR};
            }
            (void)BatchRead(driver, std::span<ParamAccess>(to_write.data(), n));
            for (size_t j = 0; j < n; ++j) {
                if (!to_write[j].ok() || to_write[j].value != chunk[j].value) {
                    note_bad(chunk[j].id, to_write[j].status, to_write[j].value);
                    ++r.mismatches;
                }
            }
        }
    }
}

}  // namespace detail

/**
 * @brief Apply a parameter image: MOTOR_TYPE first, then the rest in image order.
 * @details Before MOTOR_TYPE is set, FW051V100 answers most axis parameters with REPLY_WRONG_TYPE (design
 *          §1.10), so it is written (and verified) before anything else. The rest goes kBatchChunk entries
 *          per pipelined batch. No allocation and a fixed stack footprint, whatever the image size.
 * @return The counts; ApplyResult::ok() when nothing was rejected or mismatched.
 */
template <typename Driver>
ApplyResult ApplyParameterImage(Driver& driver, std::span<const ParamWrite> image, ApplyOptions opt) noexcept {
    ApplyResult r{};
    r.entries = static_cast<uint16_t>(std::min<size_t>(image.size(), UINT16_MAX));
    detail::ApplyPhase(driver, image, opt, [](const ParamWrite& e) { return e.id == Param::MOTOR_TYPE; }, r);
    detail::ApplyPhase(driver, image, opt, [](const ParamWrite& e) { return e.id != Param::MOTOR_TYPE; }, r);
    return r;
}

}  // namespace vortex::motor

#endif  // VORTEX_TMC9660_DRIVE_OBSERVABILITY_H_
