/**
 * @file test_motor_observability.cpp
 * @brief The drive-observability hooks: pipelined batches, telemetry snapshots, fault decoding,
 *        parameter-image apply. Run on the TMC9660 SPI driver over MockSpi with a TMCL slave model.
 *
 * MotorController calls the same helpers (api_helpers/Tmc9660DriveObservability.h) inside
 * Tmc9660Handler::visitDriver(); the handler's bootloader bring-up is not modelled on the host, so the
 * helpers are driven here on the driver directly, and MotorController's own gatekeeping is tested
 * separately (no device, device not up).
 *
 * The model answers SAP/GAP from a parameter store. TMCL over SPI is pipelined: the MISO of a frame is
 * the reply to the previous frame; a NO_OP (0xFF) fetches the last reply. Options mimic FW051V100:
 * axis parameters other than MOTOR_TYPE answer REPLY_WRONG_TYPE until MOTOR_TYPE is set, and a
 * parameter can be made to clamp writes (so read-back differs) or reject them.
 */

#include "HfCoreMocks.h"
#include "HostTest.h"
#include "Tmc9660Handler.h"
#include "managers/MotorController.h"

#include <map>
#include <vector>

using hf::host::MockGpio;
using hf::host::MockSpi;
using hf::host::SpiTransaction;
namespace motor = vortex::motor;
using motor::Param;
using motor::ReplyCode;

namespace {

constexpr uint8_t kSap = 5;
constexpr uint8_t kGap = 6;
constexpr uint8_t kNoOp = 0xFF;
constexpr uint8_t kSpiOk = 0xFF;

uint8_t Checksum(const uint8_t* b, size_t n) {
    uint8_t s = 0;
    for (size_t i = 0; i < n; ++i) s = static_cast<uint8_t>(s + b[i]);
    return s;
}

uint16_t Id(Param p) { return static_cast<uint16_t>(p); }

class TmclModel {
public:
    explicit TmclModel(MockSpi& spi) {
        spi.SetResponder([this](const uint8_t* tx, uint8_t* rx, uint16_t len) { OnFrame(tx, rx, len); });
    }
    std::map<uint16_t, uint32_t> params;         ///< type -> value
    std::map<uint16_t, uint8_t> reject;          ///< type -> TMCL status to answer
    std::map<uint16_t, uint32_t> clamp;          ///< type -> maximum stored value
    bool gate_on_motor_type{false};              ///< FW051V100: WRONG_TYPE until MOTOR_TYPE != 0
    std::vector<uint16_t> sap_log;               ///< SAP order

private:
    void OnFrame(const uint8_t* tx, uint8_t* rx, uint16_t len) {
        if (len != 8) return;
        std::copy(pending_, pending_ + 8, rx);
        if (tx[0] == kNoOp) return;
        const uint8_t op = tx[0];
        const auto type = static_cast<uint16_t>(tx[1] | ((tx[2] & 0xF0u) << 4));
        const uint32_t value = (static_cast<uint32_t>(tx[3]) << 24) | (static_cast<uint32_t>(tx[4]) << 16) |
                               (static_cast<uint32_t>(tx[5]) << 8) | tx[6];
        if (auto it = reject.find(type); it != reject.end()) {
            Fill(it->second, op, 0);
            return;
        }
        if (gate_on_motor_type && type != Id(Param::MOTOR_TYPE) && params[Id(Param::MOTOR_TYPE)] == 0) {
            Fill(static_cast<uint8_t>(ReplyCode::REPLY_WRONG_TYPE), op, 0);
            return;
        }
        if (op == kSap) {
            sap_log.push_back(type);
            uint32_t v = value;
            if (auto c = clamp.find(type); c != clamp.end()) v = std::min(v, c->second);
            params[type] = v;
            Fill(100, op, value);
        } else if (op == kGap) {
            Fill(100, op, params[type]);
        } else {
            Fill(2, op, 0);
        }
    }
    void Fill(uint8_t status, uint8_t op, uint32_t value) {
        pending_[0] = kSpiOk;
        pending_[1] = status;
        pending_[2] = op;
        pending_[3] = static_cast<uint8_t>(value >> 24);
        pending_[4] = static_cast<uint8_t>(value >> 16);
        pending_[5] = static_cast<uint8_t>(value >> 8);
        pending_[6] = static_cast<uint8_t>(value);
        pending_[7] = Checksum(pending_, 7);
    }
    uint8_t pending_[8] = {kSpiOk, 100, 0, 0, 0, 0, 0, 0};
};

struct Rig {
    MockSpi spi;
    TmclModel chip{spi};
    MockGpio rst{1, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT};
    MockGpio drv_en{2, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT};
    MockGpio faultn{3, hf_gpio_direction_t::HF_GPIO_DIRECTION_INPUT, hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW};
    MockGpio wake{4, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT, hf_gpio_active_state_t::HF_GPIO_ACTIVE_LOW};
    HalSpiTmc9660Comm comm{spi, rst, drv_en, faultn, wake};
    Tmc9660Handler::SpiDriver drv{comm, 0x00};

    size_t Frames(uint8_t op) const {
        size_t n = 0;
        for (const SpiTransaction& t : spi.Transactions()) n += (t.tx.size() == 8 && t.tx[0] == op) ? 1 : 0;
        return n;
    }
};

}  // namespace

HF_TEST(Batch_ReadIsPipelined) {
    Rig r;
    r.chip.params[Id(Param::ACTUAL_POSITION)] = 1000;
    r.chip.params[Id(Param::ACTUAL_VELOCITY)] = static_cast<uint32_t>(-25);
    r.chip.params[Id(Param::SUPPLY_VOLTAGE)] = 240;
    std::array<motor::ParamAccess, 3> items{};
    items[0].id = Param::ACTUAL_POSITION;
    items[1].id = Param::ACTUAL_VELOCITY;
    items[2].id = Param::SUPPLY_VOLTAGE;
    CHECK_EQ(motor::BatchRead(r.drv, std::span<motor::ParamAccess>(items)), 3u);
    CHECK_EQ(items[0].value, 1000u);
    CHECK_EQ(static_cast<int32_t>(items[1].value), -25);
    CHECK_EQ(items[2].value, 240u);
    // N commands + one NO_OP to fetch the last reply (not 2N frames).
    CHECK_EQ(r.Frames(kGap), 3u);
    CHECK_EQ(r.Frames(kNoOp), 1u);
}

HF_TEST(Batch_PerElementStatusOnRejection) {
    Rig r;
    r.chip.reject[Id(Param::ACTUAL_VELOCITY)] = static_cast<uint8_t>(ReplyCode::REPLY_WRONG_TYPE);
    std::array<motor::ParamAccess, 2> items{};
    items[0].id = Param::ACTUAL_POSITION;
    items[1].id = Param::ACTUAL_VELOCITY;
    CHECK_EQ(motor::BatchRead(r.drv, std::span<motor::ParamAccess>(items)), 1u);
    CHECK(items[0].ok());
    CHECK(items[1].status == ReplyCode::REPLY_WRONG_TYPE);
}

HF_TEST(Batch_LongBatchesAreChunked) {
    Rig r;
    std::array<motor::ParamAccess, 40> items{};
    for (size_t i = 0; i < items.size(); ++i) {
        items[i].id = Param::IIT_LIMIT_1;
        items[i].value = static_cast<uint32_t>(i);
    }
    CHECK_EQ(motor::BatchWrite(r.drv, std::span<motor::ParamAccess>(items)), items.size());
    CHECK_EQ(r.chip.params[Id(Param::IIT_LIMIT_1)], 39u);
    CHECK_EQ(r.Frames(kSap), 40u);
}

HF_TEST(Telemetry_CyclicSetAlternatesTheErrorWords) {
    Rig r;
    r.chip.params[Id(Param::TORQUE_FLUX_COMBINED_ACTUAL_VALUES)] = (0x0123u << 16) | 0xFFFEu;
    r.chip.params[Id(Param::GENERAL_ERROR_FLAGS)] = 0x200;
    r.chip.params[Id(Param::GDRV_ERROR_FLAGS)] = 0x1;
    motor::MotorTelemetrySnapshot snap{};

    CHECK_EQ(motor::ReadTelemetry(r.drv, motor::TelemetrySet::Cyclic, snap), 5u);  // seq 1: 299
    CHECK_EQ(snap.sequence, 1u);
    CHECK_EQ(snap.accesses, 5u);
    CHECK(snap.Valid(motor::TelemetryField::GeneralErrors));
    CHECK(!snap.Valid(motor::TelemetryField::GateDriverErrors));
    CHECK_EQ(snap.TorqueCurrentRaw(), 0x0123);
    CHECK_EQ(snap.FluxCurrentRaw(), -2);

    CHECK_EQ(motor::ReadTelemetry(r.drv, motor::TelemetrySet::Cyclic, snap), 5u);  // seq 2: 300
    CHECK(snap.Valid(motor::TelemetryField::GateDriverErrors));
    CHECK_EQ(snap.Raw(motor::TelemetryField::GateDriverErrors), 1u);
    CHECK_EQ(snap.Age(motor::TelemetryField::GeneralErrors), 1u);    // read one call ago
    CHECK_EQ(snap.Age(motor::TelemetryField::ActualPosition), 0u);
    CHECK_EQ(snap.Age(motor::TelemetryField::SupplyVoltage), UINT32_MAX);  // never read
}

HF_TEST(Telemetry_FailedFieldKeepsItsValueAndAges) {
    Rig r;
    r.chip.params[Id(Param::SUPPLY_VOLTAGE)] = 245;
    r.chip.params[Id(Param::CHIP_TEMPERATURE)] = 18000;
    motor::MotorTelemetrySnapshot snap{};
    CHECK_EQ(motor::ReadTelemetry(r.drv, motor::TelemetrySet::Status, snap), 10u);
    CHECK_NEAR(snap.SupplyVoltageV(), 24.5, 1e-4);
    CHECK_NEAR(snap.ChipTemperatureC(), 18000 * 0.01615 - 268.15, 1e-2);

    r.chip.reject[Id(Param::SUPPLY_VOLTAGE)] = static_cast<uint8_t>(ReplyCode::REPLY_INVALID_VALUE);
    r.chip.params[Id(Param::SUPPLY_VOLTAGE)] = 100;
    CHECK_EQ(motor::ReadTelemetry(r.drv, motor::TelemetrySet::Status, snap), 9u);
    CHECK_EQ(snap.failed, 1u);
    CHECK(!snap.Valid(motor::TelemetryField::SupplyVoltage));
    CHECK_NEAR(snap.SupplyVoltageV(), 24.5, 1e-4);  // stale value kept, not overwritten
    CHECK_EQ(snap.Age(motor::TelemetryField::SupplyVoltage), 1u);
}

HF_TEST(Telemetry_FullSetReadsEveryFieldInOneBatch) {
    Rig r;
    motor::MotorTelemetrySnapshot snap{};
    CHECK_EQ(motor::ReadTelemetry(r.drv, motor::TelemetrySet::Full, snap), motor::kTelemetryFieldCount);
    CHECK_EQ(r.Frames(kNoOp), 1u);
}

HF_TEST(Diagnostics_DecodeNamesTheConditions) {
    using tmc9660::tmcl::GeneralErrorFlags;
    using tmc9660::tmcl::GeneralStatusFlags;
    const uint32_t status = static_cast<uint32_t>(GeneralStatusFlags::REGULATION_STOPPED) |
                            static_cast<uint32_t>(GeneralStatusFlags::ADC_OFFSET_CALIBRATED);
    const uint32_t errors = static_cast<uint32_t>(GeneralErrorFlags::WATCHDOG_EVENT) |
                            static_cast<uint32_t>(GeneralErrorFlags::SUPPLY_UNDERVOLTAGE_WARNING);
    motor::MotorDriveDiagnostics d = motor::DecodeDriveStatus(status, errors, 0, 0);
    CHECK(d.regulation_stopped);
    CHECK(d.adc_offset_calibrated);
    CHECK(d.chip_reset_detected);
    CHECK(d.supply_undervoltage_warning);
    CHECK(d.severity == motor::DriveSeverity::Warning);

    // V high-side overcurrent (b17) and U low-side charge short (b8): a fault, located per phase.
    d = motor::DecodeDriveStatus(0, 0, (1u << 17) | (1u << 8), 0);
    CHECK_EQ(d.overcurrent_high_side, 0x2u);
    CHECK_EQ(d.vgs_short_low_side, 0x1u);
    CHECK(d.severity == motor::DriveSeverity::Fault);

    d = motor::DecodeDriveStatus(0, 0, 0, 0);
    CHECK(d.severity == motor::DriveSeverity::Ok);
}

HF_TEST(Diagnostics_ReadAndClearLatches) {
    Rig r;
    r.chip.params[Id(Param::GENERAL_ERROR_FLAGS)] = 1u << 9;
    motor::MotorDriveDiagnostics d{};
    CHECK(motor::ReadDriveDiagnostics(r.drv, d));
    CHECK_EQ(d.words_valid, 0xFu);
    CHECK(d.chip_reset_detected);
    CHECK(motor::ClearLatchedFlags(r.drv, 1u << 9, 0, 0));
    CHECK_EQ(r.chip.sap_log.size(), 1u);
    CHECK_EQ(r.chip.sap_log[0], Id(Param::GENERAL_ERROR_FLAGS));
}

HF_TEST(Image_MotorTypeIsWrittenFirst) {
    Rig r;
    r.chip.gate_on_motor_type = true;  // FW051V100: nothing else answers until MOTOR_TYPE is set
    const std::array<motor::ParamWrite, 3> image{{
        {Param::MOTOR_POLE_PAIRS, 7},
        {Param::MOTOR_TYPE, 3},
        {Param::IIT_LIMIT_1, 5000},
    }};
    motor::ApplyResult res = motor::ApplyParameterImage(r.drv, std::span<const motor::ParamWrite>(image),
                                                        motor::ApplyOptions{false, true});
    CHECK(res.ok());
    CHECK_EQ(res.written, 3u);
    REQUIRE(!r.chip.sap_log.empty());
    CHECK_EQ(r.chip.sap_log.front(), Id(Param::MOTOR_TYPE));
    CHECK_EQ(r.chip.params[Id(Param::MOTOR_POLE_PAIRS)], 7u);
}

HF_TEST(Image_DifferentialSkipsEqualEntries) {
    Rig r;
    r.chip.params[Id(Param::MOTOR_POLE_PAIRS)] = 7;
    const std::array<motor::ParamWrite, 2> image{{{Param::MOTOR_POLE_PAIRS, 7}, {Param::IIT_LIMIT_1, 5000}}};
    motor::ApplyResult res = motor::ApplyParameterImage(r.drv, std::span<const motor::ParamWrite>(image),
                                                        motor::ApplyOptions{true, true});
    CHECK(res.ok());
    CHECK_EQ(res.skipped, 1u);
    CHECK_EQ(res.written, 1u);
    REQUIRE(r.chip.sap_log.size() == 1u);
    CHECK_EQ(r.chip.sap_log[0], Id(Param::IIT_LIMIT_1));
}

HF_TEST(Image_VerifyReportsMismatchAndRejection) {
    Rig r;
    r.chip.clamp[Id(Param::IIT_LIMIT_1)] = 1000;  // the chip stores less than asked
    r.chip.reject[Id(Param::MOTOR_POLE_PAIRS)] = static_cast<uint8_t>(ReplyCode::REPLY_INVALID_VALUE);
    const std::array<motor::ParamWrite, 2> image{{{Param::IIT_LIMIT_1, 5000}, {Param::MOTOR_POLE_PAIRS, 7}}};
    motor::ApplyResult res = motor::ApplyParameterImage(r.drv, std::span<const motor::ParamWrite>(image),
                                                        motor::ApplyOptions{false, true});
    CHECK(!res.ok());
    CHECK_EQ(res.rejected, 1u);
    CHECK(res.mismatches >= 1u);
    CHECK(res.first_bad == Param::MOTOR_POLE_PAIRS);  // the first problem: the rejected write
}

HF_TEST(MotorController_ObservabilityRefusesWithoutAReadyDevice) {
    auto& mc = MotorController::GetInstance();
    motor::MotorTelemetrySnapshot snap{};
    CHECK(mc.ReadTelemetry(motor::TelemetrySet::Cyclic, snap) == MotorError::DEVICE_NOT_FOUND);
    CHECK_EQ(snap.sequence, 0u);

    // A registered device whose bring-up never ran is not driven (no bootloader from an observability call).
    Rig r;
    Tmc9660ControlPins pins{r.rst, r.drv_en, r.faultn, r.wake};
    REQUIRE(mc.CreateOnboardDevice(r.spi, 1, pins, nullptr) == MotorError::SUCCESS);
    r.spi.ClearTransactions();
    CHECK(mc.ReadTelemetry(motor::TelemetrySet::Cyclic, snap) == MotorError::NOT_INITIALIZED);
    std::array<motor::ParamAccess, 1> items{};
    CHECK(mc.ReadParameters(std::span<motor::ParamAccess>(items)) == MotorError::NOT_INITIALIZED);
    motor::MotorDriveDiagnostics d{};
    CHECK(mc.ReadDriveDiagnostics(d) == MotorError::NOT_INITIALIZED);
    CHECK(r.spi.Transactions().empty());
    MotorAccessStats stats{};
    CHECK(mc.GetAccessStats(stats) == MotorError::SUCCESS);
    CHECK_EQ(stats.batches, 0u);
    (void)mc.Deinitialize();
}
