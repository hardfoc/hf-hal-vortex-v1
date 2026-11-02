/**
 * @file test_encoder_manager.cpp
 * @brief EncoderManager on an AS5047U register model over MockSpi.
 *
 * The model follows the AS5047U framing the driver uses: the MISO of a frame answers the previous
 * command; 24-bit frames carry a CRC-8; 16-bit frames are reads without CRC (poly 0x1D, init 0xC4, xor 0xFF). It logs which registers
 * were read, so a test can tell which SPI reads a manager call made.
 */

#include "EncoderManager.h"
#include "HfCoreMocks.h"
#include "HostTest.h"

#include <map>
#include <optional>
#include <vector>

using hf::host::MockSpi;

namespace {

constexpr uint16_t kErrfl = 0x0001;
constexpr uint16_t kAgc = 0x3FF9;
constexpr uint16_t kMag = 0x3FFD;
constexpr uint16_t kAnglecom = 0x3FFF;
constexpr uint16_t kMagHalf = 1u << 1;

uint8_t Crc8(uint16_t data16) {
    uint8_t crc = 0xC4;
    for (int i = 0; i < 16; ++i) {
        const bool bit = (((data16 >> (15 - i)) & 1u) ^ ((crc >> 7) & 1u)) != 0;
        crc = static_cast<uint8_t>((crc << 1) ^ (bit ? 0x1D : 0x00));
    }
    return static_cast<uint8_t>(crc ^ 0xFF);
}

class As5047uModel {
public:
    explicit As5047uModel(MockSpi& spi) {
        spi.SetResponder([this](const uint8_t* tx, uint8_t* rx, uint16_t len) { OnFrame(tx, rx, len); });
    }

    std::map<uint16_t, uint16_t> regs;
    uint16_t errfl_persistent{0};
    std::vector<uint16_t> reads;  ///< register addresses read, in order

    size_t ReadsOf(uint16_t addr) const {
        size_t n = 0;
        for (uint16_t a : reads) {
            n += (a == addr) ? 1 : 0;
        }
        return n;
    }

private:
    uint16_t ReadValue(uint16_t addr) {
        reads.push_back(addr);
        if (addr == kErrfl) {
            return errfl_persistent;
        }
        return static_cast<uint16_t>(regs[addr] & 0x3FFF);
    }

    void OnFrame(const uint8_t* tx, uint8_t* rx, uint16_t len) {
        const uint16_t word = static_cast<uint16_t>((tx[0] << 8) | tx[1]);
        if (len == 2) {  // 16-bit frame: read only, no CRC
            rx[0] = static_cast<uint8_t>((pending_ >> 8) & 0x3F);
            rx[1] = static_cast<uint8_t>(pending_ & 0xFF);
            if ((word & 0x4000u) != 0) {
                pending_ = ReadValue(word & 0x3FFF);
            }
            return;
        }
        if (len != 3) {
            return;
        }
        if (write_addr_) {
            Answer(static_cast<uint16_t>(regs[*write_addr_] & 0x3FFF), rx);
            regs[*write_addr_] = word & 0x3FFF;
            pending_ = regs[*write_addr_];
            write_addr_.reset();
            return;
        }
        Answer(pending_, rx);
        const uint16_t addr = word & 0x3FFF;
        if ((tx[0] & 0x40u) != 0) {
            pending_ = ReadValue(addr);
        } else {
            write_addr_ = addr;
        }
    }

    static void Answer(uint16_t value, uint8_t* rx) {
        rx[0] = static_cast<uint8_t>((value >> 8) & 0x3F);
        rx[1] = static_cast<uint8_t>(value & 0xFF);
        rx[2] = Crc8(static_cast<uint16_t>((rx[0] << 8) | rx[1]));
    }

    uint16_t pending_{0};
    std::optional<uint16_t> write_addr_;
};

constexpr uint8_t kSlot = EncoderManager::ENCODER_SLOT_1;

/** One encoder in slot kSlot on its own MockSpi, removed again when the test ends. */
struct Fixture {
    MockSpi spi;
    As5047uModel chip{spi};
    EncoderManager& enc = EncoderManager::GetInstance();

    Fixture() {
        chip.regs[kAnglecom] = 0x1234;
        chip.regs[kAgc] = 0x80;
        chip.regs[kMag] = 0x0ABC;
        REQUIRE(enc.CreateAs5047uDevice(kSlot, spi));
        std::array<bool, EncoderManager::MAX_ENCODER_DEVICES> results{};
        REQUIRE(enc.InitializeAllDevices(results) == 1u);
        REQUIRE(results[kSlot]);
        chip.reads.clear();
    }
    ~Fixture() { (void)enc.DeleteDevice(kSlot); }
};

}  // namespace

HF_TEST(Encoder_ReadDiagnosticsReadsOnlyTheErrorRegister) {
    Fixture f;
    f.chip.errfl_persistent = kMagHalf;
    As5047uDiagnostics d{};
    CHECK(f.enc.ReadDiagnostics(kSlot, d) == EncoderError::SUCCESS);
    CHECK(f.chip.ReadsOf(kErrfl) >= 1u);
    CHECK_EQ(f.chip.ReadsOf(kAgc), 0u);  // used to be read and thrown away
    CHECK_EQ(f.chip.ReadsOf(kMag), 0u);  // used to be read and thrown away
    CHECK(!d.magnetic_field_ok);
    CHECK(d.communication_ok);
    CHECK_EQ(d.last_error_flags, kMagHalf);
}

HF_TEST(Encoder_ReadFieldStrengthReturnsAgcAndMagnitude) {
    Fixture f;
    EncoderFieldStrength field{};
    CHECK(f.enc.ReadFieldStrength(kSlot, field) == EncoderError::SUCCESS);
    CHECK_EQ(field.agc, 0x80u);
    CHECK_EQ(field.magnitude, 0x0ABCu);
}

HF_TEST(Encoder_ReadAngleCountsAMeasurement) {
    Fixture f;
    const uint32_t before = f.enc.GetMeasurementCount(kSlot);
    uint16_t angle = 0;
    CHECK(f.enc.ReadAngle(kSlot, angle) == EncoderError::SUCCESS);
    CHECK_EQ(angle, 0x1234u);
    CHECK_EQ(f.enc.GetMeasurementCount(kSlot), before + 1);
}

HF_TEST(Encoder_EmptyOrUninitialisedSlotIsRefusedWithoutSpi) {
    EncoderManager& enc = EncoderManager::GetInstance();
    uint16_t angle = 0;
    As5047uDiagnostics d{};
    EncoderFieldStrength field{};
    CHECK(enc.ReadAngle(kSlot, angle) == EncoderError::NOT_INITIALIZED);
    CHECK(enc.ReadDiagnostics(kSlot, d) == EncoderError::NOT_INITIALIZED);
    CHECK(enc.ReadFieldStrength(kSlot, field) == EncoderError::NOT_INITIALIZED);
    CHECK(enc.ReadAngle(EncoderManager::MAX_ENCODER_DEVICES, angle) == EncoderError::NOT_INITIALIZED);

    // Created but never initialised: still refused, and the chip sees no traffic.
    MockSpi spi;
    REQUIRE(enc.CreateAs5047uDevice(kSlot, spi));
    CHECK(enc.ReadAngle(kSlot, angle) == EncoderError::NOT_INITIALIZED);
    CHECK(spi.Transactions().empty());
    CHECK(enc.DeleteDevice(kSlot));
}

HF_TEST(Encoder_VisitDriverRunsOnlyOnAReadySlot) {
    Fixture f;
    uint16_t angle = 0;
    CHECK(f.enc.VisitDriver(kSlot, [&](auto& drv) { angle = drv.GetAngle(); }));
    CHECK_EQ(angle, 0x1234u);
    CHECK(!f.enc.VisitDriver(EncoderManager::ENCODER_SLOT_3, [](auto&) {}));
    CHECK(!f.enc.VisitDriver(EncoderManager::MAX_ENCODER_DEVICES, [](auto&) {}));
}
