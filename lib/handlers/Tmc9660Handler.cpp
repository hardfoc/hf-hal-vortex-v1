#include "Tmc9660Handler.h"
#include <cstring>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsAbstraction.h"
#include "core/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"
#include "handlers/Logger.h"

// ESP-IDF microsecond delay (available on ESP32 targets)
#if defined(ESP_PLATFORM)
#include "esp_rom_sys.h"
#endif

//==============================================================================
// DEFAULT BOOTLOADER CONFIGURATION
//==============================================================================

/**
 * @brief Default TMC9660 bootloader configuration based on TMC9660-3PH-EVAL board settings.
 *
 * Configuration highlights:
 * - LDO: VEXT1=5.0V, VEXT2=3.3V with 3ms slope control
 * - Boot Mode: Parameter mode for TMCL parameter-based control
 * - UART: Auto16x baud rate detection, GPIO6/7 pins, device address 1
 * - External Clock: 16MHz crystal with PLL for stable 40MHz system clock
 * - SPI Flash: Enabled on SPI0 (GPIO11 SCK, GPIO12 CS) at 10MHz
 * - GPIO: GPIO5 analog input, GPIO17/18 digital inputs with pull-down
 * - New sections (hall, abn1, abn2, ref, stepDir, spiEnc, mechBrake, brakeChopper,
 *   memStorage) use safe defaults (disabled)
 */
const tmc9660::BootloaderConfig Tmc9660Handler::kDefaultBootConfig = {
    // LDO Configuration
    {
        tmc9660::bootcfg::LDOVoltage::V5_0,      // vext1
        tmc9660::bootcfg::LDOVoltage::V3_3,      // vext2
        tmc9660::bootcfg::LDOSlope::Slope3ms,    // slope_vext1
        tmc9660::bootcfg::LDOSlope::Slope3ms,    // slope_vext2
        false                                     // ldo_short_fault
    },
    // Boot Configuration
    {
        tmc9660::bootcfg::BootMode::Parameter,    // boot_mode
        false,                                    // bl_ready_fault
        true,                                     // bl_exit_fault
        false,                                    // disable_selftest
        false,                                    // bl_config_fault
        true                                      // start_motor_control
    },
    // UART Configuration
    {
        1,                                         // device_address
        255,                                       // host_address (broadcast)
        false,                                     // disable_uart
        tmc9660::bootcfg::UartRxPin::GPIO7,       // rx_pin
        tmc9660::bootcfg::UartTxPin::GPIO6,       // tx_pin
        tmc9660::bootcfg::BaudRate::Auto16x       // baud_rate
    },
    // RS485 Configuration (disabled)
    {
        false,                                     // enable_rs485
        tmc9660::bootcfg::RS485TxEnPin::None,     // txen_pin
        0,                                         // txen_pre_delay
        0                                          // txen_post_delay
    },
    // SPI Boot Configuration
    {
        false,                                     // disable_spi
        tmc9660::bootcfg::SPIInterface::IFACE0,   // boot_spi_iface
        tmc9660::bootcfg::SPI0SckPin::GPIO6       // spi0_sck_pin
    },
    // SPI Flash Configuration
    {
        true,                                      // enable_flash
        tmc9660::bootcfg::SPIInterface::IFACE0,   // flash_spi_iface
        tmc9660::bootcfg::SPI0SckPin::GPIO11,     // spi0_sck_pin
        12,                                        // cs_pin (GPIO12)
        tmc9660::bootcfg::SPIFlashFreq::Div1      // freq_div (10MHz)
    },
    // I2C EEPROM Configuration (disabled)
    {
        false,                                     // enable_eeprom
        tmc9660::bootcfg::I2CSdaPin::GPIO5,       // sda_pin
        tmc9660::bootcfg::I2CSclPin::GPIO4,       // scl_pin
        0,                                         // address_bits
        tmc9660::bootcfg::I2CFreq::Freq100k       // freq_code
    },
    // Clock Configuration
    {
        tmc9660::bootcfg::ClockSource::External,       // use_external
        tmc9660::bootcfg::ExtSourceType::Oscillator,   // ext_source_type
        tmc9660::bootcfg::XtalDrive::Freq16MHz,        // xtal_drive
        false,                                         // xtal_boost
        tmc9660::bootcfg::SysClkSource::PLL,           // pll_selection
        14,                                            // rdiv
        tmc9660::bootcfg::SysClkDiv::Div1              // sysclk_div
    },
    // GPIO Configuration
    {
        0x00000000,  // outputMask
        0x00000000,  // directionMask
        0x00000000,  // pullUpMask
        0x00060000,  // pullDownMask (GPIO17, GPIO18 pull-down)
        0x00000020   // analogMask (GPIO5 analog)
    },
    // Hall Configuration (disabled by default)
    {},
    // ABN Encoder 1 Configuration (disabled by default)
    {},
    // ABN Encoder 2 Configuration (disabled by default)
    {},
    // Reference Switches Configuration (disabled by default)
    {},
    // Step/Direction Configuration (disabled by default)
    {},
    // SPI Encoder Configuration (disabled by default)
    {},
    // Mechanical Brake Configuration (disabled by default)
    {},
    // Brake Chopper Configuration (disabled by default)
    {},
    // Memory Storage Configuration (disabled by default)
    {}
};

//==============================================================================
// COMMON HELPERS FOR HAL COMM INTERFACES
//==============================================================================

namespace {

static void halDebugLog(int level, const char* tag, const char* format, va_list args) noexcept {
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    switch (level) {
        case 0: Logger::GetInstance().Error(tag, "%s", buffer); break;
        case 1: Logger::GetInstance().Warning(tag, "%s", buffer); break;
        case 2: Logger::GetInstance().Info(tag, "%s", buffer); break;
        case 3: Logger::GetInstance().Debug(tag, "%s", buffer); break;
        default: Logger::GetInstance().Debug(tag, "%s", buffer); break;
    }
}

static void halDelayMs(uint32_t ms) noexcept {
    os_delay_msec(static_cast<uint16_t>(ms));
}

static void halDelayUs(uint32_t us) noexcept {
    if (us >= 1000) {
        os_delay_msec(static_cast<uint16_t>((us + 999) / 1000));
    } else {
#if defined(ESP_PLATFORM)
        esp_rom_delay_us(us);
#else
        // Fallback: busy-wait using cycle counter
        uint32_t start = os_get_processor_cycle_count();
        uint32_t cycles_to_wait = (us * 240); // Assumes 240MHz clock
        while ((os_get_processor_cycle_count() - start) < cycles_to_wait) {
            // Busy wait
        }
#endif
    }
}

/**
 * @brief Set a BaseGpio pin level based on a TMC9660 signal and its active-level config.
 */
static bool setGpioFromSignal(BaseGpio& gpio_pin, tmc9660::GpioSignal signal, bool active_high) noexcept {
    bool physical_high = (signal == tmc9660::GpioSignal::ACTIVE) ? active_high : !active_high;
    auto level = physical_high ? hf_gpio_level_t::HF_GPIO_LEVEL_HIGH : hf_gpio_level_t::HF_GPIO_LEVEL_LOW;
    return gpio_pin.SetPinLevel(level) == hf_gpio_err_t::GPIO_SUCCESS;
}

/**
 * @brief Read a BaseGpio pin and convert to TMC9660 signal based on active-level config.
 */
static bool readGpioToSignal(BaseGpio& gpio_pin, tmc9660::GpioSignal& signal, bool active_high) noexcept {
    hf_gpio_level_t level;
    if (gpio_pin.GetPinLevel(level) != hf_gpio_err_t::GPIO_SUCCESS) {
        return false;
    }
    bool physical_high = (level == hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
    signal = (physical_high == active_high) ? tmc9660::GpioSignal::ACTIVE : tmc9660::GpioSignal::INACTIVE;
    return true;
}

} // anonymous namespace

//==============================================================================
// HAL SPI COMM INTERFACE IMPLEMENTATION
//==============================================================================

HalSpiTmc9660Comm::HalSpiTmc9660Comm(BaseSpi& spi, BaseGpio& rst, BaseGpio& drv_en,
                                       BaseGpio& faultn, BaseGpio& wake,
                                       bool rst_active_high, bool drv_en_active_high,
                                       bool faultn_active_low, bool wake_active_low) noexcept
    : SpiCommInterface<HalSpiTmc9660Comm>(rst_active_high, drv_en_active_high,
                                           wake_active_low, faultn_active_low),
      spi_(spi), ctrl_pins_{rst, drv_en, faultn, wake} {
}

bool HalSpiTmc9660Comm::spiTransferTMCL(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    if (!spi_.EnsureInitialized()) {
        return false;
    }
    hf_spi_err_t result = spi_.Transfer(tx.data(), rx.data(), hf_u16_t(8), hf_u32_t(0));
    return result == hf_spi_err_t::SPI_SUCCESS;
}

bool HalSpiTmc9660Comm::spiTransferBootloader(std::array<uint8_t, 5>& tx, std::array<uint8_t, 5>& rx) noexcept {
    if (!spi_.EnsureInitialized()) {
        return false;
    }
    hf_spi_err_t result = spi_.Transfer(tx.data(), rx.data(), hf_u16_t(5), hf_u32_t(0));
    return result == hf_spi_err_t::SPI_SUCCESS;
}

bool HalSpiTmc9660Comm::gpioSet(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal signal) noexcept {
    return setGpioFromSignal(ctrl_pins_.get(pin), signal,
                             this->pinActiveLevels_[static_cast<int>(pin)]);
}

bool HalSpiTmc9660Comm::gpioRead(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal& signal) noexcept {
    return readGpioToSignal(ctrl_pins_.get(pin), signal,
                            this->pinActiveLevels_[static_cast<int>(pin)]);
}

void HalSpiTmc9660Comm::debugLog(int level, const char* tag, const char* format, va_list args) noexcept {
    halDebugLog(level, tag, format, args);
}

void HalSpiTmc9660Comm::delayMs(uint32_t ms) noexcept { halDelayMs(ms); }
void HalSpiTmc9660Comm::delayUs(uint32_t us) noexcept { halDelayUs(us); }

//==============================================================================
// HAL UART COMM INTERFACE IMPLEMENTATION
//==============================================================================

HalUartTmc9660Comm::HalUartTmc9660Comm(BaseUart& uart, BaseGpio& rst, BaseGpio& drv_en,
                                         BaseGpio& faultn, BaseGpio& wake,
                                         bool rst_active_high, bool drv_en_active_high,
                                         bool faultn_active_low, bool wake_active_low) noexcept
    : UartCommInterface<HalUartTmc9660Comm>(rst_active_high, drv_en_active_high,
                                             wake_active_low, faultn_active_low),
      uart_(uart), ctrl_pins_{rst, drv_en, faultn, wake} {
}

bool HalUartTmc9660Comm::uartSendTMCL(const std::array<uint8_t, 9>& data) noexcept {
    if (!uart_.EnsureInitialized()) {
        return false;
    }
    hf_uart_err_t result = uart_.Write(data.data(), 9);
    return result == hf_uart_err_t::UART_SUCCESS;
}

bool HalUartTmc9660Comm::uartReceiveTMCL(std::array<uint8_t, 9>& data) noexcept {
    if (!uart_.EnsureInitialized()) {
        return false;
    }
    hf_uart_err_t result = uart_.Read(data.data(), 9, 1000); // 1 second timeout
    return result == hf_uart_err_t::UART_SUCCESS;
}

bool HalUartTmc9660Comm::uartTransferBootloader(const std::array<uint8_t, 8>& tx,
                                                  std::array<uint8_t, 8>& rx) noexcept {
    if (!uart_.EnsureInitialized()) {
        return false;
    }
    hf_uart_err_t write_result = uart_.Write(tx.data(), 8);
    if (write_result != hf_uart_err_t::UART_SUCCESS) {
        return false;
    }
    hf_uart_err_t read_result = uart_.Read(rx.data(), 8, 1000);
    return read_result == hf_uart_err_t::UART_SUCCESS;
}

bool HalUartTmc9660Comm::gpioSet(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal signal) noexcept {
    return setGpioFromSignal(ctrl_pins_.get(pin), signal,
                             this->pinActiveLevels_[static_cast<int>(pin)]);
}

bool HalUartTmc9660Comm::gpioRead(tmc9660::TMC9660CtrlPin pin, tmc9660::GpioSignal& signal) noexcept {
    return readGpioToSignal(ctrl_pins_.get(pin), signal,
                            this->pinActiveLevels_[static_cast<int>(pin)]);
}

void HalUartTmc9660Comm::debugLog(int level, const char* tag, const char* format, va_list args) noexcept {
    halDebugLog(level, tag, format, args);
}

void HalUartTmc9660Comm::delayMs(uint32_t ms) noexcept { halDelayMs(ms); }
void HalUartTmc9660Comm::delayUs(uint32_t us) noexcept { halDelayUs(us); }

//==============================================================================
// TMC9660 HANDLER - CONSTRUCTORS / DESTRUCTOR
//==============================================================================

Tmc9660Handler::Tmc9660Handler(BaseSpi& spi, BaseGpio& rst, BaseGpio& drv_en,
                                BaseGpio& faultn, BaseGpio& wake,
                                uint8_t address,
                                const tmc9660::BootloaderConfig* bootCfg)
    : use_spi_(true),
      bootCfg_(bootCfg),
      device_address_(address) {
    // Create SPI comm interface and driver (lazy - driver created in Initialize)
    spi_comm_ = std::make_unique<HalSpiTmc9660Comm>(spi, rst, drv_en, faultn, wake);
}

Tmc9660Handler::Tmc9660Handler(BaseUart& uart, BaseGpio& rst, BaseGpio& drv_en,
                                BaseGpio& faultn, BaseGpio& wake,
                                uint8_t address,
                                const tmc9660::BootloaderConfig* bootCfg)
    : use_spi_(false),
      bootCfg_(bootCfg),
      device_address_(address) {
    // Create UART comm interface and driver (lazy - driver created in Initialize)
    uart_comm_ = std::make_unique<HalUartTmc9660Comm>(uart, rst, drv_en, faultn, wake);
}

Tmc9660Handler::~Tmc9660Handler() = default;

//==============================================================================
// INITIALIZATION
//==============================================================================

bool Tmc9660Handler::Initialize(bool performReset, bool retrieveBootloaderInfo,
                                 bool failOnVerifyError) {
    static constexpr const char* TAG = "Tmc9660Handler";

    // Create driver instance if not already created
    if (use_spi_) {
        if (!spi_comm_) {
            Logger::GetInstance().Error(TAG, "SPI comm interface not available");
            return false;
        }
        if (!spi_driver_) {
            spi_driver_ = std::make_unique<SpiDriver>(*spi_comm_, device_address_, bootCfg_);
        }
    } else {
        if (!uart_comm_) {
            Logger::GetInstance().Error(TAG, "UART comm interface not available");
            return false;
        }
        if (!uart_driver_) {
            uart_driver_ = std::make_unique<UartDriver>(*uart_comm_, device_address_, bootCfg_);
        }
    }

    // Run bootloader initialization
    auto result = visitDriver([&](auto& driver) {
        return driver.bootloaderInit(bootCfg_, performReset, retrieveBootloaderInfo, failOnVerifyError);
    });

    // Check result type - both SpiDriver and UartDriver share the same enum
    bool success;
    if (use_spi_) {
        success = (result == SpiDriver::BootloaderInitResult::Success);
    } else {
        success = (result == UartDriver::BootloaderInitResult::Success);
    }

    if (!success) {
        Logger::GetInstance().Error(TAG, "Bootloader initialization failed");
        return false;
    }

    // Create GPIO and ADC wrappers
    if (!gpioWrappers_[0]) {
        gpioWrappers_[0] = std::make_unique<Gpio>(*this, 17);
        gpioWrappers_[1] = std::make_unique<Gpio>(*this, 18);
        adcWrapper_ = std::make_unique<Adc>(*this);
        temperatureWrapper_ = std::make_unique<Temperature>(*this);
    }

    Logger::GetInstance().Info(TAG, "TMC9660 initialized successfully via %s",
                               use_spi_ ? "SPI" : "UART");
    return true;
}

bool Tmc9660Handler::IsDriverReady() const noexcept {
    if (use_spi_) return spi_driver_ != nullptr;
    return uart_driver_ != nullptr;
}

//==============================================================================
// CORE PARAMETER ACCESS
//==============================================================================

bool Tmc9660Handler::WriteParameter(tmc9660::tmcl::Parameters id, uint32_t value,
                                     uint8_t motorIndex) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.writeParameter(id, value, motorIndex); });
}

bool Tmc9660Handler::ReadParameter(tmc9660::tmcl::Parameters id, uint32_t& value,
                                    uint8_t motorIndex) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.readParameter(id, value, motorIndex); });
}

bool Tmc9660Handler::SendCommand(tmc9660::tmcl::Op opcode, uint16_t type, uint8_t motor,
                                  uint32_t value, uint32_t* reply) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.sendCommand(opcode, type, motor, value, reply); });
}

//==============================================================================
// MOTOR CONTROL CONVENIENCE METHODS
//==============================================================================

bool Tmc9660Handler::SetMotorType(tmc9660::tmcl::MotorType type, uint8_t polePairs) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.motorConfig.setType(type, polePairs); });
}

bool Tmc9660Handler::SetPWMFrequency(uint32_t freq_hz) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.motorConfig.setPWMFrequency(freq_hz); });
}

bool Tmc9660Handler::SetCommutationMode(tmc9660::tmcl::CommutationMode mode) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.motorConfig.setCommutationMode(mode); });
}

bool Tmc9660Handler::EnableMotor() noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.motorConfig.enable(); });
}

bool Tmc9660Handler::DisableMotor() noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.motorConfig.disable(); });
}

bool Tmc9660Handler::SetTargetVelocity(int32_t velocity) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.velocityControl.setTargetVelocity(velocity); });
}

bool Tmc9660Handler::SetTargetPosition(int32_t position) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.positionControl.setTargetPosition(position); });
}

bool Tmc9660Handler::SetTargetTorque(int16_t torque_ma) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.torqueFluxControl.setTargetTorque(torque_ma); });
}

//------------------------------------------------------------------------------
// DRV_EN Pin Control
//------------------------------------------------------------------------------

bool Tmc9660Handler::EnableDriverOutput() noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([](auto& driver) {
        return driver.comm().gpioSetActive(tmc9660::TMC9660CtrlPin::DRV_EN);
    });
}

bool Tmc9660Handler::DisableDriverOutput() noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([](auto& driver) {
        return driver.comm().gpioSetInactive(tmc9660::TMC9660CtrlPin::DRV_EN);
    });
}

//------------------------------------------------------------------------------
// Feedback Sensor Configuration
//------------------------------------------------------------------------------

bool Tmc9660Handler::ConfigureHallSensor(
        tmc9660::tmcl::HallSectorOffset sector_offset,
        tmc9660::tmcl::Direction inverted,
        tmc9660::tmcl::EnableDisable enable_extrapolation,
        uint8_t filter_length) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) {
        return driver.feedbackSense.configureHall(
            sector_offset, inverted, enable_extrapolation, filter_length);
    });
}

bool Tmc9660Handler::ConfigureABNEncoder(
        uint32_t counts_per_rev,
        tmc9660::tmcl::Direction inverted,
        tmc9660::tmcl::EnableDisable n_channel_inverted) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) {
        return driver.feedbackSense.configureABNEncoder(
            counts_per_rev, inverted, n_channel_inverted);
    });
}

//------------------------------------------------------------------------------
// Current Sensing Calibration
//------------------------------------------------------------------------------

bool Tmc9660Handler::CalibrateCurrentSensing(bool wait_for_completion,
                                              uint32_t timeout_ms) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) {
        return driver.currentSensing.calibrateOffsets(wait_for_completion, timeout_ms);
    });
}

//------------------------------------------------------------------------------
// PID Loop Gains
//------------------------------------------------------------------------------

bool Tmc9660Handler::SetCurrentLoopGains(uint16_t p, uint16_t i,
                                          bool separate,
                                          uint16_t flux_p, uint16_t flux_i) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) {
        return driver.torqueFluxControl.setCurrentLoopGains(p, i, separate, flux_p, flux_i);
    });
}

bool Tmc9660Handler::SetVelocityLoopGains(uint16_t p, uint16_t i) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) {
        return driver.velocityControl.setVelocityLoopGains(p, i);
    });
}

bool Tmc9660Handler::SetPositionLoopGains(uint16_t p, uint16_t i) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) {
        return driver.positionControl.setPositionLoopGains(p, i);
    });
}

//==============================================================================
// TELEMETRY
//==============================================================================

float Tmc9660Handler::GetSupplyVoltage() noexcept {
    if (!IsDriverReady()) return std::nanf("");
    return visitDriver([](auto& driver) { return driver.telemetry.getSupplyVoltage(); });
}

float Tmc9660Handler::GetChipTemperature() noexcept {
    if (!IsDriverReady()) return std::nanf("");
    return visitDriver([](auto& driver) { return driver.telemetry.getChipTemperature(); });
}

int16_t Tmc9660Handler::GetMotorCurrent() noexcept {
    if (!IsDriverReady()) return 0;
    return visitDriver([](auto& driver) { return driver.telemetry.getMotorCurrent(); });
}

int32_t Tmc9660Handler::GetActualVelocity() noexcept {
    if (!IsDriverReady()) return 0;
    return visitDriver([](auto& driver) { return driver.telemetry.getActualVelocity(); });
}

int32_t Tmc9660Handler::GetActualPosition() noexcept {
    if (!IsDriverReady()) return 0;
    return visitDriver([](auto& driver) { return driver.telemetry.getActualPosition(); });
}

uint16_t Tmc9660Handler::GetExternalTemperature() noexcept {
    if (!IsDriverReady()) return 0;
    return visitDriver([](auto& driver) { return driver.telemetry.getExternalTemperature(); });
}

bool Tmc9660Handler::GetStatusFlags(uint32_t& flags) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.telemetry.getGeneralStatusFlags(flags); });
}

bool Tmc9660Handler::GetErrorFlags(uint32_t& flags) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.telemetry.getGeneralErrorFlags(flags); });
}

bool Tmc9660Handler::ClearErrorFlags(uint32_t mask) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.telemetry.clearGeneralErrorFlags(mask); });
}

bool Tmc9660Handler::GetGateDriverErrorFlags(uint32_t& flags) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.telemetry.getGateDriverErrorFlags(flags); });
}

bool Tmc9660Handler::ClearGateDriverErrorFlags(uint32_t mask) noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([&](auto& driver) { return driver.telemetry.clearGateDriverErrorFlags(mask); });
}

//==============================================================================
// BOOTLOADER
//==============================================================================

bool Tmc9660Handler::EnterBootloaderMode() noexcept {
    if (!IsDriverReady()) return false;
    return visitDriver([](auto& driver) { return driver.enterBootloaderMode(); });
}

//==============================================================================
// PERIPHERAL ACCESSORS
//==============================================================================

Tmc9660Handler::Gpio& Tmc9660Handler::gpio(uint8_t gpioNumber) {
    if (gpioNumber == 17) return *gpioWrappers_[0];
    if (gpioNumber == 18) return *gpioWrappers_[1];
    return *gpioWrappers_[0]; // Fallback
}

Tmc9660Handler::Adc& Tmc9660Handler::adc() {
    return *adcWrapper_;
}

Tmc9660Handler::Temperature& Tmc9660Handler::temperature() {
    return *temperatureWrapper_;
}

//==============================================================================
// COMMUNICATION INFO
//==============================================================================

tmc9660::CommMode Tmc9660Handler::GetCommMode() const noexcept {
    return use_spi_ ? tmc9660::CommMode::SPI : tmc9660::CommMode::UART;
}

//==============================================================================
// GPIO WRAPPER IMPLEMENTATION
//==============================================================================

Tmc9660Handler::Gpio::Gpio(Tmc9660Handler& parent, uint8_t gpioNumber)
    : BaseGpio(gpioNumber, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT),
      parent_(parent), gpioNumber_(gpioNumber) {
    std::snprintf(description_, sizeof(description_), "TMC9660 GPIO%u", gpioNumber_);
}

bool Tmc9660Handler::Gpio::Initialize() noexcept {
    if (!parent_.IsDriverReady()) return false;
    return parent_.visitDriver([&](auto& driver) {
        return driver.gpio.setMode(gpioNumber_, true, false, true);
    });
}

bool Tmc9660Handler::Gpio::Deinitialize() noexcept { return true; }

hf_gpio_err_t Tmc9660Handler::Gpio::SetPinLevelImpl(hf_gpio_level_t level) noexcept {
    if (!parent_.IsDriverReady()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    if (direction_ != hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT)
        return hf_gpio_err_t::GPIO_ERR_INVALID_CONFIGURATION;
    bool pin_high = (level == hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
    bool ok = parent_.visitDriver([&](auto& driver) {
        return driver.gpio.writePin(gpioNumber_, pin_high);
    });
    return ok ? hf_gpio_err_t::GPIO_SUCCESS : hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
}

hf_gpio_err_t Tmc9660Handler::Gpio::GetPinLevelImpl(hf_gpio_level_t& level) noexcept {
    if (!parent_.IsDriverReady()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    bool pin_state = false;
    bool ok = parent_.visitDriver([&](auto& driver) {
        return driver.gpio.readDigital(gpioNumber_, pin_state);
    });
    if (!ok) return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    level = pin_state ? hf_gpio_level_t::HF_GPIO_LEVEL_HIGH : hf_gpio_level_t::HF_GPIO_LEVEL_LOW;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetDirectionImpl(hf_gpio_direction_t direction) noexcept {
    if (!parent_.IsDriverReady()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;

    bool is_output = (direction == hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT);
    bool ok = parent_.visitDriver([&](auto& driver) {
        return driver.gpio.setMode(gpioNumber_, is_output, false, true);
    });
    if (!ok) return hf_gpio_err_t::GPIO_ERR_FAILURE;

    direction_ = direction;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept {
    if (mode != hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL)
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept {
    if (mode != hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING)
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_pull_mode_t Tmc9660Handler::Gpio::GetPullModeImpl() const noexcept {
    return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
}

bool Tmc9660Handler::Gpio::IsPinAvailable() const noexcept {
    return gpioNumber_ == 17 || gpioNumber_ == 18;
}

hf_u8_t Tmc9660Handler::Gpio::GetMaxPins() const noexcept { return 2; }

const char* Tmc9660Handler::Gpio::GetDescription() const noexcept { return description_; }

hf_gpio_err_t Tmc9660Handler::Gpio::GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept {
    direction = direction_;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept {
    mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

//==============================================================================
// ADC WRAPPER IMPLEMENTATION
//==============================================================================

Tmc9660Handler::Adc::Adc(Tmc9660Handler& parent) : parent_(parent) {}

bool Tmc9660Handler::Adc::Initialize() noexcept { return true; }
bool Tmc9660Handler::Adc::Deinitialize() noexcept { return true; }
hf_u8_t Tmc9660Handler::Adc::GetMaxChannels() const noexcept { return 15; }

bool Tmc9660Handler::Adc::IsChannelAvailable(hf_channel_id_t channel_id) const noexcept {
    return ValidateChannelId(channel_id) == hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                                                 hf_u8_t /*numOfSamplesToAvg*/,
                                                 hf_time_t /*timeBetweenSamples*/) noexcept {
    RtosMutex::LockGuard lock(mutex_);
    const uint64_t start_time_us = GetCurrentTimeUs();
    hf_u32_t raw = 0;
    hf_adc_err_t result = ReadChannelLocked(channel_id, raw, channel_reading_v);
    UpdateStatistics(result, start_time_us);
    return result;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannelCount(hf_channel_id_t channel_id,
                                                     hf_u32_t& channel_reading_count,
                                                     hf_u8_t /*numOfSamplesToAvg*/,
                                                     hf_time_t /*timeBetweenSamples*/) noexcept {
    RtosMutex::LockGuard lock(mutex_);
    const uint64_t start_time_us = GetCurrentTimeUs();
    float voltage = 0.0f;
    hf_adc_err_t result = ReadChannelLocked(channel_id, channel_reading_count, voltage);
    UpdateStatistics(result, start_time_us);
    return result;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannel(hf_channel_id_t channel_id,
                                                hf_u32_t& channel_reading_count,
                                                float& channel_reading_v,
                                                hf_u8_t /*numOfSamplesToAvg*/,
                                                hf_time_t /*timeBetweenSamples*/) noexcept {
    RtosMutex::LockGuard lock(mutex_);
    const uint64_t start_time_us = GetCurrentTimeUs();
    hf_adc_err_t result = ReadChannelLocked(channel_id, channel_reading_count, channel_reading_v);
    UpdateStatistics(result, start_time_us);
    return result;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadMultipleChannels(const hf_channel_id_t* channel_ids,
                                                         hf_u8_t num_channels,
                                                         hf_u32_t* readings, float* voltages) noexcept {
    if (!channel_ids || !readings || !voltages)
        return hf_adc_err_t::ADC_ERR_NULL_POINTER;

    RtosMutex::LockGuard lock(mutex_);
    for (hf_u8_t i = 0; i < num_channels; ++i) {
        hf_adc_err_t result = ReadChannelLocked(channel_ids[i], readings[i], voltages[i]);
        if (result != hf_adc_err_t::ADC_SUCCESS) return result;
    }
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::GetStatistics(hf_adc_statistics_t& statistics) noexcept {
    RtosMutex::LockGuard lock(mutex_);
    statistics = statistics_;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::GetDiagnostics(hf_adc_diagnostics_t& diagnostics) noexcept {
    RtosMutex::LockGuard lock(mutex_);
    diagnostics = diagnostics_;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ResetStatistics() noexcept {
    RtosMutex::LockGuard lock(mutex_);
    statistics_ = hf_adc_statistics_t{};
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ResetDiagnostics() noexcept {
    RtosMutex::LockGuard lock(mutex_);
    diagnostics_ = hf_adc_diagnostics_t{};
    last_error_.store(hf_adc_err_t::ADC_SUCCESS);
    return hf_adc_err_t::ADC_SUCCESS;
}

// Private ADC helpers
hf_adc_err_t Tmc9660Handler::Adc::ValidateChannelId(hf_channel_id_t channel_id) const noexcept {
    if (channel_id <= 3) return hf_adc_err_t::ADC_SUCCESS;
    if (channel_id >= 10 && channel_id <= 13) return hf_adc_err_t::ADC_SUCCESS;
    if (channel_id >= 20 && channel_id <= 21) return hf_adc_err_t::ADC_SUCCESS;
    if (channel_id >= 30 && channel_id <= 31) return hf_adc_err_t::ADC_SUCCESS;
    if (channel_id >= 40 && channel_id <= 42) return hf_adc_err_t::ADC_SUCCESS;
    return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannelLocked(hf_channel_id_t channel_id,
                                                      hf_u32_t& raw, float& voltage) noexcept {
    // Caller must hold mutex_.
    hf_adc_err_t validation_result = ValidateChannelId(channel_id);
    if (validation_result != hf_adc_err_t::ADC_SUCCESS) {
        UpdateDiagnostics(validation_result);
        return validation_result;
    }

    hf_adc_err_t result = hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    voltage = 0.0f;

    if (channel_id <= 3) {
        result = ReadAinChannel(channel_id, raw, voltage);
    } else if (channel_id >= 10 && channel_id <= 13) {
        result = ReadCurrentSenseChannel(channel_id - 10, raw, voltage);
    } else if (channel_id >= 20 && channel_id <= 21) {
        result = ReadVoltageChannel(channel_id - 20, raw, voltage);
    } else if (channel_id >= 30 && channel_id <= 31) {
        result = ReadTemperatureChannel(channel_id - 30, raw, voltage);
    } else if (channel_id >= 40 && channel_id <= 42) {
        result = ReadMotorDataChannel(channel_id - 40, raw, voltage);
    }

    if (result != hf_adc_err_t::ADC_SUCCESS) UpdateDiagnostics(result);
    return result;
}

hf_adc_err_t Tmc9660Handler::Adc::UpdateStatistics(hf_adc_err_t result, uint64_t start_time_us) noexcept {
    const uint64_t end_time_us = GetCurrentTimeUs();
    const uint32_t conversion_time_us = static_cast<uint32_t>(end_time_us - start_time_us);

    statistics_.totalConversions++;
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        statistics_.successfulConversions++;
        if (statistics_.totalConversions == 1) {
            statistics_.minConversionTimeUs = conversion_time_us;
            statistics_.maxConversionTimeUs = conversion_time_us;
            statistics_.averageConversionTimeUs = conversion_time_us;
        } else {
            statistics_.minConversionTimeUs = std::min(statistics_.minConversionTimeUs, conversion_time_us);
            statistics_.maxConversionTimeUs = std::max(statistics_.maxConversionTimeUs, conversion_time_us);
            const uint32_t total_time = statistics_.averageConversionTimeUs * (statistics_.successfulConversions - 1) + conversion_time_us;
            statistics_.averageConversionTimeUs = total_time / statistics_.successfulConversions;
        }
    } else {
        statistics_.failedConversions++;
    }
    return result;
}

uint64_t Tmc9660Handler::Adc::GetCurrentTimeUs() const noexcept {
    OS_Ulong ticks = os_time_get();
    return static_cast<uint64_t>(ticks) * 1000000 / osTickRateHz;
}

void Tmc9660Handler::Adc::UpdateDiagnostics(hf_adc_err_t error) noexcept {
    last_error_.store(error);
    if (error != hf_adc_err_t::ADC_SUCCESS) {
        diagnostics_.consecutiveErrors++;
        diagnostics_.lastErrorCode = error;
        diagnostics_.lastErrorTimestamp = GetCurrentTimeUs();
        if (diagnostics_.consecutiveErrors > 10) diagnostics_.adcHealthy = false;
    } else {
        diagnostics_.consecutiveErrors = 0;
        diagnostics_.adcHealthy = true;
    }
}

// TMC9660-specific ADC channel methods
hf_adc_err_t Tmc9660Handler::Adc::ReadAinChannel(uint8_t ain_channel,
                                                    hf_u32_t& raw_value, float& voltage) noexcept {
    if (!parent_.IsDriverReady()) return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;

    uint16_t analog_value = 0;
    bool ok = parent_.visitDriver([&](auto& driver) {
        return driver.gpio.readAnalog(ain_channel, analog_value);
    });
    if (!ok) return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;

    raw_value = static_cast<hf_u32_t>(analog_value);
    voltage = static_cast<float>(analog_value) * 3.3f / 65535.0f;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadCurrentSenseChannel(uint8_t current_channel,
                                                            hf_u32_t& raw_value, float& voltage) noexcept {
    if (!parent_.IsDriverReady()) return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;

    tmc9660::tmcl::Parameters param;
    switch (current_channel) {
        case 0: param = tmc9660::tmcl::Parameters::ADC_I0; break;
        case 1: param = tmc9660::tmcl::Parameters::ADC_I1; break;
        case 2: param = tmc9660::tmcl::Parameters::ADC_I2; break;
        case 3: param = tmc9660::tmcl::Parameters::ADC_I3; break;
        default: return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    }

    uint32_t value = 0;
    if (!parent_.ReadParameter(param, value)) return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;

    raw_value = static_cast<hf_u32_t>(value);
    voltage = static_cast<float>(value) * 3.3f / 65535.0f;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadVoltageChannel(uint8_t voltage_channel,
                                                       hf_u32_t& raw_value, float& voltage) noexcept {
    if (!parent_.IsDriverReady()) return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;

    switch (voltage_channel) {
        case 0: // Supply voltage
            voltage = parent_.GetSupplyVoltage();
            raw_value = static_cast<hf_u32_t>(voltage * 1000.0f);
            break;
        case 1: // Driver voltage (use supply as approximation)
            voltage = parent_.GetSupplyVoltage();
            raw_value = static_cast<hf_u32_t>(voltage * 1000.0f);
            break;
        default:
            return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    }
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadTemperatureChannel(uint8_t temp_channel,
                                                           hf_u32_t& raw_value, float& voltage) noexcept {
    if (!parent_.IsDriverReady()) return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;

    switch (temp_channel) {
        case 0: { // Chip temperature
            float temp_c = parent_.GetChipTemperature();
            raw_value = static_cast<hf_u32_t>(temp_c * 100.0f);
            voltage = temp_c;
            break;
        }
        case 1: { // External temperature
            uint16_t ext_temp_raw = parent_.GetExternalTemperature();
            raw_value = static_cast<hf_u32_t>(ext_temp_raw);
            voltage = static_cast<float>(ext_temp_raw) * 3.3f / 65535.0f;
            break;
        }
        default:
            return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    }
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadMotorDataChannel(uint8_t motor_channel,
                                                         hf_u32_t& raw_value, float& voltage) noexcept {
    if (!parent_.IsDriverReady()) return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;

    switch (motor_channel) {
        case 0: { // Motor current
            int16_t current_ma = parent_.GetMotorCurrent();
            raw_value = static_cast<hf_u32_t>(current_ma);
            voltage = static_cast<float>(current_ma) / 1000.0f;
            break;
        }
        case 1: { // Motor velocity
            int32_t vel = parent_.GetActualVelocity();
            raw_value = static_cast<hf_u32_t>(vel);
            voltage = static_cast<float>(vel);
            break;
        }
        case 2: { // Motor position
            int32_t pos = parent_.GetActualPosition();
            raw_value = static_cast<hf_u32_t>(pos);
            voltage = static_cast<float>(pos);
            break;
        }
        default:
            return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    }
    return hf_adc_err_t::ADC_SUCCESS;
}

const char* Tmc9660Handler::Adc::GetChannelTypeString(hf_channel_id_t channel_id) const noexcept {
    if (channel_id <= 3) return "AIN";
    if (channel_id >= 10 && channel_id <= 13) return "Current";
    if (channel_id >= 20 && channel_id <= 21) return "Voltage";
    if (channel_id >= 30 && channel_id <= 31) return "Temperature";
    if (channel_id >= 40 && channel_id <= 42) return "Motor";
    return "Unknown";
}

//==============================================================================
// TEMPERATURE WRAPPER IMPLEMENTATION
//==============================================================================

Tmc9660Handler::Temperature::Temperature(Tmc9660Handler& parent)
    : parent_(parent), last_error_(hf_temp_err_t::TEMP_SUCCESS) {
    statistics_ = hf_temp_statistics_t{};
    diagnostics_ = hf_temp_diagnostics_t{};
}

bool Tmc9660Handler::Temperature::Initialize() noexcept {
    static constexpr const char* TAG = "Tmc9660Handler::Temperature";
    if (IsInitialized()) return true;
    if (!parent_.IsDriverReady()) {
        Logger::GetInstance().Error(TAG, "Parent TMC9660 driver not ready");
        return false;
    }
    Logger::GetInstance().Info(TAG, "Temperature sensor initialized");
    return true;
}

bool Tmc9660Handler::Temperature::Deinitialize() noexcept { return true; }

hf_temp_err_t Tmc9660Handler::Temperature::ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept {
    static constexpr const char* TAG = "Tmc9660Handler::Temperature";

    if (temperature_celsius == nullptr) {
        UpdateDiagnostics(hf_temp_err_t::TEMP_ERR_NULL_POINTER);
        return hf_temp_err_t::TEMP_ERR_NULL_POINTER;
    }

    RtosMutex::LockGuard lock(mutex_);
    uint64_t start_time_us = GetCurrentTimeUs();

    if (!parent_.IsDriverReady()) {
        UpdateDiagnostics(hf_temp_err_t::TEMP_ERR_NOT_INITIALIZED);
        return hf_temp_err_t::TEMP_ERR_NOT_INITIALIZED;
    }

    float temp_c = parent_.GetChipTemperature();

    // Check for error condition (TMC9660 returns -273.0f on error)
    if (temp_c < -270.0f || std::isnan(temp_c)) {
        Logger::GetInstance().Error(TAG, "Failed to read chip temperature");
        UpdateDiagnostics(hf_temp_err_t::TEMP_ERR_READ_FAILED);
        return hf_temp_err_t::TEMP_ERR_READ_FAILED;
    }

    if (temp_c < -40.0f || temp_c > 150.0f) {
        Logger::GetInstance().Warning(TAG, "Temperature out of range: %.2f°C", temp_c);
        UpdateDiagnostics(hf_temp_err_t::TEMP_ERR_OUT_OF_RANGE);
        return hf_temp_err_t::TEMP_ERR_OUT_OF_RANGE;
    }

    *temperature_celsius = temp_c;
    UpdateStatistics(hf_temp_err_t::TEMP_SUCCESS, start_time_us);
    UpdateDiagnostics(hf_temp_err_t::TEMP_SUCCESS);
    return hf_temp_err_t::TEMP_SUCCESS;
}

hf_temp_err_t Tmc9660Handler::Temperature::GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept {
    if (info == nullptr) return hf_temp_err_t::TEMP_ERR_NULL_POINTER;

    info->sensor_type = HF_TEMP_SENSOR_TYPE_INTERNAL;
    info->min_temp_celsius = -40.0f;
    info->max_temp_celsius = 150.0f;
    info->resolution_celsius = 0.1f;
    info->accuracy_celsius = 2.0f;
    info->response_time_ms = 100;
    info->capabilities = HF_TEMP_CAP_HIGH_PRECISION | HF_TEMP_CAP_FAST_RESPONSE;
    info->manufacturer = "Trinamic";
    info->model = "TMC9660";
    info->version = "Internal Chip Temperature Sensor";
    return hf_temp_err_t::TEMP_SUCCESS;
}

hf_u32_t Tmc9660Handler::Temperature::GetCapabilities() const noexcept {
    return HF_TEMP_CAP_HIGH_PRECISION | HF_TEMP_CAP_FAST_RESPONSE;
}

hf_temp_err_t Tmc9660Handler::Temperature::UpdateStatistics(hf_temp_err_t result,
                                                              uint64_t start_time_us) noexcept {
    uint64_t end_time_us = GetCurrentTimeUs();
    uint32_t operation_time_us = static_cast<uint32_t>(end_time_us - start_time_us);

    statistics_.total_operations++;
    if (result == hf_temp_err_t::TEMP_SUCCESS) {
        statistics_.successful_operations++;
        statistics_.temperature_readings++;
    } else {
        statistics_.failed_operations++;
    }

    if (statistics_.total_operations == 1) {
        statistics_.min_operation_time_us = operation_time_us;
        statistics_.max_operation_time_us = operation_time_us;
        statistics_.average_operation_time_us = operation_time_us;
    } else {
        statistics_.min_operation_time_us = std::min(statistics_.min_operation_time_us, operation_time_us);
        statistics_.max_operation_time_us = std::max(statistics_.max_operation_time_us, operation_time_us);
        statistics_.average_operation_time_us =
            (statistics_.average_operation_time_us * (statistics_.total_operations - 1) + operation_time_us) /
            statistics_.total_operations;
    }
    return result;
}

uint64_t Tmc9660Handler::Temperature::GetCurrentTimeUs() const noexcept {
    OS_Ulong ticks = os_time_get();
    return static_cast<uint64_t>(ticks) * 1000000 / osTickRateHz;
}

void Tmc9660Handler::Temperature::UpdateDiagnostics(hf_temp_err_t error) noexcept {
    diagnostics_.last_error_code = error;
    diagnostics_.last_error_timestamp = static_cast<hf_u32_t>(GetCurrentTimeUs() / 1000);

    if (error != hf_temp_err_t::TEMP_SUCCESS) {
        diagnostics_.consecutive_errors++;
        diagnostics_.sensor_healthy = false;
    } else {
        diagnostics_.consecutive_errors = 0;
        diagnostics_.sensor_healthy = true;
    }
    diagnostics_.sensor_available = parent_.IsDriverReady();
    last_error_.store(error);
}

//==============================================================================
// DIAGNOSTICS
//==============================================================================

void Tmc9660Handler::DumpDiagnostics() noexcept {
    static constexpr const char* TAG = "Tmc9660Handler";

    Logger::GetInstance().Info(TAG, "=== TMC9660 HANDLER DIAGNOSTICS ===");
    Logger::GetInstance().Info(TAG, "Driver Ready: %s", IsDriverReady() ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "Comm Mode: %s", use_spi_ ? "SPI" : "UART");
    Logger::GetInstance().Info(TAG, "Device Address: %d", device_address_);

    if (IsDriverReady()) {
        uint32_t status_flags = 0, error_flags = 0;
        float voltage = GetSupplyVoltage();
        float temp = GetChipTemperature();
        GetStatusFlags(status_flags);
        GetErrorFlags(error_flags);

        Logger::GetInstance().Info(TAG, "Supply Voltage: %.2fV", voltage);
        Logger::GetInstance().Info(TAG, "Chip Temperature: %.2f°C", temp);
        Logger::GetInstance().Info(TAG, "Status Flags: 0x%08X", status_flags);
        Logger::GetInstance().Info(TAG, "Error Flags: 0x%08X", error_flags);
    }

    // ADC diagnostics
    if (adcWrapper_) {
        Logger::GetInstance().Info(TAG, "ADC Wrapper: ACTIVE");
    }

    // GPIO diagnostics
    int active_gpio_count = 0;
    for (size_t i = 0; i < gpioWrappers_.size(); ++i) {
        if (gpioWrappers_[i]) active_gpio_count++;
    }
    Logger::GetInstance().Info(TAG, "Active GPIO Wrappers: %d/%d",
                               active_gpio_count, static_cast<int>(gpioWrappers_.size()));

    // Bootloader config
    if (bootCfg_) {
        Logger::GetInstance().Info(TAG, "Boot Mode: %s",
            bootCfg_->boot.boot_mode == tmc9660::bootcfg::BootMode::Parameter ? "Parameter" :
            bootCfg_->boot.boot_mode == tmc9660::bootcfg::BootMode::Register ? "Register" : "Unknown");
    }

    Logger::GetInstance().Info(TAG, "=== END TMC9660 HANDLER DIAGNOSTICS ===");
}
