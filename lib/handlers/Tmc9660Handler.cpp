#include "Tmc9660Handler.h"
#include <cstring>
#include <algorithm>
#include "utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/include/OsAbstraction.h"
#include "utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/include/OsUtility.h"
#include "utils-and-drivers/driver-handlers/Logger.h"

/**
 * @brief Default TMC9660 bootloader configuration based on TMC9660-3PH-EVAL board settings.
 * 
 * This configuration is derived from the official TMC9660-3PH-EVAL evaluation board 
 * bootloader TOML file and provides a comprehensive setup for motor control applications.
 * 
 * Key configuration highlights:
 * - LDO Regulators: VEXT1=5.0V, VEXT2=3.3V with 3ms slope control
 * - Boot Mode: Parameter mode for flexible motor control via TMCL parameters
 * - UART: Auto16x baud rate detection, GPIO6/7 pins, device address 1
 * - External Clock: 16MHz crystal oscillator with PLL for stable 40MHz system clock
 * - SPI Flash: Enabled on SPI0 interface with 10MHz operation (GPIO11 SCK, GPIO12 CS)
 * - GPIO Configuration:
 *   - GPIO5: Analog input for sensor feedback
 *   - GPIO17: Digital input with pull-down resistor
 *   - GPIO18: Digital input with pull-down resistor
 * - Additional Features: Hall sensors (GPIO2/3/4), ABN encoders, watchdog enabled
 * 
 * This configuration ensures compatibility with the TMC9660-3PH-EVAL documentation
 * and reference designs, providing a solid foundation for motor control development.
 * 
 * @note Configuration matches TMC9660-3PH-EVKIT User Guide Rev 1 Table 3 specifications
 * @see TMC9660-3PH-EVKIT Evaluation Board User Guide for complete pin assignments
 */
// Static member initialization
const tmc9660::BootloaderConfig Tmc9660Handler::kDefaultBootConfig = {
    // LDO Configuration - Enable both LDOs per evaluation board settings
    {
        tmc9660::bootcfg::LDOVoltage::V5_0,      // vext1 - 5V output
        tmc9660::bootcfg::LDOVoltage::V3_3,      // vext2 - 3.3V output
        tmc9660::bootcfg::LDOSlope::Slope3ms,    // slope_vext1 - 3ms slope
        tmc9660::bootcfg::LDOSlope::Slope3ms,    // slope_vext2 - 3ms slope
        false                                    // ldo_short_fault - Disabled per config
    },
    
    // Boot Configuration - Parameter mode for parameter-based control
    {
        tmc9660::bootcfg::BootMode::Parameter,   // boot_mode - Parameter mode
        false,                                   // bl_ready_fault
        true,                                    // bl_exit_fault
        false,                                   // disable_selftest
        false,                                   // bl_config_fault
        false                                    // start_motor_control - Let application control
    },
    
    // UART Configuration - Standard settings for TMCL communication
    {
        1,                                          // device_address
        255,                                        // host_address (broadcast)
        false,                                      // disable_uart
        tmc9660::bootcfg::UartRxPin::GPIO7,        // rx_pin
        tmc9660::bootcfg::UartTxPin::GPIO6,        // tx_pin
        tmc9660::bootcfg::BaudRate::Auto16x        // baud_rate - Auto 16x oversampling
    },
    
    // RS485 Configuration - Disabled by default
    {
        false,                                      // enable_rs485
        tmc9660::bootcfg::RS485TxEnPin::None,       // txen_pin
        0,                                          // txen_pre_delay
        0                                           // txen_post_delay
    },
    
    // SPI Boot Configuration - Use interface 0 with standard pins
    {
        false,                                      // disable_spi
        tmc9660::bootcfg::SPIInterface::IFACE0,     // boot_spi_iface
        tmc9660::bootcfg::SPI0SckPin::GPIO6         // spi0_sck_pin
    },
    
    // SPI Flash Configuration - Enable per TMC9660-3PH-EVAL board settings
    {
        true,                                       // enable_flash - Enable SPI flash
        tmc9660::bootcfg::SPIInterface::IFACE0,     // flash_spi_iface - Use SPI0 block
        tmc9660::bootcfg::SPI0SckPin::GPIO11,       // spi0_sck_pin - GPIO11 per eval board
        12,                                         // cs_pin - GPIO12 chip select
        tmc9660::bootcfg::SPIFlashFreq::Div1        // freq_div - 10MHz frequency (Div1)
    },
    
    // I2C EEPROM Configuration - Disabled by default
    {
        false,                                      // enable_eeprom
        tmc9660::bootcfg::I2CSdaPin::GPIO5,         // sda_pin
        tmc9660::bootcfg::I2CSclPin::GPIO4,         // scl_pin
        0,                                          // address_bits
        tmc9660::bootcfg::I2CFreq::Freq100k         // freq_code
    },
    
    // Clock Configuration - Use external 16MHz crystal with PLL
    {
        tmc9660::bootcfg::ClockSource::External,        // use_external - External crystal
        tmc9660::bootcfg::ExtSourceType::Oscillator,    // ext_source_type - Crystal oscillator
        tmc9660::bootcfg::XtalDrive::Freq16MHz,         // xtal_drive - 16MHz crystal
        false,                                          // xtal_boost
        tmc9660::bootcfg::SysClkSource::PLL,            // pll_selection - Use PLL
        14,                                             // rdiv - PLL reference divider for 16MHz
        tmc9660::bootcfg::SysClkDiv::Div1               // sysclk_div
    },
    
    // GPIO Configuration - Configure per TMC9660-3PH-EVAL board settings
    {
        0x00000000,  // outputMask - GPIO17 and GPIO18 as inputs (bits clear)
        0x00000000,  // directionMask - GPIO17 and GPIO18 configured as inputs
        0x00000000,  // pullUpMask - No pull-up resistors
        0x00060000,  // pullDownMask - GPIO17(bit17)=1, GPIO18(bit18)=1 pull-down
        0x00000020   // analogMask - GPIO5 as analog input (bit5=1)
    }
};

// ==================== TMC9660 Communication Interface Implementations ====================

// SPI Communication Interface Implementation
Tmc9660SpiCommInterface::Tmc9660SpiCommInterface(BaseSpi& spi_interface) noexcept
    : spi_interface_(spi_interface) {}

bool Tmc9660SpiCommInterface::spiTransfer(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept {
    // Ensure SPI is initialized
    if (!spi_interface_.EnsureInitialized()) {
        return false;
    }
    
    // Use the error-returning version and check for success
    hf_spi_err_t result = spi_interface_.Transfer(tx.data(), rx.data(), hf_u16_t(8), hf_u32_t(0));
    return result == hf_spi_err_t::SPI_SUCCESS;
}

// UART Communication Interface Implementation  
Tmc9660UartCommInterface::Tmc9660UartCommInterface(BaseUart& uart_interface) noexcept
    : uart_interface_(uart_interface) {}

bool Tmc9660UartCommInterface::sendUartDatagram(const std::array<uint8_t, 9>& data) noexcept {
    // Ensure UART is initialized
    if (!uart_interface_.EnsureInitialized()) {
        return false;
    }
    
    // Send the 9-byte datagram
    hf_uart_err_t result = uart_interface_.Write(data.data(), 9);
    return result == hf_uart_err_t::UART_SUCCESS;
}

bool Tmc9660UartCommInterface::receiveUartDatagram(std::array<uint8_t, 9>& data) noexcept {
    // Ensure UART is initialized
    if (!uart_interface_.EnsureInitialized()) {
        return false;
    }
    
    // Read the 9-byte datagram with timeout
    hf_uart_err_t result = uart_interface_.Read(data.data(), 9, 1000); // 1 second timeout
    return result == hf_uart_err_t::UART_SUCCESS;
}

// ==================== Tmc9660Handler Implementation ====================

Tmc9660Handler::Tmc9660Handler(BaseSpi& spi_interface, uint8_t address,
                               const tmc9660::BootloaderConfig* bootCfg)
    : spi_ref_(&spi_interface),
      uart_ref_(nullptr),
      active_comm_interface_(nullptr),
      tmc9660_(nullptr), // Will be created in Initialize()
      bootCfg_(bootCfg),
      device_address_(address) {
    // Interfaces will be created lazily in Initialize()
}

Tmc9660Handler::Tmc9660Handler(BaseUart& uart_interface, uint8_t address,
                               const tmc9660::BootloaderConfig* bootCfg)
    : spi_ref_(nullptr),
      uart_ref_(&uart_interface),
      active_comm_interface_(nullptr),
      tmc9660_(nullptr), // Will be created in Initialize()
      bootCfg_(bootCfg),
      device_address_(address) {
    // Interfaces will be created lazily in Initialize()
}

Tmc9660Handler::Tmc9660Handler(BaseSpi& spi_interface, BaseUart& uart_interface, uint8_t address,
                               const tmc9660::BootloaderConfig* bootCfg)
    : spi_ref_(&spi_interface),
      uart_ref_(&uart_interface),
      active_comm_interface_(nullptr),
      tmc9660_(nullptr), // Will be created in Initialize()
      bootCfg_(bootCfg),
      device_address_(address) {
    // Interfaces will be created lazily in Initialize()
    // SPI takes precedence when both are available
}

Tmc9660Handler::~Tmc9660Handler() = default;

TMC9660CommInterface* Tmc9660Handler::InitializeCommInterface() {
    // Prefer SPI if available, fallback to UART
    if (comm_interface_spi_) {
        return comm_interface_spi_.get();
    } else if (comm_interface_uart_) {
        return comm_interface_uart_.get();
    }
    return nullptr;
}

bool Tmc9660Handler::Initialize() {
    // Create communication interfaces only when needed
    if (spi_ref_ && !comm_interface_spi_) {
        comm_interface_spi_ = std::make_unique<Tmc9660SpiCommInterface>(*spi_ref_);
        if (!active_comm_interface_) {
            active_comm_interface_ = comm_interface_spi_.get();
        }
    }
    
    if (uart_ref_ && !comm_interface_uart_) {
        comm_interface_uart_ = std::make_unique<Tmc9660UartCommInterface>(*uart_ref_);
        if (!active_comm_interface_) {
            active_comm_interface_ = comm_interface_uart_.get();
        }
    }
    
    // SPI takes precedence if both are available
    if (spi_ref_ && comm_interface_spi_) {
        active_comm_interface_ = comm_interface_spi_.get();
    }
    
    if (!active_comm_interface_) {
        return false; // No valid communication interface
    }
    
    // Create TMC9660 driver instance with active interface
    if (!tmc9660_) {
        tmc9660_ = std::make_shared<TMC9660>(*active_comm_interface_, device_address_, bootCfg_);
    }
    
    // Create GPIO and ADC wrappers if not already created
    if (!gpioWrappers_[0]) {
        gpioWrappers_[0] = std::make_unique<Gpio>(*this, 17);
        gpioWrappers_[1] = std::make_unique<Gpio>(*this, 18);
        adcWrapper_ = std::make_unique<Adc>(*this);
        temperatureWrapper_ = std::make_unique<Temperature>(*this);
    }
    
    return tmc9660_->bootloaderInit(bootCfg_) == TMC9660::BootloaderInitResult::Success;
}

CommMode Tmc9660Handler::GetCommMode() const noexcept {
    if (active_comm_interface_) {
        return active_comm_interface_->mode();
    }
    return CommMode::SPI; // Default fallback
}

bool Tmc9660Handler::SwitchCommInterface(CommMode mode) {
    switch (mode) {
        case CommMode::SPI:
            if (comm_interface_spi_) {
                active_comm_interface_ = comm_interface_spi_.get();
                return true;
            }
            break;
        case CommMode::UART:
            if (comm_interface_uart_) {
                active_comm_interface_ = comm_interface_uart_.get();
                return true;
            }
            break;
    }
    return false;
}

Tmc9660Handler::Gpio& Tmc9660Handler::gpio(uint8_t gpioNumber) {
    if (gpioNumber == 17) return *gpioWrappers_[0];
    if (gpioNumber == 18) return *gpioWrappers_[1];
    // Return first GPIO as fallback for invalid numbers (no exceptions)
    return *gpioWrappers_[0];
}

Tmc9660Handler::Adc& Tmc9660Handler::adc() {
    return *adcWrapper_;
}

Tmc9660Handler::Temperature& Tmc9660Handler::temperature() {
    return *temperatureWrapper_;
}

// ==================== Gpio Wrapper Implementation ====================

Tmc9660Handler::Gpio::Gpio(Tmc9660Handler& parent, uint8_t gpioNumber)
    : BaseGpio(gpioNumber, hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT),
      parent_(parent), gpioNumber_(gpioNumber) {
    std::snprintf(description_, sizeof(description_), "TMC9660 GPIO%u", gpioNumber_);
}

bool Tmc9660Handler::Gpio::Initialize() noexcept {
    if (!parent_.tmc9660_) {
        return false; // Handler not initialized
    }
    // Set as output, no pull, active high
    return parent_.tmc9660_->gpio.setMode(gpioNumber_, true, false, true);
}

bool Tmc9660Handler::Gpio::Deinitialize() noexcept {
    // No hardware deinit needed
    return true;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetPinLevelImpl(hf_gpio_level_t level) noexcept {
    if (!Initialize()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    bool pin_high = (level == hf_gpio_level_t::HF_GPIO_LEVEL_HIGH);
    if (!parent_.tmc9660_ || !parent_.tmc9660_->gpio.writePin(gpioNumber_, pin_high)) {
        return hf_gpio_err_t::GPIO_ERR_WRITE_FAILURE;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::GetPinLevelImpl(hf_gpio_level_t& level) noexcept {
    if (!Initialize()) return hf_gpio_err_t::GPIO_ERR_NOT_INITIALIZED;
    
    bool pin_state;
    if (!parent_.tmc9660_ || !parent_.tmc9660_->gpio.readDigital(gpioNumber_, pin_state)) {
        return hf_gpio_err_t::GPIO_ERR_READ_FAILURE;
    }
    
    level = pin_state ? hf_gpio_level_t::HF_GPIO_LEVEL_HIGH : hf_gpio_level_t::HF_GPIO_LEVEL_LOW;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetDirectionImpl(hf_gpio_direction_t direction) noexcept {
    // TMC9660 GPIO pins are output-only, so we only support output direction
    if (direction != hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT) {
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept {
    // TMC9660 GPIO pins are push-pull only
    if (mode != hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL) {
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept {
    // TMC9660 GPIO pins don't support pull resistors
    if (mode != hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING) {
        return hf_gpio_err_t::GPIO_ERR_UNSUPPORTED_OPERATION;
    }
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_pull_mode_t Tmc9660Handler::Gpio::GetPullModeImpl() const noexcept {
    // TMC9660 GPIO pins are always floating (no pull resistors)
    return hf_gpio_pull_mode_t::HF_GPIO_PULL_MODE_FLOATING;
}

bool Tmc9660Handler::Gpio::IsPinAvailable() const noexcept {
    return gpioNumber_ == 17 || gpioNumber_ == 18;
}

hf_u8_t Tmc9660Handler::Gpio::GetMaxPins() const noexcept {
    return 2;
}

const char* Tmc9660Handler::Gpio::GetDescription() const noexcept {
    return description_;
}

hf_gpio_err_t Tmc9660Handler::Gpio::GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept {
    // TMC9660 GPIO pins are always output-only
    direction = hf_gpio_direction_t::HF_GPIO_DIRECTION_OUTPUT;
    return hf_gpio_err_t::GPIO_SUCCESS;
}

hf_gpio_err_t Tmc9660Handler::Gpio::GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept {
    // TMC9660 GPIO pins are always push-pull
    mode = hf_gpio_output_mode_t::HF_GPIO_OUTPUT_MODE_PUSH_PULL;
    return hf_gpio_err_t::GPIO_SUCCESS;
}


// ==================== Adc Wrapper Implementation ====================

Tmc9660Handler::Adc::Adc(Tmc9660Handler& parent) : parent_(parent) {}

bool Tmc9660Handler::Adc::Initialize() noexcept {
    // No hardware init needed for TMC9660 internal ADC
    return true;
}

bool Tmc9660Handler::Adc::Deinitialize() noexcept {
    // No hardware deinit needed
    return true;
}

hf_u8_t Tmc9660Handler::Adc::GetMaxChannels() const noexcept {
    return 4; // ADC_I0, ADC_I1, ADC_I2, ADC_I3
}

bool Tmc9660Handler::Adc::IsChannelAvailable(hf_channel_id_t channel_id) const noexcept {
    return (channel_id < 4);
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                                               hf_u8_t numOfSamplesToAvg,
                                               hf_time_t timeBetweenSamples) noexcept {
    hf_u32_t raw = 0;
    hf_adc_err_t err = ReadChannelCount(channel_id, raw, numOfSamplesToAvg, timeBetweenSamples);
    if (err != hf_adc_err_t::ADC_SUCCESS) return err;
    
    // Convert raw ADC value to voltage (adjust scale factor as needed for your hardware)
    channel_reading_v = static_cast<float>(raw) * 3.3f / 65535.0f;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                                   hf_u8_t numOfSamplesToAvg,
                                                   hf_time_t timeBetweenSamples) noexcept {
    RtosMutex::LockGuard lock(mutex_);
    
    const uint64_t start_time_us = GetCurrentTimeUs();
    
    // Validate channel ID
    hf_adc_err_t validation_result = ValidateChannelId(channel_id);
    if (validation_result != hf_adc_err_t::ADC_SUCCESS) {
        UpdateDiagnostics(validation_result);
        return validation_result;
    }
    
    // Route to appropriate channel type handler based on channel ID
    hf_adc_err_t result = hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    
    if (channel_id <= 3) {
        // AIN channels (0-3) - external analog inputs
        float voltage;
        result = ReadAinChannel(channel_id, channel_reading_count, voltage);
    } else if (channel_id >= 10 && channel_id <= 13) {
        // Current sense channels (10-13) - map to I0-I3
        float voltage;
        result = ReadCurrentSenseChannel(channel_id - 10, channel_reading_count, voltage);
    } else if (channel_id >= 20 && channel_id <= 21) {
        // Voltage monitoring channels (20-21)
        float voltage;
        result = ReadVoltageChannel(channel_id - 20, channel_reading_count, voltage);
    } else if (channel_id >= 30 && channel_id <= 31) {
        // Temperature channels (30-31)
        float voltage;
        result = ReadTemperatureChannel(channel_id - 30, channel_reading_count, voltage);
    } else if (channel_id >= 40 && channel_id <= 42) {
        // Motor data channels (40-42)
        float voltage;
        result = ReadMotorDataChannel(channel_id - 40, channel_reading_count, voltage);
    }
    
    // Update statistics
    UpdateStatistics(result, start_time_us);
    
    if (result != hf_adc_err_t::ADC_SUCCESS) {
        UpdateDiagnostics(result);
    }
    
    return result;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                              float& channel_reading_v, hf_u8_t numOfSamplesToAvg,
                                              hf_time_t timeBetweenSamples) noexcept {
    // Read the raw count first
    hf_adc_err_t err = ReadChannelCount(channel_id, channel_reading_count, numOfSamplesToAvg, timeBetweenSamples);
    if (err != hf_adc_err_t::ADC_SUCCESS) {
        return err;
    }
    
    // Convert to voltage
    channel_reading_v = static_cast<float>(channel_reading_count) * 3.3f / 65535.0f;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadMultipleChannels(const hf_channel_id_t* channel_ids, hf_u8_t num_channels,
                                                       hf_u32_t* readings, float* voltages) noexcept {
    RtosMutex::LockGuard lock(mutex_);
    
    if (!channel_ids || !readings || !voltages) {
        UpdateDiagnostics(hf_adc_err_t::ADC_ERR_NULL_POINTER);
        return hf_adc_err_t::ADC_ERR_NULL_POINTER;
    }
    
    const uint64_t start_time_us = GetCurrentTimeUs();
    
    // Validate all channel IDs
    for (hf_u8_t i = 0; i < num_channels; ++i) {
        hf_adc_err_t validation_result = ValidateChannelId(channel_ids[i]);
        if (validation_result != hf_adc_err_t::ADC_SUCCESS) {
            UpdateDiagnostics(validation_result);
            return validation_result;
        }
    }
    
    // Read each channel individually (TMC9660 doesn't support simultaneous reads)
    for (hf_u8_t i = 0; i < num_channels; ++i) {
        hf_adc_err_t result = ReadChannelCount(channel_ids[i], readings[i], 1, 0);
        if (result != hf_adc_err_t::ADC_SUCCESS) {
            UpdateDiagnostics(result);
            return result;
        }
        
        // Convert to voltage
        result = RawToVoltage(readings[i], voltages[i]);
        if (result != hf_adc_err_t::ADC_SUCCESS) {
            UpdateDiagnostics(result);
            return result;
        }
    }
    
    // Update statistics
    UpdateStatistics(hf_adc_err_t::ADC_SUCCESS, start_time_us);
    
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

// Private helper methods
hf_adc_err_t Tmc9660Handler::Adc::ValidateChannelId(hf_channel_id_t channel_id) const noexcept {
    // Validate channel ID ranges for different channel types
    if (channel_id <= 3) {
        // AIN channels (0-3) - external analog inputs
        return hf_adc_err_t::ADC_SUCCESS;
    } else if (channel_id >= 10 && channel_id <= 13) {
        // Current sense channels (10-13) - I0-I3
        return hf_adc_err_t::ADC_SUCCESS;
    } else if (channel_id >= 20 && channel_id <= 21) {
        // Voltage monitoring channels (20-21) - supply, driver
        return hf_adc_err_t::ADC_SUCCESS;
    } else if (channel_id >= 30 && channel_id <= 31) {
        // Temperature channels (30-31) - chip, external
        return hf_adc_err_t::ADC_SUCCESS;
    } else if (channel_id >= 40 && channel_id <= 42) {
        // Motor data channels (40-42) - current, velocity, position
        return hf_adc_err_t::ADC_SUCCESS;
    }
    
    return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
}

hf_adc_err_t Tmc9660Handler::Adc::RawToVoltage(hf_u32_t raw_count, float& voltage) noexcept {
    // Convert raw ADC value to voltage (TMC9660 uses 16-bit ADC)
    voltage = static_cast<float>(raw_count) * 3.3f / 65535.0f;
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::UpdateStatistics(hf_adc_err_t result, uint64_t start_time_us) noexcept {
    const uint64_t end_time_us = GetCurrentTimeUs();
    const uint32_t conversion_time_us = static_cast<uint32_t>(end_time_us - start_time_us);
    
    statistics_.totalConversions++;
    
    if (result == hf_adc_err_t::ADC_SUCCESS) {
        statistics_.successfulConversions++;
        
        // Update timing statistics
        if (statistics_.totalConversions == 1) {
            statistics_.minConversionTimeUs = conversion_time_us;
            statistics_.maxConversionTimeUs = conversion_time_us;
            statistics_.averageConversionTimeUs = conversion_time_us;
        } else {
            statistics_.minConversionTimeUs = std::min(statistics_.minConversionTimeUs, conversion_time_us);
            statistics_.maxConversionTimeUs = std::max(statistics_.maxConversionTimeUs, conversion_time_us);
            
            // Update average (simple moving average)
            const uint32_t total_time = statistics_.averageConversionTimeUs * (statistics_.successfulConversions - 1) + conversion_time_us;
            statistics_.averageConversionTimeUs = total_time / statistics_.successfulConversions;
        }
    } else {
        statistics_.failedConversions++;
        UpdateDiagnostics(result);
    }
    
    return result;
}

uint64_t Tmc9660Handler::Adc::GetCurrentTimeUs() const noexcept {
    // Use OS abstraction time function
    // os_time_get() returns ticks, convert to microseconds using proper tick rate
    OS_Ulong ticks = os_time_get();
    return static_cast<uint64_t>(ticks) * 1000000 / osTickRateHz; // Convert ticks to microseconds
}

void Tmc9660Handler::Adc::UpdateDiagnostics(hf_adc_err_t error) noexcept {
    last_error_.store(error);
    
    if (error != hf_adc_err_t::ADC_SUCCESS) {
        diagnostics_.consecutiveErrors++;
        diagnostics_.lastErrorCode = error;
        diagnostics_.lastErrorTimestamp = GetCurrentTimeUs();
        
        // Mark as unhealthy if too many consecutive errors
        if (diagnostics_.consecutiveErrors > 10) {
            diagnostics_.adcHealthy = false;
        }
    } else {
        diagnostics_.consecutiveErrors = 0;
        diagnostics_.adcHealthy = true;
    }
}

//==============================================//
// TMC9660-SPECIFIC CHANNEL READING METHODS
//==============================================//

hf_adc_err_t Tmc9660Handler::Adc::ReadAinChannel(uint8_t ain_channel, hf_u32_t& raw_value, float& voltage) noexcept {
    if (!parent_.tmc9660_) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;
    }
    
    // Use TMC9660 GPIO readAnalog for AIN channels
    uint16_t analog_value = 0;
    if (!parent_.tmc9660_->gpio.readAnalog(ain_channel, analog_value)) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;
    }
    
    raw_value = static_cast<hf_u32_t>(analog_value);
    voltage = static_cast<float>(analog_value) * 3.3f / 65535.0f; // 16-bit ADC
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadCurrentSenseChannel(uint8_t current_channel, hf_u32_t& raw_value, float& voltage) noexcept {
    if (!parent_.tmc9660_) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;
    }
    
    // Map current channel to TMC9660 ADC parameter
    tmc9660::tmcl::Parameters param;
    switch (current_channel) {
        case 0: param = tmc9660::tmcl::Parameters::ADC_I0; break;
        case 1: param = tmc9660::tmcl::Parameters::ADC_I1; break;
        case 2: param = tmc9660::tmcl::Parameters::ADC_I2; break;
        case 3: param = tmc9660::tmcl::Parameters::ADC_I3; break;
        default: return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    }
    
    uint32_t value = 0;
    if (!parent_.tmc9660_->readParameter(param, value)) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;
    }
    
    raw_value = static_cast<hf_u32_t>(value);
    voltage = static_cast<float>(value) * 3.3f / 65535.0f; // 16-bit ADC
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadVoltageChannel(uint8_t voltage_channel, hf_u32_t& raw_value, float& voltage) noexcept {
    if (!parent_.tmc9660_) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;
    }
    
    switch (voltage_channel) {
        case 0: // Supply voltage
            voltage = parent_.tmc9660_->telemetry.getSupplyVoltage();
            raw_value = static_cast<hf_u32_t>(voltage * 1000.0f); // Convert to millivolts
            break;
        case 1: // Driver voltage (not directly available, use supply voltage as approximation)
            voltage = parent_.tmc9660_->telemetry.getSupplyVoltage();
            raw_value = static_cast<hf_u32_t>(voltage * 1000.0f);
            break;
        default:
            return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    }
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadTemperatureChannel(uint8_t temp_channel, hf_u32_t& raw_value, float& voltage) noexcept {
    if (!parent_.tmc9660_) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;
    }
    
    switch (temp_channel) {
        case 0: // Chip temperature
            {
                float temp_c = parent_.tmc9660_->telemetry.getChipTemperature();
                raw_value = static_cast<hf_u32_t>(temp_c * 100.0f); // Convert to centidegrees
                voltage = temp_c; // Store temperature as voltage for compatibility
            }
            break;
        case 1: // External temperature
            {
                uint16_t ext_temp_raw = parent_.tmc9660_->telemetry.getExternalTemperature();
                raw_value = static_cast<hf_u32_t>(ext_temp_raw);
                voltage = static_cast<float>(ext_temp_raw) * 3.3f / 65535.0f; // Convert to voltage
            }
            break;
        default:
            return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    }
    
    return hf_adc_err_t::ADC_SUCCESS;
}

hf_adc_err_t Tmc9660Handler::Adc::ReadMotorDataChannel(uint8_t motor_channel, hf_u32_t& raw_value, float& voltage) noexcept {
    if (!parent_.tmc9660_) {
        return hf_adc_err_t::ADC_ERR_CHANNEL_READ_ERR;
    }
    
    switch (motor_channel) {
        case 0: // Motor current
            {
                int16_t current_ma = parent_.tmc9660_->telemetry.getMotorCurrent();
                raw_value = static_cast<hf_u32_t>(current_ma);
                voltage = static_cast<float>(current_ma) / 1000.0f; // Convert to amps
            }
            break;
        case 1: // Motor velocity
            {
                int32_t velocity = parent_.tmc9660_->telemetry.getActualVelocity();
                raw_value = static_cast<hf_u32_t>(velocity);
                voltage = static_cast<float>(velocity); // Store as voltage for compatibility
            }
            break;
        case 2: // Motor position
            {
                int32_t position = parent_.tmc9660_->telemetry.getActualPosition();
                raw_value = static_cast<hf_u32_t>(position);
                voltage = static_cast<float>(position); // Store as voltage for compatibility
            }
            break;
        default:
            return hf_adc_err_t::ADC_ERR_INVALID_CHANNEL;
    }
    
    return hf_adc_err_t::ADC_SUCCESS;
}

const char* Tmc9660Handler::Adc::GetChannelTypeString(hf_channel_id_t channel_id) const noexcept {
    if (channel_id <= 3) {
        return "AIN";
    } else if (channel_id >= 10 && channel_id <= 13) {
        return "Current";
    } else if (channel_id >= 20 && channel_id <= 21) {
        return "Voltage";
    } else if (channel_id >= 30 && channel_id <= 31) {
        return "Temperature";
    } else if (channel_id >= 40 && channel_id <= 42) {
        return "Motor";
    }
    return "Unknown";
}

void Tmc9660Handler::DumpDiagnostics() const noexcept {
    static constexpr const char* TAG = "Tmc9660Handler";
    
    Logger::GetInstance().Info(TAG, "=== TMC9660 HANDLER DIAGNOSTICS ===");
    
    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", IsInitialized() ? "YES" : "NO");
    
    // Communication Interface Status
    Logger::GetInstance().Info(TAG, "Communication Interface:");
    Logger::GetInstance().Info(TAG, "  Current Mode: %s", 
        GetCommMode() == CommMode::SPI ? "SPI" : 
        GetCommMode() == CommMode::UART ? "UART" : "UNKNOWN");
    Logger::GetInstance().Info(TAG, "  SPI Available: %s", HasSpiInterface() ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  UART Available: %s", HasUartInterface() ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Device Address: %d", device_address_);
    
    // TMC9660 Driver Status
    Logger::GetInstance().Info(TAG, "TMC9660 Driver:");
    if (tmc9660_) {
        Logger::GetInstance().Info(TAG, "  Driver Instance: ACTIVE");
        // Add more TMC9660-specific diagnostics here if available
    } else {
        Logger::GetInstance().Info(TAG, "  Driver Instance: NOT_INITIALIZED");
    }
    
    // ADC Diagnostics
    Logger::GetInstance().Info(TAG, "ADC Subsystem:");
    if (adcWrapper_) {
        Logger::GetInstance().Info(TAG, "  ADC Wrapper: ACTIVE");
        
        // Get ADC statistics
        hf_adc_statistics_t adc_stats;
        hf_adc_diagnostics_t adc_diagnostics;
        
        if (adcWrapper_->GetStatistics(adc_stats) == hf_adc_err_t::ADC_SUCCESS) {
            Logger::GetInstance().Info(TAG, "  ADC Statistics:");
            Logger::GetInstance().Info(TAG, "    Total Operations: %d", adc_stats.total_operations);
            Logger::GetInstance().Info(TAG, "    Successful Operations: %d", adc_stats.successful_operations);
            Logger::GetInstance().Info(TAG, "    Failed Operations: %d", adc_stats.failed_operations);
            Logger::GetInstance().Info(TAG, "    Average Read Time: %d us", adc_stats.average_read_time_us);
            Logger::GetInstance().Info(TAG, "    Last Read Time: %d us", adc_stats.last_read_time_us);
            
            if (adc_stats.total_operations > 0) {
                float success_rate = (float)adc_stats.successful_operations / adc_stats.total_operations * 100.0f;
                Logger::GetInstance().Info(TAG, "    Success Rate: %.2f%%", success_rate);
            }
        }
        
        if (adcWrapper_->GetDiagnostics(adc_diagnostics) == hf_adc_err_t::ADC_SUCCESS) {
            Logger::GetInstance().Info(TAG, "  ADC Diagnostics:");
            Logger::GetInstance().Info(TAG, "    Communication Errors: %d", adc_diagnostics.communication_errors);
            Logger::GetInstance().Info(TAG, "    Hardware Errors: %d", adc_diagnostics.hardware_errors);
            Logger::GetInstance().Info(TAG, "    Last Error: %s", adc_err_to_string(adc_diagnostics.last_error));
            Logger::GetInstance().Info(TAG, "    System Healthy: %s", adc_diagnostics.system_healthy ? "YES" : "NO");
        }
    } else {
        Logger::GetInstance().Info(TAG, "  ADC Wrapper: NOT_INITIALIZED");
    }
    
    // GPIO Diagnostics
    Logger::GetInstance().Info(TAG, "GPIO Subsystem:");
    int active_gpio_count = 0;
    for (size_t i = 0; i < gpioWrappers_.size(); ++i) {
        if (gpioWrappers_[i]) {
            active_gpio_count++;
            Logger::GetInstance().Info(TAG, "  GPIO%d: ACTIVE", static_cast<int>(i + 17)); // GPIO17, GPIO18
        }
    }
    Logger::GetInstance().Info(TAG, "  Active GPIO Count: %d/%d", active_gpio_count, static_cast<int>(gpioWrappers_.size()));
    
    // Bootloader Configuration
    Logger::GetInstance().Info(TAG, "Bootloader Configuration:");
    if (bootCfg_) {
        Logger::GetInstance().Info(TAG, "  Boot Mode: %s", 
            bootCfg_->boot.boot_mode == tmc9660::bootcfg::BootMode::Parameter ? "Parameter" :
            bootCfg_->boot.boot_mode == tmc9660::bootcfg::BootMode::Standalone ? "Standalone" : "Unknown");
        Logger::GetInstance().Info(TAG, "  VEXT1: %s", 
            bootCfg_->ldo.vext1 == tmc9660::bootcfg::LDOVoltage::V5_0 ? "5.0V" :
            bootCfg_->ldo.vext1 == tmc9660::bootcfg::LDOVoltage::V3_3 ? "3.3V" : "Other");
        Logger::GetInstance().Info(TAG, "  VEXT2: %s", 
            bootCfg_->ldo.vext2 == tmc9660::bootcfg::LDOVoltage::V5_0 ? "5.0V" :
            bootCfg_->ldo.vext2 == tmc9660::bootcfg::LDOVoltage::V3_3 ? "3.3V" : "Other");
    } else {
        Logger::GetInstance().Info(TAG, "  Configuration: NOT_AVAILABLE");
    }
    
    // Temperature Diagnostics
    Logger::GetInstance().Info(TAG, "Temperature Subsystem:");
    if (temperatureWrapper_) {
        Logger::GetInstance().Info(TAG, "  Temperature Wrapper: ACTIVE");
        
        // Get temperature statistics
        hf_temp_statistics_t temp_stats;
        hf_temp_diagnostics_t temp_diagnostics;
        
        if (temperatureWrapper_->GetStatistics(temp_stats) == hf_temp_err_t::TEMP_SUCCESS) {
            Logger::GetInstance().Info(TAG, "  Temperature Statistics:");
            Logger::GetInstance().Info(TAG, "    Total Operations: %d", temp_stats.total_operations);
            Logger::GetInstance().Info(TAG, "    Successful Operations: %d", temp_stats.successful_operations);
            Logger::GetInstance().Info(TAG, "    Failed Operations: %d", temp_stats.failed_operations);
            Logger::GetInstance().Info(TAG, "    Temperature Readings: %d", temp_stats.temperature_readings);
            Logger::GetInstance().Info(TAG, "    Average Operation Time: %d us", temp_stats.average_operation_time_us);
            Logger::GetInstance().Info(TAG, "    Min Temperature: %.2f°C", temp_stats.min_temperature_celsius);
            Logger::GetInstance().Info(TAG, "    Max Temperature: %.2f°C", temp_stats.max_temperature_celsius);
            Logger::GetInstance().Info(TAG, "    Average Temperature: %.2f°C", temp_stats.avg_temperature_celsius);
            
            if (temp_stats.total_operations > 0) {
                float success_rate = (float)temp_stats.successful_operations / temp_stats.total_operations * 100.0f;
                Logger::GetInstance().Info(TAG, "    Success Rate: %.2f%%", success_rate);
            }
        }
        
        if (temperatureWrapper_->GetDiagnostics(temp_diagnostics) == hf_temp_err_t::TEMP_SUCCESS) {
            Logger::GetInstance().Info(TAG, "  Temperature Diagnostics:");
            Logger::GetInstance().Info(TAG, "    Sensor Healthy: %s", temp_diagnostics.sensor_healthy ? "YES" : "NO");
            Logger::GetInstance().Info(TAG, "    Sensor Available: %s", temp_diagnostics.sensor_available ? "YES" : "NO");
            Logger::GetInstance().Info(TAG, "    Last Error: %s", GetTempErrorString(temp_diagnostics.last_error_code));
            Logger::GetInstance().Info(TAG, "    Consecutive Errors: %d", temp_diagnostics.consecutive_errors);
            Logger::GetInstance().Info(TAG, "    Current Raw Temperature: %d", temp_diagnostics.current_temperature_raw);
        }
        
        // Get current temperature reading
        float current_temp = 0.0f;
        if (temperatureWrapper_->ReadTemperatureCelsius(&current_temp) == hf_temp_err_t::TEMP_SUCCESS) {
            Logger::GetInstance().Info(TAG, "  Current Temperature: %.2f°C", current_temp);
        } else {
            Logger::GetInstance().Info(TAG, "  Current Temperature: READ_ERROR");
        }
    } else {
        Logger::GetInstance().Info(TAG, "  Temperature Wrapper: NOT_INITIALIZED");
    }
    
    // Memory Usage
    Logger::GetInstance().Info(TAG, "Memory Usage:");
    size_t estimated_memory = sizeof(*this);
    if (tmc9660_) estimated_memory += sizeof(TMC9660);
    if (adcWrapper_) estimated_memory += sizeof(Adc);
    if (temperatureWrapper_) estimated_memory += sizeof(Temperature);
    for (const auto& gpio : gpioWrappers_) {
        if (gpio) estimated_memory += sizeof(Gpio);
    }
    Logger::GetInstance().Info(TAG, "  Estimated Total: %d bytes", static_cast<int>(estimated_memory));
    
    Logger::GetInstance().Info(TAG, "=== END TMC9660 HANDLER DIAGNOSTICS ===");
} 

//==============================================================//
// TEMPERATURE WRAPPER IMPLEMENTATION
//==============================================================//

Tmc9660Handler::Temperature::Temperature(Tmc9660Handler& parent) 
    : parent_(parent), last_error_(hf_temp_err_t::TEMP_SUCCESS) {
    // Initialize statistics and diagnostics
    statistics_ = hf_temp_statistics_t{};
    diagnostics_ = hf_temp_diagnostics_t{};
}

bool Tmc9660Handler::Temperature::Initialize() noexcept {
    static constexpr const char* TAG = "Tmc9660Handler::Temperature";
    
    if (IsInitialized()) {
        Logger::GetInstance().Warning(TAG, "Temperature sensor already initialized");
        return true;
    }
    
    // Check if parent TMC9660 is ready
    if (!parent_.IsDriverReady()) {
        Logger::GetInstance().Error(TAG, "Parent TMC9660 driver not ready");
        UpdateDiagnostics(hf_temp_err_t::TEMP_ERR_NOT_INITIALIZED);
        return false;
    }
    
    Logger::GetInstance().Info(TAG, "Temperature sensor initialized successfully");
    return true;
}

bool Tmc9660Handler::Temperature::Deinitialize() noexcept {
    static constexpr const char* TAG = "Tmc9660Handler::Temperature";
    
    if (!IsInitialized()) {
        Logger::GetInstance().Warning(TAG, "Temperature sensor not initialized");
        return true;
    }
    
    Logger::GetInstance().Info(TAG, "Temperature sensor deinitialized");
    return true;
}

hf_temp_err_t Tmc9660Handler::Temperature::ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept {
    static constexpr const char* TAG = "Tmc9660Handler::Temperature";
    
    if (temperature_celsius == nullptr) {
        UpdateDiagnostics(hf_temp_err_t::TEMP_ERR_NULL_POINTER);
        return hf_temp_err_t::TEMP_ERR_NULL_POINTER;
    }
    
    RtosMutex::LockGuard lock(mutex_);
    uint64_t start_time_us = GetCurrentTimeUs();
    
    // Check if parent TMC9660 is ready
    if (!parent_.IsDriverReady()) {
        Logger::GetInstance().Error(TAG, "Parent TMC9660 driver not ready");
        UpdateDiagnostics(hf_temp_err_t::TEMP_ERR_NOT_INITIALIZED);
        return hf_temp_err_t::TEMP_ERR_NOT_INITIALIZED;
    }
    
    // Read temperature from TMC9660 using the telemetry interface
    float temp_c = parent_.tmc9660_->telemetry.getChipTemperature();
    
    // Check for error condition (TMC9660 returns -273.0f on error)
    if (temp_c < -270.0f) {
        Logger::GetInstance().Error(TAG, "Failed to read chip temperature from TMC9660");
        UpdateDiagnostics(hf_temp_err_t::TEMP_ERR_READ_FAILED);
        return hf_temp_err_t::TEMP_ERR_READ_FAILED;
    }
    
    // Validate temperature range (reasonable range for chip temperature)
    if (temp_c < -40.0f || temp_c > 150.0f) {
        Logger::GetInstance().Warning(TAG, "Temperature reading out of expected range: %.2f°C", temp_c);
        UpdateDiagnostics(hf_temp_err_t::TEMP_ERR_OUT_OF_RANGE);
        return hf_temp_err_t::TEMP_ERR_OUT_OF_RANGE;
    }
    
    *temperature_celsius = temp_c;
    
    // Update statistics and diagnostics
    UpdateStatistics(hf_temp_err_t::TEMP_SUCCESS, start_time_us);
    UpdateDiagnostics(hf_temp_err_t::TEMP_SUCCESS);
    
    Logger::GetInstance().Debug(TAG, "Temperature read successfully: %.2f°C", temp_c);
    return hf_temp_err_t::TEMP_SUCCESS;
}

hf_temp_err_t Tmc9660Handler::Temperature::GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept {
    static constexpr const char* TAG = "Tmc9660Handler::Temperature";
    
    if (info == nullptr) {
        return hf_temp_err_t::TEMP_ERR_NULL_POINTER;
    }
    
    // Fill in TMC9660 internal temperature sensor information
    info->sensor_type = HF_TEMP_SENSOR_TYPE_INTERNAL;
    info->min_temp_celsius = -40.0f;  // Typical chip temperature range
    info->max_temp_celsius = 150.0f;  // Typical chip temperature range
    info->resolution_celsius = 0.1f;  // Approximate resolution
    info->accuracy_celsius = 2.0f;    // Typical accuracy for chip temperature sensors
    info->response_time_ms = 100;     // Typical response time
    info->capabilities = HF_TEMP_CAP_HIGH_PRECISION | HF_TEMP_CAP_FAST_RESPONSE;
    info->manufacturer = "Trinamic";
    info->model = "TMC9660";
    info->version = "Internal Chip Temperature Sensor";
    
    return hf_temp_err_t::TEMP_SUCCESS;
}

hf_u32_t Tmc9660Handler::Temperature::GetCapabilities() const noexcept {
    return HF_TEMP_CAP_HIGH_PRECISION | HF_TEMP_CAP_FAST_RESPONSE;
}

hf_temp_err_t Tmc9660Handler::Temperature::UpdateStatistics(hf_temp_err_t result, uint64_t start_time_us) noexcept {
    uint64_t end_time_us = GetCurrentTimeUs();
    uint32_t operation_time_us = static_cast<uint32_t>(end_time_us - start_time_us);
    
    statistics_.total_operations++;
    
    if (result == hf_temp_err_t::TEMP_SUCCESS) {
        statistics_.successful_operations++;
        statistics_.temperature_readings++;
        
        // Update min/max temperature tracking
        float current_temp = 0.0f;
        if (ReadTemperatureCelsius(&current_temp) == hf_temp_err_t::TEMP_SUCCESS) {
            if (statistics_.total_operations == 1) {
                statistics_.min_temperature_celsius = current_temp;
                statistics_.max_temperature_celsius = current_temp;
            } else {
                if (current_temp < statistics_.min_temperature_celsius) {
                    statistics_.min_temperature_celsius = current_temp;
                }
                if (current_temp > statistics_.max_temperature_celsius) {
                    statistics_.max_temperature_celsius = current_temp;
                }
            }
            
            // Update average temperature (simple moving average)
            if (statistics_.temperature_readings == 1) {
                statistics_.avg_temperature_celsius = current_temp;
            } else {
                statistics_.avg_temperature_celsius = 
                    (statistics_.avg_temperature_celsius * (statistics_.temperature_readings - 1) + current_temp) / 
                    statistics_.temperature_readings;
            }
        }
    } else {
        statistics_.failed_operations++;
    }
    
    // Update timing statistics
    if (statistics_.total_operations == 1) {
        statistics_.min_operation_time_us = operation_time_us;
        statistics_.max_operation_time_us = operation_time_us;
        statistics_.average_operation_time_us = operation_time_us;
    } else {
        if (operation_time_us < statistics_.min_operation_time_us) {
            statistics_.min_operation_time_us = operation_time_us;
        }
        if (operation_time_us > statistics_.max_operation_time_us) {
            statistics_.max_operation_time_us = operation_time_us;
        }
        
        // Update average operation time
        statistics_.average_operation_time_us = 
            (statistics_.average_operation_time_us * (statistics_.total_operations - 1) + operation_time_us) / 
            statistics_.total_operations;
    }
    
    return result;
}

uint64_t Tmc9660Handler::Temperature::GetCurrentTimeUs() const noexcept {
    return OsAbstraction::GetTimeUs();
}

void Tmc9660Handler::Temperature::UpdateDiagnostics(hf_temp_err_t error) noexcept {
    diagnostics_.last_error_code = error;
    diagnostics_.last_error_timestamp = static_cast<hf_u32_t>(GetCurrentTimeUs() / 1000); // Convert to ms
    
    if (error != hf_temp_err_t::TEMP_SUCCESS) {
        diagnostics_.consecutive_errors++;
        diagnostics_.sensor_healthy = false;
    } else {
        diagnostics_.consecutive_errors = 0;
        diagnostics_.sensor_healthy = true;
    }
    
    // Update sensor availability based on parent TMC9660 status
    diagnostics_.sensor_available = parent_.IsDriverReady();
    
    // Update current raw temperature reading
    float current_temp = 0.0f;
    if (ReadTemperatureCelsius(&current_temp) == hf_temp_err_t::TEMP_SUCCESS) {
        diagnostics_.current_temperature_raw = static_cast<hf_u32_t>(current_temp * 100); // Store as 0.01°C units
    }
    
    last_error_.store(error);
} 