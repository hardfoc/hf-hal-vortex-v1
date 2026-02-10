#ifndef COMPONENT_HANDLER_TMC9660_HANDLER_H_
#define COMPONENT_HANDLER_TMC9660_HANDLER_H_

#include <cstdint>
#include <memory>
#include <array>
#include "utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc/TMC9660.hpp"
#include "utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc/TMC9660CommInterface.hpp"
#include "base/BaseGpio.h"
#include "base/BaseAdc.h"
#include "base/BaseTemperature.h"
#include "base/BaseSpi.h"
#include "base/BaseUart.h"

/**
 * @brief Concrete SPI communication implementation using BaseSpi interface.
 * 
 * This class bridges the TMC9660 SPITMC9660CommInterface with the HardFOC BaseSpi interface,
 * allowing the TMC9660 driver to communicate over any SPI implementation that inherits from BaseSpi.
 */
class Tmc9660SpiCommInterface : public SPITMC9660CommInterface {
public:
    /**
     * @brief Constructor that takes a BaseSpi interface.
     * @param spi_interface Reference to BaseSpi implementation
     */
    explicit Tmc9660SpiCommInterface(BaseSpi& spi_interface) noexcept;

    /**
     * @brief Perform 8-byte SPI transfer using BaseSpi interface.
     * @param tx Buffer containing 8 bytes to transmit
     * @param rx Buffer to receive 8 bytes from device
     * @return true if SPI transfer completed successfully
     */
    bool spiTransfer(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept override;

private:
    BaseSpi& spi_interface_;  ///< Reference to the BaseSpi implementation
};

/**
 * @brief Concrete UART communication implementation using BaseUart interface.
 * 
 * This class bridges the TMC9660 UARTTMC9660CommInterface with the HardFOC BaseUart interface,
 * allowing the TMC9660 driver to communicate over any UART implementation that inherits from BaseUart.
 */
class Tmc9660UartCommInterface : public UARTTMC9660CommInterface {
public:
    /**
     * @brief Constructor that takes a BaseUart interface.
     * @param uart_interface Reference to BaseUart implementation
     */
    explicit Tmc9660UartCommInterface(BaseUart& uart_interface) noexcept;

    /**
     * @brief Send 9-byte UART TMCL datagram using BaseUart interface.
     * @param data Array of 9 bytes including sync, fields, and checksum
     * @return true if transmission succeeded
     */
    bool sendUartDatagram(const std::array<uint8_t, 9>& data) noexcept override;

    /**
     * @brief Receive 9-byte UART TMCL datagram using BaseUart interface.
     * @param data Array to store 9 received bytes
     * @return true if reception succeeded
     */
    bool receiveUartDatagram(std::array<uint8_t, 9>& data) noexcept override;

private:
    BaseUart& uart_interface_;  ///< Reference to the BaseUart implementation
};

/**
 * @file Tmc9660Handler.h
 * @brief Unified handler for TMC9660 motor controller, GPIO, and ADC integration.
 *
 * This class provides a modern, unified interface for a single TMC9660 device.
 * - Accepts BaseSpi and/or BaseUart interfaces (at least one required)
 * - Creates appropriate TMC9660CommInterface internally
 * - Owns a TMC9660 driver instance
 * - Exposes BaseGpio and BaseAdc compatible wrappers for internal GPIO/ADC
 * - Exposes the TMC9660 instance for advanced parameter access
 * - Accepts a bootloader config (defaulted, but user-overridable)
 * - Handles initialization, error reporting, and safe access patterns
 * - Designed for single-device use (multi-device can be handled by user if needed)
 */
class Tmc9660Handler {
public:
    /**
     * @brief Default bootloader configuration (can be overridden at construction).
     *        These values should be tuned for your hardware.
     */
    static const tmc9660::BootloaderConfig kDefaultBootConfig;

    /**
     * @brief Construct a Tmc9660Handler with SPI interface only.
     * @param spi_interface Reference to BaseSpi implementation
     * @param address 7-bit device address
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @note Communication interfaces are created lazily during Initialize() to save memory
     */
    Tmc9660Handler(BaseSpi& spi_interface, uint8_t address,
                  const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig);

    /**
     * @brief Construct a Tmc9660Handler with UART interface only.
     * @param uart_interface Reference to BaseUart implementation
     * @param address 7-bit device address
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @note Communication interfaces are created lazily during Initialize() to save memory
     */
    Tmc9660Handler(BaseUart& uart_interface, uint8_t address,
                  const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig);

    /**
     * @brief Construct a Tmc9660Handler with both SPI and UART interfaces.
     * @param spi_interface Reference to BaseSpi implementation
     * @param uart_interface Reference to BaseUart implementation
     * @param address 7-bit device address
     * @param bootCfg Optional bootloader config (defaults to kDefaultBootConfig)
     * @note SPI interface takes precedence for communication
     * @note Communication interfaces are created lazily during Initialize() to save memory
     */
    Tmc9660Handler(BaseSpi& spi_interface, BaseUart& uart_interface, uint8_t address,
                  const tmc9660::BootloaderConfig* bootCfg = &kDefaultBootConfig);

    ~Tmc9660Handler();

    /**
     * @brief Initialize the TMC9660 device (bootloader, parameter mode, etc).
     * @return true if successful, false otherwise
     */
    bool Initialize();

    /**
     * @brief Get shared access to the underlying TMC9660 driver instance.
     *        Use this for advanced parameter access and tweaking.
     * @note Driver is created lazily in Initialize()
     * @note Returns shared_ptr for safe lifetime management - you can store the 
     *       returned pointer and it will remain valid even if the handler is destroyed
     * @return Shared pointer to TMC9660 driver or nullptr if not initialized
     */
    std::shared_ptr<TMC9660> driver() noexcept { 
        return tmc9660_;
    }

    /**
     * @brief Get shared access to the underlying TMC9660 driver instance (const version).
     * @note Driver is created lazily in Initialize()
     * @return Shared pointer to TMC9660 driver or nullptr if not initialized
     */
    std::shared_ptr<const TMC9660> driver() const noexcept { 
        return tmc9660_;
    }

    /**
     * @brief Check if the TMC9660 driver is initialized and ready for use.
     * @return true if driver is available, false otherwise
     */
    bool IsDriverReady() const noexcept {
        return static_cast<bool>(tmc9660_);
    }

    /**
     * @brief GPIO wrapper for TMC9660 internal GPIO channels.
     *        Implements BaseGpio interface.
     */
    class Gpio : public BaseGpio {
    public:
        Gpio(Tmc9660Handler& parent, uint8_t gpioNumber);
        ~Gpio() override = default;
        
        // Pure virtual methods from BaseGpio
        bool Initialize() noexcept override;
        bool Deinitialize() noexcept override;
        bool IsPinAvailable() const noexcept override;
        hf_u8_t GetMaxPins() const noexcept override;
        const char* GetDescription() const noexcept override;
        
        // Protected implementation methods from BaseGpio
        hf_gpio_err_t SetDirectionImpl(hf_gpio_direction_t direction) noexcept override;
        hf_gpio_err_t SetOutputModeImpl(hf_gpio_output_mode_t mode) noexcept override;
        hf_gpio_err_t SetPullModeImpl(hf_gpio_pull_mode_t mode) noexcept override;
        hf_gpio_err_t SetPinLevelImpl(hf_gpio_level_t level) noexcept override;
        hf_gpio_err_t GetPinLevelImpl(hf_gpio_level_t& level) noexcept override;
        hf_gpio_pull_mode_t GetPullModeImpl() const noexcept override;
        hf_gpio_err_t GetDirectionImpl(hf_gpio_direction_t& direction) const noexcept override;
        hf_gpio_err_t GetOutputModeImpl(hf_gpio_output_mode_t& mode) const noexcept override;
        
    private:
        Tmc9660Handler& parent_;
        uint8_t gpioNumber_;
        char description_[32];
    };

    /**
     * @brief Enhanced ADC wrapper for TMC9660 ADC channels.
     *        Implements BaseAdc interface and supports all TMC9660 ADC channel types.
     * 
     * @details This class provides access to:
     * - AIN channels (0-3) for external analog inputs
     * - Current sensing channels (I0-I3) 
     * - Voltage monitoring channels (supply, driver)
     * - Temperature channels (chip, external)
     * - Motor control data channels (current, velocity, position)
     */
    class Adc : public BaseAdc {
    public:
        Adc(Tmc9660Handler& parent);
        ~Adc() override = default;
        bool Initialize() noexcept override;
        bool Deinitialize() noexcept override;
        hf_u8_t GetMaxChannels() const noexcept override;
        bool IsChannelAvailable(hf_channel_id_t channel_id) const noexcept override;
        hf_adc_err_t ReadChannelV(hf_channel_id_t channel_id, float& channel_reading_v,
                                 hf_u8_t numOfSamplesToAvg = 1,
                                 hf_time_t timeBetweenSamples = 0) noexcept override;
        hf_adc_err_t ReadChannelCount(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                                     hf_u8_t numOfSamplesToAvg = 1,
                                     hf_time_t timeBetweenSamples = 0) noexcept override;
        hf_adc_err_t ReadChannel(hf_channel_id_t channel_id, hf_u32_t& channel_reading_count,
                               float& channel_reading_v, hf_u8_t numOfSamplesToAvg = 1,
                               hf_time_t timeBetweenSamples = 0) noexcept override;
        hf_adc_err_t ReadMultipleChannels(const hf_channel_id_t* channel_ids, hf_u8_t num_channels,
                                         hf_u32_t* readings, float* voltages) noexcept override;
        hf_adc_err_t GetStatistics(hf_adc_statistics_t& statistics) noexcept override;
        hf_adc_err_t GetDiagnostics(hf_adc_diagnostics_t& diagnostics) noexcept override;
        hf_adc_err_t ResetStatistics() noexcept override;
        hf_adc_err_t ResetDiagnostics() noexcept override;

        //==============================================//
        // TMC9660-SPECIFIC CHANNEL READING METHODS
        //==============================================//

        /**
         * @brief Read AIN channel value (external analog input).
         * @param ain_channel AIN channel number (0-3)
         * @param raw_value Reference to store raw value
         * @param voltage Reference to store voltage
         * @return Error code
         */
        hf_adc_err_t ReadAinChannel(uint8_t ain_channel, hf_u32_t& raw_value, float& voltage) noexcept;

        /**
         * @brief Read current sense channel value.
         * @param current_channel Current channel number (0-3)
         * @param raw_value Reference to store raw value
         * @param voltage Reference to store voltage
         * @return Error code
         */
        hf_adc_err_t ReadCurrentSenseChannel(uint8_t current_channel, hf_u32_t& raw_value, float& voltage) noexcept;

        /**
         * @brief Read voltage monitoring channel value.
         * @param voltage_channel Voltage channel number (0=supply, 1=driver)
         * @param raw_value Reference to store raw value
         * @param voltage Reference to store voltage
         * @return Error code
         */
        hf_adc_err_t ReadVoltageChannel(uint8_t voltage_channel, hf_u32_t& raw_value, float& voltage) noexcept;

        /**
         * @brief Read temperature channel value.
         * @param temp_channel Temperature channel number (0=chip, 1=external)
         * @param raw_value Reference to store raw value
         * @param voltage Reference to store voltage
         * @return Error code
         */
        hf_adc_err_t ReadTemperatureChannel(uint8_t temp_channel, hf_u32_t& raw_value, float& voltage) noexcept;

        /**
         * @brief Read motor data channel value.
         * @param motor_channel Motor channel number (0=current, 1=velocity, 2=position)
         * @param raw_value Reference to store raw value
         * @param voltage Reference to store voltage
         * @return Error code
         */
        hf_adc_err_t ReadMotorDataChannel(uint8_t motor_channel, hf_u32_t& raw_value, float& voltage) noexcept;

    private:
        Tmc9660Handler& parent_;
        mutable RtosMutex mutex_;
        mutable hf_adc_statistics_t statistics_;
        mutable hf_adc_diagnostics_t diagnostics_;
        std::atomic<hf_adc_err_t> last_error_;
        
        hf_adc_err_t ValidateChannelId(hf_channel_id_t channel_id) const noexcept;
        hf_adc_err_t RawToVoltage(hf_u32_t raw_count, float& voltage) noexcept;
        hf_adc_err_t UpdateStatistics(hf_adc_err_t result, uint64_t start_time_us) noexcept;
        uint64_t GetCurrentTimeUs() const noexcept;
        void UpdateDiagnostics(hf_adc_err_t error) noexcept;

        /**
         * @brief Determine channel type from channel ID.
         * @param channel_id Channel ID to analyze
         * @return Channel type string for logging
         */
        const char* GetChannelTypeString(hf_channel_id_t channel_id) const noexcept;
    };

    /**
     * @brief Temperature wrapper for TMC9660 internal temperature sensor.
     *        Implements BaseTemperature interface.
     * 
     * @details This class provides access to the TMC9660's internal chip temperature sensor
     * through the unified BaseTemperature interface. The TMC9660 has an internal temperature
     * sensor that can be read via the CHIP_TEMPERATURE parameter.
     */
    class Temperature : public BaseTemperature {
    public:
        Temperature(Tmc9660Handler& parent);
        ~Temperature() override = default;
        
        // Pure virtual methods from BaseTemperature
        bool Initialize() noexcept override;
        bool Deinitialize() noexcept override;
        hf_temp_err_t ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept override;
        hf_temp_err_t GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept override;
        hf_u32_t GetCapabilities() const noexcept override;

    private:
        Tmc9660Handler& parent_;
        mutable RtosMutex mutex_;
        mutable hf_temp_statistics_t statistics_;
        mutable hf_temp_diagnostics_t diagnostics_;
        std::atomic<hf_temp_err_t> last_error_;
        
        hf_temp_err_t UpdateStatistics(hf_temp_err_t result, uint64_t start_time_us) noexcept;
        uint64_t GetCurrentTimeUs() const noexcept;
        void UpdateDiagnostics(hf_temp_err_t error) noexcept;
    };

    /**
     * @brief Get a reference to the GPIO wrapper for a given internal GPIO channel.
     * @param gpioNumber The TMC9660 internal GPIO number (e.g., 17, 18)
     * @return Reference to the GPIO wrapper
     */
    Gpio& gpio(uint8_t gpioNumber);

    /**
     * @brief Get a reference to the ADC wrapper.
     */
    Adc& adc();

    /**
     * @brief Get a reference to the Temperature wrapper.
     */
    Temperature& temperature();

    /**
     * @brief Get the current communication mode being used.
     * @return CommMode::SPI if using SPI, CommMode::UART if using UART
     */
    CommMode GetCommMode() const noexcept;

    /**
     * @brief Check if SPI interface is available.
     * @return true if SPI interface was provided during construction
     */
    bool HasSpiInterface() const noexcept { return comm_interface_spi_ != nullptr; }

    /**
     * @brief Check if UART interface is available.
     * @return true if UART interface was provided during construction
     */
    bool HasUartInterface() const noexcept { return comm_interface_uart_ != nullptr; }

    /**
     * @brief Switch communication interface (if both SPI and UART are available).
     * @param mode Desired communication mode
     * @return true if switch successful, false if requested interface not available
     */
    bool SwitchCommInterface(CommMode mode);

    /**
     * @brief Get the current bootloader config in use.
     */
    const tmc9660::BootloaderConfig& bootConfig() const noexcept { return *bootCfg_; }
    
    /**
     * @brief Dump comprehensive diagnostics and statistics to log as INFO level.
     * Logs TMC9660 device status, ADC statistics, GPIO status, and communication health.
     */
    void DumpDiagnostics() const noexcept;

private:
    // Interface references (stored for lazy initialization)
    BaseSpi* spi_ref_;     ///< Reference to SPI interface (nullptr if not provided)
    BaseUart* uart_ref_;   ///< Reference to UART interface (nullptr if not provided)
    
    // Communication interfaces (created lazily in Initialize())
    std::unique_ptr<Tmc9660SpiCommInterface> comm_interface_spi_;
    std::unique_ptr<Tmc9660UartCommInterface> comm_interface_uart_;
    TMC9660CommInterface* active_comm_interface_;  ///< Pointer to currently active interface
    
    // TMC9660 driver instance (initialized with active interface)
    std::shared_ptr<TMC9660> tmc9660_;
    
    // GPIO and ADC wrappers (created lazily in Initialize())
    std::array<std::unique_ptr<Gpio>, 2> gpioWrappers_; // e.g., for GPIO17, GPIO18
    std::unique_ptr<Adc> adcWrapper_;
    std::unique_ptr<Temperature> temperatureWrapper_;
    
    // Configuration
    const tmc9660::BootloaderConfig* bootCfg_;
    uint8_t device_address_;

    /**
     * @brief Initialize the TMC9660 communication interface.
     * @return Pointer to initialized communication interface, or nullptr on failure
     */
    TMC9660CommInterface* InitializeCommInterface();
};

#endif // COMPONENT_HANDLER_TMC9660_HANDLER_H_ 