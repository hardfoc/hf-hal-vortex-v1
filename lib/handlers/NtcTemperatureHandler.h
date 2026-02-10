/**
 * @file NtcTemperatureHandler.h
 * @brief NTC temperature sensor handler for the HardFOC system.
 *
 * This handler provides a BaseTemperature interface for NTC thermistor temperature
 * sensors using a BaseAdc pointer for voltage measurements. It wraps the NtcThermistor
 * library to provide a unified temperature sensor interface.
 *
 * @author HardFOC Development Team
 * @date 2025
 * @copyright HardFOC
 */

#pragma once

#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseTemperature.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseAdc.h"
#include "utils-and-drivers/hf-core-drivers/external/hf-ntc-thermistor-driver/include/NtcThermistor.h"

#include <memory>
#include <mutex>

//--------------------------------------
//  NTC Temperature Handler Configuration
//--------------------------------------

/**
 * @brief NTC temperature handler configuration structure
 */
typedef struct {
    ntc_type_t ntc_type;                    ///< NTC thermistor type
    uint8_t adc_channel;                    ///< ADC channel for voltage measurement
    float series_resistance;                ///< Series resistance in voltage divider (ohms)
    float reference_voltage;                ///< Reference voltage (V)
    float calibration_offset;               ///< Calibration offset (°C)
    ntc_conversion_method_t conversion_method; ///< Temperature conversion method
    uint32_t sample_count;                  ///< Number of samples to average
    uint32_t sample_delay_ms;               ///< Delay between samples (ms)
    float min_temperature;                  ///< Minimum temperature (°C)
    float max_temperature;                  ///< Maximum temperature (°C)
    bool enable_filtering;                  ///< Enable temperature filtering
    float filter_alpha;                     ///< Filter alpha value (0.0-1.0)
    const char* sensor_name;                ///< Sensor name/identifier
    const char* sensor_description;         ///< Sensor description
} ntc_temp_handler_config_t;

/**
 * @brief Default NTC temperature handler configuration
 */
#define NTC_TEMP_HANDLER_CONFIG_DEFAULT() { \
    .ntc_type = NTC_TYPE_NTCG163JFT103FT1S, \
    .adc_channel = 0, \
    .series_resistance = 10000.0f, \
    .reference_voltage = 3.3f, \
    .calibration_offset = 0.0f, \
    .conversion_method = NTC_CONVERSION_AUTO, \
    .sample_count = 1, \
    .sample_delay_ms = 0, \
    .min_temperature = -40.0f, \
    .max_temperature = 125.0f, \
    .enable_filtering = false, \
    .filter_alpha = 0.1f, \
    .sensor_name = "NTC_Temperature_Sensor", \
    .sensor_description = "NTC Thermistor Temperature Sensor" \
}

//--------------------------------------
//  NtcTemperatureHandler Class
//--------------------------------------

/**
 * @class NtcTemperatureHandler
 * @brief NTC temperature sensor handler implementing BaseTemperature interface
 * 
 * This class provides a complete implementation of the BaseTemperature interface
 * for NTC thermistor temperature sensors. It uses a BaseAdc pointer for voltage
 * measurements and wraps the NtcThermistor library to provide comprehensive
 * temperature sensing capabilities.
 * 
 * Key features:
 * - BaseTemperature interface compliance
 * - Hardware-agnostic design using BaseAdc
 * - Support for multiple NTC types
 * - Dual conversion methods (lookup table and mathematical)
 * - Built-in calibration and filtering
 * - Comprehensive error handling
 * - Thread-safe operations
 * - Statistics and diagnostics
 * - Threshold monitoring support
 * - Continuous monitoring support
 * 
 * @note This class is thread-safe and can be used in multi-threaded applications
 * @note The handler requires a BaseAdc interface for voltage measurements
 * @note Supports all BaseTemperature features including threshold and continuous monitoring
 */
class NtcTemperatureHandler : public BaseTemperature {
public:
    //==============================================================//
    // CONSTRUCTORS AND DESTRUCTOR
    //==============================================================//
    
    /**
     * @brief Constructor with NTC type and ADC interface
     * @param ntc_type NTC thermistor type
     * @param adc_interface Pointer to BaseAdc interface
     * @param sensor_name Optional sensor name
     */
    NtcTemperatureHandler(ntc_type_t ntc_type, BaseAdc* adc_interface, const char* sensor_name = nullptr) noexcept;
    
    /**
     * @brief Constructor with configuration and ADC interface
     * @param config NTC temperature handler configuration
     * @param adc_interface Pointer to BaseAdc interface
     */
    NtcTemperatureHandler(const ntc_temp_handler_config_t& config, BaseAdc* adc_interface) noexcept;
    
    /**
     * @brief Copy constructor is deleted
     */
    NtcTemperatureHandler(const NtcTemperatureHandler&) = delete;
    
    /**
     * @brief Assignment operator is deleted
     */
    NtcTemperatureHandler& operator=(const NtcTemperatureHandler&) = delete;
    
    /**
     * @brief Move constructor
     */
    NtcTemperatureHandler(NtcTemperatureHandler&&) noexcept = default;
    
    /**
     * @brief Move assignment operator
     */
    NtcTemperatureHandler& operator=(NtcTemperatureHandler&&) noexcept = default;
    
    /**
     * @brief Virtual destructor
     */
    virtual ~NtcTemperatureHandler() noexcept = default;
    
    //==============================================================//
    // BASE TEMPERATURE INTERFACE IMPLEMENTATION
    //==============================================================//
    
    // Pure virtual implementations
    bool Initialize() noexcept override;
    bool Deinitialize() noexcept override;
    hf_temp_err_t ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept override;
    
    // Information interface
    hf_temp_err_t GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept override;
    hf_u32_t GetCapabilities() const noexcept override;
    
    // Advanced features (supported)
    hf_temp_err_t SetRange(float min_celsius, float max_celsius) noexcept override;
    hf_temp_err_t GetRange(float* min_celsius, float* max_celsius) const noexcept override;
    hf_temp_err_t GetResolution(float* resolution_celsius) const noexcept override;
    hf_temp_err_t SetThresholds(float low_threshold_celsius, float high_threshold_celsius) noexcept override;
    hf_temp_err_t GetThresholds(float* low_threshold_celsius, float* high_threshold_celsius) const noexcept override;
    hf_temp_err_t EnableThresholdMonitoring(hf_temp_threshold_callback_t callback, void* user_data) noexcept override;
    hf_temp_err_t DisableThresholdMonitoring() noexcept override;
    hf_temp_err_t StartContinuousMonitoring(hf_u32_t sample_rate_hz, hf_temp_reading_callback_t callback, void* user_data) noexcept override;
    hf_temp_err_t StopContinuousMonitoring() noexcept override;
    bool IsMonitoringActive() const noexcept override;
    hf_temp_err_t SetCalibrationOffset(float offset_celsius) noexcept override;
    hf_temp_err_t GetCalibrationOffset(float* offset_celsius) const noexcept override;
    hf_temp_err_t ResetCalibration() noexcept override;
    hf_temp_err_t EnterSleepMode() noexcept override;
    hf_temp_err_t ExitSleepMode() noexcept override;
    bool IsSleeping() const noexcept override;
    hf_temp_err_t SelfTest() noexcept override;
    hf_temp_err_t CheckHealth() noexcept override;
    hf_temp_err_t GetStatistics(hf_temp_statistics_t& statistics) noexcept override;
    hf_temp_err_t GetDiagnostics(hf_temp_diagnostics_t& diagnostics) noexcept override;
    hf_temp_err_t ResetStatistics() noexcept override;
    hf_temp_err_t ResetDiagnostics() noexcept override;
    
    //==============================================================//
    // NTC-SPECIFIC METHODS
    //==============================================================//
    
    /**
     * @brief Get NTC thermistor instance
     * @return Pointer to NtcThermistor instance
     */
    NtcThermistor* GetNtcThermistor() noexcept;
    
    /**
     * @brief Get NTC thermistor instance (const)
     * @return Pointer to const NtcThermistor instance
     */
    const NtcThermistor* GetNtcThermistor() const noexcept;
    
    /**
     * @brief Get NTC configuration
     * @param config Pointer to store configuration
     * @return Error code
     */
    ntc_err_t GetNtcConfiguration(ntc_config_t* config) const noexcept;
    
    /**
     * @brief Set NTC configuration
     * @param config New configuration
     * @return Error code
     */
    ntc_err_t SetNtcConfiguration(const ntc_config_t& config) noexcept;
    
    /**
     * @brief Get thermistor resistance
     * @param resistance_ohms Pointer to store resistance
     * @return Error code
     */
    ntc_err_t GetResistance(float* resistance_ohms) noexcept;
    
    /**
     * @brief Get voltage across thermistor
     * @param voltage_volts Pointer to store voltage
     * @return Error code
     */
    ntc_err_t GetVoltage(float* voltage_volts) noexcept;
    
    /**
     * @brief Get raw ADC value
     * @param adc_value Pointer to store ADC value
     * @return Error code
     */
    ntc_err_t GetRawAdcValue(uint32_t* adc_value) noexcept;
    
    /**
     * @brief Calibrate the thermistor
     * @param reference_temperature_celsius Known reference temperature
     * @return Error code
     */
    ntc_err_t Calibrate(float reference_temperature_celsius) noexcept;
    
    /**
     * @brief Set conversion method
     * @param method Conversion method
     * @return Error code
     */
    ntc_err_t SetConversionMethod(ntc_conversion_method_t method) noexcept;
    
    /**
     * @brief Set voltage divider parameters
     * @param series_resistance Series resistance (ohms)
     * @return Error code
     */
    ntc_err_t SetVoltageDivider(float series_resistance) noexcept;
    
    /**
     * @brief Set reference voltage
     * @param reference_voltage Reference voltage (V)
     * @return Error code
     */
    ntc_err_t SetReferenceVoltage(float reference_voltage) noexcept;
    
    /**
     * @brief Set beta value
     * @param beta_value Beta value (K)
     * @return Error code
     */
    ntc_err_t SetBetaValue(float beta_value) noexcept;
    
    /**
     * @brief Set ADC channel
     * @param adc_channel ADC channel number
     * @return Error code
     */
    ntc_err_t SetAdcChannel(uint8_t adc_channel) noexcept;
    
    /**
     * @brief Set sampling parameters
     * @param sample_count Number of samples
     * @param sample_delay_ms Delay between samples (ms)
     * @return Error code
     */
    ntc_err_t SetSamplingParameters(uint32_t sample_count, uint32_t sample_delay_ms) noexcept;
    
    /**
     * @brief Enable/disable filtering
     * @param enable Enable filtering
     * @param alpha Filter alpha value (0.0-1.0)
     * @return Error code
     */
    ntc_err_t SetFiltering(bool enable, float alpha = 0.1f) noexcept;
    
    /**
     * @brief Get NTC statistics
     * @param statistics Pointer to store statistics
     * @return Error code
     */
    ntc_err_t GetNtcStatistics(ntc_statistics_t* statistics) noexcept;
    
    /**
     * @brief Get NTC diagnostics
     * @param diagnostics Pointer to store diagnostics
     * @return Error code
     */
    ntc_err_t GetNtcDiagnostics(ntc_diagnostics_t* diagnostics) noexcept;
    
    /**
     * @brief Get sensor name
     * @return Sensor name string
     */
    const char* GetSensorName() const noexcept;
    
    /**
     * @brief Get sensor description
     * @return Sensor description string
     */
    const char* GetSensorDescription() const noexcept;

private:
    //==============================================================//
    // PRIVATE MEMBER VARIABLES
    //==============================================================//
    
    mutable std::mutex mutex_;              ///< Thread safety mutex
    std::unique_ptr<NtcThermistor> ntc_thermistor_; ///< NTC thermistor instance
    BaseAdc* adc_interface_;                ///< ADC interface pointer
    ntc_temp_handler_config_t config_;      ///< Handler configuration
    
    // BaseTemperature state
    bool initialized_;                      ///< Initialization status
    hf_temp_state_t current_state_;         ///< Current state
    hf_temp_config_t base_config_;          ///< Base configuration
    
    // Threshold monitoring
    float low_threshold_celsius_;           ///< Low temperature threshold
    float high_threshold_celsius_;          ///< High temperature threshold
    bool threshold_monitoring_enabled_;     ///< Threshold monitoring status
    hf_temp_threshold_callback_t threshold_callback_; ///< Threshold callback
    void* threshold_user_data_;             ///< Threshold callback user data
    
    // Continuous monitoring
    bool continuous_monitoring_active_;     ///< Continuous monitoring status
    hf_u32_t sample_rate_hz_;               ///< Sample rate for continuous monitoring
    hf_temp_reading_callback_t monitoring_callback_; ///< Monitoring callback
    void* monitoring_user_data_;            ///< Monitoring callback user data
    
    // Statistics and diagnostics
    hf_temp_statistics_t statistics_;       ///< BaseTemperature statistics
    hf_temp_diagnostics_t diagnostics_;     ///< BaseTemperature diagnostics
    
    //==============================================================//
    // PRIVATE HELPER METHODS
    //==============================================================//
    
    /**
     * @brief Initialize NTC thermistor with current configuration
     * @return true if successful, false otherwise
     */
    bool InitializeNtcThermistor() noexcept;
    
    /**
     * @brief Convert NTC error to BaseTemperature error
     * @param ntc_error NTC error code
     * @return BaseTemperature error code
     */
    hf_temp_err_t ConvertNtcError(ntc_err_t ntc_error) const noexcept;
    
    /**
     * @brief Convert NTC reading to BaseTemperature reading
     * @param ntc_reading NTC reading structure
     * @param base_reading BaseTemperature reading structure
     */
    void ConvertNtcReadingToBaseReading(const ntc_reading_t& ntc_reading, 
                                       hf_temp_reading_t& base_reading) noexcept;
    
    /**
     * @brief Convert NTC statistics to BaseTemperature statistics
     * @param ntc_stats NTC statistics structure
     * @param base_stats BaseTemperature statistics structure
     */
    void ConvertNtcStatisticsToBaseStatistics(const ntc_statistics_t& ntc_stats, 
                                             hf_temp_statistics_t& base_stats) noexcept;
    
    /**
     * @brief Convert NTC diagnostics to BaseTemperature diagnostics
     * @param ntc_diag NTC diagnostics structure
     * @param base_diag BaseTemperature diagnostics structure
     */
    void ConvertNtcDiagnosticsToBaseDiagnostics(const ntc_diagnostics_t& ntc_diag, 
                                               hf_temp_diagnostics_t& base_diag) noexcept;
    
    /**
     * @brief Check thresholds and trigger callback if needed
     * @param temperature_celsius Current temperature reading
     */
    void CheckThresholds(float temperature_celsius) noexcept;
    
    /**
     * @brief Update BaseTemperature statistics
     * @param operation_successful Whether operation was successful
     * @param operation_time_us Operation time in microseconds
     * @param temperature_celsius Temperature reading
     */
    void UpdateBaseStatistics(bool operation_successful, hf_u32_t operation_time_us, float temperature_celsius) noexcept;
    
    /**
     * @brief Update BaseTemperature diagnostics
     * @param error Error code
     */
    void UpdateBaseDiagnostics(hf_temp_err_t error) noexcept;
    
    /**
     * @brief Validate configuration
     * @param config Configuration to validate
     * @return Error code
     */
    hf_temp_err_t ValidateConfiguration(const ntc_temp_handler_config_t& config) const noexcept;
    
    /**
     * @brief Get current timestamp in microseconds
     * @return Current timestamp
     */
    static hf_u64_t GetCurrentTimeUs() noexcept;
}; 