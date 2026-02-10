/**
 * @file NtcTemperatureHandler.cpp
 * @brief Implementation of NTC temperature sensor handler for HardFOC system.
 *
 * This file implements the NtcTemperatureHandler class, which provides a BaseTemperature
 * interface for NTC thermistors. It wraps the NtcThermistor library and uses a BaseAdc
 * pointer for underlying ADC operations.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#include "NtcTemperatureHandler.h"
#include "utils-and-drivers/hf-core-drivers/external/hf-ntc-thermistor-driver/include/NtcThermistor.h"
#include "utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseAdc.h"
#include "utils-and-drivers/driver-handlers/Logger.h"
#include "utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/include/OsAbstraction.h"

static const char* TAG = "NtcTempHandler";

//--------------------------------------
//  NtcTemperatureHandler Implementation
//--------------------------------------

NtcTemperatureHandler::NtcTemperatureHandler(BaseAdc* adc_interface, 
                                             const ntc_temp_handler_config_t& config) noexcept
    : BaseTemperature()
    , adc_interface_(adc_interface)
    , config_(config)
    , ntc_thermistor_(nullptr)
    , threshold_callback_(nullptr)
    , threshold_user_data_(nullptr)
    , continuous_callback_(nullptr)
    , continuous_user_data_(nullptr)
    , monitoring_timer_(nullptr)
    , monitoring_active_(false)
    , calibration_offset_(0.0f)
    , statistics_({})
    , diagnostics_({}) {
    
    // Initialize statistics
    statistics_.total_operations = 0;
    statistics_.successful_operations = 0;
    statistics_.failed_operations = 0;
    statistics_.temperature_readings = 0;
    statistics_.calibration_count = 0;
    statistics_.threshold_violations = 0;
    statistics_.average_operation_time_us = 0;
    statistics_.max_operation_time_us = 0;
    statistics_.min_operation_time_us = UINT32_MAX;
    statistics_.min_temperature_celsius = FLT_MAX;
    statistics_.max_temperature_celsius = -FLT_MAX;
    statistics_.avg_temperature_celsius = 0.0f;
    
    // Initialize diagnostics
    diagnostics_.sensor_healthy = true;
    diagnostics_.last_error_code = TEMP_SUCCESS;
    diagnostics_.last_error_timestamp = 0;
    diagnostics_.consecutive_errors = 0;
    diagnostics_.sensor_available = (adc_interface_ != nullptr);
    diagnostics_.threshold_monitoring_supported = true;
    diagnostics_.threshold_monitoring_enabled = false;
    diagnostics_.continuous_monitoring_active = false;
    diagnostics_.current_temperature_raw = 0;
    diagnostics_.calibration_valid = false;
}

NtcTemperatureHandler::~NtcTemperatureHandler() noexcept {
    // Stop monitoring if active
    if (monitoring_active_) {
        StopContinuousMonitoring();
    }
    
    // Clean up timer
    if (monitoring_timer_) {
        esp_timer_delete(monitoring_timer_);
        monitoring_timer_ = nullptr;
    }
    
    // Clean up thermistor
    if (ntc_thermistor_) {
        ntc_thermistor_->Deinitialize();
        ntc_thermistor_.reset();
    }
}

bool NtcTemperatureHandler::Initialize() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_) {
        Logger::GetInstance().Warn(TAG, "Already initialized");
        return true;
    }
    
    if (adc_interface_ == nullptr) {
        Logger::GetInstance().Error(TAG, "ADC interface is null");
        SetLastError(TEMP_ERR_NULL_POINTER);
        return false;
    }
    
    // Ensure ADC interface is initialized
    if (!adc_interface_->EnsureInitialized()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize ADC interface");
        SetLastError(TEMP_ERR_RESOURCE_UNAVAILABLE);
        return false;
    }
    
    // Create NTC thermistor instance
    try {
        ntc_thermistor_ = std::make_unique<NtcThermistor>(config_.ntc_type, adc_interface_);
    } catch (const std::exception& e) {
        Logger::GetInstance().Error(TAG, "Failed to create NTC thermistor: %s", e.what());
        SetLastError(TEMP_ERR_OUT_OF_MEMORY);
        return false;
    }
    
    // Initialize NTC thermistor
    if (!ntc_thermistor_->Initialize()) {
        Logger::GetInstance().Error(TAG, "Failed to initialize NTC thermistor");
        SetLastError(TEMP_ERR_FAILURE);
        return false;
    }
    
    // Apply configuration
    if (config_.conversion_method != NTC_CONVERSION_METHOD_DEFAULT) {
        ntc_thermistor_->SetConversionMethod(config_.conversion_method);
    }
    
    if (config_.voltage_divider_series_resistance > 0) {
        ntc_thermistor_->SetVoltageDivider(config_.voltage_divider_series_resistance, 
                                          config_.voltage_divider_parallel_resistance);
    }
    
    if (config_.reference_voltage > 0) {
        ntc_thermistor_->SetReferenceVoltage(config_.reference_voltage);
    }
    
    if (config_.beta_value > 0) {
        ntc_thermistor_->SetBetaValue(config_.beta_value);
    }
    
    // Apply calibration offset if set
    if (calibration_offset_ != 0.0f) {
        ntc_thermistor_->SetCalibrationOffset(calibration_offset_);
        diagnostics_.calibration_valid = true;
    }
    
    // Set thresholds if enabled
    if (config_.enable_threshold_monitoring) {
        SetThresholds(config_.low_threshold_celsius, config_.high_threshold_celsius);
        EnableThresholdMonitoring(config_.threshold_callback, config_.threshold_user_data);
    }
    
    initialized_ = true;
    current_state_ = HF_TEMP_STATE_INITIALIZED;
    
    Logger::GetInstance().Info(TAG, "NTC temperature handler initialized successfully");
    return true;
}

bool NtcTemperatureHandler::Deinitialize() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        return true;
    }
    
    // Stop continuous monitoring
    if (monitoring_active_) {
        StopContinuousMonitoring();
    }
    
    // Clean up timer
    if (monitoring_timer_) {
        esp_timer_delete(monitoring_timer_);
        monitoring_timer_ = nullptr;
    }
    
    // Deinitialize thermistor
    if (ntc_thermistor_) {
        ntc_thermistor_->Deinitialize();
        ntc_thermistor_.reset();
    }
    
    // Reset callbacks
    threshold_callback_ = nullptr;
    threshold_user_data_ = nullptr;
    continuous_callback_ = nullptr;
    continuous_user_data_ = nullptr;
    
    initialized_ = false;
    current_state_ = HF_TEMP_STATE_UNINITIALIZED;
    
    Logger::GetInstance().Info(TAG, "NTC temperature handler deinitialized");
    return true;
}

hf_temp_err_t NtcTemperatureHandler::ReadTemperatureCelsiusImpl(float* temperature_celsius) noexcept {
    if (ntc_thermistor_ == nullptr) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    const auto start_time = os_time_get();
    
    ntc_err_t result = ntc_thermistor_->ReadTemperatureCelsius(temperature_celsius);
    
    const auto end_time = os_time_get();
    const auto operation_time = static_cast<hf_u32_t>(end_time - start_time);
    
    if (result == NTC_SUCCESS) {
        UpdateStatistics(true, operation_time);
        UpdateDiagnostics(TEMP_SUCCESS);
        
        // Update diagnostics with raw reading
        float raw_value;
        if (ntc_thermistor_->GetRawAdcValue(&raw_value) == NTC_SUCCESS) {
            diagnostics_.current_temperature_raw = static_cast<hf_u32_t>(raw_value);
        }
        
        // Check thresholds
        CheckThresholds(*temperature_celsius);
        
    } else {
        UpdateStatistics(false, operation_time);
        hf_temp_err_t temp_error = ConvertNtcError(result);
        UpdateDiagnostics(temp_error);
        return temp_error;
    }
    
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::GetSensorInfo(hf_temp_sensor_info_t* info) const noexcept {
    if (info == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    if (!initialized_) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    // Fill in NTC-specific information
    info->sensor_type = HF_TEMP_SENSOR_TYPE_THERMISTOR;
    info->min_temp_celsius = -40.0f;  // Typical NTC range
    info->max_temp_celsius = 125.0f;  // Typical NTC range
    info->resolution_celsius = 0.1f;  // Typical resolution
    info->accuracy_celsius = 1.0f;    // Typical accuracy
    info->response_time_ms = 100;     // Typical response time
    info->capabilities = HF_TEMP_CAP_THRESHOLD_MONITORING | 
                        HF_TEMP_CAP_CONTINUOUS_READING | 
                        HF_TEMP_CAP_CALIBRATION |
                        HF_TEMP_CAP_HIGH_PRECISION;
    info->manufacturer = "Generic";
    info->model = "NTC Thermistor";
    info->version = "1.0";
    
    return TEMP_SUCCESS;
}

hf_u32_t NtcTemperatureHandler::GetCapabilities() const noexcept {
    return HF_TEMP_CAP_THRESHOLD_MONITORING | 
           HF_TEMP_CAP_CONTINUOUS_READING | 
           HF_TEMP_CAP_CALIBRATION |
           HF_TEMP_CAP_HIGH_PRECISION;
}

hf_temp_err_t NtcTemperatureHandler::SetThresholds(float low_threshold_celsius, float high_threshold_celsius) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    if (low_threshold_celsius >= high_threshold_celsius) {
        return TEMP_ERR_INVALID_THRESHOLD;
    }
    
    config_.low_threshold_celsius = low_threshold_celsius;
    config_.high_threshold_celsius = high_threshold_celsius;
    
    Logger::GetInstance().Info(TAG, "Thresholds set: low=%.2f°C, high=%.2f°C", low_threshold_celsius, high_threshold_celsius);
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::GetThresholds(float* low_threshold_celsius, float* high_threshold_celsius) const noexcept {
    if (low_threshold_celsius == nullptr || high_threshold_celsius == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    if (!initialized_) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    *low_threshold_celsius = config_.low_threshold_celsius;
    *high_threshold_celsius = config_.high_threshold_celsius;
    
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::EnableThresholdMonitoring(hf_temp_threshold_callback_t callback, void* user_data) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    threshold_callback_ = callback;
    threshold_user_data_ = user_data;
    diagnostics_.threshold_monitoring_enabled = (callback != nullptr);
    
    Logger::GetInstance().Info(TAG, "Threshold monitoring %s", callback ? "enabled" : "disabled");
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::DisableThresholdMonitoring() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    threshold_callback_ = nullptr;
    threshold_user_data_ = nullptr;
    diagnostics_.threshold_monitoring_enabled = false;
    
    Logger::GetInstance().Info(TAG, "Threshold monitoring disabled");
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::StartContinuousMonitoring(hf_u32_t sample_rate_hz, 
                                                              hf_temp_reading_callback_t callback, 
                                                              void* user_data) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    if (callback == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    if (sample_rate_hz == 0) {
        return TEMP_ERR_INVALID_PARAMETER;
    }
    
    // Stop existing monitoring if active
    if (monitoring_active_) {
        StopContinuousMonitoring();
    }
    
    // Create timer for continuous monitoring
    esp_timer_create_args_t timer_args = {
        .callback = ContinuousMonitoringCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ntc_monitor",
        .skip_unhandled_events = false
    };
    
    esp_err_t esp_result = esp_timer_create(&timer_args, &monitoring_timer_);
    if (esp_result != ESP_OK) {
        Logger::GetInstance().Error(TAG, "Failed to create monitoring timer: %s", esp_err_to_name(esp_result));
        return TEMP_ERR_RESOURCE_UNAVAILABLE;
    }
    
    // Calculate interval in microseconds
    const uint64_t interval_us = 1000000ULL / sample_rate_hz;
    
    // Start timer
    esp_result = esp_timer_start_periodic(monitoring_timer_, interval_us);
    if (esp_result != ESP_OK) {
        Logger::GetInstance().Error(TAG, "Failed to start monitoring timer: %s", esp_err_to_name(esp_result));
        esp_timer_delete(monitoring_timer_);
        monitoring_timer_ = nullptr;
        return TEMP_ERR_FAILURE;
    }
    
    continuous_callback_ = callback;
    continuous_user_data_ = user_data;
    monitoring_active_ = true;
    diagnostics_.continuous_monitoring_active = true;
    
    Logger::GetInstance().Info(TAG, "Continuous monitoring started at %u Hz", sample_rate_hz);
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::StopContinuousMonitoring() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!monitoring_active_) {
        return TEMP_SUCCESS;
    }
    
    // Stop timer
    if (monitoring_timer_) {
        esp_timer_stop(monitoring_timer_);
        esp_timer_delete(monitoring_timer_);
        monitoring_timer_ = nullptr;
    }
    
    continuous_callback_ = nullptr;
    continuous_user_data_ = nullptr;
    monitoring_active_ = false;
    diagnostics_.continuous_monitoring_active = false;
    
    Logger::GetInstance().Info(TAG, "Continuous monitoring stopped");
    return TEMP_SUCCESS;
}

bool NtcTemperatureHandler::IsMonitoringActive() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return monitoring_active_;
}

hf_temp_err_t NtcTemperatureHandler::Calibrate(float reference_temperature_celsius) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_ || ntc_thermistor_ == nullptr) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    // Read current temperature
    float current_temperature;
    ntc_err_t result = ntc_thermistor_->ReadTemperatureCelsius(&current_temperature);
    if (result != NTC_SUCCESS) {
        return ConvertNtcError(result);
    }
    
    // Calculate offset
    calibration_offset_ = reference_temperature_celsius - current_temperature;
    
    // Apply offset to thermistor
    ntc_thermistor_->SetCalibrationOffset(calibration_offset_);
    
    diagnostics_.calibration_valid = true;
    statistics_.calibration_count++;
    
    Logger::GetInstance().Info(TAG, "Calibration completed: offset=%.2f°C", calibration_offset_);
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::SetCalibrationOffset(float offset_celsius) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_ || ntc_thermistor_ == nullptr) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    calibration_offset_ = offset_celsius;
    ntc_thermistor_->SetCalibrationOffset(offset_celsius);
    diagnostics_.calibration_valid = true;
    
    Logger::GetInstance().Info(TAG, "Calibration offset set: %.2f°C", offset_celsius);
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::GetCalibrationOffset(float* offset_celsius) const noexcept {
    if (offset_celsius == nullptr) {
        return TEMP_ERR_NULL_POINTER;
    }
    
    if (!initialized_) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    *offset_celsius = calibration_offset_;
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::ResetCalibration() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_ || ntc_thermistor_ == nullptr) {
        return TEMP_ERR_NOT_INITIALIZED;
    }
    
    calibration_offset_ = 0.0f;
    ntc_thermistor_->SetCalibrationOffset(0.0f);
    diagnostics_.calibration_valid = false;
    
    Logger::GetInstance().Info(TAG, "Calibration reset");
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::GetStatistics(hf_temp_statistics_t& statistics) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    statistics = statistics_;
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::GetDiagnostics(hf_temp_diagnostics_t& diagnostics) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    diagnostics = diagnostics_;
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::ResetStatistics() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    statistics_ = {};
    statistics_.min_temperature_celsius = FLT_MAX;
    statistics_.max_temperature_celsius = -FLT_MAX;
    statistics_.min_operation_time_us = UINT32_MAX;
    
    Logger::GetInstance().Info(TAG, "Statistics reset");
    return TEMP_SUCCESS;
}

hf_temp_err_t NtcTemperatureHandler::ResetDiagnostics() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    
    diagnostics_.last_error_code = TEMP_SUCCESS;
    diagnostics_.last_error_timestamp = 0;
    diagnostics_.consecutive_errors = 0;
    diagnostics_.sensor_healthy = true;
    
    Logger::GetInstance().Info(TAG, "Diagnostics reset");
    return TEMP_SUCCESS;
}

//--------------------------------------
//  NTC-Specific Methods
//--------------------------------------

NtcThermistor* NtcTemperatureHandler::GetNtcThermistor() const noexcept {
    return ntc_thermistor_.get();
}

ntc_err_t NtcTemperatureHandler::GetNtcConfiguration(ntc_config_t* config) const noexcept {
    if (config == nullptr || ntc_thermistor_ == nullptr) {
        return NTC_ERR_NULL_POINTER;
    }
    
    return ntc_thermistor_->GetConfiguration(config);
}

ntc_err_t NtcTemperatureHandler::GetNtcReading(ntc_reading_t* reading) const noexcept {
    if (reading == nullptr || ntc_thermistor_ == nullptr) {
        return NTC_ERR_NULL_POINTER;
    }
    
    return ntc_thermistor_->GetReading(reading);
}

ntc_err_t NtcTemperatureHandler::GetNtcStatistics(ntc_statistics_t* statistics) const noexcept {
    if (statistics == nullptr || ntc_thermistor_ == nullptr) {
        return NTC_ERR_NULL_POINTER;
    }
    
    return ntc_thermistor_->GetStatistics(statistics);
}

ntc_err_t NtcTemperatureHandler::GetNtcDiagnostics(ntc_diagnostics_t* diagnostics) const noexcept {
    if (diagnostics == nullptr || ntc_thermistor_ == nullptr) {
        return NTC_ERR_NULL_POINTER;
    }
    
    return ntc_thermistor_->GetDiagnostics(diagnostics);
}

//--------------------------------------
//  Private Helper Methods
//--------------------------------------

void NtcTemperatureHandler::SetLastError(hf_temp_err_t error) noexcept {
    diagnostics_.last_error_code = error;
            diagnostics_.last_error_timestamp = static_cast<hf_u32_t>(os_time_get() / 1000); // Convert to ms
    diagnostics_.consecutive_errors++;
    
    if (diagnostics_.consecutive_errors > 5) {
        diagnostics_.sensor_healthy = false;
    }
}

void NtcTemperatureHandler::UpdateStatistics(bool operation_successful, hf_u32_t operation_time_us) noexcept {
    statistics_.total_operations++;
    
    if (operation_successful) {
        statistics_.successful_operations++;
        diagnostics_.consecutive_errors = 0;
        diagnostics_.sensor_healthy = true;
    } else {
        statistics_.failed_operations++;
    }
    
    // Update operation time statistics
    if (operation_time_us > 0) {
        if (operation_time_us > statistics_.max_operation_time_us) {
            statistics_.max_operation_time_us = operation_time_us;
        }
        if (operation_time_us < statistics_.min_operation_time_us) {
            statistics_.min_operation_time_us = operation_time_us;
        }
        
        // Update average (simple moving average)
        const uint32_t total_ops = statistics_.successful_operations + statistics_.failed_operations;
        if (total_ops > 0) {
            statistics_.average_operation_time_us = 
                (statistics_.average_operation_time_us * (total_ops - 1) + operation_time_us) / total_ops;
        }
    }
}

void NtcTemperatureHandler::UpdateDiagnostics(hf_temp_err_t error) noexcept {
    if (error != TEMP_SUCCESS) {
        SetLastError(error);
    } else {
        diagnostics_.consecutive_errors = 0;
        diagnostics_.sensor_healthy = true;
    }
}

void NtcTemperatureHandler::CheckThresholds(float temperature_celsius) noexcept {
    if (!diagnostics_.threshold_monitoring_enabled || threshold_callback_ == nullptr) {
        return;
    }
    
    if (temperature_celsius < config_.low_threshold_celsius) {
        statistics_.threshold_violations++;
        threshold_callback_(this, temperature_celsius, 0, threshold_user_data_); // 0 = low threshold
    } else if (temperature_celsius > config_.high_threshold_celsius) {
        statistics_.threshold_violations++;
        threshold_callback_(this, temperature_celsius, 1, threshold_user_data_); // 1 = high threshold
    }
}

hf_temp_err_t NtcTemperatureHandler::ConvertNtcError(ntc_err_t ntc_error) const noexcept {
    switch (ntc_error) {
        case NTC_SUCCESS:
            return TEMP_SUCCESS;
        case NTC_ERR_NULL_POINTER:
            return TEMP_ERR_NULL_POINTER;
        case NTC_ERR_NOT_INITIALIZED:
            return TEMP_ERR_NOT_INITIALIZED;
        case NTC_ERR_ALREADY_INITIALIZED:
            return TEMP_ERR_ALREADY_INITIALIZED;
        case NTC_ERR_INVALID_PARAMETER:
            return TEMP_ERR_INVALID_PARAMETER;
        case NTC_ERR_OUT_OF_MEMORY:
            return TEMP_ERR_OUT_OF_MEMORY;
        case NTC_ERR_READ_FAILED:
            return TEMP_ERR_READ_FAILED;
        case NTC_ERR_INVALID_READING:
            return TEMP_ERR_INVALID_READING;
        case NTC_ERR_OUT_OF_RANGE:
            return TEMP_ERR_OUT_OF_RANGE;
        case NTC_ERR_TIMEOUT:
            return TEMP_ERR_TIMEOUT;
        case NTC_ERR_CALIBRATION_FAILED:
            return TEMP_ERR_CALIBRATION_FAILED;
        case NTC_ERR_COMMUNICATION_FAILED:
            return TEMP_ERR_COMMUNICATION_FAILED;
        case NTC_ERR_HARDWARE_FAULT:
            return TEMP_ERR_HARDWARE_FAULT;
        case NTC_ERR_RESOURCE_BUSY:
            return TEMP_ERR_RESOURCE_BUSY;
        case NTC_ERR_RESOURCE_UNAVAILABLE:
            return TEMP_ERR_RESOURCE_UNAVAILABLE;
        case NTC_ERR_OPERATION_ABORTED:
            return TEMP_ERR_OPERATION_ABORTED;
        case NTC_ERR_INVALID_STATE:
            return TEMP_ERR_INVALID_STATE;
        case NTC_ERR_CONVERSION_FAILED:
            return TEMP_ERR_CONVERSION_FAILED;
        case NTC_ERR_DRIVER_ERROR:
            return TEMP_ERR_DRIVER_ERROR;
        default:
            return TEMP_ERR_FAILURE;
    }
}

//--------------------------------------
//  Static Callback Functions
//--------------------------------------

void NtcTemperatureHandler::ContinuousMonitoringCallback(void* arg) {
    auto* handler = static_cast<NtcTemperatureHandler*>(arg);
    if (handler == nullptr) {
        return;
    }
    
    // Read temperature
    hf_temp_reading_t reading = {};
    hf_temp_err_t error = handler->ReadTemperature(&reading);
    
    // Call user callback if provided
    if (handler->continuous_callback_ != nullptr) {
        handler->continuous_callback_(handler, &reading, handler->continuous_user_data_);
    }
} 