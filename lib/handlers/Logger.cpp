/**
 * @file Logger.cpp
 * @brief Implementation of the advanced logging system.
 *
 * This file provides the implementation for the Logger class,
 * which supports various formatting options including colors,
 * styles, and ASCII art display.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#include "Logger.h"

// Include the base logger from the internal interface wrapper
#include "../hf-core-drivers/internal/hf-internal-interface-wrap/inc/base/BaseLogger.h"
#ifdef HF_MCU_FAMILY_ESP32
#include "../hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspLogger.h"
#endif

#include <cstdarg>
#include <cstring>
#include <algorithm>
#include <sstream>
#include <iomanip>

//==============================================================================
// CONSTRUCTOR/DESTRUCTOR
//==============================================================================

Logger::Logger() noexcept 
    : initialized_(false)
    , config_()
    , tag_levels_()
    , base_logger_(nullptr) {
    
    // Set default configuration
    config_.level = LogLevel::INFO;
    config_.color = LogColor::DEFAULT;
    config_.background = LogBackground::DEFAULT;
    config_.style = LogStyle::NORMAL;
    config_.enable_colors = true;
    config_.enable_effects = true;
    config_.max_width = 80;
    config_.center_text = false;
    config_.add_border = false;
    config_.border_char = '*';
    config_.border_padding = 2;
    config_.enable_ascii_art = true;
    config_.format_ascii_art = true;
}

Logger::~Logger() noexcept {
    Deinitialize();
}

//==============================================================================
// SINGLETON ACCESS
//==============================================================================

Logger& Logger::GetInstance() noexcept {
    static Logger instance;
    return instance;
}

//==============================================================================
// INITIALIZATION
//==============================================================================

bool Logger::Initialize(const LogConfig& config) noexcept {
    if (initialized_.load()) {
        return true; // Already initialized
    }

    config_ = config;
    
    // Create base logger
    base_logger_ = CreateBaseLogger();
    if (!base_logger_) {
        return false;
    }

    // Initialize base logger
    hf_logger_config_t base_config;
    base_config.enable_colors = config.enable_colors;
    base_config.enable_effects = config.enable_effects;
    base_config.max_width = config.max_width;
    base_config.default_level = static_cast<hf_log_level_t>(config.level);
    
    if (base_logger_->Initialize(base_config) != hf_logger_err_t::LOGGER_SUCCESS) {
        return false;
    }

    initialized_.store(true);
    return true;
}

void Logger::Deinitialize() noexcept {
    if (!initialized_.load()) {
        return;
    }

    if (base_logger_) {
        base_logger_->Deinitialize();
        base_logger_.reset();
    }

    tag_levels_.clear();
    initialized_.store(false);
}

bool Logger::IsInitialized() const noexcept {
    return initialized_.load();
}

//==============================================================================
// LOG LEVEL MANAGEMENT
//==============================================================================

void Logger::SetLogLevel(const std::string& tag, LogLevel level) noexcept {
    if (!initialized_.load()) {
        return;
    }

    tag_levels_[tag] = level;
    
    if (base_logger_) {
        base_logger_->SetLogLevel(tag.c_str(), static_cast<hf_log_level_t>(level));
    }
}

LogLevel Logger::GetLogLevel(const std::string& tag) const noexcept {
    if (!initialized_.load()) {
        return config_.level;
    }

    auto it = tag_levels_.find(tag);
    if (it != tag_levels_.end()) {
        return it->second;
    }

    return config_.level;
}

//==============================================================================
// BASIC LOGGING METHODS
//==============================================================================

void Logger::Error(const std::string& tag, const char* format, ...) noexcept {
    if (!IsLevelEnabled(LogLevel::ERROR, tag)) {
        return;
    }

    va_list args;
    va_start(args, format);
    LogInternal(LogLevel::ERROR, tag, config_.color, config_.style, format, args);
    va_end(args);
}

void Logger::Warn(const std::string& tag, const char* format, ...) noexcept {
    if (!IsLevelEnabled(LogLevel::WARN, tag)) {
        return;
    }

    va_list args;
    va_start(args, format);
    LogInternal(LogLevel::WARN, tag, config_.color, config_.style, format, args);
    va_end(args);
}

void Logger::Info(const std::string& tag, const char* format, ...) noexcept {
    if (!IsLevelEnabled(LogLevel::INFO, tag)) {
        return;
    }

    va_list args;
    va_start(args, format);
    LogInternal(LogLevel::INFO, tag, config_.color, config_.style, format, args);
    va_end(args);
}

void Logger::Debug(const std::string& tag, const char* format, ...) noexcept {
    if (!IsLevelEnabled(LogLevel::DEBUG, tag)) {
        return;
    }

    va_list args;
    va_start(args, format);
    LogInternal(LogLevel::DEBUG, tag, config_.color, config_.style, format, args);
    va_end(args);
}

void Logger::Verbose(const std::string& tag, const char* format, ...) noexcept {
    if (!IsLevelEnabled(LogLevel::VERBOSE, tag)) {
        return;
    }

    va_list args;
    va_start(args, format);
    LogInternal(LogLevel::VERBOSE, tag, config_.color, config_.style, format, args);
    va_end(args);
}

//==============================================================================
// FORMATTED LOGGING METHODS
//==============================================================================

void Logger::Error(const std::string& tag, LogColor color, LogStyle style, const char* format, ...) noexcept {
    if (!IsLevelEnabled(LogLevel::ERROR, tag)) {
        return;
    }

    va_list args;
    va_start(args, format);
    LogInternal(LogLevel::ERROR, tag, color, style, format, args);
    va_end(args);
}

void Logger::Warn(const std::string& tag, LogColor color, LogStyle style, const char* format, ...) noexcept {
    if (!IsLevelEnabled(LogLevel::WARN, tag)) {
        return;
    }

    va_list args;
    va_start(args, format);
    LogInternal(LogLevel::WARN, tag, color, style, format, args);
    va_end(args);
}

void Logger::Info(const std::string& tag, LogColor color, LogStyle style, const char* format, ...) noexcept {
    if (!IsLevelEnabled(LogLevel::INFO, tag)) {
        return;
    }

    va_list args;
    va_start(args, format);
    LogInternal(LogLevel::INFO, tag, color, style, format, args);
    va_end(args);
}

void Logger::Debug(const std::string& tag, LogColor color, LogStyle style, const char* format, ...) noexcept {
    if (!IsLevelEnabled(LogLevel::DEBUG, tag)) {
        return;
    }

    va_list args;
    va_start(args, format);
    LogInternal(LogLevel::DEBUG, tag, color, style, format, args);
    va_end(args);
}

void Logger::Verbose(const std::string& tag, LogColor color, LogStyle style, const char* format, ...) noexcept {
    if (!IsLevelEnabled(LogLevel::VERBOSE, tag)) {
        return;
    }

    va_list args;
    va_start(args, format);
    LogInternal(LogLevel::VERBOSE, tag, color, style, format, args);
    va_end(args);
}

//==============================================================================
// ASCII ART LOGGING METHODS
//==============================================================================

void Logger::LogAsciiArt(const std::string& tag, const std::string& ascii_art, 
                        const AsciiArtFormat& format) noexcept {
    if (!config_.enable_ascii_art || !IsLevelEnabled(LogLevel::INFO, tag)) {
        return;
    }

    std::string formatted_art = FormatAsciiArt(ascii_art, format);
    
    // Split into lines and log each line
    std::istringstream iss(formatted_art);
    std::string line;
    
    while (std::getline(iss, line)) {
        if (!line.empty()) {
            base_logger_->Info(tag.c_str(), "%s", line.c_str());
        }
    }
}

void Logger::LogAsciiArt(LogLevel level, const std::string& tag, const std::string& ascii_art, 
                        const AsciiArtFormat& format) noexcept {
    if (!config_.enable_ascii_art || !IsLevelEnabled(level, tag)) {
        return;
    }

    std::string formatted_art = FormatAsciiArt(ascii_art, format);
    
    // Split into lines and log each line
    std::istringstream iss(formatted_art);
    std::string line;
    
    while (std::getline(iss, line)) {
        if (!line.empty()) {
            base_logger_->Log(static_cast<hf_log_level_t>(level), tag.c_str(), "%s", line.c_str());
        }
    }
}

void Logger::LogBanner(const std::string& tag, const std::string& ascii_art, 
                      const AsciiArtFormat& format) noexcept {
    if (!config_.enable_ascii_art || !IsLevelEnabled(LogLevel::INFO, tag)) {
        return;
    }

    // Create a banner format with default styling
    AsciiArtFormat banner_format = format;
    if (banner_format.color == LogColor::DEFAULT) {
        banner_format.color = LogColor::BRIGHT_CYAN;
    }
    if (banner_format.style == LogStyle::NORMAL) {
        banner_format.style = LogStyle::BOLD;
    }
    if (!banner_format.center_art) {
        banner_format.center_art = true;
    }
    if (!banner_format.add_border) {
        banner_format.add_border = true;
        banner_format.border_char = '=';
        banner_format.border_padding = 1;
    }

    LogAsciiArt(tag, ascii_art, banner_format);
}

//==============================================================================
// UTILITY METHODS
//==============================================================================

void Logger::SetConfig(const LogConfig& config) noexcept {
    config_ = config;
    
    if (base_logger_) {
        hf_logger_config_t base_config;
        base_config.enable_colors = config.enable_colors;
        base_config.enable_effects = config.enable_effects;
        base_config.max_width = config.max_width;
        base_config.default_level = static_cast<hf_log_level_t>(config.level);
        
        base_logger_->Initialize(base_config);
    }
}

LogConfig Logger::GetConfig() const noexcept {
    return config_;
}

void Logger::EnableColors(bool enable) noexcept {
    config_.enable_colors = enable;
    if (base_logger_) {
        hf_logger_config_t base_config;
        base_config.enable_colors = enable;
        base_config.enable_effects = config_.enable_effects;
        base_config.max_width = config_.max_width;
        base_config.default_level = static_cast<hf_log_level_t>(config_.level);
        
        base_logger_->Initialize(base_config);
    }
}

void Logger::EnableEffects(bool enable) noexcept {
    config_.enable_effects = enable;
    if (base_logger_) {
        hf_logger_config_t base_config;
        base_config.enable_colors = config_.enable_colors;
        base_config.enable_effects = enable;
        base_config.max_width = config_.max_width;
        base_config.default_level = static_cast<hf_log_level_t>(config_.level);
        
        base_logger_->Initialize(base_config);
    }
}

void Logger::EnableAsciiArt(bool enable) noexcept {
    config_.enable_ascii_art = enable;
}

void Logger::Flush() noexcept {
    if (base_logger_) {
        base_logger_->Flush();
    }
}

//==============================================================================
// PRIVATE METHODS
//==============================================================================

void Logger::LogInternal(LogLevel level, const std::string& tag, LogColor color, LogStyle style, 
                        const char* format, va_list args) noexcept {
    if (!initialized_.load() || !base_logger_) {
        return;
    }

    // Format the message
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    std::string message(buffer);

    // Add color codes if enabled
    if (config_.enable_colors) {
        message = AddColorCodes(message, color, config_.background, style);
    }

    // Log using base logger
    base_logger_->Log(static_cast<hf_log_level_t>(level), tag.c_str(), "%s", message.c_str());
}

std::string Logger::FormatAsciiArt(const std::string& ascii_art, const AsciiArtFormat& format) const noexcept {
    if (!config_.format_ascii_art) {
        return ascii_art;
    }

    std::string result = ascii_art;

    // Split into lines for processing
    std::istringstream iss(ascii_art);
    std::vector<std::string> lines;
    std::string line;
    
    while (std::getline(iss, line)) {
        lines.push_back(line);
    }

    // Apply centering
    if (format.center_art) {
        size_t max_length = 0;
        for (const auto& l : lines) {
            max_length = std::max(max_length, l.length());
        }

        size_t padding = (format.max_width > max_length) ? (format.max_width - max_length) / 2 : 0;
        
        for (auto& l : lines) {
            l = std::string(padding, ' ') + l;
        }
    }

    // Apply border
    if (format.add_border) {
        size_t max_length = 0;
        for (const auto& l : lines) {
            max_length = std::max(max_length, l.length());
        }

        std::string top_border = std::string(format.border_padding, format.border_char) + 
                                std::string(max_length + 2, format.border_char) + 
                                std::string(format.border_padding, format.border_char);

        std::vector<std::string> bordered_lines;
        bordered_lines.push_back(top_border);

        // Add padding lines
        for (size_t i = 0; i < format.border_padding; ++i) {
            bordered_lines.push_back(std::string(top_border.length(), format.border_char));
        }

        // Add art lines with side borders
        for (const auto& l : lines) {
            std::string bordered_line = std::string(format.border_padding, format.border_char) + 
                                       " " + l + 
                                       std::string(max_length - l.length(), ' ') + " " + 
                                       std::string(format.border_padding, format.border_char);
            bordered_lines.push_back(bordered_line);
        }

        // Add padding lines
        for (size_t i = 0; i < format.border_padding; ++i) {
            bordered_lines.push_back(std::string(top_border.length(), format.border_char));
        }

        // Add bottom border
        bordered_lines.push_back(top_border);

        lines = bordered_lines;
    }

    // Apply color codes to each line
    if (config_.enable_colors) {
        for (auto& l : lines) {
            l = AddColorCodes(l, format.color, format.background, format.style);
        }
    }

    // Reconstruct the result
    result.clear();
    for (size_t i = 0; i < lines.size(); ++i) {
        result += lines[i];
        if (i < lines.size() - 1) {
            result += "\n";
        }
    }

    return result;
}

std::string Logger::AddColorCodes(const std::string& text, LogColor color, LogBackground background, 
                                 LogStyle style) const noexcept {
    if (!config_.enable_colors) {
        return text;
    }

    std::string result;
    
    // Add color sequence
    if (color != LogColor::DEFAULT) {
        result += GetColorSequence(color);
    }
    
    // Add background sequence
    if (background != LogBackground::DEFAULT) {
        result += GetBackgroundSequence(background);
    }
    
    // Add style sequence
    if (style != LogStyle::NORMAL) {
        result += GetStyleSequence(style);
    }
    
    result += text;
    result += GetResetSequence();
    
    return result;
}

std::string Logger::GetColorSequence(LogColor color) const noexcept {
    if (color == LogColor::DEFAULT) {
        return "";
    }
    return "\033[" + std::to_string(static_cast<uint8_t>(color)) + "m";
}

std::string Logger::GetBackgroundSequence(LogBackground background) const noexcept {
    if (background == LogBackground::DEFAULT) {
        return "";
    }
    return "\033[" + std::to_string(static_cast<uint8_t>(background)) + "m";
}

std::string Logger::GetStyleSequence(LogStyle style) const noexcept {
    switch (style) {
        case LogStyle::BOLD:
            return "\033[1m";
        case LogStyle::ITALIC:
            return "\033[3m";
        case LogStyle::UNDERLINE:
            return "\033[4m";
        case LogStyle::STRIKETHROUGH:
            return "\033[9m";
        case LogStyle::DOUBLE_UNDERLINE:
            return "\033[21m";
        default:
            return "";
    }
}

std::string Logger::GetResetSequence() const noexcept {
    return "\033[0m";
}

bool Logger::IsLevelEnabled(LogLevel level, const std::string& tag) const noexcept {
    if (!initialized_.load()) {
        return false;
    }

    LogLevel tag_level = GetLogLevel(tag);
    return static_cast<uint8_t>(level) <= static_cast<uint8_t>(tag_level);
}

std::unique_ptr<BaseLogger> Logger::CreateBaseLogger() noexcept {
    // Create an EspLogger directly
#ifdef HF_MCU_FAMILY_ESP32
    return std::make_unique<EspLogger>();
#else
    return nullptr; // Fallback for other platforms
#endif
}

void Logger::DumpStatistics() const noexcept {
    static constexpr const char* TAG = "Logger";
    
    // Use direct output since we're logging the logger itself
    if (!base_logger_) {
        printf("[%s] ERROR: Logger not initialized, cannot dump statistics\n", TAG);
        return;
    }
    
    printf("[%s] INFO: === LOGGER STATISTICS ===\n", TAG);
    
    // System Health
    printf("[%s] INFO: System Health:\n", TAG);
    printf("[%s] INFO:   Initialized: %s\n", TAG, initialized_ ? "YES" : "NO");
    printf("[%s] INFO:   Base Logger: %s\n", TAG, base_logger_ ? "ACTIVE" : "INACTIVE");
    
    // Configuration
    printf("[%s] INFO: Configuration:\n", TAG);
    printf("[%s] INFO:   Default Log Level: %s\n", TAG, 
        config_.level == LogLevel::ERROR ? "ERROR" :
        config_.level == LogLevel::WARN ? "WARN" :
        config_.level == LogLevel::INFO ? "INFO" :
        config_.level == LogLevel::DEBUG ? "DEBUG" :
        config_.level == LogLevel::VERBOSE ? "VERBOSE" : "UNKNOWN");
    
    printf("[%s] INFO:   Colors Enabled: %s\n", TAG, config_.enable_colors ? "YES" : "NO");
    printf("[%s] INFO:   Effects Enabled: %s\n", TAG, config_.enable_effects ? "YES" : "NO");
    printf("[%s] INFO:   ASCII Art Enabled: %s\n", TAG, config_.enable_ascii_art ? "YES" : "NO");
    printf("[%s] INFO:   Max Width: %d\n", TAG, config_.max_width);
    printf("[%s] INFO:   Border Character: '%c'\n", TAG, config_.border_char);
    
    // Tag-specific levels
    printf("[%s] INFO: Tag-specific Log Levels:\n", TAG);
    if (tag_levels_.empty()) {
        printf("[%s] INFO:   No tag-specific levels configured\n", TAG);
    } else {
        printf("[%s] INFO:   %d tag-specific levels configured:\n", TAG, static_cast<int>(tag_levels_.size()));
        int count = 0;
        for (const auto& pair : tag_levels_) {
            if (count >= 10) { // Limit output to prevent spam
                printf("[%s] INFO:   ... and %d more\n", TAG, static_cast<int>(tag_levels_.size()) - count);
                break;
            }
            const char* level_str = 
                pair.second == LogLevel::ERROR ? "ERROR" :
                pair.second == LogLevel::WARN ? "WARN" :
                pair.second == LogLevel::INFO ? "INFO" :
                pair.second == LogLevel::DEBUG ? "DEBUG" :
                pair.second == LogLevel::VERBOSE ? "VERBOSE" : "UNKNOWN";
            printf("[%s] INFO:     %s: %s\n", TAG, pair.first.c_str(), level_str);
            count++;
        }
    }
    
    // Memory Usage
    printf("[%s] INFO: Memory Usage:\n", TAG);
    size_t config_memory = sizeof(config_);
    size_t tag_levels_memory = tag_levels_.size() * (sizeof(std::string) + sizeof(LogLevel));
    size_t total_memory = sizeof(*this) + tag_levels_memory;
    
    printf("[%s] INFO:   Logger Instance: %d bytes\n", TAG, static_cast<int>(sizeof(*this)));
    printf("[%s] INFO:   Configuration: %d bytes\n", TAG, static_cast<int>(config_memory));
    printf("[%s] INFO:   Tag Levels Map: %d bytes\n", TAG, static_cast<int>(tag_levels_memory));
    printf("[%s] INFO:   Total Estimated: %d bytes\n", TAG, static_cast<int>(total_memory));
    
    // Platform Information
    printf("[%s] INFO: Platform Information:\n", TAG);
#ifdef HF_MCU_FAMILY_ESP32
    printf("[%s] INFO:   Platform: ESP32\n", TAG);
    printf("[%s] INFO:   Base Logger Type: EspLogger\n", TAG);
#else
    printf("[%s] INFO:   Platform: Other\n", TAG);
    printf("[%s] INFO:   Base Logger Type: Generic\n", TAG);
#endif
    
    // Feature Support
    printf("[%s] INFO: Feature Support:\n", TAG);
    printf("[%s] INFO:   ANSI Colors: %s\n", TAG, "SUPPORTED");
    printf("[%s] INFO:   Text Styles: %s\n", TAG, "SUPPORTED");
    printf("[%s] INFO:   ASCII Art: %s\n", TAG, "SUPPORTED");
    printf("[%s] INFO:   Formatted Output: %s\n", TAG, "SUPPORTED");
    printf("[%s] INFO:   Thread Safety: %s\n", TAG, "SUPPORTED");
    
    printf("[%s] INFO: === END LOGGER STATISTICS ===\n", TAG);
} 