/**
 * @file Logger.h
 * @brief Advanced logging system with formatting capabilities.
 *
 * This file provides a comprehensive logging system that supports
 * various formatting options including colors, styles, and effects.
 * It can also handle ASCII art formatting and display.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <map>

// Forward declarations
class BaseLogger;

/**
 * @brief Text formatting styles
 */
enum class LogStyle : uint8_t {
    NORMAL = 0,         ///< Normal text
    BOLD = 1,           ///< Bold text
    ITALIC = 2,         ///< Italic text
    UNDERLINE = 3,      ///< Underlined text
    STRIKETHROUGH = 4,  ///< Strikethrough text
    DOUBLE_UNDERLINE = 5 ///< Double underlined text
};

/**
 * @brief Text colors (ANSI color codes)
 */
enum class LogColor : uint8_t {
    DEFAULT = 0,        ///< Default color
    BLACK = 30,         ///< Black
    RED = 31,           ///< Red
    GREEN = 32,         ///< Green
    YELLOW = 33,        ///< Yellow
    BLUE = 34,          ///< Blue
    MAGENTA = 35,       ///< Magenta
    CYAN = 36,          ///< Cyan
    WHITE = 37,         ///< White
    BRIGHT_BLACK = 90,  ///< Bright black
    BRIGHT_RED = 91,    ///< Bright red
    BRIGHT_GREEN = 92,  ///< Bright green
    BRIGHT_YELLOW = 93, ///< Bright yellow
    BRIGHT_BLUE = 94,   ///< Bright blue
    BRIGHT_MAGENTA = 95,///< Bright magenta
    BRIGHT_CYAN = 96,   ///< Bright cyan
    BRIGHT_WHITE = 97   ///< Bright white
};

/**
 * @brief Background colors (ANSI color codes)
 */
enum class LogBackground : uint8_t {
    DEFAULT = 0,        ///< Default background
    BLACK = 40,         ///< Black background
    RED = 41,           ///< Red background
    GREEN = 42,         ///< Green background
    YELLOW = 43,        ///< Yellow background
    BLUE = 44,          ///< Blue background
    MAGENTA = 45,       ///< Magenta background
    CYAN = 46,          ///< Cyan background
    WHITE = 47,         ///< White background
    BRIGHT_BLACK = 100, ///< Bright black background
    BRIGHT_RED = 101,   ///< Bright red background
    BRIGHT_GREEN = 102, ///< Bright green background
    BRIGHT_YELLOW = 103,///< Bright yellow background
    BRIGHT_BLUE = 104,  ///< Bright blue background
    BRIGHT_MAGENTA = 105,///< Bright magenta background
    BRIGHT_CYAN = 106,  ///< Bright cyan background
    BRIGHT_WHITE = 107  ///< Bright white background
};

/**
 * @brief Log levels
 */
enum class LogLevel : uint8_t {
    ERROR = 0,          ///< Error level
    WARN = 1,           ///< Warning level
    INFO = 2,           ///< Info level
    DEBUG = 3,          ///< Debug level
    VERBOSE = 4         ///< Verbose level
};

/**
 * @brief Logging configuration
 */
struct LogConfig {
    LogLevel level = LogLevel::INFO;           ///< Default log level
    LogColor color = LogColor::DEFAULT;        ///< Default text color
    LogBackground background = LogBackground::DEFAULT; ///< Default background
    LogStyle style = LogStyle::NORMAL;         ///< Default text style
    bool enable_colors = true;                 ///< Enable ANSI colors
    bool enable_effects = true;                ///< Enable special effects
    uint32_t max_width = 80;                   ///< Maximum output width
    bool center_text = false;                  ///< Center the text
    bool add_border = false;                   ///< Add border around text
    char border_char = '*';                    ///< Border character
    uint32_t border_padding = 2;               ///< Border padding
    bool enable_ascii_art = true;              ///< Enable ASCII art support
    bool format_ascii_art = true;              ///< Apply formatting to ASCII art
};

/**
 * @brief ASCII art formatting options
 */
struct AsciiArtFormat {
    LogColor color = LogColor::DEFAULT;        ///< ASCII art color
    LogBackground background = LogBackground::DEFAULT; ///< ASCII art background
    LogStyle style = LogStyle::NORMAL;         ///< ASCII art style
    bool center_art = false;                   ///< Center the ASCII art
    bool add_border = false;                   ///< Add border around ASCII art
    char border_char = '#';                    ///< Border character
    uint32_t border_padding = 1;               ///< Border padding
    uint32_t max_width = 80;                   ///< Maximum width for centering
};

/**
 * @class Logger
 * @brief Advanced logging system with formatting capabilities.
 *
 * This class provides a comprehensive logging solution that supports
 * various formatting options including colors, styles, and effects.
 * It can also handle ASCII art formatting and display.
 *
 * Key Features:
 * - **Multiple Log Levels**: Error, Warn, Info, Debug, Verbose
 * - **Color Support**: ANSI color codes for text and background
 * - **Text Styling**: Bold, italic, underline, strikethrough
 * - **ASCII Art Support**: Format and display ASCII art
 * - **Effects**: Borders, centering, padding
 * - **Performance Optimized**: Efficient string handling
 * - **Thread Safe**: Safe for multi-threaded environments
 *
 * Usage Example:
 * @code
 * Logger& logger = Logger::GetInstance();
 * 
 * // Basic logging
 * logger.Info("TAG", "Hello World");
 * 
 * // With formatting
 * logger.Info("TAG", "Hello World", LogColor::RED, LogStyle::BOLD);
 * 
 * // ASCII art
 * std::string ascii_art = "..."; // ASCII art string
 * logger.LogAsciiArt("BANNER", ascii_art, AsciiArtFormat{});
 * @endcode
 */
class Logger {
public:
    /**
     * @brief Get singleton instance
     * @return Logger instance
     */
    static Logger& GetInstance() noexcept;

    /**
     * @brief Initialize the logger
     * @param config Configuration options
     * @return true if successful, false otherwise
     */
    bool Initialize(const LogConfig& config = LogConfig{}) noexcept;

    /**
     * @brief Deinitialize the logger
     */
    void Deinitialize() noexcept;

    /**
     * @brief Check if logger is initialized
     * @return true if initialized, false otherwise
     */
    bool IsInitialized() const noexcept;

    /**
     * @brief Set log level for a tag
     * @param tag Log tag
     * @param level Log level
     */
    void SetLogLevel(const std::string& tag, LogLevel level) noexcept;

    /**
     * @brief Get log level for a tag
     * @param tag Log tag
     * @return Log level
     */
    LogLevel GetLogLevel(const std::string& tag) const noexcept;

    //==============================================================================
    // BASIC LOGGING METHODS
    //==============================================================================

    /**
     * @brief Log error message
     * @param tag Log tag
     * @param format Format string
     * @param ... Variable arguments
     */
    void Error(const std::string& tag, const char* format, ...) noexcept;

    /**
     * @brief Log warning message
     * @param tag Log tag
     * @param format Format string
     * @param ... Variable arguments
     */
    void Warn(const std::string& tag, const char* format, ...) noexcept;

    /**
     * @brief Log info message
     * @param tag Log tag
     * @param format Format string
     * @param ... Variable arguments
     */
    void Info(const std::string& tag, const char* format, ...) noexcept;

    /**
     * @brief Log debug message
     * @param tag Log tag
     * @param format Format string
     * @param ... Variable arguments
     */
    void Debug(const std::string& tag, const char* format, ...) noexcept;

    /**
     * @brief Log verbose message
     * @param tag Log tag
     * @param format Format string
     * @param ... Variable arguments
     */
    void Verbose(const std::string& tag, const char* format, ...) noexcept;

    //==============================================================================
    // FORMATTED LOGGING METHODS
    //==============================================================================

    /**
     * @brief Log error message with formatting
     * @param tag Log tag
     * @param color Text color
     * @param style Text style
     * @param format Format string
     * @param ... Variable arguments
     */
    void Error(const std::string& tag, LogColor color, LogStyle style, const char* format, ...) noexcept;

    /**
     * @brief Log warning message with formatting
     * @param tag Log tag
     * @param color Text color
     * @param style Text style
     * @param format Format string
     * @param ... Variable arguments
     */
    void Warn(const std::string& tag, LogColor color, LogStyle style, const char* format, ...) noexcept;

    /**
     * @brief Log info message with formatting
     * @param tag Log tag
     * @param color Text color
     * @param style Text style
     * @param format Format string
     * @param ... Variable arguments
     */
    void Info(const std::string& tag, LogColor color, LogStyle style, const char* format, ...) noexcept;

    /**
     * @brief Log debug message with formatting
     * @param tag Log tag
     * @param color Text color
     * @param style Text style
     * @param format Format string
     * @param ... Variable arguments
     */
    void Debug(const std::string& tag, LogColor color, LogStyle style, const char* format, ...) noexcept;

    /**
     * @brief Log verbose message with formatting
     * @param tag Log tag
     * @param color Text color
     * @param style Text style
     * @param format Format string
     * @param ... Variable arguments
     */
    void Verbose(const std::string& tag, LogColor color, LogStyle style, const char* format, ...) noexcept;

    //==============================================================================
    // ASCII ART LOGGING METHODS
    //==============================================================================

    /**
     * @brief Log ASCII art
     * @param tag Log tag
     * @param ascii_art ASCII art string
     * @param format Formatting options
     */
    void LogAsciiArt(const std::string& tag, const std::string& ascii_art, 
                    const AsciiArtFormat& format = AsciiArtFormat{}) noexcept;

    /**
     * @brief Log ASCII art with log level
     * @param level Log level
     * @param tag Log tag
     * @param ascii_art ASCII art string
     * @param format Formatting options
     */
    void LogAsciiArt(LogLevel level, const std::string& tag, const std::string& ascii_art, 
                    const AsciiArtFormat& format = AsciiArtFormat{}) noexcept;

    /**
     * @brief Log ASCII art banner
     * @param tag Log tag
     * @param ascii_art ASCII art string
     * @param format Formatting options
     */
    void LogBanner(const std::string& tag, const std::string& ascii_art, 
                  const AsciiArtFormat& format = AsciiArtFormat{}) noexcept;

    //==============================================================================
    // UTILITY METHODS
    //==============================================================================

    /**
     * @brief Set default configuration
     * @param config Configuration
     */
    void SetConfig(const LogConfig& config) noexcept;

    /**
     * @brief Get current configuration
     * @return Current configuration
     */
    LogConfig GetConfig() const noexcept;

    /**
     * @brief Enable/disable colors
     * @param enable true to enable, false to disable
     */
    void EnableColors(bool enable) noexcept;

    /**
     * @brief Enable/disable effects
     * @param enable true to enable, false to disable
     */
    void EnableEffects(bool enable) noexcept;

    /**
     * @brief Enable/disable ASCII art support
     * @param enable true to enable, false to disable
     */
    void EnableAsciiArt(bool enable) noexcept;

    /**
     * @brief Flush any buffered output
     */
    void Flush() noexcept;

private:
    //==============================================================================
    // PRIVATE MEMBERS
    //==============================================================================

    std::atomic<bool> initialized_;
    LogConfig config_;
    std::map<std::string, LogLevel> tag_levels_;
    std::unique_ptr<BaseLogger> base_logger_;

    //==============================================================================
    // PRIVATE METHODS
    //==============================================================================

    /**
     * @brief Constructor
     */
    Logger() noexcept;

    /**
     * @brief Destructor
     */
    ~Logger() noexcept;

    /**
     * @brief Copy constructor (deleted)
     */
    Logger(const Logger&) = delete;

    /**
     * @brief Assignment operator (deleted)
     */
    Logger& operator=(const Logger&) = delete;

    /**
     * @brief Move constructor (deleted)
     */
    Logger(Logger&&) = delete;

    /**
     * @brief Move assignment operator (deleted)
     */
    Logger& operator=(Logger&&) = delete;

    /**
     * @brief Internal logging method
     * @param level Log level
     * @param tag Log tag
     * @param color Text color
     * @param style Text style
     * @param format Format string
     * @param args Variable arguments
     */
    void LogInternal(LogLevel level, const std::string& tag, LogColor color, LogStyle style, 
                    const char* format, va_list args) noexcept;

    /**
     * @brief Format ASCII art
     * @param ascii_art ASCII art string
     * @param format Formatting options
     * @return Formatted ASCII art
     */
    std::string FormatAsciiArt(const std::string& ascii_art, const AsciiArtFormat& format) const noexcept;

    /**
     * @brief Add color codes to text
     * @param text Text to colorize
     * @param color Text color
     * @param background Background color
     * @param style Text style
     * @return Colorized text
     */
    std::string AddColorCodes(const std::string& text, LogColor color, LogBackground background, 
                             LogStyle style) const noexcept;

    /**
     * @brief Get ANSI escape sequence for color
     * @param color Color to get sequence for
     * @return ANSI escape sequence
     */
    std::string GetColorSequence(LogColor color) const noexcept;

    /**
     * @brief Get ANSI escape sequence for background
     * @param background Background to get sequence for
     * @return ANSI escape sequence
     */
    std::string GetBackgroundSequence(LogBackground background) const noexcept;

    /**
     * @brief Get ANSI escape sequence for style
     * @param style Style to get sequence for
     * @return ANSI escape sequence
     */
    std::string GetStyleSequence(LogStyle style) const noexcept;

    /**
     * @brief Get ANSI reset sequence
     * @return ANSI reset sequence
     */
    std::string GetResetSequence() const noexcept;

    /**
     * @brief Check if log level is enabled for tag
     * @param level Log level
     * @param tag Log tag
     * @return true if enabled, false otherwise
     */
    bool IsLevelEnabled(LogLevel level, const std::string& tag) const noexcept;

    /**
     * @brief Create base logger instance
     * @return Base logger instance
     */
    std::unique_ptr<BaseLogger> CreateBaseLogger() noexcept;
    
    /**
     * @brief Dump comprehensive logger statistics to log as INFO level.
     * Logs internal logger statistics, configuration, and performance metrics.
     */
    void DumpStatistics() const noexcept;
};

//==============================================================================
// CONVENIENCE MACROS
//==============================================================================

#define LOG_ERROR(tag, ...) Logger::GetInstance().Error(tag, __VA_ARGS__)
#define LOG_WARN(tag, ...) Logger::GetInstance().Warn(tag, __VA_ARGS__)
#define LOG_INFO(tag, ...) Logger::GetInstance().Info(tag, __VA_ARGS__)
#define LOG_DEBUG(tag, ...) Logger::GetInstance().Debug(tag, __VA_ARGS__)
#define LOG_VERBOSE(tag, ...) Logger::GetInstance().Verbose(tag, __VA_ARGS__)

#define LOG_ERROR_FORMATTED(tag, color, style, ...) Logger::GetInstance().Error(tag, color, style, __VA_ARGS__)
#define LOG_WARN_FORMATTED(tag, color, style, ...) Logger::GetInstance().Warn(tag, color, style, __VA_ARGS__)
#define LOG_INFO_FORMATTED(tag, color, style, ...) Logger::GetInstance().Info(tag, color, style, __VA_ARGS__)
#define LOG_DEBUG_FORMATTED(tag, color, style, ...) Logger::GetInstance().Debug(tag, color, style, __VA_ARGS__)
#define LOG_VERBOSE_FORMATTED(tag, color, style, ...) Logger::GetInstance().Verbose(tag, color, style, __VA_ARGS__)

#define LOG_ASCII_ART(tag, art, format) Logger::GetInstance().LogAsciiArt(tag, art, format)
#define LOG_BANNER(tag, art, format) Logger::GetInstance().LogBanner(tag, art, format) 