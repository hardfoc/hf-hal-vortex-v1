###############################################################################
#  HardFOC HAL – **single-component** build script (Vortex V1)
#
#  Uses core/cmake/hf_core_build_settings.cmake for configurable driver
#  selection. Only Vortex-needed drivers are enabled — see Feature Config.
###############################################################################

cmake_minimum_required(VERSION 3.16)

set(HF_HAL_ROOT ${CMAKE_CURRENT_LIST_DIR})

# Include the API directory for Vortex API component
list(APPEND EXTRA_COMPONENT_DIRS "${HF_HAL_ROOT}/lib/api")

#==============================================================================
# 1. CORE FEATURE CONFIGURATION — Enable only what Vortex needs
#==============================================================================

# ── Platform Selection ────────────────────────────────────────────────────
set(HF_CORE_MCU   "ESP32")              # MCU family (ESP32, STM32 future)
set(HF_CORE_RTOS  "FREERTOS")           # RTOS (FREERTOS, THREADX future)

# ── External Drivers + Handlers ───────────────────────────────────────────
set(HF_CORE_ENABLE_TMC9660        ON)   # BLDC/stepper motor controller
set(HF_CORE_ENABLE_PCAL95555      ON)   # I2C GPIO expander
set(HF_CORE_ENABLE_AS5047U        ON)   # SPI magnetic encoder
set(HF_CORE_ENABLE_BNO08X         ON)   # I2C/SPI IMU
set(HF_CORE_ENABLE_NTC_THERMISTOR ON)   # ADC temperature sensor
set(HF_CORE_ENABLE_WS2812         ON)   # RMT LED strip
set(HF_CORE_ENABLE_LOGGER         ON)   # Logging system

# ── Optional Utilities ────────────────────────────────────────────────────
set(HF_CORE_ENABLE_UTILS_CANOPEN  ON)   # CANopen protocol helpers

# ── Optional Interfaces ──────────────────────────────────────────────────
# UART auto-enabled by TMC9660, but explicit for clarity
set(HF_CORE_ENABLE_UART           ON)
set(HF_CORE_ENABLE_CAN            ON)
set(HF_CORE_ENABLE_PWM            ON)
set(HF_CORE_ENABLE_PIO            ON)

# ── Include core build settings (collects sources, includes, IDF deps) ───
include("${HF_HAL_ROOT}/lib/core/cmake/hf_core_build_settings.cmake")

#==============================================================================
# 2. HAL-SPECIFIC SOURCES
#==============================================================================

# API Layer
set(API_SOURCES
    "${HF_HAL_ROOT}/lib/api/Vortex.cpp"
)

# Manager Sources (8 managers)
set(MANAGER_SOURCES
    "${HF_HAL_ROOT}/lib/managers/CommChannelsManager.cpp"
    "${HF_HAL_ROOT}/lib/managers/GpioManager.cpp"
    "${HF_HAL_ROOT}/lib/managers/AdcManager.cpp"
    "${HF_HAL_ROOT}/lib/managers/MotorController.cpp"
    "${HF_HAL_ROOT}/lib/managers/ImuManager.cpp"
    "${HF_HAL_ROOT}/lib/managers/EncoderManager.cpp"
    "${HF_HAL_ROOT}/lib/managers/LedManager.cpp"
    "${HF_HAL_ROOT}/lib/managers/TemperatureManager.cpp"
)

# Combine HAL + core sources
set(HF_HAL_SRCS
    ${API_SOURCES}
    ${MANAGER_SOURCES}
    ${HF_CORE_SOURCES}
)

#==============================================================================
# 3. INCLUDE DIRECTORIES
#==============================================================================

set(HF_HAL_INC_DIRS
    # HAL-specific
    "${HF_HAL_ROOT}/lib"
    "${HF_HAL_ROOT}/lib/api"
    "${HF_HAL_ROOT}/lib/managers"
    # Core (handlers, drivers, utils, interfaces)
    ${HF_CORE_INCLUDE_DIRS}
)

#==============================================================================
# 4. COMPONENT REGISTRATION
#==============================================================================

idf_component_register(
    SRCS          ${HF_HAL_SRCS}
    INCLUDE_DIRS  ${HF_HAL_INC_DIRS}
    REQUIRES      ${HF_CORE_IDF_REQUIRES}
)

#==============================================================================
# 5. COMPILE OPTIONS AND DEFINITIONS
#==============================================================================

# Strict compile options for HAL code quality
# Note: -Werror=padded, -Werror=suggest-*, -Werror=unused-function removed
# because they fire on core/third-party struct layouts, header-only static
# functions, and virtual method patterns the HAL cannot control.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-error=undef
    -Werror=null-dereference
    -Werror=nonnull
    -Werror=nonnull-compare
    -Werror=address
    -Werror=alloca
    -Werror=builtin-macro-redefined
    -Werror=div-by-zero
    -Wno-error=duplicated-cond
    -Werror=empty-body
    -Werror=enum-compare
    -Werror=enum-conversion
    -Werror=free-nonheap-object
    -Werror=ignored-qualifiers
    -Werror=implicit
    -Werror=implicit-function-declaration
    -Werror=init-self
    -Werror=logical-not-parentheses
    -Werror=logical-op
    -Werror=main
    -Werror=maybe-uninitialized
    -Werror=memset-transposed-args
    -Werror=mismatched-dealloc
    -Werror=mismatched-new-delete
    -Werror=missing-include-dirs
    -Werror=multichar
    -Werror=multistatement-macros
    -Werror=narrowing
    -Wno-error=noexcept
    -Werror=non-template-friend
    -Werror=parentheses
    -Werror=pointer-arith
    -Werror=pointer-sign
    -Werror=reorder
    -Werror=return-local-addr
    -Werror=sequence-point
    -Werror=shift-count-negative
    -Werror=shift-count-overflow
    -Werror=shift-negative-value
    -Werror=shift-overflow
    -Werror=shift-overflow=2
    -Werror=switch-bool
    -Werror=tautological-compare
    -Werror=trigraphs
    -Werror=type-limits
    -Werror=uninitialized
    -Werror=unknown-pragmas
    -Werror=unreachable-code
    -Werror=varargs
    -Werror=variadic-macros
    -Wno-error=vla
    -Werror=volatile-register-var
    -Werror=write-strings
    -Wno-error=zero-as-null-pointer-constant
    -Wno-error=format=
    # Warnings kept but not treated as errors (fire on core/third-party code)
    -Wpadded
    -Wunused-function
    -Wno-unused-parameter
    -Wno-missing-field-initializers
    -Wno-sign-compare
    -Wno-volatile
    -Wno-duplicated-branches
    -Wno-missing-braces
    -Wno-missing-declarations
    -Wno-overloaded-virtual
)

# WS2812 Kconfig defaults — PUBLIC because ws2812 headers are transitively
# included via LedManager → ws2812_cpp.hpp → ws2812_control.h
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    CONFIG_WS2812_NUM_LEDS=1
    CONFIG_WS2812_LED_TYPE_RGB=1
    CONFIG_WS2812_T0H=350
    CONFIG_WS2812_T1H=900
    CONFIG_WS2812_T0L=900
    CONFIG_WS2812_T1L=350
    CONFIG_WS2812_DEFAULT_BRIGHTNESS=25
    CONFIG_WS2812_LED_RMT_TX_GPIO=48
    CONFIG_WS2812_LED_RMT_TX_CHANNEL=0
)

# HAL-specific compile definitions (merged with core feature definitions)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    # Core feature definitions (from build settings)
    ${HF_CORE_COMPILE_DEFINITIONS}
    # HAL identity
    HARDFOC_HAL=1
    HARDFOC_VORTEX_V1=1
    # HAL component flags
    HARDFOC_COMPONENT_HANDLER=1
    HARDFOC_GPIO_MANAGER=1
    HARDFOC_ADC_MANAGER=1
    HARDFOC_MOTOR_CONTROLLER=1
    HARDFOC_IMU_MANAGER=1
    HARDFOC_ENCODER_MANAGER=1
    HARDFOC_LED_MANAGER=1
    HARDFOC_TEMPERATURE_MANAGER=1
    HARDFOC_COMM_CHANNELS_MANAGER=1
    HARDFOC_PLATFORM_MAPPING=1
    HARDFOC_VORTEX_API=1
    HARDFOC_DRIVER_HANDLERS=1
)

# GNU++20 for Vortex HAL (required by TMC9660 driver's std::span usage;
# GNU extensions required because ESP-IDF HAL headers use typeof/statement-exprs)
set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)

#==============================================================================
# 6. BUILD SUMMARY
#==============================================================================

list(LENGTH HF_HAL_SRCS _hal_total_srcs)
list(LENGTH API_SOURCES _hal_api_count)
list(LENGTH MANAGER_SOURCES _hal_mgr_count)
message(STATUS "")
message(STATUS "╔══════════════════════════════════════════════════════╗")
message(STATUS "║         Vortex V1 HAL Build Configuration           ║")
message(STATUS "╚══════════════════════════════════════════════════════╝")
message(STATUS "  HAL sources:    ${_hal_api_count} API + ${_hal_mgr_count} managers")
message(STATUS "  Total sources:  ${_hal_total_srcs}")
message(STATUS "  Component:      ${COMPONENT_LIB}")
message(STATUS "")
