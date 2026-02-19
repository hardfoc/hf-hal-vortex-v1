###############################################################################
#  HardFOC HAL – **single-component** build script
#  Copy this file to hf-hal/CMakeLists.txt
###############################################################################

cmake_minimum_required(VERSION 3.16)

set(HF_HAL_ROOT ${CMAKE_CURRENT_LIST_DIR})

#==============================================================================
# SYSTEMATIC FILE COLLECTION - VORTEX API DEPENDENCY ANALYSIS
#==============================================================================

# Expose external drivers as components for header access (if they exist in workspace)
# list(APPEND EXTRA_COMPONENT_DIRS "${HF_HAL_ROOT}/lib/core/hf-core-drivers/external/")
# list(APPEND EXTRA_COMPONENT_DIRS "${HF_HAL_ROOT}/lib/core/hf-core-drivers/external/hf-ws2812-rmt-driver")

# Include the API directory for Vortex API component
list(APPEND EXTRA_COMPONENT_DIRS "${HF_HAL_ROOT}/lib/api")

#==============================================================================
# 1. SYSTEMATIC SOURCE FILE COLLECTION
#==============================================================================

# API Layer Sources
set(API_SOURCES
    "${HF_HAL_ROOT}/lib/api/Vortex.cpp"
)

# Manager Sources (8 managers, formerly component-handlers)
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

# Handler Sources (from hf-core submodule: lib/core/handlers/<driver>/)
set(HANDLER_SOURCES
    "${HF_HAL_ROOT}/lib/core/handlers/tmc9660/Tmc9660Handler.cpp"
    "${HF_HAL_ROOT}/lib/core/handlers/tmc9660/Tmc9660AdcWrapper.cpp"
    "${HF_HAL_ROOT}/lib/core/handlers/pcal95555/Pcal95555Handler.cpp"
    "${HF_HAL_ROOT}/lib/core/handlers/as5047u/As5047uHandler.cpp"
    "${HF_HAL_ROOT}/lib/core/handlers/bno08x/Bno08xHandler.cpp"
    "${HF_HAL_ROOT}/lib/core/handlers/ntc/NtcTemperatureHandler.cpp"
    "${HF_HAL_ROOT}/lib/core/handlers/logger/Logger.cpp"
)

# Core Drivers Sources (Interface Wrappers)
set(CORE_DRIVER_SOURCES
    # ESP32 implementations
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspSpi.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspI2c.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspGpio.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspAdc.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspUart.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspCan.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspPwm.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspPio.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspNvs.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspLogger.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspTemperature.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/src/mcu/esp32/EspPeriodicTimer.cpp"
)

# Core Utils Sources
set(CORE_UTILS_SOURCES
    # RTOS wrappers
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-rtos-wrap/src/BaseThread.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-rtos-wrap/src/CriticalSection.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-rtos-wrap/src/FreeRTOSUtils.c"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-rtos-wrap/src/Mutex.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-rtos-wrap/src/MutexGuard.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-rtos-wrap/src/OsUtility.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-rtos-wrap/src/PeriodicTimer.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-rtos-wrap/src/SignalSemaphore.cpp"
    
    # General utilities
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-general/src/SoftwareVersion.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-general/src/TimestampedVariable.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-general/src/CrcCalculator.c"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-general/src/Utility.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-general/src/VariableAnomalyMonitor.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-general/src/VariableMonitor.cpp"
    
    # CAN utilities
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-canopen/src/CanOpenUtils.cpp"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-canopen/src/CanOpenMotorUtils.cpp"
)

# Combine all sources
set(HF_HAL_SRCS
    ${API_SOURCES}
    ${MANAGER_SOURCES}
    ${HANDLER_SOURCES}
    ${CORE_DRIVER_SOURCES}
    ${CORE_UTILS_SOURCES}
)

#==============================================================================
# 2. SYSTEMATIC INCLUDE DIRECTORY COLLECTION
#==============================================================================

# Library root include (resolves api/, managers/, handlers/, core/ prefixed includes)
set(LIB_ROOT_INCLUDE_DIR
    "${HF_HAL_ROOT}/lib"
)

# API include directories
set(API_INCLUDE_DIRS
    "${HF_HAL_ROOT}/lib/api"
)

# Manager include directories
set(MANAGER_INCLUDE_DIRS
    "${HF_HAL_ROOT}/lib/managers"
)

# Handler include directories (handlers in hf-core: lib/core/handlers)
set(HANDLER_INCLUDE_DIRS
    "${HF_HAL_ROOT}/lib/core"
)

# Core drivers include directories
set(CORE_DRIVER_INCLUDE_DIRS
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/inc"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/base"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/utils"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/internal/hf-pincfg/src"
)

# Core utils include directories
set(CORE_UTILS_INCLUDE_DIRS
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-rtos-wrap/include"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-general/include"
    "${HF_HAL_ROOT}/lib/core/hf-core-utils/hf-utils-canopen/include"
)

# External driver include directories
set(EXTERNAL_DRIVER_INCLUDE_DIRS
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/external/hf-as5047u-driver/src"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/external/hf-bno08x-driver/src"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/external/hf-ntc-thermistor-driver/inc"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/external/hf-pcal95555-driver/src"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/external/hf-tmc9660-driver/inc"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/external/hf-tmc9660-driver/inc/register_mode"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/external/hf-tmc9660-driver/inc/parameter_mode"
    "${HF_HAL_ROOT}/lib/core/hf-core-drivers/external/hf-ws2812-rmt-driver/inc"
)

# Combine all include directories
set(HF_HAL_INC_DIRS
    ${LIB_ROOT_INCLUDE_DIR}
    ${API_INCLUDE_DIRS}
    ${MANAGER_INCLUDE_DIRS}
    ${HANDLER_INCLUDE_DIRS}
    ${CORE_DRIVER_INCLUDE_DIRS}
    ${CORE_UTILS_INCLUDE_DIRS}
    ${EXTERNAL_DRIVER_INCLUDE_DIRS}
)

#==============================================================================
# 3. COMPONENT REGISTRATION
#==============================================================================

idf_component_register(
    SRCS          ${HF_HAL_SRCS}
    INCLUDE_DIRS  ${HF_HAL_INC_DIRS}
    
    # PRIV_REQUIRES: Minimal set of verified ESP-IDF 5.5 components
    PRIV_REQUIRES
        driver
        freertos
        nvs_flash
        esp_timer
        hal
        soc
        log
)

#==============================================================================
# 4. COMPILE OPTIONS AND DEFINITIONS
#==============================================================================

# Compile options for the entire HAL
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -Wall 
    -Wextra 
    -Wpedantic
    -Wno-error=undef
    -Werror=null-dereference
    -Werror=nonnull
    -Werror=nonnull-compare
    -Werror=address
    -Werror=aggressive-loop-optimizations
    -Werror=alloca
    -Werror=builtin-macro-redefined
    -Werror=div-by-zero
    -Werror=double-promotion
    -Werror=duplicated-branches
    -Werror=duplicated-cond
    -Werror=empty-body
    -Werror=enum-compare
    -Werror=enum-conversion
    -Werror=extra
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
    -Werror=missing-braces
    -Werror=missing-declarations
    -Werror=missing-include-dirs
    -Werror=missing-profile
    -Werror=multichar
    -Werror=multistatement-macros
    -Werror=narrowing
    -Werror=noexcept
    -Werror=non-template-friend
    -Werror=old-style-cast
    -Werror=openmp-simd
    -Werror=overloaded-virtual
    -Werror=packed
    -Werror=packed-bitfield-compat
    -Werror=padded
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
    -Werror=suggest-attribute=const
    -Werror=suggest-attribute=format
    -Werror=suggest-attribute=malloc
    -Werror=suggest-attribute=noreturn
    -Werror=suggest-attribute=pure
    -Werror=suggest-final-methods
    -Werror=suggest-final-types
    -Werror=suggest-override
    -Werror=switch-bool
    -Werror=tautological-compare
    -Werror=trigraphs
    -Werror=type-limits
    -Werror=undef
    -Werror=uninitialized
    -Werror=unknown-pragmas
    -Werror=unreachable-code
    -Werror=unsafe-loop-optimizations
    -Werror=unused
    -Werror=unused-but-set-parameter
    -Werror=unused-but-set-variable
    -Werror=unused-function
    -Werror=unused-label
    -Werror=unused-local-typedefs
    -Werror=unused-macros
    -Werror=unused-parameter
    -Werror=unused-result
    -Werror=unused-value
    -Werror=unused-variable
    -Werror=useless-cast
    -Werror=varargs
    -Werror=variadic-macros
    -Werror=vector-operation-performance
    -Werror=vla
    -Werror=volatile-register-var
    -Werror=write-strings
    -Werror=zero-as-null-pointer-constant
)

# Compile definitions
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    -DHARDFOC_HAL=1
    -DHARDFOC_VORTEX_V1=1
    -DHARDFOC_COMPONENT_HANDLER=1
    -DHARDFOC_GPIO_MANAGER=1
    -DHARDFOC_ADC_MANAGER=1
    -DHARDFOC_MOTOR_CONTROLLER=1
    -DHARDFOC_IMU_MANAGER=1
    -DHARDFOC_ENCODER_MANAGER=1
    -DHARDFOC_LED_MANAGER=1
    -DHARDFOC_TEMPERATURE_MANAGER=1
    -DHARDFOC_COMM_CHANNELS_MANAGER=1
    -DHARDFOC_PCAL95555_SUPPORT=1
    -DHARDFOC_TMC9660_SUPPORT=1
    -DHARDFOC_AS5047U_SUPPORT=1
    -DHARDFOC_BNO08X_SUPPORT=1
    -DHARDFOC_NTC_THERMISTOR_SUPPORT=1
    -DHARDFOC_WS2812_SUPPORT=1
    -DHARDFOC_PLATFORM_MAPPING=1
    -DHARDFOC_VORTEX_API=1
    -DHARDFOC_LOGGER=1
    -DHARDFOC_RTOS_WRAP=1
    -DHARDFOC_CORE_UTILS=1
    -DHARDFOC_CORE_DRIVERS=1
    -DHARDFOC_DRIVER_HANDLERS=1
)

# Set C++ standard
set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

#==============================================================================
# 5. BUILD INFORMATION
#==============================================================================

# Print build information
message(STATUS "=== HardFOC HAL Build Configuration ===")
message(STATUS "Root Directory: ${HF_HAL_ROOT}")
message(STATUS "API Sources: ${API_SOURCES}")
message(STATUS "Manager Sources: ${MANAGER_SOURCES}")
message(STATUS "Handler Sources: ${HANDLER_SOURCES}")
message(STATUS "Core Driver Sources: ${CORE_DRIVER_SOURCES}")
message(STATUS "Core Utils Sources: ${CORE_UTILS_SOURCES}")
message(STATUS "Total Sources: ${HF_HAL_SRCS}")
message(STATUS "Include Directories: ${HF_HAL_INC_DIRS}")
message(STATUS "Component: ${COMPONENT_LIB}")
message(STATUS "=== End Build Configuration ===")
