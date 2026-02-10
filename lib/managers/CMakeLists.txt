# Component Handler CMakeLists.txt
# This file builds the component handler library for the HardFOC system

cmake_minimum_required(VERSION 3.16)

# Set component name
set(COMPONENT_NAME "component-handler")

# Source files for the component handler
set(COMPONENT_SOURCES
    # Core system files
    SystemInit.cpp
    CommonIDs.h
    ThingsToString.h
    
    # Logger system (moved to driver-handlers)
    
    # GPIO management system
    GpioManager.cpp
    GpioManager.h
    
    # ADC management system
    AdcManager.cpp
    AdcManager.h
    
    # Temperature management system
    TemperatureManager.cpp
    TemperatureManager.h
    
    # TMC9660 motor controller system
    Tmc9660Handler.cpp
    Tmc9660Handler.h
    MotorController.cpp
    MotorController.h
    
    # AS5047U encoder manager system
    EncoderManager.cpp
    EncoderManager.h
    
    # Communication channels management
    CommChannelsManager.cpp
    CommChannelsManager.h
    
    # IMU management system
    ImuManager.cpp
    ImuManager.h
    
    # Example and test files
    ModernHalExample.cpp
    Tmc9660Example.cpp
    
    # Header files
)

# Create component library
idf_component_register(
    SRCS ${COMPONENT_SOURCES}
    INCLUDE_DIRS 
    "."
    "../utils-and-drivers/driver-handlers"
    "../utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc"
    "../utils-and-drivers/hf-core-drivers/internal/hf-pincfg/src"
    REQUIRES 
        driver
        esp_timer
        esp_system
        esp_common
        esp_rom
        esp_hw_support
        hal
        soc
        freertos
        log
        esp_adc
        esp_gpio
        esp_i2c
        esp_spi
        esp_can
        esp_uart
        esp_pm
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        esp_partition
        esp_ota
        esp_http_client
        esp_https_ota
        esp_websocket_client
        esp_http_server
        esp_netif_stack
        esp_wifi_stack
        esp_event_stack
        esp_system_stack
        esp_common_stack
        esp_rom_stack
        esp_hw_support_stack
        hal_stack
        soc_stack
        freertos_stack
        log_stack
        esp_adc_stack
        esp_gpio_stack
        esp_i2c_stack
        esp_spi_stack
        esp_can_stack
        esp_uart_stack
        esp_pm_stack
        esp_wifi_stack
        esp_netif_stack
        esp_event_stack
        nvs_flash_stack
        esp_partition_stack
        esp_ota_stack
        esp_http_client_stack
        esp_https_ota_stack
        esp_websocket_client_stack
        esp_http_server_stack
)

# Set component properties
set_component_properties(${COMPONENT_NAME} PROPERTIES
    COMPONENT_LIB "component-handler"
    COMPONENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Add compile definitions
target_compile_definitions(${COMPONENT_NAME} INTERFACE
    -DHARDFOC_COMPONENT_HANDLER=1
    -DHARDFOC_GPIO_MANAGER=1
    -DHARDFOC_PCAL95555_SUPPORT=1
    -DHARDFOC_TMC9660_SUPPORT=1
    -DHARDFOC_PLATFORM_MAPPING=1
)

# Add compile options
target_compile_options(${COMPONENT_NAME} INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    -Werror=return-type
    -Werror=unused-variable
    -Werror=unused-function
    -Werror=unused-parameter
    -Werror=missing-field-initializers
    -Werror=switch
    -Werror=switch-enum
    -Werror=switch-default
    -Werror=implicit-fallthrough
    -Werror=format
    -Werror=format-security
    -Werror=format-nonliteral
    -Werror=cast-align
    -Werror=cast-qual
    -Werror=conversion
    -Werror=sign-conversion
    -Werror=float-conversion
    -Werror=float-equal
    -Werror=shadow
    -Werror=undef
    -Werror=redundant-decls
    -Werror=inline
    -Werror=disabled-optimization
    -Werror=stack-protector
    -Werror=stack-usage=4096
    -Werror=frame-larger-than=4096
    -Werror=larger-than=8192
    -Werror=array-bounds
    -Werror=array-bounds=2
    -Werror=stringop-overflow
    -Werror=stringop-overread
    -Werror=stringop-truncation
    -Werror=restrict
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

# Add include directories
target_include_directories(${COMPONENT_NAME} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils-and-drivers/hf-core-drivers/internal/hf-internal-interface-wrap/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils-and-drivers/hf-core-drivers/internal/hf-pincfg/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils-and-drivers/hf-core-drivers/external/hf-pcal95555-driver/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc/register_mode
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils-and-drivers/hf-core-drivers/external/hf-tmc9660-driver/inc/parameter_mode
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils-and-drivers/hf-core-utils/hf-utils-general/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils-and-drivers/hf-core-utils/hf-utils-rtos-wrap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils-and-drivers/hf-core-utils/hf-utils-canopen/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../API
)

# Set C++ standard
set_target_properties(${COMPONENT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# Add test directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
    add_subdirectory(test)
endif()

# Print component information
message(STATUS "Component Handler: ${COMPONENT_NAME}")
message(STATUS "Component Sources: ${COMPONENT_SOURCES}")
message(STATUS "Component Include Dirs: ${CMAKE_CURRENT_SOURCE_DIR}")
