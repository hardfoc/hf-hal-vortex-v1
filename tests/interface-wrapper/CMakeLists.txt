# CMakeLists.txt for Communication Bus Tests
cmake_minimum_required(VERSION 3.16)

# Set the component name
set(COMPONENT_NAME "hf_comm_bus_tests")

# Define test sources
set(TEST_SOURCES
    CommBusTests.cpp
    test_runner.cpp
)

# Define test headers
set(TEST_HEADERS
    CommBusTests.h
)

# Create the test executable (if building standalone)
if(NOT DEFINED ENV{IDF_PATH})
    # Standalone build for unit testing
    project(hf_comm_bus_tests)
    
    # Set C++ standard
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    
    # Include directories
    include_directories(
        ../include
        ../include/base
        ../include/mcu
        ../include/drivers
        ../../lib/core/hf-core-utils/hf-utils-general/include
    )
    
    # Add the test executable
    add_executable(${COMPONENT_NAME}
        ${TEST_SOURCES}
    )
    
    # Link with pthread for threading tests
    find_package(Threads REQUIRED)
    target_link_libraries(${COMPONENT_NAME} PRIVATE Threads::Threads)
    
    # Add test runner as a CTest target
    enable_testing()
    add_test(NAME CommBusTests COMMAND ${COMPONENT_NAME})
    
else()
    # ESP-IDF component build
    idf_component_register(
        SRCS ${TEST_SOURCES}
        INCLUDE_DIRS "."
        REQUIRES 
            driver
            esp_common
            freertos
        PRIV_REQUIRES 
            hf-internal-interface-wrap
    )
    
    # Add compile definitions for ESP32
    target_compile_definitions(${COMPONENT_NAME} PRIVATE
        CONFIG_IDF_TARGET_ESP32=1
    )
endif()

# Compiler flags for tests
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${COMPONENT_NAME} PRIVATE
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Wno-missing-field-initializers
    )
endif()

# Add custom target for running tests
add_custom_target(run_comm_bus_tests
    COMMAND ${COMPONENT_NAME}
    DEPENDS ${COMPONENT_NAME}
    COMMENT "Running communication bus tests"
)

# Add targets for running specific test categories
add_custom_target(run_i2c_tests
    COMMAND ${COMPONENT_NAME} i2c
    DEPENDS ${COMPONENT_NAME}
    COMMENT "Running I2C tests only"
)

add_custom_target(run_spi_tests
    COMMAND ${COMPONENT_NAME} spi
    DEPENDS ${COMPONENT_NAME}
    COMMENT "Running SPI tests only"
)

add_custom_target(run_uart_tests
    COMMAND ${COMPONENT_NAME} uart
    DEPENDS ${COMPONENT_NAME}
    COMMENT "Running UART tests only"
)

add_custom_target(run_integration_tests
    COMMAND ${COMPONENT_NAME} integration
    DEPENDS ${COMPONENT_NAME}
    COMMENT "Running integration tests only"
)
