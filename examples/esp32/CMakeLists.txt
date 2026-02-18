# =============================================================================
# HardFOC Vortex HAL — ESP32 Examples Project Root CMakeLists.txt
# =============================================================================
# Main project configuration for Vortex HAL API examples/tests.
# Integrates with the ESP-IDF build system via build_app.sh.
#
# Key Features:
# - Shared variable system for consistent defaults across components
# - Build parameter validation with helpful error messages
# - Integration with build_app.sh script for app type management
# - ESP-IDF project configuration and naming
# =============================================================================

cmake_minimum_required(VERSION 3.16)

# =============================================================================
# Shared Default Variables
# =============================================================================
set(DEFAULT_APP_TYPE "vortex_api_test" CACHE STRING "Default app type for requirements phase")
set(DEFAULT_BUILD_TYPE "Release" CACHE STRING "Default build type for requirements phase")
set(DEFAULT_SOURCE_FILE "vortex_api_test.cpp" CACHE STRING "Default source file for requirements phase")

# =============================================================================
# Build Parameter Validation
# =============================================================================
if(NOT DEFINED APP_TYPE)
    message(FATAL_ERROR
        "APP_TYPE not defined. Please use build_app.sh to build this project.\n"
        "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} ${DEFAULT_BUILD_TYPE}\n"
        "Use './scripts/build_app.sh list' to see available app types."
    )
endif()

if(NOT DEFINED BUILD_TYPE)
    message(FATAL_ERROR
        "BUILD_TYPE not defined. Please use build_app.sh to build this project.\n"
        "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} ${DEFAULT_BUILD_TYPE}\n"
        "Use './scripts/build_app.sh list' to see available app types."
    )
endif()

# =============================================================================
# Build Configuration
# =============================================================================
message(STATUS "Building app type: ${APP_TYPE}")
message(STATUS "Build type: ${BUILD_TYPE}")

set(APP_TYPE "${APP_TYPE}" CACHE STRING "App type to build" FORCE)
set(BUILD_TYPE "${BUILD_TYPE}" CACHE STRING "Build type (Debug/Release)" FORCE)

# =============================================================================
# ESP-IDF Integration
# =============================================================================
# The HAL component lives two directories up (../../) and is registered as an
# ESP-IDF component via the top-level CMakeLists.txt.
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# =============================================================================
# Project Configuration
# =============================================================================
project(esp32_vortex_${APP_TYPE}_app)
