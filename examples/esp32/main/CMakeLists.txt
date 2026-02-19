# =============================================================================
# HardFOC Vortex HAL — Main Component CMakeLists.txt
# =============================================================================
# Works with build_app.sh to select the active test/example source file.
#
# Supports ESP-IDF's two-phase build:
#   1. Requirements phase: uses defaults for component discovery
#   2. Build phase: enforces proper usage with helpful error messages
# =============================================================================

# =============================================================================
# Variable Validation and Defaults
# =============================================================================
if(NOT DEFINED APP_TYPE)
    if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        set(APP_TYPE "${DEFAULT_APP_TYPE}")
        message(STATUS "APP_TYPE not defined during requirements phase, using default: ${APP_TYPE}")
    else()
        message(FATAL_ERROR
            "APP_TYPE not defined. Please use build_app.sh to build this project.\n"
            "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} Release\n"
            "Use './scripts/build_app.sh list' to see available app types."
        )
    endif()
endif()

if(NOT DEFINED APP_SOURCE_FILE)
    if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        set(APP_SOURCE_FILE "${DEFAULT_SOURCE_FILE}")
        message(STATUS "APP_SOURCE_FILE not defined during requirements phase, using default: ${APP_SOURCE_FILE}")
    else()
        message(FATAL_ERROR
            "APP_SOURCE_FILE not defined. Please use build_app.sh to build this project.\n"
            "Example: ./scripts/build_app.sh ${DEFAULT_APP_TYPE} ${DEFAULT_BUILD_TYPE}\n"
            "Use './scripts/build_app.sh list' to see available app types."
        )
    endif()
endif()

# =============================================================================
# Status
# =============================================================================
message(STATUS "Building app type: ${APP_TYPE}")
message(STATUS "Using source file: ${APP_SOURCE_FILE}")

# =============================================================================
# Component Dependencies
# =============================================================================
set(MAIN_REQUIRES
    hf-hal-vortex-v1        # The HAL component (registered via EXTRA_COMPONENT_DIRS)
    esp_timer               # Required by TestFramework.h
    esp_driver_gpio         # Required by TestFramework.h (driver/gpio.h)
)

# =============================================================================
# Component Registration
# =============================================================================
idf_component_register(
    SRCS "${APP_SOURCE_FILE}"
    INCLUDE_DIRS "."
    REQUIRES ${MAIN_REQUIRES}
)

# =============================================================================
# Compiler Configuration
# =============================================================================
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)

if(BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall -Wextra -O0 -g3 -DHF_DEBUG=1
    )
else()
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall -Wextra -O2 -g -DNDEBUG
    )
endif()

# =============================================================================
# Example Type Compile Definitions
# =============================================================================
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    "EXAMPLE_TYPE_${APP_TYPE}=1"
)
