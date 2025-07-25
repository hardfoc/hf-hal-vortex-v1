###############################################################################
#  HardFOC HAL – **single-component** build script
#  Copy this file to hf-hal/CMakeLists.txt
###############################################################################

cmake_minimum_required(VERSION 3.16)

set(HF_HAL_ROOT ${CMAKE_CURRENT_LIST_DIR})

# Expose optional WS2812 driver so its headers are available when the HAL is
# consumed as a component. The driver lives under utils-and-drivers and is
# treated as an extra component to keep the HAL self-contained.
list(APPEND EXTRA_COMPONENT_DIRS "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/")
list(APPEND EXTRA_COMPONENT_DIRS "${HF_HAL_ROOT}/utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver")

#
# ─── 1 · Locate all implementation files (skip docs, examples, tests) ──────────
#
set(_SRC_GLOBS
    "${HF_HAL_ROOT}/*.c"  "${HF_HAL_ROOT}/*.cpp"  "${HF_HAL_ROOT}/*.cc"
    "${HF_HAL_ROOT}/*.S"  "${HF_HAL_ROOT}/**/*.c" "${HF_HAL_ROOT}/**/*.cpp"
    "${HF_HAL_ROOT}/**/*.cc" "${HF_HAL_ROOT}/**/*.S"
)
file(GLOB_RECURSE _ALL_SRCS ${_SRC_GLOBS})

set(_EXCLUDE_REGEX "/docs/|/Datasheet/|/datasheet/|/examples/|/tests/")

foreach(_f IN LISTS _ALL_SRCS)
    if(NOT _f MATCHES "${_EXCLUDE_REGEX}")
        list(APPEND HF_HAL_SRCS "${_f}")
    endif()
endforeach()

#
# ─── 2 · Collect every directory that owns a header we kept ───────────────────
#
file(GLOB_RECURSE _ALL_HDRS
    "${HF_HAL_ROOT}/*.h"    "${HF_HAL_ROOT}/*.hpp"
    "${HF_HAL_ROOT}/**/*.h" "${HF_HAL_ROOT}/**/*.hpp"
)

foreach(_h IN LISTS _ALL_HDRS)
    if(NOT _h MATCHES "${_EXCLUDE_REGEX}")
        get_filename_component(_dir "${_h}" DIRECTORY)
        file(RELATIVE_PATH _rel_dir "${HF_HAL_ROOT}" "${_dir}")
        list(APPEND HF_HAL_INC_DIRS "${_rel_dir}")
    endif()
endforeach()
list(REMOVE_DUPLICATES HF_HAL_INC_DIRS)

#
# ─── 3 · Register as ONE ESP-IDF component ────────────────────────────────────
#
idf_component_register(
    SRCS          ${HF_HAL_SRCS}
    INCLUDE_DIRS  ${HF_HAL_INC_DIRS}
    #
    # PUBLIC deps: anything the *application* must also link against.
    # (change to taste – these are the most common for a HAL).
    #
    REQUIRES      driver freertos hf-ws2812-rmt-driver nvs_flash
    # PRIVATE deps: used only while building this lib.
    #
    PRIV_REQUIRES hal esp_timer esp_adc esp_driver_spi esp_driver_i2c esp_driver_uart esp_driver_gpio 
)

#
# ─── 4 · Optional: tighten warnings for hf-hal only ───────────────────────────
#
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
