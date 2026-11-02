# ===========================================================================
# hf-hal-vortex-v1 host build: the MCU-free part of the HAL, compiled for the
# development machine and tested against the hf-core host mocks.
#
#   cmake --preset host-debug        (from tests/host)
#   cmake --build --preset host-debug
#   ctest --preset host-debug
#
# hf-core is added as a subdirectory, so the chip handlers and drivers build
# with HF_CORE_MCU=NONE / HF_CORE_RTOS=NONE exactly as in hf-core's own host
# CI, and hf_core_host_mocks provides the scriptable Base* fakes.
#
# SPDX-License-Identifier: GPL-3.0-or-later
# ===========================================================================

cmake_minimum_required(VERSION 3.22)
project(hf_hal_vortex_host LANGUAGES C CXX)

if(CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "[hf-hal-vortex] tests/host is the host build; cross builds use the IDF component.")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(HF_HAL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." REALPATH)
if(NOT EXISTS "${HF_HAL_ROOT}/lib/core/CMakeLists.txt")
    message(FATAL_ERROR "lib/core is missing: git submodule update --init --recursive lib/core")
endif()

# hf-core host build (handlers, drivers, mocks, test harness); its own tests stay off.
set(HF_CORE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${HF_HAL_ROOT}/lib/core" hf_core)

set(HF_HAL_HOST_WARNINGS -Wall -Wextra -Werror)

# ── HAL sources that build on the host ─────────────────────────────────────
add_library(vortex_hal_host STATIC
    "${HF_HAL_ROOT}/lib/managers/EncoderManager.cpp"
    # Link seams for the managers whose sources still construct ESP32 peripherals
    # directly. They go away once those managers bind through bsp_bind/.
    seams/HostManagerSeams.cpp
)
# HAL sources include "managers/...", "api/..." and "core/...". "core/..." must resolve through
# hf-core's SYSTEM include root (submodule code is not held to this build's -Werror), so lib/ itself
# is not an include directory: a link tree exposes only lib/api and lib/managers.
set(_hal_inc_root "${CMAKE_CURRENT_BINARY_DIR}/hal_include_root")
foreach(_dir api managers)
    if(NOT EXISTS "${_hal_inc_root}/${_dir}")
        file(MAKE_DIRECTORY "${_hal_inc_root}")
        file(CREATE_LINK "${HF_HAL_ROOT}/lib/${_dir}" "${_hal_inc_root}/${_dir}" SYMBOLIC)
    endif()
endforeach()
target_include_directories(vortex_hal_host PUBLIC
    "${_hal_inc_root}"
    "${HF_HAL_ROOT}/lib/api"
    "${HF_HAL_ROOT}/lib/managers"
)
target_compile_definitions(vortex_hal_host PUBLIC HARDFOC_BOARD_VORTEX_V1=1)
target_link_libraries(vortex_hal_host PUBLIC hf_core)
target_compile_options(vortex_hal_host PRIVATE ${HF_HAL_HOST_WARNINGS})

# ── Tests: one executable and one ctest entry per file ─────────────────────
enable_testing()

function(vortex_host_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE vortex_hal_host hf_core_host_test_main hf_core_host_mocks)
    target_include_directories(${name} PRIVATE "${HF_HAL_ROOT}/examples/esp32/main/common")
    target_compile_options(${name} PRIVATE ${HF_HAL_HOST_WARNINGS})
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES TIMEOUT 60)
endfunction()

vortex_host_test(test_board_config   test_board_config.cpp)
vortex_host_test(test_boot_report    test_boot_report.cpp)
vortex_host_test(test_encoder_manager test_encoder_manager.cpp)
