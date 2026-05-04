###############################################################################
#  vortex_project_setup.cmake
#
#  HAL-owned project-level setup. Must be `include()`d from the parent
#  application's root `CMakeLists.txt` BEFORE
#  `include($ENV{IDF_PATH}/tools/cmake/project.cmake)` because
#  `EXCLUDE_COMPONENTS` is consumed by IDF when it builds the component
#  graph during that include.
#
#  Vortex V1 (ESP32-C6) uses UART / CAN / USB Serial/JTAG-capable SoC
#  with no on-board Wi-Fi requirement for the reference HAL — same policy
#  as Flux: exclude the IDF Wi-Fi stack unless the product enables it.
###############################################################################

list(APPEND EXCLUDE_COMPONENTS
    esp_wifi
    wpa_supplicant
    wifi_provisioning
)

set(VORTEX_HAL_EXCLUDED_IDF_COMPONENTS
    esp_wifi
    wpa_supplicant
    wifi_provisioning
    CACHE INTERNAL "IDF components excluded by Vortex V1 HAL policy")
