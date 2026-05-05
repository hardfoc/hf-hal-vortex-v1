/**
 * @file vortex_brake_dissipator_gpio.cpp
 * @brief GPIO-level exercise of TMC9660 chip GPIO lines (brake / dissipator paths per schematic).
 */
#include "api/Vortex.h"
#include "managers/GpioManager.h"

#include "core/hf-core-drivers/internal/hf-pincfg/src/hf_functional_pin_config_vortex_v1.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_brake_gpio";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "TMC9660 GPIO17/18 toggle smoke — no motion; verify loads disconnected or safe");

    if (!VORTEX_API.EnsureInitialized()) {
        ESP_LOGE(TAG, "Vortex init failed");
        return;
    }

    auto& gpio = VORTEX_API.gpio;
    auto t17 = gpio.Get(HfFunctionalGpioPin::TMC_GPIO17);
    auto t18 = gpio.Get(HfFunctionalGpioPin::TMC_GPIO18);
    if (!t17 || !t18) {
        ESP_LOGW(TAG, "TMC GPIO17/18 not registered — skip");
        return;
    }

    for (int i = 0; i < 3; ++i) {
        (void)t17->SetActive();
        (void)t18->SetInactive();
        vTaskDelay(pdMS_TO_TICKS(200));
        (void)t17->SetInactive();
        (void)t18->SetActive();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    (void)t17->SetInactive();
    (void)t18->SetInactive();

    ESP_LOGI(TAG, "TMC GPIO toggle sequence done");
}
