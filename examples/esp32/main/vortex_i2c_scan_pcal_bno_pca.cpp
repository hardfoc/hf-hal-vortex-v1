/**
 * @file vortex_i2c_scan_pcal_bno_pca.cpp
 * @brief I2C bring-up: shared bus devices after HAL init — PCAL95555, BNO08x, optional PCA9685.
 *
 * I2C SDA/SCL GPIOs match hf-pincfg (`vortex_board_pins.hpp`).
 */
#include "api/Vortex.h"
#include "managers/CommChannelsManager.h"
#include "managers/GpioManager.h"
#include "managers/ImuManager.h"
#include "handlers/pca9685/Pca9685Handler.h"

#include "vortex_board_pins.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "vortex_i2c_scan";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "I2C / PCAL / BNO / PCA9685 bench (SDA=%d SCL=%d per pincfg)",
             vortex_board_pins::kI2cSdaGpio, vortex_board_pins::kI2cSclGpio);

    if (!VORTEX_API.EnsureInitialized()) {
        ESP_LOGE(TAG, "Vortex init failed");
        return;
    }

    auto& comms = VORTEX_API.comms;
    auto& gpio = VORTEX_API.gpio;
    auto& imu = VORTEX_API.imu;

    {
        bool high = false;
        if (gpio.Read("GPIO_PCAL_TMC_DRV_EN", high) == hf_gpio_err_t::GPIO_SUCCESS) {
            ESP_LOGI(TAG, "PCAL DRV_EN read (logical): %d", static_cast<int>(high));
        } else {
            ESP_LOGW(TAG, "PCAL DRV_EN read failed");
        }
    }

    if (!imu.EnsureInitialized()) {
        ESP_LOGW(TAG, "ImuManager init failed (check BNO08x / I2C)");
    } else {
        ESP_LOGI(TAG, "ImuManager OK (BNO08x on shared I2C @ 0x4A when populated)");
    }

    constexpr uint8_t kPcaAddr = 0x40;
    const int pca_idx = comms.CreateI2cDevice(kPcaAddr, 400000);
    if (pca_idx < 0) {
        ESP_LOGW(TAG, "Could not add runtime I2C device 0x%02X (missing or no slot)", kPcaAddr);
    } else {
        BaseI2c* pca_i2c = comms.GetI2cDevice(0, pca_idx);
        if (pca_i2c == nullptr) {
            ESP_LOGW(TAG, "GetI2cDevice(0, %d) returned null", pca_idx);
        } else {
            Pca9685Handler pca(*pca_i2c);
            if (pca.EnsureInitialized()) {
                ESP_LOGI(TAG, "PCA9685 @ 0x%02X: handler initialized", kPcaAddr);
            } else {
                ESP_LOGW(TAG, "PCA9685 @ 0x%02X: handler init failed (device absent?)", kPcaAddr);
            }
        }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "I2C bench app done");
}
