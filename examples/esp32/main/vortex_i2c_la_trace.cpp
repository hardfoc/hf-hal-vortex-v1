/**
 * @file vortex_i2c_la_trace.cpp
 * @brief Minimal, slow I²C patterns for logic-analyzer correlation with UART.
 *
 * Flash this app, connect LA to SDA/SCL (see `vortex_board_pins.hpp`), reset the
 * board, and match UART lines to bus transactions. Long delays separate phases so
 * you can scroll the capture even at low sample depth.
 *
 * Build / flash:
 *   ./scripts/build_app.sh vortex_i2c_la_trace Release
 *   ./scripts/flash_app.sh flash_monitor vortex_i2c_la_trace Release
 */
#include "api/Vortex.h"
#include "managers/CommChannelsManager.h"

#include "vortex_board_pins.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdio>

static const char* TAG = "i2c_la_trace";

static void gap_ms(const char* label, uint32_t ms) {
    ESP_LOGW(TAG, "======== %s — idle %lu ms (no I2C) ========", label,
             static_cast<unsigned long>(ms));
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void trace_pcal_reg(BaseI2c* dev, const char* marker, uint8_t reg) {
    ESP_LOGI(TAG, "I2C_LA_MARKER %s — PCAL @0x20 WriteRead reg=0x%02X (tx 1, rx 1)", marker,
             static_cast<unsigned>(reg));
    vTaskDelay(pdMS_TO_TICKS(40));

    uint8_t rx = 0;
    const hf_i2c_err_t e =
        dev->WriteRead(&reg, 1, &rx, 1, /*timeout_ms*/ 200);
    ESP_LOGI(TAG, "I2C_LA_MARKER %s_END err=%d rx=0x%02X", marker, static_cast<int>(e),
             static_cast<unsigned>(rx));
}

extern "C" void app_main(void) {
    ESP_LOGW(TAG, "################################################################");
    ESP_LOGW(TAG, "# vortex_i2c_la_trace — correlate these UART lines with LA   #");
    ESP_LOGW(TAG, "# SDA GPIO %d  SCL GPIO %d  (Vortex pincfg)                    #",
             vortex_board_pins::kI2cSdaGpio, vortex_board_pins::kI2cSclGpio);
    ESP_LOGW(TAG, "# Expect: 0x20 = PCAL95555, 0x4A = BNO08x (if populated)     #");
    ESP_LOGW(TAG, "################################################################");

    gap_ms("PRE_INIT", 400);

    if (!VORTEX_API.EnsureInitialized()) {
        ESP_LOGE(TAG, "VORTEX_API.EnsureInitialized failed — still running trace if bus exists");
    } else {
        ESP_LOGI(TAG, "Vortex init returned OK (GPIO/IMU errors above may still appear)");
    }

    auto& comms = VORTEX_API.comms;
    BaseI2c* pcal = comms.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    BaseI2c* bno  = comms.GetI2cDevice(I2cDeviceId::BNO08X_IMU);

    gap_ms("POST_INIT_BEFORE_MANUAL_I2C", 500);

    for (int round = 0; round < 3; ++round) {
        char round_tag[24];
        std::snprintf(round_tag, sizeof(round_tag), "ROUND_%d", round);
        ESP_LOGW(TAG, "################ %s begin ################", round_tag);

        gap_ms("ROUND_LEADING_GAP", 350);

        ESP_LOGI(TAG, "I2C_LA_MARKER %s_PROBE_0x20 — address-only probe", round_tag);
        vTaskDelay(pdMS_TO_TICKS(30));
        const bool ok20 = comms.I2cProbeAddress(0x20, 150);
        ESP_LOGI(TAG, "I2C_LA_MARKER %s_PROBE_0x20_END present=%d", round_tag, static_cast<int>(ok20));

        vTaskDelay(pdMS_TO_TICKS(200));

        ESP_LOGI(TAG, "I2C_LA_MARKER %s_PROBE_0x4A — address-only probe", round_tag);
        vTaskDelay(pdMS_TO_TICKS(30));
        const bool ok4a = comms.I2cProbeAddress(0x4A, 150);
        ESP_LOGI(TAG, "I2C_LA_MARKER %s_PROBE_0x4A_END present=%d", round_tag, static_cast<int>(ok4a));

        vTaskDelay(pdMS_TO_TICKS(200));

        if (pcal) {
            if (!pcal->EnsureInitialized()) {
                ESP_LOGW(TAG, "PCAL BaseI2c::EnsureInitialized failed — trying WriteRead anyway");
            }
            trace_pcal_reg(pcal, "TRACE_PCAL_INPUT0", 0x00);
            vTaskDelay(pdMS_TO_TICKS(180));
            trace_pcal_reg(pcal, "TRACE_PCAL_INPUT1", 0x01);
            vTaskDelay(pdMS_TO_TICKS(180));
            trace_pcal_reg(pcal, "TRACE_PCAL_REG_4F", 0x4F);
        } else {
            ESP_LOGE(TAG, "GetI2cDevice(PCAL9555_GPIO_EXPANDER) null");
        }

        vTaskDelay(pdMS_TO_TICKS(200));

        if (bno) {
            ESP_LOGI(TAG, "I2C_LA_MARKER %s_BNO_WRITEREAD_2 — first byte 0x00 read 2 bytes (may NACK)",
                     round_tag);
            vTaskDelay(pdMS_TO_TICKS(40));
            uint8_t tx[1] = {0x00};
            uint8_t rx[2] = {0, 0};
            const hf_i2c_err_t eb =
                bno->WriteRead(tx, 1, rx, 2, 200);
            ESP_LOGI(TAG, "I2C_LA_MARKER %s_BNO_END err=%d rx=0x%02X%02X", round_tag,
                     static_cast<int>(eb), static_cast<unsigned>(rx[0]),
                     static_cast<unsigned>(rx[1]));
        } else {
            ESP_LOGE(TAG, "GetI2cDevice(BNO08X_IMU) null");
        }

        gap_ms("ROUND_TRAILING_GAP", 600);
    }

    ESP_LOGW(TAG, "################################################################");
    ESP_LOGW(TAG, "# Trace loop finished — capture complete. Post LA + UART.   #");
    ESP_LOGW(TAG, "################################################################");
}
