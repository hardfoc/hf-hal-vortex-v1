/**
 * @file vortex_pcal_gpio_imu_bringup.cpp
 * @brief PCAL95555 + IMU (BNO08x) bring-up with full I²C diagnostics.
 *
 * Order of operations (each phase logs and continues even if previous fails):
 *   1. I²C scan via `CommChannelsManager::I2cScanBus()` (uses address-only probe).
 *   2. Direct probes for PCAL95555 @ 0x20 and BNO08x @ 0x4A.
 *   3. PCAL95555 raw pin sweep via `GpioManager::DebugReadPcal95555Pin(0..15)`.
 *   4. Registry read of IMU control nets (RST / BOOT / INT).
 *   5. ImuManager bring-up + accelerometer samples.
 *
 * SDA/SCL are not touched with `gpio_config` before init — that can fight the I²C
 * pad mux; use a scope or LA on GPIO21/22 if you need pre-boot line state.
 */
#include "api/Vortex.h"
#include "managers/CommChannelsManager.h"
#include "managers/GpioManager.h"
#include "managers/ImuManager.h"
#include "handlers/bno08x/Bno08xHandler.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <array>

static const char* TAG = "pcal_imu_bringup";

static void log_registry_pin(GpioManager& gpio, const char* name) noexcept {
    bool active = false;
    const hf_gpio_err_t e = gpio.IsActive(name, active);
    if (e == hf_gpio_err_t::GPIO_SUCCESS) {
        ESP_LOGI(TAG, "  registry %s: logical_active=%d", name, static_cast<int>(active));
    } else {
        ESP_LOGW(TAG, "  registry %s: not readable err=%d", name, static_cast<int>(e));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== PCAL95555 + BNO08x bring-up (I2C scan + GPIO + IMU) ===");

    if (!VORTEX_API.EnsureInitialized()) {
        ESP_LOGE(TAG, "Vortex init failed");
        return;
    }

    auto& comms = VORTEX_API.comms;
    auto& gpio  = VORTEX_API.gpio;

    //--- Phase 1: full bus scan -----------------------------------------------
    ESP_LOGI(TAG, "--- Phase 1: I2C bus scan (0x08..0x77, address-only probe) ---");
    std::array<uint8_t, 32> found{};
    const size_t hits = comms.I2cScanBus(found.data(), found.size());
    if (hits == 0) {
        ESP_LOGE(TAG, "I2C scan: NO devices ACKed.");
        ESP_LOGE(TAG,
                 "  Likely causes: no external pull-ups on SDA/SCL, slaves unpowered,"
                 " wrong SDA/SCL pins, or stuck bus. Aborting.");
        return;
    }
    ESP_LOGI(TAG, "I2C scan: %zu device(s) ACKed:", hits);
    for (size_t i = 0; i < hits; ++i) {
        ESP_LOGI(TAG, "    0x%02X", static_cast<unsigned>(found[i]));
    }

    //--- Phase 2: targeted probes for the two on-board addresses --------------
    const bool pcal_present = comms.I2cProbeAddress(0x20, /*timeout_ms=*/50);
    const bool bno_present  = comms.I2cProbeAddress(0x4A, /*timeout_ms=*/50);
    ESP_LOGI(TAG, "Targeted probes: PCAL95555 @ 0x20 = %s, BNO08x @ 0x4A = %s",
             pcal_present ? "ACK" : "NACK", bno_present ? "ACK" : "NACK");

    if (!pcal_present && !bno_present) {
        ESP_LOGE(TAG, "Neither PCAL95555 nor BNO08x ACKed — bus is up but on-board");
        ESP_LOGE(TAG, "chips are silent. Stop here and verify wiring / power.");
        comms.I2cResetBus();
        return;
    }

    //--- Phase 3: PCAL raw read sweep -----------------------------------------
    if (pcal_present) {
        ESP_LOGI(TAG, "--- Phase 3: PCAL95555 raw input read sweep (0..15) ---");
        for (uint8_t pin = 0; pin < 16; ++pin) {
            bool level = false;
            const hf_gpio_err_t err = gpio.DebugReadPcal95555Pin(pin, level);
            if (err == hf_gpio_err_t::GPIO_SUCCESS) {
                ESP_LOGI(TAG, "  P%-2u: ok level=%d", static_cast<unsigned>(pin),
                         static_cast<int>(level));
            } else {
                ESP_LOGW(TAG, "  P%-2u: FAIL err=%d", static_cast<unsigned>(pin),
                         static_cast<int>(err));
            }
        }
    } else {
        ESP_LOGW(TAG, "Skipping PCAL raw sweep (chip not present).");
    }

    //--- Phase 4: registry-side IMU nets (PCAL-routed) ------------------------
    ESP_LOGI(TAG, "--- Phase 4: PCAL nets for IMU (registry IsActive) ---");
    log_registry_pin(gpio, "GPIO_PCAL_IMU_RST");
    log_registry_pin(gpio, "GPIO_PCAL_IMU_BOOT");
    log_registry_pin(gpio, "GPIO_PCAL_IMU_INT");

    //--- Phase 5: BNO08x bring-up + accel sample ------------------------------
    if (!bno_present) {
        ESP_LOGW(TAG, "BNO08x not present on bus — skipping IMU phase.");
        return;
    }

    auto& imu = VORTEX_API.imu;
    if (!imu.EnsureInitialized()) {
        ESP_LOGW(TAG, "ImuManager init failed (BNO08x ACKs but handler errored).");
        imu.DumpStatistics();
        return;
    }

    ESP_LOGI(TAG, "ImuManager OK — sampling accelerometer (poll a few frames)");
    IBno08xDriverOps* sensor = imu.GetSensor(ImuManager::ONBOARD_IMU_INDEX);
    if (!sensor) {
        ESP_LOGW(TAG, "GetSensor(0) null");
        return;
    }

    for (int i = 0; i < 25; ++i) {
        sensor->Update();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (sensor->HasNewData(BNO085Sensor::Accelerometer)) {
        const SensorEvent ev = sensor->GetLatest(BNO085Sensor::Accelerometer);
        ESP_LOGI(TAG, "Accel: x=%.3f y=%.3f z=%.3f m/s2 accuracy=%u",
                 static_cast<double>(ev.vector.x), static_cast<double>(ev.vector.y),
                 static_cast<double>(ev.vector.z), static_cast<unsigned>(ev.vector.accuracy));
    } else {
        ESP_LOGW(TAG, "No accelerometer report yet (sensor enable / report rate path).");
    }

    imu.DumpStatistics();
    ESP_LOGI(TAG, "Bring-up done");
}
