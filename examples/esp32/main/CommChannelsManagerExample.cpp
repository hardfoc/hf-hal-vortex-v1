/**
 * @file CommChannelsManagerExample.cpp
 * @brief Documentation example: Communication channels on Vortex V1.
 *
 * Demonstrates the CommChannelsManager API:
 *   - Channel enumeration
 *   - UART / SPI channel access
 *   - Transmit / Receive
 *   - Diagnostics
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "managers/CommChannelsManager.h"

#include "esp_log.h"
static const char* TAG = "VortexCommExample";

static void example_channel_info() {
    ESP_LOGI(TAG, "=== Registered Channels ===");
    auto& comms = VORTEX_API.comms;

    ESP_LOGI(TAG, "Initialized: %s", comms.IsInitialized() ? "YES" : "NO");
    comms.DumpStatistics();
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Vortex CommChannelsManager Example");
    Vortex::GetInstance().EnsureInitialized();

    example_channel_info();

    ESP_LOGI(TAG, "CommChannelsManager example complete");
}
