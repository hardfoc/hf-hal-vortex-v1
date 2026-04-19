/**
 * @file GpioManagerExample.cpp
 * @brief Documentation example: Multi-chip GPIO on Vortex V1.
 *
 * Demonstrates the GpioManager API for the Vortex motor-control board:
 *   - Pin lookup by name and by functional enum
 *   - Digital output: Set, SetActive, SetInactive, Toggle
 *   - Digital input: Read, IsActive
 *   - Pin configuration: direction, pull mode, output mode
 *   - Interrupt support
 *   - Multi-chip GPIO: ESP32 internal, PCAL95555 expander, TMC9660
 *   - Diagnostics dump
 *
 * @author HardFOC Team
 * @date 2025
 * @copyright HardFOC
 */

#include "api/Vortex.h"
#include "managers/GpioManager.h"

#include "esp_log.h"
#include "OsUtility.h"
static const char* TAG = "VortexGpioExample";

static void example_pin_lookup() {
    ESP_LOGI(TAG, "=== Pin Lookup ===");
    auto& gpio = VORTEX_API.gpio;
    ESP_LOGI(TAG, "Registered pins: %zu", gpio.Size());

    const char* pins[] = {"GPIO_TMC_FAULTN_0", "GPIO_ENCODER_CSN", "GPIO_USER_LED"};
    for (const char* name : pins) {
        ESP_LOGI(TAG, "  %-25s: %s", name, gpio.Contains(name) ? "found" : "NOT found");
    }
}

static void example_digital_io() {
    ESP_LOGI(TAG, "=== Digital I/O ===");
    auto& gpio = VORTEX_API.gpio;

    // Read fault pin
    bool fault = false;
    auto err = gpio.Read("GPIO_TMC_FAULTN_0", fault);
    if (err == hf_gpio_err_t::GPIO_SUCCESS) {
        ESP_LOGI(TAG, "TMC FAULT_N: %s", fault ? "OK (HIGH)" : "FAULT (LOW)");
    }

    // Toggle test pin
    gpio.Toggle("GPIO_USER_LED");
    os_delay_msec(100);
    gpio.Toggle("GPIO_USER_LED");
}

static void example_diagnostics() {
    ESP_LOGI(TAG, "=== GPIO Diagnostics ===");
    auto& gpio = VORTEX_API.gpio;

    GpioSystemDiagnostics diag{};
    if (gpio.GetSystemDiagnostics(diag) == hf_gpio_err_t::GPIO_SUCCESS) {
        ESP_LOGI(TAG, "Healthy: %s, Pins: %u, Ops: %u/%u/%u (total/ok/fail)",
                 diag.system_healthy ? "YES" : "NO",
                 diag.total_pins_registered,
                 diag.total_operations, diag.successful_operations, diag.failed_operations);
    }
    gpio.DumpStatistics();
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Vortex GpioManager Example");
    Vortex::GetInstance().EnsureInitialized();

    example_pin_lookup();
    os_delay_msec(100);
    example_digital_io();
    os_delay_msec(100);
    example_diagnostics();

    ESP_LOGI(TAG, "GpioManager example complete");
}
