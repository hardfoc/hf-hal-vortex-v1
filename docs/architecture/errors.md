# Errors and diagnostics

The HAL does not throw: the examples build with C++ exceptions off, every call is `noexcept` and failure comes back
in the return value. This page lists the error types, where to read the last error, and the system diagnostics.

## Rules of thumb

1. **Check the return value.** Most methods are `[[nodiscard]]`; the compiler warns if you drop a result.
2. A method that returns `bool` returns `false` on failure; a method that returns a pointer returns `nullptr` when
   the manager is not initialised or the thing you asked for does not exist.
3. Every manager has `GetLastError()`. Prefer the value a call returns: the last error is shared by every caller
   of that manager and may already describe a later call.

## One catalog: `ManagerError`

Each manager keeps its own error type (table below), and the facade maps all of them onto one catalog,
[`ManagerError`](../../lib/api/VortexDiagnostics.h): `None`, `NotInitialized`, `DependencyNotReady`,
`BusUnavailable`, `DeviceNotFound`, `InitFailed`, `CommunicationFailed`, `InvalidParameter`, `HardwareFault`,
`Unknown`. `Vortex::GetLastError()` uses it to say why the last `EnsureInitialized()` failed, and
`CollectDiagnostics()` reports every manager in it, with the manager's own code alongside.

## Error types by manager

| Manager | Error type | To text |
| --- | --- | --- |
| `NvsManager` | `hf_nvs_err_t` (set on initialisation) | `HfNvsErrToString()` |
| `CommChannelsManager` | `CommError` | `CommErrorToString()` |
| `GpioManager` | `hf_gpio_err_t` | `HfGpioErrToString()` |
| `MotorController` | `MotorError` | `MotorErrorToString()` |
| `AdcManager` | `hf_adc_err_t` | `HfAdcErrToString()` |
| `ImuManager` | `ImuError` | `ImuErrorToString()` |
| `EncoderManager` | `EncoderError` | `EncoderErrorToString()` |
| `LedManager` | `LedError` | `LedErrorToString()` |
| `TemperatureManager` | `hf_temp_err_t` | `HfTempErrToString()` |

The board-specific enums (`CommError`, `MotorError`, `ImuError`, `EncoderError`, `LedError`) start with `SUCCESS`
(0), then `NOT_INITIALIZED`, then manager-specific values. The `hf_*_err_t` types come from the interface wrap and
are shared with every HardFOC board.

## System diagnostics

All of these are on the `Vortex` object. `CollectManagerHealth()` and `PerformHealthCheck()` return `false` until
`EnsureInitialized()` has succeeded; `CollectDiagnostics()` works at any time.

| Call | Gives you |
| --- | --- |
| `CollectDiagnostics(VortexBoardDiagnostics&)` | Everything in one snapshot, without a bus transfer or an allocation: per manager (in init order) whether it is required, up, and its `ManagerError`; the four buses; GPIO pin, motor, IMU, encoder, ADC and temperature counts; failed components, warnings, start-up time. Safe from a periodic health task |
| `GetLastError()` | `ManagerError` of the last failed `EnsureInitialized()` (`None` after a success) |
| `GetSystemDiagnostics(VortexSystemDiagnostics&)` | Which managers initialised, how many failed (and their names), start-up time, uptime; `system_healthy` |
| `GetComponentInitializationStatus()` | `std::array<bool, 9>`, one flag per manager |
| `GetFailedComponents(out, max)` | Names of managers whose initialisation failed |
| `CollectManagerHealth(ManagerHealthSnapshot&)` | Per manager: initialised, has an error, last error code (as `uint8_t`), and an overall `all_healthy` |
| `PerformHealthCheck()` | Logs the health snapshot and returns whether all managers are healthy (`false` before initialisation) |
| `DumpSystemStatistics()` | Logs every manager's statistics |
| `GetSystemUptimeMs()`, `GetInitializationTimeMs()`, `GetSystemVersion()` | Timing and the HAL version string |

The last error code in a health entry is the manager's own enum cast to `uint8_t`; read it with the table above.
Each manager also has `GetSystemDiagnostics(...)` and `DumpStatistics()`; see the manager pages.

```cpp
ManagerHealthSnapshot health;
if (Vortex::GetInstance().CollectManagerHealth(health) && !health.all_healthy) {
    for (size_t i = 0; i < health.count; ++i) {
        const auto& e = health.entries[i];
        if (!e.initialized || e.has_error) {
            ESP_LOGW("app", "%s: init=%d error=%u", e.name, e.initialized, e.last_error_code);
        }
    }
}
```

`GetSystemWarnings()` lists what `EnsureInitialized()` ran without: each best-effort manager that did not come up
(motors, ADC, IMU, encoders, temperature) and motor bring-up problems such as no TMC9660 registered. The texts have
static storage; the list is cleared at the start of each attempt and holds up to 12 entries.

```cpp
const char* warnings[12];
const size_t n = Vortex::GetInstance().GetSystemWarnings(warnings, 12);
for (size_t i = 0; i < n; ++i) {
    ESP_LOGW("app", "%s", warnings[i]);
}
```
