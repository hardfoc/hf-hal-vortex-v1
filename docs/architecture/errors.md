# Errors and diagnostics

The HAL does not throw: the examples build with C++ exceptions off, every call is `noexcept` and failure comes back
in the return value. This page lists the error types, where to read the last error, and the system diagnostics.

## Rules of thumb

1. **Check the return value.** Most methods are `[[nodiscard]]`; the compiler warns if you drop a result.
2. A method that returns `bool` returns `false` on failure; a method that returns a pointer returns `nullptr` when
   the manager is not initialised or the thing you asked for does not exist.
3. Every manager has `GetLastError()`. Prefer the value a call returns: the last error is shared by every caller
   of that manager and may already describe a later call.

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
`EnsureInitialized()` has succeeded.

| Call | Gives you |
| --- | --- |
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

!!! note "Known gap"
    `GetSystemWarnings()` always returns no entries today: the facade has a warning list, but nothing adds to it.
    Best-effort start-up failures are visible in the log (`Initialized in degraded mode ...`) and through
    `GetFailedComponents()` / `GetComponentInitializationStatus()`.
