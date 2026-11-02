# NvsManager

Persistent key-value storage in the ESP32-C6's flash (ESP-IDF NVS), split into three namespaces and exposed as the
board-independent `BaseNvs` interface. It starts first, so calibration and configuration can be read before the
buses come up. Source: [`NvsManager.h`](../../lib/managers/NvsManager.h).

## The namespaces

| Accessor | NVS namespace | Use it for |
| --- | --- | --- |
| `App()` | `vortex` | General application keys |
| `Calibration()` | `vortex_cal` | Calibration and factory tuning blobs |
| `Config()` | `vortex_cfg` | Structured device configuration |

Each returns a `BaseNvs&`. Code written against `BaseNvs` keeps working if the storage moves to another medium.

## API

| Call | Notes |
| --- | --- |
| `App()`, `Calibration()`, `Config()` | `BaseNvs&` for one namespace |
| On a `BaseNvs`: `SetU32` / `GetU32`, `SetString`, `SetBlob`, `GetSize`, `KeyExists`, `EraseKey`, `Commit` | Return `hf_nvs_err_t` (except `KeyExists`) |
| `GetLastError()` | Set when initialisation fails |
| `DumpStatistics()` | Logs each namespace |

## Example: count boots

```cpp
#include "api/Vortex.h"
#include "managers/NvsManager.h"

BaseNvs& app = Vortex::GetInstance().nvs.App();
hf_u32_t boots = 0;
(void)app.GetU32("boots", boots);           // stays 0 if the key does not exist yet
if (app.SetU32("boots", boots + 1) == hf_nvs_err_t::NVS_SUCCESS) {
    (void)app.Commit();
}
```

## Pitfalls

- **Commit.** Writes are not guaranteed to reach flash until `Commit()`.
- **Key length.** ESP-IDF limits keys to 15 characters (`GetMaxKeyLength()`).
- **No lock in the manager.** Initialise it once (the facade does); concurrent access to one namespace relies on
  ESP-IDF NVS itself.
