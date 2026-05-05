# HardFOC Vortex HAL — documentation index

Single-board HAL for **HardFOC Vortex V1** (ESP32-C6). The public entry point is the **`Vortex`** singleton in [`lib/api/Vortex.h`](lib/api/Vortex.h).

## Start here

| Document | Purpose |
|----------|---------|
| [README.md](README.md) | Product overview, quick start snippets |
| [`docs/hal/README.md`](docs/hal/README.md) | **Canonical** HAL doc hub — architecture, manager map, links to examples |
| [`docs/hal/architecture.md`](docs/hal/architecture.md) | Init order, degraded bring-up, diagnostics (aligned with `Vortex.cpp`) |
| [`docs/hal/managers-and-handlers.md`](docs/hal/managers-and-handlers.md) | Each `vortex.*` manager ↔ `hf-core` handlers and headers |
| [`examples/esp32/docs/README.md`](examples/esp32/docs/README.md) | **Every** on-target `APP_TYPE`, entry `.cpp`, CI flag |
| [`examples/esp32/docs/BENCH_MATRIX.md`](examples/esp32/docs/BENCH_MATRIX.md) | Bench apps, supply/motion notes |
| [`lib/api/README.md`](lib/api/README.md) | API integration notes |

## Shared handbook (submodule)

Coding standards, layered model, handler/manager patterns, CMake contract: [`docs/hf-development-handbook/README.md`](docs/hf-development-handbook/README.md).

## Published core docs

Handler-level detail and core test matrix: [hf-core GitHub Pages](https://hardfoc.github.io/hf-core/).

## Correct usage pattern

Prefer the façade and subsystem references — **not** copy-pasting older snippets that call `SomeManager::GetInstance()` without context:

```cpp
#include "api/Vortex.h"

extern "C" void app_main(void) {
    auto& vortex = Vortex::GetInstance();
    if (!vortex.EnsureInitialized()) {
        return;
    }
    auto& gpio = vortex.gpio;
    auto& motors = vortex.motors;
    (void)gpio;
    (void)motors;
}
```

`EnsureInitialized()` can succeed in **degraded** mode when optional subsystems (motors, ADC, IMU, encoders, temperature) fail; see [`docs/hal/architecture.md`](docs/hal/architecture.md).

## Former `docs/component-handlers/` / `docs/driver-handlers/`

Those folders no longer contain topic READMEs (they were removed as stale). Each folder keeps only a **README** that points to [`docs/hal/`](docs/hal/README.md).

## API generation

Doxygen/Jekyll configuration lives under [`_config/`](_config/). Generated output is separate from the markdown hubs above.
