# Vortex HAL — documentation hub

This folder is the **canonical** place for Vortex-specific architecture and navigation. It reflects how the tree is actually wired today: a **`Vortex`** façade over **board managers** in `lib/managers/`, which compose **handlers and drivers** from the **`hf-core`** submodule (`lib/core/`).

## Read next

| Document | What it covers |
|----------|----------------|
| [architecture.md](architecture.md) | Call stack from app → `Vortex` → managers → `hf-core` handlers, init order, degraded bring-up, health APIs |
| [managers-and-handlers.md](managers-and-handlers.md) | One table: each manager, its sources in this repo, and the handler/driver types it owns or uses |
| [../../examples/esp32/docs/README.md](../../examples/esp32/docs/README.md) | **Full** on-target example apps: every `APP_TYPE`, entry source file, CI flag, and links into `main/` |

## Cross-repo references

- **Shared engineering handbook** (standards, layered model, handler/manager patterns): [`../hf-development-handbook/README.md`](../hf-development-handbook/README.md), especially [`../hf-development-handbook/process/layered-architecture.md`](../hf-development-handbook/process/layered-architecture.md).
- **Published hf-core docs** (handlers, testing matrix in core): [hf-core GitHub Pages](https://hardfoc.github.io/hf-core/).
- **Top-level API and version macros**: [`../../lib/api/Vortex.h`](../../lib/api/Vortex.h), [`../../lib/api/README.md`](../../lib/api/README.md).

## Former `docs/component-handlers/` and `docs/driver-handlers/`

Those directories used to hold long per-topic READMEs; those files are **gone** to avoid stale duplicates. Each directory keeps a single **README** explaining the move. Prefer this `docs/hal/` tree and [`../../examples/esp32/docs/README.md`](../../examples/esp32/docs/README.md).
