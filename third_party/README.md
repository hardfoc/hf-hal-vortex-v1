# Third-party sources (HAL)

## CANopenNode

Submodule path: `third_party/CANopenNode` → [CANopenNode/CANopenNode](https://github.com/CANopenNode/CANopenNode).

Initialize from the **hf-hal-vortex-v1** repository root:

```bash
git submodule update --init --recursive third_party/CANopenNode
```

If that fails with **fatal: not a git repository** (broken `gitdir` under a parent checkout), repair the HAL submodule from the **Vortex** repo root (`git submodule update --init --recursive hal/hf-hal-vortex-v1`), then retry. As a **build-only** fallback you can clone into this folder:

```bash
git clone https://github.com/CANopenNode/CANopenNode.git third_party/CANopenNode
```

Then enable **HardFOC Vortex HAL → Build CANopenNode slave stack** in `idf.py menuconfig` (sets `CONFIG_HARDFOC_HAL_CANOPENNODE_SLAVE`).

The stack is compiled through `lib/core/cmake/hf_core_build_settings.cmake` (not a separate ESP-IDF component under the firmware project’s `components/`).

If you previously used **`CONFIG_HARDFOC_CANOPENNODE_SLAVE`** from the removed `components/canopennode` Kconfig, re-enable the option under **HardFOC Vortex HAL** and remove the old symbol from `sdkconfig` if CMake warns about unknown legacy options.
