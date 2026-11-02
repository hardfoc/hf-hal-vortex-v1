# Vortex V1 HAL API reference

The C++ API of the HardFOC Vortex V1 hardware abstraction layer: the `Vortex` facade (`lib/api/Vortex.h`) and one
manager per board function (`lib/managers/`).

Start with the `Vortex` class: `Vortex::GetInstance()` returns the board object and `EnsureInitialized()` brings
every manager up in order. Each manager class page lists its methods and the error codes they return.

The guides, the pin map and the examples are on the documentation site:
<https://hardfoc.github.io/hf-hal-vortex-v1/latest/>
