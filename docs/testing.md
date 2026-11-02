# Testing and CI

What runs on a pull request, what each check means, and how to run it on your machine first. All workflows are in
[`.github/workflows`](../.github/workflows); most call reusable workflows in
[hf-general-ci-tools](https://github.com/hardfoc/hf-general-ci-tools) or
[hf-espidf-ci-tools](https://github.com/hardfoc/hf-espidf-ci-tools).

## Workflows

| Workflow | File | Runs on | Checks |
| --- | --- | --- | --- |
| ESP32 Examples Build CI | `esp32-examples-build-ci.yml` | push, pull request | Builds every app in `app_config.yml` (all have `ci_enabled: true`), Debug and Release, with ESP-IDF release/v5.5 |
| Host Tests CI | `ci-host-tests.yml` | push, every pull request | Builds every HAL source above `bsp_bind/` on Linux with GCC and Clang against the host binding and runs `tests/host`; runs the HAL boundary lint (`tools/lint/lint_hal_boundary.sh`: no MCU header above `bsp_bind/`, no raw driver accessor in `lib/`, allowlist may not grow) |
| C++ Lint CI | `ci-cpp-lint.yml` | push, pull request | clang-format and clang-tidy with `_config/.clang-format` and `_config/.clang-tidy` |
| C++ Analysis CI | `ci-cpp-analysis.yml` | push, pull request | cppcheck on `lib/managers` and `lib/api` (C++20) |
| Docs Publish CI | `ci-docs-publish.yml` | push to main, `v*` tags, pull request | `hardfoc-docs check` (figures and generated tables current), strict MkDocs build, Doxygen under `api/`, offline link check of the built site, Lighthouse budget, phone and desktop screenshots (artifact `docs-screenshots`); publishes on main and tags |
| Docs Link Check CI | `ci-docs-linkcheck.yml` | push, pull request | lychee over the Markdown files, config `_config/lychee.toml` |
| Markdown Lint CI | `ci-markdown-lint.yml` | push, pull request | markdownlint, config `_config/.markdownlint.json` |
| YAML Lint CI | `ci-yaml-lint.yml` | push, pull request | yamllint, config `_config/.yamllint` |
| Release | `release.yml` | `v*` tags | Builds the release artifacts |

The examples are integration tests: they run on a board, not in CI. CI proves they compile; a person flashes them.

## Host unit tests

[`tests/host`](../tests/host) builds every source in `lib/api` and `lib/managers`, with hf-core added as a
subdirectory (`HF_CORE_MCU=NONE`, `HF_CORE_RTOS=NONE`) and the host board binding (`bsp_bind/host/`) in place of the
ESP32-C6 one. The binding hands the managers hf-core's scriptable `Base*` fakes (`hf_core_host_mocks`) and logs
what they asked for; a test reaches the fakes through `vortex::bsp::host` to put a chip model behind them or inject
a failure. One file is one executable and one ctest entry.

| Test | Proves |
| --- | --- |
| `test_board_config` | The board NTC config reads TMC9660 AIN3 (not AIN0) on a shared ADC; the examples' GPIO numbers equal the pin map |
| `test_boot_report` | What `Vortex::GetFailedComponents()` and `GetSystemWarnings()` return: no duplicates, bounded, overflow counted |
| `test_encoder_manager` | `EncoderManager` reads on an AS5047U register model: `ReadDiagnostics()` makes one ERRFL read, `ReadFieldStrength()` returns AGC and magnitude, empty slots touch no SPI |
| `test_fail_closed` | A required manager that fails (no bus, LED) shuts every manager down again and sets `GetLastError()`; a retry succeeds; `CollectDiagnostics()` after a degraded boot; GPIO pointers are stable and cleared on shutdown; steady-state calls make zero heap allocations |
| `test_lock_discipline` | `ImuManager::VisitSensor()` refuses an absent IMU; `MotorController::visitDriver()` returns a default and runs nothing without a device; SPI statistics need a ready SPI device |
| `test_motor_observability` | On the TMC9660 SPI driver over a TMCL slave model: batches are pipelined (N + 1 frames), chunked and report per-element status; telemetry snapshots alternate the error words, age and keep stale values; fault decoding; latch clearing; image apply writes `MOTOR_TYPE` first, skips equal entries and reports rejections and mismatches; `MotorController` refuses without a ready device and sends nothing |
| `test_board_bind` | `Vortex::EnsureInitialized()` on the host binding: bus pins, SPI modes, I2C devices, UART, CAN, LED pin and NVS namespaces as the pin map says; LED frames; `Shutdown()` closes what init opened |

```bash
cd tests/host
cmake --preset host-debug
cmake --build --preset host-debug
ctest --preset host-debug
```

The host RTOS layer has no scheduler, so its mutex is a no-op: lock *order* and lock *coverage* are reviewed, not
tested here.

## Run the checks locally

```bash
# Build one example (needs ESP-IDF, see Get started)
cd examples/esp32 && ./scripts/build_app.sh vortex_api_test Release && cd ../..

# Format check
clang-format --style=file:_config/.clang-format --dry-run --Werror lib/managers/*.cpp lib/managers/*.h

# Docs: drift checks, then the strict site build
python3 -m pip install "git+https://github.com/hardfoc/hf-general-ci-tools@v1#subdirectory=packages/hardfoc-mkdocs-theme"
hardfoc-docs gen --check
hardfoc-docs figures --check
mkdocs build --strict

# HAL boundary (no MCU header above bsp_bind/)
tools/lint/lint_hal_boundary.sh --base origin/main

# Markdown and YAML lint
npx markdownlint-cli2 --config _config/.markdownlint.json "docs/**/*.md" README.md
yamllint -c _config/.yamllint .github/workflows mkdocs.yml
```

## When the docs checks fail

| Message | Fix |
| --- | --- |
| `generated region is stale` | The pin map, `app_config.yml` or a manager's `#include`s changed. Run `hardfoc-docs gen` and commit |
| `... is stale (run hardfoc-docs figures)` | A figure's source data changed. Run `hardfoc-docs figures`, look at the SVGs, commit |
| `contains a link ... not found` | A page links to a file that does not exist. Fix the link |
| `image(s) have no alt text` | Every image needs `alt` text |
