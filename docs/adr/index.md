# Decisions

Architecture decision records (ADRs): one page per decision that shapes the code, with its context, the options
considered and the consequences. A record is not rewritten after it is accepted; a later record supersedes it. The
Flux V1 HAL is built the same way and records the same three decisions.

| ADR | Title | Status |
| --- | --- | --- |
| [0001](0001-mcu-agnostic-boundary.md) | Keep the MCU behind the interface wrap; only managers know the board | Accepted |
| [0002](0002-static-allocation.md) | Allocate once at start-up; fixed tables sized from the pin map | Accepted |
| [0003](0003-pin-map-ownership.md) | The hf-pincfg X-macro table is the single source of pin truth | Accepted |

Start a new record from the [template](template.md).
