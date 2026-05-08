# BLDC bring-up over the TMC9660 UART transport

This is the running log of what it took to spin a 24 V / 30 W bench BLDC
through `FOC_OPENLOOP_CURRENT_MODE` from the Vortex V1 carrier board over the
UART link to the on-board TMC9660. The matching application is
[`vortex_bldc_open_loop_uart.cpp`](../main/vortex_bldc_open_loop_uart.cpp);
this document captures the *why* behind the non-obvious bits so that the
next person bringing up a new motor or board does not have to re-discover
the chip's quirks.

> **Companion docs**
> * [`TMCL_PARAM_SCAN_FAILURES_UART.md`](TMCL_PARAM_SCAN_FAILURES_UART.md) —
>   exhaustive UART-side `GAP` baseline (which parameters the chip will
>   accept, which are `WRONG_TYPE`, etc.).
> * [`TMCL_PARAM_SCAN_FAILURES.md`](TMCL_PARAM_SCAN_FAILURES.md) — same
>   baseline on the SPI bus, before the UART comm-layer fixes.
> * [Parameter Mode Reference Manual](../../../lib/core/hf-core-drivers/external/hf-tmc9660-driver/docs/datasheet/tmc9660-parameter-mode-reference-manual.pdf)
>   — the ADI manual we are following.

## TL;DR — the working sequence

1. Force `VortexOnboardTmc9660Transport::Uart`, then `VORTEX_API.EnsureInitialized()`.
2. `configure_complete_bldc(..., with_oc_vgs_protection=false,
   with_gate_current_limits=false, skip_y2_phase=true)`.
3. **`disable_vortex_uvw_vgs_short_protection()`** in
   `vortex_motor_bench_common.hpp` (SAPs UVW VGS-short enables off, max UVW
   VGS deglitch, then `clear_fault_flags()`). Without this the chip silently
   reverts `COMMUTATION_MODE` to `SYSTEM_OFF` after fault retries.
4. `clear_fault_flags()` to drop any latched events from the gate-driver
   programming step.
5. `configure_ramp_for_open_loop_voltage()` — `RAMP_ENABLE=1`,
   `DIRECT_VELOCITY_MODE=0`, `RAMP_VMAX = 2_000_000` so the hardware ramper
   is what advances `phi_e`.
6. `setOpenloopCurrent(kSafeOpenLoopCurrentMa)` **before** the mode change
   (must be ≤ `kMaxFluxCurrentMa` in `vortex_bench_safety.hpp`; SPI app uses
   `kOpenLoopCurrentMa`).
7. `setCommutationMode(FOC_OPENLOOP_CURRENT_MODE)`. Read back
   `COMMUTATION_MODE` and retry once after another fault clear if it has
   reverted.
8. Slow-ramp `TARGET_VELOCITY` from `kMinRampVelocity` (1 000) to
   `kCruiseVelocity` (see `vortex_bldc_open_loop_uart.cpp`; lower cruise
   helps open-loop pull-in) in 8 steps × 200 ms.
9. Hold cruise for 2 s, ramp back to 0, `motor_stop_safe()`.

Telemetry per step prints the supply voltage, chip temperature,
`vel`/`pos`, motor current, `Iq`, **open-loop electrical angle `phi_e`**,
the three phase currents `Iuvw` and the inverter voltage commands `Uuvw` /
`Uq` so the operator can see the ramper rotating the field.

A successful run looks like this (snippet from a real bench capture):

```
[ramp1@1000] Vbus=24.10V T_chip=24.0°C vel=1000 pos=517 I_motor=134 mA Iq=-2 mA
[ramp1@1000] phi_e=  609  Iuvw=( 58,-82, 28) Iq=-21  Uuvw=( 129,-20,-110) Uq= 43
...
[ramp8@5000] phi_e=19149  Iuvw=(-22, 14,-20) Iq= 23  Uuvw=(  -6,119,-114) Uq=-43
CRUISE at TARGET_VELOCITY=5000 for 2000 ms
[cruise@0]   phi_e=23591  Iuvw=(-22, 14,-60) Iq=-43  Uuvw=( -40,120, -73) Uq=-46
[cruise@600] phi_e=-28760 Iuvw=(-22,  6, 28) Iq=  9  Uuvw=( -61, 33,  29) Uq=-26
...
[post-stop] gen_status=0x382D8E01 gen_err=0x00000021 gd_err=0x00000000 adc_status=0x00000000
Open-loop UART profile finished OK
```

`phi_e` wrapping from positive into negative (around `±32767`) and
`Iuvw`/`Uuvw` rotating phase-by-phase confirms the inverter is genuinely
modulating and advancing the rotor flux.

## Oscilloscope vs Saleae — which tool for what

**Use the oscilloscope on the motor (this is the “perfect” check for your
symptom).** A Saleae (logic analyzer) is the wrong primary tool for the power
stage: its inputs are **low-voltage logic** (typically ≤ 5 V). **Do not**
clip a Saleae probe to inverter phase nodes at **24 V PWM** without a
proper high-voltage / attenuating front-end; you risk destroying the analyzer
and creating unsafe ground paths.

| Goal | Tool | What you should see |
| ---- | ---- | ------------------- |
| Confirm **U / V / W** are really switching at the **motor connector** | **Scope** (10× probe, short ground lead) | Three phase nodes show **PWM** (not DC). While the open-loop profile runs, **all three** legs move; average line-line voltage and pulse pattern **rotate** over tens of ms. **Bad:** one phase stuck at ~0 V or ~24 V flat while the others switch → **open wire, wrong pad, or dead leg**. |
| Same, with less ground-loop risk | **Scope, two channels math** | e.g. CH1 = phase A, CH2 = phase B, **MATH = A − B**. You want a **living** differential waveform, not a flatline. |
| Verify **TMCL / UART** bytes and timing | **Saleae** on **ESP32 ↔ TMC9660 UART** (3.3 V TX/RX only) | Decoded frames, `REPLY_OK`, ramping traffic — confirms **software** side only. |

**Grounding (read once):** If the bench supply is **floating** and you
connect the scope probe **ground** to a phase that is not the same node as
the supply return your probe expects, you can **short the bridge through the
scope earth**. Safest patterns: **battery-powered / isolated** scope,
**differential** probes on line-line, or scope **GND only** to the **DC−**
that the inverter already shares with your measurement reference — and
know your supply topology.

**If scope shows three healthy PWM legs but the rotor still does not
track:** open-loop is **slipping** — try **lower** `kCruiseVelocity` first
(slower `phi_e`), then **raise** open-loop current (staying ≤
`kMaxFluxCurrentMa` and ≤ `kMaxPhaseCurrentMa`). Also confirm **pole pair
count** matches *your* motor (`kDefaultPolePairs`); wrong pairs make the
forced electrical angle advance at the wrong rate vs the magnet rotor.

## Why this took several iterations

### 1. UART comm layer hardening (resolved before this doc)

The chip was silent on UART because of a stack of bugs unrelated to the
TMC9660 itself:

* `EspUart::Read()` returned success on a 0-byte read, which made the
  Trinamic-CRC check pass on garbage. Replaced with a `ReadExactBytesUart`
  helper that loops until the requested byte count is in or the timeout
  fires.
* `EspUart` did not flush the RX FIFO before each transaction, so stale
  bytes from a previous boot/reset poisoned the very first reply.
* `Tmc9660Handler` had a double-inversion bug on the active-low PCAL95555
  control pins (`WAKE_CTRL`, `SPI_COMM_EN`, `RST_CTRL`), so the chip was
  effectively held in reset / asleep regardless of what the high-level code
  asked for.
* The TMC9660 UART bus and the ESP-IDF console were both bound to UART0,
  fighting over the same TX line. Moved the chip onto UART1 in
  `CommChannelsManager.cpp`.

### 2. `TMCLFrame::toUart` packing bug (resolved before this doc)

`TMCLFrame::toUart` was dropping the upper 4 bits of the 12-bit `type`
field, so any parameter NR ≥ 256 aliased to a low NR (e.g. NR 258 became
NR 2). That made the gate-driver per-side configuration parameters (`UVW
LOW SIDE THRESHOLD` 258, `UVW HIGH SIDE THRESHOLD` 261, etc.) appear to be
"`WRONG_TYPE`" on the chip. Fixed by packing `type[11:8]` into the high
nibble of `out[3]` and `motor[3:0]` into the low nibble — exactly per the
Trinamic reference packing.

After this fix, the param-scan numbers dropped from
≈103 `WRONG_TYPE` failures to a small handful of genuinely-not-implemented
parameters.

### 3. Chip-side `INVALID_VALUE` rejections

Several parameters the high-level helpers tried to set were rejected by the
chip:

| Parameter (NR)                              | Rejected value | Fix |
| ------------------------------------------- | -------------- | --- |
| `UVW_SINK_CURRENT` (245), `UVW_SOURCE_CURRENT` (246) | enums for our Rds(on)/Qg | Don't program — `with_gate_current_limits = false` in the gate-driver helper. |
| `THERMAL_WINDING_TIME_CONSTANT_1` (224)     | 100 ms         | Bumped `kI2tWindow1Ms` to 2 000 ms (chip floor ≈ 1 s). |
| `THERMAL_WINDING_TIME_CONSTANT_2` (225)     | 1 000 ms       | Bumped `kI2tWindow2Ms` to 5 000 ms. |
| `TARGET_VELOCITY` (124), all values         | n/a — all rejected | Root cause was the **chip silently reverting `COMMUTATION_MODE` back to `SYSTEM_OFF`**; see below. |

### 4. The big one — `COMMUTATION_MODE` silently reverting to `SYSTEM_OFF`

After every successful `SAP COMMUTATION_MODE = 4` (`REPLY_OK`), reading
`COMMUTATION_MODE` back returned `0`. Without an active regulating mode the
chip rejects every `TARGET_VELOCITY` write with `REPLY_INVALID_VALUE`, which
looked superficially like a bad value but was really "no mode to apply this
to".

The fault aggregates told the story:

* `gen_err = 0x00C00021` → `CONFIG_ERROR | HALL_ERROR | ADC_IN_OVERVOLTAGE
  | FAULT_RETRY_HAPPENED | FAULT_RETRIES_FAILED`.
* `gd_err  = 0x07000700` → `U/V/W_LOW_SIDE_CHARGE_SHORT |
  U/V/W_HIGH_SIDE_CHARGE_SHORT`.

Per the Parameter Mode manual section *Fault Handling*: any gate-driver
fault triggers the retry mechanism (default 5 retries via
`FAULT_HANDLER_NUMBER_OF_RETRIES`); when all retries fail the chip applies
`DRIVE_FAULT_BEHAVIOUR` (default `OPEN_CIRCUIT`) which means the system is
forced off — i.e. `COMMUTATION_MODE → SYSTEM_OFF`.

The `*_CHARGE_SHORT` flags are not real shorts; they are the chip's
**VGS-short protection** detecting that the gate-source voltage during a
charge transition does not match what the chip expects. The chip's defaults
for these protections are tuned for the TMC9660-3PH-EVKIT FETs; on the
Vortex power stage they trip on every transition.

Fix: actively **disable** `VGS_SHORT_ON_PROTECTION_UVW_LOW_SIDE_ENABLE`
(NR 272), `VGS_SHORT_OFF_PROTECTION_UVW_LOW_SIDE_ENABLE` (NR 273),
`VGS_SHORT_ON_PROTECTION_UVW_HIGH_SIDE_ENABLE` (NR 274), and
`VGS_SHORT_OFF_PROTECTION_UVW_HIGH_SIDE_ENABLE` (NR 275). For belt-and-
braces also push `VGS_SHORT_PROTECTION_UVW_DEGLITCH` (NR 282) to the
maximum 8 µs setting (value `7`). The current sense-based overcurrent
protection and the I²t monitors stay armed; only the **VGS-short**
protection on UVW is suppressed.

> 🛠 **Follow-up work**: the right long-term fix is to **tune** the VGS-
> short blanking and deglitch (NR 280-282) for the actual Vortex FETs
> rather than disabling the protection entirely. That requires
> characterising the FET turn-on / turn-off transients on a scope.

### 5. Fault-flag mask in the safety stop

Even after a successful spin, a few flags stay set in `gen_err` and would
trip a naïve "any non-zero flag = abort" check:

| Bit | Flag | Why we ignore it |
| --- | ---- | ---------------- |
| 0   | `CONFIG_ERROR` | Read-only, indicates no NVS config has been verified. |
| 1   | `TMCL_SCRIPT_ERROR` | Read-only, no TMCL script loaded. |
| 2   | `HOMESWITCH_NOT_FOUND` | Read-only, we don't run homing. |
| 5   | `HALL_ERROR` | Open-loop modes still poke the hall pipeline; we have no halls. |
| 18  | `EXT_TEMP_WARNING` | Warning, not exceedance. |
| 19  | `SUPPLY_OVERVOLTAGE_WARNING` | Warning, only `_EXCEEDED` should kill us. |
| 20  | `SUPPLY_UNDERVOLTAGE_WARNING` | Same. |
| 21  | `ADC_IN_OVERVOLTAGE` | Unused AINx noise. |
| 24  | `CHIP_TEMP_WARNING` | Warning only. |

Anything else in `gen_err` or any bit in `gd_err` triggers the emergency
stop (`emergency_stop()` in the app), which logs the latched aggregates and
calls `motor_stop_safe()`.

## Was the SPI failure GPIO-side or chip-side?

The original SPI complaint was "BLDC enable failed; gate driver was not
switching". Now that UART works end-to-end and we have a clean baseline
both for `GAP` parameter access (`TMCL_PARAM_SCAN_FAILURES_UART.md`) and
for full motor spin, we can reason more confidently about the SPI side:

* The PCAL95555 control-line setup is **shared** between SPI and UART
  (`Tmc9660Handler::setGpioFromSignal` was double-inverting active-low pins
  for both). The fix is in the host-side helper, so SPI benefits from it
  too — this rules out a "GPIO never enabled the chip on SPI" excuse.
* **`TMCLFrame::toSpi` used the wrong nibble order** for the merged
  `type[11:8]` + `motor` byte (motor was in the high nibble; the chip expects
  the same layout as UART / `TMC9660.c::sendRequestUART`). That made **every
  axis parameter with NR ≥ 256** return `REPLY_WRONG_TYPE` on SPI while UART
  succeeded (72 extra GAP failures in a Table 57 sweep on Vortex hardware, 2026-05).
  `toSpi` / `fromSpi` now match `toUart` for that field. The older `toUart` bug
  (dropping `type[11:8]` entirely) was UART-only and already fixed separately.
* The VGS-short trip is **chip-side**, not bus-side. It will fire on SPI
  too as soon as the chip programs gate transitions. The SPI bring-up
  app needs the same `SAP NR272..275 <- 0` + `NR282 <- 7` patch.
* The fault-handler revert chain (`FAULT_RETRIES_FAILED → SYSTEM_OFF`) is
  also bus-independent.

So the SPI bring-up will need:

1. The same VGS-short protection disable on UVW.
2. The same fault-flag mask in any safety-stop loop.
3. The `clear_fault_flags()` immediately before mode change.
4. `OPENLOOP_CURRENT` set **before** `COMMUTATION_MODE`.
5. The same `RAMP_ENABLE=1, DIRECT_VELOCITY_MODE=0` ramper config for
   open-loop modes.

### Confirmed: SPI parity (2026-05 bench run)

`vortex_bldc_open_loop` (forces `SetOnboardTmc9660Transport(Spi)` before
`VORTEX_API.EnsureInitialized()`) now spins the bench motor with the exact
same workaround stack: `with_oc_vgs_protection=false`,
`with_gate_current_limits=false`, `skip_y2_phase=true`,
`disable_vortex_uvw_vgs_short_protection()`, `clear_fault_flags()`,
`configure_ramp_for_open_loop_voltage()`, `setOpenloopCurrent(100 mA)`,
`setCommutationMode(FOC_OPENLOOP_CURRENT_MODE)`, then ramp.

`vortex_bldc_telemetry_sweep` (UART transport via the new Vortex default)
also passes end-to-end now that `configure_complete_bldc()` defaults to
`with_oc_vgs_protection=false`, `with_gate_current_limits=false`,
`skip_y2_phase=true` — `gateDriver.configurePowerStageProtection` no longer
gets fed the EvKit-only Y2 / VGS-short SAPs that the Vortex 3-phase board
firmware (FW051V100) rejects with `REPLY_INVALID_CMD` /
`REPLY_INVALID_VALUE`.

So the answer is: **the SPI failure was chip-side**, not a bus-side packing
regression. The historical SPI nibble bug (`TMCLFrame::toSpi` swap on
`type[11:8] | motor`) is fixed and verified by the round-trip raw frame
log (compile-time `TMC9660_LOG_TMCL_RAW_FRAMES`). The remaining failure was
the bench helper trying to program EvKit-class gate-driver protection that
the Vortex firmware does not accept.

## Gate-current SAP rejection on FW051V100 (NR 245-248)

Bench evidence (`diagnose_uvw_gate_current_tmcl()` in
`vortex_motor_bench_common.hpp`, plus raw TMCL TX/RX frames captured with
`-DTMC9660_LOG_TMCL_RAW_FRAMES=1`) — chip reports firmware version
`0051V100`:

* GAP NR 245/246/247/248 read back the datasheet defaults 4/4/4/4 cleanly
  (status `REPLY_OK`, value byte echoed in the reply).
* SAP NR 245/246 with **any** plain enum 0..15 → `REPLY_INVALID_VALUE`
  (status byte 2 = `0x04`). This includes the chip's own datasheet default
  (`4`) and the value GAP just returned (write-back-echo).
* The SAP is rejected before reaching the gate driver hardware — the
  control test (SAP NR 241 `DRIVE_TIME_SINK_UVW = 100`) succeeds in the
  same script and GAP confirms the new value, so the framing
  (`type[11:8] | motor` nibbles, big-endian 32-bit value field, checksum)
  is correct.
* Probing the value field with `(value & 0xF0) != 0` (e.g. `0x40`,
  `0x44`, `0x111 * E`) flips the reply to `REPLY_OK`, but GAP read-back
  is **not** a reliable indicator of what the chip stores: across runs the
  same `0x444` write produced GAP=4 once and GAP=0 the next time,
  depending on the prior SAP sequence. Treat any "REPLY_OK" on these
  parameters as a no-op until proven otherwise.

**Workaround (current state):** leave NR 245-248 at the silicon defaults.
The defaults map to ~270 mA sink / ~135 mA source, which is below the
~430 mA / ~290 mA the auto-config would have picked for the
58 nC bench FET (slower edges, marginally higher switching loss), but
**both `vortex_bldc_open_loop` (SPI) and `vortex_bldc_open_loop_uart`
spin the bench motor cleanly with these defaults — `gd_err=0x00000000`
through ramp-up, cruise and ramp-down**. So the rejection is a tuning
concern, not a "motor will not move" blocker.

Action items if/when this becomes a real problem:

* Re-run `diagnose_uvw_gate_current_tmcl(d, TAG, "pre/post")` after
  setting `kBenchPreTmclGateCurrentDiag = true` in
  `vortex_bldc_telemetry_sweep.cpp`. The probe sweeps 0..15 plain, then
  `(value & 0xF0)`-guarded patterns, then bit-replicated `0x111 * E`,
  always with raw frame logging.
* Cross-check against an EvKit running the same firmware revision
  (`0051V100`) to confirm the chip-side behaviour is independent of board
  routing.
* If silicon Y2 brake / Y2 brake-chopper boot config (`mech_brake.enable`
  + `brake_chopper.enable`, both pinned to Y2_LS / Y2_HS in
  `Tmc9660Handler::kDefaultBootConfig` to mirror the EvKit) turns out to
  be the gating condition, the workaround is to disable both at boot for
  the Vortex 3-phase variant — but bench evidence so far does **not**
  blame this (EvKit BLDC test ships with the same boot config).

**EvKit ordering does not change the rule** (verified 2026-05). The EvKit
runs `configureCompleteBLDCMotor` with the gate-driver SAPs **before**
setting `MOTOR_TYPE = BLDC` (motor type stays at `NO_MOTOR = 0` during
gate driver config). Vortex's bring-up sets `MOTOR_TYPE = BLDC` first,
then the gate driver. We tested the EvKit ordering on Vortex hardware via
`probe_gate_current_with_no_motor_type()` (toggleable in
`vortex_bldc_telemetry_sweep.cpp` via `kBenchProbeGateCurrentNoMotor`):
even with `MOTOR_TYPE = NO_MOTOR`, SAP NR 245 = 4 returns
`REPLY_INVALID_VALUE`. Motor type is **not** the gating condition —
this is purely a chip-side firmware rule on FW051V100.

## Closed-loop FOC bring-up (Hall, ABN)

Same VGS-short / fault-clear workaround applies to every commutation
mode. `vortex_bldc_velocity_foc_hall` and `vortex_bldc_velocity_foc_abn`
now mirror the open-loop pattern:

1. `configure_complete_bldc(...)` (defaults to
   `with_oc_vgs_protection=false`, `with_gate_current_limits=false`,
   `skip_y2_phase=true`).
2. `configure_hall(...)` / `configure_abn(...)` to program the chip's
   feedback path.
3. **`disable_vortex_uvw_vgs_short_protection(...)`** (NR 272-275 → 0,
   NR 282 → 7). Same SAP block the open-loop apps use.
4. `clear_fault_flags(...)` immediately before mode change.
5. `setCommutationMode(FOC_HALL_SENSOR | FOC_ABN)`, settle, then
   read-back `COMMUTATION_MODE`. If reverted, log fault flags, clear,
   retry once. Bail out (no `setTargetVelocity`) if the second attempt
   also reverts.

Bench result on the Vortex 3-phase board (2026-05): both modes **arm
cleanly** (`gd_err = 0x00000000`, no `COMMUTATION_MODE` revert,
`FAULT_RETRIES_FAILED` not raised). The shaft does **not** rotate in
either mode because the bench does not have:

* Hall sensors wired to TMC9660 GPIO2/3/4 (the EvKit map). Position
  reads as a static `pos = 10922` in the Hall log.
* ABN encoder wired to TMC9660 GPIO8/13/14. Position reads as
  `pos = 0` in the ABN log.

So closed-loop FOC is a **hardware enablement** task on the Vortex
carrier, not a firmware/driver issue. Once those sensors are wired, the
arm-and-spin code path is in place. For a quick spin without sensors,
use the open-loop apps (current-mode + ramper drives `phi_e` from
software).
