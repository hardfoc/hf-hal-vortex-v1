# TMC9660 Driver Deep Audit & BLDC Coverage Report

**Scope:** `hf-tmc9660-driver` + `hf-core/handlers/tmc9660` + `hf-hal-vortex-v1` examples & manager
**Datasheets reviewed:** `tmc9660.pdf` (main IC), `tmc9660-parameter-mode-reference-manual.pdf`,
`tmc9660-register- mode-reference manual.pdf`, `tmc9660-configuration-and-bootstrapping.pdf`,
`tmc9660-3ph-eval-kit-ug.pdf`, `TMCL_reference_2015.pdf` — all converted to text under
`/tmp/tmc9660_text/` and grepped against source.

---

## 1. Executive Summary

> **Driver math is correct.** Two independent deep audits (one focused on unit conversions and
> scaling, one focused on BLDC feature coverage and user-friendliness) cross-checked **40+ formulas
> and constants** against the datasheets. Every velocity/acceleration/position/current/voltage
> conversion, every register encoding, every default value used by the auto-config helpers, and
> every PI-norm enum matches the parameter-mode reference manual.
>
> What remained were two real findings — both already fixed in commit
> `5d18d4e` of `hf-tmc9660-driver` and propagated up through the submodule chain
> (`hf-core-drivers @ 0a42f13` → `hf-core @ ff405f4` → `hf-hal-vortex-v1 @ 7fe3659` and
> `hf-hal-flux-v1 @ 50a691d` → `hf-all-hals @ 3401738`):
>
> 1. **CSA-gain auto-selection** in `CurrentSensing::configureAuto` now picks the *highest* gain
>    with adequate full-scale headroom (was: lowest qualifying), giving up to **8× better ADC
>    current resolution** for low-current motors.
> 2. **`AbnConfig::initOpenloopCurrent_mA`** field added so the ABN auto-configurator writes
>    `OPENLOOP_CURRENT` (param 46) — the parameter the chip *actually* uses for the
>    `FORCED_PHI_E_*` rotor-alignment step. Old behaviour (chip default 1000 mA) silently
>    completed init with a bogus phi_e offset on gearboxed motors and produced FOC vibration only.
>
> The remaining gaps in the audit are **opportunities to push more responsibility into the library**
> (and out of the bench/example layer) — none of them are correctness bugs. A prioritised plan is in
> §5 below.

---

## 2. Datasheet ↔ Driver Verification Matrix (selected)

| Quantity | Driver location | Formula in code | Datasheet anchor | Verdict |
|---|---|---|---|---|
| Velocity register ↔ RPM | `tmc9660_units.hpp` | `k_RPM = CPR · 2²⁴ / (40 MHz · 60)` | param-mode §"Velocity Scaling" / param 124 | ✅ |
| Acceleration register ↔ RPM/s | `tmc9660_units.hpp` | `k_a = 2¹⁷ / 40 MHz` | param-mode §"Acceleration scaling" | ✅ |
| Velocity saturation | `tmc9660_units.hpp` | `±(2²⁷ − 1)` | param-mode | ✅ |
| Acceleration range | `tmc9660.ipp` Ramp | `[1, 2²³ − 1]` | param-mode | ✅ |
| Current scaling (peak) | `tmc9660.ipp` `CurrentSensing::configureAuto` | `Factor = 39.06 / (G·Rₛₕᵤₙₜ)` | param-mode p.45 | ✅ |
| Current scaling (RMS) | same | `Factor = 27.62 / (G·Rₛₕᵤₙₜ)` | param-mode p.45 | ✅ |
| CSA gain choice | same | now: **highest** gain s.t. `I_FS ≥ 1.5·I_peak`, fallback `5×` | param-mode p.45, eval-kit ug | ✅ (improved) |
| ABN `STEPS` | `tmc9660.ipp` `configureABNEncoder` | post-quad CPR (e.g. 16384 for 4096 PPR) | param-mode §"ABN encoder" | ✅ |
| ABN init forced-phi alignment | `tmc9660.ipp` `configureAuto(AbnConfig)` | now writes `OPENLOOP_CURRENT` (param 46, mA) when `initMethod ∈ {FORCED_PHI_E_*}` and field non-zero | param-mode §"ABN Initialization Methods" lines 2411-2422 + p.50 | ✅ (added) |
| Hall offsets | `tmc9660.ipp` `configureAuto(HallConfig)` | degrees → `0..32767` mapping | param-mode §"Digital Hall" | ✅ |
| Electrical angle (`phi_e`) | various | `int16` ±32768 = ±180° electrical | param-mode §"Phi_e wrap" | ✅ |
| Mechanical angle | derived | `phi_m = phi_e / pole_pairs` | physics + param-mode | ✅ |
| BBM dead-time | `tmc9660.ipp` `GateDriver` | `t_ns = reg · 8.33 ns` (120 MHz tick) | param-mode §"Gate driver" | ✅ |
| Gate drive time | `tmc9660.ipp` `GateDriver` | `t = (1/120 MHz)·(2·reg + 3)` | param-mode §"Gate driver" | ✅ |
| OVP/UVP encoding | `tmc9660.ipp` `Protection::configureVoltage` | mV → register | param-mode §"Voltage protection" | ✅ |
| I²t windows | `tmc9660.ipp` `Protection::configureAuto` | dual-window thermal model | param-mode §"I²t" | ✅ |
| PI normalization shifts | enum `*PNormalization`, `*INormalization` | `0..15 → 2⁰..2¹⁵` (X-macro values) | param-mode tables | ✅ |
| MotorContext.cpr() | `tmc9660_units.hpp` | `SAME_AS_COMMUTATION → 65536·pp`, `HALL → 6·pp`, `ABN → encoder_cpr` | param-mode §"Velocity feedback selector" | ✅ |
| Brake-chopper threshold | `tmc9660.ipp` `BrakeChopper::configureAuto` | mV → register, hysteresis encoded as % | param-mode §"Brake chopper" | ✅ |

A full row-by-row table (40+ items) is preserved in the audit report at
`/tmp/tmc9660_audit_units.md`.

---

## 3. BLDC Feature Coverage Summary

A separate coverage audit cross-referenced the chip's **parameter-mode** capability surface with
the driver subsystems exposed under `tmc9660::TMC9660<Comm>::*`. Result: **all features needed for
the Vortex board's BLDC + brake-chopper-on-Y2 topology are wrapped**, with one missing surface
(sensorless startup helpers — see §5):

| Subsystem | Coverage |
|---|---|
| MotorConfig (type, pole pairs, fPWM) | ✅ Full |
| GateDriver (UVW + Y2, BBM, drive time, IDRIVE/IBLANK trim, VGS-short protection, charge-pump UVLO) | ✅ Full (chip-side; capability-gated for Y2) |
| CurrentSensing (shunt selection, CSA gain, scaling factor auto-calc, ADC offset calibration) | ✅ Full |
| VoltageSense (supply telemetry, OVP/UVP) | ✅ Full |
| Protection (OVP, UVP, OT, I²t dual-window, fault-latch policy) | ✅ Full |
| FeedbackSense — Hall | ✅ Full |
| FeedbackSense — ABN1 | ✅ Full (now incl. `initOpenloopCurrent_mA`) |
| FeedbackSense — ABN2 (mech-shaft) | ✅ Full |
| FeedbackSense — SPI absolute encoder | ✅ Full |
| FeedbackSense — Hall+ABN fusion / index latching | ⚠ Partial (no high-level wrapper) |
| Commutation modes (SYSTEM_OFF, FOC_*OPENLOOP, FOC_HALL, FOC_ABN, FOC_ABN2, FOC_SPI) | ✅ Full |
| Sensorless startup (IPD / HFI) | ❌ Missing high-level helper (chip supports) |
| FOC tuning (torque-PI, flux-PI, velocity-PI, position-PI) | ✅ Full |
| Ramp generator (8-point, direct-velocity, RFS reference search) | ✅ Full (S-curve = host-side) |
| Brake (passive, active short, brake chopper, mechanical brake) | ✅ Full |
| Step/Dir input | ✅ Full (bootloader-gated) |
| Stepper motor mode | ✅ Full (capability-gated; Y2 NOT driven on Vortex) |
| Telemetry (supply, currents, voltages, temperature, ramp, encoder, FOC currents/voltages) | ✅ Full |
| Diagnostics (compact + full snapshots in `tmc9660_diagnostics.hpp`) | ✅ Full (new) |
| Persistent storage (STAP / global parameter banks) | ✅ Full |
| TMCL command set (RFS, error recovery) | ✅ Full |

A full coverage matrix (~50 rows) is preserved at `/tmp/tmc9660_audit_coverage.md`.

---

## 4. Stepper / Y2 / Brake-Repurposing Safety

The Vortex 3-phase board uses U/V/W as the BLDC half-bridges and **repurposes Y2 for the
brake-chopper / dissipator path**. The audit confirmed the driver does **not** drive Y2 in BLDC
mode unless the user opts in:

* `Tmc9660Handler::DeviceCapabilities` carries a `y2Phase` flag. Vortex boards that wire Y2 to a
  brake resistor set this `false`, and every Y2-touching SAP (NR 237–248, 272–282) early-returns
  with a logged ERROR.
* Bootloader `STEPPER_ENABLE` is `0` on Vortex; the chip never enters stepper mode, so Y2 cannot
  be silently driven as the second stepper coil.
* `Brake::configureMechanicalBrake` and `BrakeChopper::configureAuto` route through Y2_LS / Y2_HS
  only when explicitly configured, and are gated by the same capability bit.

**Verdict:** ✅ Safe. The brake-chopper repurposing on Y2 is correctly isolated.

---

## 5. Findings & Recommended Next Steps (prioritised)

### Priority 1 — Library-side, *low effort, high impact*

1. **Document `OPENLOOP_CURRENT` requirement on `FORCED_PHI_E_*` init in the driver README and
   troubleshooting guide.** Already done in `inc/tmc9660.hpp` Doxygen and
   `docs/troubleshooting.md`; cross-link from the BLDC quickstart.
2. **Expose `initOpenloopCurrent_mA` in the matching `Abn2Config` and `SpiEncoderConfig`** (the
   same datasheet section applies to both — the chip uses `OPENLOOP_CURRENT` for any
   `FORCED_PHI_E_*` alignment regardless of feedback type). Field already added for ABN1; mirror it.
3. **Expose `OUTPUT_VOLTAGE_LIMIT` and `MAX_TORQUE` in `TorqueFluxConfig`** so users don't have to
   remember to write them by SAP — they're protective limits and should be part of any auto-config.

### Priority 2 — Library-side, *medium effort, high impact*

4. **`MotorNameplate` + `BoardSpec` + `bringUpBldc()` one-call.** Today the bench in
   `examples/esp32/main/common/vortex_motor_bench_common.hpp` orchestrates **13** sub-config calls
   (`configure_complete_bldc` lines ≈860–915) plus per-app one-offs. Promote that orchestration into
   `Tmc9660Handler::bringUpBldc(MotorNameplate, BoardSpec, FeedbackConfig)` so the example shrinks
   to a handful of lines and so PI gains, current scaling, OVP/UVP, BBM, IDRIVE and brake-chopper
   thresholds are *derived from datasheet-grade nameplate data* rather than copy-pasted constants.
   Concrete inputs (per audit):

   * `MotorNameplate{ pole_pairs, V_nom, I_max, R_phase, L_phase, Kt, Kv, J_rotor, omega_max }`
   * `BoardSpec{ shunt_R_mOhm, V_supply, V_overvolt, V_undervolt, mosfet_Qg_nC, mosfet_Rds_mOhm, has_y2_phase, vgs_short_workaround }`
   * `FeedbackConfig{ type, encoder_cpr, init_method, init_openloop_current_mA, ... }`

5. **Auto-derive PI gains from `MotorNameplate`.** Today `torqueP=400, torqueI=400, velocityP=1000,
   velocityI=2` are tuning constants chosen by trial-and-error. The audit recommends using the
   standard cascade-form torque-loop bandwidth from `Kt`, `L_phase` and a target bandwidth in Hz,
   and the velocity loop from rotor inertia `J` — both derivations are textbook (e.g. *Bose, Modern
   Power Electronics*, ch.8) and can live as a `tmc9660::tuning::derivePiGains(...)` helper.

6. **Wrap the silicon-quirk workarounds** that today live in
   `vortex_motor_bench_common.hpp` (`disable_vortex_uvw_vgs_short_protection`, the 120-line
   `diagnose_uvw_gate_current_tmcl`) inside the driver as opt-in `GateDriver::*` helpers. This way
   any HAL using this silicon — not just Vortex — benefits, and the example layer stops carrying
   firmware-version-specific dead code.

7. **`setCommutationModeWithRecovery(mode, retries=3)`** in the handler. Today every example does
   the SYSTEM_OFF → clear-faults → re-arm dance manually; a single helper is a five-line addition
   in `Tmc9660Handler` and removes ~20 lines from each bench app.

### Priority 3 — Library-side, *higher effort, situational impact*

8. **Sensorless startup helpers** (`FeedbackSense::configureIPD`, `configureHFI`). The chip
   supports both per the parameter-mode manual; we have no driver wrapper. Useful when an encoder
   is unavailable.
9. **Encoder index / N-channel latching helper** (`FeedbackSense::configureIndexLatching`).
   Multi-revolution position reset; not blocking for the current Vortex bring-up.
10. **Host-side S-curve / jerk-limited ramp profile generator** to layer over the chip's 8-point
    linear ramper.
11. **`Telemetry::getThermalState()`** decoding the I²t cumulative-energy registers into a
    "remaining trip time" — useful for predictive thermal back-off in apps.

### Priority 4 — Vortex HAL & docs

12. **Migrate `vortex_bldc_*` examples to use the new `bringUpBldc()` once it lands** in the
    handler. Today they all call into `vortex_motor_bench_common.hpp`; the bench helpers should
    become thin shims that call into the handler.
13. **Cross-link the new `docs/units_and_diagnostics.md`** from
    `hf-hal-vortex-v1/docs/component-handlers/README.md` and the example `examples/esp32/README.md`,
    and reference it from the Vortex bring-up guide. Same for `hf-hal-flux-v1`.
14. **Update `Tmc9660Handler.h` Doxygen** examples to show the unit-aware
    `setTargetVelocityRpm(rpm, motorContext)` pattern (already done in the latest `hf-core` commit
    `ff405f4`).

---

## 6. What the Audit Could *Not* Verify From Source Alone

* The chip-internal anti-windup behaviour of the cascaded PI controllers (no datasheet equation;
  recommended to characterise empirically per motor and document in a tuning note).
* The exact CSA gain mapping inside the IC for `GAIN_5×/10×/20×/40×` selectors (the datasheet
  states the *nominal* gains; the formula `I_FS = 2.5 V / (G · Rₛₕᵤₙₜ)` and the
  `CURRENT_SCALING_FACTOR = 39.06 / (G·Rₛₕᵤₙₜ)` chain are correct *given* those nominal gains).
* Whether the `FORCED_PHI_E_90_ZERO` swing test ever rotates the rotor enough on a high-stiction
  10:1 gearbox at any practical OPENLOOP_CURRENT — this is a system-level question (motor + load),
  not a driver-correctness question.

---

## 7. Status of the In-Progress FOC_ABN Bring-Up (Vortex)

Independent of the audit, the Vortex bench has been instrumented with a deep per-tick FOC-chain
trace (see [vortex_bldc_abn_as5047_dynamics.cpp](../main/vortex_bldc_abn_as5047_dynamics.cpp)
lines ≈700-735). Once the test rig is reconnected, capturing one ramp will pinpoint which stage of
the velocity-PI → torque-PI → Uq pipeline is currently failing on the bench motor, with the new
`OPENLOOP_CURRENT = 2000 mA` alignment current confirmed in the post-init log.

---

## 8. Commit / Push Status

All audit-driven changes have been committed and pushed in dependency order:

| Repo | Commit | Branch |
|---|---|---|
| `hf-tmc9660-driver` | `5d18d4e` — feat: engineering-unit helpers, diagnostic snapshots, ABN/CSA improvements | `main` |
| `hf-core-drivers` | `0a42f13` — chore(tmc9660): bump driver | `main` |
| `hf-core` | `ff405f4` — chore: bump hf-core-drivers; align Tmc9660Handler examples | `main` |
| `hf-hal-vortex-v1` | `7fe3659` — feat(esp32): TMC9660 BLDC bring-up apps + FOC_ABN diagnostics + driver bump | `main` |
| `hf-hal-flux-v1` | `50a691d` — chore: bump hf-core to engineering-units + ABN/CSA improvements | `main` |
| `hf-all-hals` | `3401738` — chore: bump vortex and flux HALs | `main` |
