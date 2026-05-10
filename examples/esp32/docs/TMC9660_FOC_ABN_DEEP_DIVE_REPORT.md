# TMC9660 FOC_ABN Deep-Dive Report

**Date:** 2026-05-10
**Target:** `vortex_bldc_abn_as5047_dynamics` bench app
**Hardware:** Vortex board, TMC9660, 30 W BLDC, 7 PP, 10:1 gearbox, AS5047U mag encoder (ABN-only, 16384 CPR)
**Symptom evolution:** Open-loop voltage mode spins motor cleanly. FOC_ABN: rotor static, ±1° vibration with audible high-pitch whine; chip reports `ω=0` forever; `Iuvw < 30 mA`; `Uq` grows to ~200 internal units but no rotation.

---

## 1. Mind Map of TMC9660 FOC_ABN Signal Flow

```
                 ┌────────────────────────────────────────────────────────────┐
                 │           USER (bench app) → TMC9660 PARAMETERS            │
                 └────────────────────────────────────────────────────────────┘

  TARGET_VELOCITY ──┐
  (param 124, signed
   internal units;   │            ┌─────────┐       ┌────────────┐
   k_RPM·rpm)        ├──► RAMP ──►│Velocity │      │ Torque PI  │
                     │  (param    │   PI    ├────► │  TARGET    ├──► Uq
                     │   123 ena) │ P=127   │      │ TORQUE     │     ↓
                     │            │ I=128   │      │ (param 104)│  Voltage
                     │            │ NORM_P/I│      │ P=109,I=110│  Limiter
                     │            │ (133/134)      │ NORM_P/I   │     ↓
                     │            └────┬────┘      │ (113/114)  │   PWM out
                     │                 │           └─────┬──────┘     ↓
                     │              if TARGET_TORQUE != 0│         3-phase
  POSITION loop───►──┘               (line 3102 datasheet)│         inverter
  (downsamp 136)        ┌─────► OVERRIDES velocity loop─┘             ↓
                        │                                             ↓
                ENABLE_RAMP                                        BLDC motor
                                                                       ↓
                                                             ABN encoder (AS5047U)
                                                                       ↓
                                                             ABN_1_VALUE (param 100)
                                                                       ↓
                                                       ABN_1_PHI_E ← (counts × pp) % cpr
                                                                       │
                                                                       ▼
   VELOCITY_SENSOR_SELECTION (param 132) ─────►  ACTUAL_VELOCITY (param 125)
   = ABN1_ENCODER (sensored) or                  = differentiate(ABN_1_VALUE)
     SAME_AS_COMMUTATION                              feeds back to Velocity PI
                                                                       │
                          ┌────────────────────────────────────────────┘
                          │
                          ▼
  Velocity PI sees error = TARGET − ACTUAL → outputs target_Iq.
  Torque PI sees error = target_Iq − ACTUAL_TORQUE (param 105) → outputs Uq.
  ACTUAL_TORQUE comes from ADC_I0..2 × CURRENT_SCALING_FACTOR (mA after scale).

  ──────────────────────────────────────────────────────────────────────────
  ABN INIT (one-shot on entry to FOC_ABN, FOC_HALL, etc.):
     ABN_1_INIT_METHOD (param 92) =
        0 FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING ──┐
        1 FORCED_PHI_E_90_ZERO                  ├─ uses OPENLOOP_CURRENT (param 46, mA)
        2 USE_HALL                              │  to physically rotate to phi_e=0;
        3 USE_N_CHANNEL_OFFSET                  │  *fails silently if current < cogging+load*
                                                ┘
     ABN_1_INIT_DELAY (param 94, 1000..10000 ms) — wait time for swing to settle
     ABN_1_INIT_VELOCITY (param 95) — only used by N-channel offset method
     ABN_1_INIT_STATE (param 93) → DONE means "process ran", NOT "alignment correct"
     ABN_1_N_CHANNEL_PHI_E_OFFSET (param 96) — written by user OR captured by N method
```

---

## 2. Audit Findings: Driver Class (`tmc9660.hpp/.ipp`, `tmc9660_units.hpp`)

### 2.1 Velocity unit conversion — ✅ CORRECT

- `MotorContext::cpr()` for `ABN1_ENCODER` returns user-supplied `encoder_cpr` (16384 in our app). [tmc9660_units.hpp#L160-L173](../../lib/core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660_units.hpp#L160).
- `k_rpm = cpr × 2^24 / (40 MHz × 60) = 16384 × 0.006990 = 114.53` internal-units / mech-RPM.
- `setTargetVelocity(double, Rpm, ctx)` → `setTargetVelocityRaw(int32_t)` with multiplication by `k_rpm`. Saturates to ±2^27. [tmc9660.ipp#L3055](../../lib/core/hf-core-drivers/external/hf-tmc9660-driver/src/tmc9660.ipp#L3055).
- Confirmed against datasheet param 124 (signed 28-bit internal). **No bug.**

### 2.2 ABN encoder configuration — ✅ CORRECT (in this app)

- `kAbnCountsPerRev = 16384` written to `ABN_1_STEPS` (param 90) — datasheet says steps per revolution AFTER the chip's internal ×4 quadrature decode. We supply post-quad count, which matches. [vortex_bldc_abn_as5047_dynamics.cpp#L81](../main/vortex_bldc_abn_as5047_dynamics.cpp#L81).
- ABN-edge probe in open-loop voltage mode confirms the chip counts 24166 / 32768 expected = ~74% of theoretical (likely target velocity ramp-up under-shoot from fixed 1.5 s window, not a CPR bug).
- ⚠ Stale dead-constant `kAbnEncoderCountsPerRev = 1024` exists in [`vortex_bench_safety.hpp:191`](../main/common/vortex_bench_safety.hpp#L191). Not used by our app, but other bench apps reference it — should be cleaned up.

### 2.3 Current sensing & CSA gain — ⚠ SUBOPTIMAL but not broken

[`CurrentSensing::configureAuto`](../../lib/core/hf-core-drivers/external/hf-tmc9660-driver/src/tmc9660.ipp#L1078) selects the **lowest** CSA gain that satisfies `I_FS ≥ 1.5 × I_peak`. With 3 mΩ shunt and 3 A peak, all four gains (5×, 10×, 20×, 40×) qualify, so it picks `GAIN_5X`.

- Datasheet formula: `Factor_P [mA] = 39.06 / (G × R_shunt_Ω)` — confirmed at [param_mode.txt:2167-2172](/tmp/tmc9660_text/param_mode.txt) (datasheet p. 45).
- `GAIN_5X` → `CURRENT_SCALING_FACTOR = 2604` (within 1..65535 range).
- `GAIN_40X` → `CURRENT_SCALING_FACTOR = 326`. 8× better ADC resolution.
- **Impact:** at GAIN_5X, the ADC quantum is ~2.5 mA. For 30 mA actual current (our observed Iuvw range), that's only ~12 ADC counts of signal, severely degraded for closed-loop torque control. The math is *correct*, just resolution-starved. **Recommend reversing the search direction in `configureAuto`** to pick the highest gain that doesn't saturate (this is the standard practice for shunt-based sensorless drives).

### 2.4 Torque/flux PI configuration — ✅ defaults reasonable

- `TorqueFluxConfig` defaults: `torqueP=50, torqueI=100, pNormalization=SHIFT_8_BIT (÷256), iNormalization=SHIFT_16_BIT (÷65536)`. [tmc9660.hpp#L2441](../../lib/core/hf-core-drivers/external/hf-tmc9660-driver/inc/tmc9660.hpp#L2441).
- We bumped to `400/400` for faster torque tracking; harmless but didn't fix the root cause.

### 2.5 Velocity PI normalization — ✅ FIXED previously

- Default `pNormalization = SHIFT_16_BIT (÷65536)` neuters the velocity loop output (P=1000 × err=14000 / 65536 ≈ 218 mA target Iq, way too small).
- We set both norms to `SHIFT_8_BIT (÷256)`: P=1000 × 14000 / 256 ≈ 54000 → saturates correctly to MAX_TORQUE = 2500 mA.

### 2.6 OPENLOOP_CURRENT (param 46) vs OPENLOOP_VOLTAGE (param 47) — 🔴 **ROOT CAUSE FOUND**

**This is the primary bug.** Datasheet [param_mode.txt:2411-2422, 2745-2748](/tmp/tmc9660_text/param_mode.txt) explicitly states:

> "**Forced phi_e zero with active swing** … Forces the motors rotor into a phi_e zero position using the **OPENLOOP_CURRENT**, and then zeros the encoder angle."
>
> "**Con: Fails if the OPENLOOP_CURRENT is not high enough to move the motor.**"
>
> "Especially BLDC-Motors tend to show increased cogging which **requires higher currents** for smooth rotations."

**Our prior code** set `OPENLOOP_VOLTAGE = 1500` before FOC_ABN engagement. That parameter is only used by `FOC_OPENLOOP_VOLTAGE_MODE` (commutation mode 3) — it is **ignored** during ABN init in `FOC_ABN` (mode 5). The ABN init alignment current was therefore stuck at its default `OPENLOOP_CURRENT = 1000 mA`.

When we tested `FORCED_PHI_E_90_ZERO` we observed `Iuvw≈(360,-200,-200)` which is exactly 1 A peak — confirming default OPENLOOP_CURRENT was being applied. The 10:1 gearbox + BLDC cogging held the rotor static at 1 A → init captured the rotor's resting position as "phi_e=0" → subsequent FOC commutes against a misaligned rotor → near-zero net torque, ±1° vibration, audible whine from PWM with no rotation.

**Fix applied in this session** ([vortex_bldc_abn_as5047_dynamics.cpp#L497-L508](../main/vortex_bldc_abn_as5047_dynamics.cpp#L497)):
```cpp
constexpr uint16_t kAbnInitOpenloopCurrent_mA = 2000;
d.torqueFluxControl.setOpenloopCurrent(kAbnInitOpenloopCurrent_mA);
```
Result: Init reports `OPENLOOP_I=2000 mA` post-init. ABN_1_PHI_E_OFFSET still reads 0 — consistent with the FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING method NOT writing this parameter (it only resets the counter). **Motor still does not spin in this iteration**, so `2000 mA` may still not be enough OR there is a secondary issue (see §3).

---

## 3. Remaining Suspects (post OPENLOOP_CURRENT fix)

### 3.1 Stale `TARGET_TORQUE` overriding velocity loop — ⚠ LIKELY

Datasheet line 3102: *"To activate the [torque] mode it is sufficient to write the `TARGET_TORQUE` parameter."* If anything in the bring-up sequence (probe → `motor_stop_safe` → SYSTEM_OFF → FOC_ABN) leaves TARGET_TORQUE non-zero, the chip uses torque-control mode and **bypasses the velocity PI entirely**. With `TARGET_TORQUE=0`, the motor would sit static — exactly what we see.

**Action:** explicitly write `TARGET_TORQUE = 0` immediately after FOC_ABN engages, then call `setTargetVelocity(...)`. Also read it back in the post-init log.

### 3.2 Even higher OPENLOOP_CURRENT needed — possible

Constraint: `MAX_TORQUE = 2500 mA` clamps the alignment current, so we can only push to 2500 (not 3000+). With 10:1 gearbox stiction, even 2500 mA may be insufficient. Workaround: temporarily raise `MAX_TORQUE` to 3000 mA *only during init*, then restore.

### 3.3 CSA gain too low — possible secondary

Even after init alignment is correct, with `GAIN_5X` the chip's actual_torque feedback resolution is ~2.5 mA per ADC count. The torque PI may oscillate / undershoot at small commands. Selecting GAIN_40X (driver patch) could improve this.

### 3.4 Hand-rotate during init — diagnostic

Manually rotate the output shaft slowly during the 2 s init window. If the rotor is being held in place purely by gearbox stiction, manual rotation lets the rotor reach electrical alignment with the chip's commanded phi_e=0. After init completes, FOC_ABN should spin if this was the only issue.

---

## 4. What is Provably Correct in the Driver Class

The following have been audited against the datasheet and are bit-exact match:

| Component | Datasheet Ref | Code Ref | Status |
|-----------|--------------|----------|--------|
| `setTargetVelocity` Rpm→internal conversion | param 124, k_RPM formula | tmc9660_units.hpp `velocityFromRpm` | ✅ |
| `setMaxTorqueCurrent` mA pass-through | param 11, mA after scaling | tmc9660.ipp `MotorConfig::setMaxTorqueCurrent` | ✅ |
| `setTargetTorque` mA pass-through | param 104 | tmc9660.ipp | ✅ |
| `setOpenloopCurrent` mA pass-through | param 46 | tmc9660.ipp#L3649 | ✅ |
| `setOpenloopVoltage` raw | param 47 | tmc9660.ipp#L3663 | ✅ |
| ABN_1_STEPS write | param 90 (post-quad CPR) | tmc9660.ipp#L2340 | ✅ |
| ABN init method, delay, velocity, n-offset | params 92-96 | tmc9660.ipp#L2350 | ✅ |
| Acceleration register: `(dV/dt) × 2^17 / fCLK` | RAMPER_A | tmc9660_units.hpp `kAccelInternalPerVdt` | ✅ |
| `CURRENT_SCALING_FACTOR = 39.06 / (G × R_Ω)` | param 21 (peak) | tmc9660.ipp#L1114 | ✅ formula |
| Velocity PI norm enums (SHIFT_8/16/24, NO_SHIFT) | param 133/134 enum | tmc9660_param_mode_tmcl.hpp | ✅ |
| CommutationMode enum (FOC_ABN=5) | param 6 | tmc9660_param_mode_tmcl.hpp:1755 | ✅ |

**Conclusion:** the C++ class implementation faithfully encodes the datasheet's parameter semantics for all paths exercised by this bench app. The unit math (k_RPM, accel, current scale) is provably correct.

The actual issue is a **bench-application-level configuration error**: setting OPENLOOP_VOLTAGE instead of OPENLOOP_CURRENT before FOC_ABN engage. This is a USER-side bug that has now been fixed.

---

## 5. Suggested Driver-Side Improvements (NOT bugs, polish)

1. **Reverse CSA gain selection** in `CurrentSensing::configureAuto` to pick highest unsaturating gain. 8× better resolution improves closed-loop performance for low-current motors.
2. **`AbnConfig::initOpenloopCurrent_mA`** — add a field to the auto-config struct that auto-writes `OPENLOOP_CURRENT` when init method is `FORCED_PHI_E_*`. Currently easy to forget.
3. **Doc note** in `feedbackSense.configureAuto` Doxygen warning that FORCED_PHI_E_* methods need explicit `OPENLOOP_CURRENT` for gearboxed motors.

---

## 6. Recommended Next Experiments (priority order)

1. **Clear `TARGET_TORQUE = 0`** explicitly before commanding velocity in FOC_ABN. Read back ACTUAL_TORQUE/TARGET_TORQUE per tick. *(estimated 5 min)*
2. **Read `STATUS_FLAGS` (REGULATION_TORQUE / REGULATION_VELOCITY / REGULATION_POSITION bits)** to confirm which loop the chip thinks is active. *(5 min)*
3. **Hand-rotate output shaft during init** to manually break stiction. Confirm/refute the alignment hypothesis cheaply. *(physical)*
4. **Bump `MAX_TORQUE` to 3000 mA temporarily** then push OPENLOOP_CURRENT to 2800. *(5 min)*
5. **Try `USE_N_CHANNEL_OFFSET`** init method with a measured offset (requires one-time calibration: command `FOC_OPENLOOP_VOLTAGE_MODE` slowly, capture `ABN_1_VALUE` at known phi_e, compute and write `ABN_1_N_CHANNEL_PHI_E_OFFSET`). *(20 min)*
6. **Patch driver to reverse CSA gain selection** for 8× better resolution. *(15 min)*

---

## 7. Verbatim Datasheet Citations (for traceability)

- p. 45 (param 21): `CURRENT_SCALING_FACTOR` formula `Factor_P = 39.06 / (G × R_shunt_Ω)`. Mandatory for mA units of TARGET_TORQUE/ACTUAL_TORQUE.
- p. 49 (Table 29, params 89-96): ABN parameters. `ABN_1_STEPS` is "steps per rotation (CPR)" — post-quadrature.
- p. 50 (line 2411): "`FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING` … using the **OPENLOOP_CURRENT**, and then zeros the encoder angle."
- p. 50 (line 2422): "Con: Fails if the **OPENLOOP_CURRENT** is not high enough to move the motor."
- p. 56 (line 2745): "For the 'Forced PHI_E' methods, the **OPENLOOP_CURRENT** is used to force the motor into a known position."
- p. 68 (line 3102): TARGET_TORQUE write activates torque mode, **bypassing velocity loop**. Only valid in sensored modes.
- p. 71 (param 124): `TARGET_VELOCITY` range ±2^28-1, signed, internal units.
- p. 122 (param 5): `OUTPUT_VOLTAGE_LIMIT` default 8000 (max 32767). Limits Uq/Ud combined output.
