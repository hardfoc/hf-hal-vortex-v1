# TMCL Parameter Scan — Post-config baseline (motor type = BLDC_MOTOR)

| Field | Value |
|-------|-------|
| Snapshot | 2026-05-?? — ESP32-C6, `vortex_tmcl_param_scan_post_config_spi`, FW051V100 silicon rev 1 var 2 |
| Transport | SPI (TMCL via Vortex onboard SPI bus, bootloader `enable_flash=false` so SPI free for TMCL) |
| Pre-config baseline | [`TMCL_PARAM_SCAN_FAILURES.md`](TMCL_PARAM_SCAN_FAILURES.md) (`MOTOR_TYPE = NO_MOTOR`) |
| Reset | DTR/RTS pulse via pyserial |

## Headline result

| Counter | Pre-config (NO_MOTOR) | Post-config (BLDC_MOTOR + DRV_EN active) | Δ |
|---------|----------------------:|------------------------------------------:|---:|
| GAP `REPLY_OK` (0x64) | ~14 | **263** | **+249** |
| GAP `REPLY_WRONG_TYPE` (0x03) | 259 | **10** | −249 |
| GAP `REPLY_CMD_NOT_AVAILABLE` (0x06) | 26 | **26** | 0 |

249 axis parameters become readable once `MOTOR_TYPE` is bound to `BLDC_MOTOR`. This includes the entire NR 256–283 gate-driver OC + VGS-short protection block (all marked **RWE** in the datasheet) — confirming the pre-config `WRONG_TYPE` was a motor-state artifact, not a stack bug.

## Residual GAP `REPLY_WRONG_TYPE` (10 NRs)

All accounted for:

| NR | Driver enum | Datasheet | Reason |
|---:|-------------|-----------|--------|
| 86, 87, 88 | UNKNOWN_PARAM | — | Gaps in this firmware — not exposed |
| 158, 159, 160 | UNKNOWN_PARAM | — | Gaps |
| 179, 180 | UNKNOWN_PARAM | — | Gaps |
| 215 | UNKNOWN_PARAM | — | Gap |
| 230 | `RESET_IIT_SUMS` | **W** | Genuinely write-only — `WRONG_TYPE` on GAP is correct |

None of these are real RWE / RW reads being denied.

## Residual GAP `REPLY_CMD_NOT_AVAILABLE` (26 NRs)

All in the bootloader-disabled subsystems:

| NR range | Subsystem | Bootloader gate (`Tmc9660Handler::kDefaultBootConfig`) |
|---------:|-----------|--------------------------------------------------------|
| 181–201 (21 NRs) | SPI encoder + SPI-LUT correction | `cfg.spiEncoder.enable = false` |
| 205–209 (5 NRs)  | Step/Dir input feedback         | `cfg.stepDir.enable = false`    |

The chip itself emits `REPLY_CMD_NOT_AVAILABLE` (0x06) for these. Any host-side "feature awareness" guard around `TMC9660<Comm>::FeedbackSense::SpiEncoder` / `StepDir` only needs to surface this status as a clear `ESP_LOGE` at the API boundary; the underlying detection is already done by silicon.

## How to reproduce

```bash
cd hf-hal-vortex-v1/examples/esp32
source ~/esp/esp-idf/export.sh
./scripts/build_app.sh vortex_tmcl_param_scan_post_config_spi Release
./scripts/flash_app.sh vortex_tmcl_param_scan_post_config_spi Release flash

# capture ~80 s of serial via pyserial DTR/RTS reset on /dev/ttyACM0
python3 - <<'PY'
import serial, time
p = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
p.dtr = False; p.rts = True; time.sleep(0.1); p.rts = False
end = time.time() + 80
buf=[]
while time.time() < end:
    c = p.read(8192)
    if c: buf.append(c.decode('utf-8','replace'))
print(''.join(buf))
PY
```

UART variant (`vortex_tmcl_param_scan_post_config`) runs the same probe through the UART transport — useful when the host SPI bus is reserved for SPI-flash on a future board variant.
