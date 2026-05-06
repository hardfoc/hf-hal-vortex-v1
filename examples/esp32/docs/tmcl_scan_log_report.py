#!/usr/bin/env python3
"""
Join vortex_tmcl_param_scan serial WARN lines with tmc9660_param_mode_tmcl.hpp definitions.

Usage (from `examples/esp32/`):

  python3 docs/tmcl_scan_log_report.py /path/to/monitor_capture.txt -o docs/tmcl_param_scan_failures.csv
  python3 docs/tmcl_scan_log_report.py /path/to/monitor_capture.txt --markdown docs/TMCL_PARAM_SCAN_FAILURES.md
  python3 docs/tmcl_scan_log_report.py /path/to/monitor_capture.txt --summary-only

Requires a full idf.py monitor log containing lines like:
  W (...) vortex_tmcl_scan: GAP type=10 (WAKE_PIN_CONTROL_ENABLE) motor=0  TMCL=REPLY_WRONG_TYPE ...
  W (...) vortex_tmcl_scan: GGP bank=0 type=5  TMCL=REPLY_WRONG_TYPE ...
"""
from __future__ import annotations

import argparse
import csv
import datetime as _dt
import re
import sys
from collections import defaultdict
from pathlib import Path

# Lists that populate `enum class Parameters` (axis / GAP TYPE field).
AXIS_PARAMETER_LIST_MACROS = [
    "GATE_DRIVER_LIST",
    "OVERCURRENT_PROTECTION_LIST",
    "UNDERVOLTAGE_PROTECTION_LIST",
    "VGS_SHORT_PROTECTION_LIST",
    "MOTOR_CONFIG_LIST",
    "ADC_CONFIG_LIST",
    "FEEDBACK_SENSOR_CONFIG_LIST",
    "TORQUE_FLUX_CONTROL_LIST",
    "VELOCITY_CONTROL_LIST",
    "POSITION_CONTROL_LIST",
    "RAMPER_STOP_CONFIG_LIST",
    "BIQUAD_FILTER_LIST",
    "FAULT_HANDLING_LIST",
    "IIT_MONITOR_LIST",
    "TEMPERATURE_PROTECTION_LIST",
    "HEARTBEAT_MONITORING_LIST",
    "BRAKE_CHOPPER_LIST",
    "MECHANICAL_BRAKE_LIST",
    "REFERENCE_SEARCH_LIST",
    "STEP_DIR_LIST",
    "HIBERNATION_WAKEUP_LIST",
    "SYSTEM_STATUS_SUPPLY_LIST",
    "INTERNAL_MEASUREMENT_LIST",
    "COMBINED_DIAGNOSTIC_VALUES_LIST",
    "ERRORS_AND_FLAGS_LIST",
]

GLOBAL_BANK_MACROS = {
    0: "GLOBAL_PARAM_BANK0_LIST",
    2: "GLOBAL_PARAM_BANK2_LIST",
    3: "GLOBAL_PARAM_BANK3_LIST",
}

X_LINE = re.compile(r"^\s*X\(\s*([A-Za-z0-9_]+)\s*,\s*(\d+)\s*,")


def strip_ansi(s: str) -> str:
    return re.sub(r"\x1b\[[0-9;]*m", "", s)


def extract_x_macro_pairs(hpp_lines: list[str], macro: str) -> list[tuple[str, int]]:
    """Body of #define MACRO(X) \\ ... contiguous X(...) lines only."""
    start_pat = f"#define {macro}(X)"
    i = 0
    while i < len(hpp_lines):
        if hpp_lines[i].strip().startswith(start_pat):
            i += 1
            out: list[tuple[str, int]] = []
            while i < len(hpp_lines):
                raw = hpp_lines[i]
                m = X_LINE.match(raw)
                if m:
                    out.append((m.group(1), int(m.group(2))))
                    i += 1
                    continue
                if raw.strip() == "" or raw.strip().startswith("//"):
                    i += 1
                    continue
                break
            return out
        i += 1
    raise SystemExit(f"Macro {macro} not found in header")


def build_axis_driver_map(hpp: Path) -> tuple[dict[int, list[str]], dict[int, list[str]]]:
    lines = hpp.read_text(encoding="utf-8", errors="replace").splitlines()
    by_val: dict[int, list[str]] = defaultdict(list)
    for macro in AXIS_PARAMETER_LIST_MACROS:
        for name, val in extract_x_macro_pairs(lines, macro):
            by_val[val].append(f"{macro}:{name}")
    dupes = {v: ns for v, ns in by_val.items() if len(ns) > 1}
    return dict(by_val), dupes


def build_global_maps(hpp: Path) -> dict[int, dict[int, list[str]]]:
    lines = hpp.read_text(encoding="utf-8", errors="replace").splitlines()
    banks: dict[int, dict[int, list[str]]] = {}
    for bank, macro in GLOBAL_BANK_MACROS.items():
        m: dict[int, list[str]] = defaultdict(list)
        for name, val in extract_x_macro_pairs(lines, macro):
            m[val].append(f"{macro}:{name}")
        banks[bank] = dict(m)
    return banks


GAP_RE = re.compile(
    r"vortex_tmcl_scan:\s*GAP type=(\d+)\s*\(([^)]*)\)\s*motor=(\d+)\s+TMCL=(\S+)\s+\(0x[0-9A-Fa-f]+\)"
)
GGP_RE = re.compile(
    r"vortex_tmcl_scan:\s*GGP bank=(\d+)\s*type=(\d+)\s+TMCL=(\S+)\s+\(0x[0-9A-Fa-f]+\)"
)


def parse_log(path: Path) -> tuple[list[dict], list[dict]]:
    gap_rows: list[dict] = []
    ggp_rows: list[dict] = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = strip_ansi(raw)
        m = GAP_RE.search(line)
        if m:
            gap_rows.append(
                {
                    "op": "GAP",
                    "motor": int(m.group(3)),
                    "type": int(m.group(1)),
                    "log_name": m.group(2).strip(),
                    "tmcl_status": m.group(4),
                }
            )
            continue
        m = GGP_RE.search(line)
        if m:
            ggp_rows.append(
                {
                    "op": "GGP",
                    "bank": int(m.group(1)),
                    "type": int(m.group(2)),
                    "tmcl_status": m.group(3),
                }
            )
    return gap_rows, ggp_rows


def _md_cell(s: str) -> str:
    """Keep GitHub-flavored markdown table cells on one line."""
    return s.replace("|", "\\|").replace("\n", " ").replace("\r", "")


def write_markdown_report(
    out: Path,
    log_file: Path,
    header: Path,
    csv_rows: list[dict[str, str]],
    gap_doc_fail: int,
    gap_undoc_fail: int,
    ggp_doc_fail: dict[int, int],
    ggp_undoc_fail: dict[int, int],
    axis_dupes: dict[int, list[str]],
) -> None:
    gap_lines = [r for r in csv_rows if r["op"] == "GAP"]
    ggp_lines = [r for r in csv_rows if r["op"] == "GGP"]
    gap_lines.sort(key=lambda r: int(r["type"]))
    ggp_lines.sort(key=lambda r: (int(r["bank"]), int(r["type"])))
    by_status: dict[str, int] = defaultdict(int)
    for r in csv_rows:
        by_status[r["tmcl_status"]] += 1

    lines: list[str] = [
        "# TMCL parameter read scan: non-OK reads",
        "",
        "This file lists every **GAP** (axis, motor 0) and **GGP** (global bank) read that returned a "
        "TMCL status other than success during `vortex_tmcl_param_scan` (numeric sweep). "
        "Rows are **ROM / probe outcomes**, not necessarily bugs in the host driver.",
        "",
        "| Field | Value |",
        "| --- | --- |",
        f"| Source log | `{log_file}` |",
        f"| Driver header | `{header}` |",
        f"| Generated (local) | {_dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} |",
        "",
        "## Summary counts",
        "",
        f"- **GAP** WARN lines: **{len(gap_lines)}** (driver `Parameters` list: {gap_doc_fail}, not in those lists: {gap_undoc_fail})",
    ]
    for b in (0, 2, 3):
        td = ggp_doc_fail.get(b, 0)
        tu = ggp_undoc_fail.get(b, 0)
        tot = td + tu
        if tot:
            lines.append(
                f"- **GGP bank {b}** WARN lines: **{tot}** (driver global list: {td}, sweep-only type id: {tu})"
            )
    lines += [
        "",
        "## Failures grouped by TMCL status",
        "",
        "| TMCL status | Count |",
        "| --- | ---: |",
    ]
    for st, c in sorted(by_status.items(), key=lambda x: (-x[1], x[0])):
        lines.append(f"| {_md_cell(st)} | {c} |")
    lines += [
        "",
        "## GAP failures (axis parameter NR / TYPE, motor 0)",
        "",
        "| NR | Name (from log) | TMCL status | In driver lists? | Driver macro:symbol |",
        "| ---: | --- | --- | --- | --- |",
    ]
    for r in gap_lines:
        lines.append(
            "| "
            + " | ".join(
                [
                    r["type"],
                    _md_cell(r["log_enum_name"]),
                    _md_cell(r["tmcl_status"]),
                    r["in_driver_Parameters_lists"],
                    _md_cell(r["driver_symbols"] or "—"),
                ]
            )
            + " |"
        )
    lines += [
        "",
        "## GGP failures (global parameter bank / type)",
        "",
        "| Bank | Type | TMCL status | In driver lists? | Driver macro:symbol |",
        "| ---: | ---: | --- | --- | --- |",
    ]
    for r in ggp_lines:
        lines.append(
            "| "
            + " | ".join(
                [
                    r["bank"],
                    r["type"],
                    _md_cell(r["tmcl_status"]),
                    r["in_driver_Parameters_lists"],
                    _md_cell(r["driver_symbols"] or "—"),
                ]
            )
            + " |"
        )
    if axis_dupes:
        lines += [
            "",
            "## Note: duplicate axis TYPE ids in header",
            "",
            "Some numeric TYPE values map to more than one `X(...)` entry in `tmc9660_param_mode_tmcl.hpp`.",
            "",
        ]
        for val, names in sorted(axis_dupes.items()):
            lines.append(f"- **{val}**: {', '.join(names)}")
    lines.append("")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("log_file", type=Path, help="idf_monitor log with vortex_tmcl_scan WARN lines")
    ap.add_argument(
        "--header",
        type=Path,
        default=None,
        help="tmc9660_param_mode_tmcl.hpp (default: vortex driver copy)",
    )
    ap.add_argument("-o", "--csv", type=Path, default=None, help="Write combined failure CSV")
    ap.add_argument(
        "--markdown",
        type=Path,
        default=None,
        help="Write markdown report (all GAP/GGP failures from log, joined with driver header)",
    )
    ap.add_argument("--summary-only", action="store_true")
    args = ap.parse_args()

    repo_driver = (
        Path(__file__).resolve().parents[3]
        / "lib/core/hf-core-drivers/external/hf-tmc9660-driver/inc/parameter_mode/tmc9660_param_mode_tmcl.hpp"
    )
    hpp = args.header or repo_driver
    if not hpp.is_file():
        print(f"Header not found: {hpp}", file=sys.stderr)
        sys.exit(1)

    axis_by_val, axis_dupes = build_axis_driver_map(hpp)
    global_maps = build_global_maps(hpp)

    gap_rows, ggp_rows = parse_log(args.log_file)
    if not gap_rows and not ggp_rows:
        print("No GAP/GGP failure lines matched. Is this a full monitor log with WARN lines?", file=sys.stderr)
        sys.exit(2)

    # --- Summary buckets
    gap_doc_fail = gap_undoc_fail = 0
    ggp_doc_fail: dict[int, int] = defaultdict(int)
    ggp_undoc_fail: dict[int, int] = defaultdict(int)

    csv_rows: list[dict[str, str]] = []

    for r in gap_rows:
        tid = r["type"]
        doc = tid in axis_by_val
        if doc:
            gap_doc_fail += 1
            driver = ";".join(axis_by_val[tid])
        else:
            gap_undoc_fail += 1
            driver = ""
        csv_rows.append(
            {
                "op": "GAP",
                "motor": str(r["motor"]),
                "bank": "",
                "type": str(tid),
                "log_enum_name": r["log_name"],
                "tmcl_status": r["tmcl_status"],
                "in_driver_Parameters_lists": "yes" if doc else "no",
                "driver_symbols": driver,
            }
        )

    for r in ggp_rows:
        b = r["bank"]
        tid = r["type"]
        doc = b in global_maps and tid in global_maps[b]
        if doc:
            ggp_doc_fail[b] += 1
            driver = ";".join(global_maps[b][tid])
        else:
            ggp_undoc_fail[b] += 1
            driver = ""
        csv_rows.append(
            {
                "op": "GGP",
                "motor": "",
                "bank": str(b),
                "type": str(tid),
                "log_enum_name": "",
                "tmcl_status": r["tmcl_status"],
                "in_driver_Parameters_lists": "yes" if doc else "no",
                "driver_symbols": driver,
            }
        )

    print(f"Log: {args.log_file}")
    print(f"Header: {hpp}")
    print(f"GAP failure lines: {len(gap_rows)}  (driver-listed type id: {gap_doc_fail}, not listed / hole sweep: {gap_undoc_fail})")
    for b in sorted(set(ggp_doc_fail.keys()) | set(ggp_undoc_fail.keys()) | {0, 2, 3}):
        if b not in (0, 2, 3):
            continue
        td = ggp_doc_fail.get(b, 0)
        tu = ggp_undoc_fail.get(b, 0)
        if td or tu:
            print(f"GGP bank {b} failure lines: {td + tu}  (driver-listed: {td}, sweep-only / undocumented id: {tu})")

    if axis_dupes:
        print(f"\nNote: {len(axis_dupes)} axis TYPE values appear under multiple symbols in header (duplicate numeric ids).")

    if args.markdown:
        write_markdown_report(
            args.markdown,
            args.log_file.resolve(),
            hpp.resolve(),
            csv_rows,
            gap_doc_fail,
            gap_undoc_fail,
            ggp_doc_fail,
            ggp_undoc_fail,
            axis_dupes,
        )
        print(f"\nWrote markdown report: {args.markdown}")

    if args.summary_only:
        return

    if args.csv:
        fields = [
            "op",
            "motor",
            "bank",
            "type",
            "log_enum_name",
            "tmcl_status",
            "in_driver_Parameters_lists",
            "driver_symbols",
        ]
        with args.csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(csv_rows)
        print(f"\nWrote {len(csv_rows)} rows to {args.csv}")


if __name__ == "__main__":
    main()
