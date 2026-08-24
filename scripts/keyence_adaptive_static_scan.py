#!/usr/bin/env python3
"""Run KEYENCE multi-preset scans selected by the OpenMV classifier.

This first version does not move the Zaber. It reads a line from the OpenMV H7
over USB serial, extracts the requested presets, applies each KEYENCE preset,
and acquires one scan per preset at the current mechanical position.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import serial


REPO_ROOT = Path(__file__).resolve().parents[1]
KEYENCE = REPO_ROOT / "scripts" / "keyence.sh"
CAPTURE_DIR = REPO_ROOT / "keyence_ljs8000" / "CPP" / "captures"


@dataclass(frozen=True)
class KeyencePreset:
    name: str
    exposure: int
    dynamic_range: int
    light_mode: int
    light_upper: int
    light_lower: int
    detection_sensitivity: int
    dead_zone_interpolation: int
    peak_width_filter: tuple[str, int | None]


PRESETS: dict[str, KeyencePreset] = {
    "NORMAL": KeyencePreset("NORMAL", 12, 6, 2, 99, 99, 5, 2, ("on", 2)),
    "DARK": KeyencePreset("DARK", 15, 9, 0, 99, 99, 5, 2, ("off", None)),
    "SHINY": KeyencePreset("SHINY", 9, 9, 2, 99, 99, 5, 2, ("on", 3)),
}

PRESET_ORDER = ("NORMAL", "DARK", "SHINY")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read OpenMV surface classes and acquire KEYENCE scans with matching presets."
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="OpenMV serial port.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--serial-timeout-s", type=float, default=12.0)
    parser.add_argument("--timeout-ms", type=int, default=60000, help="KEYENCE acquisition timeout.")
    parser.add_argument(
        "--save-mode",
        choices=("raw", "all"),
        default="all",
        help="KEYENCE save mode. 'all' also saves invalid-red PNG previews.",
    )
    parser.add_argument(
        "--preset",
        action="append",
        choices=PRESET_ORDER,
        help="Bypass OpenMV and force preset(s). Can be passed multiple times.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print actions without touching KEYENCE.")
    return parser.parse_args()


def latest_meta() -> Path | None:
    metas = sorted(CAPTURE_DIR.glob("*_meta.csv"), key=lambda p: p.stat().st_mtime)
    return metas[-1] if metas else None


def read_meta(path: Path | None) -> dict[str, str]:
    if path is None or not path.exists():
        return {}
    values: dict[str, str] = {}
    with path.open("r", newline="", encoding="utf-8") as fp:
        for row in csv.reader(fp):
            if len(row) >= 2:
                values[row[0]] = row[1]
    return values


def run_keyence(args: list[str], dry_run: bool) -> None:
    cmd = [str(KEYENCE), *args]
    print("+", " ".join(cmd), flush=True)
    if not dry_run:
        subprocess.run(cmd, check=True)


def apply_preset(preset: KeyencePreset, dry_run: bool) -> None:
    run_keyence(["setting", "set", "exposure", str(preset.exposure)], dry_run)
    run_keyence(["setting", "set", "dynamic_range", str(preset.dynamic_range)], dry_run)
    run_keyence(["setting", "set", "light_mode", str(preset.light_mode)], dry_run)
    run_keyence(["setting", "set", "light_upper", str(preset.light_upper)], dry_run)
    run_keyence(["setting", "set", "light_lower", str(preset.light_lower)], dry_run)
    run_keyence(["setting", "set", "detection_sensitivity", str(preset.detection_sensitivity)], dry_run)
    run_keyence(["setting", "set", "dead_zone_interpolation", str(preset.dead_zone_interpolation)], dry_run)
    peak_mode, peak_strength = preset.peak_width_filter
    if peak_mode == "off":
        run_keyence(["setting", "set", "peak_width_filter", "off"], dry_run)
    else:
        run_keyence(["setting", "set", "peak_width_filter", "on", str(peak_strength)], dry_run)


def scan_once(preset_name: str, timeout_ms: int, save_mode: str, dry_run: bool) -> dict[str, str]:
    if dry_run:
        flag = "--save-all" if save_mode == "all" else "--save-raw"
        run_keyence(["scan", flag, "--timeout-ms", str(timeout_ms)], dry_run)
        return {
            "preset": preset_name,
            "meta_path": "",
            "invalid_pixel_percent": "",
            "height_min_raw": "",
            "height_max_raw": "",
        }

    before = latest_meta()
    flag = "--save-all" if save_mode == "all" else "--save-raw"
    run_keyence(["scan", flag, "--timeout-ms", str(timeout_ms)], dry_run)
    after = latest_meta()
    if after == before:
        raise RuntimeError("KEYENCE scan did not create a new metadata file.")
    meta = read_meta(after)
    return {
        "preset": preset_name,
        "meta_path": str(after) if after else "",
        "invalid_pixel_percent": meta.get("invalid_pixel_percent", ""),
        "height_min_raw": meta.get("height_min_raw", ""),
        "height_max_raw": meta.get("height_max_raw", ""),
    }


def parse_presets_from_line(line: str) -> list[str]:
    presets_value = ""
    for part in line.strip().split():
        if part.startswith("presets="):
            presets_value = part.split("=", 1)[1]
            break
    if not presets_value:
        return ["NORMAL"]

    requested = {name.strip().upper() for name in presets_value.split(",") if name.strip()}
    selected = [name for name in PRESET_ORDER if name in requested and name in PRESETS]
    return selected or ["NORMAL"]


def read_openmv_presets(port: str, baud: int, timeout_s: float) -> tuple[list[str], str]:
    deadline = time.monotonic() + timeout_s
    last_line = ""
    with serial.Serial(port, baudrate=baud, timeout=0.5) as ser:
        ser.reset_input_buffer()
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            last_line = line
            print("openmv:", line, flush=True)
            if line.startswith("OPENMV_SURFACE"):
                return parse_presets_from_line(line), line
    raise TimeoutError(f"No OPENMV_SURFACE line received from {port}. Last line: {last_line!r}")


def write_manifest(openmv_line: str, results: list[dict[str, str]], dry_run: bool) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = CAPTURE_DIR / f"adaptive_{stamp}_manifest.json"
    payload = {
        "created_at": stamp,
        "openmv_line": openmv_line,
        "results": results,
    }
    print("manifest:", path)
    if not dry_run:
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path


def main() -> int:
    args = parse_args()
    if args.preset:
        presets = [name for name in PRESET_ORDER if name in set(args.preset)]
        openmv_line = "forced presets=" + ",".join(presets)
    else:
        presets, openmv_line = read_openmv_presets(args.port, args.baud, args.serial_timeout_s)

    print("selected presets:", ",".join(presets), flush=True)
    results: list[dict[str, str]] = []
    for name in presets:
        preset = PRESETS[name]
        print(f"\n=== {name} ===", flush=True)
        apply_preset(preset, args.dry_run)
        results.append(scan_once(name, args.timeout_ms, args.save_mode, args.dry_run))

    write_manifest(openmv_line, results, args.dry_run)
    print("\nDone.")
    for result in results:
        print(
            "{preset}: invalid={invalid_pixel_percent} meta={meta_path}".format(**result),
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
