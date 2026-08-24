#!/usr/bin/env python3
"""Step a Zaber stage and acquire one KEYENCE scan at each stopped position."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
KEYENCE_CPP_DIR = REPO_ROOT / "keyence_ljs8000" / "CPP"
KEYENCE_SCRIPT = REPO_ROOT / "scripts" / "keyence.sh"
KEYENCE_CAPTURE_DIR = KEYENCE_CPP_DIR / "captures"
ZABER_STAGE_SCRIPT = REPO_ROOT / "scripts" / "zaber_stage.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move-stop-scan loop: KEYENCE distance/profile width determines the Zaber step."
    )
    parser.add_argument("--start-mm", type=float, default=0.0, help="First absolute Zaber position.")
    parser.add_argument("--start-current", action="store_true", help="Use current Zaber position as the first scan position.")
    parser.add_argument("--max-mm", type=float, help="Legacy upward stop position. Prefer --end-mm or --travel-mm.")
    parser.add_argument("--end-mm", type=float, help="Final absolute Zaber position.")
    parser.add_argument("--travel-mm", type=float, help="Travel distance from start. Sign is chosen by --direction.")
    parser.add_argument("--direction", choices=("up", "down"), default="up", help="Sweep direction along the Zaber axis.")
    parser.add_argument("--velocity-mm-s", type=float, default=5.0, help="Zaber move velocity.")
    parser.add_argument("--overlap", type=float, default=0.10, help="Profile overlap ratio, 0..0.95.")
    parser.add_argument(
        "--width-percentile",
        type=float,
        default=10.0,
        help="Conservative percentile of per-point profile widths used for the next step.",
    )
    parser.add_argument(
        "--coverage-margin-mm",
        type=float,
        default=0.0,
        help="Extra margin subtracted from the effective profile width before applying overlap.",
    )
    parser.add_argument("--max-scans", type=int, default=0, help="Runtime guard. 0 means no count limit.")
    parser.add_argument("--settle-s", type=float, default=0.25, help="Delay after each Zaber move before scanning.")
    parser.add_argument(
        "--distance-bias-mm",
        type=float,
        default=0.0,
        help="Calibration bias added to KEYENCE distance samples before computing profile width.",
    )
    parser.add_argument("--fov-near-mm", type=float, default=385.0, help="Profile width at clearance distance.")
    parser.add_argument("--fov-far-mm", type=float, default=2000.0, help="Profile width at far measurement distance.")
    parser.add_argument("--clearance-mm", type=float, default=325.0, help="Minimum measurement distance.")
    parser.add_argument("--range-mm", type=float, default=1550.0, help="Measurement range beyond clearance.")
    parser.add_argument("--zaber-port", default="/dev/ttyUSB0", help="Zaber serial port.")
    parser.add_argument("--zaber-protocol", choices=("auto", "ascii"), default="ascii")
    parser.add_argument("--keyence-save-mode", choices=("raw", "all"), default="raw")
    parser.add_argument("--keyence-timeout-ms", type=int, default=60000, help="Timeout passed to KEYENCE scan.")
    return parser.parse_args()


def current_zaber_position_mm(args: argparse.Namespace) -> float:
    from zaber_motion import Units

    if args.zaber_protocol != "ascii":
        raise RuntimeError("--start-current currently requires --zaber-protocol ascii")

    from zaber_motion.ascii import Connection

    connection = Connection.open_serial_port(args.zaber_port)
    try:
        connection.enable_alerts()
        devices = connection.detect_devices(identify_devices=True)
        if not devices:
            raise RuntimeError("No Zaber devices detected.")
        return float(devices[0].get_axis(1).get_position(Units.LENGTH_MILLIMETRES))
    finally:
        connection.close()


def resolve_end_mm(args: argparse.Namespace, start_mm: float) -> float:
    sign = 1.0 if args.direction == "up" else -1.0
    if args.travel_mm is not None:
        if args.travel_mm <= 0:
            raise RuntimeError("--travel-mm must be > 0")
        return start_mm + sign * args.travel_mm
    if args.end_mm is not None:
        return args.end_mm
    if args.max_mm is not None:
        return args.max_mm
    raise RuntimeError("Provide one of --travel-mm, --end-mm, or --max-mm")


def sweep_active(position_mm: float, end_mm: float, direction: str) -> bool:
    if direction == "up":
        return position_mm <= end_mm
    return position_mm >= end_mm


def beyond_end(position_mm: float, end_mm: float, direction: str) -> bool:
    if direction == "up":
        return position_mm > end_mm
    return position_mm < end_mm


def latest_meta() -> Path:
    metas = sorted(KEYENCE_CAPTURE_DIR.glob("*_meta.csv"))
    if not metas:
        raise RuntimeError(f"No KEYENCE metadata found in {KEYENCE_CAPTURE_DIR}")
    return metas[-1]


def read_meta(meta_path: Path) -> dict[str, str]:
    meta = {}
    for line in meta_path.read_text().splitlines()[1:]:
        if "," in line:
            key, value = line.split(",", 1)
            meta[key] = value
    return meta


def run_keyence_scan(save_mode: str, timeout_ms: int) -> Path:
    flag = "--save-raw" if save_mode == "raw" else "--save-all"
    before = latest_meta() if KEYENCE_CAPTURE_DIR.exists() and list(KEYENCE_CAPTURE_DIR.glob("*_meta.csv")) else None
    subprocess.run([str(KEYENCE_SCRIPT), "scan", flag, "--timeout-ms", str(timeout_ms)], check=True)
    after = latest_meta()
    if before is not None and after == before:
        raise RuntimeError("KEYENCE scan did not create a new metadata file.")
    return after


def distance_samples_mm(meta_path: Path, distance_bias_mm: float) -> np.ndarray:
    meta = read_meta(meta_path)
    height_path = Path(meta["height_file"])
    if not height_path.is_absolute():
        height_path = KEYENCE_CPP_DIR / height_path
    if not height_path.exists():
        height_path = meta_path.with_name(meta_path.name.replace("_meta.csv", "_height_u16le.raw"))

    width = int(float(meta["x_pointnum"]))
    height = int(float(meta["y_pointnum"]))
    z_pitch_mm = float(meta["z_pitch_um"]) / 1000.0
    raw = np.fromfile(height_path, dtype="<u2")
    if raw.size != width * height:
        raise RuntimeError(f"Unexpected raw size {raw.size}, expected {width * height}")

    valid = raw != 0
    if not np.any(valid):
        raise RuntimeError(f"No valid KEYENCE height pixels in {height_path}")

    return (raw[valid].astype(np.float64) - 32768.0) * z_pitch_mm + distance_bias_mm


def profile_width_mm(distance_mm: float | np.ndarray, args: argparse.Namespace) -> float | np.ndarray:
    far_distance_mm = args.clearance_mm + args.range_mm
    alpha = (distance_mm - args.clearance_mm) / (far_distance_mm - args.clearance_mm)
    alpha = np.clip(alpha, 0.0, 1.0)
    width = args.fov_near_mm + alpha * (args.fov_far_mm - args.fov_near_mm)
    if np.isscalar(distance_mm):
        return float(width)
    return width


def effective_profile_width_mm(distances_mm: np.ndarray, args: argparse.Namespace) -> float:
    widths_mm = profile_width_mm(distances_mm, args)
    effective_width_mm = float(np.percentile(widths_mm, args.width_percentile))
    return max(0.0, effective_width_mm - args.coverage_margin_mm)


def send_zaber(position_mm: float, velocity_mm_s: float, args: argparse.Namespace) -> None:
    subprocess.run(
        [
            sys.executable,
            str(ZABER_STAGE_SCRIPT),
            "--port",
            args.zaber_port,
            "--protocol",
            args.zaber_protocol,
            "--command",
            "send",
            "--position-mm",
            f"{position_mm:.6f}",
            "--velocity-mm-s",
            f"{velocity_mm_s:.6f}",
        ],
        check=True,
    )


def main() -> int:
    args = parse_args()
    if not KEYENCE_SCRIPT.exists():
        print(f"Missing KEYENCE script: {KEYENCE_SCRIPT}", file=sys.stderr)
        return 2
    if args.velocity_mm_s <= 0:
        print("--velocity-mm-s must be > 0", file=sys.stderr)
        return 2
    if args.keyence_timeout_ms <= 0:
        print("--keyence-timeout-ms must be > 0", file=sys.stderr)
        return 2
    try:
        start_mm = current_zaber_position_mm(args) if args.start_current else args.start_mm
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    try:
        end_mm = resolve_end_mm(args, start_mm)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.direction == "up" and end_mm < start_mm:
        print("For --direction up, final position must be >= --start-mm", file=sys.stderr)
        return 2
    if args.direction == "down" and end_mm > start_mm:
        print("For --direction down, final position must be <= --start-mm", file=sys.stderr)
        return 2
    if not 0 <= args.overlap < 0.95:
        print("--overlap must be in [0, 0.95)", file=sys.stderr)
        return 2
    if not 0 <= args.width_percentile <= 100:
        print("--width-percentile must be in [0, 100]", file=sys.stderr)
        return 2
    if args.coverage_margin_mm < 0:
        print("--coverage-margin-mm must be >= 0", file=sys.stderr)
        return 2

    position_mm = start_mm
    scan_index = 0
    send_zaber(position_mm, args.velocity_mm_s, args)
    time.sleep(args.settle_s)

    step_sign = 1.0 if args.direction == "up" else -1.0
    while sweep_active(position_mm, end_mm, args.direction):
        if args.max_scans > 0 and scan_index >= args.max_scans:
            break

        meta_path = run_keyence_scan(args.keyence_save_mode, args.keyence_timeout_ms)
        distances_mm = distance_samples_mm(meta_path, args.distance_bias_mm)
        wall_distance_mm = float(np.median(distances_mm))
        width_mm = effective_profile_width_mm(distances_mm, args)
        step_mm = width_mm * (1.0 - args.overlap)

        print(
            "scan={scan} position_mm={position:.3f} wall_distance_mm={distance:.3f} "
            "effective_profile_width_mm={width:.3f} next_step_mm={step:.3f} "
            "direction={direction} end_mm={end:.3f} meta={meta}".format(
                scan=scan_index,
                position=position_mm,
                distance=wall_distance_mm,
                width=width_mm,
                step=step_mm,
                direction=args.direction,
                end=end_mm,
                meta=meta_path,
            )
        )

        if step_mm <= 0:
            raise RuntimeError(f"Computed non-positive step: {step_mm}")

        next_position_mm = position_mm + step_sign * step_mm
        if beyond_end(next_position_mm, end_mm, args.direction):
            break
        send_zaber(next_position_mm, args.velocity_mm_s, args)
        time.sleep(args.settle_s)
        position_mm = next_position_mm
        scan_index += 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
