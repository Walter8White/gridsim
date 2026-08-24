#!/usr/bin/env python3
"""Sweep KEYENCE acquisition parameters and save annotated invalid-red images.

The script keeps the stage fixed. For each parameter set it:
  1. applies KEYENCE settings,
  2. runs one scan with --save-invalid-image only,
  3. annotates the PNG with the parameter values,
  4. stores it under keyence_ljs8000/CPP/captures/sweep/<run_id>/.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import shutil
import subprocess
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


REPO_ROOT = Path(__file__).resolve().parents[1]
KEYENCE = REPO_ROOT / "scripts" / "keyence.sh"
CAPTURE_DIR = REPO_ROOT / "keyence_ljs8000" / "CPP" / "captures"
SWEEP_ROOT = CAPTURE_DIR / "sweep"


@dataclass(frozen=True)
class SweepCase:
    exposure: int
    dynamic_range: int
    light_mode: int
    light_upper: int = 99
    light_lower: int = 99
    detection_sensitivity: int = 5
    dead_zone_interpolation: int = 2
    peak_width_filter: str = "on"
    peak_width_strength: int = 3

    @property
    def name(self) -> str:
        peak = "off" if self.peak_width_filter == "off" else f"on{self.peak_width_strength}"
        return (
            f"exp{self.exposure:02d}_dr{self.dynamic_range}_lm{self.light_mode}_"
            f"lu{self.light_upper}_ll{self.light_lower}_ds{self.detection_sensitivity}_"
            f"dz{self.dead_zone_interpolation}_peak{peak}"
        )

    @property
    def title(self) -> str:
        peak = "off" if self.peak_width_filter == "off" else f"on {self.peak_width_strength}"
        light_name = {0: "manual", 1: "auto", 2: "slope"}.get(self.light_mode, str(self.light_mode))
        return (
            f"exposure={self.exposure}  dynamic_range={self.dynamic_range}  "
            f"light_mode={self.light_mode}({light_name})  "
            f"light={self.light_lower}-{self.light_upper}  "
            f"detection={self.detection_sensitivity}  "
            f"dead_zone={self.dead_zone_interpolation}  peak={peak}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a KEYENCE parameter sweep and save annotated invalid-red PNGs."
    )
    parser.add_argument("--timeout-ms", type=int, default=60000)
    parser.add_argument(
        "--profile",
        choices=("handle", "quick", "full", "exposure12"),
        default="handle",
        help="Sweep size. 'exposure12' keeps exposure fixed and sweeps other useful params.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        help="Output directory. Default: captures/sweep/<timestamp>_<profile>",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def build_cases(profile: str) -> list[SweepCase]:
    if profile == "quick":
        return [
            SweepCase(8, 9, 2, peak_width_filter="on", peak_width_strength=3),
            SweepCase(9, 9, 2, peak_width_filter="on", peak_width_strength=3),
            SweepCase(10, 9, 2, peak_width_filter="on", peak_width_strength=3),
            SweepCase(11, 9, 2, peak_width_filter="on", peak_width_strength=3),
            SweepCase(12, 8, 0, peak_width_filter="off"),
            SweepCase(15, 9, 0, peak_width_filter="off"),
        ]

    if profile == "full":
        cases: list[SweepCase] = []
        for exposure, dynamic_range, light_mode, peak in itertools.product(
            range(5, 14),
            (7, 8, 9),
            (0, 2),
            ("off", "on"),
        ):
            cases.append(
                SweepCase(
                    exposure,
                    dynamic_range,
                    light_mode,
                    peak_width_filter=peak,
                    peak_width_strength=3,
                )
            )
        return cases

    if profile == "exposure12":
        cases: list[SweepCase] = []
        for dynamic_range, light_mode, peak, dead_zone in itertools.product(
            (6, 7, 8, 9),
            (0, 1, 2),
            ("off", "on"),
            (0, 2),
        ):
            cases.append(
                SweepCase(
                    12,
                    dynamic_range,
                    light_mode,
                    dead_zone_interpolation=dead_zone,
                    peak_width_filter=peak,
                    peak_width_strength=3,
                )
            )
        cases.extend(
            [
                SweepCase(12, 9, 0, light_upper=60, light_lower=60, peak_width_filter="on", peak_width_strength=3),
                SweepCase(12, 9, 0, light_upper=75, light_lower=75, peak_width_filter="on", peak_width_strength=3),
                SweepCase(12, 9, 0, light_upper=90, light_lower=90, peak_width_filter="on", peak_width_strength=3),
                SweepCase(12, 9, 2, light_upper=60, light_lower=60, peak_width_filter="on", peak_width_strength=3),
                SweepCase(12, 9, 2, light_upper=75, light_lower=75, peak_width_filter="on", peak_width_strength=3),
                SweepCase(12, 9, 2, light_upper=90, light_lower=90, peak_width_filter="on", peak_width_strength=3),
            ]
        )
        return cases

    # Reflective metal / handle sweep. This stays reasonably short while
    # covering low/mid exposure values that often rescue shiny surfaces.
    cases = []
    for exposure, dynamic_range, light_mode in itertools.product(
        range(6, 13),
        (8, 9),
        (0, 2),
    ):
        cases.append(
            SweepCase(
                exposure,
                dynamic_range,
                light_mode,
                peak_width_filter="on",
                peak_width_strength=3,
            )
        )
    cases.extend(
        [
            SweepCase(13, 9, 0, peak_width_filter="off"),
            SweepCase(14, 9, 0, peak_width_filter="off"),
            SweepCase(15, 9, 0, peak_width_filter="off"),
        ]
    )
    return cases


def latest_invalid_png() -> Path | None:
    images = sorted(CAPTURE_DIR.glob("*_invalid_red.png"), key=lambda p: p.stat().st_mtime)
    return images[-1] if images else None


def run_command(args: list[str], dry_run: bool) -> str:
    cmd = [str(KEYENCE), *args]
    print("+", " ".join(cmd), flush=True)
    if dry_run:
        return ""
    result = subprocess.run(cmd, check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if result.stdout:
        print(result.stdout, end="", flush=True)
    return result.stdout


def set_keyence(case: SweepCase, dry_run: bool) -> None:
    run_command(["setting", "set", "exposure", str(case.exposure)], dry_run)
    run_command(["setting", "set", "dynamic_range", str(case.dynamic_range)], dry_run)
    run_command(["setting", "set", "light_mode", str(case.light_mode)], dry_run)
    run_command(["setting", "set", "light_upper", str(case.light_upper)], dry_run)
    run_command(["setting", "set", "light_lower", str(case.light_lower)], dry_run)
    run_command(["setting", "set", "detection_sensitivity", str(case.detection_sensitivity)], dry_run)
    run_command(["setting", "set", "dead_zone_interpolation", str(case.dead_zone_interpolation)], dry_run)
    if case.peak_width_filter == "off":
        run_command(["setting", "set", "peak_width_filter", "off"], dry_run)
    else:
        run_command(
            ["setting", "set", "peak_width_filter", "on", str(case.peak_width_strength)],
            dry_run,
        )


def parse_invalid_percent(output: str) -> str:
    for line in output.splitlines():
        if "Invalid pixels shown in red:" not in line:
            continue
        if "(" in line and "%)" in line:
            return line.rsplit("(", 1)[1].split("%", 1)[0]
    return ""


def annotate_image(src: Path, dst: Path, title: str, subtitle: str) -> None:
    image = Image.open(src).convert("RGB")
    font = ImageFont.load_default()
    draw_probe = ImageDraw.Draw(image)
    title_box = draw_probe.textbbox((0, 0), title, font=font)
    subtitle_box = draw_probe.textbbox((0, 0), subtitle, font=font)
    title_h = title_box[3] - title_box[1]
    subtitle_h = subtitle_box[3] - subtitle_box[1]
    bar_h = title_h + subtitle_h + 18

    annotated = Image.new("RGB", (image.width, image.height + bar_h), (20, 20, 20))
    annotated.paste(image, (0, bar_h))
    draw = ImageDraw.Draw(annotated)
    draw.text((8, 5), title, fill=(255, 255, 255), font=font)
    draw.text((8, 9 + title_h), subtitle, fill=(255, 210, 80), font=font)
    annotated.save(dst)


def write_manifest(path: Path, rows: list[dict[str, object]]) -> None:
    json_path = path / "manifest.json"
    csv_path = path / "manifest.csv"
    json_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    if rows:
        with csv_path.open("w", newline="", encoding="utf-8") as fp:
            writer = csv.DictWriter(fp, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)


def main() -> int:
    args = parse_args()
    cases = build_cases(args.profile)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = args.out or (SWEEP_ROOT / f"{stamp}_{args.profile}")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Output: {out_dir}")
    print(f"Cases: {len(cases)}")

    rows: list[dict[str, object]] = []
    for index, case in enumerate(cases, start=1):
        print(f"\n=== {index:03d}/{len(cases):03d} {case.name} ===", flush=True)
        before = latest_invalid_png()
        set_keyence(case, args.dry_run)
        output = run_command(["scan", "--save-invalid-image", "--timeout-ms", str(args.timeout_ms)], args.dry_run)

        if args.dry_run:
            rows.append({"index": index, **asdict(case), "invalid_percent": "", "image": ""})
            continue

        after = latest_invalid_png()
        if after is None or after == before:
            raise RuntimeError("KEYENCE scan did not create a new invalid-red PNG.")

        invalid_percent = parse_invalid_percent(output)
        dst = out_dir / f"{index:03d}_{case.name}_invalid_red.png"
        subtitle = f"case={index}/{len(cases)}  invalid={invalid_percent or 'unknown'}%  source={after.name}"
        annotate_image(after, dst, case.title, subtitle)
        after.unlink()

        row = {
            "index": index,
            **asdict(case),
            "invalid_percent": invalid_percent,
            "image": str(dst),
        }
        rows.append(row)
        write_manifest(out_dir, rows)

    write_manifest(out_dir, rows)
    print(f"\nDone. Images saved in: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
