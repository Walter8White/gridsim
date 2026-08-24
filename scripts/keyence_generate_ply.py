#!/usr/bin/env python3
import argparse
from pathlib import Path

from keyence_export_mesh_shared import build_grid_mesh, default_output, latest_meta, load_capture


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAPTURE_DIR = REPO_ROOT / "keyence_ljs8000" / "CPP" / "captures"


def write_ply(path, coords, faces, height_path, stride):
    with path.open("w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"comment generated from {height_path.name}\n")
        f.write("comment units millimeters\n")
        f.write(f"comment stride {stride}\n")
        f.write(f"element vertex {len(coords)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")
        for x, y, z in coords:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
        for a, b, c in faces:
            f.write(f"3 {a} {b} {c}\n")


def main():
    parser = argparse.ArgumentParser(description="Generate a PLY mesh from a KEYENCE LJ-S capture.")
    parser.add_argument("meta", nargs="?", type=Path, help="Capture *_meta.csv. Defaults to latest KEYENCE capture.")
    parser.add_argument("--stride", type=int, default=4, help="Downsampling stride. Default: 4.")
    parser.add_argument("--x-min-mm", type=float, help="Crop minimum X coordinate in mm.")
    parser.add_argument("--x-max-mm", type=float, help="Crop maximum X coordinate in mm.")
    parser.add_argument("--y-min-mm", type=float, help="Crop minimum Y coordinate in mm.")
    parser.add_argument("--y-max-mm", type=float, help="Crop maximum Y coordinate in mm.")
    parser.add_argument("--max-z-jump-mm", type=float, help="Drop faces whose vertices span more than this Z range.")
    parser.add_argument("--output", "-o", type=Path, help="Output .ply path.")
    args = parser.parse_args()

    if args.stride < 1:
        raise SystemExit("--stride must be >= 1")

    meta_path = args.meta or latest_meta(DEFAULT_CAPTURE_DIR)
    _, height_path, valid, z, x_pitch_mm, y_pitch_mm = load_capture(meta_path)
    coords, faces, mesh_width, mesh_height = build_grid_mesh(
        valid,
        z,
        x_pitch_mm,
        y_pitch_mm,
        args.stride,
        x_min_mm=args.x_min_mm,
        x_max_mm=args.x_max_mm,
        y_min_mm=args.y_min_mm,
        y_max_mm=args.y_max_mm,
        max_z_jump_mm=args.max_z_jump_mm,
    )
    output = args.output or default_output(meta_path, f"_mesh_stride{args.stride}.ply")
    write_ply(output, coords, faces, height_path, args.stride)

    print(f"input: {height_path}")
    print(f"mesh grid: {mesh_width} x {mesh_height}")
    print(f"valid vertices: {len(coords)}")
    print(f"faces: {len(faces)}")
    print(f"output: {output} ({output.stat().st_size / 1024 / 1024:.2f} MiB)")


if __name__ == "__main__":
    main()
