#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np

from export_mesh_shared import build_grid_mesh, default_output, latest_meta, load_capture


def write_stl(path, coords, faces):
    with path.open("w") as f:
        f.write("solid keyence_ljs640\n")
        for a, b, c in faces:
            p0 = coords[a]
            p1 = coords[b]
            p2 = coords[c]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal = normal / norm
            else:
                normal[:] = 0
            f.write(f"  facet normal {normal[0]:.6e} {normal[1]:.6e} {normal[2]:.6e}\n")
            f.write("    outer loop\n")
            f.write(f"      vertex {p0[0]:.6f} {p0[1]:.6f} {p0[2]:.6f}\n")
            f.write(f"      vertex {p1[0]:.6f} {p1[1]:.6f} {p1[2]:.6f}\n")
            f.write(f"      vertex {p2[0]:.6f} {p2[1]:.6f} {p2[2]:.6f}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write("endsolid keyence_ljs640\n")


def main():
    parser = argparse.ArgumentParser(description="Generate an STL mesh from a KEYENCE LJ-S capture.")
    parser.add_argument("meta", nargs="?", type=Path, help="Capture *_meta.csv. Defaults to latest in captures/.")
    parser.add_argument("--stride", type=int, default=4, help="Downsampling stride. Default: 4.")
    parser.add_argument("--output", "-o", type=Path, help="Output .stl path.")
    args = parser.parse_args()

    if args.stride < 1:
        raise SystemExit("--stride must be >= 1")

    meta_path = args.meta or latest_meta(Path("captures"))
    _, height_path, valid, z, x_pitch_mm, y_pitch_mm = load_capture(meta_path)
    coords, faces, mesh_width, mesh_height = build_grid_mesh(valid, z, x_pitch_mm, y_pitch_mm, args.stride)
    output = args.output or default_output(meta_path, f"_mesh_stride{args.stride}.stl")
    write_stl(output, coords, faces)

    print(f"input: {height_path}")
    print(f"mesh grid: {mesh_width} x {mesh_height}")
    print(f"valid vertices: {len(coords)}")
    print(f"faces: {len(faces)}")
    print(f"output: {output} ({output.stat().st_size / 1024 / 1024:.2f} MiB)")


if __name__ == "__main__":
    main()
