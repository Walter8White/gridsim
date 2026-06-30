#!/usr/bin/env python3
"""Reconstruct a facade mesh from a Gocator point cloud.

The current scanner setup observes a mostly vertical facade, so the first
useful reconstruction is a 2.5D surface: X/Z define the facade grid and Y is
the measured depth/offset.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from demo_gocator2690_scan import synthetic_facade_y  # noqa: E402


def _read_ascii_ply_points(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as fp:
        first = fp.readline().strip()
        if first != "ply":
            raise ValueError(f"{path} is not a PLY file")

        vertex_count = None
        properties: list[str] = []
        in_vertex_element = False
        while True:
            line = fp.readline()
            if not line:
                raise ValueError("PLY header ended unexpectedly")
            line = line.strip()
            if line == "end_header":
                break
            tokens = line.split()
            if len(tokens) >= 3 and tokens[:2] == ["element", "vertex"]:
                vertex_count = int(tokens[2])
                in_vertex_element = True
            elif len(tokens) >= 3 and tokens[0] == "element":
                in_vertex_element = False
            elif in_vertex_element and len(tokens) >= 3 and tokens[0] == "property":
                properties.append(tokens[-1])

        if vertex_count is None:
            raise ValueError("PLY file has no vertex element")
        try:
            x_index = properties.index("x")
            y_index = properties.index("y")
            z_index = properties.index("z")
        except ValueError as exc:
            raise ValueError("PLY vertices must contain x, y, z properties") from exc

        points = np.empty((vertex_count, 3), dtype=np.float64)
        for index in range(vertex_count):
            values = fp.readline().split()
            if len(values) < len(properties):
                raise ValueError(f"vertex {index} has too few values")
            points[index] = (
                float(values[x_index]),
                float(values[y_index]),
                float(values[z_index]),
            )
    return points


def _estimate_resolution(values: np.ndarray, fallback: float) -> float:
    unique = np.unique(np.round(values, decimals=6))
    if unique.size < 2:
        return fallback
    diffs = np.diff(unique)
    diffs = diffs[diffs > 1e-6]
    if diffs.size == 0:
        return fallback
    return float(np.percentile(diffs, 5))


def _rasterize_height_map(
    points: np.ndarray,
    *,
    x_resolution_m: float | None,
    z_resolution_m: float | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    dx = x_resolution_m or _estimate_resolution(x, fallback=0.001)
    dz = z_resolution_m or _estimate_resolution(z, fallback=0.001)
    if dx <= 0.0 or dz <= 0.0:
        raise ValueError("grid resolution must be positive")

    x_min = math.floor(float(x.min()) / dx) * dx
    x_max = math.ceil(float(x.max()) / dx) * dx
    z_min = math.floor(float(z.min()) / dz) * dz
    z_max = math.ceil(float(z.max()) / dz) * dz

    width = int(round((x_max - x_min) / dx)) + 1
    height = int(round((z_max - z_min) / dz)) + 1
    sums = np.zeros((height, width), dtype=np.float64)
    counts = np.zeros((height, width), dtype=np.int32)

    xi = np.clip(np.round((x - x_min) / dx).astype(np.int64), 0, width - 1)
    zi = np.clip(np.round((z - z_min) / dz).astype(np.int64), 0, height - 1)
    np.add.at(sums, (zi, xi), y)
    np.add.at(counts, (zi, xi), 1)

    height_map = np.full((height, width), np.nan, dtype=np.float64)
    valid = counts > 0
    height_map[valid] = sums[valid] / counts[valid]
    x_axis = x_min + np.arange(width, dtype=np.float64) * dx
    z_axis = z_min + np.arange(height, dtype=np.float64) * dz
    return height_map, counts, x_axis, z_axis


def _fill_small_holes(height_map: np.ndarray, iterations: int) -> np.ndarray:
    filled = height_map.copy()
    for _ in range(max(0, iterations)):
        missing = ~np.isfinite(filled)
        if not missing.any():
            break
        sums = np.zeros_like(filled)
        counts = np.zeros(filled.shape, dtype=np.int32)
        for dz in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dz == 0:
                    continue
                shifted = np.full_like(filled, np.nan)
                src_z = slice(max(0, -dz), filled.shape[0] - max(0, dz))
                dst_z = slice(max(0, dz), filled.shape[0] - max(0, -dz))
                src_x = slice(max(0, -dx), filled.shape[1] - max(0, dx))
                dst_x = slice(max(0, dx), filled.shape[1] - max(0, -dx))
                shifted[dst_z, dst_x] = filled[src_z, src_x]
                valid = np.isfinite(shifted)
                sums[valid] += shifted[valid]
                counts[valid] += 1
        can_fill = missing & (counts > 0)
        filled[can_fill] = sums[can_fill] / counts[can_fill]
    return filled


def _mesh_from_height_map(
    height_map: np.ndarray,
    x_axis: np.ndarray,
    z_axis: np.ndarray,
    *,
    x_offset_m: float = 0.0,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int, int]]]:
    vertex_index = np.full(height_map.shape, -1, dtype=np.int64)
    vertices = []
    for row, z in enumerate(z_axis):
        for col, x in enumerate(x_axis):
            y = height_map[row, col]
            if np.isfinite(y):
                vertex_index[row, col] = len(vertices)
                vertices.append((float(x + x_offset_m), float(y), float(z)))

    faces = []
    for row in range(height_map.shape[0] - 1):
        for col in range(height_map.shape[1] - 1):
            ids = [
                vertex_index[row, col],
                vertex_index[row, col + 1],
                vertex_index[row + 1, col + 1],
                vertex_index[row + 1, col],
            ]
            if all(index >= 0 for index in ids):
                faces.append(tuple(int(index) for index in ids))
    return vertices, faces


def _write_mesh_ply(path: Path, height_map: np.ndarray, x_axis: np.ndarray, z_axis: np.ndarray) -> dict:
    path.parent.mkdir(parents=True, exist_ok=True)
    vertices, faces = _mesh_from_height_map(height_map, x_axis, z_axis)
    with path.open("w", encoding="utf-8") as fp:
        fp.write("ply\nformat ascii 1.0\n")
        fp.write(f"element vertex {len(vertices)}\n")
        fp.write("property float x\nproperty float y\nproperty float z\n")
        fp.write(f"element face {len(faces)}\n")
        fp.write("property list uchar int vertex_indices\n")
        fp.write("end_header\n")
        for vertex in vertices:
            fp.write(f"{vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
        for face in faces:
            fp.write(f"4 {face[0]} {face[1]} {face[2]} {face[3]}\n")
    return {"vertices": len(vertices), "faces": len(faces)}


def _write_colored_mesh_ply(
    path: Path,
    meshes: list[tuple[list[tuple[float, float, float]], list[tuple[int, int, int, int]], tuple[int, int, int]]],
) -> dict:
    path.parent.mkdir(parents=True, exist_ok=True)
    all_vertices: list[tuple[float, float, float]] = []
    all_colors: list[tuple[int, int, int]] = []
    all_faces: list[tuple[int, int, int, int]] = []
    for vertices, faces, color in meshes:
        offset = len(all_vertices)
        all_vertices.extend(vertices)
        all_colors.extend([color] * len(vertices))
        all_faces.extend(tuple(index + offset for index in face) for face in faces)

    with path.open("w", encoding="utf-8") as fp:
        fp.write("ply\nformat ascii 1.0\n")
        fp.write(f"element vertex {len(all_vertices)}\n")
        fp.write("property float x\nproperty float y\nproperty float z\n")
        fp.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        fp.write(f"element face {len(all_faces)}\n")
        fp.write("property list uchar int vertex_indices\n")
        fp.write("end_header\n")
        for vertex, color in zip(all_vertices, all_colors):
            fp.write(
                f"{vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f} "
                f"{color[0]} {color[1]} {color[2]}\n"
            )
        for face in all_faces:
            fp.write(f"4 {face[0]} {face[1]} {face[2]} {face[3]}\n")
    return {"vertices": len(all_vertices), "faces": len(all_faces)}


def _reference_height_map(x_axis: np.ndarray, z_axis: np.ndarray) -> np.ndarray:
    x_grid, z_grid = np.meshgrid(x_axis, z_axis)
    return synthetic_facade_y(x_grid, z_grid)


def _write_height_map_png_if_available(path: Path, height_map: np.ndarray, title: str, colorbar_label: str) -> bool:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return False

    relative_mm = (height_map - np.nanmedian(height_map)) * 1000.0
    plt.figure(figsize=(12, 8))
    image = plt.imshow(relative_mm, cmap="coolwarm", origin="lower", aspect="auto")
    plt.colorbar(image, label=colorbar_label)
    plt.title(title)
    plt.xlabel("X grid cells")
    plt.ylabel("Z grid cells")
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()
    return True


def _write_error_png_if_available(path: Path, error_map_m: np.ndarray) -> bool:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return False

    error_mm = error_map_m * 1000.0
    max_abs = float(np.nanmax(np.abs(error_mm))) if np.isfinite(error_mm).any() else 1.0
    max_abs = max(max_abs, 1e-6)
    plt.figure(figsize=(12, 8))
    image = plt.imshow(
        error_mm,
        cmap="coolwarm",
        origin="lower",
        aspect="auto",
        vmin=-max_abs,
        vmax=max_abs,
    )
    plt.colorbar(image, label="reconstruction error vs original (mm)")
    plt.title("Reconstructed - original facade")
    plt.xlabel("X grid cells")
    plt.ylabel("Z grid cells")
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=Path("outputs/gocator_demo/gocator_point_cloud.ply"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/facade_reconstruction"))
    parser.add_argument("--x-resolution", type=float, help="Output grid X resolution in meters. Default: estimate from cloud.")
    parser.add_argument("--z-resolution", type=float, help="Output grid Z resolution in meters. Default: estimate from cloud.")
    parser.add_argument("--fill-iterations", type=int, default=3, help="Neighbor-average iterations for small holes.")
    args = parser.parse_args()

    points = _read_ascii_ply_points(args.input)
    if points.size == 0:
        raise ValueError(f"{args.input} contains no points")

    height_map, counts, x_axis, z_axis = _rasterize_height_map(
        points,
        x_resolution_m=args.x_resolution,
        z_resolution_m=args.z_resolution,
    )
    filled_height_map = _fill_small_holes(height_map, args.fill_iterations)

    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    np.save(out / "reconstructed_height_map.npy", filled_height_map)
    np.save(out / "reconstruction_sample_counts.npy", counts)

    reference_height_map = _reference_height_map(x_axis, z_axis)
    error_map = filled_height_map - reference_height_map
    np.save(out / "reference_height_map.npy", reference_height_map)
    np.save(out / "reconstruction_error_map.npy", error_map)

    mesh_stats = _write_mesh_ply(out / "reconstructed_facade_mesh.ply", filled_height_map, x_axis, z_axis)
    reference_mesh_stats = _write_mesh_ply(out / "original_reference_facade_mesh.ply", reference_height_map, x_axis, z_axis)

    x_span = float(x_axis.max() - x_axis.min()) if len(x_axis) else 0.0
    comparison_offset_x_m = x_span + 0.5
    original_vertices, original_faces = _mesh_from_height_map(reference_height_map, x_axis, z_axis)
    reconstructed_vertices, reconstructed_faces = _mesh_from_height_map(
        filled_height_map,
        x_axis,
        z_axis,
        x_offset_m=comparison_offset_x_m,
    )
    comparison_stats = _write_colored_mesh_ply(
        out / "original_vs_reconstructed_side_by_side.ply",
        [
            (original_vertices, original_faces, (185, 185, 185)),
            (reconstructed_vertices, reconstructed_faces, (20, 210, 235)),
        ],
    )
    height_png_written = _write_height_map_png_if_available(
        out / "reconstructed_height_map.png",
        filled_height_map,
        "Reconstructed facade height map",
        "relative facade depth (mm)",
    )
    error_png_written = _write_error_png_if_available(out / "reconstruction_error_map.png", error_map)

    finite = np.isfinite(height_map)
    filled_finite = np.isfinite(filled_height_map)
    error_finite = np.isfinite(error_map)
    error_mm = error_map[error_finite] * 1000.0
    metrics = {
        "input": str(args.input),
        "input_points": int(len(points)),
        "x_resolution_m": float(x_axis[1] - x_axis[0]) if len(x_axis) > 1 else None,
        "z_resolution_m": float(z_axis[1] - z_axis[0]) if len(z_axis) > 1 else None,
        "grid_shape_z_x": [int(filled_height_map.shape[0]), int(filled_height_map.shape[1])],
        "raw_valid_cells": int(finite.sum()),
        "filled_valid_cells": int(filled_finite.sum()),
        "raw_coverage_ratio": float(finite.mean()),
        "filled_coverage_ratio": float(filled_finite.mean()),
        "depth_min_m": float(np.nanmin(filled_height_map)),
        "depth_max_m": float(np.nanmax(filled_height_map)),
        "depth_peak_to_peak_mm": float((np.nanmax(filled_height_map) - np.nanmin(filled_height_map)) * 1000.0),
        "depth_std_mm": float(np.nanstd(filled_height_map) * 1000.0),
        "reconstruction_error_mean_mm": float(np.nanmean(error_mm)) if error_mm.size else None,
        "reconstruction_error_rms_mm": float(np.sqrt(np.nanmean(error_mm**2))) if error_mm.size else None,
        "reconstruction_error_max_abs_mm": float(np.nanmax(np.abs(error_mm))) if error_mm.size else None,
        "mesh_vertices": mesh_stats["vertices"],
        "mesh_faces": mesh_stats["faces"],
        "reference_mesh_vertices": reference_mesh_stats["vertices"],
        "reference_mesh_faces": reference_mesh_stats["faces"],
        "comparison_mesh_vertices": comparison_stats["vertices"],
        "comparison_mesh_faces": comparison_stats["faces"],
        "comparison_offset_x_m": comparison_offset_x_m,
        "height_map_png_written": height_png_written,
        "error_map_png_written": error_png_written,
    }
    with (out / "reconstruction_metrics.json").open("w", encoding="utf-8") as fp:
        json.dump(metrics, fp, indent=2)

    print(f"input_points: {metrics['input_points']}")
    print(f"grid_shape_z_x: {metrics['grid_shape_z_x']}")
    print(f"resolution_m: x={metrics['x_resolution_m']:.6f}, z={metrics['z_resolution_m']:.6f}")
    print(f"coverage: raw={metrics['raw_coverage_ratio']:.3f}, filled={metrics['filled_coverage_ratio']:.3f}")
    print(f"depth_peak_to_peak_mm: {metrics['depth_peak_to_peak_mm']:.3f}")
    print(f"error_rms_mm: {metrics['reconstruction_error_rms_mm']:.6f}")
    print(f"mesh: {out / 'reconstructed_facade_mesh.ply'}")
    print(f"reference_mesh: {out / 'original_reference_facade_mesh.ply'}")
    print(f"side_by_side: {out / 'original_vs_reconstructed_side_by_side.ply'}")
    print(f"height_map: {out / 'reconstructed_height_map.npy'}")
    print(f"error_map: {out / 'reconstruction_error_map.npy'}")
    print(f"metrics: {out / 'reconstruction_metrics.json'}")
    if height_png_written:
        print(f"png: {out / 'reconstructed_height_map.png'}")
    if error_png_written:
        print(f"error_png: {out / 'reconstruction_error_map.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
