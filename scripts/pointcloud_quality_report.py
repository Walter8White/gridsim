#!/usr/bin/env python3
"""Generate a visual and metric QA report for a Pix4D LAS point cloud."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path

import laspy
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
from scipy.spatial import ConvexHull, cKDTree


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("las_path", type=Path, help="Input LAS file.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Report output directory. Defaults to LAS_DIR/quality_report.",
    )
    parser.add_argument(
        "--session-dir",
        type=Path,
        default=None,
        help="GridCapture session directory. Auto-detected from Pix4D output name when omitted.",
    )
    parser.add_argument(
        "--sample-size",
        type=int,
        default=500_000,
        help="Maximum number of points used for plots and nearest-neighbor metrics.",
    )
    parser.add_argument("--seed", type=int, default=7, help="Random seed for point sampling.")
    return parser.parse_args()


def load_points(las_path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    las = laspy.read(las_path)
    points = np.column_stack((las.x, las.y, las.z)).astype(np.float64)
    colors = None
    if all(hasattr(las, channel) for channel in ("red", "green", "blue")):
        colors = np.column_stack((las.red, las.green, las.blue)).astype(np.float64)
        max_color = np.nanmax(colors) if colors.size else 0
        if max_color > 255:
            colors = colors / 65535.0
        elif max_color > 0:
            colors = colors / 255.0
        colors = np.clip(colors, 0.0, 1.0)
    return points, colors


def sample_indices(count: int, sample_size: int, seed: int) -> np.ndarray:
    if count <= sample_size:
        return np.arange(count)
    rng = np.random.default_rng(seed)
    return np.sort(rng.choice(count, size=sample_size, replace=False))


def fit_plane(points: np.ndarray) -> tuple[np.ndarray, float, np.ndarray, float]:
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    signed_distances = centered @ normal

    # Robust refit: remove the noisiest plane outliers, then solve again.
    abs_distances = np.abs(signed_distances)
    cutoff = np.percentile(abs_distances, 90)
    inliers = points[abs_distances <= cutoff]
    centroid = inliers.mean(axis=0)
    centered = inliers - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    signed_distances = (points - centroid) @ normal
    d = -float(centroid @ normal)
    inlier_ratio = len(inliers) / len(points)
    return normal, d, signed_distances, inlier_ratio


def plane_axes(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    world_vertical = np.array([0.0, 0.0, 1.0])
    vertical = world_vertical - normal * float(world_vertical @ normal)
    if np.linalg.norm(vertical) < 1e-9:
        vertical = np.array([0.0, 1.0, 0.0])
    vertical = vertical / np.linalg.norm(vertical)
    horizontal = np.cross(vertical, normal)
    horizontal = horizontal / np.linalg.norm(horizontal)
    return horizontal, vertical


def projected_coordinates(
    points: np.ndarray,
    origin: np.ndarray,
    horizontal: np.ndarray,
    vertical: np.ndarray,
) -> np.ndarray:
    centered = points - origin
    return np.column_stack((centered @ horizontal, centered @ vertical))


def polygon_area(projected: np.ndarray) -> float:
    if len(projected) < 3:
        return 0.0
    hull = ConvexHull(projected)
    vertices = projected[hull.vertices]
    x = vertices[:, 0]
    y = vertices[:, 1]
    return float(0.5 * np.abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def nearest_neighbor_stats(projected: np.ndarray, sample_size: int, seed: int) -> dict[str, float]:
    idx = sample_indices(len(projected), min(sample_size, 150_000), seed)
    sample = projected[idx]
    tree = cKDTree(sample)
    distances, _ = tree.query(sample, k=2, workers=-1)
    nn = distances[:, 1]
    return {
        "median_mm": float(np.median(nn) * 1000.0),
        "p90_mm": float(np.percentile(nn, 90) * 1000.0),
        "p95_mm": float(np.percentile(nn, 95) * 1000.0),
    }


def residual_stats(signed_distances: np.ndarray) -> dict[str, float]:
    mm = signed_distances * 1000.0
    return {
        "mean_mm": float(np.mean(mm)),
        "median_abs_mm": float(np.median(np.abs(mm))),
        "rms_mm": float(math.sqrt(np.mean(mm**2))),
        "p90_abs_mm": float(np.percentile(np.abs(mm), 90)),
        "p95_abs_mm": float(np.percentile(np.abs(mm), 95)),
        "max_abs_mm": float(np.max(np.abs(mm))),
    }


def read_manifest(session_dir: Path | None) -> dict:
    if session_dir is None:
        return {}
    manifest_path = session_dir / "manifest.jsonl"
    if not manifest_path.exists():
        return {}
    records = []
    with manifest_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                records.append(json.loads(line))
    captures = [record for record in records if record.get("type") == "capture"]
    positions = [float(r["measured_position_mm"]) for r in captures if "measured_position_mm" in r]
    x_offsets = [float(r.get("x_offset_m", 0.0)) for r in captures]
    pass_indices = [int(r.get("pass_index", 0)) for r in captures]
    return {
        "session_dir": str(session_dir),
        "capture_count": len(captures),
        "pass_count": len(set(pass_indices)) if pass_indices else 0,
        "zaber_min_mm": min(positions) if positions else None,
        "zaber_max_mm": max(positions) if positions else None,
        "x_offset_min_m": min(x_offsets) if x_offsets else None,
        "x_offset_max_m": max(x_offsets) if x_offsets else None,
    }


def autodetect_session_dir(las_path: Path) -> Path | None:
    match = re.search(r"(session_\d+T\d+Z(?:_multipass)?)", str(las_path))
    if not match:
        return None
    candidate = Path("captures/iphone_zaber") / match.group(1)
    return candidate if candidate.exists() else None


def read_pose_hints(output_dir: Path) -> dict:
    path = output_dir / "camera_pose_hints.csv"
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    wall_distances = [
        float(row["camera_y_m"])
        for row in rows
        if row.get("camera_y_m") not in (None, "")
    ]
    pose_sources = sorted({row.get("pose_source", "") for row in rows if row.get("pose_source")})
    return {
        "pose_hint_count": len(rows),
        "pose_sources": pose_sources,
        "wall_distance_hint_m": float(np.median(wall_distances)) if wall_distances else None,
    }


def set_equal_aspect_3d(ax, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = (mins + maxs) / 2
    radius = np.max(maxs - mins) / 2
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)


def save_front_view(path: Path, projected: np.ndarray, colors: np.ndarray | None) -> None:
    fig, ax = plt.subplots(figsize=(12, 7), dpi=180)
    if colors is None:
        ax.scatter(projected[:, 0], projected[:, 1], s=0.08, c="#222222", linewidths=0)
    else:
        ax.scatter(projected[:, 0], projected[:, 1], s=0.08, c=colors, linewidths=0)
    ax.set_title("Projected facade view, fitted plane coordinates")
    ax.set_xlabel("Facade horizontal axis [m]")
    ax.set_ylabel("Facade vertical axis [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def save_beauty_front_view(
    path: Path,
    projected: np.ndarray,
    colors: np.ndarray | None,
    metrics: dict,
) -> None:
    fig, ax = plt.subplots(figsize=(14, 10), dpi=220)
    fig.patch.set_facecolor("#0d1117")
    ax.set_facecolor("#0d1117")
    if colors is None:
        ax.scatter(projected[:, 0], projected[:, 1], s=0.18, c="#d0d7de", linewidths=0)
    else:
        ax.scatter(projected[:, 0], projected[:, 1], s=0.18, c=colors, linewidths=0)
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")
    width = metrics.get("projected_width_m")
    height = metrics.get("projected_height_m")
    density = metrics.get("density_points_per_m2")
    rms = metrics.get("plane_residuals", {}).get("rms_mm")
    title = "Pix4D dense point cloud - facade projection"
    subtitle = (
        f"{metrics['point_count']:,} points | "
        f"{width:.2f} m x {height:.2f} m | "
        f"{density/1_000_000:.2f}M pts/m2 | "
        f"plane RMS {rms:.2f} mm"
    ).replace(",", " ")
    ax.text(
        0.02,
        0.98,
        title,
        transform=ax.transAxes,
        ha="left",
        va="top",
        color="white",
        fontsize=18,
        fontweight="bold",
    )
    ax.text(
        0.02,
        0.935,
        subtitle,
        transform=ax.transAxes,
        ha="left",
        va="top",
        color="#c9d1d9",
        fontsize=11,
    )
    fig.tight_layout(pad=0.6)
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)


def save_residual_view(path: Path, projected: np.ndarray, signed_distances: np.ndarray) -> None:
    fig, ax = plt.subplots(figsize=(12, 7), dpi=180)
    residual_mm = signed_distances * 1000.0
    limit = max(5.0, np.percentile(np.abs(residual_mm), 98))
    sc = ax.scatter(
        projected[:, 0],
        projected[:, 1],
        s=0.12,
        c=residual_mm,
        cmap="coolwarm",
        vmin=-limit,
        vmax=limit,
        linewidths=0,
    )
    ax.set_title("Distance to fitted plane [mm]")
    ax.set_xlabel("Facade horizontal axis [m]")
    ax.set_ylabel("Facade vertical axis [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)
    fig.colorbar(sc, ax=ax, label="Signed residual [mm]")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def save_density_view(path: Path, projected: np.ndarray) -> None:
    fig, ax = plt.subplots(figsize=(12, 7), dpi=180)
    hist = ax.hist2d(projected[:, 0], projected[:, 1], bins=180, cmap="viridis")
    ax.set_title("Projected point density")
    ax.set_xlabel("Facade horizontal axis [m]")
    ax.set_ylabel("Facade vertical axis [m]")
    ax.set_aspect("equal", adjustable="box")
    fig.colorbar(hist[3], ax=ax, label="Points per bin")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def save_3d_view(
    path: Path,
    points: np.ndarray,
    colors: np.ndarray | None,
    origin: np.ndarray,
    normal: np.ndarray,
    horizontal: np.ndarray,
    vertical: np.ndarray,
    projected: np.ndarray,
) -> None:
    fig = plt.figure(figsize=(12, 8), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    if colors is None:
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=0.1, c="#222222", alpha=0.45)
    else:
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=0.1, c=colors, alpha=0.45)

    mins = projected.min(axis=0)
    maxs = projected.max(axis=0)
    u = np.linspace(mins[0], maxs[0], 2)
    v = np.linspace(mins[1], maxs[1], 2)
    uu, vv = np.meshgrid(u, v)
    plane = origin + uu[..., None] * horizontal + vv[..., None] * vertical
    ax.plot_surface(
        plane[..., 0],
        plane[..., 1],
        plane[..., 2],
        color="#00a3ff",
        alpha=0.22,
        linewidth=0,
    )
    ax.quiver(
        origin[0],
        origin[1],
        origin[2],
        normal[0],
        normal[1],
        normal[2],
        length=0.25,
        color="#ff2a2a",
        linewidth=2,
    )
    ax.set_title("Point cloud with fitted facade plane")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    set_equal_aspect_3d(ax, points)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def write_html(path: Path, metrics: dict, image_paths: list[Path]) -> None:
    def fmt(value, unit="", digits=3):
        if value is None:
            return "n/a"
        if isinstance(value, int):
            return f"{value:,}".replace(",", " ")
        if isinstance(value, float):
            return f"{value:.{digits}f} {unit}".strip()
        return str(value)

    rows = [
        ("Total points", fmt(metrics["point_count"])),
        ("Captures", fmt(metrics.get("capture_count"))),
        ("Passes", fmt(metrics.get("pass_count"))),
        ("Wall distance hint", fmt(metrics.get("wall_distance_hint_m"), "m")),
        ("Zaber range", f"{fmt(metrics.get('zaber_min_mm'), 'mm', 1)} - {fmt(metrics.get('zaber_max_mm'), 'mm', 1)}"),
        ("Manual horizontal span", f"{fmt(metrics.get('x_offset_min_m'), 'm')} - {fmt(metrics.get('x_offset_max_m'), 'm')}"),
        ("Projected facade area", fmt(metrics["projected_area_m2"], "m2")),
        ("Point density", fmt(metrics["density_points_per_m2"], "pts/m2", 0)),
        ("Median nearest-neighbor spacing", fmt(metrics["nearest_neighbor"]["median_mm"], "mm")),
        ("Plane RMS residual", fmt(metrics["plane_residuals"]["rms_mm"], "mm")),
        ("Plane median abs residual", fmt(metrics["plane_residuals"]["median_abs_mm"], "mm")),
        ("Plane p95 abs residual", fmt(metrics["plane_residuals"]["p95_abs_mm"], "mm")),
    ]
    rows_html = "\n".join(f"<tr><th>{name}</th><td>{value}</td></tr>" for name, value in rows)
    images_html = "\n".join(
        f'<figure><img src="{img.name}" alt="{img.stem}"><figcaption>{img.stem}</figcaption></figure>'
        for img in image_paths
    )
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Pix4D Point Cloud QA Report</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 32px; color: #1f2933; }}
    h1 {{ margin-bottom: 0; }}
    .subtitle {{ color: #667085; margin-top: 6px; }}
    table {{ border-collapse: collapse; margin: 24px 0; min-width: 620px; }}
    th, td {{ border: 1px solid #d0d5dd; padding: 9px 12px; text-align: left; }}
    th {{ background: #f2f4f7; width: 320px; }}
    figure {{ margin: 28px 0; }}
    img {{ max-width: 1100px; width: 100%; border: 1px solid #d0d5dd; }}
    figcaption {{ color: #667085; margin-top: 6px; }}
    code {{ background: #f2f4f7; padding: 2px 4px; }}
  </style>
</head>
<body>
  <h1>Pix4D Point Cloud QA Report</h1>
  <p class="subtitle">Input: <code>{metrics["las_path"]}</code></p>
  <h2>Key Metrics</h2>
  <table>{rows_html}</table>
  <h2>Visual Diagnostics</h2>
  {images_html}
</body>
</html>
"""
    path.write_text(html, encoding="utf-8")


def write_pdf(path: Path, metrics: dict, image_paths: list[Path]) -> None:
    def metric_text(label: str, value: str) -> str:
        return f"{label}: {value}"

    def fmt(value, unit="", digits=2):
        if value is None:
            return "n/a"
        if isinstance(value, int):
            return f"{value:,}".replace(",", " ")
        if isinstance(value, float):
            return f"{value:.{digits}f} {unit}".strip()
        return str(value)

    with PdfPages(path) as pdf:
        fig = plt.figure(figsize=(11.69, 8.27), dpi=160)
        fig.patch.set_facecolor("white")
        ax_title = fig.add_axes([0.05, 0.86, 0.9, 0.1])
        ax_title.axis("off")
        ax_title.text(0, 0.8, "Pix4D Point Cloud QA Report", fontsize=20, fontweight="bold")
        ax_title.text(0, 0.35, metrics["las_path"], fontsize=8, color="#555555")

        ax_img = fig.add_axes([0.05, 0.12, 0.58, 0.70])
        ax_img.imshow(plt.imread(image_paths[0]))
        ax_img.axis("off")

        rows = [
            metric_text("Points", fmt(metrics["point_count"])),
            metric_text("Captures", fmt(metrics.get("capture_count"))),
            metric_text("Passes", fmt(metrics.get("pass_count"))),
            metric_text("Wall distance hint", fmt(metrics.get("wall_distance_hint_m"), "m")),
            metric_text("Projected size", f"{fmt(metrics.get('projected_width_m'), 'm')} x {fmt(metrics.get('projected_height_m'), 'm')}"),
            metric_text("Projected area", fmt(metrics.get("projected_area_m2"), "m2")),
            metric_text("Density", fmt(metrics.get("density_points_per_m2"), "pts/m2", 0)),
            metric_text("Median spacing", fmt(metrics["nearest_neighbor"]["median_mm"], "mm")),
            metric_text("Plane RMS", fmt(metrics["plane_residuals"]["rms_mm"], "mm")),
            metric_text("Plane median abs", fmt(metrics["plane_residuals"]["median_abs_mm"], "mm")),
            metric_text("Plane p95 abs", fmt(metrics["plane_residuals"]["p95_abs_mm"], "mm")),
        ]
        ax_metrics = fig.add_axes([0.67, 0.18, 0.30, 0.62])
        ax_metrics.axis("off")
        ax_metrics.text(0, 1.0, "Key metrics", fontsize=14, fontweight="bold", va="top")
        for index, row in enumerate(rows):
            ax_metrics.text(0, 0.92 - index * 0.075, row, fontsize=10, va="top")
        ax_metrics.text(
            0,
            0.02,
            "Note: wall distance is a pose hint used during processing, not an independent laser/station measurement.",
            fontsize=8,
            color="#666666",
            wrap=True,
        )
        pdf.savefig(fig, bbox_inches="tight")
        plt.close(fig)

        fig, axs = plt.subplots(1, 2, figsize=(11.69, 8.27), dpi=160)
        for ax, image_path, title in zip(
            axs,
            image_paths[1:3],
            ["Distance to fitted plane", "Projected point density"],
        ):
            ax.imshow(plt.imread(image_path))
            ax.set_title(title, fontsize=12)
            ax.axis("off")
        fig.tight_layout()
        pdf.savefig(fig, bbox_inches="tight")
        plt.close(fig)


def main() -> int:
    args = parse_args()
    las_path = args.las_path
    output_dir = args.output_dir or las_path.parent / "quality_report"
    output_dir.mkdir(parents=True, exist_ok=True)

    points, colors = load_points(las_path)
    idx = sample_indices(len(points), args.sample_size, args.seed)
    sample = points[idx]
    sample_colors = colors[idx] if colors is not None else None

    normal, d, signed_sample, inlier_ratio = fit_plane(sample)
    origin = -d * normal
    signed_all = (points - origin) @ normal
    horizontal, vertical = plane_axes(normal)
    projected_all = projected_coordinates(points, origin, horizontal, vertical)
    projected_all = projected_all - projected_all.min(axis=0)
    projected_sample = projected_all[idx]

    area = polygon_area(projected_sample)
    metrics = {
        "las_path": str(las_path),
        "point_count": int(len(points)),
        "bounds_min": points.min(axis=0).tolist(),
        "bounds_max": points.max(axis=0).tolist(),
        "fitted_plane": {
            "normal": normal.tolist(),
            "d": d,
            "inlier_ratio_initial_refit": inlier_ratio,
        },
        "projected_width_m": float(projected_sample[:, 0].max() - projected_sample[:, 0].min()),
        "projected_height_m": float(projected_sample[:, 1].max() - projected_sample[:, 1].min()),
        "projected_area_m2": area,
        "density_points_per_m2": float(len(points) / area) if area > 0 else None,
        "plane_residuals": residual_stats(signed_all),
        "nearest_neighbor": nearest_neighbor_stats(projected_all, args.sample_size, args.seed),
    }

    session_dir = args.session_dir or autodetect_session_dir(las_path)
    metrics.update(read_manifest(session_dir))
    metrics.update(read_pose_hints(las_path.parents[1]))

    beauty = output_dir / "00_pointcloud_front_beauty.png"
    front = output_dir / "01_facade_projection_rgb.png"
    residual = output_dir / "02_plane_residuals_mm.png"
    density = output_dir / "03_projected_density.png"
    view_3d = output_dir / "04_cloud_with_fitted_plane.png"
    save_beauty_front_view(beauty, projected_sample, sample_colors, metrics)
    save_front_view(front, projected_sample, sample_colors)
    save_residual_view(residual, projected_sample, signed_sample)
    save_density_view(density, projected_sample)
    save_3d_view(view_3d, sample, sample_colors, origin, normal, horizontal, vertical, projected_sample)

    metrics_path = output_dir / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    report_path = output_dir / "report.html"
    pdf_path = output_dir / "report.pdf"
    write_html(report_path, metrics, [beauty, front, residual, density, view_3d])
    write_pdf(pdf_path, metrics, [beauty, residual, density])

    print(f"[report] input: {las_path}")
    print(f"[report] points: {len(points):,}")
    print(f"[report] area: {metrics['projected_area_m2']:.3f} m2")
    print(f"[report] density: {metrics['density_points_per_m2']:.0f} pts/m2")
    print(f"[report] plane RMS: {metrics['plane_residuals']['rms_mm']:.2f} mm")
    print(f"[report] output: {report_path}")
    print(f"[report] pdf: {pdf_path}")
    print(f"[report] beauty png: {beauty}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
