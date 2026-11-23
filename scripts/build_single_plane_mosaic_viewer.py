#!/usr/bin/env python3
"""Build a navigable 3D viewer with all captures composited onto one wall plane."""

from __future__ import annotations

import argparse
import json
import math
import urllib.request
from pathlib import Path

import laspy
import numpy as np
from PIL import Image


THREE_VERSION = "0.160.0"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--wall-distance-m", type=float, default=0.35)
    parser.add_argument("--zaber-height-offset-mm", type=float, default=150.0)
    parser.add_argument("--camera-35mm-equivalent-mm", type=float, default=24.0)
    parser.add_argument("--px-per-m", type=int, default=2400)
    parser.add_argument("--blend", choices=("winner", "average"), default="winner")
    parser.add_argument("--las-path", type=Path, default=None)
    parser.add_argument("--point-sample-size", type=int, default=0)
    parser.add_argument("--seed", type=int, default=7)
    return parser.parse_args()


def load_records(session_dir: Path) -> list[dict]:
    records = []
    with (session_dir / "manifest.jsonl").open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("type") == "capture":
                records.append(record)
    records.sort(key=lambda item: int(item.get("global_capture_index", 0)))
    return records


def image_filename(record: dict) -> str:
    result = record.get("iphone_result") or {}
    return result.get("photo_filename") or f"{record['capture_id']}.jpg"


def read_camera(session_dir: Path, capture_id: str) -> dict:
    return json.loads((session_dir / "metadata" / f"{capture_id}.json").read_text()).get("camera", {})


def fov_from_camera(camera: dict, equivalent_35mm_mm: float) -> tuple[float, float]:
    focal_mm = float(camera["focal_length"])
    image_width = float(camera["image_width"])
    image_height = float(camera["image_height"])
    sensor_width_mm = 36.0 * focal_mm / equivalent_35mm_mm
    sensor_height_mm = sensor_width_mm * image_height / image_width
    return (
        2.0 * math.atan(sensor_width_mm / (2.0 * focal_mm)),
        2.0 * math.atan(sensor_height_mm / (2.0 * focal_mm)),
    )


def feather_mask(width: int, height: int) -> np.ndarray:
    x = np.linspace(-1.0, 1.0, width, dtype=np.float32)
    y = np.linspace(-1.0, 1.0, height, dtype=np.float32)
    xx, yy = np.meshgrid(x, y)
    edge = np.maximum(np.abs(xx), np.abs(yy))
    return np.clip((1.0 - edge) / 0.22, 0.0, 1.0).astype(np.float32)


def build_mosaic(
    session_dir: Path,
    records: list[dict],
    wall_distance_m: float,
    zaber_height_offset_mm: float,
    equivalent_35mm_mm: float,
    px_per_m: int,
    blend: str,
    output_path: Path,
) -> dict:
    camera = read_camera(session_dir, records[0]["capture_id"])
    hfov, vfov = fov_from_camera(camera, equivalent_35mm_mm)
    frame_w_m = 2.0 * wall_distance_m * math.tan(hfov / 2.0)
    frame_h_m = 2.0 * wall_distance_m * math.tan(vfov / 2.0)

    centers = []
    for record in records:
        x = -float(record.get("x_offset_m", 0.0))
        z = (float(record["measured_position_mm"]) + zaber_height_offset_mm) / 1000.0
        centers.append((x, z))
    centers_np = np.array(centers, dtype=np.float64)

    min_x = float(centers_np[:, 0].min() - frame_w_m / 2.0)
    max_x = float(centers_np[:, 0].max() + frame_w_m / 2.0)
    min_z = float(centers_np[:, 1].min() - frame_h_m / 2.0)
    max_z = float(centers_np[:, 1].max() + frame_h_m / 2.0)
    wall_w_m = max_x - min_x
    wall_h_m = max_z - min_z
    width_px = int(math.ceil(wall_w_m * px_per_m))
    height_px = int(math.ceil(wall_h_m * px_per_m))
    tile_w = max(2, int(round(frame_w_m * px_per_m)))
    tile_h = max(2, int(round(frame_h_m * px_per_m)))

    if blend == "average":
        accum = np.zeros((height_px, width_px, 3), dtype=np.float32)
        weights = np.zeros((height_px, width_px), dtype=np.float32)
    else:
        mosaic = np.zeros((height_px, width_px, 3), dtype=np.uint8)
        best = np.full((height_px, width_px), -1.0, dtype=np.float32)

    mask = feather_mask(tile_w, tile_h)
    for index, record in enumerate(records):
        filename = image_filename(record)
        with Image.open(session_dir / "images" / filename) as image:
            image = image.convert("RGB")
            image = image.resize((tile_w, tile_h), Image.Resampling.LANCZOS)
        tile = np.asarray(image, dtype=np.float32 if blend == "average" else np.uint8)
        cx_m, cz_m = centers[index]
        cx_px = int(round((cx_m - min_x) * px_per_m))
        cy_px = int(round((max_z - cz_m) * px_per_m))
        x0 = max(0, cx_px - tile_w // 2)
        y0 = max(0, cy_px - tile_h // 2)
        x1 = min(width_px, x0 + tile_w)
        y1 = min(height_px, y0 + tile_h)
        tx0 = max(0, tile_w // 2 - cx_px)
        ty0 = max(0, tile_h // 2 - cy_px)
        tx1 = tx0 + (x1 - x0)
        ty1 = ty0 + (y1 - y0)
        local_mask = mask[ty0:ty1, tx0:tx1]
        if blend == "average":
            accum[y0:y1, x0:x1] += tile[ty0:ty1, tx0:tx1] * local_mask[..., None]
            weights[y0:y1, x0:x1] += local_mask
        else:
            region = best[y0:y1, x0:x1]
            keep = local_mask > region
            if np.any(keep):
                mosaic_region = mosaic[y0:y1, x0:x1]
                mosaic_region[keep] = tile[ty0:ty1, tx0:tx1][keep]
                region[keep] = local_mask[keep]

    if blend == "average":
        mosaic = np.zeros_like(accum, dtype=np.uint8)
        valid = weights > 0
        mosaic[valid] = np.clip(accum[valid] / weights[valid, None], 0, 255).astype(np.uint8)
        mosaic[~valid] = np.array([16, 20, 24], dtype=np.uint8)
    else:
        mosaic[best < 0] = np.array([16, 20, 24], dtype=np.uint8)

    Image.fromarray(mosaic, "RGB").save(output_path, quality=92, optimize=True)
    return {
        "session": session_dir.name,
        "captureCount": len(records),
        "wallDistanceM": wall_distance_m,
        "frameWidthM": frame_w_m,
        "frameHeightM": frame_h_m,
        "wallWidthM": wall_w_m,
        "wallHeightM": wall_h_m,
        "pxPerM": px_per_m,
        "textureWidthPx": width_px,
        "textureHeightPx": height_px,
        "blend": blend,
        "xMinM": min_x,
        "xMaxM": max_x,
        "zMinM": min_z,
        "zMaxM": max_z,
    }


def ensure_three(output_dir: Path) -> None:
    three_path = output_dir / "three.module.js"
    controls_path = output_dir / "OrbitControls.js"
    if not three_path.exists():
        with urllib.request.urlopen(f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/build/three.module.js", timeout=20) as response:
            three_path.write_bytes(response.read())
    if not controls_path.exists():
        with urllib.request.urlopen(f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/examples/jsm/controls/OrbitControls.js", timeout=20) as response:
            text = response.read().decode("utf-8")
        controls_path.write_text(text.replace("from 'three';", "from './three.module.js';"), encoding="utf-8")


def fit_plane(points: np.ndarray) -> tuple[np.ndarray, float]:
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    distances = centered @ normal
    cutoff = np.percentile(np.abs(distances), 90)
    inliers = points[np.abs(distances) <= cutoff]
    centroid = inliers.mean(axis=0)
    centered = inliers - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    return normal, -float(centroid @ normal)


def plane_axes(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    world_vertical = np.array([0.0, 0.0, 1.0])
    vertical = world_vertical - normal * float(world_vertical @ normal)
    if np.linalg.norm(vertical) < 1e-9:
        vertical = np.array([0.0, 1.0, 0.0])
    vertical /= np.linalg.norm(vertical)
    horizontal = np.cross(vertical, normal)
    horizontal /= np.linalg.norm(horizontal)
    return horizontal, vertical


def write_pointcloud_data(las_path: Path, output_path: Path, meta: dict, sample_size: int, seed: int) -> dict:
    las = laspy.read(las_path)
    points = np.column_stack((las.x, las.y, las.z)).astype(np.float64)
    colors = None
    if all(hasattr(las, channel) for channel in ("red", "green", "blue")):
        colors = np.column_stack((las.red, las.green, las.blue)).astype(np.float64)
        max_color = float(np.nanmax(colors)) if colors.size else 0.0
        colors = colors / 65535.0 if max_color > 255.0 else colors / 255.0
        colors = np.clip(colors, 0.0, 1.0)

    if sample_size > 0 and len(points) > sample_size:
        rng = np.random.default_rng(seed)
        idx = np.sort(rng.choice(len(points), size=sample_size, replace=False))
        sample = points[idx]
        sample_colors = colors[idx] if colors is not None else None
    else:
        sample = points
        sample_colors = colors

    normal, d = fit_plane(sample)
    origin = -d * normal
    horizontal, vertical = plane_axes(normal)
    centered = sample - origin
    proj_x = centered @ horizontal
    proj_z = centered @ vertical
    depth = centered @ normal
    proj_x -= float(proj_x.min())
    proj_z -= float(proj_z.min())
    cloud_w = float(proj_x.max())
    cloud_h = float(proj_z.max())
    proj_x += (float(meta["wallWidthM"]) - cloud_w) / 2.0
    proj_z += (float(meta["wallHeightM"]) - cloud_h) / 2.0

    if sample_colors is None:
        rgb = np.tile(np.array([[0.95, 0.95, 0.95]], dtype=np.float32), (len(sample), 1))
    else:
        rgb = sample_colors.astype(np.float32)

    payload = {
        "pointCountTotal": int(len(points)),
        "pointCountDisplayed": int(len(sample)),
        "projectedWidthM": cloud_w,
        "projectedHeightM": cloud_h,
        "depthSigmaMm": float(np.std(depth * 1000.0)),
        "positions": np.column_stack((proj_x, depth + 0.035, proj_z)).astype(np.float32).round(5).tolist(),
        "colors": (rgb * 255.0).astype(np.uint8).tolist(),
    }
    output_path.write_text("window.POINTCLOUD_DATA = " + json.dumps(payload) + ";\n", encoding="utf-8")
    return {key: payload[key] for key in ("pointCountTotal", "pointCountDisplayed", "projectedWidthM", "projectedHeightM", "depthSigmaMm")}


def write_html(output_dir: Path, meta: dict, cloud_meta: dict | None) -> None:
    cloud_line = ""
    cloud_controls = ""
    cloud_loader = ""
    cloud_script = ""
    if cloud_meta is not None:
        cloud_line = (
            f"<br>pointcloud: <code>{cloud_meta['pointCountDisplayed']:,} / "
            f"{cloud_meta['pointCountTotal']:,} pts</code>"
        )
        cloud_loader = "<script src=\"pointcloud_data.js\"></script>"
        cloud_controls = """
      <label>Cloud opacity</label><input id="cloudOpacity" type="range" min="0" max="1" step="0.05" value="0.85">
      <label>Point size</label><input id="pointSize" type="range" min="0.5" max="4" step="0.25" value="1">
      <label>Cloud offset</label><input id="cloudOffset" type="range" min="0" max="0.15" step="0.005" value="0.035">
"""
        cloud_script = """
    const cloudPositions = new Float32Array(window.POINTCLOUD_DATA.positions.flat());
    const cloudColors = new Float32Array(window.POINTCLOUD_DATA.colors.flat().map(v => v / 255));
    const cloudGeometry = new THREE.BufferGeometry();
    cloudGeometry.setAttribute('position', new THREE.BufferAttribute(cloudPositions, 3));
    cloudGeometry.setAttribute('color', new THREE.BufferAttribute(cloudColors, 3));
    const cloudMaterial = new THREE.PointsMaterial({ size: 1, sizeAttenuation: false, vertexColors: true, transparent: true, opacity: 0.85 });
    const pointCloud = new THREE.Points(cloudGeometry, cloudMaterial);
    pointCloud.userData.offset = 0.035;
    scene.add(pointCloud);

    document.getElementById('cloudOpacity').addEventListener('input', event => cloudMaterial.opacity = Number(event.target.value));
    document.getElementById('pointSize').addEventListener('input', event => cloudMaterial.size = Number(event.target.value));
    document.getElementById('cloudOffset').addEventListener('input', event => {
      const desired = Number(event.target.value);
      const delta = desired - pointCloud.userData.offset;
      const arr = cloudGeometry.attributes.position.array;
      for (let i = 1; i < arr.length; i += 3) arr[i] += delta;
      pointCloud.userData.offset = desired;
      cloudGeometry.attributes.position.needsUpdate = true;
    });
"""
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Single Plane Mosaic Viewer</title>
  <style>
    html, body {{ margin: 0; height: 100%; overflow: hidden; background: #0d1117; color: #e6edf3; font-family: Arial, sans-serif; }}
    #app {{ width: 100%; height: 100%; }}
    #hud {{ position: fixed; left: 18px; top: 18px; width: 500px; padding: 14px 16px; background: rgba(13,17,23,0.86); border: 1px solid rgba(230,237,243,0.18); border-radius: 10px; font-size: 15px; line-height: 1.45; }}
    #hud code {{ color: #79c0ff; }}
    #controls {{ margin-top: 12px; display: grid; grid-template-columns: 120px 1fr; gap: 8px 12px; align-items: center; }}
    input {{ width: 100%; }}
  </style>
</head>
<body>
  <div id="app"></div>
  <div id="hud">
    <b>Single plane mosaic viewer</b><br>
    Session: <code>{meta["session"]}</code><br>
    captures: <code>{meta["captureCount"]}</code><br>
    wall distance: <code>{meta["wallDistanceM"]:.3f} m</code><br>
    frame footprint: <code>{meta["frameWidthM"]:.3f} m x {meta["frameHeightM"]:.3f} m</code><br>
    wall texture: <code>{meta["wallWidthM"]:.3f} m x {meta["wallHeightM"]:.3f} m</code><br>
    blend mode: <code>{meta["blend"]}</code>{cloud_line}
    <div id="controls">
      <label>Opacity</label><input id="opacity" type="range" min="0.05" max="1" step="0.05" value="1">
      <label>Grid</label><input id="gridToggle" type="checkbox" checked>
      {cloud_controls}
    </div>
  </div>
  <script>window.MOSAIC_META = {json.dumps(meta)};</script>
  {cloud_loader}
  <script type="module">
    import * as THREE from './three.module.js';
    import {{ OrbitControls }} from './OrbitControls.js';

    const meta = window.MOSAIC_META;
    const app = document.getElementById('app');
    const renderer = new THREE.WebGLRenderer({{ antialias: true }});
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setClearColor(0x0d1117, 1);
    app.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 100);
    camera.position.set(meta.wallWidthM / 2, -2.2, meta.wallHeightM / 2);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(meta.wallWidthM / 2, 0, meta.wallHeightM / 2);
    controls.enableDamping = true;

    const texture = new THREE.TextureLoader().load('single_plane_mosaic.jpg');
    texture.colorSpace = THREE.SRGBColorSpace;
    const material = new THREE.MeshBasicMaterial({{ map: texture, transparent: true, opacity: 1, side: THREE.DoubleSide }});
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(meta.wallWidthM, meta.wallHeightM), material);
    mesh.rotation.x = Math.PI / 2;
    mesh.position.set(meta.wallWidthM / 2, 0, meta.wallHeightM / 2);
    scene.add(mesh);

    const grid = new THREE.GridHelper(Math.max(meta.wallWidthM, meta.wallHeightM), 12, 0x30363d, 0x30363d);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(meta.wallWidthM / 2, -0.01, meta.wallHeightM / 2);
    scene.add(grid);

    {cloud_script}

    document.getElementById('opacity').addEventListener('input', event => material.opacity = Number(event.target.value));
    document.getElementById('gridToggle').addEventListener('change', event => grid.visible = event.target.checked);

    window.addEventListener('resize', () => {{
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    }});

    function animate() {{
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    }}
    animate();
  </script>
</body>
</html>
"""
    (output_dir / "viewer.html").write_text(html, encoding="utf-8")


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir or args.session_dir / "single_plane_mosaic_viewer"
    output_dir.mkdir(parents=True, exist_ok=True)
    records = load_records(args.session_dir)
    meta = build_mosaic(
        args.session_dir,
        records,
        args.wall_distance_m,
        args.zaber_height_offset_mm,
        args.camera_35mm_equivalent_mm,
        args.px_per_m,
        args.blend,
        output_dir / "single_plane_mosaic.jpg",
    )
    ensure_three(output_dir)
    cloud_meta = None
    if args.las_path is not None:
        cloud_meta = write_pointcloud_data(
            args.las_path,
            output_dir / "pointcloud_data.js",
            meta,
            args.point_sample_size,
            args.seed,
        )
        meta["pointcloud"] = cloud_meta
    (output_dir / "viewer_data.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    write_html(output_dir, meta, cloud_meta)
    print(f"[single-plane] output: {output_dir / 'viewer.html'}")
    print(f"[single-plane] texture: {meta['textureWidthPx']}x{meta['textureHeightPx']} px")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
