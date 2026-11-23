#!/usr/bin/env python3
"""Build an interactive wall-image/point-cloud coverage viewer."""

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
    parser.add_argument("--las-path", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--wall-distance-m", type=float, required=True)
    parser.add_argument("--zaber-height-offset-mm", type=float, default=150.0)
    parser.add_argument("--camera-35mm-equivalent-mm", type=float, default=24.0)
    parser.add_argument("--texture-px-per-m", type=int, default=1700)
    parser.add_argument("--point-sample-size", type=int, default=180_000)
    parser.add_argument("--seed", type=int, default=7)
    return parser.parse_args()


def read_capture_records(session_dir: Path) -> list[dict]:
    manifest_path = session_dir / "manifest.jsonl"
    records = []
    with manifest_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("type") == "capture":
                records.append(record)
    records.sort(key=lambda item: int(item.get("global_capture_index", 0)))
    return records


def read_camera_metadata(session_dir: Path, capture_id: str) -> dict:
    path = session_dir / "metadata" / f"{capture_id}.json"
    return json.loads(path.read_text(encoding="utf-8")).get("camera", {})


def fov_from_metadata(camera: dict, equivalent_35mm_mm: float) -> tuple[float, float]:
    focal_mm = float(camera["focal_length"])
    image_width = float(camera["image_width"])
    image_height = float(camera["image_height"])
    sensor_width_mm = 36.0 * focal_mm / equivalent_35mm_mm
    sensor_height_mm = sensor_width_mm * image_height / image_width
    hfov = 2.0 * math.atan(sensor_width_mm / (2.0 * focal_mm))
    vfov = 2.0 * math.atan(sensor_height_mm / (2.0 * focal_mm))
    return hfov, vfov


def image_path_for_record(session_dir: Path, record: dict) -> Path:
    result = record.get("iphone_result", {})
    filename = result.get("photo_filename") or f"{record['capture_id']}.jpg"
    return session_dir / "images" / filename


def make_feather_mask(width: int, height: int) -> np.ndarray:
    x = np.linspace(-1.0, 1.0, width, dtype=np.float32)
    y = np.linspace(-1.0, 1.0, height, dtype=np.float32)
    xx, yy = np.meshgrid(x, y)
    edge = np.maximum(np.abs(xx), np.abs(yy))
    mask = np.clip((1.0 - edge) / 0.18, 0.0, 1.0)
    return np.maximum(mask, 0.12).astype(np.float32)


def build_wall_texture(
    session_dir: Path,
    records: list[dict],
    wall_distance_m: float,
    zaber_height_offset_mm: float,
    equivalent_35mm_mm: float,
    px_per_m: int,
    output_path: Path,
) -> dict:
    first_camera = read_camera_metadata(session_dir, records[0]["capture_id"])
    hfov, vfov = fov_from_metadata(first_camera, equivalent_35mm_mm)
    footprint_w_m = 2.0 * wall_distance_m * math.tan(hfov / 2.0)
    footprint_h_m = 2.0 * wall_distance_m * math.tan(vfov / 2.0)

    centers = []
    for record in records:
        # User convention: first pass is top-right, subsequent passes move left.
        x_m = -float(record.get("x_offset_m", 0.0))
        z_m = float(record["measured_position_mm"] + zaber_height_offset_mm) / 1000.0
        centers.append((x_m, z_m))
    centers_array = np.array(centers, dtype=np.float64)

    min_x = float(np.min(centers_array[:, 0]) - footprint_w_m / 2.0)
    max_x = float(np.max(centers_array[:, 0]) + footprint_w_m / 2.0)
    min_z = float(np.min(centers_array[:, 1]) - footprint_h_m / 2.0)
    max_z = float(np.max(centers_array[:, 1]) + footprint_h_m / 2.0)
    width_m = max_x - min_x
    height_m = max_z - min_z
    width_px = int(math.ceil(width_m * px_per_m))
    height_px = int(math.ceil(height_m * px_per_m))

    accum = np.zeros((height_px, width_px, 3), dtype=np.float32)
    weights = np.zeros((height_px, width_px), dtype=np.float32)
    tile_w = max(2, int(round(footprint_w_m * px_per_m)))
    tile_h = max(2, int(round(footprint_h_m * px_per_m)))
    feather = make_feather_mask(tile_w, tile_h)

    for index, record in enumerate(records, start=1):
        path = image_path_for_record(session_dir, record)
        with Image.open(path) as image:
            # The iPhone files already store landscape pixels for this capture.
            # Applying EXIF orientation here rotates the mosaic tiles and hurts
            # the metric wall layout.
            image = image.convert("RGB")
            image = image.resize((tile_w, tile_h), Image.Resampling.LANCZOS)
        tile = np.asarray(image, dtype=np.float32)
        cx_m, cz_m = centers[index - 1]
        cx_px = int(round((cx_m - min_x) * px_per_m))
        cy_px = int(round((max_z - cz_m) * px_per_m))
        x0 = max(0, cx_px - tile_w // 2)
        y0 = max(0, cy_px - tile_h // 2)
        x1 = min(width_px, x0 + tile_w)
        y1 = min(height_px, y0 + tile_h)
        tile_x0 = max(0, tile_w // 2 - cx_px)
        tile_y0 = max(0, tile_h // 2 - cy_px)
        tile_x1 = tile_x0 + (x1 - x0)
        tile_y1 = tile_y0 + (y1 - y0)
        w = feather[tile_y0:tile_y1, tile_x0:tile_x1]
        accum[y0:y1, x0:x1] += tile[tile_y0:tile_y1, tile_x0:tile_x1] * w[..., None]
        weights[y0:y1, x0:x1] += w

    texture = np.zeros_like(accum, dtype=np.uint8)
    valid = weights > 0
    texture[valid] = np.clip(accum[valid] / weights[valid, None], 0, 255).astype(np.uint8)
    texture[~valid] = np.array([18, 22, 28], dtype=np.uint8)
    Image.fromarray(texture, "RGB").save(output_path, quality=94)

    return {
        "capture_count": len(records),
        "wall_distance_m": wall_distance_m,
        "camera_horizontal_fov_deg": math.degrees(hfov),
        "camera_vertical_fov_deg": math.degrees(vfov),
        "single_frame_wall_width_m": footprint_w_m,
        "single_frame_wall_height_m": footprint_h_m,
        "wall_min_x_m": min_x,
        "wall_max_x_m": max_x,
        "wall_min_z_m": min_z,
        "wall_max_z_m": max_z,
        "wall_width_m": width_m,
        "wall_height_m": height_m,
        "wall_area_m2": width_m * height_m,
        "texture_width_px": width_px,
        "texture_height_px": height_px,
        "texture_px_per_m": px_per_m,
        "zaber_min_m": float(np.min(centers_array[:, 1])),
        "zaber_max_m": float(np.max(centers_array[:, 1])),
        "x_offset_span_m": float(np.max(centers_array[:, 0]) - np.min(centers_array[:, 0])),
    }


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
    d = -float(centroid @ normal)
    return normal, d


def plane_axes(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    world_vertical = np.array([0.0, 0.0, 1.0])
    vertical = world_vertical - normal * float(world_vertical @ normal)
    if np.linalg.norm(vertical) < 1e-9:
        vertical = np.array([0.0, 1.0, 0.0])
    vertical /= np.linalg.norm(vertical)
    horizontal = np.cross(vertical, normal)
    horizontal /= np.linalg.norm(horizontal)
    return horizontal, vertical


def projected_pointcloud(
    las_path: Path,
    sample_size: int,
    seed: int,
    wall_meta: dict,
    output_path: Path,
) -> dict:
    las = laspy.read(las_path)
    points = np.column_stack((las.x, las.y, las.z)).astype(np.float64)
    colors = None
    if all(hasattr(las, channel) for channel in ("red", "green", "blue")):
        colors = np.column_stack((las.red, las.green, las.blue)).astype(np.float64)
        max_color = float(np.nanmax(colors)) if colors.size else 0.0
        colors = colors / 65535.0 if max_color > 255.0 else colors / 255.0
        colors = np.clip(colors, 0.0, 1.0)

    rng = np.random.default_rng(seed)
    if sample_size > 0 and len(points) > sample_size:
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

    proj_x -= float(np.min(proj_x))
    proj_z -= float(np.min(proj_z))
    cloud_width = float(np.max(proj_x))
    cloud_height = float(np.max(proj_z))

    wall_w = float(wall_meta["wall_width_m"])
    wall_h = float(wall_meta["wall_height_m"])
    proj_x += (wall_w - cloud_width) / 2.0
    proj_z += (wall_h - cloud_height) / 2.0

    if sample_colors is None:
        rgb = np.tile(np.array([[0.1, 0.65, 1.0]], dtype=np.float32), (len(sample), 1))
    else:
        rgb = sample_colors.astype(np.float32)

    depth_mm = (depth * 1000.0).astype(np.float32)
    hist_counts, hist_edges = np.histogram(depth_mm, bins=100)
    payload = {
        "pointCountTotal": int(len(points)),
        "pointCountDisplayed": int(len(sample)),
        "projectedWidthM": cloud_width,
        "projectedHeightM": cloud_height,
        "projectedAreaBBoxM2": cloud_width * cloud_height,
        "depthMeanMm": float(np.mean(depth_mm)),
        "depthSigmaMm": float(np.std(depth_mm)),
        "depthP05Mm": float(np.percentile(depth_mm, 5)),
        "depthP95Mm": float(np.percentile(depth_mm, 95)),
        "depthHistCounts": hist_counts.astype(int).tolist(),
        "depthHistEdgesMm": hist_edges.astype(float).round(4).tolist(),
        "positions": np.column_stack((proj_x, depth + 0.035, proj_z)).astype(np.float32).round(5).tolist(),
        "colors": (rgb * 255.0).astype(np.uint8).tolist(),
    }
    output_path.write_text("window.COVERAGE_POINT_DATA = " + json.dumps(payload) + ";\n", encoding="utf-8")
    return {
        key: payload[key]
        for key in (
            "pointCountTotal",
            "pointCountDisplayed",
            "projectedWidthM",
            "projectedHeightM",
            "projectedAreaBBoxM2",
            "depthMeanMm",
            "depthSigmaMm",
            "depthP05Mm",
            "depthP95Mm",
        )
    }


def ensure_three_vendor(output_dir: Path) -> None:
    three_path = output_dir / "three.module.js"
    controls_path = output_dir / "OrbitControls.js"
    if not three_path.exists():
        url = f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/build/three.module.js"
        with urllib.request.urlopen(url, timeout=20) as response:
            three_path.write_bytes(response.read())
    if not controls_path.exists():
        url = f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/examples/jsm/controls/OrbitControls.js"
        with urllib.request.urlopen(url, timeout=20) as response:
            text = response.read().decode("utf-8")
        text = text.replace("from 'three';", "from './three.module.js';")
        controls_path.write_text(text, encoding="utf-8")


def write_html(path: Path, wall_meta: dict, cloud_meta: dict) -> None:
    meta_json = json.dumps({"wall": wall_meta, "cloud": cloud_meta}, indent=2)
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Wall Coverage Viewer</title>
  <style>
    html, body {{ margin: 0; height: 100%; overflow: hidden; background: #0d1117; color: #e6edf3; font-family: Arial, sans-serif; }}
    #app {{ width: 100%; height: 100%; }}
    #hud {{ position: fixed; left: 18px; top: 18px; width: 680px; max-height: calc(100vh - 42px); overflow: auto; padding: 18px 20px; background: rgba(13,17,23,0.88); border: 1px solid rgba(230,237,243,0.2); border-radius: 10px; font-size: 18px; line-height: 1.48; box-shadow: 0 12px 34px rgba(0,0,0,0.3); }}
    #hud b {{ color: white; }}
    #hud code {{ color: #79c0ff; }}
    #status {{ font-size: 16px; margin-top: 2px; color: #c9d1d9; }}
    #controls {{ margin-top: 14px; display: grid; grid-template-columns: 170px 1fr; gap: 10px 14px; align-items: center; }}
    input {{ width: 100%; transform: scale(1.05); transform-origin: left center; }}
    .tabs {{ display: flex; gap: 8px; margin: 14px 0 12px; }}
    .tab {{ border: 1px solid rgba(230,237,243,0.22); background: rgba(255,255,255,0.04); color: #c9d1d9; border-radius: 8px; padding: 8px 13px; cursor: pointer; font-size: 16px; }}
    .tab.active {{ background: #238636; color: white; border-color: #2ea043; }}
    .panel {{ display: none; }}
    .panel.active {{ display: block; }}
    .row {{ display: flex; justify-content: space-between; gap: 20px; border-bottom: 1px solid rgba(230,237,243,0.1); padding: 7px 0; }}
    .row span:last-child {{ color: #79c0ff; text-align: right; }}
    canvas.chart {{ width: 100%; height: 420px; display: block; background: rgba(255,255,255,0.03); border: 1px solid rgba(230,237,243,0.14); border-radius: 8px; }}
    #axisInset {{ position: fixed; left: 22px; bottom: 22px; width: 190px; height: 190px; pointer-events: none; background: rgba(13,17,23,0.64); border: 1px solid rgba(230,237,243,0.14); border-radius: 10px; }}
    #error {{ display: none; margin-top: 12px; padding: 10px; border: 1px solid #f85149; border-radius: 8px; color: #ffdcd7; background: rgba(248,81,73,0.12); white-space: pre-wrap; }}
  </style>
</head>
<body>
  <div id="app"></div>
  <div id="axisInset"></div>
  <div id="hud">
    <b>Wall coverage viewer</b><br>
    <div id="status">Loading 3D scene...</div>
    <div class="tabs">
      <button class="tab active" data-panel="viewPanel">View</button>
      <button class="tab" data-panel="dataPanel">Data</button>
    </div>
    <div id="viewPanel" class="panel active">
      Image wall: <code>{wall_meta["wall_width_m"]:.3f} m x {wall_meta["wall_height_m"]:.3f} m</code><br>
      Point cloud: <code>{cloud_meta["projectedWidthM"]:.3f} m x {cloud_meta["projectedHeightM"]:.3f} m</code><br>
      Surface ratio bbox: <code>{cloud_meta["projectedAreaBBoxM2"] / wall_meta["wall_area_m2"] * 100.0:.1f}%</code><br>
      Points shown: <code>{cloud_meta["pointCountDisplayed"]:,}</code> / <code>{cloud_meta["pointCountTotal"]:,}</code>
    </div>
    <div id="dataPanel" class="panel">
      <div class="row"><span>Depth mean</span><span id="depthMean">n/a</span></div>
      <div class="row"><span>Depth sigma</span><span id="depthSigma">n/a</span></div>
      <div class="row"><span>Depth P05 / P95</span><span id="depthP">n/a</span></div>
      <div class="row"><span>FOV H / V</span><span>{wall_meta["camera_horizontal_fov_deg"]:.2f} deg / {wall_meta["camera_vertical_fov_deg"]:.2f} deg</span></div>
      <div class="row"><span>Single image footprint</span><span>{wall_meta["single_frame_wall_width_m"]:.3f} m x {wall_meta["single_frame_wall_height_m"]:.3f} m</span></div>
      <div class="row"><span>Selected point</span><span id="selectedIndex">none</span></div>
      <div class="row"><span>Selected X / Z</span><span id="selectedXZ">n/a</span></div>
      <div class="row"><span>Selected depth</span><span id="selectedDepth">n/a</span></div>
      <div class="row"><span>Measure A / B</span><span id="measurePoints">select 2 points</span></div>
      <div class="row"><span>Distance total</span><span id="measureTotal">n/a</span></div>
      <div class="row"><span>Delta X</span><span id="measureDx">n/a</span></div>
      <div class="row"><span>Delta Y depth</span><span id="measureDy">n/a</span></div>
      <div class="row"><span>Delta Z</span><span id="measureDz">n/a</span></div>
      <canvas id="depthChart" class="chart" width="1320" height="820"></canvas>
    </div>
    <div id="controls">
      <label>Cloud opacity</label><input id="cloudOpacity" type="range" min="0" max="1" step="0.02" value="0.95">
      <label>Wall opacity</label><input id="wallOpacity" type="range" min="0" max="1" step="0.02" value="0.72">
      <label>Cloud offset</label><input id="cloudOffset" type="range" min="0" max="0.25" step="0.005" value="0.035">
      <label>Point size</label><input id="pointSize" type="range" min="0.5" max="4" step="0.25" value="1">
      <label>Fit plane</label><input id="fitPlaneToggle" type="checkbox">
      <label>Plane opacity</label><input id="fitPlaneOpacity" type="range" min="0.02" max="0.45" step="0.02" value="0.10">
    </div>
    <div id="error"></div>
  </div>
  <script>
    window.onerror = (message, source, lineno, colno, error) => {{
      const box = document.getElementById('error');
      box.style.display = 'block';
      box.textContent = String(message) + '\\n' + String(source || '') + ':' + String(lineno || '') + ':' + String(colno || '');
    }};
  </script>
  <script>window.COVERAGE_META = {meta_json};</script>
  <script src="pointcloud_data.js"></script>
  <script type="module">
    import * as THREE from './three.module.js';
    import {{ OrbitControls }} from './OrbitControls.js';

    const app = document.getElementById('app');
    const renderer = new THREE.WebGLRenderer({{ antialias: true }});
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setClearColor(0x0d1117, 1);
    app.appendChild(renderer.domElement);

    const axisInset = document.getElementById('axisInset');
    const axisRenderer = new THREE.WebGLRenderer({{ antialias: true, alpha: true }});
    axisRenderer.setPixelRatio(window.devicePixelRatio);
    axisRenderer.setSize(axisInset.clientWidth, axisInset.clientHeight);
    axisRenderer.setClearColor(0x000000, 0);
    axisInset.appendChild(axisRenderer.domElement);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 100);
    const wall = window.COVERAGE_META.wall;
    camera.position.set(wall.wall_width_m / 2, -2.25, wall.wall_height_m / 2);

    const axisScene = new THREE.Scene();
    const axisCamera = new THREE.PerspectiveCamera(45, 1, 0.01, 10);
    axisCamera.position.set(0, 0, 3.2);
    axisCamera.lookAt(0, 0, 0);

    function makeAxisLabel(text, color) {{
      const canvas = document.createElement('canvas');
      canvas.width = 192;
      canvas.height = 96;
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.font = 'bold 42px Arial';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillStyle = color;
      ctx.strokeStyle = 'rgba(0,0,0,0.65)';
      ctx.lineWidth = 8;
      ctx.strokeText(text, canvas.width / 2, canvas.height / 2);
      ctx.fillText(text, canvas.width / 2, canvas.height / 2);
      const texture = new THREE.CanvasTexture(canvas);
      const material = new THREE.SpriteMaterial({{ map: texture, transparent: true, depthTest: false }});
      const sprite = new THREE.Sprite(material);
      sprite.scale.set(0.42, 0.21, 1);
      return sprite;
    }}

    const axisRoot = new THREE.Group();
    const axisLength = 1.05;
    const xArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), axisLength, 0xff4d4d, 0.18, 0.1);
    const yArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), axisLength, 0x55d66b, 0.18, 0.1);
    const zArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0), axisLength, 0x58a6ff, 0.18, 0.1);
    axisRoot.add(xArrow, yArrow, zArrow);
    const xLabel = makeAxisLabel('X', '#ff6b6b');
    const yLabel = makeAxisLabel('Y depth', '#6ee787');
    const zLabel = makeAxisLabel('Z', '#79c0ff');
    xLabel.position.set(1.32, 0, 0);
    yLabel.position.set(0, 1.32, 0);
    zLabel.position.set(0, 0, 1.32);
    axisRoot.add(xLabel, yLabel, zLabel);
    axisScene.add(axisRoot);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(wall.wall_width_m / 2, 0.0, wall.wall_height_m / 2);
    controls.enableDamping = true;

    const texture = new THREE.TextureLoader().load('wall_mosaic.jpg');
    texture.colorSpace = THREE.SRGBColorSpace;
    const wallGeometry = new THREE.PlaneGeometry(wall.wall_width_m, wall.wall_height_m);
    const wallMaterial = new THREE.MeshBasicMaterial({{ map: texture, transparent: true, opacity: 0.72, side: THREE.DoubleSide }});
    const wallMesh = new THREE.Mesh(wallGeometry, wallMaterial);
    wallMesh.rotation.x = Math.PI / 2;
    wallMesh.position.set(wall.wall_width_m / 2, 0, wall.wall_height_m / 2);
    scene.add(wallMesh);

    const border = new THREE.LineSegments(
      new THREE.EdgesGeometry(wallGeometry),
      new THREE.LineBasicMaterial({{ color: 0xffffff, transparent: true, opacity: 0.45 }})
    );
    border.rotation.copy(wallMesh.rotation);
    border.position.copy(wallMesh.position);
    scene.add(border);

    const fitPlaneMaterial = new THREE.MeshBasicMaterial({{ color: 0xff2f2f, transparent: true, opacity: 0.10, side: THREE.DoubleSide, depthWrite: false }});
    const fitPlaneMesh = new THREE.Mesh(wallGeometry, fitPlaneMaterial);
    fitPlaneMesh.rotation.x = Math.PI / 2;
    fitPlaneMesh.position.set(wall.wall_width_m / 2, 0.035, wall.wall_height_m / 2);
    fitPlaneMesh.visible = false;
    scene.add(fitPlaneMesh);

    const positions = new Float32Array(window.COVERAGE_POINT_DATA.positions.flat());
    const colors = new Float32Array(window.COVERAGE_POINT_DATA.colors.flat().map(v => v / 255));
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    const cloudMaterial = new THREE.PointsMaterial({{ size: 1, sizeAttenuation: false, vertexColors: true, transparent: true, opacity: 0.95 }});
    const pointCloud = new THREE.Points(geometry, cloudMaterial);
    pointCloud.userData.offset = 0.035;
    scene.add(pointCloud);

    const measureLineGeometry = new THREE.BufferGeometry();
    measureLineGeometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array([0, 0, 0, 0, 0, 0]), 3));
    const measureLineMaterial = new THREE.LineBasicMaterial({{ color: 0xff0000, linewidth: 2, depthTest: true, transparent: true, opacity: 0.95 }});
    const measureLine = new THREE.Line(measureLineGeometry, measureLineMaterial);
    measureLine.visible = false;
    scene.add(measureLine);

    const grid = new THREE.GridHelper(Math.max(wall.wall_width_m, wall.wall_height_m), 10, 0x30363d, 0x30363d);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(wall.wall_width_m / 2, -0.01, wall.wall_height_m / 2);
    scene.add(grid);

    document.getElementById('cloudOpacity').addEventListener('input', event => cloudMaterial.opacity = Number(event.target.value));
    document.getElementById('wallOpacity').addEventListener('input', event => wallMaterial.opacity = Number(event.target.value));
    document.getElementById('pointSize').addEventListener('input', event => cloudMaterial.size = Number(event.target.value));
    document.getElementById('fitPlaneToggle').addEventListener('change', event => {{
      fitPlaneMesh.visible = event.target.checked;
      drawDepthChart();
    }});
    document.getElementById('fitPlaneOpacity').addEventListener('input', event => fitPlaneMaterial.opacity = Number(event.target.value));
    document.getElementById('cloudOffset').addEventListener('input', event => {{
      const desired = Number(event.target.value);
      const arr = geometry.attributes.position.array;
      const currentMean = pointCloud.userData.offset;
      const delta = desired - currentMean;
      for (let i = 1; i < arr.length; i += 3) arr[i] += delta;
      pointCloud.userData.offset = desired;
      fitPlaneMesh.position.y = desired;
      if (measureLine.visible) {{
        const lineArr = measureLine.geometry.attributes.position.array;
        lineArr[1] += delta;
        lineArr[4] += delta;
        measureLine.geometry.attributes.position.needsUpdate = true;
      }}
      geometry.attributes.position.needsUpdate = true;
    }});

    for (const tab of document.querySelectorAll('.tab')) {{
      tab.addEventListener('click', () => {{
        document.querySelectorAll('.tab').forEach(item => item.classList.remove('active'));
        document.querySelectorAll('.panel').forEach(item => item.classList.remove('active'));
        tab.classList.add('active');
        document.getElementById(tab.dataset.panel).classList.add('active');
      }});
    }}

    let selectedDepthMm = null;
    let measurePoints = [];
    const selectedOriginalColors = new Map();

    function setPointColor(index, r, g, b) {{
      const colorAttr = geometry.attributes.color;
      colorAttr.setXYZ(index, r, g, b);
      colorAttr.needsUpdate = true;
    }}

    function rememberOriginalColor(index) {{
      if (selectedOriginalColors.has(index)) return;
      const colorAttr = geometry.attributes.color;
      selectedOriginalColors.set(index, [
        colorAttr.getX(index),
        colorAttr.getY(index),
        colorAttr.getZ(index),
      ]);
    }}

    function clearSelectedPointColors() {{
      const colorAttr = geometry.attributes.color;
      for (const [index, color] of selectedOriginalColors.entries()) {{
        colorAttr.setXYZ(index, color[0], color[1], color[2]);
      }}
      selectedOriginalColors.clear();
      colorAttr.needsUpdate = true;
    }}

    function drawDepthChart() {{
      const data = window.COVERAGE_POINT_DATA;
      document.getElementById('depthMean').textContent = `${{data.depthMeanMm.toFixed(2)}} mm`;
      document.getElementById('depthSigma').textContent = `${{data.depthSigmaMm.toFixed(2)}} mm`;
      document.getElementById('depthP').textContent = `${{data.depthP05Mm.toFixed(2)}} / ${{data.depthP95Mm.toFixed(2)}} mm`;
      const canvas = document.getElementById('depthChart');
      const ctx = canvas.getContext('2d');
      const w = canvas.width;
      const h = canvas.height;
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#0d1117';
      ctx.fillRect(0, 0, w, h);
      const counts = data.depthHistCounts;
      const edges = data.depthHistEdgesMm;
      const maxCount = Math.max(...counts);
      const padLeft = 64;
      const padRight = 22;
      const padTop = 42;
      const padBottom = 58;
      const plotW = w - padLeft - padRight;
      const plotH = h - padTop - padBottom;
      ctx.strokeStyle = 'rgba(230,237,243,0.25)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(padLeft, padTop);
      ctx.lineTo(padLeft, h - padBottom);
      ctx.lineTo(w - padRight, h - padBottom);
      ctx.stroke();
      for (let i = 0; i < counts.length; i++) {{
        const x0 = padLeft + i * plotW / counts.length;
        const bw = Math.max(2, plotW / counts.length - 1);
        const bh = counts[i] / maxCount * plotH;
        const inSelectedBin = selectedDepthMm !== null && selectedDepthMm >= edges[i] && selectedDepthMm < edges[i + 1];
        ctx.fillStyle = inSelectedBin ? '#ff3030' : '#79c0ff';
        ctx.fillRect(x0, h - padBottom - bh, bw, bh);
      }}
      const xForDepth = depthMm => padLeft + (depthMm - edges[0]) / (edges[edges.length - 1] - edges[0]) * plotW;
      if (document.getElementById('fitPlaneToggle').checked) {{
        const x0 = xForDepth(0);
        if (x0 >= padLeft && x0 <= w - padRight) {{
          ctx.strokeStyle = '#ff3030';
          ctx.lineWidth = 4;
          ctx.beginPath();
          ctx.moveTo(x0, padTop);
          ctx.lineTo(x0, h - padBottom);
          ctx.stroke();
          ctx.fillStyle = '#ffb3b3';
          ctx.font = '24px Arial';
          ctx.fillText('fit plane', x0 + 8, padTop + 28);
        }}
      }}
      if (selectedDepthMm !== null) {{
        const xs = xForDepth(selectedDepthMm);
        if (xs >= padLeft && xs <= w - padRight) {{
          ctx.strokeStyle = '#ff3030';
          ctx.lineWidth = 5;
          ctx.beginPath();
          ctx.moveTo(xs, padTop);
          ctx.lineTo(xs, h - padBottom);
          ctx.stroke();
          ctx.fillStyle = '#ff3030';
          ctx.font = '24px Arial';
          ctx.fillText(`selected ${{selectedDepthMm.toFixed(2)}} mm`, Math.min(xs + 8, w - 270), padTop + 58);
        }}
      }}
      ctx.fillStyle = '#c9d1d9';
      ctx.font = '22px Arial';
      ctx.fillText(`${{edges[0].toFixed(1)}} mm`, padLeft, h - 18);
      ctx.fillText(`${{edges[edges.length - 1].toFixed(1)}} mm`, w - 150, h - 18);
      ctx.font = '24px Arial';
      ctx.fillText('Depth distribution relative to fitted plane', padLeft, 30);
    }}
    drawDepthChart();

    const raycaster = new THREE.Raycaster();
    raycaster.params.Points.threshold = 0.008;
    const pointer = new THREE.Vector2();
    const clickStart = new THREE.Vector2();

    function updateMeasureDisplay() {{
      if (measurePoints.length === 0) {{
        document.getElementById('measurePoints').textContent = 'select 2 points';
        document.getElementById('measureTotal').textContent = 'n/a';
        document.getElementById('measureDx').textContent = 'n/a';
        document.getElementById('measureDy').textContent = 'n/a';
        document.getElementById('measureDz').textContent = 'n/a';
        measureLine.visible = false;
        return;
      }}
      if (measurePoints.length === 1) {{
        document.getElementById('measurePoints').textContent = `A #${{measurePoints[0].index}} / select B`;
        document.getElementById('measureTotal').textContent = 'n/a';
        document.getElementById('measureDx').textContent = 'n/a';
        document.getElementById('measureDy').textContent = 'n/a';
        document.getElementById('measureDz').textContent = 'n/a';
        measureLine.visible = false;
        return;
      }}
      const a = measurePoints[0];
      const b = measurePoints[1];
      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const dz = b.z - a.z;
      const total = Math.hypot(dx, dy, dz);
      document.getElementById('measurePoints').textContent = `A #${{a.index}} / B #${{b.index}}`;
      document.getElementById('measureTotal').textContent = `${{total.toFixed(4)}} m`;
      document.getElementById('measureDx').textContent = `${{dx.toFixed(4)}} m`;
      document.getElementById('measureDy').textContent = `${{dy.toFixed(4)}} m`;
      document.getElementById('measureDz').textContent = `${{dz.toFixed(4)}} m`;
      const lineArr = measureLine.geometry.attributes.position.array;
      lineArr[0] = a.x; lineArr[1] = a.y; lineArr[2] = a.z;
      lineArr[3] = b.x; lineArr[4] = b.y; lineArr[5] = b.z;
      measureLine.geometry.attributes.position.needsUpdate = true;
      measureLine.visible = true;
    }}

    renderer.domElement.addEventListener('pointerdown', event => {{
      clickStart.set(event.clientX, event.clientY);
    }});

    renderer.domElement.addEventListener('pointerup', event => {{
      const dx = event.clientX - clickStart.x;
      const dy = event.clientY - clickStart.y;
      if (Math.hypot(dx, dy) > 4) return;
      const rect = renderer.domElement.getBoundingClientRect();
      pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.params.Points.threshold = 0.006;
      raycaster.setFromCamera(pointer, camera);
      const hits = raycaster.intersectObject(pointCloud, false);
      if (!hits.length) return;
      const hit = hits[0];
      const index = hit.index;
      const arr = geometry.attributes.position.array;
      const base = index * 3;
      const x = arr[base];
      const y = arr[base + 1];
      const z = arr[base + 2];
      const depthMm = (y - pointCloud.userData.offset) * 1000.0;
      if (measurePoints.length >= 2) {{
        measurePoints = [];
        clearSelectedPointColors();
      }}
      measurePoints.push({{ index, x, y, z }});
      rememberOriginalColor(index);
      setPointColor(index, 1, 0, 0);
      selectedDepthMm = depthMm;
      document.getElementById('selectedIndex').textContent = `#${{index}}`;
      document.getElementById('selectedXZ').textContent = `${{x.toFixed(4)}} m / ${{z.toFixed(4)}} m`;
      document.getElementById('selectedDepth').textContent = `${{depthMm.toFixed(2)}} mm`;
      document.querySelectorAll('.tab').forEach(item => item.classList.remove('active'));
      document.querySelectorAll('.panel').forEach(item => item.classList.remove('active'));
      document.querySelector('[data-panel="dataPanel"]').classList.add('active');
      document.getElementById('dataPanel').classList.add('active');
      updateMeasureDisplay();
      drawDepthChart();
    }});

    document.getElementById('status').textContent = '3D scene ready';

    window.addEventListener('resize', () => {{
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
      axisRenderer.setSize(axisInset.clientWidth, axisInset.clientHeight);
    }});

    function animate() {{
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
      axisRoot.quaternion.copy(camera.quaternion).invert();
      axisRenderer.render(axisScene, axisCamera);
    }}
    animate();
  </script>
</body>
</html>
"""
    path.write_text(html, encoding="utf-8")


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir or args.las_path.parent / "coverage_viewer"
    output_dir.mkdir(parents=True, exist_ok=True)
    records = read_capture_records(args.session_dir)
    wall_meta = build_wall_texture(
        args.session_dir,
        records,
        args.wall_distance_m,
        args.zaber_height_offset_mm,
        args.camera_35mm_equivalent_mm,
        args.texture_px_per_m,
        output_dir / "wall_mosaic.jpg",
    )
    cloud_meta = projected_pointcloud(
        args.las_path,
        args.point_sample_size,
        args.seed,
        wall_meta,
        output_dir / "pointcloud_data.js",
    )
    ensure_three_vendor(output_dir)
    (output_dir / "viewer_metrics.json").write_text(
        json.dumps({"wall": wall_meta, "cloud": cloud_meta}, indent=2),
        encoding="utf-8",
    )
    write_html(output_dir / "viewer.html", wall_meta, cloud_meta)
    print(f"[viewer] output: {output_dir / 'viewer.html'}")
    print(f"[viewer] wall: {wall_meta['wall_width_m']:.3f} x {wall_meta['wall_height_m']:.3f} m")
    print(f"[viewer] cloud bbox: {cloud_meta['projectedWidthM']:.3f} x {cloud_meta['projectedHeightM']:.3f} m")
    print(f"[viewer] bbox surface ratio: {cloud_meta['projectedAreaBBoxM2'] / wall_meta['wall_area_m2'] * 100.0:.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
