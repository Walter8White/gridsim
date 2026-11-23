#!/usr/bin/env python3
"""Build a navigable 3D viewer with each capture image as a positioned card."""

from __future__ import annotations

import argparse
import json
import math
import urllib.request
from pathlib import Path

from PIL import Image


THREE_VERSION = "0.160.0"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--wall-distance-m", type=float, default=0.43)
    parser.add_argument("--zaber-height-offset-mm", type=float, default=150.0)
    parser.add_argument("--camera-35mm-equivalent-mm", type=float, default=24.0)
    parser.add_argument("--texture-width", type=int, default=640)
    return parser.parse_args()


def read_records(session_dir: Path) -> list[dict]:
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


def make_textures(session_dir: Path, output_dir: Path, records: list[dict], texture_width: int) -> None:
    texture_dir = output_dir / "textures"
    texture_dir.mkdir(parents=True, exist_ok=True)
    for record in records:
        filename = image_filename(record)
        target = texture_dir / filename
        if target.exists():
            continue
        with Image.open(session_dir / "images" / filename) as image:
            image = image.convert("RGB")
            width, height = image.size
            texture_height = max(1, round(height * texture_width / width))
            image = image.resize((texture_width, texture_height), Image.Resampling.LANCZOS)
            image.save(target, quality=86, optimize=True)


def ensure_three_vendor(output_dir: Path) -> None:
    three_path = output_dir / "three.module.js"
    controls_path = output_dir / "OrbitControls.js"
    if not three_path.exists():
        with urllib.request.urlopen(
            f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/build/three.module.js",
            timeout=20,
        ) as response:
            three_path.write_bytes(response.read())
    if not controls_path.exists():
        with urllib.request.urlopen(
            f"https://cdn.jsdelivr.net/npm/three@{THREE_VERSION}/examples/jsm/controls/OrbitControls.js",
            timeout=20,
        ) as response:
            text = response.read().decode("utf-8")
        controls_path.write_text(text.replace("from 'three';", "from './three.module.js';"), encoding="utf-8")


def build_payload(
    session_dir: Path,
    records: list[dict],
    wall_distance_m: float,
    zaber_height_offset_mm: float,
    equivalent_35mm_mm: float,
) -> dict:
    camera = read_camera(session_dir, records[0]["capture_id"])
    hfov, vfov = fov_from_camera(camera, equivalent_35mm_mm)
    card_width_m = 2.0 * wall_distance_m * math.tan(hfov / 2.0)
    card_height_m = 2.0 * wall_distance_m * math.tan(vfov / 2.0)

    cards = []
    for record in records:
        # Use raw x_offset convention: current session has negative x offsets.
        x = -float(record.get("x_offset_m", 0.0))
        z = (float(record["measured_position_mm"]) + zaber_height_offset_mm) / 1000.0
        cards.append(
            {
                "id": record["capture_id"],
                "filename": image_filename(record),
                "passIndex": int(record.get("pass_index", 0)),
                "passCaptureIndex": int(record.get("pass_capture_index", 0)),
                "globalIndex": int(record.get("global_capture_index", 0)),
                "x": x,
                "z": z,
            }
        )
    xs = [card["x"] for card in cards]
    zs = [card["z"] for card in cards]
    return {
        "session": session_dir.name,
        "cardWidthM": card_width_m,
        "cardHeightM": card_height_m,
        "wallDistanceM": wall_distance_m,
        "minX": min(xs) - card_width_m / 2.0,
        "maxX": max(xs) + card_width_m / 2.0,
        "minZ": min(zs) - card_height_m / 2.0,
        "maxZ": max(zs) + card_height_m / 2.0,
        "cards": cards,
    }


def write_html(output_dir: Path, payload: dict) -> None:
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>3D Image Card Viewer</title>
  <style>
    html, body {{ margin: 0; height: 100%; overflow: hidden; background: #0d1117; color: #e6edf3; font-family: Arial, sans-serif; }}
    #app {{ width: 100%; height: 100%; }}
    #hud {{
      position: fixed; left: 18px; top: 18px; width: 430px; padding: 14px 16px;
      background: rgba(13,17,23,0.86); border: 1px solid rgba(230,237,243,0.18);
      border-radius: 10px; font-size: 15px; line-height: 1.45;
    }}
    #hud code {{ color: #79c0ff; }}
    #controls {{ margin-top: 12px; display: grid; grid-template-columns: 140px 1fr; gap: 8px 12px; align-items: center; }}
    input {{ width: 100%; }}
  </style>
</head>
<body>
  <div id="app"></div>
  <div id="hud">
    <b>3D image card viewer</b><br>
    Session: <code>{payload["session"]}</code><br>
    Cards: <code>{len(payload["cards"])}</code><br>
    Card size: <code>{payload["cardWidthM"]:.3f} m x {payload["cardHeightM"]:.3f} m</code>
    <div id="selected">Selected: none</div>
    <div id="controls">
      <label>Card opacity</label><input id="opacity" type="range" min="0.05" max="1" step="0.05" value="0.78">
      <label>Depth spacing</label><input id="depthSpacing" type="range" min="0" max="0.08" step="0.005" value="0.012">
      <label>Show sequence depth</label><input id="sequenceDepth" type="checkbox" checked>
    </div>
  </div>
  <script>window.IMAGE_CARD_PAYLOAD = {json.dumps(payload)};</script>
  <script type="module">
    import * as THREE from './three.module.js';
    import {{ OrbitControls }} from './OrbitControls.js';

    const payload = window.IMAGE_CARD_PAYLOAD;
    const app = document.getElementById('app');
    const renderer = new THREE.WebGLRenderer({{ antialias: true }});
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setClearColor(0x0d1117, 1);
    app.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 100);
    const centerX = (payload.minX + payload.maxX) / 2;
    const centerZ = (payload.minZ + payload.maxZ) / 2;
    camera.position.set(centerX, -3.2, centerZ);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(centerX, 0, centerZ);
    controls.enableDamping = true;

    const cards = [];
    const loader = new THREE.TextureLoader();
    const geometry = new THREE.PlaneGeometry(payload.cardWidthM, payload.cardHeightM);
    for (const card of payload.cards) {{
      const texture = loader.load(`textures/${{card.filename}}`);
      texture.colorSpace = THREE.SRGBColorSpace;
      const material = new THREE.MeshBasicMaterial({{ map: texture, transparent: true, opacity: 0.78, side: THREE.DoubleSide }});
      const mesh = new THREE.Mesh(geometry, material);
      mesh.rotation.x = Math.PI / 2;
      mesh.position.set(card.x, 0, card.z);
      mesh.userData.card = card;
      scene.add(mesh);
      cards.push(mesh);
    }}

    const grid = new THREE.GridHelper(Math.max(payload.maxX - payload.minX, payload.maxZ - payload.minZ), 12, 0x30363d, 0x30363d);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(centerX, -0.01, centerZ);
    scene.add(grid);

    const raycaster = new THREE.Raycaster();
    const pointer = new THREE.Vector2();
    let selected = null;

    function updateDepth() {{
      const enabled = document.getElementById('sequenceDepth').checked;
      const spacing = Number(document.getElementById('depthSpacing').value);
      for (const mesh of cards) {{
        mesh.position.y = enabled ? mesh.userData.card.globalIndex * spacing / Math.max(1, payload.cards.length - 1) : 0;
      }}
    }}
    updateDepth();

    document.getElementById('opacity').addEventListener('input', event => {{
      for (const mesh of cards) mesh.material.opacity = Number(event.target.value);
    }});
    document.getElementById('depthSpacing').addEventListener('input', updateDepth);
    document.getElementById('sequenceDepth').addEventListener('change', updateDepth);

    renderer.domElement.addEventListener('click', event => {{
      const rect = renderer.domElement.getBoundingClientRect();
      pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      const hits = raycaster.intersectObjects(cards, false);
      if (!hits.length) return;
      if (selected) selected.material.color.set(0xffffff);
      selected = hits[0].object;
      selected.material.color.set(0xff5555);
      const card = selected.userData.card;
      document.getElementById('selected').textContent =
        `Selected: #${{card.globalIndex}} p${{String(card.passIndex).padStart(2, '0')}}-${{String(card.passCaptureIndex).padStart(2, '0')}} x=${{card.x.toFixed(3)}} z=${{card.z.toFixed(3)}}`;
    }});

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
    output_dir = args.output_dir or args.session_dir / "image_card_3d_viewer"
    output_dir.mkdir(parents=True, exist_ok=True)
    records = read_records(args.session_dir)
    make_textures(args.session_dir, output_dir, records, args.texture_width)
    ensure_three_vendor(output_dir)
    payload = build_payload(
        args.session_dir,
        records,
        args.wall_distance_m,
        args.zaber_height_offset_mm,
        args.camera_35mm_equivalent_mm,
    )
    (output_dir / "viewer_data.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    write_html(output_dir, payload)
    print(f"[image-card-3d] output: {output_dir / 'viewer.html'}")
    print(f"[image-card-3d] cards: {len(records)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
