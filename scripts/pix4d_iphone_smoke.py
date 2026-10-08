#!/usr/bin/env python3
"""Run a minimal PIX4Dengine smoke test on GridCapture iPhone images.

By default this can run with synthetic weak positions for a visual-only smoke
test. For GridCapture sessions where the phone is rigidly mounted to the Zaber,
use ``--pose-source zaber`` to turn measured Zaber positions into camera pose
hints.
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import os
import shutil
import sys
import tempfile
import time
from functools import partial
from pathlib import Path

import psutil
from pix4dvortex import calib, cameras, dense, dmodel, proc, session
from pix4dvortex.io import las


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--session-dir",
        type=Path,
        default=None,
        help="GridCapture session directory. Defaults to latest session with images.",
    )
    parser.add_argument("--last", type=int, default=18, help="Number of latest captures to use.")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("pix4d_outputs/iphone_smoke"),
        help="Output root for prepared data and processing results.",
    )
    parser.add_argument("--no-gpu", action="store_true", help="Disable GPU use.")
    parser.add_argument(
        "--skip-imu",
        action="store_true",
        help="Do not pass iPhone IMU orientation hints to Pix4D.",
    )
    parser.add_argument(
        "--pose-source",
        choices=("synthetic", "zaber", "none"),
        default="synthetic",
        help="Position source for Pix4D geotags.",
    )
    parser.add_argument(
        "--wall-distance-m",
        type=float,
        default=1.0,
        help="Camera-to-wall distance used for --pose-source zaber.",
    )
    parser.add_argument(
        "--zaber-height-offset-mm",
        type=float,
        default=150.0,
        help="Floor height offset: camera height ~= measured_zaber_mm + offset.",
    )
    parser.add_argument(
        "--column-x-m",
        type=float,
        default=0.0,
        help="Horizontal column position used for --pose-source zaber.",
    )
    return parser.parse_args()


def timestamp() -> str:
    return time.strftime("%Y%m%dT%H%M%S")


def latest_session(root: Path) -> Path:
    sessions = []
    for path in root.glob("session_*"):
        image_dir = path / "images"
        if image_dir.exists() and any(image_dir.glob("*.jpg")):
            sessions.append(path)
    if not sessions:
        raise SystemExit(f"No GridCapture sessions with images found under {root}")
    return max(sessions, key=lambda p: p.stat().st_mtime)


def capture_pairs(session_dir: Path, limit: int) -> list[tuple[Path, Path]]:
    images = {p.stem: p for p in (session_dir / "images").glob("*.jpg")}
    metadata = {p.stem: p for p in (session_dir / "metadata").glob("*.json")}
    stems = sorted(set(images) & set(metadata))
    if not stems:
        raise SystemExit(f"No matching image/metadata pairs found in {session_dir}")
    selected = stems[-limit:]
    return [(images[stem], metadata[stem]) for stem in selected]


def load_manifest_by_capture_id(session_dir: Path) -> dict[str, dict]:
    manifest_path = session_dir / "manifest.jsonl"
    if not manifest_path.exists():
        return {}
    records = {}
    with manifest_path.open("r", encoding="utf-8") as manifest:
        for line in manifest:
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            capture_id = record.get("capture_id")
            if capture_id:
                records[capture_id] = record
    return records


def get_logger(name: str, log_file: Path, level: int):
    logger = logging.getLogger(name)
    logger.setLevel(level)
    handler = logging.FileHandler(log_file)
    formatter = logging.Formatter(
        "%(asctime)s | %(name)-8s | PID %(process)-7d | %(levelname)-8s | %(message)s"
    )
    handler.setFormatter(formatter)
    handler.setLevel(level)
    logger.addHandler(handler)
    return logger.log


def progress(task: str, percentage: int) -> None:
    print(f"[pix4d] {task}: {percentage}%", flush=True)


def resources(work_dir: tempfile.TemporaryDirectory[str], use_gpu: bool) -> proc.Resources:
    return proc.Resources(
        max_ram=int(0.75 * psutil.virtual_memory().available),
        max_threads=int(psutil.cpu_count() or 1),
        use_gpu=use_gpu,
        max_gpu_mem=12_000_000_000,
        work_dir=work_dir.name,
    )


def prepare_inputs(
    pairs: list[tuple[Path, Path]],
    output_dir: Path,
    use_imu: bool,
    pose_source: str,
    wall_distance_m: float,
    zaber_height_offset_mm: float,
    column_x_m: float,
    manifest_by_capture_id: dict[str, dict],
) -> tuple[list[Path], dict[Path, cameras.GeoTag]]:
    input_dir = output_dir / "input_data"
    image_dir = input_dir / "images"
    meta_dir = input_dir / "metadata"
    image_dir.mkdir(parents=True, exist_ok=False)
    meta_dir.mkdir(parents=True, exist_ok=False)

    image_paths: list[Path] = []
    geotags: dict[Path, cameras.GeoTag] = {}
    imu_csv = output_dir / "imu_orientations.csv"
    pose_csv = output_dir / "camera_pose_hints.csv"

    with (
        imu_csv.open("w", newline="", encoding="utf-8") as imu_file,
        pose_csv.open("w", newline="", encoding="utf-8") as pose_file,
    ):
        imu_writer = csv.writer(imu_file)
        imu_writer.writerow(
            [
                "image",
                "roll_deg_ios_raw",
                "pitch_deg_ios_raw",
                "yaw_deg_ios_raw",
                "imu_used_as_pix4d_yaw_pitch_roll_hint",
            ],
        )
        pose_writer = csv.writer(pose_file)
        pose_writer.writerow(
            [
                "image",
                "pose_source",
                "camera_x_m",
                "camera_y_m",
                "camera_z_m",
                "lat_deg_synthetic",
                "lon_deg_synthetic",
                "alt_m",
                "sigma_east_m",
                "sigma_north_m",
                "sigma_up_m",
            ],
        )
        for image_path, metadata_path in pairs:
            copied_image = image_dir / image_path.name
            copied_meta = meta_dir / metadata_path.name
            shutil.copy2(image_path, copied_image)
            shutil.copy2(metadata_path, copied_meta)
            image_paths.append(copied_image)

            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            motion = metadata.get("motion", {})
            roll = math.degrees(float(motion.get("rollRadians", 0.0)))
            pitch = math.degrees(float(motion.get("pitchRadians", 0.0)))
            yaw = math.degrees(float(motion.get("yawRadians", 0.0)))
            imu_writer.writerow([copied_image.name, roll, pitch, yaw, use_imu])

            geolocation = None
            camera_x_m = camera_y_m = camera_z_m = None
            sigma_east_m = sigma_north_m = sigma_up_m = None
            lat = lon = alt = None
            if pose_source != "none":
                lat0 = 46.0
                lon0 = 6.0
                if pose_source == "zaber":
                    measured_mm = metadata.get("measuredPositionMM")
                    if measured_mm is None:
                        raise SystemExit(f"{metadata_path} has no measuredPositionMM")
                    capture_id = metadata.get("captureID") or metadata_path.stem
                    manifest_record = manifest_by_capture_id.get(capture_id, {})
                    camera_x_m = float(manifest_record.get("x_offset_m", column_x_m))
                    camera_y_m = wall_distance_m
                    camera_z_m = (float(measured_mm) + zaber_height_offset_mm) / 1000.0
                    sigma_east_m = 0.20
                    sigma_north_m = 0.10
                    sigma_up_m = 0.005
                else:
                    # PIX4Dengine requires a geolocation in order to accept yaw/pitch/roll.
                    # These positions deliberately have huge sigmas: they only bootstrap a
                    # no-GPS iPhone smoke test and must not be interpreted as metric truth.
                    index = len(image_paths) - 1
                    camera_x_m = (index - (len(pairs) - 1) / 2.0) * 0.05
                    camera_y_m = 0.0
                    camera_z_m = 0.5
                    sigma_east_m = sigma_north_m = sigma_up_m = 100.0

                meters_east = camera_x_m
                meters_north = camera_y_m
                lat = lat0 + meters_north / 111_320.0
                lon = lon0 + meters_east / (111_320.0 * math.cos(math.radians(lat0)))
                alt = camera_z_m
                geolocation = dmodel.Geolocation(
                    crs=dmodel.CRS(definition="EPSG:4326"),
                    coordinates=[lat, lon, alt],
                    sigmas=[sigma_east_m, sigma_north_m, sigma_up_m],
                )

            pose_writer.writerow(
                [
                    copied_image.name,
                    pose_source,
                    "" if camera_x_m is None else f"{camera_x_m:.6f}",
                    "" if camera_y_m is None else f"{camera_y_m:.6f}",
                    "" if camera_z_m is None else f"{camera_z_m:.6f}",
                    "" if lat is None else f"{lat:.10f}",
                    "" if lon is None else f"{lon:.10f}",
                    "" if alt is None else f"{alt:.6f}",
                    "" if sigma_east_m is None else f"{sigma_east_m:.6f}",
                    "" if sigma_north_m is None else f"{sigma_north_m:.6f}",
                    "" if sigma_up_m is None else f"{sigma_up_m:.6f}",
                ],
            )

            orientation = None
            if use_imu:
                orientation = dmodel.YawPitchRoll(
                    angles_deg=[yaw, pitch, roll],
                    sigmas_deg=[60.0, 45.0, 45.0],
                )

            if geolocation is not None or orientation is not None:
                geotags[copied_image] = cameras.GeoTag(
                    geolocation=geolocation,
                    orientation=orientation,
                )

    return image_paths, geotags


def require_pix4d_credentials() -> None:
    missing = [key for key in ("CLIENT_ID", "CLIENT_SECRET", "LICENSE_KEY") if not os.environ.get(key)]
    if missing:
        raise SystemExit(
            "Missing Pix4D credentials in environment: "
            + ", ".join(missing)
            + "\nExport them before running this script."
        )


def main() -> int:
    args = parse_args()
    session_dir = args.session_dir or latest_session(Path("captures/iphone_zaber"))
    pairs = capture_pairs(session_dir, args.last)
    manifest_by_capture_id = load_manifest_by_capture_id(session_dir)
    out_dir = args.output_root / f"{timestamp()}_{session_dir.name}"
    out_dir.mkdir(parents=True, exist_ok=False)
    log_file = out_dir / "processing_results.log"
    pcl_dir = out_dir / "pcl"
    pcl_dir.mkdir()

    print(f"[pix4d] session: {session_dir}")
    print(f"[pix4d] captures: {len(pairs)}")
    print(f"[pix4d] output: {out_dir}")
    print(f"[pix4d] pose source: {args.pose_source}")
    if args.pose_source == "zaber":
        print(f"[pix4d] zaber height offset: {args.zaber_height_offset_mm:.1f} mm")
        print(f"[pix4d] wall distance: {args.wall_distance_m:.3f} m")
        print(f"[pix4d] column x: {args.column_x_m:.3f} m")
    print(f"[pix4d] using iPhone IMU: {'no' if args.skip_imu else 'yes, weak orientation hints'}")

    image_paths, geotags = prepare_inputs(
        pairs,
        out_dir,
        use_imu=not args.skip_imu,
        pose_source=args.pose_source,
        wall_distance_m=args.wall_distance_m,
        zaber_height_offset_mm=args.zaber_height_offset_mm,
        column_x_m=args.column_x_m,
        manifest_by_capture_id=manifest_by_capture_id,
    )
    require_pix4d_credentials()

    work_dir = tempfile.TemporaryDirectory()
    pix4d_resources = resources(work_dir, use_gpu=not args.no_gpu)

    print("[pix4d] login")
    session.login(
        client_id=os.environ["CLIENT_ID"],
        client_secret=os.environ["CLIENT_SECRET"],
        license_key=os.environ["LICENSE_KEY"],
    )
    assert session.is_logged_in()

    print("[pix4d] make input cameras")
    try:
        input_cameras = cameras.make_input_cameras(
            image_info=image_paths,
            external_geotags=geotags,
            logger=get_logger("camera", log_file, logging.INFO),
            progress_callback=progress,
        )
    except Exception as exc:
        if not geotags:
            raise
        print(f"[pix4d] geotag import failed, retrying without geotags: {exc}")
        input_cameras = cameras.make_input_cameras(
            image_info=image_paths,
            logger=get_logger("camera", log_file, logging.INFO),
            progress_callback=progress,
        )

    projected_cameras = cameras.ProjectedCameras(input_cameras=input_cameras)

    print("[pix4d] calibrate")
    calib_settings = calib.Settings.TEMPLATES.LARGE
    calib_settings.pipeline = calib.Settings.CalibrationType.Standard
    calibrated_scene = calib.calibrate(
        cameras=projected_cameras,
        settings=calib_settings,
        resources=pix4d_resources,
        logger=get_logger("calib", log_file, logging.INFO),
        progress_callback=progress,
    )

    print("[pix4d] densification")
    dense_settings = dense.Settings.TEMPLATES.OBLIQUE
    dense_settings.point_density = 4
    point_cloud, _ = dense.densification(
        scene=calibrated_scene,
        input_cameras=input_cameras,
        settings=dense_settings,
        resources=pix4d_resources,
        logger=get_logger("dense", log_file, logging.INFO),
        progress_callback=progress,
    )
    if point_cloud.empty():
        raise SystemExit("Pix4D produced an empty dense point cloud.")

    las_path = pcl_dir / "dense_pcl.las"
    print(f"[pix4d] write LAS: {las_path}")
    las.write_pcl(output_path=las_path, point_cloud=point_cloud, compress=False)
    print("[pix4d] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
