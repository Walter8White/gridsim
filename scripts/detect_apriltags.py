#!/usr/bin/env python3
"""Detect AprilTags in GridCapture iPhone sessions.

This does not make PIX4Dengine automatically use the tags yet. It creates a
machine-readable detection file that the facade QA / scaling pipeline can use.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


APRILTAG_FAMILIES = {
    "tag16h5": "DICT_APRILTAG_16h5",
    "tag25h9": "DICT_APRILTAG_25h9",
    "tag36h10": "DICT_APRILTAG_36h10",
    "tag36h11": "DICT_APRILTAG_36h11",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--session-dir",
        type=Path,
        default=None,
        help="GridCapture session directory. Defaults to latest session with images.",
    )
    parser.add_argument(
        "--family",
        choices=sorted(APRILTAG_FAMILIES),
        default="tag36h11",
        help="AprilTag family used by the printed tags.",
    )
    parser.add_argument(
        "--last",
        type=int,
        default=None,
        help="Only scan the last N images.",
    )
    parser.add_argument(
        "--tag-size-mm",
        type=float,
        default=None,
        help="Printed black-square tag size. Stored in the output for later scaling.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Detection JSON path. Defaults to SESSION/apriltags.json.",
    )
    parser.add_argument(
        "--annotated-dir",
        type=Path,
        default=None,
        help="Directory for debug images with detected tag outlines.",
    )
    parser.add_argument(
        "--no-annotated",
        action="store_true",
        help="Do not write debug images.",
    )
    return parser.parse_args()


def latest_session(root: Path) -> Path:
    sessions = []
    for path in root.glob("session_*"):
        image_dir = path / "images"
        if image_dir.exists() and any(image_dir.glob("*.jpg")):
            sessions.append(path)
    if not sessions:
        raise SystemExit(f"No GridCapture sessions with images found under {root}")
    return max(sessions, key=lambda p: p.stat().st_mtime)


def image_paths(session_dir: Path, limit: int | None) -> list[Path]:
    paths = sorted((session_dir / "images").glob("*.jpg"))
    if not paths:
        raise SystemExit(f"No JPG images found in {session_dir / 'images'}")
    if limit is not None:
        paths = paths[-limit:]
    return paths


def make_detector(family: str):
    dictionary_name = APRILTAG_FAMILIES[family]
    if not hasattr(cv2, "aruco"):
        raise SystemExit(
            "This OpenCV build has no cv2.aruco module. Install opencv-contrib-python-headless."
        )
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    parameters = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(dictionary, parameters)
    return dictionary, parameters


def detect(detector: Any, gray: np.ndarray):
    if hasattr(detector, "detectMarkers"):
        return detector.detectMarkers(gray)
    dictionary, parameters = detector
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def detection_record(image_path: Path, tag_id: int, corners: np.ndarray) -> dict[str, Any]:
    pts = corners.reshape(4, 2)
    side_lengths = [
        float(np.linalg.norm(pts[(index + 1) % 4] - pts[index])) for index in range(4)
    ]
    center = pts.mean(axis=0)
    return {
        "image": image_path.name,
        "id": int(tag_id),
        "center_px": [float(center[0]), float(center[1])],
        "corners_px": [[float(x), float(y)] for x, y in pts],
        "mean_side_px": float(np.mean(side_lengths)),
        "min_side_px": float(np.min(side_lengths)),
        "max_side_px": float(np.max(side_lengths)),
    }


def main() -> int:
    args = parse_args()
    session_dir = args.session_dir or latest_session(Path("captures/iphone_zaber"))
    output_path = args.output or session_dir / "apriltags.json"
    annotated_dir = args.annotated_dir or session_dir / "apriltag_debug"

    paths = image_paths(session_dir, args.last)
    detector = make_detector(args.family)
    records: list[dict[str, Any]] = []
    per_image_counts: dict[str, int] = {}

    if not args.no_annotated:
        annotated_dir.mkdir(parents=True, exist_ok=True)

    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            raise SystemExit(f"Could not read image: {path}")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = detect(detector, gray)

        count = 0 if ids is None else len(ids)
        per_image_counts[path.name] = count
        if ids is not None and count:
            for tag_id, tag_corners in zip(ids.flatten(), corners):
                records.append(detection_record(path, int(tag_id), tag_corners))
            if not args.no_annotated:
                debug = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
                cv2.imwrite(str(annotated_dir / path.name), debug)

    summary = {
        "session_dir": str(session_dir),
        "family": args.family,
        "tag_size_mm": args.tag_size_mm,
        "images_scanned": len(paths),
        "detections": len(records),
        "unique_tag_ids": sorted({record["id"] for record in records}),
        "per_image_counts": per_image_counts,
        "records": records,
    }
    output_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"[apriltag] session: {session_dir}")
    print(f"[apriltag] family: {args.family}")
    print(f"[apriltag] images scanned: {len(paths)}")
    print(f"[apriltag] detections: {len(records)}")
    print(f"[apriltag] unique ids: {summary['unique_tag_ids']}")
    print(f"[apriltag] output: {output_path}")
    if not args.no_annotated:
        print(f"[apriltag] debug images: {annotated_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
