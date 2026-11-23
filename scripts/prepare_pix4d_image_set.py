#!/usr/bin/env python3
"""Prepare a derived GridCapture image set for Pix4D calibration experiments."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--scale",
        type=float,
        default=0.5,
        help="Image scale factor. 0.5 turns 8064x6048 into 4032x3024.",
    )
    parser.add_argument(
        "--clahe-clip",
        type=float,
        default=2.0,
        help="CLAHE clip limit for local contrast.",
    )
    parser.add_argument(
        "--sharpen",
        type=float,
        default=0.8,
        help="Unsharp mask amount.",
    )
    parser.add_argument("--quality", type=int, default=98, help="JPEG quality.")
    return parser.parse_args()


def enhance_image(path: Path, output_path: Path, scale: float, clahe_clip: float, sharpen: float, quality: int) -> None:
    image = Image.open(path)
    exif = image.info.get("exif")
    rgb = np.asarray(image.convert("RGB"))
    if scale != 1.0:
        rgb = cv2.resize(rgb, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB)
    l_channel, a_channel, b_channel = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(8, 8))
    l_channel = clahe.apply(l_channel)
    enhanced = cv2.cvtColor(cv2.merge((l_channel, a_channel, b_channel)), cv2.COLOR_LAB2RGB)

    if sharpen > 0:
        blur = cv2.GaussianBlur(enhanced, (0, 0), sigmaX=1.1)
        enhanced = cv2.addWeighted(enhanced, 1.0 + sharpen, blur, -sharpen, 0)

    output = Image.fromarray(np.clip(enhanced, 0, 255).astype(np.uint8), mode="RGB")
    save_kwargs = {"quality": quality, "subsampling": 0, "optimize": True}
    if exif:
        save_kwargs["exif"] = exif
    output.save(output_path, **save_kwargs)


def main() -> int:
    args = parse_args()
    source = args.session_dir
    target = args.output_dir
    if target.exists():
        raise SystemExit(f"Output already exists: {target}")

    image_source = source / "images"
    image_target = target / "images"
    metadata_source = source / "metadata"
    metadata_target = target / "metadata"
    image_target.mkdir(parents=True)
    metadata_target.mkdir()

    for item in ("manifest.jsonl",):
        if (source / item).exists():
            shutil.copy2(source / item, target / item)
    if metadata_source.exists():
        for metadata_path in metadata_source.glob("*.json"):
            shutil.copy2(metadata_path, metadata_target / metadata_path.name)

    images = sorted(image_source.glob("*.jpg"))
    if not images:
        raise SystemExit(f"No JPG images found under {image_source}")
    for index, image_path in enumerate(images, start=1):
        enhance_image(
            image_path,
            image_target / image_path.name,
            scale=args.scale,
            clahe_clip=args.clahe_clip,
            sharpen=args.sharpen,
            quality=args.quality,
        )
        if index % 20 == 0 or index == len(images):
            print(f"[prep] {index}/{len(images)} images")

    print(f"[prep] output: {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
