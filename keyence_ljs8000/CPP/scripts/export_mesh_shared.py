from pathlib import Path

import numpy as np


def read_meta(meta_path):
    meta = {}
    for line in meta_path.read_text().splitlines()[1:]:
        if "," in line:
            key, value = line.split(",", 1)
            meta[key] = value
    return meta


def latest_meta(capture_dir):
    metas = sorted(capture_dir.glob("*_meta.csv"))
    if not metas:
        raise SystemExit(f"No *_meta.csv found in {capture_dir}")
    return metas[-1]


def load_capture(meta_path):
    meta = read_meta(meta_path)
    height_path = Path(meta["height_file"])
    if not height_path.is_absolute():
        height_path = Path.cwd() / height_path
    if not height_path.exists():
        height_path = meta_path.with_name(meta_path.name.replace("_meta.csv", "_height_u16le.raw"))
    if not height_path.exists():
        raise SystemExit(f"Height raw file not found: {height_path}")

    width = int(float(meta["x_pointnum"]))
    height = int(float(meta["y_pointnum"]))
    x_pitch_mm = float(meta["x_pitch_um"]) / 1000.0
    y_pitch_mm = float(meta["y_pitch_um"]) / 1000.0
    z_pitch_mm = float(meta["z_pitch_um"]) / 1000.0

    raw = np.fromfile(height_path, dtype="<u2")
    expected = width * height
    if raw.size != expected:
        raise SystemExit(f"Unexpected raw size {raw.size}, expected {expected}")

    raw = raw.reshape(height, width)
    z = (raw.astype(np.float32) - 32768.0) * z_pitch_mm
    valid = raw != 0
    return meta, height_path, valid, z, x_pitch_mm, y_pitch_mm


def build_grid_mesh(valid, z, x_pitch_mm, y_pitch_mm, stride):
    valid_s = valid[::stride, ::stride]
    z_s = z[::stride, ::stride]
    height, width = z_s.shape

    vertex_index = -np.ones((height, width), dtype=np.int32)
    ys, xs = np.where(valid_s)
    coords = np.empty((len(xs), 3), dtype=np.float32)
    coords[:, 0] = xs * x_pitch_mm * stride
    coords[:, 1] = ys * y_pitch_mm * stride
    coords[:, 2] = z_s[ys, xs]
    vertex_index[ys, xs] = np.arange(len(xs), dtype=np.int32)

    faces = []
    for y in range(height - 1):
        for x in range(width - 1):
            v00 = vertex_index[y, x]
            v10 = vertex_index[y, x + 1]
            v01 = vertex_index[y + 1, x]
            v11 = vertex_index[y + 1, x + 1]
            if v00 >= 0 and v10 >= 0 and v01 >= 0:
                faces.append((v00, v10, v01))
            if v10 >= 0 and v11 >= 0 and v01 >= 0:
                faces.append((v10, v11, v01))

    return coords, faces, width, height


def default_output(meta_path, suffix):
    prefix = meta_path.with_name(meta_path.name.replace("_meta.csv", ""))
    return prefix.with_name(prefix.name + suffix)
