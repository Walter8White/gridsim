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
        height_path = meta_path.parent / Path(height_path).name
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


def _slice_from_mm(min_mm, max_mm, pitch_mm, count, axis_name):
    start = 0 if min_mm is None else int(np.floor(min_mm / pitch_mm))
    stop = count if max_mm is None else int(np.ceil(max_mm / pitch_mm)) + 1
    start = max(0, min(count, start))
    stop = max(start, min(count, stop))
    if stop <= start:
        raise SystemExit(f"Empty {axis_name} crop: {min_mm}..{max_mm} mm")
    return start, stop


def build_grid_mesh(
    valid,
    z,
    x_pitch_mm,
    y_pitch_mm,
    stride,
    x_min_mm=None,
    x_max_mm=None,
    y_min_mm=None,
    y_max_mm=None,
    max_z_jump_mm=None,
):
    raw_height, raw_width = z.shape
    x0, x1 = _slice_from_mm(x_min_mm, x_max_mm, x_pitch_mm, raw_width, "X")
    y0, y1 = _slice_from_mm(y_min_mm, y_max_mm, y_pitch_mm, raw_height, "Y")

    valid_crop = valid[y0:y1, x0:x1]
    z_crop = z[y0:y1, x0:x1]
    valid_s = valid_crop[::stride, ::stride]
    z_s = z_crop[::stride, ::stride]
    height, width = z_s.shape

    vertex_index = -np.ones((height, width), dtype=np.int32)
    ys, xs = np.where(valid_s)
    coords = np.empty((len(xs), 3), dtype=np.float32)
    coords[:, 0] = (x0 + xs * stride) * x_pitch_mm
    coords[:, 1] = (y0 + ys * stride) * y_pitch_mm
    coords[:, 2] = z_s[ys, xs]
    vertex_index[ys, xs] = np.arange(len(xs), dtype=np.int32)

    def accept_face(a, b, c):
        if a < 0 or b < 0 or c < 0:
            return False
        if max_z_jump_mm is None:
            return True
        face_z = coords[[a, b, c], 2]
        return float(face_z.max() - face_z.min()) <= max_z_jump_mm

    faces = []
    for y in range(height - 1):
        for x in range(width - 1):
            v00 = vertex_index[y, x]
            v10 = vertex_index[y, x + 1]
            v01 = vertex_index[y + 1, x]
            v11 = vertex_index[y + 1, x + 1]
            if accept_face(v00, v10, v01):
                faces.append((v00, v10, v01))
            if accept_face(v10, v11, v01):
                faces.append((v10, v11, v01))

    return coords, faces, width, height


def default_output(meta_path, suffix):
    prefix = meta_path.with_name(meta_path.name.replace("_meta.csv", ""))
    return prefix.with_name(prefix.name + suffix)
