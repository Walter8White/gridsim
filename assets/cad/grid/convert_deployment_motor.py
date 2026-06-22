#!/usr/bin/env python3
"""Convert deployment motor.step to a centered, oriented USD asset.

Long axis (X in raw STEP) is rotated to local Y to match the horizontal_rail
and vertical_grid_bar conventions.  Run with the CadQuery + usd-core conda env:

    conda run -n base python3 assets/cad/grid/convert_deployment_motor.py
"""

import json
import struct
import tempfile
from pathlib import Path

import cadquery as cq

ASSETS_DIR = Path(__file__).parent
STEP_PATH = ASSETS_DIR / "deployment motor.step"
OUT_USD = ASSETS_DIR / "deployment_motor.usd"
OUT_JSON = ASSETS_DIR / "deployment_motor.json"

# ── 1. load & inspect ────────────────────────────────────────────────────────

shape = cq.importers.importStep(str(STEP_PATH))
bb = shape.val().BoundingBox()
cx = (bb.xmin + bb.xmax) / 2
cy = (bb.ymin + bb.ymax) / 2
cz = (bb.zmin + bb.zmax) / 2
dx = bb.xmax - bb.xmin   # long axis (X) = 2000 mm
dy = bb.ymax - bb.ymin   # 117.250 mm
dz = bb.zmax - bb.zmin   # 106.000 mm

print(f"raw bbox: dx={dx:.3f}  dy={dy:.3f}  dz={dz:.3f}  center=({cx:.3f},{cy:.3f},{cz:.3f})")

# ── 2. center at origin then rotate X→Y (Rz +90°) ───────────────────────────

centered = (
    shape
    .translate((-cx, -cy, -cz))
    # Rz(+90°) maps +X → +Y, so the 2000 mm long axis becomes local Y
    .rotate((0, 0, 0), (0, 0, 1), 90)
)

bb2 = centered.val().BoundingBox()
print(f"after transform: x={bb2.xmin:.3f}..{bb2.xmax:.3f}  "
      f"y={bb2.ymin:.3f}..{bb2.ymax:.3f}  "
      f"z={bb2.zmin:.3f}..{bb2.zmax:.3f}")

# ── 3. export to STL (binary) ────────────────────────────────────────────────

with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
    stl_path = Path(tmp.name)

cq.exporters.export(centered, str(stl_path), exportType="STL")
print(f"STL written: {stl_path}  ({stl_path.stat().st_size // 1024} KB)")

# ── 4. STL → USD mesh via pxr ────────────────────────────────────────────────

from pxr import Sdf, Usd, UsdGeom, UsdShade, Gf, Vt   # noqa: E402  (needs usd-core)


def _read_binary_stl(path: Path):
    """Return (vertices, face_vertex_counts, face_indices) lists."""
    raw = path.read_bytes()
    n_tri = struct.unpack_from("<I", raw, 80)[0]
    verts, counts, indices = [], [], []
    offset = 84
    for _ in range(n_tri):
        # skip normal (3×4 bytes), read 3 vertices (3×3×4 bytes), skip attr (2 bytes)
        offset += 12
        for v in range(3):
            x, y, z = struct.unpack_from("<fff", raw, offset)
            indices.append(len(verts))
            verts.append(Gf.Vec3f(x, y, z))
            offset += 12
        counts.append(3)
        offset += 2
    return verts, counts, indices


verts, face_counts, face_indices = _read_binary_stl(stl_path)
stl_path.unlink()
print(f"mesh: {len(verts)} verts, {len(face_counts)} tris")

stage = Usd.Stage.CreateNew(str(OUT_USD))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)   # match other CAD assets
stage.SetMetadata("metersPerUnit", 0.01)

root = UsdGeom.Xform.Define(stage, "/node_STL_BINARY_")
looks = UsdGeom.Scope.Define(stage, "/node_STL_BINARY_/Looks")
material = UsdShade.Material.Define(stage, looks.GetPath().AppendChild("DefaultMaterial"))
shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("DefaultMaterial"))
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.45, 0.02))
shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.1)
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

mesh_parent = UsdGeom.Xform.Define(stage, "/node_STL_BINARY_/node_")
mesh = UsdGeom.Mesh.Define(stage, "/node_STL_BINARY_/node_/mesh_")
mesh.CreatePointsAttr(Vt.Vec3fArray(verts))
mesh.CreateFaceVertexCountsAttr(Vt.IntArray(face_counts))
mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(face_indices))
mesh.CreateSubdivisionSchemeAttr("none")
mesh.CreateDoubleSidedAttr(True)
mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.45, 0.02)]))
mesh.CreateExtentAttr(Vt.Vec3fArray([
    Gf.Vec3f(bb2.xmin, bb2.ymin, bb2.zmin),
    Gf.Vec3f(bb2.xmax, bb2.ymax, bb2.zmax),
]))
UsdShade.MaterialBindingAPI(mesh).Bind(material)

stage.SetDefaultPrim(root.GetPrim())
stage.GetRootLayer().Save()
print(f"USD written: {OUT_USD}  ({OUT_USD.stat().st_size // 1024} KB)")

# ── 5. JSON metadata ─────────────────────────────────────────────────────────

width_asset_units = bb2.xmax - bb2.xmin
length_asset_units = bb2.ymax - bb2.ymin
depth_asset_units = bb2.zmax - bb2.zmin

meta = {
    "source_step": str(STEP_PATH),
    "generated_from": "STEP via CadQuery; mesh centered, materialized, and rotated so long axis is local Y",
    "asset_units": "millimeters",
    "scene_unit_scale": 0.001,
    "long_axis": "Y",
    "length_asset_units": round(length_asset_units, 3),
    "length_m": round(length_asset_units * 0.001, 6),
    "width_m": round(width_asset_units * 0.001, 6),
    "depth_m": round(depth_asset_units * 0.001, 6),
    "bbox_asset_units": {
        "xmin": round(bb2.xmin, 3), "ymin": round(bb2.ymin, 3), "zmin": round(bb2.zmin, 3),
        "xmax": round(bb2.xmax, 3), "ymax": round(bb2.ymax, 3), "zmax": round(bb2.zmax, 3),
    },
}
OUT_JSON.write_text(json.dumps(meta, indent=2))
print(f"JSON written: {OUT_JSON}")
print("Done.")
