#!/usr/bin/env python3
"""Convert the official Micro-Epsilon LLT30xx-600 STEP CAD to USD.

The source CAD includes the large measurement field helper geometry.  The USD is
kept visual-only and converted to meters with a sensor-friendly local frame:

    X: laser profile direction
    Y: profile stacking / robot travel direction
    Z: nominal measurement direction away from the sensor

Run with the conda environment that has cadquery + usd-core:

    conda run -n base python assets/cad/sensors/llt3000_600/convert_llt3000_600.py
"""

from __future__ import annotations

import json
import struct
import tempfile
from pathlib import Path

import cadquery as cq
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

ASSETS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = ASSETS_DIR.parents[3]
SOURCE_STEP = PROJECT_ROOT / "source_assets/micro_epsilon/LLT30xx-600.STEP"
LOCAL_STEP = ASSETS_DIR / "LLT30xx-600.STEP"
OUT_USD = ASSETS_DIR / "llt3000_600_visual.usd"
OUT_JSON = ASSETS_DIR / "llt3000_600.json"

MM_TO_M = 0.001


def _read_binary_stl(path: Path):
    raw = path.read_bytes()
    n_tri = struct.unpack_from("<I", raw, 80)[0]
    verts, counts, indices = [], [], []
    offset = 84
    for _ in range(n_tri):
        offset += 12  # normal
        for _vertex in range(3):
            raw_x, raw_y, raw_z = struct.unpack_from("<fff", raw, offset)
            # Canonical sensor frame: raw STEP X -> X, raw STEP Z -> Y,
            # raw STEP -Y -> Z.  Convert mm to meters immediately.
            verts.append(Gf.Vec3f(
                raw_x * MM_TO_M,
                raw_z * MM_TO_M,
                -raw_y * MM_TO_M,
            ))
            indices.append(len(verts) - 1)
            offset += 12
        counts.append(3)
        offset += 2
    return verts, counts, indices


def _bbox_from_points(points: list[Gf.Vec3f]) -> tuple[Gf.Vec3f, Gf.Vec3f]:
    mins = [min(p[i] for p in points) for i in range(3)]
    maxs = [max(p[i] for p in points) for i in range(3)]
    return Gf.Vec3f(*mins), Gf.Vec3f(*maxs)


def main() -> int:
    if not SOURCE_STEP.exists():
        raise FileNotFoundError(f"missing official STEP source: {SOURCE_STEP}")
    LOCAL_STEP.write_bytes(SOURCE_STEP.read_bytes())

    shape = cq.importers.importStep(str(SOURCE_STEP))
    raw_bb = shape.val().BoundingBox()
    print(
        "raw STEP bbox mm: "
        f"x={raw_bb.xmin:.3f}..{raw_bb.xmax:.3f} "
        f"y={raw_bb.ymin:.3f}..{raw_bb.ymax:.3f} "
        f"z={raw_bb.zmin:.3f}..{raw_bb.zmax:.3f}"
    )

    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        stl_path = Path(tmp.name)

    cq.exporters.export(shape, str(stl_path), exportType="STL")
    verts, face_counts, face_indices = _read_binary_stl(stl_path)
    stl_path.unlink()
    bbox_min, bbox_max = _bbox_from_points(verts)

    stage = Usd.Stage.CreateNew(str(OUT_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetMetadata("metersPerUnit", 1.0)

    root = UsdGeom.Xform.Define(stage, "/LLT3000_600_visual")
    looks = UsdGeom.Scope.Define(stage, "/LLT3000_600_visual/Looks")
    material = UsdShade.Material.Define(stage, looks.GetPath().AppendChild("DefaultMaterial"))
    shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("PreviewSurface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.12, 0.14, 0.16))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.15)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    mesh = UsdGeom.Mesh.Define(stage, "/LLT3000_600_visual/mesh")
    mesh.CreatePointsAttr(Vt.Vec3fArray(verts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(face_counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(face_indices))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr(Vt.Vec3fArray([bbox_min, bbox_max]))
    mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.12, 0.14, 0.16)]))
    UsdShade.MaterialBindingAPI(mesh).Bind(material)

    stage.SetDefaultPrim(root.GetPrim())
    stage.GetRootLayer().Save()

    meta = {
        "model": "scanCONTROL LLT3000-600",
        "source_step": str(LOCAL_STEP.relative_to(PROJECT_ROOT)),
        "source_download": "https://www.micro-epsilon.com/fileadmin/download/cad/scanCONTROL-LLT-30xx-430-600--STEP.zip",
        "generated_usd": str(OUT_USD.relative_to(PROJECT_ROOT)),
        "units": "meters",
        "axis_convention": {
            "x": "horizontal profile direction across the facade",
            "y": "robot travel / profile stacking direction",
            "z": "nominal measurement direction from sensor toward the facade",
        },
        "cad_axis_mapping": {
            "asset_x": "STEP +X",
            "asset_y": "STEP +Z",
            "asset_z": "STEP -Y",
        },
        "bbox_m": {
            "xmin": round(float(bbox_min[0]), 6),
            "ymin": round(float(bbox_min[1]), 6),
            "zmin": round(float(bbox_min[2]), 6),
            "xmax": round(float(bbox_max[0]), 6),
            "ymax": round(float(bbox_max[1]), 6),
            "zmax": round(float(bbox_max[2]), 6),
        },
        "housing_collision_envelope_m": {
            "x": 0.4799,
            "y": 0.23,
            "z": 0.10,
        },
        "scanner_frame_from_housing": {
            "translation_m": [0.0, 0.0, 0.770],
            "rpy_deg": [0.0, 0.0, 0.0],
            "basis": "X profile, Y stacking, Z measurement; origin at nominal mid measuring range (MMR)",
        },
        "datasheet_values": {
            "x_fov_mm": 600,
            "z_measurement_range_mm": 480,
            "smr_mm": 530,
            "mmr_mm": 770,
            "emr_mm": 1010,
            "points_per_profile": 2048,
            "nominal_profile_rate_hz": 10000,
            "wavelength_nm": 658,
            "ip_rating": "IP67",
        },
    }
    OUT_JSON.write_text(json.dumps(meta, indent=2), encoding="utf-8")

    print(f"mesh: {len(verts)} verts, {len(face_counts)} tris")
    print(f"USD written: {OUT_USD}")
    print(f"JSON written: {OUT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
