#!/usr/bin/env python3
"""Convert the local Gocator 2690 STEP CAD to a visual-only USD asset.

The generated asset is in meters and uses the same local scanner convention as
the rest of the Isaac scene:

    X: horizontal profile direction across the facade
    Y: robot travel / profile stacking direction
    Z: nominal measurement direction from sensor toward the facade

Run with the conda environment that has cadquery + usd-core:

    conda run -n base python assets/cad/sensors/gocator2690/convert_gocator2690.py
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
SOURCE_STEP = PROJECT_ROOT / "gocator2690/gocator2690.step"
LOCAL_STEP = ASSETS_DIR / "gocator2690.step"
OUT_USD = ASSETS_DIR / "gocator2690_visual.usd"
OUT_JSON = ASSETS_DIR / "gocator2690.json"

MM_TO_M = 0.001


def _read_binary_stl(path: Path):
    raw = path.read_bytes()
    n_tri = struct.unpack_from("<I", raw, 80)[0]
    verts, counts, indices = [], [], []
    offset = 84
    for _ in range(n_tri):
        offset += 12
        for _vertex in range(3):
            raw_x, raw_y, raw_z = struct.unpack_from("<fff", raw, offset)
            # Canonical sensor frame:
            # STEP +Y is the optical/measurement direction, STEP +X is kept as
            # profile direction, STEP +Z becomes stacking direction.
            verts.append(Gf.Vec3f(
                raw_x * MM_TO_M,
                raw_z * MM_TO_M,
                raw_y * MM_TO_M,
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
        raise FileNotFoundError(f"missing Gocator STEP source: {SOURCE_STEP}")
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
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

    root = UsdGeom.Xform.Define(stage, "/Gocator2690_visual")
    looks = UsdGeom.Scope.Define(stage, "/Gocator2690_visual/Looks")
    material = UsdShade.Material.Define(stage, looks.GetPath().AppendChild("DefaultMaterial"))
    shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("PreviewSurface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.08, 0.10, 0.12))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.42)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.2)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    mesh = UsdGeom.Mesh.Define(stage, "/Gocator2690_visual/mesh")
    mesh.CreatePointsAttr(Vt.Vec3fArray(verts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(face_counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(face_indices))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr(Vt.Vec3fArray([bbox_min, bbox_max]))
    mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.08, 0.10, 0.12)]))
    UsdShade.MaterialBindingAPI(mesh).Bind(material)

    stage.SetDefaultPrim(root.GetPrim())
    stage.GetRootLayer().Save()

    # From the drawing package: scan width up to 2000 mm, CD 325 mm, MR 1550 mm.
    # Nominal center of measurement range is therefore 325 + 1550/2 = 1100 mm.
    meta = {
        "model": "Gocator 2690",
        "source_step": str(LOCAL_STEP.relative_to(PROJECT_ROOT)),
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
            "asset_z": "STEP +Y",
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
            "x": 0.055,
            "y": 0.105,
            "z": 0.291,
        },
        "scanner_frame_from_housing": {
            "translation_m": [0.0, 0.0, 0.0],
            "rpy_deg": [0.0, 0.0, 0.0],
            "basis": "X profile, Y stacking, Z measurement; origin at sensor optical frame. Measurement volume starts at clearance_distance_mm and ends at clearance_distance_mm + z_measurement_range_mm.",
        },
        "datasheet_values": {
            "x_fov_near_mm": 385,
            "x_fov_far_mm": 2000,
            "z_measurement_range_mm": 1550,
            "clearance_distance_mm": 325,
            "nominal_standoff_mm": 1100,
        },
    }
    OUT_JSON.write_text(json.dumps(meta, indent=2), encoding="utf-8")

    print(f"mesh: {len(verts)} verts, {len(face_counts)} tris")
    print(f"USD written: {OUT_USD}")
    print(f"JSON written: {OUT_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
