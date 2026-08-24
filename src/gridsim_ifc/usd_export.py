"""Convert IFC walls/windows/doors/roofs into a visual-only USD asset for Isaac Sim.

Needs `pxr` (usd-core) in addition to `ifcopenshell` — kept out of extract.py so
the pure-extraction path never requires it. Mirrors the per-asset conversion
scripts under assets/cad/sensors/*/convert_*.py (run via the conda `base` env
that has cadquery + usd-core).
"""

from __future__ import annotations

import re
from pathlib import Path

import ifcopenshell
import ifcopenshell.geom
import ifcopenshell.util.unit
import numpy as np
from pxr import Gf, Usd, UsdGeom, Vt

from .extract import _is_roof_slab

_CATEGORY_COLORS = {
    "walls": (0.75, 0.75, 0.72),
    "windows": (0.55, 0.75, 0.90),
    "doors": (0.45, 0.30, 0.20),
    "roofs": (0.55, 0.25, 0.20),
}


def _sanitize_prim_name(guid: str) -> str:
    """IFC GUIDs can contain characters (e.g. '$') that aren't valid USD prim names."""
    name = re.sub(r"[^0-9a-zA-Z_]", "_", guid)
    if name[:1].isdigit():
        name = f"_{name}"
    return name


def _geom_settings() -> "ifcopenshell.geom.settings":
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)
    return settings


def _add_mesh(stage, path: str, verts_m: np.ndarray, faces: np.ndarray, color: tuple[float, float, float]) -> None:
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in verts_m]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces.ravel().tolist()))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    mesh.CreateDoubleSidedAttr(True)


def build_ifc_usd(
    ifc_path: str | Path,
    out_usd_path: str | Path,
    *,
    root_path: str = "/World/ifc_facade",
) -> Path:
    """Tessellate every IfcWall/IfcWindow/IfcDoor and write one USD mesh per element."""
    ifc_path = Path(ifc_path)
    out_usd_path = Path(out_usd_path)
    out_usd_path.parent.mkdir(parents=True, exist_ok=True)

    ifc_file = ifcopenshell.open(str(ifc_path))
    settings = _geom_settings()
    scale_to_m = float(ifcopenshell.util.unit.calculate_unit_scale(ifc_file))

    stage = Usd.Stage.CreateNew(str(out_usd_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root_prim = stage.DefinePrim(root_path, "Xform")
    stage.SetDefaultPrim(root_prim)

    element_groups = (
        ("walls", ifc_file.by_type("IfcWall")),
        ("windows", ifc_file.by_type("IfcWindow")),
        ("doors", ifc_file.by_type("IfcDoor")),
        ("roofs", [slab for slab in ifc_file.by_type("IfcSlab") if _is_roof_slab(slab)]),
    )
    written = 0
    skipped = 0
    for category, elements in element_groups:
        color = _CATEGORY_COLORS[category]
        for element in elements:
            try:
                shape = ifcopenshell.geom.create_shape(settings, element)
            except RuntimeError:
                skipped += 1
                continue
            verts = np.array(shape.geometry.verts, dtype=np.float64).reshape(-1, 3) * scale_to_m
            faces = np.array(shape.geometry.faces, dtype=np.int64).reshape(-1, 3)
            if verts.size == 0 or faces.size == 0:
                skipped += 1
                continue
            prim_name = _sanitize_prim_name(element.GlobalId)
            _add_mesh(stage, f"{root_path}/{category}/{prim_name}", verts, faces, color)
            written += 1

    stage.GetRootLayer().Save()
    print(f"[gridsim_ifc] wrote {written} elements ({skipped} skipped, no geometry) to {out_usd_path}")
    return out_usd_path
