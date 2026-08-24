"""Extract walls, openings, levels, materials, and units from an IFC file.

Pure IfcOpenShell + numpy; no `pxr`/USD dependency (see usd_export.py for the
USD-conversion counterpart, which needs the conda `base` environment).
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import ifcopenshell
import ifcopenshell.geom
import ifcopenshell.util.element
import ifcopenshell.util.placement
import ifcopenshell.util.unit
import numpy as np

_EXTERNAL_NAME_HINTS = ("ext", "aussen", "außen", "exterior")
_INTERNAL_NAME_HINTS = ("int", "innen", "interior")


@dataclass
class IfcUnits:
    length_unit_name: str
    scale_to_m: float


@dataclass
class IfcLevel:
    name: str
    elevation_m: float


@dataclass
class IfcMaterialInfo:
    kind: str  # "single", "layer_set", "constituent_set", "unknown"
    names: list[str] = field(default_factory=list)
    layer_thicknesses_m: list[float] = field(default_factory=list)


@dataclass
class IfcBoundingBox:
    min_m: list[float]
    max_m: list[float]


@dataclass
class IfcWallInfo:
    guid: str
    name: str | None
    is_external: bool | None
    level_name: str | None
    material: IfcMaterialInfo | None
    bbox_m: IfcBoundingBox


@dataclass
class IfcOpeningInfo:
    guid: str
    name: str | None
    kind: str  # "window" or "door"
    host_wall_guid: str | None
    level_name: str | None
    bbox_m: IfcBoundingBox


@dataclass
class IfcRoofInfo:
    guid: str
    name: str | None
    level_name: str | None
    material: IfcMaterialInfo | None
    bbox_m: IfcBoundingBox


@dataclass
class IfcExtractionResult:
    source_path: str
    units: IfcUnits
    levels: list[IfcLevel]
    walls: list[IfcWallInfo]
    windows: list[IfcOpeningInfo]
    doors: list[IfcOpeningInfo]
    roofs: list[IfcRoofInfo]


def _geom_settings() -> "ifcopenshell.geom.settings":
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)
    return settings


def _bbox_m(element, settings, scale_to_m: float) -> IfcBoundingBox:
    shape = ifcopenshell.geom.create_shape(settings, element)
    verts = np.array(shape.geometry.verts, dtype=np.float64).reshape(-1, 3) * scale_to_m
    return IfcBoundingBox(min_m=verts.min(axis=0).tolist(), max_m=verts.max(axis=0).tolist())


def _level_name(element) -> str | None:
    container = ifcopenshell.util.element.get_container(element)
    if container is not None and container.is_a("IfcBuildingStorey"):
        return container.Name
    return None


def _is_external(wall) -> bool | None:
    """Pset_WallCommon.IsExternal when the authoring tool set it; otherwise a
    name-based heuristic (many real-world exports, e.g. this ArchiCAD sample,
    never populate the property). Returns None if neither resolves it."""
    psets = ifcopenshell.util.element.get_psets(wall)
    is_external = psets.get("Pset_WallCommon", {}).get("IsExternal")
    if isinstance(is_external, bool):
        return is_external
    name = (wall.Name or "").lower()
    if any(hint in name for hint in _EXTERNAL_NAME_HINTS):
        return True
    if any(hint in name for hint in _INTERNAL_NAME_HINTS):
        return False
    return None


def _material_info(element) -> IfcMaterialInfo | None:
    material = ifcopenshell.util.element.get_material(element)
    if material is None:
        return None
    if material.is_a("IfcMaterialLayerSetUsage"):
        layer_set = material.ForLayerSet
        names, thicknesses = [], []
        for layer in layer_set.MaterialLayers or []:
            names.append(layer.Material.Name if layer.Material else "unknown")
            thicknesses.append(float(layer.LayerThickness))
        return IfcMaterialInfo(kind="layer_set", names=names, layer_thicknesses_m=thicknesses)
    if material.is_a("IfcMaterialConstituentSet"):
        names = [
            c.Material.Name
            for c in (material.MaterialConstituents or [])
            if c.Material is not None
        ]
        return IfcMaterialInfo(kind="constituent_set", names=names)
    if material.is_a("IfcMaterial"):
        return IfcMaterialInfo(kind="single", names=[material.Name])
    return IfcMaterialInfo(kind="unknown", names=[])


def _is_roof_slab(slab) -> bool:
    """IFC has no dedicated IfcRoof in many real exports (this sample included) — roofs are
    IfcSlab with PredefinedType ROOF, falling back to a name hint ("Dach" = German for roof)."""
    if getattr(slab, "PredefinedType", None) == "ROOF":
        return True
    name = (slab.Name or "").lower()
    return "dach" in name or "roof" in name


def _host_wall_guid(opening_filler) -> str | None:
    for rel in getattr(opening_filler, "FillsVoids", []) or []:
        opening = rel.RelatingOpeningElement
        for rel2 in getattr(opening, "VoidsElements", []) or []:
            host = rel2.RelatingBuildingElement
            if host is not None:
                return host.GlobalId
    return None


def extract_ifc(path: str | Path) -> IfcExtractionResult:
    path = Path(path)
    ifc_file = ifcopenshell.open(str(path))
    settings = _geom_settings()

    scale_to_m = float(ifcopenshell.util.unit.calculate_unit_scale(ifc_file))
    length_unit = ifcopenshell.util.unit.get_project_unit(ifc_file, "LENGTHUNIT")
    length_unit_name = getattr(length_unit, "Name", None) or type(length_unit).__name__
    units = IfcUnits(length_unit_name=str(length_unit_name), scale_to_m=scale_to_m)

    levels = [
        IfcLevel(name=storey.Name, elevation_m=float(storey.Elevation or 0.0) * scale_to_m)
        for storey in ifc_file.by_type("IfcBuildingStorey")
    ]

    walls: list[IfcWallInfo] = []
    for wall in ifc_file.by_type("IfcWall"):
        walls.append(
            IfcWallInfo(
                guid=wall.GlobalId,
                name=wall.Name,
                is_external=_is_external(wall),
                level_name=_level_name(wall),
                material=_material_info(wall),
                bbox_m=_bbox_m(wall, settings, scale_to_m),
            )
        )
    if not walls:
        raise ValueError(f"no IfcWall elements found in {path}")

    def _openings(ifc_type: str, kind: str) -> list[IfcOpeningInfo]:
        result = []
        for element in ifc_file.by_type(ifc_type):
            result.append(
                IfcOpeningInfo(
                    guid=element.GlobalId,
                    name=element.Name,
                    kind=kind,
                    host_wall_guid=_host_wall_guid(element),
                    level_name=_level_name(element),
                    bbox_m=_bbox_m(element, settings, scale_to_m),
                )
            )
        return result

    windows = _openings("IfcWindow", "window")
    doors = _openings("IfcDoor", "door")

    roofs = [
        IfcRoofInfo(
            guid=slab.GlobalId,
            name=slab.Name,
            level_name=_level_name(slab),
            material=_material_info(slab),
            bbox_m=_bbox_m(slab, settings, scale_to_m),
        )
        for slab in ifc_file.by_type("IfcSlab")
        if _is_roof_slab(slab)
    ]

    return IfcExtractionResult(
        source_path=str(path),
        units=units,
        levels=levels,
        walls=walls,
        windows=windows,
        doors=doors,
        roofs=roofs,
    )


def to_json(result: IfcExtractionResult) -> dict[str, Any]:
    return asdict(result)


def export_json(result: IfcExtractionResult, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fp:
        json.dump(to_json(result), fp, indent=2)
