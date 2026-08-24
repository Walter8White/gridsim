#!/usr/bin/env python3
"""Convert the local FZK-Haus IFC sample to extracted JSON + a visual USD asset.

Run with the conda environment that has ifcopenshell + usd-core:

    conda run -n base python assets/facades/fzk_haus/convert_fzk_haus.py
"""

from __future__ import annotations

import sys
from pathlib import Path

ASSETS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = ASSETS_DIR.parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from gridsim_ifc import export_json, extract_ifc  # noqa: E402
from gridsim_ifc.usd_export import build_ifc_usd  # noqa: E402

SOURCE_IFC = ASSETS_DIR / "AC20-FZK-Haus.ifc"
OUT_JSON = ASSETS_DIR / "fzk_haus_extracted.json"
OUT_USD = ASSETS_DIR / "fzk_haus_visual.usd"


def main() -> None:
    result = extract_ifc(SOURCE_IFC)
    export_json(result, OUT_JSON)
    print(
        f"[fzk_haus] extracted {len(result.walls)} walls, {len(result.windows)} windows, "
        f"{len(result.doors)} doors, {len(result.levels)} levels -> {OUT_JSON}"
    )
    build_ifc_usd(SOURCE_IFC, OUT_USD)


if __name__ == "__main__":
    main()
