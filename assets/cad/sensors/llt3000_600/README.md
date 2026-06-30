# scanCONTROL LLT3000-600 Visual Asset

This directory stores the official Micro-Epsilon `LLT30xx-600.STEP` CAD model
converted to a visual-only USD asset for Isaac Sim scene inspection.

The USD visual mesh is intentionally not used for PhysX collision.  The scene
creates a separate simplified collision box under:

`/World/Robot/scanner_mount_link/LLT3000_600/collision/body`

## Files

- `LLT30xx-600.STEP`: official STEP source from Micro-Epsilon CAD download.
- `llt3000_600_visual.usd`: reusable visual USD, converted to meters.
- `llt3000_600.json`: scale, metadata, datasheet values, and frame convention.
- `convert_llt3000_600.py`: repeatable STEP-to-USD conversion script.

## Axis Convention

The converted asset uses a sensor-local frame:

- `X`: horizontal profile direction across the facade
- `Y`: robot travel / profile stacking direction
- `Z`: nominal measurement direction from sensor toward the facade

The STEP axes are mapped as:

- asset `X` = STEP `+X`
- asset `Y` = STEP `+Z`
- asset `Z` = STEP `-Y`

## Scanner Frame

`scanner_frame` is placed at the nominal middle of the measuring range from the
LLT30x0-600 datasheet:

- transform from sensor housing frame: translation `[0.0, 0.0, 0.770]` m
- rotation: `[0.0, 0.0, 0.0]` deg in the sensor-local axis convention

This corresponds to the documented MMR of `770 mm`, with SMR `530 mm`, EMR
`1010 mm`, and a nominal Z measuring range of `480 mm`.
