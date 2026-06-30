# Gocator 2690 Visual Asset

This directory stores the local LMI Gocator 2690 STEP CAD converted to a
visual-only USD asset for Isaac Sim scene inspection.

The detailed USD mesh is not used for PhysX collision.  The scene creates one
simplified collision box under:

`/World/Robot/scanner_mount_link/Gocator2690/collision/body`

## Files

- `gocator2690.step`: local STEP source copied from `gocator2690/gocator2690.step`.
- `gocator2690_visual.usd`: reusable visual USD, converted to meters.
- `gocator2690.json`: scale, metadata, drawing values, and frame convention.
- `convert_gocator2690.py`: repeatable STEP-to-USD conversion script.

## Axis Convention

The converted asset uses the scanner-local convention:

- `X`: horizontal profile direction across the facade
- `Y`: robot travel / profile stacking direction
- `Z`: nominal measurement direction from sensor toward the facade

The STEP axes are mapped as:

- asset `X` = STEP `+X`
- asset `Y` = STEP `+Z`
- asset `Z` = STEP `+Y`

## Scanner Frame And Scan Volume

`scanner_frame` is the optical measurement frame, not a point on the wall:

- transform from sensor housing frame: translation `[0.0, 0.0, 0.0]` m
- rotation: `[0.0, 0.0, 0.0]` deg in the sensor-local axis convention

The scene represents the actual widening profile field under:

`/World/Robot/scanner_mount_link/Gocator2690/scan_volume`

The Gocator 2690 datasheet values are:

- clearance distance CD: `325 mm`
- measurement range MR: `1550 mm`
- near field of view: `385 mm`
- far field of view: `2000 mm`

At any wall distance `d` inside `[CD, CD + MR]`, the scan width is linearly
interpolated between `385 mm` and `2000 mm`.
