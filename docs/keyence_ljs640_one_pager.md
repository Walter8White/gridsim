# Keyence LJ-S640 - Quick Assessment

## Why It Is Interesting

The LJ-S640 is a strong option for high-resolution local 3D metrology. When the surface reflects the laser correctly, it gives dense, repeatable height data that is much more useful for dimensional facade scanning than RGB vision alone.

Good at:

- matte walls, painted surfaces, flat panels;
- dense local geometry and edge/profile measurement;
- repeated scans at the same pose;
- multi-preset acquisition, since scans taken without moving the Zaber are directly stackable.

## Main Problems

The limitation is not raw resolution. The limitation is robustness across real facade materials.

Hard cases:

- black/dark surfaces: weak return;
- steel/chrome handles: specular reflection, saturation, invalid pixels;
- glass: reflections/transmission, often unreliable;
- sharp edges/recesses: geometric occlusion and dead zones;
- strong ambient light or shadows: unstable optimal settings.

Dead zones are especially important: the sensor may simply not see a valid laser return near edges, corners, or occluded areas. Interpolation can make the image look better, but it may invent geometry, so it should not be blindly trusted for dimensional measurement.

## Distance Constraint

The current setup should stay close to **1.1 m from the wall**. There is not much margin.

Changing distance changes:

- effective profile width;
- valid measurement range;
- overlap/step size;
- invalid pixel rate;
- dead-zone behavior.

For full facade scanning, distance should be measured continuously or at least regularly, and the Zaber step should depend on the effective profile width.

## Useful Parameters

- `exposure`: main control. High helps dark surfaces, low helps shiny metal/glass.
- `dynamic_range`: helps mixed scenes; use high values for black + shiny + bright zones.
- `light_mode`: manual/auto/slope illumination behavior. Manual is more repeatable.
- `light_upper`, `light_lower`: laser/light intensity limits. Lower values may help reflective metal.
- `detection_sensitivity`: higher detects weaker returns, but may accept more noise.
- `dead_zone_interpolation`: fills holes visually, but can fake geometry.
- `peak_width_filter`: can reject bad/specular peaks; useful on metal, risky on weak dark returns.
- `x_subsample`, `y_subsample`: reduce resolution; keep at 1 for accurate dimensional scans.

## Practical Strategy

A single Keyence preset will not work for a whole facade.

Recommended path:

1. Use a camera or simple vision module to detect which surface types are present in the next pass: normal, dark, shiny/metal/glass-like.
2. At the same Zaber pose, run 1 to N Keyence scans with different presets.
3. Locally fuse the scans: keep valid pixels and prefer the scan with the best quality score.
4. Move the Zaber and repeat.
5. Globally assemble all fused passes.

Typical presets:

- normal: exposure 11-12, dynamic range 6-8;
- dark: exposure 14-15, dynamic range 8-9;
- shiny/metal: exposure 7-10, dynamic range 9, peak filter on.

## Bottom Line

The LJ-S640 is promising if treated as a precise local metrology sensor inside an adaptive acquisition loop. It is not a one-shot facade scanner with one universal setting.

The roadmap should be:

**camera/scene analysis -> preset selection -> multi-scan at fixed pose -> quality-based fusion -> Zaber move -> global stitching.**

The critical engineering work is calibration, distance control, dead-zone handling, and fusion quality, not object recognition.
