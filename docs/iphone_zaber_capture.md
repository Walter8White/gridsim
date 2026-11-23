# iPhone + Zaber capture

`scripts/iphone_zaber_capture.py` moves the vertical Zaber stage, waits for it
to stop, reads its actual position, and asks the GridCapture iPhone app to take
one photo. The stage advances only after the iPhone confirms that the JPEG and
JSON metadata were saved.

## Linux setup

```bash
cd ~/deploya/gridsim
source .venv/bin/activate
python3 -m pip install websockets zaber-motion
hostname -I
```

The iPhone and Linux workstation must be on the same local network. Enter the
Linux IPv4 address shown by `hostname -I` in GridCapture, without `http://`.

## Safe connection test (no Zaber movement)

Start the server first:

```bash
python3 scripts/iphone_zaber_capture.py \
  --dry-run \
  --start-mm 0 \
  --end-mm 2 \
  --step-mm 1
```

Then open GridCapture, enter the Linux IP, press **Connect**, and approve the
local-network permission. Three captures should be saved with simulated
positions 0, 1, and 2 mm. Each JPEG and JSON file is transferred to Linux and
verified with SHA-256 before the stage is allowed to continue.

## First real test

Read the current position without moving:

```bash
python3 scripts/zaber_stage.py \
  --port /dev/ttyUSB0 \
  --protocol ascii \
  --command position
```

For a small upward test beginning at the current position, replace `CURRENT`
below with the position printed above plus 2 mm:

```bash
python3 scripts/iphone_zaber_capture.py \
  --zaber-port /dev/ttyUSB0 \
  --start-current \
  --end-mm CURRENT \
  --step-mm 1 \
  --velocity-mm-s 1 \
  --settle-s 0.5
```

The script never homes the stage. Press `Ctrl-C` to stop the server. Zaber
device limits remain authoritative. Linux creates one directory per session:

```text
captures/iphone_zaber/session_<timestamp>/
├── images/
├── metadata/
└── manifest.jsonl
```

The JPEG and per-photo JSON also remain under
`On My iPhone/GridCapture/Captures` as a local backup.

## Manual camera controls and photogrammetry passes

On branch `iphone-pix4d`, open **Camera settings** in GridCapture:

- Choose the physical rear **Wide / main** or **Ultra-wide** camera, when present.
  There is no synthetic 2× camera or digital zoom; capture stays at 1× on the
  selected physical lens.
- The resolution picker enumerates actual `supportedMaxPhotoDimensions` across
  the device formats. Selecting a size selects a compatible active format and
  configures both `AVCapturePhotoOutput.maxPhotoDimensions` and the settings for
  each capture. Swipe the resolution wheel up or down to choose a supported size.
- Choose continuous autofocus, tap the preview to focus on the wall, lock focus,
  or use the lens-position slider if supported. Exposure offers auto/lock,
  ISO/shutter controls when custom exposure is supported, and EV compensation.
  White balance offers auto/lock.
- **Photogrammetry Mode** selects wide and the maximum supported size, enables
  automatic adjustment, waits for a stable interval, then locks focus, exposure
  and white balance. Aim at the wall before pressing it. A timeout leaves an
  explicit error instead of claiming a successful lock.
- To preserve manually chosen ISO, shutter and focus, use **Lock current settings
  for pass** instead. Settings and tap-to-focus are disabled while a pass is
  locked. Use **End pass / unlock controls** to adjust the next pass. Unlocking
  enables controls but preserves the current camera values until changed.
  The remote capture commands do not reset these locks between photos/passes.
- Captures use JPEG compression quality 1.0 and photo-quality priority, with
  flash off. Video stabilization is disabled; no digital zoom is requested.
  Apple's computational still-photo processing may still affect framing.

### 24 MP and actual image dimensions

Apple's 5712×4284 mode requires deferred photo processing through Photos. When
selected, GridCapture requests Photos read/write access, creates a library asset
from the proxy, requests the processed full-quality data and only then saves and
uploads the final JPEG. The Photos copy remains in the library. Denying access
reports an error; direct 12/48 MP capture does not require Photos access. Deferred
processing can increase capture latency; increase `--capture-timeout-s` on Linux
if needed. Do not treat the proxy as a completed full-resolution photo.

AVFoundation specifies a maximum, not a guaranteed final size. The overlay shows
actual saved dimensions and capture latency after each shot and reports a size
mismatch. Validate full-resolution output on the target iPhone in the intended
lighting before a scan.

### JSON sidecar

Existing top-level capture/position/motion fields remain compatible. Added
`pass_index` and `x_offset_cm` are copied from the multi-pass command (with
`x_offset_m` converted to cm when necessary); they are absent when unknown.
`targetPositionMM` and `measuredPositionMM` retain the Zaber target/readback.

The nested `camera` object contains `selected_camera`, `camera_device_type`,
`selected_resolution`, `maxPhotoDimensions`, `codec`, `lens_position`,
`focus_mode`, `exposure_mode`, `exposure_duration` (seconds), `ISO`, `aperture`,
`focal_length` (EXIF, mm, when supplied), `white_balance_mode`, RGB
`white_balance_gains`, `white_balance_temperature`, `white_balance_tint`,
`image_width`, `image_height`, EXIF `orientation`, `timestamp_unix` (fractional
seconds), `photo_timestamp_seconds` (sensor host clock), and
`capture_time_seconds` (request through final-image processing).
Image dimensions, orientation, aperture, focal length, shutter and ISO are read
from the final image when present; other camera values are sampled at capture.

### Device acceptance check

1. On a physical iPhone, check each available camera and size, including Photos
   approval/denial for 24 MP; compare the final JPEG and JSON dimensions.
2. Aim at a textured wall, exercise tap/manual focus and manual ISO/shutter,
   then lock the current pass. Capture multiple positions and confirm unchanged
   lens/exposure/WB values and correct per-photo Zaber/pass/offset metadata.
3. End the pass, apply Photogrammetry Mode and confirm wide/max resolution and
   all three locks before starting the Linux scan.
4. Check portrait/landscape images against EXIF orientation, then verify upload
   hashes and a processing failure/timeout before a production session.

Reference: [Apple maxPhotoDimensions documentation](https://developer.apple.com/documentation/avfoundation/avcapturephotooutput/maxphotodimensions)
(and the installed AVFoundation SDK headers for the 24 MP deferred-delivery requirement).

### Settings preview

A small live preview of the active capture camera stays at the top right of
Camera settings while the controls scroll beneath it. Tap that preview to focus
when the pass is unlocked. The full-screen preview resumes when settings close.
The IMU readout and camera-format catalogue are hidden from the interface;
motion samples continue to be recorded in the photo metadata.

### Focus lock and compact status bar

The main screen and Focus settings share one switch: red **Lock focus** means
locked, green **Autofocus** means continuous autofocus. Both the full-screen
preview and the settings preview ignore taps while focus is locked. Unlocking
explicitly returns to continuous autofocus; a tap then updates its target point.
A locked photogrammetry pass disables the switch until **End pass / unlock
controls** is pressed. Manual lens positioning also puts the camera into locked
focus, which is reflected by the switch.

The top status pills show the active camera, requested megapixels and pixel
dimensions, lens position, ISO, shutter, exposure mode, white balance, and (after
capture) saved dimensions and elapsed capture time. Pills wrap to fit the screen.
The megapixel value is rounded from the real pixel dimensions, not hard-coded.

LiDAR hardware discovery is isolated in `LiDARService`: it checks the rear
`builtInLiDARDepthCamera` and supplies the status and LiDAR settings section.
**No LiDAR** has a red dot; detected hardware has a green dot. Detection does
not add an input/output to the capture session, start depth capture, or write
any LiDAR fields into the photo metadata.

Device validation: lock focus, tap both previews, and confirm the lens position
stays fixed; unlock and tap again to check autofocus resumes. Repeat after a
manual lens adjustment and a locked pass. Check status wrapping in portrait and
landscape, and LiDAR status on phones with and without the sensor.

Exposure and white balance use the same red locked / green automatic switches
in the main header and settings. Unlocking resumes continuous auto exposure or
continuous auto white balance. Custom ISO/shutter is shown as **Exp manual**
with the exposure switch on (red); switching it off returns to auto exposure.
Exposure compensation is disabled while exposure is locked/manual. Deliberate
manual ISO/shutter adjustments remain available outside a locked pass. Preview
taps only adjust focus and never unlock exposure or white balance. All three
switches remain disabled for a locked pass until the pass is ended.

On-device check: lock exposure and WB, change lighting and tap both previews;
verify ISO/shutter and WB gains stay fixed. Unlock each independently and verify
its automatic adjustment resumes, including after manual ISO/shutter settings.

### Main-screen layout and app icon

Camera status, pixel dimensions and the settings gear occupy the top row. The
gear at the top right opens Camera settings. Focus, Exposure and WB sit in three
equal-width columns, followed by a two-row grid for MP, ISO, shutter, lens,
LiDAR and capture duration. There is no GridCapture wordmark or separate text
button for settings on the camera screen, and pixel dimensions have no “Saved”
prefix.

The iOS AppIcon asset uses `AppIcon-1024.png`, an opaque square adaptation of the
provided déploya :robotics logo. Generated with the built-in imagegen tool, then
resized to the required 1024×1024 asset dimensions. Prompt: preserve the red
two-line wordmark and white background, center the entire composition on a
square canvas with margins, and add no decorations or rounded corners.
