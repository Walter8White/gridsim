# Linux motor control station

## Launch

Use Python 3.11 or newer (the capture protocol uses `asyncio.timeout`):

```bash
cd ~/deploya/gridsim
source .venv/bin/activate
python -m pip install -e '.[station]'
python scripts/control_station.py
```

For a hardware-free motor demonstration:

```bash
python scripts/control_station.py --demo
```

Demo mode simulates all three motors; it still accepts a real GridCapture
connection for preview and photo uploads. It does not connect to hardware.

## Connect

The station scans automatically at startup and retries missing devices every
15 seconds. **Detect** scans without disconnecting devices that are already
working. The gear menu contains optional USB/BLE overrides and shows detected
addresses. **Save & detect** applies those overrides and reconnects. PINE II
is found by its BLE service; motor ports default to **1 and 2**. Close the
PINE phone app if it holds the BLE connection.

Zaber detection probes available serial ports using the ASCII protocol and
selects the first responding device's first axis. Detection does not move the
stage. If the Zaber is not already homed, its jog and recording controls stay
disabled until the user presses **Home to switch**. Homing uses the Zaber speed
field, initially **40 mm/s**, and moves the stage. Legacy Binary devices need
to be configured for ASCII first. Linux must grant your account serial-port
access and Bluetooth must be enabled.

Install Python Bonjour support through `python -m pip install -e ".[station]"`.
The Linux station advertises `_gridsim-control._tcp` using Bonjour/mDNS. Build
and install the updated `ios/GridCapture` app using Xcode on a Mac. On launch,
GridCapture searches for the station and connects automatically, retrying every
five seconds if Linux is not ready yet. Accept the iOS Local Network prompt.
Manual IP entry remains available if Bonjour discovery is blocked. Use `hostname -I` on Linux to find the IP. The station
listens on port **8765**; do not run another capture script on that port.
Use `--host` and `--port` to change the listener. Discovery requires both devices to see each other on the local network; some iPhone Personal Hotspot and client-isolation configurations block Bonjour. In that case, use the manual IP connection.

The camera panel remains visible beside both tabs. Preview uses small JPEGs
at up to five frames per second; recorded photos retain full resolution.
Frames older than two seconds are cleared and marked stale. Old app builds
can capture but cannot provide live preview. Preview is enabled only by the
station's greeting, preserving compatibility with the existing CLI scripts.

## Manual control and home

Hold the left mouse button on a dial and drag around it. Direction and distance
from centre control the signed jogging demand. Release to centre the dial and
stop requesting motion. Tab changes and application focus loss also release
all dials. Mouse wheel and keyboard input cannot latch a jog demand.

The Zaber is shown as a vertical linear slider, with **0 mm at the top** and
**1500 mm at the bottom**. Hold and drag to continuously track the slider
position with a position-feedback PID loop. On release, the last slider
setpoint stays active: the stage continues toward it and holds that position.
**STOP ALL / Esc** stops motion and clears the hold. Slider resolution is
**0.1 mm**. This assumes the stage is calibrated so its usable travel is
0–1500 mm. Each motor has its own speed field: **0.001–100 mm/s** for Zaber
and **1–100% controller speed** for each PINE motor, initially set to 1.
The PINE circular controls now drag position targets in raw controller units;
the latest target remains active after release. They send absolute `MPS`
position commands and let PINE execute each move. This is not a host-side PID:
the BLE protocol used here exposes position reads and target moves, but no
verified continuous-velocity or immediate-stop command. A move already accepted
by PINE may finish after release, disconnect, or **STOP ALL**. Keep hardware
power-off accessible.

PINE positions are read through the controller's relative raw-position
command; this is not confirmed as encoder feedback. Use **Set visual
reference** at a known physical origin; **Return to reference** sends that
position as the next target. References are valid only for the current
connection and cleared on reconnect. They cannot recover an origin after
power loss or missed steps.

## Loop demo

After both PINE motors are connected and the Zaber is homed, **LOOP DEMO**
moves the Zaber between **300 and 900 mm** at **40 mm/s**. PINE motor A moves
from its start position to +45° while motor B moves to −45°, then they swap
targets on the next leg. The cycle begins by moving to 300 mm. Configure each
PINE motor's raw position counts per full output-axis revolution in Settings;
the demo requires these values to convert degrees safely. The default of 200
counts/revolution is used only in software demo mode.

**STOP ALL** cancels the loop and stops the Zaber. PINE may finish its most
recent position target because this connection has no verified immediate-stop
command; use a low PINE speed for initial hardware trials.

Zaber's **Home to switch** uses the hardware home command at the configured
maximum speed. It does **not** subsequently move to the far limit.
**STOP ALL / Esc** cancels a scan or reference return and requests Zaber stop.
Manual motion and homing are unavailable while recording. Recording travel
speed remains separately configurable in the recording tab.

Close the window, press **Quit**, or use **Ctrl+Q** to stop motors, release
connections, and exit.

## Photo recording

Set a start position (or select current position), end position, positive
step, travel speed, settle time, upload timeout and output directory. Both
ascending and descending scans are supported. The endpoint is included even
if the step does not divide the distance. Scans over 1,000 photos are rejected.

**Start photo scan** requires the phone and Zaber connection (or demo Zaber).
For each position the station waits for motion to finish, settles, requests
a still photo, verifies the uploaded JPEG and metadata size/SHA-256, and only
then moves onward. A failed photo, timeout or disconnection stops the scan.
The camera preview is routed separately from the capture upload messages.

Files are saved as in the CLI workflow:

```text
captures/iphone_zaber/session_<timestamp>/
  images/
  metadata/
  manifest.jsonl
```

After completion or cancellation, reconnect GridCapture for the next scan;
this prevents a late upload from being mistaken for the next scan's photo.
The existing multi-pass CLI remains available for scans with manual horizontal
offsets. The desktop recording tab currently runs one vertical pass.

## Validation

`python -m pytest tests/test_control_station.py -q` exercises demand expiry,
scan validation, cancellation, and a real loopback WebSocket session with
preview frames interleaved between capture-upload messages. Motor tests use
simulation; physical BLE/USB motion and iOS camera streaming must be checked
on the actual rig after installing the app.
