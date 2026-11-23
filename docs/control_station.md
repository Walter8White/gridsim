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

Press **Detect connections**. PINE is discovered by its BLE service; the two
motor ports default to **1 and 3**, matching the existing scripts. You can
change them before detection. If several controllers are present, enter the
BLE address explicitly. Close the PINE phone app if it holds the connection.

Zaber detection probes serial ports using the ASCII protocol, skips CP210
adapters normally used by PINE, and selects the first responding device's
first axis. Specify a USB path if necessary, including to override the CP210
filter. Legacy Binary devices need to be configured for ASCII first. Linux
must grant your account serial-port access and Bluetooth must be enabled.
Detection never intentionally moves motors.

Build and install the updated `ios/GridCapture` app using Xcode on a Mac.
Connect the iPhone and workstation to the same local network. In GridCapture,
enter the workstation IP and press **Connect**, accepting camera and local
network permissions. Use `hostname -I` on Linux to find the IP. The station
listens on port **8765**; do not run another capture script on that port.
Use `--host` and `--port` to change the listener. This is a local network
connection, not automatic camera access over an iPhone USB cable.

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

Each motor has its own speed field: **0.001–100 mm/s** for Zaber and
**1–100% controller speed** for each PINE motor, initially set to 1. The Zaber
dial scales the selected speed by its deflection. PINE uses bounded
**1–10 raw-position-unit** steps at the selected controller speed; the small
steps and BLE exchange rate also limit its effective travel speed. PINE's last small step may complete after
release or a BLE disconnect; no undocumented velocity/stop command is sent.
The controller's existing motor configuration is used; raw units have not
been calibrated to degrees. Demand expires after 400 ms without a GUI
heartbeat. Software stops depend on a functioning OS and hardware link;
keep the hardware power-off accessible.

The PINE motors have no home switches. **Set reference here** records the
current controller position; **Return to reference** returns with short,
interruptible steps. References are valid only for the current connection
and cleared on detection/disconnect. They are not absolute encoder readings
and cannot recover an origin after power loss or missed steps.

Zaber's **Home to switch** uses the hardware home command at a configured
maximum speed selected in its manual control panel. It does **not** subsequently move to the far limit.
**STOP ALL / Esc** cancels a scan or reference return and requests Zaber stop.
Manual motion and homing are unavailable while recording. Recording travel
speed remains separately configurable in the recording tab.

Close the window, press **Quitter**, or use **Ctrl+Q** to stop motors, release
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
