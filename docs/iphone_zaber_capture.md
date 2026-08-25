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
