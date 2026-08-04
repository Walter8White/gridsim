# KEYENCE LJ-S8000 Linux Tools

This folder contains the LJ-S8000 Linux communication sources and small helper tools used to acquire data from a KEYENCE LJ-S640 over direct Ethernet.

## Build

```bash
cd keyence_ljs8000/CPP
make
```

The helper scripts in `CPP/bin` are versioned. Compiled binaries in `CPP/bin` and intermediate files in `CPP/obj` are ignored.

## Network

The sample tools expect the sensor at:

```text
IP: 192.168.0.1
Command TCP port: 24691
High-speed TCP port: 24692
```

Configure the PC Ethernet interface on the same subnet, for example:

```bash
sudo ip addr flush dev enp131s0
sudo ip addr add 192.168.0.10/24 dev enp131s0
sudo ip link set enp131s0 up
```

## Acquisition

```bash
./bin/check_status
./bin/main
```

By default `main` acquires data and prints statistics without saving files.

Save raw data:

```bash
./bin/main --save-raw
```

Save a PNG preview with invalid pixels in red:

```bash
./bin/main --save-invalid-image
```

Save everything:

```bash
./bin/main --save-all
```

Generated captures are written to `CPP/captures/` and ignored by Git.

## Mesh Export

Generate a PLY mesh from the latest capture:

```bash
./bin/generate_ply --stride 8
```

Generate an STL mesh:

```bash
./bin/generate_stl --stride 8
```

Lower stride means more detail and larger files. For FreeCAD, start with `--stride 16` or `--stride 8`.

## Settings

Read the active program settings:

```bash
./bin/keyence_setting get all
```

Example transient setting changes:

```bash
./bin/keyence_setting set detection_sensitivity 5
./bin/keyence_setting set dead_zone_interpolation 2
./bin/keyence_setting set y_subsample 1
```

Settings are written to RUNNING by default and are not saved across power cycles. Use `--save` only when intentionally writing to the head save area.
