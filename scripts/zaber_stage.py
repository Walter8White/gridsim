#!/usr/bin/env python3
"""Small helper to detect and cautiously control a Zaber linear stage."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass


DEFAULT_PORT = "/dev/ttyUSB0"


@dataclass(frozen=True)
class Detected:
    protocol: str
    connection: object
    devices: list[object]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect and control a Zaber stage over ASCII or legacy Binary protocol."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port, default: {DEFAULT_PORT}")
    parser.add_argument(
        "--protocol",
        choices=("auto", "ascii", "binary"),
        default="auto",
        help="Protocol to use. A-Series devices often default to binary.",
    )
    parser.add_argument(
        "--command",
        choices=("detect", "home", "send", "stop", "position"),
        default="detect",
        help="Action to run. detect never moves the stage.",
    )
    parser.add_argument(
        "--position-mm",
        type=float,
        help="Absolute target position in mm for send.",
    )
    parser.add_argument(
        "--velocity-mm-s",
        type=float,
        default=1.0,
        help="Move velocity in mm/s for send and the move-to-top part of home. Must be positive.",
    )
    parser.add_argument(
        "--no-identify",
        action="store_true",
        help="Skip online/offline device identification during detection.",
    )
    return parser.parse_args()


def detect_ascii(port: str, identify: bool) -> Detected:
    from zaber_motion.ascii import Connection

    connection = Connection.open_serial_port(port)
    try:
        connection.enable_alerts()
        devices = connection.detect_devices(identify_devices=identify)
        return Detected("ascii", connection, devices)
    except Exception:
        connection.close()
        raise


def detect_binary(port: str, identify: bool) -> Detected:
    from zaber_motion.binary import Connection

    connection = Connection.open_serial_port(port)
    try:
        devices = connection.detect_devices(identify_devices=identify)
        return Detected("binary", connection, devices)
    except Exception:
        connection.close()
        raise


def detect(port: str, protocol: str, identify: bool) -> Detected:
    attempts = []
    protocols = ("ascii", "binary") if protocol == "auto" else (protocol,)
    for candidate in protocols:
        try:
            if candidate == "ascii":
                return detect_ascii(port, identify)
            return detect_binary(port, identify)
        except Exception as exc:
            attempts.append(f"{candidate}: {exc}")
    raise RuntimeError("Could not detect a Zaber device:\n" + "\n".join(attempts))


def print_devices(result: Detected) -> None:
    print(f"Protocol: {result.protocol}")
    print(f"Found {len(result.devices)} device(s)")
    for index, device in enumerate(result.devices, start=1):
        label = getattr(device, "name", None) or repr(device)
        address = getattr(device, "device_address", None)
        if address is None:
            print(f"{index}: {label}")
        else:
            print(f"{index}: address={address} {label}")


def first_axis_or_device(result: Detected) -> object:
    if not result.devices:
        raise RuntimeError("No Zaber devices detected.")
    device = result.devices[0]
    if result.protocol == "ascii":
        return device.get_axis(1)
    return device


def home(result: Detected, velocity_mm_s: float) -> None:
    from zaber_motion import Units

    if velocity_mm_s <= 0:
        raise RuntimeError("--velocity-mm-s must be > 0")

    target = first_axis_or_device(result)
    target.settings.set("maxspeed", velocity_mm_s, Units.VELOCITY_MILLIMETRES_PER_SECOND)
    print("Homing first detected axis/device...")
    target.home()
    print("Moving first detected axis/device to top/limit.max after homing...")
    target.move_max()


def send(result: Detected, position_mm: float | None, velocity_mm_s: float) -> None:
    from zaber_motion import Units

    if position_mm is None:
        raise RuntimeError("send requires --position-mm")
    if velocity_mm_s <= 0:
        raise RuntimeError("--velocity-mm-s must be > 0")

    target = first_axis_or_device(result)
    print(f"Sending first detected axis/device to {position_mm} mm at {velocity_mm_s} mm/s...")
    if result.protocol == "ascii":
        target.settings.set("maxspeed", velocity_mm_s, Units.VELOCITY_MILLIMETRES_PER_SECOND)
        target.move_absolute(
            position_mm,
            Units.LENGTH_MILLIMETRES,
        )
    else:
        raise RuntimeError("send with --velocity-mm-s requires ASCII protocol.")


def stop(result: Detected) -> None:
    target = first_axis_or_device(result)
    print("Stopping first detected axis/device...")
    target.stop()


def position(result: Detected) -> None:
    from zaber_motion import Units

    target = first_axis_or_device(result)
    if result.protocol != "ascii":
        raise RuntimeError("position command currently requires ASCII protocol.")
    position_mm = target.get_position(Units.LENGTH_MILLIMETRES)
    print(f"Current first detected axis/device position: {position_mm:.6f} mm")


def main() -> int:
    args = parse_args()
    try:
        result = detect(args.port, args.protocol, identify=not args.no_identify)
        try:
            print_devices(result)
            if args.command == "home":
                home(result, args.velocity_mm_s)
            elif args.command == "send":
                send(result, args.position_mm, args.velocity_mm_s)
            elif args.command == "stop":
                stop(result)
            elif args.command == "position":
                position(result)
        finally:
            result.connection.close()
    except ImportError:
        print("Missing dependency. Install with: python3 -m pip install --user zaber-motion")
        return 2
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
