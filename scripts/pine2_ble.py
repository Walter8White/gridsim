#!/usr/bin/env python3
"""Detect and control a Black Forest Motion PINE II over Bluetooth LE."""

from __future__ import annotations

import argparse
import asyncio
import re
from typing import Any

SERVICE_UUID = "00035b03-58e6-07dd-021a-08123a000300"
CHARACTERISTIC_UUID = "00035b03-58e6-07dd-021a-08123a000301"


async def find_pine(address: str | None) -> Any:
    from bleak import BleakScanner

    if address:
        return address
    devices = await BleakScanner.discover(timeout=8.0, service_uuids=[SERVICE_UUID])
    if len(devices) == 1:
        return devices[0].address
    if not devices:
        raise SystemExit("No PINE BLE device found. Power the PINE and retry.")
    choices = ", ".join(f"{device.name or '<unnamed>'}={device.address}" for device in devices)
    raise SystemExit(f"Multiple PINE BLE devices found: {choices}; use --address.")


async def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", help="BLE address; otherwise scan by the PINE service UUID.")
    parser.add_argument("--motor", nargs=2, type=int, default=(1, 2), metavar=("MOTOR_A", "MOTOR_B"))
    parser.add_argument("--step", type=int, default=1, help="Position increment for --move-small.")
    parser.add_argument("--move-small", action="store_true", help="Move each selected motor by --step at 1%% speed.")
    args = parser.parse_args()
    if any(motor < 1 or motor > 4 for motor in args.motor):
        raise SystemExit("Motor ports must be between 1 and 4.")
    if args.step <= 0:
        raise SystemExit("--step must be positive.")

    from bleak import BleakClient

    address = await find_pine(args.address)
    print(f"Connecting to PINE BLE device: {address}")
    async with BleakClient(address) as client:
        services = await client.get_services()
        characteristic = services.get_characteristic(CHARACTERISTIC_UUID)
        if characteristic is None:
            raise SystemExit(f"Characteristic not found: {CHARACTERISTIC_UUID}")
        print(f"Connected; characteristic properties: {', '.join(characteristic.properties)}")
        notifications: asyncio.Queue[str] = asyncio.Queue()

        def on_notify(_: Any, data: bytearray) -> None:
            text = bytes(data).decode("ascii", errors="replace").strip()
            if text:
                print(f"RX {text}")
                notifications.put_nowait(text)

        await client.start_notify(characteristic, on_notify)

        async def command(frame: str, settle_s: float = 0.5) -> list[str]:
            print(f"TX {frame}")
            await client.write_gatt_char(characteristic, (frame + "\r\n").encode("ascii"), response=True)
            responses: list[str] = []
            deadline = asyncio.get_running_loop().time() + settle_s
            while True:
                remaining = deadline - asyncio.get_running_loop().time()
                if remaining <= 0:
                    break
                try:
                    responses.append(await asyncio.wait_for(notifications.get(), remaining))
                except asyncio.TimeoutError:
                    break
            return responses

        await command("<0,ID,1,1>")
        await command("<1,MAC,0>")
        await command("<1,FWV,0>")
        await command("<1,VIN,0>")
        positions: dict[int, int] = {}
        for motor in args.motor:
            responses = await command(f"<1,MGP,1,{motor}>")
            matches = [re.search(rf"<{1},{motor}\.(-?\d+)>", response) for response in responses]
            match = next((item for item in reversed(matches) if item), None)
            if match is None:
                raise SystemExit(f"Could not read position for motor {motor}.")
            positions[motor] = int(match.group(1))
        print("Positions:", ", ".join(f"motor {motor}={position}" for motor, position in positions.items()))

        if args.move_small:
            for motor in args.motor:
                target = positions[motor] + args.step
                await command(f"<1,MPS,3,{motor},{target},1>", settle_s=1.5)
            for motor in args.motor:
                await command(f"<1,MGP,1,{motor}>")
        else:
            print("BLE detection only; no motor command sent.")
        await client.stop_notify(characteristic)
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
