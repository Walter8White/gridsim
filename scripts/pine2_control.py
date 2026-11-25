#!/usr/bin/env python3
"""Detect and cautiously control a Black Forest Motion PINE II over USB."""

from __future__ import annotations

import argparse
import re
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports


SERVICE_HINT = "Silicon Labs CP210"


@dataclass
class MotorState:
    port: int
    raw_position: int


def detect_port(requested: str | None) -> str:
    if requested:
        return requested
    candidates = [
        port.device
        for port in list_ports.comports()
        if "CP210" in (port.description or "") or "Silicon Labs" in (port.manufacturer or "")
    ]
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise SystemExit("No PINE USB serial adapter detected.")
    raise SystemExit(f"Multiple possible PINE ports detected: {', '.join(candidates)}; use --port.")


class Pine2:
    def __init__(self, port: str, baud: int, timeout: float = 1.0) -> None:
        self.serial = serial.Serial(port, baudrate=baud, timeout=timeout, write_timeout=timeout)

    def close(self) -> None:
        self.serial.close()

    def command(self, controller: int, name: str, *parameters: int, wait: float = 0.15) -> list[str]:
        frame = f"<{controller},{name},{len(parameters)}"
        if parameters:
            frame += "," + ",".join(str(value) for value in parameters)
        frame += ">\r\n"
        self.serial.reset_input_buffer()
        self.serial.write(frame.encode("ascii"))
        self.serial.flush()
        time.sleep(wait)
        responses: list[str] = []
        while True:
            line = self.serial.readline()
            if not line:
                break
            text = line.decode("ascii", errors="replace").strip()
            if text:
                responses.append(text)
        print(f"TX {frame.rstrip()}")
        for response in responses:
            print(f"RX {response}")
        return responses

    def detect(self, motors: tuple[int, ...]) -> dict[int, MotorState]:
        print(f"Using USB port: {self.serial.port}")
        self.command(0, "ID", 1)
        self.command(1, "MAC")
        self.command(1, "FWV")
        self.command(1, "VIN")
        states: dict[int, MotorState] = {}
        for motor in motors:
            responses = self.command(1, "MGP", motor)
            numbers = re.findall(r"-?\d+", responses[-1] if responses else "")
            if not numbers:
                raise RuntimeError(f"Could not read position for motor {motor}.")
            states[motor] = MotorState(motor, int(numbers[-1]))
        return states


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB serial device; auto-detects a CP210x adapter by default.")
    parser.add_argument("--baud", type=int, default=115200, help="USB serial baud rate.")
    parser.add_argument("--motor", nargs=2, type=int, default=(1, 2), metavar=("MOTOR_A", "MOTOR_B"))
    parser.add_argument("--step", type=int, default=1, help="Raw position increment for each test move.")
    parser.add_argument("--move-small", action="store_true", help="Move each selected motor by --step from its current position.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if any(motor < 1 or motor > 4 for motor in args.motor):
        raise SystemExit("Motor ports must be between 1 and 4.")
    if args.step <= 0:
        raise SystemExit("--step must be positive.")

    pine = Pine2(detect_port(args.port), args.baud)
    try:
        states = pine.detect(tuple(args.motor))
        print("Detected positions:", ", ".join(f"motor {m}={s.raw_position}" for m, s in states.items()))
        if args.move_small:
            for motor in args.motor:
                target = states[motor].raw_position + args.step
                pine.command(1, "MPS", motor, target, 10)
                print(f"Requested motor {motor} -> {target} (speed 10%).")
        else:
            print("Detection only; pass --move-small to issue the two test moves.")
    finally:
        pine.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
