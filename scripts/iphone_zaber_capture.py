#!/usr/bin/env python3
"""Move a vertical Zaber stage and trigger GridCapture on an iPhone at each stop."""

from __future__ import annotations

import argparse
import asyncio
import json
import signal
import sys
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move-stop-photo loop using a Zaber stage and the GridCapture iPhone app."
    )
    parser.add_argument("--host", default="0.0.0.0", help="WebSocket listen address.")
    parser.add_argument("--port", type=int, default=8765, help="WebSocket listen port.")
    parser.add_argument("--zaber-port", default="/dev/ttyUSB0", help="Zaber serial port.")
    parser.add_argument("--velocity-mm-s", type=float, default=5.0)
    start = parser.add_mutually_exclusive_group(required=True)
    start.add_argument("--start-mm", type=float, help="Explicit first absolute position.")
    start.add_argument(
        "--start-current", action="store_true", help="Use the current Zaber position as the first position."
    )
    parser.add_argument("--end-mm", type=float, required=True)
    parser.add_argument("--step-mm", type=float, required=True)
    parser.add_argument("--settle-s", type=float, default=0.25)
    parser.add_argument("--capture-timeout-s", type=float, default=30.0)
    parser.add_argument("--max-captures", type=int, default=1000)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Do not connect to or move the Zaber; use target positions as measured positions.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("captures/iphone_zaber"),
        help="Linux directory for the session JSONL log.",
    )
    return parser.parse_args()


class ZaberStage:
    """Persistent ASCII connection; all methods must run on one worker thread."""

    def __init__(self, port: str, velocity_mm_s: float) -> None:
        self.port = port
        self.velocity_mm_s = velocity_mm_s
        self.connection: Any = None
        self.axis: Any = None

    def open(self) -> float:
        from zaber_motion import Units
        from zaber_motion.ascii import Connection

        self.connection = Connection.open_serial_port(self.port)
        self.connection.enable_alerts()
        devices = self.connection.detect_devices(identify_devices=True)
        if not devices:
            self.connection.close()
            self.connection = None
            raise RuntimeError(f"No Zaber device detected on {self.port}")
        self.axis = devices[0].get_axis(1)
        self.axis.settings.set(
            "maxspeed", self.velocity_mm_s, Units.VELOCITY_MILLIMETRES_PER_SECOND
        )
        return float(self.axis.get_position(Units.LENGTH_MILLIMETRES))

    def move_and_read(self, target_mm: float) -> float:
        from zaber_motion import Units

        if self.axis is None:
            raise RuntimeError("Zaber connection is not open")
        self.axis.move_absolute(target_mm, Units.LENGTH_MILLIMETRES)
        return float(self.axis.get_position(Units.LENGTH_MILLIMETRES))

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
        self.connection = None
        self.axis = None


def positions(start_mm: float, end_mm: float, step_mm: float, limit: int) -> list[float]:
    if step_mm <= 0:
        raise ValueError("--step-mm must be > 0")
    if limit <= 0:
        raise ValueError("--max-captures must be > 0")
    direction = 1.0 if end_mm >= start_mm else -1.0
    values: list[float] = []
    current = start_mm
    epsilon = 1e-9
    while len(values) < limit:
        if direction > 0 and current > end_mm + epsilon:
            break
        if direction < 0 and current < end_mm - epsilon:
            break
        values.append(round(current, 9))
        current += direction * step_mm
    if values and abs(values[-1] - end_mm) > epsilon and len(values) < limit:
        values.append(end_mm)
    return values


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


async def receive_capture_result(websocket: Any, capture_id: str, timeout_s: float) -> dict[str, Any]:
    async with asyncio.timeout(timeout_s):
        while True:
            raw = await websocket.recv()
            message = json.loads(raw)
            if message.get("type") != "capture_result":
                continue
            if message.get("capture_id") != capture_id:
                raise RuntimeError(
                    f"Capture response ID mismatch: expected {capture_id}, got {message.get('capture_id')}"
                )
            return message


async def run_session(websocket: Any, args: argparse.Namespace) -> None:
    executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="zaber")
    stage = None if args.dry_run else ZaberStage(args.zaber_port, args.velocity_mm_s)
    loop = asyncio.get_running_loop()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    log_path = args.output_dir / f"session_{session_id}.jsonl"

    try:
        if stage is None:
            if args.start_current:
                raise RuntimeError("--dry-run requires an explicit --start-mm")
            current_mm = args.start_mm
            mode = "dry-run"
        else:
            current_mm = await loop.run_in_executor(executor, stage.open)
            mode = "zaber"
        start_mm = current_mm if args.start_current else args.start_mm
        assert start_mm is not None
        scan_positions = positions(start_mm, args.end_mm, args.step_mm, args.max_captures)
        if not scan_positions:
            raise RuntimeError("The requested scan contains no positions")

        await websocket.send(json.dumps({"type": "hello", "mode": mode}))
        print(f"iPhone connected. mode={mode} captures={len(scan_positions)} log={log_path}")

        with log_path.open("a", encoding="utf-8") as log_file:
            for index, target_mm in enumerate(scan_positions):
                if stage is None:
                    measured_mm = target_mm
                else:
                    measured_mm = await loop.run_in_executor(
                        executor, stage.move_and_read, target_mm
                    )
                await asyncio.sleep(args.settle_s)

                capture_id = f"{session_id}-{index:05d}"
                command = {
                    "type": "capture",
                    "capture_id": capture_id,
                    "target_position_mm": target_mm,
                    "measured_position_mm": measured_mm,
                    "sent_at": utc_now(),
                }
                print(
                    f"capture={index + 1}/{len(scan_positions)} "
                    f"target_mm={target_mm:.3f} measured_mm={measured_mm:.6f}"
                )
                await websocket.send(json.dumps(command))
                result = await receive_capture_result(
                    websocket, capture_id, args.capture_timeout_s
                )
                record = {**command, "iphone_result": result, "ack_received_at": utc_now()}
                log_file.write(json.dumps(record, sort_keys=True) + "\n")
                log_file.flush()
                if not result.get("success"):
                    raise RuntimeError(
                        f"iPhone capture failed at {measured_mm:.6f} mm: {result.get('error')}"
                    )
                print(f"  saved_on_iphone={result.get('photo_filename')}")

        await websocket.send(json.dumps({"type": "scan_complete", "captures": len(scan_positions)}))
        print("Scan complete.")
    finally:
        if stage is not None:
            await loop.run_in_executor(executor, stage.close)
        executor.shutdown(wait=True, cancel_futures=True)


async def async_main(args: argparse.Namespace) -> None:
    try:
        import websockets
    except ImportError as exc:
        raise RuntimeError(
            "Missing dependency. Install with: python3 -m pip install websockets zaber-motion"
        ) from exc

    active = asyncio.Lock()

    async def handle_client(websocket: Any, _path: Any = None) -> None:
        if active.locked():
            await websocket.close(code=1013, reason="A GridCapture session is already active")
            return
        async with active:
            try:
                await run_session(websocket, args)
            except Exception as exc:
                print(f"Session stopped: {exc}", file=sys.stderr)
                try:
                    await websocket.send(json.dumps({"type": "error", "message": str(exc)}))
                except Exception:
                    pass

    stop = asyncio.get_running_loop().create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(sig, lambda: stop.cancel())

    async with websockets.serve(handle_client, args.host, args.port, ping_interval=20):
        print(f"Waiting for GridCapture on ws://{args.host}:{args.port}")
        try:
            await stop
        except asyncio.CancelledError:
            pass


def main() -> int:
    args = parse_args()
    if args.velocity_mm_s <= 0 or args.settle_s < 0 or args.capture_timeout_s <= 0:
        print("Velocity and timeout must be positive; settle time cannot be negative.", file=sys.stderr)
        return 2
    try:
        asyncio.run(async_main(args))
    except (ValueError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
