#!/usr/bin/env python3
"""Capture several vertical iPhone/Zaber passes with manual horizontal offsets."""

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

from capture_common import ZaberStage, positions, receive_capture_result, utc_now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run repeated vertical Zaber scans. Between passes, prompt for the "
            "manual horizontal displacement measured by the user."
        )
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--zaber-port", default="/dev/ttyUSB0")
    parser.add_argument("--start-mm", type=float, required=True)
    parser.add_argument("--end-mm", type=float, required=True)
    parser.add_argument("--step-mm", type=float, required=True)
    parser.add_argument("--passes", type=int, default=3)
    parser.add_argument("--velocity-mm-s", type=float, default=20.0)
    parser.add_argument("--settle-s", type=float, default=0.3)
    parser.add_argument("--capture-timeout-s", type=float, default=60.0)
    parser.add_argument("--max-captures-per-pass", type=int, default=1000)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("captures/iphone_zaber"),
        help="Linux directory for the multi-pass session.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.passes <= 0:
        raise ValueError("--passes must be > 0")
    if args.step_mm <= 0:
        raise ValueError("--step-mm must be > 0")
    if args.velocity_mm_s <= 0:
        raise ValueError("--velocity-mm-s must be > 0")
    if args.settle_s < 0:
        raise ValueError("--settle-s cannot be negative")
    if args.capture_timeout_s <= 0:
        raise ValueError("--capture-timeout-s must be > 0")


async def prompt_offset_cm(pass_index: int) -> float:
    prompt = (
        f"\nPass {pass_index} complete. Move the phone/chariot horizontally, "
        "measure the displacement from the previous pass, then enter it in cm "
        "(signed, e.g. 15 or -15): "
    )
    while True:
        raw = await asyncio.to_thread(input, prompt)
        raw = raw.strip().replace(",", ".")
        try:
            return float(raw)
        except ValueError:
            print("Please enter a number in cm.")


async def run_multi_pass(websocket: Any, args: argparse.Namespace) -> None:
    executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="zaber")
    stage = ZaberStage(args.zaber_port, args.velocity_mm_s)
    loop = asyncio.get_running_loop()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    session_dir = args.output_dir / f"session_{session_id}_multipass"
    (session_dir / "images").mkdir(parents=True, exist_ok=False)
    (session_dir / "metadata").mkdir()
    log_path = session_dir / "manifest.jsonl"

    base_scan_positions = positions(
        args.start_mm,
        args.end_mm,
        args.step_mm,
        args.max_captures_per_pass,
    )
    if not base_scan_positions:
        raise RuntimeError("The requested scan contains no positions")

    cumulative_x_cm = 0.0
    capture_global_index = 0

    try:
        current_mm = await loop.run_in_executor(executor, stage.open)
        await websocket.send(
            json.dumps(
                {
                    "type": "hello",
                    "mode": "zaber-multipass",
                    "passes": args.passes,
                    "captures_per_pass": len(base_scan_positions),
                }
            )
        )
        print(
            f"iPhone connected. mode=zaber-multipass passes={args.passes} "
            f"captures_per_pass={len(base_scan_positions)} current_mm={current_mm:.6f} "
            f"log={log_path}"
        )

        with log_path.open("a", encoding="utf-8") as log_file:
            for pass_index in range(args.passes):
                if pass_index > 0:
                    delta_cm = await prompt_offset_cm(pass_index)
                    cumulative_x_cm += delta_cm

                print(
                    f"\nStarting pass {pass_index + 1}/{args.passes}: "
                    f"x_offset_cm={cumulative_x_cm:.3f}"
                )
                scan_positions = (
                    base_scan_positions
                    if pass_index % 2 == 0
                    else list(reversed(base_scan_positions))
                )
                for pass_capture_index, target_mm in enumerate(scan_positions):
                    measured_mm = await loop.run_in_executor(
                        executor, stage.move_and_read, target_mm
                    )
                    await asyncio.sleep(args.settle_s)

                    capture_id = f"{session_id}-p{pass_index:02d}-{pass_capture_index:05d}"
                    command = {
                        "type": "capture",
                        "capture_id": capture_id,
                        "target_position_mm": target_mm,
                        "measured_position_mm": measured_mm,
                        "pass_index": pass_index,
                        "pass_capture_index": pass_capture_index,
                        "x_offset_cm": cumulative_x_cm,
                        "x_offset_m": cumulative_x_cm / 100.0,
                        "sent_at": utc_now(),
                    }
                    print(
                        f"pass={pass_index + 1}/{args.passes} "
                        f"capture={pass_capture_index + 1}/{len(scan_positions)} "
                        f"target_mm={target_mm:.3f} measured_mm={measured_mm:.6f} "
                        f"x_cm={cumulative_x_cm:.3f}"
                    )
                    await websocket.send(json.dumps(command))
                    result = await receive_capture_result(
                        websocket,
                        capture_id,
                        args.capture_timeout_s,
                        session_dir,
                    )
                    record = {
                        **command,
                        "global_capture_index": capture_global_index,
                        "iphone_result": result,
                        "ack_received_at": utc_now(),
                    }
                    log_file.write(json.dumps(record, sort_keys=True) + "\n")
                    log_file.flush()
                    capture_global_index += 1
                    if not result.get("success"):
                        raise RuntimeError(
                            f"iPhone capture failed at {measured_mm:.6f} mm: {result.get('error')}"
                        )
                    print(f"  received_on_linux={result.get('photo_filename')}")

        await websocket.send(
            json.dumps(
                {
                    "type": "scan_complete",
                    "captures": capture_global_index,
                    "passes": args.passes,
                }
            )
        )
        print(f"\nMulti-pass scan complete: {capture_global_index} captures.")
        print(f"Session directory: {session_dir}")
    finally:
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
                await run_multi_pass(websocket, args)
            except Exception as exc:
                print(f"Session stopped: {exc}", file=sys.stderr)
                try:
                    await websocket.send(json.dumps({"type": "error", "message": str(exc)}))
                except Exception:
                    pass

    stop = asyncio.get_running_loop().create_future()
    for sig in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(sig, lambda: stop.cancel())

    async with websockets.serve(
        handle_client,
        args.host,
        args.port,
        ping_interval=20,
        max_size=100 * 1024 * 1024,
    ):
        print(f"Waiting for GridCapture on ws://{args.host}:{args.port}")
        try:
            await stop
        except asyncio.CancelledError:
            pass


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        asyncio.run(async_main(args))
    except (ValueError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
