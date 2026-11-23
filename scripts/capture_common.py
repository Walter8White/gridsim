"""Shared Zaber and iPhone capture protocol helpers."""
from __future__ import annotations
import asyncio, hashlib, json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

def positions(start_mm: float, end_mm: float, step_mm: float, limit: int) -> list[float]:
    if step_mm <= 0 or limit <= 0: raise ValueError("step_mm and limit must be > 0")
    direction = 1.0 if end_mm >= start_mm else -1.0
    values, current = [], start_mm
    while len(values) < limit:
        if direction > 0 and current > end_mm + 1e-9: break
        if direction < 0 and current < end_mm - 1e-9: break
        values.append(round(current, 9)); current += direction * step_mm
    if values and abs(values[-1] - end_mm) > 1e-9 and len(values) < limit: values.append(end_mm)
    return values

def utc_now() -> str: return datetime.now(timezone.utc).isoformat()

class ZaberStage:
    """Persistent ASCII Zaber connection; methods run on its worker thread."""
    def __init__(self, port: str, velocity_mm_s: float):
        self.port, self.velocity_mm_s = port, velocity_mm_s
        self.connection: Any = None
        self.axis: Any = None
    def open(self) -> float:
        from zaber_motion import Units
        from zaber_motion.ascii import Connection
        self.connection = Connection.open_serial_port(self.port)
        try:
            self.connection.enable_alerts(); devices = self.connection.detect_devices(identify_devices=True)
            if not devices: raise RuntimeError(f"No Zaber device detected on {self.port}")
            self.axis = devices[0].get_axis(1)
            self.axis.settings.set("maxspeed", self.velocity_mm_s, Units.VELOCITY_MILLIMETRES_PER_SECOND)
            return float(self.axis.get_position(Units.LENGTH_MILLIMETRES))
        except Exception:
            self.connection.close(); self.connection = self.axis = None; raise
    def move_and_read(self, target_mm: float) -> float:
        from zaber_motion import Units
        if self.axis is None: raise RuntimeError("Zaber connection is not open")
        self.axis.move_absolute(target_mm, Units.LENGTH_MILLIMETRES)
        return float(self.axis.get_position(Units.LENGTH_MILLIMETRES))
    def close(self):
        if self.connection is not None: self.connection.close()
        self.connection = self.axis = None

def _filename(value, suffix):
    if not isinstance(value, str) or not value or Path(value).name != value or Path(value).suffix.lower() != suffix:
        raise RuntimeError(f"Unsafe or unexpected upload filename: {value!r}")
    return value

def _payload(data, size, digest, label):
    if not isinstance(data, bytes) or not isinstance(size, int) or size < 0 or len(data) != size:
        raise RuntimeError(f"{label} size mismatch")
    if not isinstance(digest, str) or hashlib.sha256(data).hexdigest() != digest.lower():
        raise RuntimeError(f"{label} SHA-256 mismatch")
    return data

def _atomic(path, data):
    temp = path.with_suffix(path.suffix + ".part"); temp.write_bytes(data); temp.replace(path)

async def receive_capture_result(websocket, capture_id, timeout_s, session_dir):
    uploaded = False
    async with asyncio.timeout(timeout_s):
        while True:
            raw = await websocket.recv()
            if isinstance(raw, bytes): raise RuntimeError("Binary payload without upload header")
            message = json.loads(raw); kind = message.get("type")
            if kind == "capture_upload_start":
                if message.get("capture_id") != capture_id: raise RuntimeError("Upload capture_id mismatch")
                photo_name = _filename(message.get("photo_filename"), ".jpg")
                metadata_name = _filename(message.get("metadata_filename"), ".json")
                photo = _payload(await websocket.recv(), message.get("photo_size"), message.get("photo_sha256"), "photo")
                metadata = _payload(await websocket.recv(), message.get("metadata_size"), message.get("metadata_sha256"), "metadata")
                _atomic(session_dir / "images" / photo_name, photo)
                _atomic(session_dir / "metadata" / metadata_name, metadata); uploaded = True
                continue
            if kind != "capture_result": continue
            if message.get("capture_id") != capture_id: raise RuntimeError("Capture response ID mismatch")
            if message.get("success") and not uploaded: raise RuntimeError("Success without uploaded files")
            return message
