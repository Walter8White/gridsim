"""Hardware and capture service for the desktop station (no GUI dependencies)."""
from __future__ import annotations

import asyncio
import base64
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

from capture_common import positions, receive_capture_result, utc_now
from pine2_ble import find_pine, CHARACTERISTIC_UUID


class CaptureInbox:
    def __init__(self):
        self.queue = asyncio.Queue(maxsize=8)

    async def recv(self):
        item = await self.queue.get()
        if isinstance(item, Exception):
            raise item
        return item


def scan_positions(start, end, step, limit=1000):
    if not all(math.isfinite(x) for x in (start, end, step)) or step <= 0:
        raise ValueError("Positions must be finite and step must be positive")
    if math.ceil(abs(end - start) / step) + 1 > limit:
        raise ValueError(f"Scan exceeds {limit} photos; increase the step")
    return positions(start, end, step, limit)


class Station:
    def __init__(self, emit, demo=False):
        self.emit = emit
        self.demo = demo
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="zaber")
        self.pine = None
        self.zaber = None
        self.axis = None
        self.phone = None
        self.inbox = CaptureInbox()
        self.motor_ports = (1, 3)
        self.position = {"pine1": 0., "pine2": 0., "zaber": 0.}
        self.references = {}
        self.demands = {}
        self.connected = set()
        self.scan = None
        self.reference_task = None
        self.detecting = False
        self.homing = False
        self.pine_lock = asyncio.Lock()
        self.notifications = asyncio.Queue()
        self.buffer = ""
        self.zaber_speed = 0.
        self.zaber_target = None
        self.last_zaber_target = None
        self.motion_epoch = 0
        self.speeds = {"pine1": 1, "pine2": 1, "zaber": 1.}

    def status(self, message):
        self.emit("status", message)

    async def serial(self, function, *args, **kwargs):
        import functools
        return await asyncio.get_running_loop().run_in_executor(
            self.pool, functools.partial(function, *args, **kwargs))

    async def start(self, host, port):
        import websockets
        self.server = await websockets.serve(self.phone_client, host, port, max_size=100*1024*1024)
        self.mdns = None
        try:
            from zeroconf import ServiceInfo, Zeroconf, IPVersion
            import ipaddress
            import socket
            from zeroconf.utils.ip_addresses import get_all_addresses
            packed = list({socket.inet_aton(address) for address in get_all_addresses()
                           if ipaddress.ip_address(address).version == 4
                           and ipaddress.ip_address(address).is_private
                           and not ipaddress.ip_address(address).is_loopback})
            if packed:
                service_name = "GridCapture Control._gridsim-control._tcp.local."
                info = ServiceInfo("_gridsim-control._tcp.local.", service_name,
                    addresses=packed, port=self.server.sockets[0].getsockname()[1],
                    properties={"path": "/", "protocol": "websocket"},
                    server=socket.gethostname().split(".")[0] + ".local.")
                self.mdns = Zeroconf(ip_version=IPVersion.V4Only)
                self.mdns.register_service(info)
                self.emit("station_ready", True)
                self.status("Bonjour discovery active: GridCapture can connect automatically")
            else:
                self.emit("station_ready", False)
                self.status("No LAN IPv4 address for Bonjour; use manual iPhone address")
        except Exception as exc:
            self.emit("station_ready", False)
            self.status(f"Bonjour unavailable ({exc}); use manual iPhone address")
        self.ticker = asyncio.create_task(self.tick())
        self.watchdog = asyncio.create_task(self.watch_deadman())
        self.status(f"Listening for GridCapture on port {port}" + (" — DEMO: simulated motors" if self.demo else ""))

    async def phone_client(self, socket, _path=None):
        if self.phone is not None:
            await socket.close(code=1013, reason="An iPhone is already connected")
            return
        self.phone = socket
        self.inbox = CaptureInbox()
        self.emit("phone", True)
        try:
            await socket.send(json.dumps({"type": "hello", "mode": "control-station"}))
            async for raw in socket:
                if isinstance(raw, str):
                    message = json.loads(raw)
                    if message.get("type") == "preview":
                        data = base64.b64decode(message["jpeg"], validate=True)
                        if len(data) <= 2_000_000:
                            self.emit("preview", data)
                        continue
                if self.scan is not None:
                    await self.inbox.queue.put(raw)
        finally:
            self.phone = None
            self.emit("phone", False)
            await self.stop()
            self.status("iPhone disconnected")

    def notify(self, _, data):
        self.buffer += bytes(data).decode("ascii", errors="replace")
        while ">" in self.buffer:
            frame, self.buffer = self.buffer.split(">", 1)
            self.notifications.put_nowait(frame.strip() + ">")
        self.buffer = self.buffer[-4096:]

    async def pine_command(self, name, *args, position=False):
        import re
        async with self.pine_lock:
            while not self.notifications.empty():
                self.notifications.get_nowait()
            frame = f"<1,{name},{len(args)}" + "".join(f",{v}" for v in args) + ">\r\n"
            await asyncio.wait_for(self.pine.write_gatt_char(CHARACTERISTIC_UUID, frame.encode(), response=True), 2)
            if position:
                async with asyncio.timeout(2):
                    while True:
                        response = await self.notifications.get()
                        match = re.search(rf"<1,{args[0]}\.(-?\d+)>", response)
                        if match:
                            return int(match[1])

    async def detect(self, port, address, motor1, motor2):
        if self.detecting or self.scan or self.homing:
            raise RuntimeError("Stop the current operation before detecting")
        if motor1 == motor2:
            raise ValueError("Choose two different PINE ports")
        self.detecting = True
        await self.stop()
        try:
            self.motor_ports = (motor1, motor2)
            self.references.clear()
            if self.demo:
                self.connected = {"pine1", "pine2", "zaber"}
                return
            if self.pine:
                await self.pine.disconnect()
                self.pine = None
            if self.zaber:
                await self.serial(self.zaber.connection.close)
                self.zaber = self.axis = None
            self.connected.clear()
            self.emit("connected", [])
            try:
                from bleak import BleakClient
                self.pine = BleakClient(await find_pine(address or None),
                                        disconnected_callback=lambda _: self.pine_disconnected())
                await self.pine.connect()
                await self.pine.start_notify(CHARACTERISTIC_UUID, self.notify)
                await self.pine.write_gatt_char(CHARACTERISTIC_UUID, b"<0,ID,1,1>\r\n", response=True)
                await asyncio.sleep(.2)
                for key, motor in zip(("pine1", "pine2"), self.motor_ports):
                    self.position[key] = await self.pine_command("MGP", motor, position=True)
                self.connected.update(("pine1", "pine2"))
                self.status("PINE connected over BLE")
            except (Exception, SystemExit) as exc:
                self.status(f"PINE: {exc}")
                if self.pine:
                    await self.pine.disconnect()
                    self.pine = None
            try:
                from serial.tools import list_ports
                from zaber_stage import detect_ascii
                candidates = [port] if port else [p.device for p in list_ports.comports()
                    if "CP210" not in (p.description or "")]
                for candidate in candidates:
                    result = None
                    try:
                        result = await self.serial(detect_ascii, candidate, True)
                        if result.devices:
                            self.zaber = result
                            self.axis = result.devices[0].get_axis(1)
                            await self.read_zaber()
                            self.connected.add("zaber")
                            self.status(f"Zaber connected: {candidate} (ASCII, first device / axis 1)")
                            break
                    except Exception as exc:
                        self.status(f"{candidate}: {exc}")
                    if result:
                        await self.serial(result.connection.close)
                        self.zaber = self.axis = None
                if "zaber" not in self.connected:
                    self.status("No Zaber ASCII axis found; select its USB port and retry")
            except Exception as exc:
                self.status(f"Zaber: {exc}")
        finally:
            self.detecting = False
            self.emit("connected", sorted(self.connected))
            self.emit("positions", dict(self.position))
            self.status("Detection complete. iPhone: " + ("connected" if self.phone else "waiting for GridCapture to connect"))

    def pine_disconnected(self):
        self.connected.difference_update(("pine1", "pine2"))
        self.demands.pop("pine1", None)
        self.demands.pop("pine2", None)
        self.references.clear()
        self.emit("connected", sorted(self.connected))

    async def read_zaber(self):
        from zaber_motion import Units
        self.position["zaber"] = await self.serial(self.axis.get_position, Units.LENGTH_MILLIMETRES)
        return self.position["zaber"]

    def set_speed(self, key, value):
        if key not in self.speeds or not math.isfinite(value):
            raise ValueError("Invalid motor speed")
        minimum = .001 if key == "zaber" else 1
        if not minimum <= value <= 100 or (key != "zaber" and value != int(value)):
            raise ValueError("Zaber speed must be 0.001–100 mm/s; PINE speed 1–100%")
        self.speeds[key] = float(value) if key == "zaber" else int(value)

    def set_zaber_target(self, target):
        if not math.isfinite(target) or not 0 <= target <= 1500:
            raise ValueError("Zaber target must be between 0 and 1500 mm")
        if "zaber" not in self.connected or self.scan or self.detecting or self.homing:
            return
        self.demands["zaber"] = (1., time.monotonic())
        self.zaber_target = float(target)

    def jog(self, key, value):
        if not math.isfinite(value):
            raise ValueError("Invalid jog speed")
        if value and (key not in self.connected or self.scan or self.detecting or self.homing):
            return
        if key == "zaber":
            self.zaber_target = None
        self.demands[key] = (max(-1., min(1., value)), time.monotonic())

    async def watch_deadman(self):
        while True:
            await asyncio.sleep(.05)
            value, stamp = self.demands.get("zaber", (0., 0.))
            if self.zaber_speed and (not value or time.monotonic() - stamp >= .4):
                self.zaber_speed = 0.
                self.last_zaber_target = None
                if self.axis and not self.demo:
                    try:
                        await self.serial(self.axis.stop, wait_until_idle=False)
                    except Exception as exc:
                        self.status(f"Deadman stop failed: {exc}")
                        self.connected.discard("zaber")
                        self.emit("connected", sorted(self.connected))

    async def tick(self):
        while True:
            try:
                if not self.detecting and not self.scan and not self.homing:
                    epoch = self.motion_epoch
                    for key in tuple(self.connected):
                        if epoch != self.motion_epoch or self.scan or self.homing or self.detecting:
                            break
                        demand, stamp = self.demands.get(key, (0., 0.))
                        value = demand if time.monotonic() - stamp < .4 else 0.
                        if self.demo:
                            if key == "zaber" and self.zaber_target is not None and value:
                                delta = self.zaber_target - self.position[key]
                                step = self.speeds[key] * .1
                                self.position[key] += max(-step, min(step, delta))
                            else:
                                self.position[key] += value * self.speeds[key] * (.1 if key == "zaber" else 2)
                        elif key == "zaber":
                            from zaber_motion import Units
                            target = self.zaber_target
                            velocity = value * self.speeds[key]
                            if value and target is not None:
                                request = (target, self.speeds[key])
                                if request != self.last_zaber_target:
                                    await self.serial(self.axis.settings.set, "maxspeed", self.speeds[key], Units.VELOCITY_MILLIMETRES_PER_SECOND)
                                    if (epoch != self.motion_epoch or self.zaber_target != target
                                            or time.monotonic() - self.demands.get(key, (0., 0.))[1] >= .4):
                                        continue
                                    await self.serial(self.axis.move_absolute, target, Units.LENGTH_MILLIMETRES, wait_until_idle=False)
                                    self.last_zaber_target = request
                                    self.zaber_speed = self.speeds[key]
                            elif self.last_zaber_target is not None:
                                await self.serial(self.axis.stop, wait_until_idle=False)
                                self.last_zaber_target = None
                                self.zaber_speed = 0.
                            elif velocity != self.zaber_speed:
                                if value:
                                    await self.serial(self.axis.move_velocity, velocity, Units.VELOCITY_MILLIMETRES_PER_SECOND)
                                else:
                                    await self.serial(self.axis.stop, wait_until_idle=False)
                                self.zaber_speed = velocity
                            await self.read_zaber()
                        else:
                            motor = self.motor_ports[0 if key == "pine1" else 1]
                            current = await self.pine_command("MGP", motor, position=True)
                            self.position[key] = current
                            if (value and epoch == self.motion_epoch
                                    and self.demands.get(key, (0., 0.))[0] == value
                                    and time.monotonic() - stamp < .4):
                                # Finite, tiny moves: loss of BLE cannot leave a velocity move running.
                                step = (1 if value > 0 else -1) * max(1, round(abs(value) * 10))
                                await self.pine_command("MPS", motor, current + step, self.speeds[key])
                    self.emit("positions", dict(self.position))
            except Exception as exc:
                self.status(f"Motor fault: {exc}")
                await self.stop()
                self.connected.clear()
                self.emit("connected", [])
            await asyncio.sleep(.1)

    async def stop(self):
        self.motion_epoch += 1
        self.demands.clear()
        self.zaber_target = None
        self.last_zaber_target = None
        if self.scan and self.scan is not asyncio.current_task():
            task = self.scan
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        if self.reference_task and self.reference_task is not asyncio.current_task():
            task = self.reference_task
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        self.homing = False
        self.zaber_speed = 0.
        if self.axis and not self.demo:
            try:
                await self.serial(self.axis.stop, wait_until_idle=False)
            except Exception as exc:
                self.status(f"Zaber stop failed: {exc}")
        # PINE only receives bounded steps; no undocumented stop opcode is sent.
        self.emit("busy", False)

    async def reference(self, key, return_home=False):
        if self.reference_task:
            raise RuntimeError("A reference operation is already active")
        self.reference_task = asyncio.current_task()
        try:
            await self._reference(key, return_home)
        finally:
            self.reference_task = None

    async def _reference(self, key, return_home=False):
        if self.scan or self.detecting or self.homing or key not in self.connected:
            raise RuntimeError("Axis unavailable or busy")
        await self.stop()
        if key == "zaber":
            self.homing = True
            self.emit("busy", True)
            try:
                if self.demo:
                    self.position[key] = 0
                else:
                    from zaber_motion import Units
                    await self.serial(self.axis.settings.set, "maxspeed", self.speeds[key], Units.VELOCITY_MILLIMETRES_PER_SECOND)
                    await self.serial(self.axis.home, wait_until_idle=False)
                    while self.homing and await self.serial(self.axis.is_busy):
                        await self.read_zaber()
                        self.emit("positions", dict(self.position))
                        await asyncio.sleep(.1)
            finally:
                self.homing = False
                self.emit("busy", False)
        elif not return_home:
            if not self.demo:
                motor = self.motor_ports[0 if key == "pine1" else 1]
                self.position[key] = await self.pine_command("MGP", motor, position=True)
            self.references[key] = self.position[key]
            self.status(f"{key}: reference saved at {self.position[key]} (valid until reconnection)")
        else:
            if key not in self.references:
                raise RuntimeError("Set a reference first")
            # Return using the same short steps as jogging, so Stop is always bounded.
            self.homing = True
            self.emit("busy", True)
            try:
                motor = self.motor_ports[0 if key == "pine1" else 1]
                while self.homing:
                    current = self.position[key] if self.demo else await self.pine_command("MGP", motor, position=True)
                    self.position[key] = current
                    delta = self.references[key] - current
                    if abs(delta) < 1:
                        break
                    target = int(current + max(-10, min(10, delta)))
                    if self.demo:
                        self.position[key] = target
                    else:
                        await self.pine_command("MPS", motor, target, self.speeds[key])
                    self.emit("positions", dict(self.position))
                    await asyncio.sleep(.15)
            finally:
                self.homing = False
                self.emit("busy", False)

    async def record(self, config):
        if self.scan or self.detecting or self.homing:
            raise RuntimeError("An operation is already active")
        if not self.phone or "zaber" not in self.connected:
            raise RuntimeError("Connect GridCapture and the Zaber first")
        start = self.position["zaber"] if config["start_current"] else config["start"]
        targets = scan_positions(start, config["end"], config["step"])
        if not all(math.isfinite(config[k]) for k in ("speed", "settle", "timeout")) or config["speed"] <= 0 or config["settle"] < 0 or config["timeout"] <= 0:
            raise ValueError("Invalid speed, settle time or timeout")
        await self.stop()
        self.scan = asyncio.current_task()
        self.emit("busy", True)
        session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        directory = Path(config["output"]).expanduser() / f"session_{session}"
        try:
            (directory / "images").mkdir(parents=True)
            (directory / "metadata").mkdir()
            self.status(f"Saving scan to {directory.resolve()}")
            if not self.demo:
                from zaber_motion import Units
                await self.serial(self.axis.settings.set, "maxspeed", config["speed"], Units.VELOCITY_MILLIMETRES_PER_SECOND)
            for index, target in enumerate(targets):
                if self.demo:
                    self.position["zaber"] = target
                else:
                    await self.serial(self.axis.move_absolute, target, Units.LENGTH_MILLIMETRES, wait_until_idle=False)
                    async with asyncio.timeout(300):
                        while await self.serial(self.axis.is_busy):
                            await self.read_zaber()
                            self.emit("positions", dict(self.position))
                            await asyncio.sleep(.1)
                    await self.read_zaber()
                await asyncio.sleep(config["settle"])
                command = {"type": "capture", "capture_id": f"{session}-{index:05d}",
                           "target_position_mm": target, "measured_position_mm": self.position["zaber"], "sent_at": utc_now()}
                await self.phone.send(json.dumps(command))
                result = await receive_capture_result(self.inbox, command["capture_id"], config["timeout"], directory)
                with (directory / "manifest.jsonl").open("a") as log:
                    log.write(json.dumps({**command, "iphone_result": result, "ack_received_at": utc_now()}) + "\n")
                if not result.get("success"):
                    raise RuntimeError(result.get("error", "Capture failed"))
                self.emit("progress", (index + 1, len(targets)))
            await self.phone.send(json.dumps({"type": "scan_complete", "captures": len(targets)}))
            self.status(f"Scan complete: {len(targets)} photos in {directory}")
        finally:
            self.scan = None
            await self.stop()
            # Late upload packets cannot contaminate a subsequent scan.
            if self.phone:
                await self.phone.close(code=1000, reason="Scan ended; reconnect for next scan")

    async def close(self):
        await self.stop()
        self.ticker.cancel()
        self.watchdog.cancel()
        await asyncio.gather(self.ticker, self.watchdog, return_exceptions=True)
        self.server.close()
        await self.server.wait_closed()
        if self.mdns:
            self.mdns.close()
        if self.pine:
            await self.pine.disconnect()
        if self.zaber:
            await self.serial(self.zaber.connection.close)
        self.pool.shutdown(wait=False, cancel_futures=True)
