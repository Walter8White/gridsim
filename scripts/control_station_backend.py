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
        self.pine_address = None
        self.zaber = None
        self.zaber_port = None
        self.zaber_homed = False
        self.axis = None
        self.phone = None
        self.inbox = CaptureInbox()
        self.motor_ports = (1, 2)
        self.position = {"pine1": 0., "pine2": 0., "zaber": 0.}
        self.references = {}
        self.pine_targets = {"pine1": None, "pine2": None}
        self.pine_commanded_targets = {"pine1": None, "pine2": None}
        self.demands = {}
        self.last_detection_errors = {}
        self.connected = set()
        self.scan = None
        self.loop_demo_task = None
        self.loop_demo_running = False
        self.loop_demo_starting = False
        self.reference_task = None
        self.detecting = False
        self.homing = False
        self.pine_lock = asyncio.Lock()
        self.notifications = asyncio.Queue()
        self.buffer = ""
        self.zaber_speed = 0.
        self.zaber_target = None
        self.zaber_tracking = False
        self.zaber_velocity_jog = False
        self.zaber_target_updated = 0.
        self.zaber_integral = 0.
        self.zaber_previous_error = None
        self.zaber_control_time = time.monotonic()
        self.zaber_configured_speed = None
        self.motion_epoch = 0
        self.speeds = {"pine1": 1, "pine2": 1, "zaber": 40.}

    def status(self, message):
        self.emit("status", message)

    async def set_camera_setting(self, settings):
        if not self.phone:
            raise RuntimeError("GridCapture is not connected")
        await self.phone.send(json.dumps({"type": "camera_settings", "settings": settings}))

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
                    if message.get("type") == "camera_state":
                        self.emit("camera_state", message)
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

    async def detect(self, port, address, motor1, motor2, automatic=False):
        if self.detecting or self.scan or self.homing or self.loop_demo_running or self.loop_demo_starting:
            raise RuntimeError("Stop the current operation before detecting")
        # The GUI sends zero-demand updates when it releases controls. Those
        # entries are not active moves and must not block startup detection.
        if automatic and any(value for value, _ in self.demands.values()):
            return
        if automatic and self.zaber_target is not None and abs(self.zaber_speed) > .1:
            return
        if motor1 == motor2:
            raise ValueError("Choose two different PINE ports")
        self.detecting = True
        if not automatic:
            await self.stop()
        try:
            self.motor_ports = (motor1, motor2)
            if self.demo:
                self.connected = {"pine1", "pine2", "zaber"}
                self.position["zaber"] = 0.
                self.zaber_homed = True
                self.emit("zaber_homed", True)
                return
            if not automatic:
                self.references.clear()
                self.zaber_homed = False
                self.emit("zaber_homed", False)
                if self.pine:
                    await self.pine.disconnect()
                    self.pine = None
                    self.pine_address = None
                if self.zaber:
                    await self.serial(self.zaber.connection.close)
                    self.zaber = self.axis = None
                    self.zaber_port = None
                    self.zaber_homed = False
                self.connected.clear()
                self.emit("connected", [])
            if self.pine is None:
                try:
                    from bleak import BleakClient
                    pine_address = await find_pine(address or None)
                    self.pine = BleakClient(pine_address,
                                            disconnected_callback=lambda _: self.pine_disconnected())
                    await self.pine.connect()
                    await self.pine.start_notify(CHARACTERISTIC_UUID, self.notify)
                    await self.pine.write_gatt_char(CHARACTERISTIC_UUID, b"<0,ID,1,1>\r\n", response=True)
                    await asyncio.sleep(.2)
                    for key, motor in zip(("pine1", "pine2"), self.motor_ports):
                        self.position[key] = await self.pine_command("MGP", motor, position=True)
                        self.pine_targets[key] = None
                        self.pine_commanded_targets[key] = None
                    self.pine_address = str(pine_address)
                    self.connected.update(("pine1", "pine2"))
                    self.last_detection_errors.pop("pine", None)
                    self.status("PINE II connected · BLE")
                except (Exception, SystemExit) as exc:
                    message = str(exc) or type(exc).__name__
                    if not automatic or self.last_detection_errors.get("pine") != message:
                        self.status(f"PINE II: {exc}")
                    self.last_detection_errors["pine"] = message
                    if self.pine:
                        await self.pine.disconnect()
                        self.pine = None
            if self.zaber is None:
                try:
                    from serial.tools import list_ports
                    from zaber_stage import detect_ascii
                    candidates = [port] if port else [
                        item.device for item in list_ports.comports()
                        if item.vid is not None or item.device.startswith(("/dev/ttyUSB", "/dev/ttyACM"))
                    ]
                    for candidate in candidates:
                        result = None
                        try:
                            result = await self.serial(detect_ascii, candidate, True)
                            if result.devices:
                                self.zaber = result
                                self.zaber_port = candidate
                                self.axis = result.devices[0].get_axis(1)
                                await self.read_zaber()
                                self.zaber_homed = await self.serial(self.axis.is_homed)
                                self.connected.add("zaber")
                                self.emit("zaber_homed", self.zaber_homed)
                                self.last_detection_errors.pop("zaber", None)
                                self.status(f"Zaber connected · {candidate} · " + ("homed" if self.zaber_homed else "home required"))
                                break
                        except Exception as exc:
                            message = f"{candidate}: {exc}"
                            if not automatic or self.last_detection_errors.get("zaber") != message:
                                self.status(message)
                            self.last_detection_errors["zaber"] = message
                            if self.zaber is result:
                                self.zaber = self.axis = None
                                self.zaber_port = None
                                self.zaber_homed = False
                        if result:
                            await self.serial(result.connection.close)
                            self.zaber = self.axis = None
                            self.zaber_port = None
                    if self.zaber is None:
                        message = "No Zaber ASCII axis found on USB ports"
                        if not automatic or self.last_detection_errors.get("zaber") != message:
                            self.status(message + "; check USB permissions or select a port in settings")
                        self.last_detection_errors["zaber"] = message
                except Exception as exc:
                    message = str(exc) or type(exc).__name__
                    if not automatic or self.last_detection_errors.get("zaber") != message:
                        self.status(f"Zaber: {exc}")
                    self.last_detection_errors["zaber"] = message
        finally:
            self.detecting = False
            self.emit("connected", sorted(self.connected))
            self.emit("positions", dict(self.position))
            self.emit("connection_details", {"pine": self.pine_address, "zaber": self.zaber_port})
            zaber_state = "disconnected" if "zaber" not in self.connected else "homed" if self.zaber_homed else "connected · home required"
            self.status("Detection complete · PINE " + ("connected" if {"pine1", "pine2"}.issubset(self.connected) else "disconnected")
                        + " · Zaber " + zaber_state)

    def pine_disconnected(self):
        self.pine = None
        self.pine_address = None
        self.connected.difference_update(("pine1", "pine2"))
        self.demands.pop("pine1", None)
        self.demands.pop("pine2", None)
        self.references.clear()
        self.emit("connected", sorted(self.connected))
        self.emit("connection_details", {"pine": None, "zaber": self.zaber_port})

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
        if key in self.pine_commanded_targets:
            self.pine_commanded_targets[key] = None

    def set_pine_target(self, key, target):
        if key not in self.pine_targets or not math.isfinite(target):
            raise ValueError("Invalid PINE target")
        target = int(round(target))
        if not -(2**31) <= target < 2**31:
            raise ValueError("PINE target is outside the controller position range")
        if (key not in self.connected or self.scan or self.detecting or self.homing):
            return
        self.pine_targets[key] = target
        self.demands[key] = (0., time.monotonic())
        self.emit("pine_target", {"key": key, "target": target})

    def set_zaber_target(self, target):
        if not math.isfinite(target) or not 0 <= target <= 1500:
            raise ValueError("Zaber target must be between 0 and 1500 mm")
        if "zaber" not in self.connected or not self.zaber_homed or self.scan or self.detecting or self.homing:
            return
        target = float(target)
        if self.zaber_target is None or abs(target - self.zaber_target) > .001:
            self.zaber_integral = 0.
            self.zaber_previous_error = None
        self.zaber_velocity_jog = False
        self.demands["zaber"] = (1., time.monotonic())
        self.zaber_target = target
        self.zaber_tracking = True
        self.zaber_target_updated = time.monotonic()

    def release_zaber_target(self):
        """Freeze the last slider setpoint and keep tracking it after release."""
        if "zaber" not in self.connected or not self.zaber_homed or self.zaber_target is None:
            return
        self.zaber_tracking = False
        self.zaber_velocity_jog = False
        self.demands["zaber"] = (0., time.monotonic())
        self.zaber_integral = 0.
        self.zaber_previous_error = None
        self.zaber_target_updated = time.monotonic()
        self.zaber_control_time = self.zaber_target_updated

    def jog(self, key, value):
        if not math.isfinite(value):
            raise ValueError("Invalid jog speed")
        if value and (key not in self.connected or (key == "zaber" and not self.zaber_homed)
                      or self.scan or self.detecting or self.homing):
            return
        if key == "zaber":
            self.zaber_target = None
            self.zaber_tracking = False
            self.zaber_velocity_jog = bool(value)
            self.zaber_integral = 0.
            self.zaber_previous_error = None
        self.demands[key] = (max(-1., min(1., value)), time.monotonic())

    async def watch_deadman(self):
        while True:
            await asyncio.sleep(.05)
            if (self.zaber_tracking and self.zaber_target is not None
                    and time.monotonic() - self.zaber_target_updated >= .4):
                self.release_zaber_target()

    async def set_zaber_velocity(self, velocity, epoch):
        from zaber_motion import Units
        limit = self.speeds["zaber"]
        velocity = max(-limit, min(limit, velocity))
        if abs(velocity) < .05:
            velocity = 0.
        if velocity == 0.:
            if self.zaber_speed != 0.:
                await self.serial(self.axis.stop, wait_until_idle=False)
                self.zaber_speed = 0.
            return
        if self.zaber_configured_speed != limit:
            await self.serial(self.axis.settings.set, "maxspeed", limit, Units.VELOCITY_MILLIMETRES_PER_SECOND)
            self.zaber_configured_speed = limit
        if epoch != self.motion_epoch:
            return
        if abs(velocity - self.zaber_speed) >= .15:
            await self.serial(self.axis.move_velocity, velocity, Units.VELOCITY_MILLIMETRES_PER_SECOND)
            self.zaber_speed = velocity

    async def update_zaber_servo(self, epoch):
        """PID position follower; sends velocity updates, not repeated absolute moves."""
        if self.demo:
            if self.zaber_velocity_jog:
                value, stamp = self.demands.get("zaber", (0., 0.))
                value = value if time.monotonic() - stamp < .4 else 0.
                self.position["zaber"] += value * self.speeds["zaber"] * .1
            elif self.zaber_target is not None:
                error = self.zaber_target - self.position["zaber"]
                step = min(abs(error), self.speeds["zaber"] * .1)
                self.position["zaber"] += math.copysign(step, error) if step else 0.
            return
        if self.zaber_velocity_jog:
            value, stamp = self.demands.get("zaber", (0., 0.))
            value = value if time.monotonic() - stamp < .4 else 0.
            await self.set_zaber_velocity(value * self.speeds["zaber"], epoch)
            return
        target = self.zaber_target
        if target is None:
            await self.set_zaber_velocity(0., epoch)
            return
        now = time.monotonic()
        dt = min(.25, max(.01, now - self.zaber_control_time))
        self.zaber_control_time = now
        error = target - self.position["zaber"]
        derivative = 0. if self.zaber_previous_error is None else (error - self.zaber_previous_error) / dt
        self.zaber_previous_error = error
        if abs(error) <= .05:
            self.zaber_integral = 0.
            velocity = 0.
        else:
            self.zaber_integral += error * dt
            integral_limit = self.speeds["zaber"] / (10. * .12)
            self.zaber_integral = max(-integral_limit, min(integral_limit, self.zaber_integral))
            velocity = 2.0 * error + .12 * self.zaber_integral + .08 * derivative
        await self.set_zaber_velocity(velocity, epoch)

    async def tick(self):
        while True:
            try:
                if not self.scan and not self.homing and not self.loop_demo_running:
                    epoch = self.motion_epoch
                    for key in tuple(self.connected):
                        if epoch != self.motion_epoch or self.scan or self.homing:
                            break
                        if self.detecting and key != "zaber":
                            continue
                        demand, stamp = self.demands.get(key, (0., 0.))
                        value = demand if time.monotonic() - stamp < .4 else 0.
                        if key == "zaber":
                            await self.update_zaber_servo(epoch)
                            if not self.demo:
                                await self.read_zaber()
                        elif key in self.pine_targets:
                            motor = self.motor_ports[0 if key == "pine1" else 1]
                            if self.demo:
                                target = self.pine_targets[key]
                                if target is not None:
                                    distance = target - self.position[key]
                                    step = min(abs(distance), max(1., self.speeds[key] * 2.))
                                    self.position[key] += math.copysign(step, distance) if step else 0.
                            else:
                                self.position[key] = await self.pine_command("MGP", motor, position=True)
                                target = self.pine_targets[key]
                                if (target is not None and target != self.pine_commanded_targets[key]
                                        and epoch == self.motion_epoch and not self.scan and not self.homing):
                                    await self.pine_command("MPS", motor, target, self.speeds[key])
                                    self.pine_commanded_targets[key] = target
                        elif self.demo:
                            self.position[key] += value * self.speeds[key] * 2
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
                if self.pine:
                    try:
                        await self.pine.disconnect()
                    except Exception:
                        pass
                if self.zaber:
                    try:
                        await self.serial(self.zaber.connection.close)
                    except Exception:
                        pass
                self.pine = self.zaber = self.axis = None
                self.pine_address = self.zaber_port = None
                self.zaber_homed = False
                self.emit("zaber_homed", False)
                self.emit("connected", [])
                self.emit("connection_details", {"pine": None, "zaber": None})
            await asyncio.sleep(.1)

    async def stop(self):
        self.motion_epoch += 1
        was_starting_loop = self.loop_demo_starting
        self.loop_demo_starting = False
        if was_starting_loop:
            self.emit("loop_demo_starting", False)
        self.demands.clear()
        self.pine_targets = {key: None for key in self.pine_targets}
        self.zaber_target = None
        self.zaber_tracking = False
        self.zaber_velocity_jog = False
        self.zaber_integral = 0.
        self.zaber_previous_error = None
        if self.scan and self.scan is not asyncio.current_task():
            task = self.scan
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        if self.loop_demo_task and self.loop_demo_task is not asyncio.current_task():
            task = self.loop_demo_task
            self.loop_demo_task = None
            self.loop_demo_running = False
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
        # An accepted PINE position move cannot be cancelled via a verified BLE stop command.
        self.emit("busy", False)

    async def start_loop_demo(self, pine_units_per_revolution):
        if self.scan or self.homing or self.detecting or self.loop_demo_running or self.loop_demo_starting:
            raise RuntimeError("Stop the current operation before starting Loop demo")
        if not {"pine1", "pine2", "zaber"}.issubset(self.connected) or not self.zaber_homed:
            raise RuntimeError("Connect both PINE motors and home the Zaber first")
        units = {key: float(pine_units_per_revolution.get(key, 0)) for key in ("pine1", "pine2")}
        if self.demo:
            units = {key: value if value > 0 else 200. for key, value in units.items()}
        if any(not math.isfinite(value) or value <= 0 for value in units.values()):
            raise RuntimeError("Set both PINE raw units per revolution in Settings first")
        pending_pine_targets = dict(self.pine_commanded_targets)
        await self.stop()
        self.loop_demo_starting = True
        self.emit("loop_demo_starting", True)
        try:
            if not self.demo:
                deadline = time.monotonic() + 300
                while True:
                    if not self.loop_demo_starting:
                        return
                    pending = False
                    for key, motor in zip(("pine1", "pine2"), self.motor_ports):
                        current = await self.pine_command("MGP", motor, position=True)
                        self.position[key] = current
                        target = pending_pine_targets.get(key)
                        pending = pending or (target is not None and abs(current - target) > 2)
                    if not pending:
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Waited for the current PINE move to finish; Loop demo was not started")
                    await asyncio.sleep(.1)
                self.position["zaber"] = await self.read_zaber()
            if not self.loop_demo_starting:
                return
            centers = {key: int(round(self.position[key])) for key in ("pine1", "pine2")}
            self.loop_demo_running = True
            self.loop_demo_starting = False
            self.motion_epoch += 1
            self.loop_demo_task = asyncio.create_task(self._run_loop_demo(centers, units, self.motion_epoch))
        except Exception:
            self.loop_demo_starting = False
            self.emit("loop_demo_starting", False)
            raise

    async def _run_loop_demo(self, centers, units, epoch):
        self.emit("loop_demo", True)
        self.emit("busy", True)
        self.status("LOOP DEMO · ZABER 300–900 mm @ 40 mm/s · PINE ±45° opposition")
        phase = 1
        endpoint = 300.
        try:
            if not self.demo:
                from zaber_motion import Units
                await self.serial(self.axis.settings.set, "maxspeed", 40., Units.VELOCITY_MILLIMETRES_PER_SECOND)
                self.zaber_configured_speed = 40.
            while self.loop_demo_running and epoch == self.motion_epoch:
                pine_targets = {
                    "pine1": centers["pine1"] + round(phase * units["pine1"] / 8),
                    "pine2": centers["pine2"] - round(phase * units["pine2"] / 8),
                }
                if self.demo:
                    start_zaber = self.position["zaber"]
                    start_pine = {key: self.position[key] for key in pine_targets}
                    durations = [abs(endpoint - start_zaber) / 40.]
                    for key, target in pine_targets.items():
                        durations.append(abs(target - start_pine[key]) / max(1., self.speeds[key] * 10.))
                    duration = max(durations)
                    started = time.monotonic()
                    while self.loop_demo_running and time.monotonic() - started < duration:
                        fraction = min(1., (time.monotonic() - started) / max(.001, duration))
                        self.position["zaber"] = start_zaber + (endpoint - start_zaber) * fraction
                        for key, target in pine_targets.items():
                            self.position[key] = start_pine[key] + (target - start_pine[key]) * fraction
                        self.emit("positions", dict(self.position))
                        await asyncio.sleep(.05)
                else:
                    for key, motor in zip(("pine1", "pine2"), self.motor_ports):
                        await self.pine_command("MPS", motor, pine_targets[key], self.speeds[key])
                        self.pine_commanded_targets[key] = pine_targets[key]
                    await self.serial(self.axis.move_absolute, endpoint, Units.LENGTH_MILLIMETRES, wait_until_idle=False)
                    while self.loop_demo_running and epoch == self.motion_epoch:
                        self.position["zaber"] = await self.read_zaber()
                        for key, motor in zip(("pine1", "pine2"), self.motor_ports):
                            self.position[key] = await self.pine_command("MGP", motor, position=True)
                        self.emit("positions", dict(self.position))
                        zaber_busy = await self.serial(self.axis.is_busy)
                        pine_done = all(abs(self.position[key] - pine_targets[key]) <= 2 for key in pine_targets)
                        if not zaber_busy and pine_done:
                            break
                        await asyncio.sleep(.1)
                if not self.loop_demo_running or epoch != self.motion_epoch:
                    break
                endpoint = 900. if endpoint == 300. else 300.
                phase *= -1
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.status(f"LOOP DEMO ERROR · {exc}")
            await self.stop()
        finally:
            self.loop_demo_running = False
            self.loop_demo_task = None
            self.emit("loop_demo", False)
            self.emit("busy", False)
            if epoch == self.motion_epoch:
                self.status("LOOP DEMO · stopped")

    async def reference(self, key, return_home=False):
        if self.reference_task:
            raise RuntimeError("A reference operation is already active")
        self.reference_task = asyncio.current_task()
        try:
            await self._reference(key, return_home)
        finally:
            self.reference_task = None

    async def home_zaber(self):
        if self.loop_demo_running or self.loop_demo_starting:
            raise RuntimeError("Stop Loop demo before homing")
        if self.demo:
            self.position["zaber"] = 0.
            self.zaber_homed = True
            self.emit("zaber_homed", True)
            return
        if self.axis is None:
            raise RuntimeError("Zaber axis is not connected")
        from zaber_motion import Units
        self.zaber_homed = False
        self.homing = True
        self.emit("busy", True)
        self.emit("zaber_homing", True)
        try:
            await self.serial(self.axis.settings.set, "maxspeed", self.speeds["zaber"], Units.VELOCITY_MILLIMETRES_PER_SECOND)
            self.zaber_configured_speed = self.speeds["zaber"]
            await self.serial(self.axis.home, wait_until_idle=False)
            while self.homing and await self.serial(self.axis.is_busy):
                await self.read_zaber()
                self.emit("positions", dict(self.position))
                await asyncio.sleep(.1)
            if not self.homing:
                raise RuntimeError("Zaber homing stopped before completion")
            await self.read_zaber()
            self.zaber_homed = True
            self.emit("zaber_homed", True)
        except Exception:
            self.zaber_homed = False
            self.emit("zaber_homed", False)
            raise
        finally:
            self.homing = False
            self.emit("zaber_homing", False)
            self.emit("busy", False)

    async def _reference(self, key, return_home=False):
        if self.scan or self.detecting or self.homing or self.loop_demo_running or self.loop_demo_starting or key not in self.connected:
            raise RuntimeError("Axis unavailable or busy")
        await self.stop()
        if key == "zaber":
            await self.home_zaber()
        elif not return_home:
            if not self.demo:
                motor = self.motor_ports[0 if key == "pine1" else 1]
                self.position[key] = await self.pine_command("MGP", motor, position=True)
            self.references[key] = self.position[key]
            self.pine_targets[key] = int(round(self.position[key]))
            self.pine_commanded_targets[key] = self.pine_targets[key]
            self.emit("pine_target", {"key": key, "target": self.pine_targets[key]})
            self.status(f"{key}: reference saved at {self.position[key]} (valid until reconnection)")
        else:
            if key not in self.references:
                raise RuntimeError("Set a reference first")
            self.set_pine_target(key, self.references[key])

    async def record(self, config):
        if self.scan or self.detecting or self.homing or self.loop_demo_running or self.loop_demo_starting:
            raise RuntimeError("An operation is already active")
        if not self.phone or "zaber" not in self.connected or not self.zaber_homed:
            raise RuntimeError("Connect GridCapture and home the Zaber first")
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
