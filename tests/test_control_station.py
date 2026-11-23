"""Motor interlocks and the capture/preview wire protocol without hardware."""
import asyncio
import base64
import hashlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from control_station_backend import Station, scan_positions


def test_scan_validation():
    assert scan_positions(2, 0, .75) == [2, 1.25, .5, 0]
    assert scan_positions(1, 1, 1) == [1]
    for args in [(0, float("nan"), 1), (0, 1, 0), (0, 10000, .01)]:
        with pytest.raises(ValueError):
            scan_positions(*args)


def test_deadman_and_interlocks():
    async def run():
        station = Station(lambda *_: None, demo=True)
        station.connected.add("zaber")
        station.jog("zaber", 1.)
        tick = asyncio.create_task(station.tick())
        await asyncio.sleep(.55)
        stopped_at = station.position["zaber"]
        await asyncio.sleep(.25)
        assert station.position["zaber"] == stopped_at
        station.scan = object()
        station.jog("zaber", -1.)
        assert station.demands["zaber"][0] == 1.
        station.scan = None
        await station.stop()
        assert not station.demands
        tick.cancel()
        await asyncio.gather(tick, return_exceptions=True)
        station.pool.shutdown()
    asyncio.run(run())


def test_preview_and_verified_scan(tmp_path):
    async def run():
        import websockets
        events = []
        station = Station(lambda *event: events.append(event), demo=True)
        await station.start("127.0.0.1", 0)
        station.connected.add("zaber")
        port = station.server.sockets[0].getsockname()[1]
        async with websockets.connect(f"ws://127.0.0.1:{port}") as phone:
            assert json.loads(await phone.recv())["mode"] == "control-station"
            scan = asyncio.create_task(station.record(dict(start_current=False, start=0,
                end=1, step=1, speed=1, settle=0, timeout=2, output=str(tmp_path))))
            for _ in range(2):
                command = json.loads(await phone.recv())
                capture_id = command["capture_id"]
                photo, metadata = b"test-photo", b'{}'
                header = dict(type="capture_upload_start", capture_id=capture_id,
                    photo_filename=capture_id + ".jpg", metadata_filename=capture_id + ".json",
                    photo_size=len(photo), photo_sha256=hashlib.sha256(photo).hexdigest(),
                    metadata_size=len(metadata), metadata_sha256=hashlib.sha256(metadata).hexdigest())
                await phone.send(json.dumps(header))
                # Preview may arrive between any two upload messages.
                await phone.send(json.dumps(dict(type="preview", jpeg=base64.b64encode(b"frame").decode())))
                await phone.send(photo)
                await phone.send(metadata)
                await phone.send(json.dumps(dict(type="capture_result", capture_id=capture_id, success=True)))
            assert json.loads(await phone.recv())["type"] == "scan_complete"
            await scan
        assert len(list(tmp_path.glob("*/images/*.jpg"))) == 2
        assert ("preview", b"frame") in events
        await station.close()
    asyncio.run(run())


def test_cancel_scan_stops_axis(tmp_path):
    async def run():
        station = Station(lambda *_: None, demo=True)
        class Phone:
            async def send(self, message): pass
            async def close(self, **kwargs): pass
        station.phone = Phone()
        station.connected.add("zaber")
        task = asyncio.create_task(station.record(dict(start_current=False, start=0,
            end=1, step=1, speed=1, settle=30, timeout=2, output=str(tmp_path))))
        await asyncio.sleep(.05)
        await station.stop()
        assert task.cancelled()
        assert station.scan is None
        assert not station.demands
        station.pool.shutdown()
    asyncio.run(run())


def test_release_during_pending_pine_read_cannot_send_next_step():
    async def run():
        station = Station(lambda *_: None)
        station.connected.add("pine1")
        started, finish = asyncio.Event(), asyncio.Event()
        moves = []
        async def command(name, *args, **kwargs):
            if name == "MGP":
                started.set()
                await finish.wait()
                return 10
            moves.append(args)
        station.pine_command = command
        station.jog("pine1", 1)
        ticker = asyncio.create_task(station.tick())
        await started.wait()
        station.jog("pine1", 0.)
        finish.set()
        await asyncio.sleep(.02)
        ticker.cancel()
        await asyncio.gather(ticker, return_exceptions=True)
        assert moves == []
        station.pool.shutdown()
    asyncio.run(run())


def test_reference_return_cancellable():
    async def run():
        station = Station(lambda *_: None, demo=True)
        station.connected.add("pine1")
        await station.reference("pine1")
        station.position["pine1"] = 10000
        task = asyncio.create_task(station.reference("pine1", True))
        await asyncio.sleep(.02)
        await station.stop()
        assert task.cancelled()
        assert station.reference_task is None
        assert not station.homing
        assert station.position["pine1"] > 0
        station.pool.shutdown()
    asyncio.run(run())


def test_motor_speed_validation_and_commands():
    pytest.importorskip("zaber_motion")
    async def run():
        polled = asyncio.Event()
        station = Station(lambda kind, value: polled.set() if kind == "positions" else None)
        for key, value in [("pine1", 0), ("pine2", 101), ("pine1", 1.5),
                           ("zaber", float("nan")), ("zaber", -.1)]:
            with pytest.raises(ValueError):
                station.set_speed(key, value)
        station.set_speed("pine1", 25)
        station.set_speed("pine2", 40)
        station.set_speed("zaber", 8)
        calls = []
        class Axis:
            def move_velocity(self, velocity, unit): calls.append(("velocity", velocity))
            def get_position(self, unit): return 0
            def stop(self, **kwargs): calls.append(("stop",))
        station.axis = Axis()
        async def pine(name, *args, **kwargs):
            if name == "MGP": return 0
            calls.append((name, *args))
        station.pine_command = pine
        station.connected = {"pine1", "pine2", "zaber"}
        for key in station.connected:
            station.jog(key, .5)
        ticker = asyncio.create_task(station.tick())
        await asyncio.wait_for(polled.wait(), 2)
        assert ("velocity", 4.) in calls
        assert ("MPS", 1, 5, 25) in calls
        assert ("MPS", 3, 5, 40) in calls
        polled.clear()
        station.set_speed("zaber", 10)
        for key in station.connected:
            station.jog(key, .5)
        await asyncio.wait_for(polled.wait(), 2)
        assert ("velocity", 5.) in calls
        await station.stop()
        ticker.cancel()
        await asyncio.gather(ticker, return_exceptions=True)
        station.pool.shutdown()
    asyncio.run(run())


@pytest.mark.parametrize("active_reference", [False, True])
def test_window_actually_exits_on_close(active_reference):
    pytest.importorskip("PySide6")
    import os
    import subprocess
    script = '''
import sys
from types import SimpleNamespace
sys.path.insert(0, "scripts")
from control_station import QApplication, Bridge, Window, QTimer
app = QApplication([])
args = SimpleNamespace(demo=True, host="127.0.0.1", port=0)
bridge = Bridge(args)
assert bridge.ready.wait(5)
window = Window(bridge, args)
window.show()
bridge.submit("detect", "", "", 1, 3).result(5)
window.speeds["pine1"].setValue(25)
window.speeds["zaber"].setValue(7.5)
bridge.submit("jog", "pine1", 0).result(5)
assert bridge.station.speeds["pine1"] == 25
assert bridge.station.speeds["zaber"] == 7.5
app.processEvents()
from PySide6.QtTest import QTest
from PySide6.QtCore import Qt, QPoint
slider = window.dials["zaber"]
assert slider.orientation() == Qt.Orientation.Vertical
assert slider.invertedAppearance()
assert slider.minimum() == 0 and slider.maximum() == 1500
QTest.mousePress(slider, Qt.MouseButton.LeftButton, pos=QPoint(slider.width() // 2, slider.height() - 12))
bridge.submit("jog", "pine1", 0).result(5)
assert slider.value() == 1500
assert bridge.station.zaber_target == 1500
QTest.mouseRelease(slider, Qt.MouseButton.LeftButton)
bridge.submit("jog", "pine1", 0).result(5)
assert bridge.station.zaber_target is None
QTest.mousePress(slider, Qt.MouseButton.LeftButton, pos=QPoint(slider.width() // 2, 12))
bridge.submit("jog", "pine1", 0).result(5)
assert slider.value() == 0
assert bridge.station.zaber_target == 0
QTest.mouseRelease(slider, Qt.MouseButton.LeftButton)
if ACTIVE:
    bridge.submit("reference", "pine1").result(5)
    bridge.station.position["pine1"] = 10000
    bridge.submit("reference", "pine1", True)
QTimer.singleShot(100, window.close)
assert app.exec() == 0
assert window.shutdown_complete
assert not window.isVisible()
bridge.thread.join(3)
assert not bridge.thread.is_alive()
'''.replace("if ACTIVE:", f"if {active_reference}:")
    result = subprocess.run([sys.executable, "-c", script],
        cwd=Path(__file__).resolve().parents[1], env={**os.environ, "QT_QPA_PLATFORM": "offscreen"},
        capture_output=True, text=True, timeout=12)
    assert result.returncode == 0, result.stdout + result.stderr


def test_linear_zaber_target_and_release():
    pytest.importorskip("zaber_motion")
    async def run():
        polled = asyncio.Event()
        station = Station(lambda kind, _: polled.set() if kind == "positions" else None)
        calls = []
        class Settings:
            def set(self, *args): calls.append(("speed", args[1]))
        class Axis:
            settings = Settings()
            def move_absolute(self, target, unit, **kwargs):
                assert kwargs == {"wait_until_idle": False}
                calls.append(("target", target))
            def get_position(self, unit): return 42
            def stop(self, **kwargs): calls.append(("stop",))
        station.axis = Axis()
        station.connected.add("zaber")
        station.set_speed("zaber", 20)
        for target in [-1, 1501, float("nan")]:
            with pytest.raises(ValueError): station.set_zaber_target(target)
        station.set_zaber_target(1200)
        ticker = asyncio.create_task(station.tick())
        try:
            await asyncio.wait_for(polled.wait(), 2)
            assert ("target", 1200) in calls
            assert ("speed", 20) in calls
            polled.clear()
            station.jog("zaber", 0)
            await asyncio.wait_for(polled.wait(), 2)
            assert calls[-1] == ("stop",)
            assert station.zaber_target is None
        finally:
            ticker.cancel()
            await asyncio.gather(ticker, return_exceptions=True)
            station.pool.shutdown()
    asyncio.run(run())
