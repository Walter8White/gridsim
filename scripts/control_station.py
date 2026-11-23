#!/usr/bin/env python3
"""Linux motor control station. Run with --demo to simulate motors."""
import argparse
import asyncio
import sys
import threading
import time

from PySide6.QtCore import Qt, QTimer, Signal, QObject, QEvent
from PySide6.QtGui import QPixmap, QShortcut, QKeySequence
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QFormLayout, QLabel, QPushButton, QLineEdit, QSpinBox,
    QDoubleSpinBox, QTabWidget, QGroupBox, QDial, QPlainTextEdit, QCheckBox,
    QProgressBar, QSplitter, QSlider)
from control_station_backend import Station


class Bridge(QObject):
    event = Signal(str, object)

    def __init__(self, args):
        super().__init__()
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self.run, args=(args,), daemon=True)
        self.thread.start()

    def run(self, args):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.jobs = set()
        self.station = Station(self.event.emit, args.demo)
        try:
            self.loop.run_until_complete(self.station.start(args.host, args.port))
        except Exception as exc:
            self.error = str(exc)
            self.ready.set()
            return
        self.ready.set()
        self.loop.run_forever()
        self.loop.close()

    def submit(self, method, *args):
        async def invoke():
            task = asyncio.current_task()
            self.jobs.add(task)
            try:
                if method == "close":
                    pending = [job for job in self.jobs if job is not task]
                    for job in pending:
                        job.cancel()
                    await asyncio.gather(*pending, return_exceptions=True)
                result = getattr(self.station, method)(*args)
                if asyncio.iscoroutine(result):
                    await result
            except asyncio.CancelledError:
                self.event.emit("status", "Operation stopped")
            except Exception as exc:
                self.event.emit("status", f"Error: {exc}")
            finally:
                self.jobs.discard(task)
        return asyncio.run_coroutine_threadsafe(invoke(), self.loop)


class JogDial(QDial):
    """Spring-centred, mouse-held speed demand; never reacts to wheel/keys."""
    def __init__(self):
        super().__init__()
        self.setRange(-100, 100)
        self.setValue(0)
        self.setNotchesVisible(True)
        self.setMinimumSize(140, 140)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.held = False

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.held = True
            super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        self.reset()

    def reset(self):
        self.held = False
        self.setValue(0)

    def wheelEvent(self, event):
        event.ignore()

    def keyPressEvent(self, event):
        event.ignore()


class LinearAxis(QSlider):
    """Absolute target while held; zero at the top of the vertical travel."""
    def __init__(self):
        super().__init__(Qt.Orientation.Vertical)
        self.setRange(0, 1500)
        self.setInvertedAppearance(True)
        self.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.setTickInterval(250)
        self.setMinimumHeight(220)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.held = False

    def update_target(self, event):
        fraction = (event.position().y() - 12) / max(1, self.height() - 24)
        self.setValue(round(max(0., min(1., fraction)) * 1500))

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.held = True
            self.setSliderDown(True)
            self.update_target(event)

    def mouseMoveEvent(self, event):
        if self.held:
            self.update_target(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.reset()

    def reset(self):
        self.held = False
        self.setSliderDown(False)

    def wheelEvent(self, event):
        event.ignore()

    def keyPressEvent(self, event):
        event.ignore()


class Window(QMainWindow):
    def __init__(self, bridge, args):
        super().__init__()
        self.bridge = bridge
        self.connected = set()
        self.busy = False
        self.phone = False
        self.station_ready = False
        self.frame_time = 0
        self.setWindowTitle("GridSim · Motor control station" + (" · DEMO" if args.demo else ""))
        self.resize(1500, 960)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        heading = QLabel("DEPLOYA : ROBOTICS  /  CONTROL STATION" + (" — DEMO" if args.demo else ""))
        heading.setStyleSheet("font: 900 25px 'DejaVu Sans Mono'; color: #d11f24; letter-spacing: -1px; padding: 8px 2px")
        layout.addWidget(heading)
        bar = QHBoxLayout()
        self.detect = QPushButton("Detect connections")
        self.detect.clicked.connect(self.detect_all)
        bar.addWidget(self.detect)
        self.usb = QLineEdit()
        self.usb.setPlaceholderText("Zaber USB port (blank = auto)")
        bar.addWidget(self.usb)
        self.ble = QLineEdit()
        self.ble.setPlaceholderText("PINE BLE address (blank = scan)")
        bar.addWidget(self.ble)
        self.ports = []
        for label, default in (("PINE A port", 1), ("PINE B port", 3)):
            bar.addWidget(QLabel(label))
            spin = QSpinBox()
            spin.setRange(1, 4)
            spin.setValue(default)
            self.ports.append(spin)
            bar.addWidget(spin)
        layout.addLayout(bar)
        status_row = QHBoxLayout()
        self.connection_label = QLabel("Motors disconnected · iPhone waiting")
        status_row.addWidget(self.connection_label, 1)
        self.phone_connect = QPushButton("Connect")
        self.phone_connect.setEnabled(False)
        self.phone_connect.setToolTip("Open GridCapture on the iPhone; it will connect automatically")
        self.phone_connect.clicked.connect(lambda: self.log.appendPlainText("GridCapture searches for this station automatically."))
        status_row.addWidget(self.phone_connect)
        layout.addLayout(status_row)
        layout.addWidget(QLabel(f"iPhone: open GridCapture on the same LAN → it will find this Linux station automatically over Bonjour (or enter IP manually)"))
        split = QSplitter()
        layout.addWidget(split, 1)
        self.tabs = QTabWidget()
        split.addWidget(self.tabs)
        controls = QWidget()
        controls_layout = QVBoxLayout(controls)
        controls_layout.addWidget(QLabel("PINE: hold and turn a dial. Zaber: hold and drag to a position.\nRelease to stop. Set each motor’s speed below."))
        row = QHBoxLayout()
        self.dials = {}
        self.labels = {}
        self.axis_boxes = {}
        self.speeds = {}
        for key, title in (("pine1", "PINE · Motor A"), ("pine2", "PINE · Motor B"), ("zaber", "Zaber · Axis 1")):
            box = QGroupBox(title)
            column = QVBoxLayout(box)
            readout = QLabel("— mm" if key == "zaber" else "— raw units")
            readout.setAlignment(Qt.AlignmentFlag.AlignCenter)
            column.addWidget(readout)
            self.labels[key] = readout
            if key == "zaber":
                column.addWidget(QLabel("0 mm · Haut"), alignment=Qt.AlignmentFlag.AlignHCenter)
            dial = LinearAxis() if key == "zaber" else JogDial()
            dial.valueChanged.connect(lambda _, k=key: self.send_jog(k))
            dial.sliderReleased.connect(lambda k=key: self.bridge.submit("jog", k, 0.))
            column.addWidget(dial, alignment=Qt.AlignmentFlag.AlignHCenter)
            if key == "zaber":
                column.addWidget(QLabel("1500 mm · Bas"), alignment=Qt.AlignmentFlag.AlignHCenter)
                self.zaber_target_label = QLabel("Cible : —")
                column.addWidget(self.zaber_target_label)
            self.dials[key] = dial
            column.addWidget(QLabel("Jog / home speed (mm/s)" if key == "zaber" else "Motor speed (%)"))
            speed = QDoubleSpinBox() if key == "zaber" else QSpinBox()
            speed.setRange(.001 if key == "zaber" else 1, 100)
            if key == "zaber":
                speed.setDecimals(3)
                speed.setSingleStep(.1)
            speed.setValue(1)
            speed.setSuffix(" mm/s" if key == "zaber" else " %")
            speed.valueChanged.connect(lambda value, k=key: self.bridge.submit("set_speed", k, value))
            self.speeds[key] = speed
            column.addWidget(speed)
            button = QPushButton("Home to switch" if key == "zaber" else "Set reference here")
            button.clicked.connect(lambda _, k=key: self.bridge.submit("reference", k))
            column.addWidget(button)
            if key != "zaber":
                button = QPushButton("Return to reference")
                button.clicked.connect(lambda _, k=key: self.bridge.submit("reference", k, True))
                column.addWidget(button)
            self.axis_boxes[key] = box
            box.setEnabled(False)
            row.addWidget(box)
        controls_layout.addLayout(row)
        note = QLabel("PINE references are session-only, not physical homing.\nSet the reference at your chosen origin; reconnecting clears it.\nPINE may finish its last short step after release. Keep hardware power-off accessible.")
        note.setWordWrap(True)
        controls_layout.addWidget(note)
        controls_layout.addStretch()
        self.tabs.addTab(controls, "Manual control")
        recording = QWidget()
        form = QFormLayout(recording)
        self.fields = {}
        for key, label, default, minimum, maximum in (
            ("start", "Start (mm)", 0, -100000, 100000),
            ("end", "End (mm)", 2, -100000, 100000),
            ("step", "Step (mm)", 1, .001, 100000),
            ("speed", "Travel speed (mm/s)", 1, .001, 100),
            ("settle", "Settle before photo (s)", .5, 0, 60),
            ("timeout", "Photo/upload timeout (s)", 60, 1, 600)):
            spin = QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setRange(minimum, maximum)
            spin.setValue(default)
            self.fields[key] = spin
            form.addRow(label, spin)
        self.start_current = QCheckBox("Start at current Zaber position")
        self.start_current.setChecked(True)
        self.fields["start"].setEnabled(False)
        self.start_current.toggled.connect(lambda value: self.fields["start"].setEnabled(not value))
        form.addRow(self.start_current)
        self.output = QLineEdit("captures/iphone_zaber")
        form.addRow("Save directory", self.output)
        self.record = QPushButton("Start photo scan")
        self.record.clicked.connect(self.start_recording)
        form.addRow(self.record)
        self.progress = QProgressBar()
        self.progress.setValue(0)
        form.addRow(self.progress)
        form.addRow(QLabel("Move → stop → settle → photo → verified upload.\nMaximum 1,000 photos per scan. Stage limits remain authoritative."))
        self.tabs.addTab(recording, "Photo recording")
        camera_box = QWidget()
        camera_layout = QVBoxLayout(camera_box)
        self.camera_status = QLabel("CAMERA · waiting for GridCapture")
        camera_layout.addWidget(self.camera_status)
        self.camera = QLabel("Live iPhone preview appears here\nin both tabs")
        self.camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera.setMinimumSize(560, 420)
        self.camera.setStyleSheet("background: #080e17; border: 1px solid #304155; border-radius: 12px")
        camera_layout.addWidget(self.camera, 1)
        split.addWidget(camera_box)
        split.setSizes([760, 700])
        stop = QPushButton("STOP ALL  ·  Esc")
        stop.setStyleSheet("background: #d11f24; color: white; font-size: 20px; padding: 12px")
        stop.clicked.connect(self.stop)
        actions = QHBoxLayout()
        actions.addWidget(stop, 1)
        self.quit_button = QPushButton("Quitter · Ctrl+Q")
        self.quit_button.clicked.connect(self.close)
        actions.addWidget(self.quit_button)
        layout.addLayout(actions)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(250)
        self.log.setMaximumHeight(120)
        layout.addWidget(self.log)
        QShortcut(QKeySequence("Escape"), self, activated=self.stop)
        QShortcut(QKeySequence("Ctrl+Q"), self, activated=self.close)
        self.tabs.currentChanged.connect(self.release_jogs)
        QApplication.instance().installEventFilter(self)
        bridge.event.connect(self.on_event)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.heartbeat)
        self.timer.start(100)
        self.refresh()

    def send_jog(self, key):
        dial = self.dials[key]
        if key == "zaber" and dial.held:
            self.zaber_target_label.setText(f"Cible : {dial.value()} mm")
            self.bridge.submit("set_zaber_target", dial.value())
        else:
            self.bridge.submit("jog", key, dial.value()/100 if dial.held else 0.)

    def heartbeat(self):
        for key, dial in self.dials.items():
            if dial.held:
                self.send_jog(key)
        if self.frame_time and time.monotonic() - self.frame_time > 2:
            self.camera_status.setText("CAMERA · preview stale / waiting for frames")
            self.camera.clear()
            self.camera.setText("No recent camera frame")

    def release_jogs(self, *_):
        for key, dial in self.dials.items():
            dial.reset()
            self.bridge.submit("jog", key, 0.)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.ApplicationDeactivate:
            self.release_jogs()
        return super().eventFilter(obj, event)

    def stop(self):
        self.release_jogs()
        self.bridge.submit("stop")

    def detect_all(self):
        self.release_jogs()
        self.detect.setEnabled(False)
        future = self.bridge.submit("detect", self.usb.text().strip(), self.ble.text().strip(), *(s.value() for s in self.ports))
        future.add_done_callback(lambda _: self.bridge.event.emit("detection_done", None))

    def start_recording(self):
        self.release_jogs()
        config = {key: field.value() for key, field in self.fields.items()}
        config.update(start_current=self.start_current.isChecked(), output=self.output.text().strip())
        self.progress.setValue(0)
        self.bridge.submit("record", config)

    def refresh(self):
        for key, box in self.axis_boxes.items():
            box.setEnabled(key in self.connected and not self.busy)
        self.record.setEnabled(self.phone and "zaber" in self.connected and not self.busy)
        self.detect.setEnabled(not self.busy)
        self.connection_label.setText("Motors: " + (", ".join(sorted(self.connected)) or "disconnected") + " · iPhone: " + ("connected" if self.phone else "ready to discover" if self.station_ready else "waiting"))
        self.phone_connect.setEnabled(self.phone or self.station_ready)
        self.phone_connect.setText("Connected" if self.phone else "Connect")
        self.phone_connect.setStyleSheet("background: #2476c7; color: white;" if self.phone else "background: #28a66a; color: white;" if self.station_ready else "background: #39414c; color: #89929e;")

    def on_event(self, kind, value):
        if getattr(self, "closing", False) and kind != "status":
            return
        if kind == "status":
            self.log.appendPlainText(value)
        elif kind == "station_ready":
            self.station_ready = bool(value)
            self.refresh()
        elif kind == "connected":
            self.connected = set(value)
            self.refresh()
        elif kind == "busy":
            self.busy = value
            self.release_jogs()
            self.refresh()
        elif kind == "phone":
            self.phone = value
            if not value:
                self.camera.clear()
                self.camera.setText("iPhone disconnected")
                self.camera_status.setText("CAMERA · disconnected")
                self.frame_time = 0
            self.refresh()
        elif kind == "positions":
            for key, position in value.items():
                self.labels[key].setText(f"{position:.3f} mm" if key == "zaber" else f"{position:.0f} raw units")
            slider = self.dials["zaber"]
            if not slider.held:
                slider.blockSignals(True)
                slider.setValue(round(value["zaber"]))
                slider.blockSignals(False)
                self.zaber_target_label.setText("Maintenir pour déplacer")
        elif kind == "preview":
            pixmap = QPixmap()
            if pixmap.loadFromData(value, "JPEG"):
                self.camera.setPixmap(pixmap.scaled(self.camera.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
                self.frame_time = time.monotonic()
                self.camera_status.setText("CAMERA · live")
        elif kind == "progress":
            self.progress.setRange(0, value[1])
            self.progress.setValue(value[0])
        elif kind == "detection_done":
            self.refresh()

    def closeEvent(self, event):
        if getattr(self, "shutdown_complete", False):
            event.accept()
            return
        if getattr(self, "closing", False):
            event.ignore()
            return
        self.closing = True
        self.timer.stop()
        self.release_jogs()
        self.setEnabled(False)
        self.log.appendPlainText("Stopping motors and closing connections…")
        self.bridge.event.connect(self.finish_close)
        future = self.bridge.submit("close")
        future.add_done_callback(lambda _: self.bridge.event.emit("shutdown", None))
        event.ignore()

    def finish_close(self, kind, _):
        if kind == "shutdown":
            self.shutdown_complete = True
            QApplication.instance().removeEventFilter(self)
            self.bridge.loop.call_soon_threadsafe(self.bridge.loop.stop)
            self.close()
            QApplication.instance().quit()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    if sys.version_info < (3, 11):
        parser.error("The control station requires Python 3.11 or newer")
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet("QWidget { background: #ffffff; color: #222222; font-family: 'DejaVu Sans Mono'; } QPushButton, QLineEdit, QSpinBox, QDoubleSpinBox { padding: 7px; border: 1px solid #d7d2d0; border-radius: 5px; background: #fff; } QPushButton { color: #cf2027; font-weight: bold; } QPushButton:disabled { color: #96908d; background: #f2f0ef; } QGroupBox { padding-top: 20px; font-weight: bold; border: 1px solid #e4dfdd; border-radius: 8px; margin-top: 8px; } QTabBar::tab { padding: 12px; color: #444; } QTabBar::tab:selected { color: #c91e24; border-bottom: 3px solid #d11f24; }")
    bridge = Bridge(args)
    bridge.ready.wait(10)
    if not bridge.ready.is_set() or hasattr(bridge, "error"):
        print(getattr(bridge, "error", "Backend startup timed out"), file=sys.stderr)
        return 1
    window = Window(bridge, args)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
