#!/usr/bin/env python3
"""Linux motor control station. Run with --demo to simulate motors."""
import argparse
import asyncio
import sys
import threading
import time
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal, QObject, QEvent
from PySide6.QtGui import QPixmap, QShortcut, QKeySequence, QTransform
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QFormLayout, QLabel, QPushButton, QLineEdit, QSpinBox,
    QDoubleSpinBox, QTabWidget, QGroupBox, QDial, QPlainTextEdit, QCheckBox,
    QProgressBar, QSplitter, QSlider, QComboBox, QToolButton, QDialog)
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
    """Circular PINE position trim; mouse dragging changes a raw-position target."""
    dragStarted = Signal()

    def __init__(self):
        super().__init__()
        self.setRange(-100, 100)
        self.setValue(0)
        self.setNotchesVisible(True)
        self.setMinimumSize(140, 140)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.held = False
        self.anchor_position = 0.

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.held = True
            self.dragStarted.emit()
            self.setProperty("jogging", True)
            self.style().unpolish(self)
            self.style().polish(self)
            super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        self.reset()

    def reset(self):
        self.held = False
        self.setProperty("jogging", False)
        self.style().unpolish(self)
        self.style().polish(self)

    def wheelEvent(self, event):
        event.ignore()

    def keyPressEvent(self, event):
        event.ignore()


class LinearAxis(QSlider):
    """Fine absolute-position target while held; zero at the top."""
    def __init__(self):
        super().__init__(Qt.Orientation.Vertical)
        self.setRange(0, 15000)  # 0.1 mm per slider step
        self.setInvertedAppearance(True)
        self.setTickPosition(QSlider.TickPosition.TicksBothSides)
        self.setTickInterval(2500)
        self.setMinimumHeight(220)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.held = False

    def update_target(self, event):
        fraction = (event.position().y() - 12) / max(1, self.height() - 24)
        self.setValue(round(max(0., min(1., fraction)) * 15000))

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.held = True
            self.setProperty("jogging", True)
            self.style().unpolish(self)
            self.style().polish(self)
            self.setSliderDown(True)
            self.update_target(event)

    def mouseMoveEvent(self, event):
        if self.held:
            self.update_target(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            super().mouseReleaseEvent(event)
            self.reset()

    def reset(self):
        self.held = False
        self.setSliderDown(False)
        self.setProperty("jogging", False)
        self.style().unpolish(self)
        self.style().polish(self)

    def wheelEvent(self, event):
        event.ignore()

    def keyPressEvent(self, event):
        event.ignore()


class Window(QMainWindow):
    def __init__(self, bridge, args):
        super().__init__()
        self.bridge = bridge
        self.demo_mode = args.demo
        self.connected = set()
        self.busy = False
        self.phone = False
        self.station_ready = False
        self.zaber_homing = False
        self.zaber_homed = False
        self.zaber_target_active = False
        self.positions = {"pine1": 0., "pine2": 0., "zaber": 0.}
        self.pine_target_active = {"pine1": False, "pine2": False}
        self.pine_targets_ui = {"pine1": None, "pine2": None}
        demo_units = 200. if self.demo_mode else 0.
        self.pine_units_per_revolution = {"pine1": demo_units, "pine2": demo_units}
        self.pine_units_fields = {}
        self.loop_demo_active = False
        self.loop_demo_starting = False
        self.detected_details = {"pine": None, "zaber": None}
        self.frame_time = 0
        self.setWindowTitle("GridSim · Motor control station" + (" · DEMO" if args.demo else ""))
        self.resize(1500, 960)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        brand_row = QHBoxLayout()
        heading = QLabel()
        logo = QPixmap(str(Path(__file__).resolve().parent / "assets" / "deploya-wordmark.png"))
        heading.setPixmap(logo.scaledToHeight(42, Qt.TransformationMode.SmoothTransformation))
        heading.setAccessibleName("déploya:robotics")
        brand_row.addWidget(heading)
        brand_row.addSpacing(16)
        product_name = QLabel("CONTROL STATION")
        product_name.setStyleSheet("font: 700 19px Helvetica; color: #000000; letter-spacing: 1px")
        brand_row.addWidget(product_name)
        if args.demo:
            demo_tag = QLabel("DEMO")
            demo_tag.setStyleSheet("font: 700 11px 'DejaVu Sans Mono'; color: #DB000D; padding: 5px 9px; border: 1px solid #DB000D")
            brand_row.addWidget(demo_tag)
        brand_row.addStretch()
        layout.addLayout(brand_row)
        bar = QHBoxLayout()
        self.detect = QPushButton("Detect")
        self.detect.clicked.connect(lambda: self.detect_all(automatic=True))
        bar.addWidget(self.detect)
        self.usb = QLineEdit()
        self.usb.setPlaceholderText("Zaber USB")
        self.ble = QLineEdit()
        self.ble.setPlaceholderText("PINE BLE")
        self.ports = []
        for label, default in (("PINE A", 1), ("PINE B", 2)):
            spin = QSpinBox()
            spin.setRange(1, 4)
            spin.setValue(default)
            self.ports.append(spin)
        self.settings_button = QToolButton()
        self.settings_button.setText("⚙")
        self.settings_button.setToolTip("Connection settings and detected devices")
        self.settings_button.setAccessibleName("Connection settings")
        self.settings_button.clicked.connect(self.open_settings)
        bar.addWidget(self.settings_button)
        bar.addStretch()
        layout.addLayout(bar)
        status_row = QHBoxLayout()
        self.pine_status = QLabel()
        self.zaber_status = QLabel()
        self.iphone_status = QLabel()
        for badge in (self.pine_status, self.zaber_status, self.iphone_status):
            status_row.addWidget(badge)
        status_row.addStretch(1)
        self.phone_connect = QPushButton("Connect")
        self.phone_connect.setEnabled(False)
        status_row.addWidget(self.phone_connect)
        layout.addLayout(status_row)
        split = QSplitter()
        layout.addWidget(split, 1)
        self.tabs = QTabWidget()
        split.addWidget(self.tabs)
        controls = QWidget()
        controls_layout = QVBoxLayout(controls)
        row = QHBoxLayout()
        self.dials = {}
        self.labels = {}
        self.pine_target_labels = {}
        self.axis_boxes = {}
        self.speeds = {}
        self.reference_buttons = {}
        self.reference_button_text = {}
        for key, title in (("pine1", "PINE · Motor A"), ("pine2", "PINE · Motor B"), ("zaber", "Zaber · Axis 1")):
            box = QGroupBox(title)
            column = QVBoxLayout(box)
            readout = QLabel("— mm" if key == "zaber" else "— raw units")
            readout.setAlignment(Qt.AlignmentFlag.AlignCenter)
            column.addWidget(readout)
            self.labels[key] = readout
            if key in ("pine1", "pine2"):
                target_readout = QLabel("TARGET · — raw")
                target_readout.setAlignment(Qt.AlignmentFlag.AlignCenter)
                column.addWidget(target_readout)
                self.pine_target_labels[key] = target_readout
            if key == "zaber":
                column.addWidget(QLabel("0 mm"), alignment=Qt.AlignmentFlag.AlignHCenter)
            dial = LinearAxis() if key == "zaber" else JogDial()
            dial.valueChanged.connect(lambda _, k=key: self.send_jog(k))
            if key == "zaber":
                dial.sliderReleased.connect(self.release_zaber_target)
            else:
                dial.dragStarted.connect(lambda k=key, d=dial: setattr(d, "anchor_position", self.positions[k] - d.value()))
                dial.sliderReleased.connect(lambda k=key: self.release_pine_target(k))
            column.addWidget(dial, alignment=Qt.AlignmentFlag.AlignHCenter)
            if key == "zaber":
                column.addWidget(QLabel("1500 mm"), alignment=Qt.AlignmentFlag.AlignHCenter)
                self.zaber_target_label = QLabel("TARGET · — mm")
                column.addWidget(self.zaber_target_label)
            self.dials[key] = dial
            speed = QDoubleSpinBox() if key == "zaber" else QSpinBox()
            speed.setRange(.001 if key == "zaber" else 1, 100)
            if key == "zaber":
                speed.setDecimals(3)
                speed.setSingleStep(.1)
            speed.setValue(40 if key == "zaber" else 1)
            speed.setSuffix(" mm/s" if key == "zaber" else " %")
            speed.valueChanged.connect(lambda value, k=key: self.bridge.submit("set_speed", k, value))
            self.speeds[key] = speed
            column.addWidget(speed)
            button = QPushButton("Home to switch" if key == "zaber" else "Set visual reference")
            button.setProperty("working", False)
            self.reference_buttons[key] = button
            self.reference_button_text[key] = button.text()
            button.clicked.connect(lambda _, k=key, b=button: self.run_reference(k, False, b))
            column.addWidget(button)
            if key == "zaber":
                self.zaber_home_button = button
            if key != "zaber":
                button = QPushButton("Return to reference")
                button.setProperty("working", False)
                self.reference_buttons[f"{key}_return"] = button
                self.reference_button_text[f"{key}_return"] = button.text()
                button.clicked.connect(lambda _, k=key, b=button: self.run_reference(k, True, b))
                column.addWidget(button)
            self.axis_boxes[key] = box
            box.setEnabled(False)
            row.addWidget(box)
        controls_layout.addLayout(row)
        note = QLabel("PINE · RELATIVE POSITION · VISUAL REF")
        note.setWordWrap(True)
        controls_layout.addWidget(note)
        controls_layout.addStretch()
        self.tabs.addTab(controls, "Manual control")
        camera_panel = QWidget()
        camera_form = QFormLayout(camera_panel)
        self.camera_settings = {}
        self.camera_device = QComboBox()
        self.camera_resolution = QComboBox()
        self.camera_device.currentIndexChanged.connect(lambda _: self.send_camera_setting("camera_id", self.camera_device.currentData()))
        self.camera_resolution.currentIndexChanged.connect(lambda _: self.send_camera_setting("resolution_id", self.camera_resolution.currentData()))
        camera_form.addRow("CAMERA", self.camera_device)
        camera_form.addRow("PHOTO PX", self.camera_resolution)
        self.focus_lock = QCheckBox("LOCK")
        self.focus_lock.toggled.connect(lambda value: self.send_camera_setting("focus_locked", value))
        camera_form.addRow("FOCUS", self.focus_lock)
        self.focus_position = QDoubleSpinBox()
        self.focus_position.setRange(0, 1); self.focus_position.setSingleStep(.01)
        self.focus_position.valueChanged.connect(lambda value: self.send_camera_setting("focus_position", value))
        camera_form.addRow("LENS", self.focus_position)
        self.exposure_lock = QCheckBox("LOCK")
        self.exposure_lock.toggled.connect(lambda value: self.send_camera_setting("exposure_locked", value))
        camera_form.addRow("EXPOSURE", self.exposure_lock)
        self.iso = QDoubleSpinBox(); self.iso.setDecimals(0); self.iso.setSingleStep(10)
        self.iso.valueChanged.connect(lambda value: self.send_camera_setting("iso", value))
        camera_form.addRow("ISO", self.iso)
        self.shutter = QDoubleSpinBox(); self.shutter.setDecimals(5); self.shutter.setSingleStep(.001)
        self.shutter.valueChanged.connect(lambda value: self.send_camera_setting("shutter_seconds", value))
        camera_form.addRow("SHUTTER S", self.shutter)
        self.exposure_bias = QDoubleSpinBox(); self.exposure_bias.setDecimals(1); self.exposure_bias.setSingleStep(.1)
        self.exposure_bias.valueChanged.connect(lambda value: self.send_camera_setting("exposure_bias", value))
        camera_form.addRow("EV", self.exposure_bias)
        self.white_balance_lock = QCheckBox("LOCK")
        self.white_balance_lock.toggled.connect(lambda value: self.send_camera_setting("white_balance_locked", value))
        camera_form.addRow("WHITE BALANCE", self.white_balance_lock)
        self.photogrammetry = QPushButton("LOCK FOR PASS")
        self.photogrammetry.clicked.connect(lambda: self.send_camera_setting("photogrammetry", True))
        camera_form.addRow(self.photogrammetry)
        self.unlock_camera = QPushButton("END PASS · UNLOCK")
        self.unlock_camera.clicked.connect(lambda: self.send_camera_setting("photogrammetry", False))
        camera_form.addRow(self.unlock_camera)
        self.camera_settings_status = QLabel("IPHONE · WAITING")
        camera_form.addRow(self.camera_settings_status)
        self.tabs.addTab(camera_panel, "Camera")
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
            form.addRow(label.upper(), spin)
        self.start_current = QCheckBox("Start at current Zaber position")
        self.start_current.setChecked(True)
        self.fields["start"].setEnabled(False)
        self.start_current.toggled.connect(lambda value: self.fields["start"].setEnabled(not value))
        form.addRow(self.start_current)
        self.output = QLineEdit("captures/iphone_zaber")
        form.addRow("OUTPUT", self.output)
        self.record = QPushButton("Start photo scan")
        self.record.clicked.connect(self.start_recording)
        form.addRow(self.record)
        self.progress = QProgressBar()
        self.progress.setValue(0)
        form.addRow(self.progress)
        self.scan_limit = QLabel("MAX 1000 PHOTOS · ZABER LIMITS ACTIVE")
        form.addRow(self.scan_limit)
        self.tabs.addTab(recording, "Photo recording")
        camera_box = QWidget()
        camera_layout = QVBoxLayout(camera_box)
        self.camera_status = QLabel("CAMERA · WAITING")
        camera_layout.addWidget(self.camera_status)
        self.camera = QLabel(" ")
        self.camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera.setMinimumSize(560, 420)
        self.camera.setStyleSheet("background: #080e17; border: 1px solid #304155; border-radius: 12px")
        camera_layout.addWidget(self.camera, 1)
        split.addWidget(camera_box)
        split.setSizes([760, 700])
        stop = QPushButton("STOP ALL  ·  Esc")
        stop.setStyleSheet("background: #DB000D; color: white; font-size: 20px; padding: 12px")
        stop.clicked.connect(self.stop)
        actions = QHBoxLayout()
        self.loop_demo_button = QPushButton("LOOP DEMO · 300–900 mm")
        self.loop_demo_button.clicked.connect(self.toggle_loop_demo)
        actions.addWidget(self.loop_demo_button)
        actions.addWidget(stop, 1)
        self.quit_button = QPushButton("Quit · Ctrl+Q")
        self.quit_button.clicked.connect(self.close)
        actions.addWidget(self.quit_button)
        layout.addLayout(actions)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(250)
        self.log.setMaximumHeight(84)
        self.log.hide()
        layout.addWidget(self.log)
        QShortcut(QKeySequence("Escape"), self, activated=self.stop)
        QShortcut(QKeySequence("Ctrl+Q"), self, activated=self.close)
        self.tabs.currentChanged.connect(self.release_jogs)
        QApplication.instance().installEventFilter(self)
        bridge.event.connect(self.on_event)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.heartbeat)
        self.timer.start(100)
        self.auto_detect_timer = QTimer(self)
        self.auto_detect_timer.setInterval(15000)
        self.auto_detect_timer.timeout.connect(self.auto_detect_missing)
        self.auto_detect_timer.start()
        self.auto_detection_running = False
        self.detection_running = False
        QTimer.singleShot(250, lambda: self.detect_all(automatic=True))
        self.refresh()

    def send_jog(self, key):
        dial = self.dials[key]
        if key == "zaber":
            if dial.held:
                target = dial.value() / 10.
                self.zaber_target_active = True
                self.zaber_target_label.setText(f"TARGET · {target:.1f} mm")
                self.bridge.submit("set_zaber_target", target)
        elif dial.held:
            target = round(dial.anchor_position + dial.value())
            self.pine_target_active[key] = True
            self.pine_targets_ui[key] = float(target)
            self.pine_target_labels[key].setText(f"TARGET · {target} raw")
            self.bridge.submit("set_pine_target", key, target)

    @staticmethod
    def set_button_working(button, working, text):
        button.setProperty("working", working)
        button.setText(text)
        button.style().unpolish(button)
        button.style().polish(button)

    def run_reference(self, key, return_home, button):
        if button.property("working"):
            return
        if key == "zaber":
            self.zaber_target_active = False
        token = f"{key}_return" if return_home else key
        self.set_button_working(button, True, "WORKING…")
        button.setEnabled(False)
        future = self.bridge.submit("reference", key, return_home)
        future.add_done_callback(lambda _, result=token: self.bridge.event.emit("reference_done", result))

    def heartbeat(self):
        if self.dials["zaber"].held:
            self.send_jog("zaber")
        if self.frame_time and time.monotonic() - self.frame_time > 2:
            self.camera_status.setText("CAMERA · NO FRAME")
            self.camera.clear()
            self.camera.setText("NO FRAME")

    def release_jogs(self, *_):
        for key, dial in self.dials.items():
            dial.reset()
            if key == "zaber":
                self.release_zaber_target()

    def release_pine_target(self, key):
        if self.pine_target_active[key]:
            target = self.dials[key].anchor_position + self.dials[key].value()
            self.pine_target_labels[key].setText(f"HOLD · {target:.0f} raw")

    def release_zaber_target(self):
        """Keep the last slider setpoint visible and tracked after mouse release."""
        if self.zaber_target_active:
            self.zaber_target_label.setText(f"HOLD · {self.dials['zaber'].value() / 10.:.1f} mm")
        self.bridge.submit("release_zaber_target")

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.ApplicationDeactivate:
            self.release_jogs()
        return super().eventFilter(obj, event)

    def stop(self):
        self.loop_demo_active = False
        self.loop_demo_starting = False
        self.loop_demo_button.setText("LOOP DEMO · 300–900 mm")
        self.zaber_target_active = False
        self.pine_target_active = {"pine1": False, "pine2": False}
        for key in self.pine_target_labels:
            self.pine_targets_ui[key] = None
            self.pine_target_labels[key].setText("TARGET · — raw")
        self.release_jogs()
        self.bridge.submit("stop")

    def toggle_loop_demo(self):
        if self.loop_demo_active or self.loop_demo_starting:
            self.stop()
            return
        if any(value <= 0 for value in self.pine_units_per_revolution.values()):
            self.open_settings()
            self.log.appendPlainText("Set PINE raw units per revolution in Settings before starting Loop demo.")
            self.log.show()
            return
        self.speeds["zaber"].setValue(40.)
        self.loop_demo_starting = True
        self.loop_demo_button.setText("STARTING LOOP…")
        future = self.bridge.submit("start_loop_demo", dict(self.pine_units_per_revolution))
        future.add_done_callback(lambda _: self.bridge.event.emit("loop_demo_start_done", None))

    def open_settings(self):
        if getattr(self, "settings_dialog", None) is not None:
            self.settings_dialog.show()
            self.settings_dialog.raise_()
            self.settings_dialog.activateWindow()
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("Connection settings")
        dialog.setMinimumWidth(420)
        layout = QVBoxLayout(dialog)
        form = QFormLayout()
        form.addRow("Zaber USB port", self.usb)
        form.addRow("PINE BLE address", self.ble)
        form.addRow("PINE A port", self.ports[0])
        form.addRow("PINE B port", self.ports[1])
        for key, title in (("pine1", "PINE A raw units / 360°"), ("pine2", "PINE B raw units / 360°")):
            field = QDoubleSpinBox()
            field.setRange(0, 1_000_000_000)
            field.setDecimals(1)
            field.setSingleStep(10)
            field.setValue(self.pine_units_per_revolution[key])
            field.valueChanged.connect(lambda value, k=key: self.pine_units_per_revolution.__setitem__(k, value))
            self.pine_units_fields[key] = field
            form.addRow(title, field)
        layout.addLayout(form)
        calibration_note = QLabel("Required for accurate ±45° Loop demo movement. Enter the measured PINE position-count change for one full output-axis revolution.")
        calibration_note.setWordWrap(True)
        layout.addWidget(calibration_note)
        self.device_details = QLabel(self.format_device_details())
        self.device_details.setWordWrap(True)
        layout.addWidget(self.device_details)
        self.manual_detect = QPushButton("Save & detect")
        self.manual_detect.clicked.connect(self.detect_manually)
        layout.addWidget(self.manual_detect)
        self.settings_dialog = dialog
        dialog.show()

    def format_device_details(self):
        zaber_state = ("HOMING" if self.zaber_homing else "CONNECTED · HOMED" if self.zaber_homed
                       else "CONNECTED · HOME REQUIRED" if "zaber" in self.connected else "DISCONNECTED")
        pine_state = "CONNECTED" if {"pine1", "pine2"}.issubset(self.connected) else "DISCONNECTED"
        return (f"ZABER · {zaber_state} · {self.detected_details.get('zaber') or '—'}\n"
                f"PINE II · {pine_state} · {self.detected_details.get('pine') or '—'}")

    def detect_all(self, automatic=False):
        if self.detection_running:
            return
        if automatic and self.auto_detection_running:
            return
        if automatic and {"pine1", "pine2", "zaber"}.issubset(self.connected):
            return
        self.release_jogs()
        self.detection_running = True
        self.auto_detection_running = automatic
        self.refresh()
        future = self.bridge.submit("detect", self.usb.text().strip(), self.ble.text().strip(),
                                    *(s.value() for s in self.ports), automatic)
        future.add_done_callback(lambda _: self.bridge.event.emit("detection_done", automatic))

    def detect_manually(self):
        self.detect_all(automatic=False)

    def auto_detect_missing(self):
        if not self.busy and not self.detection_running:
            self.detect_all(automatic=True)

    def start_recording(self):
        self.release_jogs()
        config = {key: field.value() for key, field in self.fields.items()}
        config.update(start_current=self.start_current.isChecked(), output=self.output.text().strip())
        self.progress.setValue(0)
        self.bridge.submit("record", config)

    def refresh(self):
        for key, box in self.axis_boxes.items():
            available = key in self.connected and not self.busy and not self.detection_running
            box.setEnabled(available)
            self.dials[key].setEnabled(available and (key != "zaber" or self.zaber_homed))
            self.speeds[key].setEnabled(available)
        for token, button in self.reference_buttons.items():
            key = token.removesuffix("_return")
            button.setEnabled(key in self.connected and not self.busy and not self.detection_running
                              and not button.property("working"))
        self.record.setEnabled(self.phone and "zaber" in self.connected and self.zaber_homed
                               and not self.busy and not self.detection_running)
        for widget in (self.camera_device, self.camera_resolution, self.focus_lock, self.focus_position,
                       self.exposure_lock, self.iso, self.shutter, self.exposure_bias,
                       self.white_balance_lock, self.photogrammetry, self.unlock_camera):
            widget.setEnabled(self.phone and not self.busy)
        self.detect.setEnabled(not self.busy and not self.detection_running)
        loop_ready = {"pine1", "pine2", "zaber"}.issubset(self.connected) and self.zaber_homed
        self.loop_demo_button.setEnabled(self.loop_demo_active or self.loop_demo_starting or
                                         (loop_ready and not self.busy and not self.detection_running))
        self.loop_demo_button.setText("STOP LOOP DEMO" if self.loop_demo_active else
            "STARTING LOOP…" if self.loop_demo_starting else "LOOP DEMO · 300–900 mm")
        self._set_status_badge(self.pine_status, "PINE II", {"pine1", "pine2"}.issubset(self.connected))
        zaber_state = ("HOMING" if self.zaber_homing else "CONNECTED" if self.zaber_homed
                       else "HOME REQUIRED" if "zaber" in self.connected else "DISCONNECTED")
        self._set_status_badge(self.zaber_status, "ZABER", zaber_state)
        self._set_status_badge(self.iphone_status, "iPHONE", self.phone)
        if hasattr(self, "device_details"):
            self.device_details.setText(self.format_device_details())
        self.phone_connect.setEnabled(self.phone or self.station_ready)
        self.phone_connect.setText("Connected" if self.phone else "Connect")
        self.phone_connect.setStyleSheet("background: #2476c7; color: white;" if self.phone else "background: #28a66a; color: white;" if self.station_ready else "background: #39414c; color: #89929e;")

    @staticmethod
    def _set_status_badge(label, name, state):
        if isinstance(state, bool):
            state = "CONNECTED" if state else "DISCONNECTED"
        color = "#16834b" if state == "CONNECTED" else "#d98b00" if state in {"HOMING", "HOME REQUIRED"} else "#DB000D"
        label.setText(f"● {name} · {state}")
        label.setStyleSheet(f"color: {color}; font-weight: 700; padding: 4px 8px")

    def send_camera_setting(self, key, value):
        if value is not None and self.phone:
            self.bridge.submit("set_camera_setting", {key: value})

    def update_camera_state(self, state):
        def refill(combo, choices, selected):
            combo.blockSignals(True)
            combo.clear()
            for item in choices:
                combo.addItem(item.get("name", item.get("label", item["id"])), item["id"])
            index = combo.findData(selected)
            if index >= 0: combo.setCurrentIndex(index)
            combo.blockSignals(False)
        refill(self.camera_device, state.get("camera_options", []), state.get("camera_id"))
        refill(self.camera_resolution, state.get("resolution_options", []), state.get("resolution_id"))
        for widget, value in ((self.focus_lock, state.get("focus_locked", False)),
                              (self.exposure_lock, state.get("exposure_locked", False)),
                              (self.white_balance_lock, state.get("white_balance_locked", False))):
            widget.blockSignals(True); widget.setChecked(bool(value)); widget.blockSignals(False)
        for widget, key, lo, hi in ((self.focus_position, "focus_position", 0, 1),
                (self.iso, "iso", "iso_min", "iso_max"),
                (self.shutter, "shutter_seconds", "shutter_min", "shutter_max"),
                (self.exposure_bias, "exposure_bias", "bias_min", "bias_max")):
            minimum = state[lo] if isinstance(lo, str) else lo
            maximum = state[hi] if isinstance(hi, str) else hi
            widget.blockSignals(True); widget.setRange(float(minimum), float(maximum))
            widget.setValue(float(state.get(key, minimum))); widget.blockSignals(False)
        self.camera_status.setText(
            f"CAMERA · live · PHOTO {state.get('resolution_id', '—')} · PC FEED UP TO 30 FPS")
        self.camera_settings_status.setText(
            f"CAMERA · {state.get('camera_state', 'unknown')} · "
            f"FOCUS {'LOCKED' if state.get('pass_locked') else 'AVAILABLE'} · "
            f"ISO {'MANUAL' if state.get('manual_exposure_supported') else 'AUTO'}")
        self.focus_position.setEnabled(self.phone and state.get("manual_focus_supported", False) and not state.get("pass_locked", False))
        self.iso.setEnabled(self.phone and state.get("manual_exposure_supported", False) and not state.get("pass_locked", False))
        self.shutter.setEnabled(self.phone and state.get("manual_exposure_supported", False) and not state.get("pass_locked", False))
        self.exposure_bias.setEnabled(self.phone and not state.get("exposure_locked", False) and not state.get("pass_locked", False))

    def on_event(self, kind, value):
        if getattr(self, "closing", False) and kind != "status":
            return
        if kind == "status":
            self.log.appendPlainText(value)
            self.log.show()
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
                self.positions[key] = position
                self.labels[key].setText(f"{position:.3f} mm" if key == "zaber" else f"{position:.0f} raw units")
                if key in self.pine_target_active:
                    target = self.pine_targets_ui[key]
                    if self.pine_target_active[key] and target is not None:
                        self.pine_target_labels[key].setText(f"HOLD · {target:.0f} raw")
                    elif not self.pine_target_active[key]:
                        self.pine_target_labels[key].setText("TARGET · — raw")
            slider = self.dials["zaber"]
            if self.zaber_target_active:
                self.zaber_target_label.setText(f"HOLD · {slider.value() / 10.:.1f} mm")
            else:
                slider.blockSignals(True)
                slider.setValue(round(value["zaber"] * 10))
                slider.blockSignals(False)
                self.zaber_target_label.setText(f"POSITION · {value['zaber']:.1f} mm")
        elif kind == "preview":
            pixmap = QPixmap()
            if pixmap.loadFromData(value, "JPEG"):
                # The iPhone sends sensor-native landscape frames while it is
                # held upright. Rotate only the live preview; still captures
                # keep their own camera orientation and full-resolution data.
                pixmap = pixmap.transformed(QTransform().rotate(90), Qt.TransformationMode.SmoothTransformation)
                self.camera.setPixmap(pixmap.scaled(self.camera.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
                self.frame_time = time.monotonic()
                self.camera_status.setText("CAMERA · live")
        elif kind == "camera_state":
            self.update_camera_state(value)
        elif kind == "progress":
            self.progress.setRange(0, value[1])
            self.progress.setValue(value[0])
        elif kind == "detection_done":
            self.detection_running = False
            if value:
                self.auto_detection_running = False
            self.refresh()
        elif kind == "connection_details":
            self.detected_details = value
            if hasattr(self, "device_details"):
                self.device_details.setText(self.format_device_details())
        elif kind == "pine_target":
            key, target = value["key"], float(value["target"])
            self.pine_targets_ui[key] = target
            self.pine_target_active[key] = True
            self.pine_target_labels[key].setText(f"HOLD · {target:.0f} raw")
        elif kind == "loop_demo":
            self.loop_demo_active = bool(value)
            self.loop_demo_starting = False
            self.refresh()
        elif kind == "loop_demo_starting":
            self.loop_demo_starting = bool(value)
            self.refresh()
        elif kind == "loop_demo_start_done":
            self.loop_demo_starting = False
            self.refresh()
        elif kind == "zaber_homing":
            self.zaber_homing = bool(value)
            if self.zaber_homing:
                self.zaber_target_active = False
            self.set_button_working(
                self.zaber_home_button, self.zaber_homing,
                "HOMING…" if self.zaber_homing else self.reference_button_text["zaber"])
            self.refresh()
        elif kind == "zaber_homed":
            self.zaber_homed = bool(value)
            self.refresh()
        elif kind == "reference_done":
            button = self.reference_buttons.get(value)
            if button is not None:
                self.set_button_working(button, False, self.reference_button_text[value])
            key = value.removesuffix("_return")
            if key in self.pine_target_active:
                self.dials[key].blockSignals(True)
                self.dials[key].setValue(0)
                self.dials[key].blockSignals(False)
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
        self.auto_detect_timer.stop()
        self.release_jogs()
        self.setEnabled(False)
        self.log.appendPlainText("STOP · CLOSING CONNECTIONS")
        self.log.show()
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
    app.setStyleSheet("""
        QWidget { background: #ffffff; color: #000000; font-family: Helvetica, Arial, sans-serif; }
        QPushButton, QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
            padding: 7px; border: 1px solid #d7d7d7; border-radius: 5px; background: #fff;
        }
        QPushButton { color: #DB000D; font-weight: bold; }
        QPushButton:hover:!disabled { background: #fff1f2; border-color: #DB000D; }
        QPushButton:disabled { color: #999999; background: #f2f2f2; }
        QPushButton:pressed, QPushButton[working="true"] {
            background: #DB000D; color: #ffffff; border-color: #a9000a;
        }
        QPushButton:disabled[working="true"] { color: #ffffff; background: #DB000D; border-color: #a9000a; }
        QToolButton:hover:!disabled, QToolButton:pressed { background: #fff1f2; color: #DB000D; }
        QDial[jogging="true"] { background: #fff1f2; border: 2px solid #DB000D; border-radius: 72px; }
        QSlider[jogging="true"] { background: #fff1f2; }
        QGroupBox { padding-top: 20px; font-weight: bold; border: 1px solid #e5e5e5; border-radius: 8px; margin-top: 8px; }
        QTabBar::tab { padding: 12px; color: #333333; }
        QTabBar::tab:selected { color: #DB000D; border-bottom: 3px solid #DB000D; }
    """)
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
