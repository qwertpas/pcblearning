#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import asyncio
import queue
import random
import sys
from dataclasses import dataclass

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
except ImportError as exc:
    raise SystemExit(
        "pyqtgraph and a Qt binding are required. Install pyqtgraph plus PyQt5/PyQt6/PySide6."
    ) from exc

UART_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
UART_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DEFAULT_NAME = "MinRC1-Stepper"
DEFAULT_RATE = 60
DEFAULT_DUTY = 60
DEFAULT_BEEP_HZ = 2000


@dataclass
class Event:
    kind: str
    value: str


class BleLink:
    def __init__(self, event_queue: queue.Queue[Event], loop: asyncio.AbstractEventLoop, address: str | None, name: str) -> None:
        self.events = event_queue
        self.loop = loop
        self.address = address
        self.name = name
        self.client = None
        self.buffer = ""

    def connect(self) -> None:
        self.loop.create_task(self._connect())

    def disconnect(self) -> None:
        self.loop.create_task(self._disconnect())

    def send_line(self, line: str) -> None:
        self.loop.create_task(self._send_line(line))

    async def close(self) -> None:
        await self._disconnect()

    async def _connect(self) -> None:
        from bleak import BleakClient, BleakScanner

        try:
            if self.client is not None and self.client.is_connected:
                return

            self.events.put(Event("status", "Scanning..."))
            if self.address:
                device = await BleakScanner.find_device_by_address(self.address, timeout=15.0)
            else:
                device = await BleakScanner.find_device_by_name(
                    self.name,
                    timeout=20.0,
                    service_uuids=[UART_SERVICE],
                )

            if device is None:
                self.events.put(Event("status", "Device not found"))
                return

            self.events.put(Event("status", f"Connecting to {device.address}"))
            self.client = BleakClient(device, timeout=15.0)
            await self.client.connect()
            await self.client.start_notify(UART_TX, self._handle_notify)
            self.events.put(Event("connected", device.address))
            self.events.put(Event("status", "Connected"))
        except Exception as exc:
            self.client = None
            self.events.put(Event("status", f"BLE error: {exc}"))

    async def _disconnect(self) -> None:
        try:
            if self.client is None:
                return
            if self.client.is_connected:
                await self.client.disconnect()
        except Exception as exc:
            self.events.put(Event("status", f"BLE error: {exc}"))
        finally:
            self.client = None
            self.events.put(Event("disconnected", ""))
            self.events.put(Event("status", "Disconnected"))

    async def _send_line(self, line: str) -> None:
        try:
            if self.client is None or not self.client.is_connected:
                return
            await self.client.write_gatt_char(UART_RX, (line + "\n").encode("utf-8"), response=False)
        except Exception as exc:
            self.events.put(Event("status", f"BLE error: {exc}"))

    def _handle_notify(self, _sender: object, data: bytearray) -> None:
        self.buffer += data.decode("utf-8", errors="replace")
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            line = line.strip()
            if line:
                self.events.put(Event("line", line))


class StepperWindow(QtWidgets.QWidget):
    def __init__(self, ble: BleLink, events: queue.Queue[Event], loop: asyncio.AbstractEventLoop) -> None:
        super().__init__()
        self.ble = ble
        self.events = events
        self.loop = loop
        self.connected = False
        self.keys: set[int] = set()
        self.last_drive = ""
        self.last_status = "Disconnected"

        pg.setConfigOptions(foreground="#e8eef2", background="#101418")

        self.setWindowTitle("MinRC1 Stepper")
        self.resize(640, 320)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)

        layout = QtWidgets.QVBoxLayout(self)

        title = QtWidgets.QLabel("MinRC1 Stepper")
        title.setStyleSheet("font-size: 24px; font-weight: 700;")
        layout.addWidget(title)

        hint = QtWidgets.QLabel("Click the window, then drive with W A S D. Press Space to beep.")
        hint.setStyleSheet("font-size: 15px;")
        layout.addWidget(hint)

        row = QtWidgets.QHBoxLayout()
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connect)
        row.addWidget(self.connect_button)

        self.status_label = QtWidgets.QLabel(self.last_status)
        row.addWidget(self.status_label)
        row.addStretch(1)
        layout.addLayout(row)

        self.vbus_label = QtWidgets.QLabel("GPIO8 / VBUS: ---.--- V")
        self.vbus_label.setStyleSheet("font-size: 18px;")
        layout.addWidget(self.vbus_label)

        self.drive_label = QtWidgets.QLabel("Drive: 0.0, 0.0")
        self.drive_label.setStyleSheet("font-size: 18px;")
        layout.addWidget(self.drive_label)

        duty_row = QtWidgets.QHBoxLayout()
        duty_row.addWidget(QtWidgets.QLabel("Duty cycle"))

        self.duty_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.duty_slider.setRange(0, 100)
        self.duty_slider.setValue(DEFAULT_DUTY)
        self.duty_slider.valueChanged.connect(self.on_duty_changed)
        duty_row.addWidget(self.duty_slider, 1)

        self.duty_value = QtWidgets.QLabel(f"{DEFAULT_DUTY}%")
        duty_row.addWidget(self.duty_value)
        layout.addLayout(duty_row)

        beep_row = QtWidgets.QHBoxLayout()
        beep_row.addWidget(QtWidgets.QLabel("Beep frequency"))

        self.beep_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.beep_slider.setRange(200, 12000)
        self.beep_slider.setValue(DEFAULT_BEEP_HZ)
        self.beep_slider.valueChanged.connect(self.on_beep_changed)
        beep_row.addWidget(self.beep_slider, 1)

        self.beep_value = QtWidgets.QLabel(f"{DEFAULT_BEEP_HZ} Hz")
        beep_row.addWidget(self.beep_value)
        layout.addLayout(beep_row)

        self.random_beep_box = QtWidgets.QCheckBox("Random Space beep (200-5000 Hz)")
        self.random_beep_box.setChecked(True)
        layout.addWidget(self.random_beep_box)

        rate_row = QtWidgets.QHBoxLayout()
        rate_row.addWidget(QtWidgets.QLabel("Max step rate"))

        self.rate_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.rate_slider.setRange(10, 200)
        self.rate_slider.setValue(DEFAULT_RATE)
        self.rate_slider.valueChanged.connect(self.on_rate_changed)
        rate_row.addWidget(self.rate_slider, 1)

        self.rate_value = QtWidgets.QLabel(f"{DEFAULT_RATE} steps/s")
        rate_row.addWidget(self.rate_value)
        layout.addLayout(rate_row)

        self.event_timer = QtCore.QTimer(self)
        self.event_timer.timeout.connect(self.drain_events)
        self.event_timer.start(30)

        self.async_timer = QtCore.QTimer(self)
        self.async_timer.timeout.connect(self.pump_asyncio)
        self.async_timer.start(10)

        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self.poll_vbus)
        self.poll_timer.start(1000)

        QtCore.QTimer.singleShot(0, self.refocus)
        QtCore.QTimer.singleShot(0, self.start_connect)

    def start_connect(self) -> None:
        if self.connected:
            return
        self.status_label.setText("Scanning...")
        self.ble.connect()
        self.refocus()

    def refocus(self) -> None:
        self.activateWindow()
        self.raise_()
        self.setFocus(QtCore.Qt.FocusReason.ActiveWindowFocusReason)

    def toggle_connect(self) -> None:
        if self.connected:
            self.status_label.setText("Disconnecting...")
            self.ble.send_line("v0,0")
            self.ble.disconnect()
        else:
            self.status_label.setText("Scanning...")
            self.ble.connect()
        QtCore.QTimer.singleShot(0, self.refocus)

    def on_rate_changed(self, value: int) -> None:
        self.rate_value.setText(f"{value} steps/s")
        self.send_drive()
        QtCore.QTimer.singleShot(0, self.refocus)

    def on_duty_changed(self, value: int) -> None:
        self.duty_value.setText(f"{value}%")
        if self.connected:
            self.ble.send_line(f"d{value}")
        QtCore.QTimer.singleShot(0, self.refocus)

    def on_beep_changed(self, value: int) -> None:
        self.beep_value.setText(f"{value} Hz")
        if self.connected:
            self.ble.send_line(f"z{value}")
        QtCore.QTimer.singleShot(0, self.refocus)

    def pump_asyncio(self) -> None:
        self.loop.call_later(0.01, self.loop.stop)
        self.loop.run_forever()

    def poll_vbus(self) -> None:
        if self.connected:
            self.ble.send_line("b")

    def drain_events(self) -> None:
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            self.handle_event(event)

    def handle_event(self, event: Event) -> None:
        if event.kind == "status":
            self.last_status = event.value
            self.status_label.setText(event.value)
            return
        if event.kind == "connected":
            self.connected = True
            self.connect_button.setText("Disconnect")
            self.last_drive = ""
            self.ble.send_line(f"d{self.duty_slider.value()}")
            self.ble.send_line(f"z{self.beep_slider.value()}")
            self.send_drive()
            self.ble.send_line("q")
            self.refocus()
            return
        if event.kind == "disconnected":
            self.connected = False
            self.connect_button.setText("Connect")
            self.last_drive = ""
            self.vbus_label.setText("GPIO8 / VBUS: ---.--- V")
            self.drive_label.setText("Drive: 0.0, 0.0")
            return
        if event.kind == "line":
            self.handle_line(event.value)

    def handle_line(self, line: str) -> None:
        for token in line.split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            if key in {"vbus_v", "battery_v"}:
                self.vbus_label.setText(f"GPIO8 / VBUS: {float(value):.3f} V")
            elif key == "duty_pct":
                duty = int(round(float(value)))
                if duty != self.duty_slider.value():
                    blocker = QtCore.QSignalBlocker(self.duty_slider)
                    self.duty_slider.setValue(duty)
                    del blocker
                self.duty_value.setText(f"{duty}%")
            elif key == "beep_hz":
                beep = int(round(float(value)))
                if beep != self.beep_slider.value():
                    blocker = QtCore.QSignalBlocker(self.beep_slider)
                    self.beep_slider.setValue(beep)
                    del blocker
                self.beep_value.setText(f"{beep} Hz")
            elif key == "left_hz":
                parts = self.drive_label.text().split(": ", 1)
                right_text = parts[1].split(", ", 1)[1] if len(parts) == 2 and ", " in parts[1] else "0.0"
                self.drive_label.setText(f"Drive: {float(value):.1f}, {right_text}")
            elif key == "right_hz":
                parts = self.drive_label.text().split(": ", 1)
                left_text = parts[1].split(", ", 1)[0] if len(parts) == 2 and ", " in parts[1] else "0.0"
                self.drive_label.setText(f"Drive: {left_text}, {float(value):.1f}")

    def current_drive(self) -> tuple[float, float]:
        forward = int(QtCore.Qt.Key.Key_W in self.keys) - int(QtCore.Qt.Key.Key_S in self.keys)
        turn = int(QtCore.Qt.Key.Key_D in self.keys) - int(QtCore.Qt.Key.Key_A in self.keys)
        left = max(-1, min(1, forward + turn))
        right = max(-1, min(1, forward - turn))
        rate = float(self.rate_slider.value())
        return left * rate, right * rate

    def send_drive(self) -> None:
        left, right = self.current_drive()
        line = f"v{left:.1f},{right:.1f}"
        self.drive_label.setText(f"Drive: {left:.1f}, {right:.1f}")
        if not self.connected or line == self.last_drive:
            return
        self.last_drive = line
        self.ble.send_line(line)

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
        if event.isAutoRepeat():
            return
        if event.key() == QtCore.Qt.Key.Key_Space:
            if self.connected:
                if self.random_beep_box.isChecked():
                    beep = random.randint(200, 5000)
                    if beep != self.beep_slider.value():
                        blocker = QtCore.QSignalBlocker(self.beep_slider)
                        self.beep_slider.setValue(beep)
                        del blocker
                    self.beep_value.setText(f"{beep} Hz")
                    self.ble.send_line(f"z{beep}")
                self.ble.send_line("x")
            event.accept()
            return
        if event.key() in {QtCore.Qt.Key.Key_W, QtCore.Qt.Key.Key_A, QtCore.Qt.Key.Key_S, QtCore.Qt.Key.Key_D}:
            self.keys.add(event.key())
            self.send_drive()
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.isAutoRepeat():
            return
        if event.key() in {QtCore.Qt.Key.Key_W, QtCore.Qt.Key.Key_A, QtCore.Qt.Key.Key_S, QtCore.Qt.Key.Key_D}:
            self.keys.discard(event.key())
            self.send_drive()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        super().mousePressEvent(event)
        QtCore.QTimer.singleShot(0, self.refocus)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.ble.send_line("v0,0")
        self.loop.run_until_complete(self.ble.close())
        self.loop.close()
        super().closeEvent(event)


def main() -> int:
    parser = argparse.ArgumentParser(description="GUI control for MinRC1 stepper-only firmware")
    parser.add_argument("--address", help="BLE address")
    parser.add_argument("--name", default=DEFAULT_NAME, help=f"BLE device name (default: {DEFAULT_NAME})")
    args = parser.parse_args()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    events: queue.Queue[Event] = queue.Queue()
    ble = BleLink(events, loop, args.address, args.name)
    window = StepperWindow(ble, events, loop)
    window.show()
    window.refocus()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
