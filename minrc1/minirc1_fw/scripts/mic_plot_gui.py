#!/usr/bin/env python3.11
from __future__ import annotations

import argparse
import queue
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from glob import glob

import serial

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
except ImportError as exc:
    raise SystemExit(
        "pyqtgraph and a Qt binding are required. Install pyqtgraph plus PyQt5/PyQt6/PySide6."
    ) from exc

BAUD = 460800
SYNC = b"MDAQ"
HEADER = struct.Struct("<4sBBHH")
WINDOW_SECONDS = 1.5
DEFAULT_STEP_HZ = 10.0
DEFAULT_DUTY = 100
CHANNELS = ["Mic 1", "Mic 2", "Mic 3"]
COLORS = ["#ff6b35", "#27ae60", "#2980b9"]
USB_PORT_PREFIXES = ("/dev/cu.usbmodem", "/dev/tty.usbmodem", "/dev/ttyACM", "/dev/ttyUSB")


def find_serial_port(pattern: str | None) -> str | None:
    if pattern:
        matches = sorted(glob(pattern))
        if matches:
            for match in matches:
                if match.startswith(USB_PORT_PREFIXES):
                    return match
            return matches[0]
        for prefix in USB_PORT_PREFIXES:
            if pattern.startswith(prefix):
                family = sorted(glob(f"{prefix}*"))
                if family:
                    return family[0]
        return pattern

    if list_ports is not None:
        ports = list(list_ports.comports())

        def score(port_info) -> tuple[int, str]:
            device = port_info.device or ""
            if not device.startswith(USB_PORT_PREFIXES):
                return -1, device
            text = " ".join(
                [
                    device,
                    port_info.description or "",
                    port_info.manufacturer or "",
                    port_info.hwid or "",
                ]
            ).lower()
            if "bluetooth" in text:
                return -1, device
            value = 0
            if getattr(port_info, "vid", None) == 0x303A:
                value += 100
            if "esp32" in text or "espressif" in text:
                value += 80
            if "usbmodem" in text or "cdc" in text:
                value += 20
            return value, device

        ranked = [item for item in ports if score(item)[0] >= 0]
        if ranked:
            return sorted(ranked, key=lambda item: (-score(item)[0], score(item)[1]))[0].device

    for candidate in ("/dev/cu.usbmodem*", "/dev/tty.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        matches = sorted(glob(candidate))
        if matches:
            return matches[0]

    return None


@dataclass
class Packet:
    mic: int
    sample_rate_hz: float
    samples: list[int]


class SampleBuffer:
    def __init__(self, seconds: float = WINDOW_SECONDS) -> None:
        self.seconds = seconds
        self.sample_rate_hz = 16000.0
        self.max_points = max(1000, int(self.seconds * self.sample_rate_hz))
        self.channels = [deque(maxlen=self.max_points) for _ in CHANNELS]
        self.packet_counts = [0, 0, 0]
        self.lock = threading.Lock()

    def _resize(self, sample_rate_hz: float) -> None:
        self.sample_rate_hz = sample_rate_hz
        self.max_points = max(1000, int(self.seconds * self.sample_rate_hz))
        self.channels = [deque(series, maxlen=self.max_points) for series in self.channels]

    def append_packet(self, packet: Packet) -> None:
        mic = packet.mic
        if mic < 0 or mic >= len(CHANNELS) or not packet.samples:
            return

        with self.lock:
            if abs(packet.sample_rate_hz - self.sample_rate_hz) > 0.01:
                self._resize(packet.sample_rate_hz)
            self.channels[mic].extend(packet.samples)
            self.packet_counts[mic] += 1

    def snapshot(self) -> tuple[list[list[int]], float, list[int]]:
        with self.lock:
            return (
                [list(series) for series in self.channels],
                self.sample_rate_hz,
                list(self.packet_counts),
            )

    def clear(self) -> None:
        with self.lock:
            for series in self.channels:
                series.clear()
            self.packet_counts = [0, 0, 0]


class SerialReader(threading.Thread):
    def __init__(self, port_hint: str | None, samples: SampleBuffer, messages: queue.Queue[str]) -> None:
        super().__init__(daemon=True)
        self.port_hint = port_hint
        self.samples = samples
        self.messages = messages
        self.stop_event = threading.Event()
        self.ser: serial.Serial | None = None
        self.current_port = port_hint
        self.buffer = bytearray()
        self.connected = False
        self.serial_lock = threading.Lock()

    def connect(self) -> bool:
        port = find_serial_port(self.port_hint)
        if port is None:
            self.connected = False
            self.current_port = self.port_hint
            return False
        try:
            self.ser = serial.Serial(port=port, baudrate=BAUD, timeout=0.05)
            time.sleep(0.15)
            self.ser.reset_input_buffer()
        except (serial.SerialException, OSError):
            self.ser = None
            self.current_port = port
            self.connected = False
            return False
        self.current_port = port
        self.connected = True
        self.buffer.clear()
        self.messages.put(f"connected {port}")
        return True

    def disconnect(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None
        if self.connected and not self.stop_event.is_set():
            self.messages.put("reconnecting")
        self.connected = False

    def send_line(self, line: str) -> bool:
        payload = f"{line}\n".encode("ascii")
        with self.serial_lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(payload)
                self.ser.flush()
            except (serial.SerialException, OSError) as exc:
                self.messages.put(f"serial error: {exc}")
                self.disconnect()
                return False
        return True

    def run(self) -> None:
        while not self.stop_event.is_set():
            if self.ser is None and not self.connect():
                time.sleep(0.5)
                continue
            try:
                with self.serial_lock:
                    chunk = self.ser.read(4096) if self.ser is not None else b""
            except (serial.SerialException, OSError) as exc:
                if self.stop_event.is_set():
                    break
                self.messages.put(f"serial error: {exc}")
                self.disconnect()
                time.sleep(0.2)
                continue
            if not chunk:
                continue
            self.buffer.extend(chunk)
            self._parse_buffer()

    def _parse_buffer(self) -> None:
        while True:
            start = self.buffer.find(SYNC)
            if start < 0:
                if len(self.buffer) > 3:
                    del self.buffer[:-3]
                return
            if start > 0:
                del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                return

            magic, mic, _reserved, count, sample_rate_hz = HEADER.unpack(self.buffer[:HEADER.size])
            if magic != SYNC:
                del self.buffer[0]
                continue

            payload_size = count * 2
            packet_size = HEADER.size + payload_size
            if len(self.buffer) < packet_size:
                return

            payload = bytes(self.buffer[HEADER.size:packet_size])
            values = list(struct.unpack(f"<{count}h", payload))
            self.samples.append_packet(
                Packet(
                    mic=mic,
                    sample_rate_hz=float(sample_rate_hz),
                    samples=values,
                )
            )
            del self.buffer[:packet_size]

    def stop(self) -> None:
        self.stop_event.set()
        self.disconnect()


class PlotWindow:
    def __init__(self, reader: SerialReader, samples: SampleBuffer, messages: queue.Queue[str]) -> None:
        self.reader = reader
        self.samples = samples
        self.messages = messages
        self.last_message = "connecting"
        self.paused = False
        self.paused_snapshot: tuple[list[list[int]], float, list[int]] | None = None

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("mindaq mic live")
        self.win.resize(1400, 900)

        layout = QtWidgets.QVBoxLayout(self.win)

        self.status = QtWidgets.QLabel("port=searching | waiting for telemetry")
        layout.addWidget(self.status)

        control_row = QtWidgets.QHBoxLayout()

        self.pause_button = QtWidgets.QPushButton("Pause")
        self.pause_button.clicked.connect(self.toggle_pause)
        control_row.addWidget(self.pause_button)

        self.clear_button = QtWidgets.QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_all)
        control_row.addWidget(self.clear_button)

        self.range_button = QtWidgets.QPushButton("Auto Range")
        self.range_button.clicked.connect(self.auto_range)
        control_row.addWidget(self.range_button)

        self.buzzer_box = QtWidgets.QCheckBox("Buzzer")
        self.buzzer_box.setChecked(True)
        self.buzzer_box.toggled.connect(self.on_buzzer_changed)
        control_row.addWidget(self.buzzer_box)

        control_row.addWidget(QtWidgets.QLabel("Step Hz"))
        self.step_box = QtWidgets.QDoubleSpinBox()
        self.step_box.setDecimals(1)
        self.step_box.setRange(0.0, 200.0)
        self.step_box.setSingleStep(1.0)
        self.step_box.setValue(DEFAULT_STEP_HZ)
        self.step_box.valueChanged.connect(self.on_step_changed)
        control_row.addWidget(self.step_box)

        control_row.addWidget(QtWidgets.QLabel("Duty %"))
        self.duty_box = QtWidgets.QSpinBox()
        self.duty_box.setRange(0, 100)
        self.duty_box.setSingleStep(5)
        self.duty_box.setValue(DEFAULT_DUTY)
        self.duty_box.valueChanged.connect(self.on_duty_changed)
        control_row.addWidget(self.duty_box)

        control_row.addStretch(1)
        layout.addLayout(control_row)

        self.grid = QtWidgets.QGridLayout()
        layout.addLayout(self.grid)

        pg.setConfigOptions(antialias=False, foreground="#e8eef2", background="#101418")

        self.plots: list[pg.PlotWidget] = []
        self.curves: list[pg.PlotDataItem] = []
        for index, name in enumerate(CHANNELS):
            plot = pg.PlotWidget(title=name)
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setLabel("left", "Amplitude")
            plot.setLabel("bottom", "Time (s)")
            plot.setMenuEnabled(False)
            plot.setMouseEnabled(x=True, y=True)
            plot.getAxis("left").setTextPen("#aab7c4")
            plot.getAxis("bottom").setTextPen("#aab7c4")
            curve = plot.plot(pen=pg.mkPen(COLORS[index], width=1.6))
            self.plots.append(plot)
            self.curves.append(curve)
            self.grid.addWidget(plot, index, 0)

        self.win.show()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)

    def send_controls(self) -> None:
        self.reader.send_line(f"b{1 if self.buzzer_box.isChecked() else 0}")
        self.reader.send_line(f"s{self.step_box.value():.1f}")
        self.reader.send_line(f"d{self.duty_box.value()}")

    def on_buzzer_changed(self, enabled: bool) -> None:
        self.reader.send_line(f"b{1 if enabled else 0}")
        self.last_message = "buzzer on" if enabled else "buzzer off"

    def on_step_changed(self, value: float) -> None:
        self.reader.send_line(f"s{value:.1f}")
        self.last_message = f"step {value:.1f} Hz"

    def on_duty_changed(self, value: int) -> None:
        self.reader.send_line(f"d{value}")
        mode = "full drive" if value >= 100 else f"{value}% at 20 kHz"
        self.last_message = f"duty {mode}"

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.paused_snapshot = self.samples.snapshot() if self.paused else None
        self.pause_button.setText("Resume" if self.paused else "Pause")
        self.last_message = "plot paused" if self.paused else "plot resumed"

    def clear_all(self) -> None:
        self.samples.clear()
        self.paused_snapshot = None
        self.last_message = "cleared"

    def auto_range(self) -> None:
        for plot in self.plots:
            plot.enableAutoRange()
        self.last_message = "auto range"

    def tick(self) -> None:
        while True:
            try:
                message = self.messages.get_nowait()
            except queue.Empty:
                break
            self.last_message = message
            if message.startswith("connected "):
                self.send_controls()

        ys_list, sample_rate_hz, packet_counts = (
            self.paused_snapshot if self.paused and self.paused_snapshot is not None else self.samples.snapshot()
        )
        port = self.reader.current_port or "searching"

        if not any(ys_list):
            for curve in self.curves:
                curve.setData([], [])
            self.status.setText(
                f"port={port} | step={self.step_box.value():.1f} Hz | duty={self.duty_box.value()}% | buzzer={'on' if self.buzzer_box.isChecked() else 'off'} | {self.last_message}"
            )
            return

        latest = []
        for index, curve in enumerate(self.curves):
            ys = ys_list[index]
            xs = [sample_index / sample_rate_hz for sample_index in range(len(ys))]
            curve.setData(xs, ys)
            self.plots[index].setXRange(0, self.samples.seconds, padding=0)
            latest.append(ys[-1] if ys else 0)

        latest_text = " ".join(f"{name}={value:+d}" for name, value in zip(CHANNELS, latest))
        packets_text = " ".join(f"m{index + 1}={count}" for index, count in enumerate(packet_counts))
        self.status.setText(
            f"port={port} | mic_rate={sample_rate_hz:.0f} Hz | span={self.samples.seconds:.2f}s | packets {packets_text} | step={self.step_box.value():.1f} Hz | duty={self.duty_box.value()}% | buzzer={'on' if self.buzzer_box.isChecked() else 'off'} | {latest_text} | {self.last_message}"
        )

    def exec(self) -> int:
        return self.app.exec()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot Minrc1 mic waveforms like mindaq")
    parser.add_argument("--port")
    parser.add_argument("--seconds", type=float, default=WINDOW_SECONDS)
    args = parser.parse_args()

    samples = SampleBuffer(seconds=args.seconds)
    messages: queue.Queue[str] = queue.Queue()
    reader = SerialReader(args.port, samples, messages)
    reader.start()

    window = PlotWindow(reader, samples, messages)
    try:
        return window.exec()
    finally:
        reader.stop()
        reader.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
