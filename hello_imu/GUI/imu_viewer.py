"""BMI088 serial monitor for the hello_imu board.

The board sends one CSV header followed by rows containing seven float values.
Serial settings live in config.ini so the UI stays focused on the sensor data.
"""

from __future__ import annotations

import configparser
import logging
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import serial
    from serial import SerialException
except ImportError:  # Keep the preview usable before dependencies are installed.
    serial = None

    class SerialException(Exception):
        pass


ROOT = Path(__file__).resolve().parent
FIELDS = (
    "gyro_x_rad_s",
    "gyro_y_rad_s",
    "gyro_z_rad_s",
    "accel_x_g",
    "accel_y_g",
    "accel_z_g",
    "temp_c",
)
COLORS = ("#ff8066", "#58c7d8", "#f2c14e")
logging.basicConfig(
    level=getattr(logging, os.getenv("BMI088_LOG_LEVEL", "INFO").upper(), logging.INFO),
    format="%(asctime)s [%(levelname)s] [%(threadName)s] %(message)s",
    datefmt="%H:%M:%S",
    handlers=[logging.StreamHandler(sys.stdout)],
)
LOGGER = logging.getLogger("bmi088.viewer")


@dataclass
class SensorSample:
    values: tuple[float, ...]
    timestamp: float


def load_config() -> configparser.ConfigParser:
    config = configparser.ConfigParser()
    config.read(ROOT / "config.ini", encoding="utf-8")
    return config


def serial_settings(config: configparser.ConfigParser) -> dict:
    section = config["serial"]
    parity = {"N": "N", "E": "E", "O": "O"}.get(section.get("parity", "N").upper(), "N")
    return {
        "port": section.get("port", "COM12"),
        "baudrate": section.getint("baudrate", 115200),
        "bytesize": section.getint("bytesize", 8),
        "parity": parity,
        "stopbits": section.getfloat("stopbits", 1),
        "timeout": section.getfloat("timeout", 0.2),
    }


class SerialReader(threading.Thread):
    def __init__(self, settings: dict, samples: queue.Queue, events: queue.Queue):
        super().__init__(daemon=True)
        self.settings = settings
        self.samples = samples
        self.events = events
        self.stop_requested = threading.Event()
        self.connection = None

    def run(self) -> None:
        if serial is None:
            LOGGER.error("pyserial 未安装，无法打开串口")
            self.events.put(("error", "未安装 pyserial，当前只能使用预览模式"))
            return
        try:
            LOGGER.info(
                "正在打开串口: port=%s baudrate=%s bytesize=%s parity=%s stopbits=%s timeout=%s",
                self.settings["port"], self.settings["baudrate"], self.settings["bytesize"],
                self.settings["parity"], self.settings["stopbits"], self.settings["timeout"],
            )
            self.connection = serial.Serial(**self.settings)
            LOGGER.info("串口打开成功: %s", self.settings["port"])
            self.events.put(("connected", self.settings["port"]))
            while not self.stop_requested.is_set():
                raw = self.connection.readline()
                if not raw:
                    continue
                LOGGER.debug("收到原始数据: %r", raw)
                self._parse_line(raw.decode("utf-8", errors="replace"))
        except (SerialException, OSError) as exc:
            LOGGER.exception("串口打开或读取失败: %s", exc)
            self.events.put(("error", f"串口打开失败: {exc}"))
        finally:
            if self.connection is not None and self.connection.is_open:
                self.connection.close()
                LOGGER.info("串口已关闭: %s", self.settings["port"])
            LOGGER.info("串口读取线程结束")
            self.events.put(("disconnected", ""))

    def _parse_line(self, line: str) -> None:
        parts = [part.strip() for part in line.strip().split(",")]
        if parts and parts[0] == "gyro_x_rad_s":
            LOGGER.info("收到协议表头: %s", line.strip())
            return
        if len(parts) != len(FIELDS):
            LOGGER.warning("丢弃数据行: 需要 %d 个字段，实际 %d 个，内容=%r", len(FIELDS), len(parts), line.strip())
            return
        try:
            values = tuple(float(part) for part in parts)
        except ValueError:
            LOGGER.warning("丢弃数据行: 存在非数字字段，内容=%r", line.strip())
            return
        if all(math.isfinite(value) for value in values):
            self.samples.put(SensorSample(values, time.monotonic()))
        else:
            LOGGER.warning("丢弃数据行: 存在 NaN 或无穷大，内容=%r", line.strip())

    def stop(self) -> None:
        self.stop_requested.set()
        if self.connection is not None:
            LOGGER.info("请求停止串口读取")
            self.connection.cancel_read()


class PlotCanvas(tk.Canvas):
    def __init__(self, master, title: str, units: str, colors: tuple[str, ...], **kwargs):
        super().__init__(master, background="#111923", highlightthickness=0, **kwargs)
        self.title = title
        self.units = units
        self.colors = colors
        self.series: list[deque[float]] = []
        self.after_idle(self.redraw)

    def set_data(self, series: list[deque[float]]) -> None:
        self.series = series
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 200)
        height = max(self.winfo_height(), 130)
        left, right, top, bottom = 42, 15, 34, 26
        plot_w, plot_h = width - left - right, height - top - bottom
        self.create_text(16, 15, anchor="w", text=self.title, fill="#ecf2f5", font=("Segoe UI", 11, "bold"))
        self.create_text(width - 15, 15, anchor="e", text=self.units, fill="#71818c", font=("Segoe UI", 9))
        if not self.series or not self.series[0]:
            self.create_text(width / 2, height / 2, text="等待数据...", fill="#71818c", font=("Segoe UI", 10))
            return
        values = [value for values in self.series for value in values]
        peak = max(abs(value) for value in values)
        peak = max(peak * 1.2, 0.01)
        for index in range(5):
            y = top + plot_h * index / 4
            level = peak - peak * index / 2
            self.create_line(left, y, width - right, y, fill="#26343f")
            self.create_text(left - 8, y, anchor="e", text=f"{level:.2g}", fill="#60717c", font=("Segoe UI", 8))
        zero_y = top + plot_h / 2
        self.create_line(left, zero_y, width - right, zero_y, fill="#3a4b56")
        for index, values in enumerate(self.series):
            if len(values) < 2:
                continue
            points = []
            for point, value in enumerate(values):
                x = left + plot_w * point / max(len(values) - 1, 1)
                y = top + plot_h * (peak - value) / (2 * peak)
                points.extend((x, y))
            self.create_line(*points, fill=self.colors[index], width=2, smooth=True)


class PoseCanvas(tk.Canvas):
    """Top-down board view using the board's measured x/y/z convention."""

    def __init__(self, master, **kwargs):
        super().__init__(master, background="#111923", highlightthickness=0, **kwargs)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.filtered_accel = [0.0, 0.0, 1.0]
        self.accel_z_sign = 1.0
        self.last_time: float | None = None
        self.redraw()

    def update_pose(self, sample: SensorSample) -> None:
        gx, gy, gz, ax, ay, az, _ = sample.values
        if self.last_time is None:
            self.filtered_accel = [ax, ay, az]
            self.accel_z_sign = 1.0 if az >= 0.0 else -1.0
        if self.last_time is None:
            dt = 0.01
        else:
            dt = min(max(sample.timestamp - self.last_time, 0.0005), 0.1)
        self.last_time = sample.timestamp

        # The board frame is x=right, y=towards the user, z=up.  The
        # accelerometer sign was established from the physical tilt tests:
        # positive ax raises the left (-x) edge and positive ay raises +y.
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if accel_norm > 0.05:
            filter_alpha = min(dt / (0.08 + dt), 1.0)
            for index, value in enumerate((ax, ay, az)):
                self.filtered_accel[index] += filter_alpha * (value - self.filtered_accel[index])

        accel_x, accel_y, accel_z = self.filtered_accel
        filtered_norm = math.sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z)
        gyro_roll = self.roll + gx * dt
        gyro_pitch = self.pitch + gy * dt
        if 0.65 < filtered_norm < 1.35:
            # These signs deliberately match the measured board convention.
            upright_z = accel_z * self.accel_z_sign
            accel_roll = math.atan2(accel_y, upright_z)
            accel_pitch = math.atan2(accel_x, upright_z)
            correction = min(dt / (0.65 + dt), 1.0)
            self.roll = gyro_roll + correction * (accel_roll - gyro_roll)
            self.pitch = gyro_pitch + correction * (accel_pitch - gyro_pitch)
        else:
            self.roll, self.pitch = gyro_roll, gyro_pitch
        self.yaw += gz * dt
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 260)
        height = max(self.winfo_height(), 260)
        self.create_text(16, 16, anchor="w", text="设备姿态", fill="#ecf2f5", font=("Segoe UI", 11, "bold"))
        self.create_text(width - 15, 16, anchor="e", text="互补滤波", fill="#71818c", font=("Segoe UI", 9))

        # x is the long edge, y is the short edge, and z is the upper face.
        vertices = [
            (-0.675, -0.5, -0.175), (0.675, -0.5, -0.175),
            (0.675, 0.5, -0.175), (-0.675, 0.5, -0.175),
            (-0.675, -0.5, 0.175), (0.675, -0.5, 0.175),
            (0.675, 0.5, 0.175), (-0.675, 0.5, 0.175),
        ]
        projected = [self._project(vertex, width, height) for vertex in vertices]

        faces = [
            ((0, 1, 2, 3), "#245164"),
            ((4, 7, 6, 5), "#34758a"),
            ((0, 4, 5, 1), "#1f4050"),
            ((3, 2, 6, 7), "#2b6072"),
            ((1, 5, 6, 2), "#3b8ca0"),
            ((0, 3, 7, 4), "#193542"),
        ]
        faces.sort(key=lambda face: sum(projected[index][2] for index in face[0]) / 4, reverse=True)
        for indexes, color in faces:
            points = [coordinate for index in indexes for coordinate in projected[index][:2]]
            self.create_polygon(*points, fill=color, outline="#8ad1d4", width=1)

        origin = self._project((0.0, 0.0, 0.0), width, height)
        x_axis = self._project((0.92, 0.0, 0.0), width, height)
        y_axis = self._project((0.0, 0.78, 0.0), width, height)
        z_axis = self._project((0.0, 0.0, 0.42), width, height)
        self.create_oval(origin[0] - 3, origin[1] - 3, origin[0] + 3, origin[1] + 3, fill="#f0f5f6", outline="")
        self.create_line(*self._axis_line(origin, x_axis), fill="#ff8066", width=2, arrow=tk.LAST)
        self.create_line(*self._axis_line(origin, y_axis), fill="#58c7d8", width=2, arrow=tk.LAST)
        self.create_line(*self._axis_line(origin, z_axis), fill="#f2c14e", width=2, arrow=tk.LAST)
        self.create_text(x_axis[0] + 10, x_axis[1], text="x", fill="#ff8066", font=("Segoe UI", 9, "bold"))
        self.create_text(y_axis[0] + 10, y_axis[1] - 4, text="y", fill="#58c7d8", font=("Segoe UI", 9, "bold"))
        self.create_text(z_axis[0], z_axis[1] - 10, text="z", fill="#f2c14e", font=("Segoe UI", 9, "bold"))
        angles = f"R {math.degrees(self.roll): .1f}°   P {math.degrees(self.pitch): .1f}°   Y {math.degrees(self.yaw): .1f}°"
        self.create_text(width / 2, height - 18, text=angles, fill="#9bb0ba", font=("Consolas", 9))

    def _rotate(self, vertex: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z = vertex
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        x, y = x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw
        cos_pitch, sin_pitch = math.cos(self.pitch), math.sin(self.pitch)
        x, z = x * cos_pitch + z * sin_pitch, -x * sin_pitch + z * cos_pitch
        cos_roll, sin_roll = math.cos(self.roll), math.sin(self.roll)
        y, z = y * cos_roll - z * sin_roll, y * sin_roll + z * cos_roll
        return x, y, z

    @staticmethod
    def _axis_line(start, end) -> tuple[float, float, float, float]:
        return (*start[:2], *end[:2])

    def _project(self, vertex: tuple[float, float, float], width: float, height: float):
        x, y, z = self._rotate(vertex)
        # Camera is above and on the +y side: x points right, +y points
        # towards the viewer (down the screen), and +z remains visible.
        elevation = math.radians(58.0)
        cos_elevation, sin_elevation = math.cos(elevation), math.sin(elevation)
        screen_x = x
        screen_y = y * cos_elevation - z * sin_elevation
        depth = y * sin_elevation + z * cos_elevation + 3.0
        scale = min(width, height) * 0.42 / depth
        return width * 0.5 + screen_x * scale, height * 0.52 + screen_y * scale, depth


class IMUViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.config_data = load_config()
        viewer = self.config_data["viewer"]
        self.history_seconds = viewer.getfloat("history_seconds", 12)
        self.refresh_ms = viewer.getint("refresh_ms", 40)
        self.samples: queue.Queue[SensorSample] = queue.Queue()
        self.events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.reader: SerialReader | None = None
        self.demo_mode = False
        self.demo_time = 0.0
        self.history = [deque(maxlen=300) for _ in FIELDS]
        self.last_sample: SensorSample | None = None
        self.value_labels: list[tk.Label] = []
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(self.refresh_ms, self.refresh)

    def _build_ui(self) -> None:
        self.title("BMI088 | IMU Telemetry")
        self.geometry("1180x760")
        self.minsize(900, 620)
        self.configure(background="#0b1118")
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TButton", background="#263743", foreground="#e8eff2", borderwidth=0, padding=(14, 8))
        style.map("TButton", background=[("active", "#35505d")])

        header = tk.Frame(self, bg="#0b1118")
        header.pack(fill="x", padx=28, pady=(24, 14))
        tk.Label(header, text="BMI088", bg="#0b1118", fg="#f0f5f6", font=("Segoe UI", 25, "bold")).pack(side="left")
        tk.Label(header, text="  MOTION TELEMETRY", bg="#0b1118", fg="#6f8792", font=("Segoe UI", 10, "bold")).pack(side="left", pady=(9, 0))
        self.status = tk.Label(header, text="● 未连接", bg="#0b1118", fg="#f2c14e", font=("Segoe UI", 10, "bold"))
        self.status.pack(side="right", pady=(9, 0))

        toolbar = tk.Frame(self, bg="#101a23")
        toolbar.pack(fill="x", padx=28, pady=(0, 18))
        settings = serial_settings(self.config_data)
        serial_text = f"{settings['port']}  ·  {settings['baudrate']}  ·  {settings['bytesize']}{settings['parity']}{settings['stopbits']:g}"
        tk.Label(toolbar, text=serial_text, bg="#101a23", fg="#9bb0ba", font=("Consolas", 10)).pack(side="left", padx=16, pady=11)
        self.demo_button = ttk.Button(toolbar, text="预览数据", command=self.toggle_demo)
        self.demo_button.pack(side="right", padx=8, pady=6)
        ttk.Button(toolbar, text="连接串口", command=self.connect).pack(side="right", padx=8, pady=6)

        cards = tk.Frame(self, bg="#0b1118")
        cards.pack(fill="x", padx=28, pady=(0, 18))
        self._make_card(cards, "GYRO X / Y / Z", "rad/s", 0)
        self._make_card(cards, "ACCEL X / Y / Z", "protocol value", 1)
        self._make_card(cards, "TEMPERATURE", "°C", 2)

        charts = tk.Frame(self, bg="#0b1118")
        charts.pack(fill="both", expand=True, padx=28, pady=(0, 24))
        self.pose_canvas = PoseCanvas(charts, width=330, height=400)
        self.pose_canvas.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        plots = tk.Frame(charts, bg="#0b1118")
        plots.grid(row=0, column=1, sticky="nsew")
        self.gyro_plot = PlotCanvas(plots, "陀螺仪", "rad/s", COLORS, width=500, height=190)
        self.accel_plot = PlotCanvas(plots, "加速度", "protocol value", COLORS, width=500, height=190)
        self.gyro_plot.pack(fill="both", expand=True, pady=(0, 6))
        self.accel_plot.pack(fill="both", expand=True, pady=(6, 0))
        charts.grid_columnconfigure(0, weight=1, minsize=300)
        charts.grid_columnconfigure(1, weight=2)
        charts.grid_rowconfigure(0, weight=1)

    def _make_card(self, parent, title: str, unit: str, group: int) -> None:
        card = tk.Frame(parent, bg="#141f29", padx=16, pady=13)
        card.pack(side="left", fill="both", expand=True, padx=(0 if group == 0 else 8, 8 if group < 2 else 0))
        tk.Label(card, text=title, bg="#141f29", fg="#78909c", font=("Segoe UI", 9, "bold")).pack(anchor="w")
        values = tk.Frame(card, bg="#141f29")
        values.pack(fill="x", pady=(8, 0))
        indexes = range(group * 3, group * 3 + (3 if group < 2 else 1))
        for index in indexes:
            cell = tk.Frame(values, bg="#141f29")
            cell.pack(side="left", fill="x", expand=True)
            label = tk.Label(cell, text="--", bg="#141f29", fg=COLORS[index % 3], font=("Consolas", 18, "bold"))
            label.pack(anchor="w")
            name = FIELDS[index].replace("gyro_", "G ").replace("accel_", "A ").replace("_rad_s", "").replace("_g", "").replace("_c", "")
            tk.Label(cell, text=f"{name}  {unit}", bg="#141f29", fg="#667b86", font=("Segoe UI", 8)).pack(anchor="w")
            self.value_labels.append(label)

    def connect(self) -> None:
        if self.reader and self.reader.is_alive():
            LOGGER.warning("连接请求被忽略: 串口读取线程仍在运行")
            return
        LOGGER.info("用户请求连接串口")
        self.demo_mode = False
        self.demo_button.configure(text="预览数据")
        self.reader = SerialReader(serial_settings(self.config_data), self.samples, self.events)
        self.reader.start()
        self.status.configure(text="● 正在连接", fg="#f2c14e")

    def toggle_demo(self) -> None:
        self.demo_mode = not self.demo_mode
        self.demo_button.configure(text="停止预览" if self.demo_mode else "预览数据")
        self.status.configure(text="● 预览模式" if self.demo_mode else "● 未连接", fg="#58c7d8" if self.demo_mode else "#f2c14e")

    def refresh(self) -> None:
        while not self.events.empty():
            event, detail = self.events.get()
            if event == "connected":
                self.status.configure(text=f"● 已连接 {detail}", fg="#6dd38d")
            elif event == "error":
                self.status.configure(text="● 串口错误", fg="#ff8066")
                messagebox.showwarning("串口", detail)
            elif event == "disconnected" and not self.demo_mode:
                self.status.configure(text="● 未连接", fg="#f2c14e")
        if self.demo_mode:
            self.demo_time += self.refresh_ms / 1000
            values = (math.sin(self.demo_time) * 0.08, math.cos(self.demo_time * 0.7) * 0.06, math.sin(self.demo_time * 1.3) * 0.04,
                      math.sin(self.demo_time * 0.5) * 0.12, math.cos(self.demo_time * 0.8) * 0.1, 1.0 + math.sin(self.demo_time) * 0.08,
                      31.6 + math.sin(self.demo_time * 0.12) * 0.15)
            self.samples.put(SensorSample(values, time.monotonic()))
        while not self.samples.empty():
            sample = self.samples.get()
            # Keep the firmware's x/y/z order.  It matches the calibrated
            # board frame used by PoseCanvas and by the plotted data.
            self.last_sample = sample
            self.pose_canvas.update_pose(sample)
            for index, value in enumerate(sample.values):
                self.history[index].append(value)
                self.value_labels[index].configure(text=f"{value: .3f}")
        self.gyro_plot.set_data(self.history[0:3])
        self.accel_plot.set_data(self.history[3:6])
        self.after(self.refresh_ms, self.refresh)

    def close(self) -> None:
        if self.reader:
            self.reader.stop()
        self.destroy()


if __name__ == "__main__":
    IMUViewer().mainloop()