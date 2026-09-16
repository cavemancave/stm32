"""BMI088 monitor for the hello_imu board (serial or WiFi/UDP).

The board sends one CSV header followed by rows containing seven float values.

Two transports carry the exact same byte stream:
  * 串口   —— USB/TTL 直连开发板
  * UDP    —— 开发板 -> ESP32(UART->UDP 透传) -> 本机，即 ESP32 把串口数据原样
              打包成 UDP 包发到 PC，本机只监听端口（纯服务端，不主动找设备）。
              ESP32 是按"串口空闲几毫秒"切包的，所以一个 UDP 包里可能是半行、
              也可能有好几行，收到后要自己按 '\n' 重新拼成整行。

Connection settings live in config.ini so the UI stays focused on the sensor data.
"""

from __future__ import annotations

import configparser
import logging
import math
import os
import queue
import socket
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
        "baudrate": section.getint("baudrate", 921600),
        "bytesize": section.getint("bytesize", 8),
        "parity": parity,
        "stopbits": section.getfloat("stopbits", 1),
        "timeout": section.getfloat("timeout", 0.2),
    }


def udp_settings(config: configparser.ConfigParser) -> dict:
    """ESP32 (UART -> UDP 透传) 侧的监听参数，默认值与 udp_server.py 一致。"""
    section = config["udp"] if config.has_section("udp") else config["DEFAULT"]
    return {
        "bind": section.get("bind", "0.0.0.0"),
        "port": section.getint("port", 3333),
        "recv_size": section.getint("recv_size", 4096),
        "device_timeout": section.getfloat("device_timeout", 30.0),
    }


class LineFramer:
    """把收到的字节流重新拼成整行。

    ESP32 是"串口空闲 UDP_UART_IDLE_MS(默认 3ms) 就把这一批打包发走"，所以一个包里
    可能是半行、也可能有好几行，必须按 '\\n' 重新拼。丢包造成的错位不用额外处理：
    拼出来的那行解析不过，会被当成坏行丢掉。
    """

    def __init__(self, limit: int = 8192):
        self.buffer = bytearray()
        self.limit = limit
        self.dropped = 0

    def feed(self, chunk: bytes) -> list[str]:
        self.buffer.extend(chunk)
        lines: list[str] = []
        while True:
            index = self.buffer.find(b"\n")
            if index < 0:
                break
            lines.append(bytes(self.buffer[:index]).decode("utf-8", errors="replace"))
            del self.buffer[:index + 1]
        if len(self.buffer) > self.limit:
            # 一直等不到换行，说明流的格式不对（波特率错、对面在发二进制），
            # 丢掉免得缓冲无限增长
            self.dropped += 1
            LOGGER.warning(
                "接收缓冲累计 %d 字节都没有换行，已丢弃（第 %d 次）", len(self.buffer), self.dropped
            )
            self.buffer.clear()
        return lines


class LineParser:
    """把一行 CSV 文本变成 SensorSample 入队；串口和 UDP 两条链路共用。"""

    def __init__(self, samples: queue.Queue, source: str):
        self.samples = samples
        self.source = source
        self.lines = 0
        self.bad_lines = 0
        self.header_seen = False

    def parse_line(self, line: str, timestamp: float | None = None) -> None:
        text = line.strip()
        if not text:
            return
        parts = [part.strip() for part in text.split(",")]
        if parts and parts[0] == "gyro_x_rad_s":
            if not self.header_seen:
                self.header_seen = True
                LOGGER.info("%s 收到协议表头: %s", self.source, text)
            return
        if len(parts) != len(FIELDS):
            self._reject(f"需要 {len(FIELDS)} 个字段，实际 {len(parts)} 个", text)
            return
        try:
            values = tuple(float(part) for part in parts)
        except ValueError:
            self._reject("存在非数字字段", text)
            return
        if not all(math.isfinite(value) for value in values):
            self._reject("存在 NaN 或无穷大", text)
            return
        self.lines += 1
        self.samples.put(SensorSample(values, time.monotonic() if timestamp is None else timestamp))

    def _reject(self, reason: str, text: str) -> None:
        self.bad_lines += 1
        if self.bad_lines <= 5:  # 格式不对时坏行会刷屏，只报前几条
            LOGGER.warning("%s 丢弃数据行(%s): %r", self.source, reason, text)


class Reader(threading.Thread):
    """串口 / UDP 读取线程的公共部分：共用行解析、上报事件、可停止。"""

    def __init__(self, samples: queue.Queue, events: queue.Queue, source: str, thread_name: str):
        super().__init__(name=thread_name, daemon=True)
        self.samples = samples
        self.events = events
        self.source = source
        self.stop_requested = threading.Event()
        self.parser = LineParser(samples, source)

    def _emit(self, kind: str, detail: str = "") -> None:
        """事件里带上线程对象：界面用对象身份忽略被替换掉的旧线程的迟到事件。"""
        self.events.put((self, kind, detail))


class SerialReader(Reader):
    def __init__(self, settings: dict, samples: queue.Queue, events: queue.Queue):
        super().__init__(samples, events, "串口", f"serial-{settings['port']}")
        self.settings = settings
        self.connection = None

    def describe(self) -> str:
        return f"串口 {self.settings['port']} @ {self.settings['baudrate']}"

    def run(self) -> None:
        if serial is None:
            LOGGER.error("pyserial 未安装，无法打开串口")
            self._emit("error", "未安装 pyserial，当前只能使用预览模式")
            self._emit("disconnected")
            return
        try:
            LOGGER.info(
                "正在打开串口: port=%s baudrate=%s bytesize=%s parity=%s stopbits=%s timeout=%s",
                self.settings["port"], self.settings["baudrate"], self.settings["bytesize"],
                self.settings["parity"], self.settings["stopbits"], self.settings["timeout"],
            )
            self.connection = serial.Serial(**self.settings)
            LOGGER.info("串口打开成功: %s", self.settings["port"])
            self._emit("connected", self.settings["port"])
            while not self.stop_requested.is_set():
                raw = self.connection.readline()
                if not raw:
                    continue
                LOGGER.debug("收到原始数据: %r", raw)
                self.parser.parse_line(raw.decode("utf-8", errors="replace"))
        except (SerialException, OSError) as exc:
            LOGGER.exception("串口打开或读取失败: %s", exc)
            self._emit("error", f"串口打开失败: {exc}")
        finally:
            if self.connection is not None and self.connection.is_open:
                self.connection.close()
                LOGGER.info("串口已关闭: %s", self.settings["port"])
            LOGGER.info("串口读取线程结束")
            self._emit("disconnected")

    def stop(self) -> None:
        self.stop_requested.set()
        if self.connection is not None:
            LOGGER.info("请求停止串口读取")
            self.connection.cancel_read()


class UdpReader(Reader):
    """监听 UDP 端口，接收 ESP32 转发上来的串口数据。

    角色分工和 tools/udp_server.py 保持一致：本机是纯服务端，只 bind 端口等人来，
    ESP32 是客户端、自己解析 server.local 主动发包。所以对端地址不做配置，而是从
    收到的第一包数据的来源地址里学；设备重启换了源端口也能自动跟上。
    """

    def __init__(self, settings: dict, samples: queue.Queue, events: queue.Queue):
        super().__init__(samples, events, "UDP", f"udp-{settings['bind']}:{settings['port']}")
        self.settings = settings
        self.socket: socket.socket | None = None
        self.framer = LineFramer()
        self.peer: tuple[str, int] | None = None
        self.rx_packets = 0
        self.rx_bytes = 0
        self.last_rx_time = 0.0
        self.last_packet_time: float | None = None
        self.bound_endpoint = ""

    def describe(self) -> str:
        endpoint = self.bound_endpoint or f"{self.settings['bind']}:{self.settings['port']}"
        return f"UDP 监听 {endpoint}"

    def run(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.settings["bind"], self.settings["port"]))
            sock.settimeout(0.2)  # 只为让收包循环能及时看到 stop_requested
        except OSError as exc:
            LOGGER.exception("UDP 端口绑定失败: %s", exc)
            self._emit("error", f"UDP 端口绑定失败: {exc}")
            self._emit("disconnected")
            return

        self.socket = sock
        self.bound_endpoint = f"{sock.getsockname()[0]}:{sock.getsockname()[1]}"
        LOGGER.info(
            "UDP 服务端已启动: 监听 %s，等待 ESP32 发来第一包数据（只监听，不主动找设备）",
            self.bound_endpoint,
        )
        self._emit("listening", self.bound_endpoint)
        try:
            while not self.stop_requested.is_set():
                try:
                    data, addr = sock.recvfrom(self.settings["recv_size"])
                except socket.timeout:
                    continue
                except OSError as exc:
                    if self.stop_requested.is_set():
                        break
                    LOGGER.exception("UDP 收包失败: %s", exc)
                    self._emit("error", f"UDP 收包失败: {exc}")
                    break
                self._on_packet(data, addr)
        finally:
            sock.close()
            self.socket = None
            LOGGER.info(
                "UDP 监听结束: 收 %d 包 / %d 字节，解析出 %d 行（坏行 %d）",
                self.rx_packets, self.rx_bytes, self.parser.lines, self.parser.bad_lines,
            )
            self._emit("disconnected")

    def _on_packet(self, data: bytes, addr) -> None:
        self.rx_packets += 1
        self.rx_bytes += len(data)
        self.last_rx_time = time.monotonic()
        if addr != self.peer:
            first = self.peer is None
            self.peer = (addr[0], addr[1])
            LOGGER.info("%s设备地址: %s:%d", "发现" if first else "切换到", addr[0], addr[1])
            self._emit("udp_peer", f"{addr[0]}:{addr[1]}")
        LOGGER.debug("收到 UDP 包 %d 字节: %r", len(data), data)

        lines = self.framer.feed(data)
        if not lines:
            return
        now = self.last_rx_time
        # 100Hz 的数据(10ms 一行)配 3ms 打包窗，一包里通常正好一行；万一一个包里有
        # 多行，就按"上一包到这一包的间隔"在包内均匀铺开时间戳，免得互补滤波把 N 行
        # 当成同一时刻、白白丢掉 N-1 份陀螺仪积分时间。
        span = 0.0 if self.last_packet_time is None else min(max(now - self.last_packet_time, 0.0), 0.05)
        self.last_packet_time = now
        step = span / len(lines)
        for index, line in enumerate(lines):
            self.parser.parse_line(line, timestamp=now - step * (len(lines) - 1 - index))

    def send(self, payload: bytes) -> bool:
        """回发给 ESP32，由它原样写进串口（UART->UDP 的反向通道）。"""
        if self.socket is None or self.peer is None:
            LOGGER.warning("还没收到过 UDP 包，不知道 ESP32 的地址，发送被忽略")
            return False
        try:
            self.socket.sendto(payload, self.peer)
        except OSError as exc:
            LOGGER.exception("UDP 发送失败: %s", exc)
            return False
        LOGGER.debug("已回发 %d 字节到 %s:%d", len(payload), self.peer[0], self.peer[1])
        return True

    def stop(self) -> None:
        LOGGER.info("请求停止 UDP 监听")
        self.stop_requested.set()


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
        self.events: queue.Queue[tuple] = queue.Queue()
        self.reader: Reader | None = None
        self.udp_config = udp_settings(self.config_data)
        self.last_error: str | None = None
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
        tk.Label(toolbar, text="串口", bg="#101a23", fg="#5d707b", font=("Segoe UI", 9, "bold")).pack(side="left", padx=(16, 6), pady=11)
        tk.Label(toolbar, text=serial_text, bg="#101a23", fg="#9bb0ba", font=("Consolas", 10)).pack(side="left", padx=(0, 14), pady=11)
        tk.Label(toolbar, text="UDP", bg="#101a23", fg="#5d707b", font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 6), pady=11)
        udp_text = f"{self.udp_config['bind']}:{self.udp_config['port']}"
        tk.Label(toolbar, text=udp_text, bg="#101a23", fg="#9bb0ba", font=("Consolas", 10)).pack(side="left", padx=(0, 16), pady=11)
        self.demo_button = ttk.Button(toolbar, text="预览数据", command=self.toggle_demo)
        self.demo_button.pack(side="right", padx=8, pady=6)
        ttk.Button(toolbar, text="断开", command=self.disconnect).pack(side="right", padx=8, pady=6)
        ttk.Button(toolbar, text="连接 UDP", command=self.connect_udp).pack(side="right", padx=8, pady=6)
        ttk.Button(toolbar, text="连接串口", command=self.connect_serial).pack(side="right", padx=8, pady=6)

        self.footer = tk.Label(self, text="", bg="#0b1118", fg="#667b86", font=("Consolas", 9), anchor="w")
        self.footer.pack(side="bottom", fill="x", padx=28, pady=(0, 10))

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

    def _start_reader(self, reader: Reader, label: str) -> None:
        """同一时刻只跑一路数据源：换源前先把旧的停掉。"""
        self._stop_reader()
        self.demo_mode = False
        self.demo_button.configure(text="预览数据")
        self.last_error = None
        self.reader = reader
        reader.start()
        LOGGER.info("开始接收: %s (线程 %s)", label, reader.name)
        self.status.configure(text=f"● 正在连接 {label}", fg="#f2c14e")

    def _stop_reader(self) -> None:
        if self.reader is not None:
            LOGGER.info("停止当前数据源: %s", self.reader.source)
            self.reader.stop()
            # 置空后，旧线程迟到的 connected/disconnected 事件会因为对象身份不匹配被忽略
            self.reader = None

    def connect_serial(self) -> None:
        settings = serial_settings(self.config_data)
        self._start_reader(
            SerialReader(settings, self.samples, self.events), f"串口 {settings['port']}"
        )

    def connect_udp(self) -> None:
        socket_settings = udp_settings(self.config_data)
        self.udp_config = socket_settings
        endpoint = f"{socket_settings['bind']}:{socket_settings['port']}"
        self._start_reader(
            UdpReader(socket_settings, self.samples, self.events), f"UDP {endpoint}"
        )

    def disconnect(self) -> None:
        self._stop_reader()
        self.demo_mode = False
        self.demo_button.configure(text="预览数据")
        self.status.configure(text="● 未连接", fg="#f2c14e")

    def toggle_demo(self) -> None:
        self.demo_mode = not self.demo_mode
        self.demo_button.configure(text="停止预览" if self.demo_mode else "预览数据")
        self.status.configure(text="● 预览模式" if self.demo_mode else "● 未连接", fg="#58c7d8" if self.demo_mode else "#f2c14e")

    def _handle_events(self) -> None:
        while not self.events.empty():
            source, event, detail = self.events.get()
            if source is not self.reader:
                continue  # 被替换掉的旧线程迟到的消息
            if event == "connected":
                self.status.configure(text=f"● 已连接 {detail}", fg="#6dd38d")
            elif event == "listening":
                self.status.configure(text=f"● 监听 {detail}，等待 ESP32 发来数据", fg="#f2c14e")
            elif event == "udp_peer":
                self.status.configure(text=f"● UDP 已连接 {detail}", fg="#6dd38d")
            elif event == "error":
                self.last_error = detail
                self.status.configure(text=f"● {source.source}错误", fg="#ff8066")
                messagebox.showwarning(source.source, detail)
            elif event == "disconnected" and not self.demo_mode and self.last_error is None:
                # 出错时线程也会以 disconnected 收尾，别把错误提示盖掉
                self.status.configure(text="● 未连接", fg="#f2c14e")

    def _refresh_status(self) -> None:
        """UDP 是单向的，没有"断开"事件，只能按有没有包来判活。"""
        reader = self.reader
        if self.demo_mode or self.last_error is not None or not isinstance(reader, UdpReader):
            return
        if reader.peer is None:
            return
        silent = time.monotonic() - reader.last_rx_time
        if silent > self.udp_config["device_timeout"]:
            self.status.configure(text=f"● 设备 {silent:.0f}s 没有数据", fg="#ff8066")
        else:
            self.status.configure(text=f"● UDP 已连接 {reader.peer[0]}:{reader.peer[1]}", fg="#6dd38d")

    def _refresh_footer(self) -> None:
        reader = self.reader
        if reader is None:
            self.footer.configure(text="未连接  ·  「连接 UDP」监听 ESP32 转发上来的数据，「连接串口」走 USB/TTL 直连")
            return
        parts = [reader.describe()]
        if isinstance(reader, UdpReader) and reader.peer is not None:
            parts.append(f"设备 {reader.peer[0]}:{reader.peer[1]}")
            parts.append(f"收 {reader.rx_packets} 包 / {reader.rx_bytes} B")
        parts.append(f"有效 {reader.parser.lines} 行")
        if reader.parser.bad_lines:
            parts.append(f"坏行 {reader.parser.bad_lines}")
        self.footer.configure(text="   ·   ".join(parts))

    def refresh(self) -> None:
        self._handle_events()
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
        self._refresh_status()
        self._refresh_footer()
        self.after(self.refresh_ms, self.refresh)

    def close(self) -> None:
        self._stop_reader()
        self.destroy()


if __name__ == "__main__":
    IMUViewer().mainloop()