"""uart_rx_dma 协议层：组帧 / 解帧 / 串口收发线程 / 消息解码。

帧格式（小端）:
    SOF 0xA5 | TYPE u8 | LEN u8 | PAYLOAD[LEN] | CRC32(4, LE) | EOF 0x5A
    CRC32 覆盖 TYPE + LEN + PAYLOAD，与 zlib.crc32 一致。
    LEN = 0..8，整帧 8..16 字节。
"""
from __future__ import annotations

import queue
import struct
import threading
import time
import zlib
from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

SOF = 0xA5
EOF = 0x5A
MAX_PAYLOAD = 8
MAX_FRAME = 16

# 请求
MSG_CALC_SUM_REQ = 0x01
MSG_CALC_SQSUM_REQ = 0x02
MSG_CALC_WEIGHTED_REQ = 0x03
MSG_PING_REQ = 0x10
MSG_STATS_REQ = 0x11

# 响应
MSG_CALC_SUM_RSP = 0x81
MSG_CALC_SQSUM_RSP = 0x82
MSG_CALC_WEIGHTED_RSP = 0x83
MSG_PONG_RSP = 0x90
MSG_STATS_RSP = 0x91

# 异步 / 错误
MSG_READY = 0x7E
MSG_HEARTBEAT = 0x7D
MSG_NACK = 0xEE

ASYNC_TYPES = {MSG_READY, MSG_HEARTBEAT}

ERR_NAMES = {
    0: "NONE",
    1: "BAD_LEN",
    2: "CRC",
    3: "TAIL",
    4: "TIMEOUT",
    5: "UNKNOWN_TYPE",
    6: "PAYLOAD_LEN",
    7: "RING_OVERFLOW",
    8: "UART",
    9: "STATS_PAGE",
}

REQ_NAMES = {
    MSG_CALC_SUM_REQ: "CALC_SUM_REQ",
    MSG_CALC_SQSUM_REQ: "CALC_SQSUM_REQ",
    MSG_CALC_WEIGHTED_REQ: "CALC_WEIGHTED_REQ",
    MSG_PING_REQ: "PING_REQ",
    MSG_STATS_REQ: "STATS_REQ",
}
RSP_NAMES = {
    MSG_CALC_SUM_RSP: "CALC_SUM_RSP",
    MSG_CALC_SQSUM_RSP: "CALC_SQSUM_RSP",
    MSG_CALC_WEIGHTED_RSP: "CALC_WEIGHTED_RSP",
    MSG_PONG_RSP: "PONG_RSP",
    MSG_STATS_RSP: "STATS_RSP",
    MSG_READY: "READY",
    MSG_HEARTBEAT: "HEARTBEAT",
    MSG_NACK: "NACK",
}


def type_name(t: int) -> str:
    return RSP_NAMES.get(t) or REQ_NAMES.get(t) or f"0x{t:02X}"


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def encode(ftype: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD}")
    body = bytes([ftype, len(payload)]) + payload
    crc = crc32(body)
    return bytes([SOF]) + body + crc.to_bytes(4, "little") + bytes([EOF])


def calc_payload(a: int, b: int, c: int) -> bytes:
    return struct.pack("<hhh", a, b, c)


@dataclass
class Frame:
    type: int
    payload: bytes
    raw: bytes
    ts: float = field(default_factory=time.monotonic)

    @property
    def name(self) -> str:
        return type_name(self.type)

    def __repr__(self) -> str:
        return f"Frame({self.name}, len={len(self.payload)}, raw={self.raw.hex(' ')})"


@dataclass
class ParseIssue:
    reason: str
    raw: bytes
    ts: float = field(default_factory=time.monotonic)


class FrameParser:
    """增量解析；遇到坏帧只丢 1 字节重新找 SOF，和固件状态机行为一致。"""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes) -> list[Frame | ParseIssue]:
        self._buf.extend(data)
        out: list[Frame | ParseIssue] = []
        while True:
            start = self._buf.find(bytes([SOF]))
            if start < 0:
                self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            if len(self._buf) < 3:
                break
            length = self._buf[2]
            if length > MAX_PAYLOAD:
                out.append(ParseIssue("bad_len", bytes(self._buf[:3])))
                self.bad_frames += 1
                del self._buf[:1]
                continue
            total = 8 + length
            if len(self._buf) < total:
                break
            raw = bytes(self._buf[:total])
            crc_recv = int.from_bytes(raw[3 + length:7 + length], "little")
            crc_calc = crc32(raw[1:3 + length])
            if raw[-1] != EOF or crc_recv != crc_calc:
                out.append(ParseIssue("crc_or_tail", raw))
                self.bad_frames += 1
                del self._buf[:1]
                continue
            out.append(Frame(raw[1], raw[3:3 + length], raw))
            del self._buf[:total]
        return out


class SerialLink:
    """带读线程的串口封装：收到的帧进 queue，测试线程按需 wait。"""

    def __init__(self, port: str, baud: int, read_timeout: float = 0.01) -> None:
        if serial is None:
            raise RuntimeError("pyserial 未安装：pip install pyserial")
        self.port = port
        self.baud = baud
        self._ser = serial.Serial(port, baud, timeout=read_timeout, write_timeout=5)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._parser = FrameParser()
        self._queue: queue.Queue[Frame] = queue.Queue()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, name="uart-reader", daemon=True)
        self._thread.start()
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.frames: list[Frame] = []
        self.issues: list[ParseIssue] = []

    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                # 不要 read(4096)：pyserial 会等满 4096 字节或超时才返回，
                # 小帧会被硬生生拖到超时（几十 ms）。先等 1 字节，再排空。
                data = self._ser.read(1)
                if not data:
                    continue
                extra = self._ser.in_waiting
                if extra:
                    data += self._ser.read(extra)
            except Exception:
                break
            with self._lock:
                self.rx_bytes += len(data)
            for item in self._parser.feed(data):
                if isinstance(item, Frame):
                    with self._lock:
                        self.frames.append(item)
                    self._queue.put(item)
                else:
                    with self._lock:
                        self.issues.append(item)

    def send_raw(self, data: bytes) -> None:
        self._ser.write(data)
        self._ser.flush()
        with self._lock:
            self.tx_bytes += len(data)

    def send(self, ftype: int, payload: bytes = b"") -> bytes:
        raw = encode(ftype, payload)
        self.send_raw(raw)
        return raw

    def send_many(self, raws: Iterable[bytes]) -> int:
        blob = b"".join(raws)
        self.send_raw(blob)
        return len(blob)

    def drain(self) -> None:
        """丢掉当前队列和驱动缓冲里的旧帧。"""
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
        self._ser.reset_input_buffer()

    def wait_frame(self, types: Optional[Iterable[int]] = None,
                   timeout: float = 1.0,
                   skip_async: bool = True,
                   predicate: Optional[Callable[[Frame], bool]] = None) -> Optional[Frame]:
        wanted = set(types) if types is not None else None
        deadline = time.monotonic() + timeout
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                return None
            try:
                frame = self._queue.get(timeout=remain)
            except queue.Empty:
                return None
            if skip_async and frame.type in ASYNC_TYPES:
                continue
            if wanted is not None and frame.type not in wanted:
                continue
            if predicate is not None and not predicate(frame):
                continue
            return frame

    def collect_frames(self, count: int, timeout: float = 3.0,
                       skip_async: bool = True) -> list[Frame]:
        out: list[Frame] = []
        deadline = time.monotonic() + timeout
        while len(out) < count:
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            try:
                frame = self._queue.get(timeout=remain)
            except queue.Empty:
                break
            if skip_async and frame.type in ASYNC_TYPES:
                continue
            out.append(frame)
        return out

    def stats(self) -> dict:
        with self._lock:
            return {
                "rx_bytes": self.rx_bytes,
                "tx_bytes": self.tx_bytes,
                "frames": len(self.frames),
                "bad_frames": self._parser.bad_frames,
            }

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass

    def __enter__(self) -> "SerialLink":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# --------------------------------------------------------------------------- #
# 消息解码
# --------------------------------------------------------------------------- #
@dataclass
class CalcResult:
    result: int
    proc_us: int
    status: int
    reserved: int


def decode_calc(frame: Frame) -> CalcResult:
    if len(frame.payload) != 8:
        raise ValueError(f"calc response payload len {len(frame.payload)}")
    result, proc_us, status, reserved = struct.unpack("<iHBB", frame.payload)
    return CalcResult(result, proc_us, status, reserved)


@dataclass
class Pong:
    uptime_ms: int
    proc_us: int
    valid_frames: int


def decode_pong(frame: Frame) -> Pong:
    uptime, proc_us, valid = struct.unpack("<IHH", frame.payload)
    return Pong(uptime, proc_us, valid)


@dataclass
class StatsPage:
    page: int
    flags: int
    values: tuple[int, int, int]


def decode_stats(frame: Frame) -> StatsPage:
    page, flags, v0, v1, v2 = struct.unpack("<BBHHH", frame.payload)
    return StatsPage(page, flags, (v0, v1, v2))


@dataclass
class Nack:
    code: int
    type: int
    length: int
    state: int
    raw: bytes

    @property
    def code_name(self) -> str:
        return ERR_NAMES.get(self.code, f"0x{self.code:02X}")


def decode_nack(frame: Frame) -> Nack:
    p = frame.payload
    if len(p) != 8:
        raise ValueError(f"nack payload len {len(p)}")
    return Nack(p[0], p[1], p[2], p[3], p[4:8])


@dataclass
class Heartbeat:
    uptime_ms: int
    valid_frames: int
    uart_errors: int


def decode_heartbeat(frame: Frame) -> Heartbeat:
    uptime, valid, errors = struct.unpack("<IHH", frame.payload)
    return Heartbeat(uptime, valid, errors)


def list_serial_ports() -> list[dict]:
    if list_ports is None:
        return []
    out = []
    for p in list_ports.comports():
        out.append({
            "device": p.device,
            "description": p.description,
            "vid": p.vid,
            "pid": p.pid,
            "serial": p.serial_number,
        })
    return out


def autodetect_port() -> Optional[str]:
    ports = list_serial_ports()
    if not ports:
        return None
    # 优先 CH343 / CH340（USB-TTL）
    for p in ports:
        desc = (p.get("description") or "").upper()
        if "CH343" in desc or "CH340" in desc or "USB-SERIAL" in desc:
            return p["device"]
    if len(ports) == 1:
        return ports[0]["device"]
    return None
