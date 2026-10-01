#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""设备协议层（USART1 那一路）：串口 + COBS + CRC32 + 组帧解帧 + Device。

三层上位机共用的最底层，**不含任何 CLI、也没有 OTA / 电机语义**：
  - `tools/ota.py`   —— 只管固件升级（info / flash / upgrade / rollback / reboot / erase）
  - `tools/motor.py` —— 只管电机（使能/切环/转速/走位/电源/电压/...）
  - `tools/car.py`   —— 只管小车（左右轮识别 / 差速走 / 看门狗 / 键盘遥控）

帧：`0x00 + COBS( SYNC(2) VER(1) TYPE(1) SEQ(1) LEN(2) payload CRC32(4) ) + 0x00`。
整帧 COBS 编码后不出现 `0x00`，裸文本日志里也不出现 `0x00` —— 两种内容在同一路
串口上靠定界符共存（见 docs/ota_design.md §5）。

帧类型/状态码要和设备侧 `Device/Ota/inc/ota_layout.h` 保持一致（改那边记得改这里）。
"""

from __future__ import annotations

import argparse
import glob
import os
import struct
import sys
import time
import zlib

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None       # 缺 pyserial 也要能跑 selftest（不碰串口的那部分）

# Windows 控制台默认是 GBK，直接打 ✅/⚠/→ 这些字符会 UnicodeEncodeError 把脚本弄崩。
# 强制 UTF-8 输出（VS Code 终端就是 UTF-8）；万一终端还是按 GBK 显示，也只是看着难看。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# ---------------------------------------------------------------------------
# 和 Device/Ota/inc/ota_layout.h 保持一致
# ---------------------------------------------------------------------------
SYNC = b"\xaa\x55"
PROTO_VER = 1
FRAME_OVERHEAD = 11                     # SYNC2 + VER + TYPE + SEQ + LEN2 + CRC4

T_INFO, T_BEGIN, T_DATA, T_END = 0x01, 0x02, 0x03, 0x04
T_ABORT, T_REBOOT, T_ERASE, T_ROLLBACK = 0x05, 0x06, 0x07, 0x08
T_CTRL, T_STATUS = 0x10, 0x11

ST_OK, ST_BADFRAME, ST_STATE, ST_OFFSET = 0, 1, 2, 3
ST_ERASE, ST_WRITE, ST_CRC, ST_PARAM, ST_NOTALLOWED = 4, 5, 6, 7, 8

ST_TEXT = {
    ST_OK: "OK", ST_BADFRAME: "坏帧", ST_STATE: "状态机不允许（先发 BEGIN）",
    ST_OFFSET: "偏移不对", ST_ERASE: "Flash 擦除失败", ST_WRITE: "Flash 写入失败",
    ST_CRC: "CRC32 不匹配", ST_PARAM: "参数非法", ST_NOTALLOWED: "不允许（比如要写正在运行的槽）",
}

# 总线上的电机台数（和 Device/Motor/motor_ctrl.c 的 MOTOR_COUNT 一致）。
# 达妙这颗的 ID 是上电时锁存的（低=ID1 / 高=ID2），所以一条总线最多两台。
N_MOTORS = 2

# 兜底串口：正常路径是"自动找"（见 resolve_port），只有系统上一个 USB 串口都没找到时
# 才用它去生成报错信息。Windows 的 COM 号是系统分配的，这里只给个样子货。
DEFAULT_PORT = "COM3" if os.name == "nt" else "/dev/ttyACM0"

# --port 的说明（顶层命令和子命令共用同一份）。常年在 Windows / Linux 之间换机器，
# 口名留不住，所以默认是自动找。
PORT_HELP = ("电脑这边的串口。不填就自动找（系统上只有一个 USB 串口时直接用它）；"
             "也可以设环境变量 CAR_PORT。Windows 形如 COM7，Linux 形如 /dev/ttyACM0")


class OtaError(Exception):
    """协议/串口层面的错误（各 CLI 统一捕获它打印友好信息）。"""


# ---------------------------------------------------------------------------
# COBS（和 Device/Ota/src/ota_link.c 同一套，已用 Python 对拍过）
# ---------------------------------------------------------------------------
def cobs_encode(data: bytes) -> bytes:
    out = bytearray(1)
    code = 1
    pos = 0

    for b in data:
        if b == 0x00:
            out[pos] = code
            code = 1
            pos = len(out)
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[pos] = code
                code = 1
                pos = len(out)
                out.append(0)

    out[pos] = code
    return bytes(out)


def cobs_decode(data: bytes):
    out = bytearray()
    pos = 0
    n = len(data)

    while pos < n:
        code = data[pos]
        if code == 0:
            return None
        pos += 1
        end = pos + code - 1
        if end > n:
            return None
        out += data[pos:end]
        if code != 0xFF:
            if end < n:
                out.append(0)
        pos = end

    return bytes(out)


# ---------------------------------------------------------------------------
# 小端解包 / 打包
# ---------------------------------------------------------------------------
def u16(b, o=0):
    return b[o] | (b[o + 1] << 8)


def u32(b, o=0):
    return b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)


def i32(b, o=0):
    v = u32(b, o)
    return v - (1 << 32) if v >= (1 << 31) else v


def i16(v: int) -> bytes:
    return struct.pack("<h", v)


def pair_unpack(v: int) -> tuple[int, int]:
    """DRIVE 命令的 arg：低 16 位 = 电机1，高 16 位 = 电机2（都是带符号的 0.1rpm）。"""
    lo = v & 0xFFFF
    hi = (v >> 16) & 0xFFFF
    if lo >= 0x8000:
        lo -= 0x10000
    if hi >= 0x8000:
        hi -= 0x10000
    return lo, hi


def pair_pack(v1: int, v2: int) -> int:
    return ((v2 & 0xFFFF) << 16) | (v1 & 0xFFFF)


# ---------------------------------------------------------------------------
# 串口 + 组帧
# ---------------------------------------------------------------------------
class Device:
    def __init__(self, port: str, baud: int, quiet: bool = False):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.quiet = quiet
        self.seq = 0
        self._reset_parser()

    def _reset_parser(self):
        """初始化“解帧用得到的那几个字段”。

        ⚠ 必须集中在这里，不要在 __init__ 里散着写：自检是用
          Device.__new__(Device) 造个假对象直接喂字节的，漏一个字段就会报 AttributeError。
        """
        self.rx_total = 0               # 一共收到多少字节（=0 基本就是波特率/模块问题）
        self._in_frame = False          # False = 正在看文本；True = 攒帧
        self._buf = bytearray()
        self._text = bytearray()
        self.frames: list[tuple[int, int, bytes]] = []

    # -- 低层 ---------------------------------------------------------------
    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _send(self, ftype: int, payload: bytes = b"", seq: int | None = None) -> int:
        if seq is None:
            self.seq = (self.seq + 1) & 0xFF
            seq = self.seq
        body = bytes([PROTO_VER, ftype, seq]) + struct.pack("<H", len(payload)) + payload
        frame = SYNC + body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)
        self.ser.write(b"\x00" + cobs_encode(frame) + b"\x00")
        self.ser.flush()
        return seq

    def _feed(self, b: int):
        self.rx_total += 1

        if b == 0x00:
            if self._in_frame:
                self._try_frame(bytes(self._buf))
                self._buf.clear()
                self._in_frame = False
            else:
                self._flush_text()
                self._in_frame = True
            return

        if self._in_frame:
            self._buf.append(b)
        else:
            self._text.append(b)
            if b == 0x0A:      # 文本按行实时打出来，monitor 才有意义
                self._flush_text()

    def _flush_text(self):
        if not self._text:
            return
        line = self._text.decode("utf-8", "replace")
        self._text.clear()
        if not self.quiet:
            sys.stdout.write(line)
            sys.stdout.flush()

    def _try_frame(self, raw: bytes):
        data = cobs_decode(raw)
        if not data or len(data) < FRAME_OVERHEAD:
            return
        if data[0:2] != SYNC or data[2] != PROTO_VER:
            return
        ln = u16(data, 5)
        if ln + FRAME_OVERHEAD != len(data):
            return
        if zlib.crc32(data[2:7 + ln]) & 0xFFFFFFFF != u32(data, 7 + ln):
            return
        self.frames.append((data[3], data[4], data[7:7 + ln]))

    def pump(self, timeout: float):
        """读串口直到超时（或收到帧），期间的文本直接打出来、帧进 self.frames。

        ⚠ 两个坑（实测每包白白多等 ~100 ms）：
          1) 不能用 self.ser.read(256) —— pyserial 的 read(n) 会一直等到读满 n 个字节
             **或者超时**才返回，所以一条 19 字节的回复每次都要把剩下的超时耗光；
             要用 in_waiting “有多少读多少”。
          2) 帧已经解出来就别再把剩下的时间等完 —— 否则调用方要等下一次循环才看得到它。
        """
        end = time.time() + timeout
        while True:
            remain = end - time.time()
            if remain <= 0:
                break

            n = self.ser.in_waiting
            if n:
                chunk = self.ser.read(n)              # 立即返回，不等满
            else:
                self.ser.timeout = min(remain, 0.005)  # 没数据时才短暂等一下
                chunk = self.ser.read(1)

            if chunk:
                for b in chunk:
                    self._feed(b)
                if self.frames:                        # 已经有帧了：别再耗时间
                    break

        self._flush_text()

    def wait_frame(self, ftype: int, seq: int, timeout: float):
        end = time.time() + timeout
        while True:
            for i, (t, s, p) in enumerate(self.frames):
                if t == ftype and (s == seq or seq is None):
                    return self.frames.pop(i)[2]
            if time.time() >= end:
                return None
            self.pump(min(0.1, end - time.time()))

    # -- 高层 ---------------------------------------------------------------
    def request(self, ftype: int, payload: bytes = b"", timeout: float = 2.0,
                retries: int = 1, expect: int | None = None):
        """发一帧等回复。返回回复载荷（第一个字节是 status），超时返回 None。"""
        expect = expect if expect is not None else (ftype | 0x80)
        for _ in range(retries):
            seq = self._send(ftype, payload)
            rsp = self.wait_frame(expect, seq, timeout)
            if rsp is not None:
                return rsp
        return None

    def get_info(self, timeout: float = 2.0, retries: int = 3):
        """读设备元数据（分区/版本/CRC/启动计数）。"""
        rsp = self.request(T_INFO, timeout=timeout, retries=retries)
        if rsp is None:
            raise OtaError("设备没有回复 INFO：检查无线串口是否连上、波特率是否一致、"
                           "设备是否在跑（或重启进 Bootloader 恢复台）")
        if rsp[0] != ST_OK:
            raise OtaError(f"INFO 返回 {ST_TEXT.get(rsp[0], rsp[0])}")
        return {
            "status": rsp[0],
            "active": rsp[1], "boot": rsp[2], "pending": rsp[3],
            "attempts": rsp[4], "flags": rsp[5],
            "slot": [
                {"size": u32(rsp, 6 + 12 * i), "crc": u32(rsp, 10 + 12 * i),
                 "ver": u32(rsp, 14 + 12 * i)}
                for i in range(2)
            ],
            "fw": u32(rsp, 30), "bl": u32(rsp, 34),
        }


# ---------------------------------------------------------------------------
# 三套 CLI 共用的“外壳”
# ---------------------------------------------------------------------------
# 串口的名字每次都变（Windows 是 COM*、Linux 是 /dev/ttyACM* 或 /dev/ttyUSB*），
# 所以默认**自动找**：系统上只有一个 USB 串口就直接用它。
# 蓝牙虚拟串口在 Windows 上也是 COM*（"蓝牙链接上的标准串行"），但不是我们的设备。
_NOT_USB_WORDS = ("bluetooth", "蓝牙")


def _usb_like_name(dev: str) -> bool:
    return os.path.basename(dev).lower().startswith(
        ("ttyacm", "ttyusb", "cu.usb", "cu.usbserial", "usbmodem"))


def list_serial_ports():
    """系统上能看到的串口 → [(device, description, 像不像 USB 串口), ...]。

    Windows 上蓝牙虚拟串口也叫 COM*（"蓝牙链接上的标准串行"），但它不是我们要的设备，
    直接标记成不像 USB —— 不然自动挑选时可能选到它。
    """
    try:
        from serial.tools import list_ports
    except Exception:                      # 没装 pyserial：至少把 /dev 下的常见名字列出来
        out = []
        for pat in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usb*", "/dev/ttyS*"):
            out += [(d, "", _usb_like_name(d)) for d in sorted(glob.glob(pat))]
        return out

    out = []
    for p in list_ports.comports():
        desc = p.description or ""
        text = f"{desc} {p.manufacturer or ''} {p.hwid or ''}".lower()
        usb = (not any(w in text for w in _NOT_USB_WORDS)) and \
              (_usb_like_name(p.device) or "usb" in text)
        out.append((p.device, desc, usb))

    return out


def auto_port():
    """自动挑串口：**只有一个**像 USB 串口的就返回 (device, description)，否则 None。

    挑不出来（0 个或多个）时宁可不猜 —— 猜错了命令就发到蓝牙/主板串口上去了，
    报错信息里会把可选的口都列出来。
    """
    usb = [p for p in list_serial_ports() if p[2]]

    return (usb[0][0], usb[0][1]) if len(usb) == 1 else None


def resolve_port(arg_port):
    """串口从哪来：`--port` > 环境变量 `CAR_PORT` > 自动挑 > 兜底串口。"""
    if arg_port:
        return arg_port

    env = os.environ.get("CAR_PORT")
    if env:
        return env

    hit = auto_port()
    if hit:
        dev, desc = hit
        # 说一句“用了哪个口”：不然用户不知道刚才那一下是发到哪儿去了。
        # 描述里一般都带口名（Windows 的 "USB 串行设备 (COM15)"），去掉免得重复；
        # 提示走 stderr —— stdout 留给命令自己的输出。
        note = desc.split("(")[0].strip() if desc else ""
        if note:
            note = f"（{note}）"
        print(f"自动选中串口 {dev}{note}；要换就 --port 或设环境变量 CAR_PORT",
              file=sys.stderr)
        return dev

    return DEFAULT_PORT


def check_port(port: str) -> None:
    """串口存在性检查：不存在就把系统上能用的口列出来（提示按当前系统给）。

    ⚠ Windows 的 `COM15` 不是文件系统路径，`os.path.exists()` 永远是假 —— 那边要拿
    系统枚举出来的名单比对（pyserial 在，所以 open_device 保证能列出来）。
    “口不存在”几乎都不是代码问题：要么设备没插好/认成别的名字，要么口在但当前
    用户读不了（Linux 要进 dialout 组，报的是 PermissionError）。
    """
    if not port or os.path.exists(port):
        return

    ports = list_serial_ports()
    usb = [p for p in ports if p[2]]

    if any(d.upper() == port.upper() for d, _, _ in ports):
        return                    # 系统里有这个口（Windows 的 COM* 就走这条路）

    if ports:
        # 主板自带的串口（/dev/ttyS*、Windows 的 COM1）不接设备，列表里看着很吵：
        # 有 USB 串口就只列它们，没有才退回全部。
        shown = usb if usb else ports[:8]
        lines = "\n".join(f"     {d}   {desc}" for d, desc, _ in shown)
        if not usb:
            lines += "\n     （上面没有 USB 串口 —— 板子真插好了吗？）"
        elif len(usb) > 1:
            lines += "\n     （有多个 USB 串口，用 --port 挑一个）"
    else:
        lines = "     （一个都没有：USB 线 / 驱动 / 模块供电都看一眼）"

    if os.name == "nt":
        hint = ("Windows 上板子以「USB 串行设备」或「USB-SERIAL CH340」出现，"
                "设备管理器 → 端口(COM 和 LPT) 里能看到，用 `--port COM7` 指定。")
    else:
        hint = ("USB-CDC 是 /dev/ttyACM*、CH340/CP210x 是 /dev/ttyUSB*，"
                "用 `--port /dev/ttyUSB0` 指定或 `ls /dev/tty*` 看一眼；"
                "口在但打不开（Permission denied）：sudo usermod -aG dialout $USER 后重新登录。")

    raise OtaError(f"串口 {port} 不存在。当前系统上找到的串口：\n{lines}\n   提示：{hint}")


def open_device(port: str, baud: int, quiet: bool = False) -> Device:
    """check_port + 打开串口；失败抛 OtaError（信息里带常见原因）。"""
    if serial is None:
        raise OtaError("缺少 pyserial：pip install pyserial")
    if not port:
        raise OtaError("没有指定串口（用 --port，或者设环境变量 CAR_PORT）")
    check_port(port)
    try:
        return Device(port, baud, quiet=quiet)
    except serial.SerialException as e:
        raise OtaError(
            f"打不开 {port}：{e}\n"
            f"   口存在但打不开 → 多半是权限（Linux: sudo usermod -aG dialout $USER 后重新登录），"
            f"或者别的程序（minicom / 上一个脚本）还占着这个口。") from e
    except Exception as e:                      # pragma: no cover
        raise OtaError(f"打不开 {port}：{e}") from e


def add_port_args(ap, baud: int = 921600, baud_help: str | None = None):
    """给 argparse 加上三套 CLI 共用的 --port / --baud / -q。

    --port 默认 None = **自动找**：常年在 Windows / Linux 之间换机器，写死
    /dev/ttyACM0 或 COM7 都留不住（见 resolve_port）。
    """
    ap.add_argument("--port", default=None, help=PORT_HELP)
    ap.add_argument("--baud", type=int, default=baud, help=baud_help or f"默认 {baud}")
    ap.add_argument("-q", "--quiet", action="store_true", help="不要把设备日志打到屏幕上")


# 子命令也能直接写这些开关：`motor.py status --port COM15` 和 `--port COM15 status` 都认
# （凭直觉写哪个位置都行）。默认值用 SUPPRESS —— 子命令里没写这几个开关时就**不要**往
# namespace 里塞东西，否则子解析器的默认值会把上层已经解析到的值盖掉。
PORT_ARGS = argparse.ArgumentParser(add_help=False)
PORT_ARGS.add_argument("--port", default=argparse.SUPPRESS, help=PORT_HELP)
PORT_ARGS.add_argument("--baud", type=int, default=argparse.SUPPRESS, help="见上面（顶层）的 --baud")
PORT_ARGS.add_argument("-q", "--quiet", action="store_true", default=argparse.SUPPRESS,
                       help="不要把设备日志打到屏幕上")


def add_subparsers(ap, **kwargs):
    """`ap.add_subparsers()` 的包装：让**每个子命令**自动带上 --port / --baud / -q。"""
    sub = ap.add_subparsers(**kwargs)
    inner = sub.add_parser

    def add_parser(name, **kw):
        kw["parents"] = list(kw.get("parents", [])) + [PORT_ARGS]
        return inner(name, **kw)

    sub.add_parser = add_parser
    return sub


def rx_hint(port: str, baud: int) -> str:
    """“一个字节都没收到”时的排查提示。"""
    where = ("Windows: 设备管理器 → 端口(COM 和 LPT)；"
             if os.name == "nt" else "Linux: `ls /dev/tty*` + `dmesg | tail`；")

    return (f"   {port} @{baud} 一个字节都没收到，优先查：\n"
            f"     1) 波特率：设备和 BL 恢复台都是 921600，**无线模块**的串口波特率也要设成 921600\n"
            f"     2) 串口号对不对（{where}模块有没有配对/上电）")


def serve(args, func, needs_port: bool = True, interrupt_msg: str = ""):
    """三套 CLI 的公共入口：连串口 → 跑 func(dev, args) → 关；出错打印并返回退出码。

    needs_port=False（比如“只改本地配置”的子命令）时 func 收到 dev=None。
    """
    if not needs_port:
        try:
            return int(func(None, args) or 0)
        except OtaError as e:
            print(f"\n❌ {e}", file=sys.stderr)
            return 1

    try:
        port = resolve_port(getattr(args, "port", None))
        args.port = port            # 后面的 func 如果打印串口名，用的是同一个
        dev = open_device(port, args.baud, quiet=getattr(args, "quiet", False))
    except OtaError as e:
        print(f"\n❌ {e}", file=sys.stderr)
        return 1

    rc = 0
    try:
        r = func(dev, args)
        if isinstance(r, int):
            rc = r
    except OtaError as e:
        print(f"\n❌ {e}", file=sys.stderr)
        if dev.rx_total == 0:
            print(rx_hint(args.port, args.baud), file=sys.stderr)
        rc = 1
    except KeyboardInterrupt:
        print("\n中断。" + interrupt_msg)
        rc = 1
    except EOFError:
        print("\n❌ 读不到输入（终端不是交互式的，或者输入被关了）", file=sys.stderr)
        rc = 1
    finally:
        dev.close()

    return rc
