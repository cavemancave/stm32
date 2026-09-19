#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无线 OTA 上位机：通过无线串口（USART1 那一路）给板子升级固件 / 看日志 / 发控制命令。

    pip install pyserial

用法（COM7 是电脑这边配对的无线串口）：
    python tools/ota.py --port COM7 info                    # 看当前在哪个槽、版本、CRC
    python tools/ota.py --port COM7 upgrade                 # ★一条命令升级：自动挑槽 + 自动挑镜像
    python tools/ota.py --port COM7 flash build/Debug/motor_control_slotB.bin
    python tools/ota.py --port COM7 monitor                 # 当串口监视器看日志
    python tools/ota.py --port COM7 rollback                # 新固件有问题 → 切回旧槽
    python tools/ota.py --port COM7 reboot --boot           # 重启进 Bootloader 恢复台
    python tools/ota.py --port COM7 status                  # 电机里程/位置/故障码
    python tools/ota.py --port COM7 ctrl disable            # 电机失能
    python tools/ota.py --port COM7 ctrl pwron              # 电机电源上电（PC14 拉高）
    python tools/ota.py --port COM7 ctrl pwcycle            # 断电重启电机（状态卡死时用）

协议：docs/ota_design.md §5。帧是 `0x00 + COBS(AA 55 VER TYPE SEQ LEN payload CRC32) + 0x00`，
所以**裸文本日志**和**二进制帧**能在同一路串口上共存：文本里不会出现 0x00，
而帧里也不会出现 0x00，靠定界符就能分开。
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time
import zlib

try:
    import serial
except ImportError:  # pragma: no cover
    sys.exit("缺少 pyserial：pip install pyserial")

# Windows 控制台默认是 GBK，直接打 ✅/⚠/→ 这些字符会 UnicodeEncodeError 把脚本弄崩。
# 强制 UTF-8 输出（VS Code 终端就是 UTF-8）；万一终端还是按 GBK 显示，也只是看着难看，不会崩。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# ---------------------------------------------------------------------------
# 和 Device/Ota/inc/ota_layout.h 保持一致（改了那边记得改这里）
# ---------------------------------------------------------------------------
SYNC = b"\xaa\x55"
PROTO_VER = 1
FRAME_OVERHEAD = 11                     # SYNC2 + VER + TYPE + SEQ + LEN2 + CRC4

SLOT_A_BASE = 0x08020000
SLOT_B_BASE = 0x08080000
SLOT_SIZE = 384 * 1024

# 构建产物目录：按**脚本所在位置**算，不从当前目录算 ——
# 这样在仓库根目录、在 tools/ 里、或者从别的地方调用都不会找不到 .bin。
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
BUILD_DIR = os.path.join(_REPO, "build", "Debug")

# 两个槽对应的镜像文件名（同一份源码按不同基址链接出来的两份）
IMAGE_FOR_SLOT = {0: "motor_control.bin", 1: "motor_control_slotB.bin"}

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

CTRL = {
    "disable": 0x00, "enable": 0x01, "posloop": 0x02, "movepos": 0x03,
    "movedeg": 0x04, "stop": 0x05, "mute": 0x06, "pause": 0x07,
    "pwr": 0x08,          # 电机电源（PC14）：--arg 0=断电 1=上电 2=断电重启
}

# 带固定参数的快捷命令：名字 → (cmd, arg)。想手动指定就再加 --arg。
#   pwron   = 上电（设备会等 500 ms 让电源轨稳定）
#   pwoff   = 断电
#   pwcycle = 断电重启（电机状态卡死时用得上）
CTRL_ARG = {"pwron": (0x08, 1), "pwoff": (0x08, 0), "pwcycle": (0x08, 2)}


# ---------------------------------------------------------------------------
# COBS（和 Device/Ota/src/ota_link.c 同一套，已用 Python 对拍过）
# ---------------------------------------------------------------------------
def cobs_encode(data: bytes) -> bytes:
    out = bytearray(1)
    code_pos, code, wr = 0, 1, 1
    for b in data:
        if b != 0:
            out.append(b)
            wr += 1
            code += 1
        if code == 0xFF:
            out[code_pos] = code
            code_pos = wr
            out.append(0)
            wr += 1
            code = 1
        elif b == 0:
            out[code_pos] = code
            code_pos = wr
            out.append(0)
            wr += 1
            code = 1
    out[code_pos] = code
    return bytes(out[:wr])


def cobs_decode(data: bytes):
    out = bytearray()
    rd = 0
    while rd < len(data):
        code = data[rd]
        rd += 1
        if code == 0:
            return None
        for _ in range(1, code):
            if rd >= len(data):
                return None
            out.append(data[rd])
            rd += 1
        if code < 0xFF and rd < len(data):
            out.append(0)
    return bytes(out)


def u16(b, o=0):
    return b[o] | (b[o + 1] << 8)


def u32(b, o=0):
    return b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)


def i32(b, o=0):
    v = u32(b, o)
    return v - (1 << 32) if v & 0x80000000 else v


# ---------------------------------------------------------------------------
# 串口 + 组帧
# ---------------------------------------------------------------------------
class OtaError(Exception):
    pass


class Device:
    def __init__(self, port: str, baud: int, quiet: bool = False):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.quiet = quiet
        self.seq = 0
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
        """读串口直到超时，期间的文本直接打出来、帧进 self.frames"""
        end = time.time() + timeout
        while True:
            remain = end - time.time()
            if remain <= 0:
                break
            self.ser.timeout = min(remain, 0.05)
            chunk = self.ser.read(256)
            if not chunk:
                continue
            for b in chunk:
                self._feed(b)
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


def slot_name(s: int) -> str:
    return {0: "A", 1: "B", 0xFF: "-"}.get(s, f"0x{s:02X}")


def ver_str(v: int) -> str:
    return f"{v >> 16}.{(v >> 8) & 0xFF}.{v & 0xFF}"


def bin_slot_of_image(image: bytes) -> int | None:
    """从 .bin 的复位向量（向量表第 2 个字）判断这份镜像是给哪个槽编的。
    镜像里的地址是链接期定死的，所以这一眼就能看出来，防止把 A 的镜像发去 B 槽。"""
    if len(image) < 8:
        return None
    pc = u32(image, 4)
    if SLOT_A_BASE <= pc < SLOT_A_BASE + SLOT_SIZE:
        return 0
    if SLOT_B_BASE <= pc < SLOT_B_BASE + SLOT_SIZE:
        return 1
    return None


# ---------------------------------------------------------------------------
# 子命令
# ---------------------------------------------------------------------------
def print_info(info: dict):
    print("设备状态 ─────────────────────────────────────────────")
    print(f"  活动槽   : {slot_name(info['active'])}"
          f"   下次启动: {slot_name(info['boot'])}"
          f"   待切换: {slot_name(info['pending'])}")
    print(f"  启动计数 : {info['attempts']}/3（到 3 没被 App 确认就自动回滚）"
          f"   flags=0x{info['flags']:02X}")
    if info["flags"] & 0x80:
        print("  ⚠ 设备正在 **Bootloader 恢复台** 里（没有 App 在跑）："
              "可以往任意一个槽写固件（--slot A / B）")
    print(f"  运行版本 : {ver_str(info['fw'])}   Bootloader: {ver_str(info['bl'])}")
    for i, nm in enumerate("AB"):
        s = info["slot"][i]
        if s["size"]:
            print(f"  槽 {nm}     : {s['size']:>7} 字节  crc32=0x{s['crc']:08X}  ver={ver_str(s['ver'])}")
        else:
            print(f"  槽 {nm}     : （元数据里没登记 / 空的）")
    print("──────────────────────────────────────────────────────")


def cmd_info(dev: Device, args):
    print_info(dev.get_info())


def cmd_monitor(dev: Device, args):
    print(f"监听 {dev.ser.port}（Ctrl-C 退出）... 设备日志会原样打出来，"
          f"上位机发出去的帧/设备回的控制帧会标出来")
    try:
        while True:
            dev.pump(0.3)
            while dev.frames:
                t, s, p = dev.frames.pop(0)
                print(f"  [帧] type=0x{t:02X} seq={s} len={len(p)} payload={p.hex()}")
    except KeyboardInterrupt:
        print()


def cmd_flash(dev: Device, args):
    image = open(args.file, "rb").read()
    if not image:
        raise OtaError("固件文件是空的")
    if len(image) > SLOT_SIZE:
        raise OtaError(f"固件 {len(image)} 字节超过一个槽（{SLOT_SIZE} 字节）")

    info = dev.get_info()
    in_bl = bool(info["flags"] & 0x80)          # 1 = 设备在 Bootloader 恢复台里
    running = info["active"] if info["active"] < 2 else 0

    # 目标槽：规则见 pick_target_slot()（和 upgrade 共用同一份逻辑，不会两边跑偏）
    target = pick_target_slot(info, args.slot)

    if (target == running) and not in_bl:
        raise OtaError(
            f"目标槽 {slot_name(target)} 就是设备正在运行的槽 —— 写它等于擦掉自己。"
            f"请升另一个槽（默认 auto 就会挑对）")

    img_slot = bin_slot_of_image(image)
    if img_slot is None:
        raise OtaError("这个 .bin 的复位向量不在任何槽里：确认它是 motor_control(.bin) 或 "
                       "motor_control_slotB.bin，而不是 motor_boot.bin / 别的工程的产物")
    if img_slot != target:
        want = f"motor_control{'_slotB' if target == 1 else ''}.bin"
        raise OtaError(
            f"镜像槽不匹配：这份 .bin 是给槽 {slot_name(img_slot)} 编的，"
            f"但目标槽是 {slot_name(target)}。\n  应该用 {want}")

    crc = zlib.crc32(image) & 0xFFFFFFFF
    fw_ver = args.version if args.version is not None else 0x00010000

    print(f"升级 {args.file}")
    print(f"  镜像 {len(image)} 字节 crc32=0x{crc:08X} → 目标槽 {slot_name(target)}"
          f"（设备{'在恢复台' if in_bl else f'跑在 {slot_name(running)}'}）")

    flags = 0x01 if args.force else 0x00
    payload = struct.pack("<BIIIB", target, len(image), crc, fw_ver, flags)

    t0 = time.time()
    rsp = dev.request(T_BEGIN, payload, timeout=10.0, retries=3)
    if rsp is None:
        raise OtaError("BEGIN 没有回复（设备可能在擦除 3 个扇区，超时给够 10 s 了吗）")
    if rsp[0] != ST_OK:
        raise OtaError(f"BEGIN 失败：{ST_TEXT.get(rsp[0], rsp[0])}")

    off = u32(rsp, 1)
    chunk = min(u16(rsp, 5) or args.chunk, args.chunk)
    if off:
        print(f"  断点续传：设备已经有 {off} 字节，从这儿接着传（要重来加 --force）")

    total = len(image)
    sent = off
    timeout = max(2.0, 0.6 + 3.0 * (chunk + 8) * 10.0 / args.baud)

    while off < total:
        block = image[off:off + chunk]
        ok = False
        for attempt in range(args.retries):
            rsp = dev.request(T_DATA, struct.pack("<I", off) + block, timeout=timeout)
            if rsp is None:
                continue
            if rsp[0] == ST_OK:
                nxt = u32(rsp, 1)
                if nxt != off + len(block):
                    print(f"  ! 设备说下一个偏移是 {nxt}，按它走")
                off = nxt
                ok = True
                break
            if rsp[0] == ST_OFFSET:
                off = u32(rsp, 1)
                print(f"  ! 偏移不对，设备要 {off} 字节处（重传）")
                ok = True
                break
            print(f"  ! 第 {off} 字节这包失败：{ST_TEXT.get(rsp[0], rsp[0])}（第 {attempt + 1} 次）")
        if not ok:
            dev.request(T_ABORT, timeout=2.0)
            raise OtaError(f"在 {off}/{total} 字节处连续失败，已 ABORT（进度保留，下次可续传）")
        sent = off
        pct = 100.0 * sent / total
        speed = sent / max(time.time() - t0, 1e-3)
        sys.stdout.write(f"\r  {sent:>7}/{total} 字节  {pct:5.1f}%  {speed / 1024:5.1f} KB/s   ")
        sys.stdout.flush()

    print()
    rsp = dev.request(T_END, timeout=15.0, retries=3)
    if rsp is None:
        raise OtaError("END 没有回复（设备在把整个槽读回来算 CRC32，超时给够 15 s）")
    if rsp[0] != ST_OK:
        raise OtaError(f"END 校验失败：{ST_TEXT.get(rsp[0], rsp[0])}"
                       f"（设备读回算出的 crc32=0x{u32(rsp, 2):08X}，本地 0x{crc:08X}）")
    got_crc = u32(rsp, 2)
    print(f"  校验通过：槽 {slot_name(rsp[1])} crc32=0x{got_crc:08X}，"
          f"元数据已提交，设备正在重启 ...")

    if args.wait_boot:
        print("  等设备重启并切槽（最多 20 s）...")
        deadline = time.time() + 20
        while time.time() < deadline:
            time.sleep(1.0)
            try:
                info2 = dev.get_info(timeout=1.5, retries=1)
            except OtaError:
                continue
            if info2["active"] == target or info2["boot"] == target:
                print_info(info2)
                print(f"✅ 升级完成：现在跑在槽 {slot_name(info2['active'])}，"
                      f"版本 {ver_str(info2['fw'])}")
                return
        print("⚠ 20 s 内没等到设备回到正常状态：用 `info` 再看看，"
              "或者 `reboot --boot` 进恢复台、`rollback` 切回旧槽")
    else:
        print("  （可以用 `info` 确认新版本，用 `rollback` 切回旧槽）")


def image_path_for_slot(slot: int) -> str:
    """这个槽该用哪份 .bin"""
    return os.path.join(BUILD_DIR, IMAGE_FOR_SLOT[slot])


def pick_target_slot(info: dict, slot_arg: str = "auto") -> int:
    """目标槽 = 要写哪个槽。规则和 cmd_flash 里完全一样，flash / upgrade 共用：
       - 指定了 A/B → 就写那个；
       - 设备在 BL 恢复台 → 写元数据里的 active 槽（就是跑不起来的那个）；
       - 上次切槽没成功（pending 还在）→ 接着写同一个槽；
       - 否则 → 写另一个槽（永远不碰正在跑的那个）。
    """
    running = info["active"] if info["active"] < 2 else 0
    in_bl = bool(info["flags"] & 0x80)

    if slot_arg != "auto":
        return 0 if slot_arg.upper() == "A" else 1
    if in_bl:
        return running
    if info["pending"] < 2:
        return info["pending"]
    return 1 - running


def cmd_upgrade(dev: Device, args):
    """和 flash 完全一样，唯一区别：**不带文件时自动挑**。

    先问设备现在跑在哪个槽，再发另一个槽对应的那份 .bin：
        A 槽在跑 → 发 motor_control_slotB.bin（写 B 槽）
        B 槽在跑 → 发 motor_control.bin（写 A 槽）
    所以升级成功后，再跑一次 upgrade 就会切回另一个槽 —— 交替升级，两个槽都是当前构建。
    """
    if args.file is None:
        info = dev.get_info()
        in_bl = bool(info["flags"] & 0x80)
        running = info["active"] if info["active"] < 2 else 0
        target = pick_target_slot(info, args.slot)
        path = image_path_for_slot(target)

        if not os.path.exists(path):
            raise OtaError(
                f"自动挑出来的镜像是 {path}，但文件不存在。\n"
                f"  先构建一次（三个镜像都会生成），或者手动指定：upgrade <file.bin>")

        print(f"设备{'在 Bootloader 恢复台' if in_bl else f'跑在槽 {slot_name(running)}'}"
              f" → 写入槽 {slot_name(target)}，自动选镜像 {os.path.basename(path)}")

        # 顺手提醒一句：目标槽里已经是同一份镜像时不拦，但先说一声（省得看着进度条以为坏了）
        same = info["slot"][target]
        crc = zlib.crc32(open(path, "rb").read()) & 0xFFFFFFFF
        if same["size"] and same["crc"] == crc:
            print(f"  注意：槽 {slot_name(target)} 里已经是同一份镜像（crc32 一致）—— "
                  f"传一遍只是确认，源码没变的话结果不会变")

        args.file = path

    return cmd_flash(dev, args)


def cmd_rollback(dev: Device, args):
    rsp = dev.request(T_ROLLBACK, timeout=5.0, retries=3)
    if rsp is None:
        raise OtaError("ROLLBACK 没有回复")
    if rsp[0] != ST_OK:
        raise OtaError(f"回滚失败：{ST_TEXT.get(rsp[0], rsp[0])}"
                       f"（另一个槽没有可用镜像？可以 reboot --boot 进恢复台重发）")
    print(f"已切到槽 {slot_name(rsp[1])} 并重启")


def cmd_erase(dev: Device, args):
    info = dev.get_info()
    target = 0 if args.slot.upper() == "A" else 1
    if target == info["active"]:
        raise OtaError("不能擦正在运行的槽")
    rsp = dev.request(T_ERASE, bytes([target]), timeout=10.0, retries=3)
    if rsp is None or rsp[0] != ST_OK:
        raise OtaError(f"擦除失败：{ST_TEXT.get(rsp[0], rsp[0]) if rsp else '无回复'}")
    print(f"槽 {slot_name(target)} 已擦除")


def cmd_reboot(dev: Device, args):
    mode = 1 if args.boot else 0
    dev.request(T_REBOOT, bytes([mode]), timeout=3.0, retries=3)
    print("已请求重启" + ("，并停在 Bootloader 恢复台（15 s 不动会自动尝试启动）" if mode else ""))


def cmd_status(dev: Device, args):
    rsp = dev.request(T_STATUS, timeout=3.0, retries=3)
    if rsp is None:
        raise OtaError("STATUS 没有回复（设备在跑吗）")
    if rsp[0] != ST_OK:
        raise OtaError(f"取状态失败：{ST_TEXT.get(rsp[0], rsp[0])}（电机没上电/没接线？）")
    # 电机电源（PC14）是后加到回帧末尾的：老固件没有这一字节，所以判一下长度
    pwr = f"  电机电源={'ON' if rsp[9] else 'OFF'}" if len(rsp) > 9 else ""
    print(f"里程={i32(rsp, 1)} 圈  位置={u16(rsp, 5)}（{u16(rsp, 5) * 360.0 / 32768:.1f}°）  "
          f"故障码=0x{rsp[7]:02X}  模式=0x{rsp[8]:02X}{pwr}")


def cmd_ctrl(dev: Device, args):
    if args.cmd in CTRL_ARG:
        cmd, arg = CTRL_ARG[args.cmd]
        if args.arg is not None:
            arg = args.arg              # 允许手动覆盖（比如 pwr --arg 2）
    else:
        cmd = CTRL[args.cmd]
        arg = args.arg if args.arg is not None else (
            1 if args.cmd in ("mute", "pause") else 0)
    rsp = dev.request(T_CTRL, struct.pack("<BI", cmd, arg), timeout=15.0, retries=2)
    if rsp is None:
        raise OtaError(f"{args.cmd} 没有回复（使能会重试约 5 s，再等等）")
    pwr = ""
    if cmd == 0x08:                     # 电源命令：data 返回当前状态（0/1）
        pwr = f"  电机电源={'ON' if u32(rsp, 1) else 'OFF'}"
    print(f"{args.cmd}({arg}) → {ST_TEXT.get(rsp[0], rsp[0])}  data={u32(rsp, 1)}{pwr}")


def cmd_console(dev: Device, args):
    """不支持的上位机（比如串口助手）也能用：直接敲命令，回显裸文本。"""
    cmd_monitor(dev, args)


def cmd_selftest(dev, args):
    """不用连板子，验证协议层：CRC32 / COBS / 组帧解帧 / 镜像槽识别。
    硬件还没接上时先跑这个，能排掉一大类“改错常量”的低级问题。"""
    print("1) CRC32 自检（必须等于 zlib.crc32）")
    assert zlib.crc32(b"123456789") & 0xFFFFFFFF == 0xCBF43926
    print("   0xCBF43926 OK")

    print("2) COBS 往返（含 0x00 / 全 0 / 254-256 边界）")
    for data in (b"", b"\x00", b"\x00\x00", b"abc", b"ab\x00c", bytes(254), bytes(255),
                 bytes(256), b"\x00" + bytes(254), bytes(255) + b"\x00", os.urandom(1024)):
        enc = cobs_encode(data)
        assert 0 not in enc, "COBS 编码结果里不能出现 0x00"
        assert cobs_decode(enc) == data, f"往返失败 len={len(data)}"
    print("   全部往返成功（且编码结果里没有 0x00）")

    print("3) 组帧 / 解帧往返 + 坏帧丢弃")
    fake = Device.__new__(Device)
    fake.quiet = True
    fake._in_frame = False
    fake._buf = bytearray()
    fake._text = bytearray()
    fake.frames = []
    payload = struct.pack("<BHI", 7, 1024, 0xDEADBEEF)
    body = bytes([PROTO_VER, T_DATA | 0x80, 9]) + struct.pack("<H", len(payload)) + payload
    good = SYNC + body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)
    bad = bytearray(good)
    bad[5] ^= 0x01                                    # 改一字节 → CRC 必须不过
    stream = b"log line\r\n" + b"\x00" + cobs_encode(good) + b"\x00" + \
        b"\x00" + cobs_encode(bytes(bad)) + b"\x00" + b"more text\r\n"
    for b in stream:
        fake._feed(b)
    assert len(fake.frames) == 1, f"应该只解出 1 帧，实际 {len(fake.frames)}"
    t, s, p = fake.frames[0]
    assert (t, s, p) == (T_DATA | 0x80, 9, payload), "解出来的帧内容不对"
    print("   文本和帧混流 OK；坏帧被丢掉、没污染后面的帧")

    print("4) 镜像槽识别（build/Debug/*.bin，没有就跳过）")
    found = False
    for f, want in (("motor_control.bin", 0), ("motor_control_slotB.bin", 1),
                    ("motor_boot.bin", None)):
        path = os.path.join(BUILD_DIR, f)
        if not os.path.exists(path):
            continue
        img = open(path, "rb").read()
        got = bin_slot_of_image(img)
        assert got == want, f"{f} 应该识别为槽 {want}，实际 {got}"
        crc = zlib.crc32(img) & 0xFFFFFFFF
        print(f"   {f}: {len(img)} 字节, 槽={slot_name(got) if got is not None else '无'}"
              f", crc32=0x{crc:08X}")
        found = True
    if not found:
        print("   （build/Debug 里没找到 .bin，先构建一次）")

    print("\n✅ 自检全部通过：协议层和固件是同一套（常量对得上）")
    return 0


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="无线 OTA 上位机（STM32H723 motor_control）",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("--port", help="电脑这边的配对串口，如 COM7 或 /dev/ttyUSB0（selftest 不需要）")
    ap.add_argument("--baud", type=int, default=115200, help="默认 115200，要和固件 OTA_PORT_BAUD 一致")
    ap.add_argument("-q", "--quiet", action="store_true", help="不要把设备日志打到屏幕上")

    sub = ap.add_subparsers(dest="action", required=True)

    # flash / upgrade 共用的参数（upgrade 只是"文件可以不填"）
    flash_args = argparse.ArgumentParser(add_help=False)
    flash_args.add_argument("--slot", default="auto", help="auto（默认，写非活动槽）/ A / B")
    flash_args.add_argument("--chunk", type=int, default=1024, help="每包字节数，默认 1024")
    flash_args.add_argument("--retries", type=int, default=5, help="每一包的重传次数")
    flash_args.add_argument("--force", action="store_true", help="强制重新擦除，不续传")
    flash_args.add_argument("--version", type=lambda s: int(s, 0), default=None,
                            help="版本号，如 0x00010200")
    flash_args.add_argument("--no-wait-boot", dest="wait_boot", action="store_false",
                            help="升完不等设备重启确认（默认会等并打印新状态）")

    sub.add_parser("selftest", help="不连板子，自检 CRC32/COBS/组帧").set_defaults(func=cmd_selftest)

    sub.add_parser("info", help="查看分区/版本/CRC 状态").set_defaults(func=cmd_info)
    sub.add_parser("monitor", help="当串口监视器，看设备日志").set_defaults(func=cmd_monitor)

    p = sub.add_parser("flash", parents=[flash_args], help="升级固件（自动挑槽、支持续传）")
    p.add_argument("file", help="build/Debug/motor_control.bin 或 motor_control_slotB.bin")
    p.set_defaults(func=cmd_flash, wait_boot=True)

    p = sub.add_parser("upgrade", parents=[flash_args],
                       help="一条命令升级：自动看设备在哪个槽，发另一个槽对应的 .bin（文件可省）")
    p.add_argument("file", nargs="?", default=None,
                   help="可省；不填就自动挑 motor_control.bin（A 槽）/ motor_control_slotB.bin（B 槽）")
    p.set_defaults(func=cmd_upgrade, wait_boot=True)

    sub.add_parser("rollback", help="切回另一个槽并重启").set_defaults(func=cmd_rollback)
    sub.add_parser("status", help="电机里程/位置/故障码").set_defaults(func=cmd_status)

    p = sub.add_parser("erase", help="擦除某个槽")
    p.add_argument("slot", choices=["A", "B", "a", "b"])
    p.set_defaults(func=cmd_erase)

    p = sub.add_parser("reboot", help="重启设备")
    p.add_argument("--boot", action="store_true", help="重启进 Bootloader 恢复台（救砖用）")
    p.set_defaults(func=cmd_reboot)

    p = sub.add_parser("ctrl", help="控制电机（不用 OTA 时这个口就是干这个的）")
    p.add_argument("cmd", choices=sorted(CTRL.keys()) + sorted(CTRL_ARG.keys()),
                   help="disable/enable/posloop/movepos/movedeg/stop/mute/pause/pwr/pwron/pwoff/pwcycle")
    p.add_argument("arg", nargs="?", type=lambda s: int(s, 0), default=None,
                   help="参数：movepos=0..32767、movedeg=0..359、mute/pause=0/1、pwr=0/1/2")
    p.set_defaults(func=cmd_ctrl)

    sub.add_parser("console", help="等同于 monitor").set_defaults(func=cmd_console)

    args = ap.parse_args()

    if args.action == "selftest":
        return cmd_selftest(None, args)

    if not args.port:
        ap.error("这个子命令需要 --port（只有当 selftest 不需要）")

    dev = Device(args.port, args.baud, quiet=args.quiet)
    try:
        args.func(dev, args)
    except OtaError as e:
        print(f"\n❌ {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print()
    finally:
        dev.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
