#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无线 OTA 上位机：只负责**固件升级**（通过无线串口 = USART1 那一路）。

电机命令在 `tools/motor.py`，小车在 `tools/car.py`。协议层（串口/COBS/CRC/组帧）在 `tools/proto.py`。

    pip install pyserial

用法（**串口不用指定**：系统上只有一个 USB 串口时脚本会自己用它，Windows 的 COM* 和
Linux 的 /dev/ttyACM*、/dev/ttyUSB* 都认；要换就 `--port COM7` 或设环境变量
CAR_PORT，写在子命令后面也行 —— `ota.py info --port COM7`）：

    python tools/ota.py selftest                 # 不连板子：自检 CRC32/COBS/组帧/镜像槽识别
    python tools/ota.py info                     # 看当前在哪个槽、版本、两个槽的 CRC
    python tools/ota.py upgrade                  # ★一条命令升级：自动挑槽 + 自动挑镜像
    python tools/ota.py flash build/Debug/motor_control_slotB.bin
    python tools/ota.py rollback                 # 新固件有问题 → 一键切回旧槽
    python tools/ota.py reboot --boot            # 重启进 Bootloader 恢复台（救砖）
    python tools/ota.py erase A                  # 擦掉某个槽
    python tools/ota.py monitor                  # 当串口监视器看设备日志

设计与取舍（A/B 双槽、异常处理）见 docs/ota_design.md；协议见它的 §5。
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time
import zlib

import proto
from proto import (SYNC, PROTO_VER, T_BEGIN, T_DATA, T_END, T_ERASE, T_REBOOT,
                   T_ROLLBACK, T_INFO, ST_OK, ST_OFFSET, ST_TEXT,
                   Device, OtaError, cobs_decode, cobs_encode, u16, u32)

# ---------------------------------------------------------------------------
# Flash 分区（要和 STM32H723xG_slots.ld 一致）
# ---------------------------------------------------------------------------
SLOT_A_BASE = 0x08020000
SLOT_B_BASE = 0x08080000
SLOT_SIZE = 384 * 1024

# 构建产物目录：按**脚本所在位置**算，不从当前目录算 ——
# 这样在仓库根目录、在 tools/ 里、或者从别的地方调用都不会找不到 .bin。
_HERE = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(os.path.dirname(_HERE), "build", "Debug")

# 两个槽对应的镜像文件名（同一份源码按不同基址链接出来的两份）
IMAGE_FOR_SLOT = {0: "motor_control.bin", 1: "motor_control_slotB.bin"}


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


def cmd_info(dev, args):
    print_info(dev.get_info())


def cmd_monitor(dev, args):
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


def image_path_for_slot(slot: int) -> str:
    """这个槽该用哪份 .bin"""
    return os.path.join(BUILD_DIR, IMAGE_FOR_SLOT[slot])


def pick_target_slot(info: dict, slot_arg: str = "auto") -> int:
    """目标槽 = 要写哪个槽。flash / upgrade 共用同一份规则，不会两边跑偏：
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


def cmd_flash(dev, args):
    image = open(args.file, "rb").read()
    if not image:
        raise OtaError("固件文件是空的")
    if len(image) > SLOT_SIZE:
        raise OtaError(f"固件 {len(image)} 字节超过一个槽（{SLOT_SIZE} 字节）")

    info = dev.get_info()
    in_bl = bool(info["flags"] & 0x80)          # 1 = 设备在 Bootloader 恢复台里
    running = info["active"] if info["active"] < 2 else 0

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

        pct = 100.0 * off / total
        speed = off / max(time.time() - t0, 1e-3)
        sys.stdout.write(f"\r  {off:>7}/{total} 字节  {pct:5.1f}%  {speed / 1024:5.1f} KB/s   ")
        sys.stdout.flush()

    print()
    rsp = dev.request(T_END, timeout=15.0, retries=3)
    if rsp is None:
        raise OtaError("END 没有回复（设备在把整个槽读回来算 CRC32，超时给够 15 s）")
    if rsp[0] != ST_OK:
        raise OtaError(f"END 校验失败：{ST_TEXT.get(rsp[0], rsp[0])}"
                       f"（设备读回算出的 crc32=0x{u32(rsp, 2):08X}，本地 0x{crc:08X}）")
    print(f"  校验通过：槽 {slot_name(rsp[1])} crc32=0x{u32(rsp, 2):08X}，"
          f"元数据已提交，设备正在重启 ...")

    if args.wait_boot:
        print("  等设备重启并切槽（最多 20 s）...")
        deadline = time.time() + 20
        while time.time() < deadline:
            time.sleep(0.2)          # 设备其实 1 s 内就起来了：轮询细一点，省掉白等
            try:
                info2 = dev.get_info(timeout=0.5, retries=1)
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


def cmd_upgrade(dev, args):
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

        # 顺手提醒一句：目标槽里已经是同一份镜像时不拦，但先说一声
        same = info["slot"][target]
        crc = zlib.crc32(open(path, "rb").read()) & 0xFFFFFFFF
        if same["size"] and same["crc"] == crc:
            print(f"  注意：槽 {slot_name(target)} 里已经是同一份镜像（crc32 一致）—— "
                  f"传一遍只是确认，源码没变的话结果不会变")

        args.file = path

    return cmd_flash(dev, args)


def cmd_rollback(dev, args):
    rsp = dev.request(T_ROLLBACK, timeout=5.0, retries=3)
    if rsp is None:
        raise OtaError("ROLLBACK 没有回复")
    if rsp[0] != ST_OK:
        raise OtaError(f"回滚失败：{ST_TEXT.get(rsp[0], rsp[0])}"
                       f"（另一个槽没有可用镜像？可以 reboot --boot 进恢复台重发）")
    print(f"已切到槽 {slot_name(rsp[1])} 并重启")


def cmd_erase(dev, args):
    info = dev.get_info()
    target = 0 if args.slot.upper() == "A" else 1
    if target == info["active"]:
        raise OtaError("不能擦正在运行的槽")
    rsp = dev.request(T_ERASE, bytes([target]), timeout=10.0, retries=3)
    if rsp is None or rsp[0] != ST_OK:
        raise OtaError(f"擦除失败：{ST_TEXT.get(rsp[0], rsp[0]) if rsp else '无回复'}")
    print(f"槽 {slot_name(target)} 已擦除")


def cmd_reboot(dev, args):
    mode = 1 if args.boot else 0
    dev.request(T_REBOOT, bytes([mode]), timeout=3.0, retries=3)
    print("已请求重启" + ("，并停在 Bootloader 恢复台（15 s 不动会自动尝试启动）" if mode else ""))


def cmd_console(dev, args):
    """等同于 monitor（老习惯的名字）。"""
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
    fake = Device.__new__(Device)          # 假对象：只为喂字节
    fake.quiet = True
    fake._reset_parser()
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
    ap = argparse.ArgumentParser(description="无线 OTA 上位机（只管固件升级）",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    proto.add_port_args(
        ap, baud_help="默认 921600（= 固件里的 OTA_PORT_BAUD，App 和 Bootloader 恢复台都是它）")

    sub = proto.add_subparsers(ap, dest="action", required=True)

    # flash / upgrade 共用的参数（upgrade 只是“文件可以不填”）
    flash_args = argparse.ArgumentParser(add_help=False)
    flash_args.add_argument("--slot", default="auto", help="auto（默认，写非活动槽）/ A / B")
    flash_args.add_argument("--chunk", type=int, default=4096,
                            help="每包字节数，默认 4096（取 min(设备上报的, 这个值)）")
    flash_args.add_argument("--retries", type=int, default=5, help="每一包的重传次数")
    flash_args.add_argument("--force", action="store_true", help="强制重新擦除，不续传")
    flash_args.add_argument("--version", type=lambda s: int(s, 0), default=None,
                            help="版本号，如 0x00010200")
    flash_args.add_argument("--no-wait-boot", dest="wait_boot", action="store_false",
                            help="升完不等设备重启确认（默认会等并打印新状态）")

    sub.add_parser("selftest", help="不连板子，自检 CRC32/COBS/组帧/镜像槽识别").set_defaults(
        func=cmd_selftest, needs_port=False)

    sub.add_parser("info", help="查看分区/版本/CRC 状态").set_defaults(func=cmd_info)
    sub.add_parser("monitor", help="当串口监视器，看设备日志").set_defaults(func=cmd_monitor)
    sub.add_parser("console", help="等同于 monitor").set_defaults(func=cmd_console)

    p = sub.add_parser("flash", parents=[flash_args], help="升级固件（自动挑槽、支持续传）")
    p.add_argument("file", help="build/Debug/motor_control.bin 或 motor_control_slotB.bin")
    p.set_defaults(func=cmd_flash, wait_boot=True)

    p = sub.add_parser("upgrade", parents=[flash_args],
                       help="一条命令升级：自动看设备在哪个槽，发另一个槽对应的 .bin（文件可省）")
    p.add_argument("file", nargs="?", default=None,
                   help="可省；不填就自动挑 motor_control.bin（A 槽）/ motor_control_slotB.bin（B 槽）")
    p.set_defaults(func=cmd_upgrade, wait_boot=True)

    sub.add_parser("rollback", help="切回另一个槽并重启").set_defaults(func=cmd_rollback)

    p = sub.add_parser("erase", help="擦除某个槽")
    p.add_argument("slot", choices=["A", "B", "a", "b"])
    p.set_defaults(func=cmd_erase)

    p = sub.add_parser("reboot", help="重启设备")
    p.add_argument("--boot", action="store_true", help="重启进 Bootloader 恢复台（救砖用）")
    p.set_defaults(func=cmd_reboot)

    args = ap.parse_args()
    return proto.serve(args, args.func, needs_port=getattr(args, "needs_port", True))


if __name__ == "__main__":
    sys.exit(main())
