#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电机控制上位机：给两台（同一条 USART10 总线，ID1/ID2）发命令 / 读状态。

只管电机本身。**固件升级**用 `tools/ota.py`；**开小车**（左右映射/差速/遥控）用 `tools/car.py`。

    python tools/motor.py status                       # 两台的里程/位置/故障码 + 电源 + 电压
    python tools/motor.py status --motor 1
    python tools/motor.py enable --motor 2             # 使能（会重试）
    python tools/motor.py speedloop --motor 1          # 切速度环（小车用这个环）
    python tools/motor.py speed 300 --motor 1          # 30.0rpm 正转（0.1rpm 为单位）
    python tools/motor.py accel 10 --motor 1           # 加速时间：每 1rpm 10ms
    python tools/motor.py posloop --motor 2            # 切位置环（转到位就停）
    python tools/motor.py movepos 8191 --motor 2       # 位置环走 90°
    python tools/motor.py stop                         # 急停（0x64 给定值 0）
    python tools/motor.py disable                      # 两台失能（不碰电源）
    python tools/motor.py pwoff / pwron / pwcycle       # 电机电源（PC14）：断电 / 上电 / 断电重启
    python tools/motor.py stacks                        # 每个任务的栈余量
    python tools/motor.py trim 2                        # 里程差闭环开，左轮=电机2

⚠ 失能（0xA0/0x09）只是让它不使劲，通信还在（里程/位置/故障码照样读）；
   电机电源（PC14）单独的开关是 pwoff / pwron，收工就两条都发。
⚠ 速度环下电机“给了转速就一直转”。子命令 = CTRL 表里的那些名字（`--help` 会列全）。
"""

from __future__ import annotations

import argparse
import struct
import sys

import proto
from proto import (T_CTRL, T_STATUS, ST_OK, ST_TEXT, N_MOTORS,
                   u16, u32, i32, pair_unpack)

# ---------------------------------------------------------------------------
# 和设备侧 OTA_CTRL_xxx（ota_layout.h）对齐：名字 → 命令码
# ---------------------------------------------------------------------------
CTRL = {
    "disable": 0x00, "enable": 0x01, "posloop": 0x02, "movepos": 0x03,
    "movedeg": 0x04, "stop": 0x05, "mute": 0x06, "pause": 0x07,
    "pwr": 0x08,          # 电机电源（PC14）：arg 0=断电 1=上电 2=断电重启
    "stacks": 0x0D,       # 打印每个任务的栈余量（明细在设备日志里）
    "speedloop": 0x0F,    # 切速度环（0xA0/0x02）并复核模式值真的是 0x02 —— 小车用这个环
    "speed": 0x10,        # 转速给定：单位 0.1rpm、带符号（±3800 ↔ ±380rpm），0 = 停
    "accel": 0x11,        # 加速度时间：每 1rpm 多少 ms（0..255，0 按 1 处理）
    "drive": 0x12,        # 两台一起走：arg = (电机2<<16)|电机1（各 0.1rpm）；--motor = 看门狗
    "trim": 0x13,         # 里程差闭环：arg = 0 关 / 1|2 开且这个数就是左轮的电机序号
}

# 带固定参数的快捷命令：名字 → (cmd, arg)
CTRL_ARG = {
    "pwron": (0x08, 1), "pwoff": (0x08, 0), "pwcycle": (0x08, 2),
}

CTRL_HELP = {
    "disable": "失能（不碰电源；不指定 --motor = 两台都失能）。断电用 pwoff",
    "enable": "使能（失败会重试）",
    "posloop": "切位置环（0xA0/0x03），“转到位就停”",
    "movepos": "位置环走位置：arg = 0..32767（0..360°）",
    "movedeg": "位置环走角度：arg = 0..359",
    "stop": "急停（0x64 给定值 0）",
    "mute": "静音/恢复日志：arg = 0/1",
    "pause": "暂停/恢复 200 ms 状态轮询：arg = 0/1",
    "pwr": "电机电源 PC14：arg = 0 断电 / 1 上电（等稳定）/ 2 断电重启",
    "stacks": "打印每个任务的栈余量（明细在设备日志里）",
    "speedloop": "切速度环（0xA0/0x02）并复核 —— 小车用这个环",
    "speed": "转速：arg = 0.1rpm、带符号（±3800 ↔ ±380rpm），0 = 停",
    "accel": "加速时间：每 1rpm 多少 ms（0..255）",
    "drive": "两台一起走：arg = (电机2<<16)|电机1（各 0.1rpm）；--motor = 看门狗（单位 50ms）",
    "trim": "里程差闭环：arg = 0 关 / 1|2 开（这个数就是左轮的电机序号）",
    "pwron": "= pwr 1：电机电源上电",
    "pwoff": "= pwr 0：电机电源断电",
    "pwcycle": "= pwr 2：断电重启",
}


# ---------------------------------------------------------------------------
def cmd_status(dev, args):
    """默认把总线上每一台都问一遍；--motor N 就只问那一台。"""
    motors = [args.motor] if args.motor else list(range(1, N_MOTORS + 1))
    pwr_note = ""
    volt_note = ""

    for mi in motors:
        rsp = dev.request(T_STATUS, bytes([mi]), timeout=3.0, retries=3)
        if rsp is None:
            raise proto.OtaError("STATUS 没有回复（设备在跑吗）")

        # 电机电源（PC14）和电源电压是后加到回帧末尾的：老固件没这些字节，所以先判长度。
        # ⚠ 这两项**跟电机答没答上没关系**，所以在判 status 之前就取出来：
        #   电机没上电（比如刚升级完，OTA 把 PC14 切了）时也该看得到电压。
        if (not pwr_note) and len(rsp) > 9:
            pwr_note = f"  电机电源={'ON' if rsp[9] else 'OFF'}"
        if (not volt_note) and len(rsp) > 11:
            mv = u16(rsp, 10)
            cells = rsp[12] if len(rsp) > 12 else 0
            flags = rsp[13] if len(rsp) > 13 else 0
            note = f"  电源电压={mv / 1000.0:.2f}V"
            pct = rsp[14] if len(rsp) > 14 else 0xFF
            if cells:
                note += f"（{cells}S，单片 {mv / cells / 1000.0:.2f}V"
                if pct <= 100:
                    note += f"，剩余≈{pct}%"
                if flags & 1:
                    note += "，⚠低压"
                elif flags & 2:
                    note += "，⚠过压"
                note += "）"
            if flags & 4:
                note += " ⚠读数无效"
            volt_note = note

        if rsp[0] != ST_OK:
            print(f"  电机{mi}: 没答上（{ST_TEXT.get(rsp[0], rsp[0])}）"
                  f"—— 没上电 / 没接线 / ID 不是 {mi}？{pwr_note}{volt_note}")
            continue

        print(f"  电机{mi}: 里程={i32(rsp, 1)} 圈  位置={u16(rsp, 5)}"
              f"（{u16(rsp, 5) * 360.0 / 32768:.1f}°）  故障码=0x{rsp[7]:02X}"
              f"  模式=0x{rsp[8]:02X}{pwr_note}{volt_note}")


def cmd_motor(dev, args):
    """所有电机命令的公共实现：按 args.cmd 查表 → 发一帧 → 打印回帧。"""
    if args.cmd in CTRL_ARG:
        cmd, arg = CTRL_ARG[args.cmd]
        if args.arg is not None:
            arg = args.arg              # 允许手动覆盖（比如 `pwr 2`）
    else:
        cmd = CTRL[args.cmd]
        # 默认参数：mute/pause 默认“开”(1)，其它 0
        arg = args.arg if args.arg is not None else (1 if args.cmd in ("mute", "pause") else 0)

    # 第 6 个字节 = 电机序号（1 起；不填 = 设备按“安全类命令管全部、运动类默认 1 号机”处理）。
    # drive 是个例外：那个字节被当成**看门狗超时**（单位 50ms）。
    # ⚠ arg 是**带符号**的（speed 反转就是负数），struct 的 "I" 装不下负数 → 先掩成 32 位。
    payload = struct.pack("<BI", cmd, arg & 0xFFFFFFFF) + (bytes([args.motor]) if args.motor else b"")
    who = f" 电机{args.motor}" if args.motor else ""

    rsp = dev.request(T_CTRL, payload, timeout=15.0, retries=2)
    if rsp is None:
        raise proto.OtaError(f"{args.cmd} 没有回复（使能会重试约 5 s，再等等）")

    data = u32(rsp, 1)
    data2 = u32(rsp, 5) if len(rsp) > 8 else 0   # 后加的第二个字段，老固件没有
    data3 = u32(rsp, 9) if len(rsp) > 12 else 0  # 第三个（trim 用：当前 trim 值）

    extra = ""
    if cmd == 0x00:                     # disable：只失能，不碰 PC14 电源
        extra = "  只失能（电机电源不变，断电用 pwoff）"
    elif cmd == 0x08:                   # 电源：data = 当前状态（0/1）
        extra = f"  电机电源={'ON' if data else 'OFF'}"
    elif cmd == 0x0D:                   # 栈余量：明细行已经随文本口回显在上面了
        extra = "  逐任务栈余量见上面 [stack] 行"
    elif cmd == 0x0F:                   # speedloop：data = 复核后的模式值（要 0x02）
        extra = (f"  模式=0x{data:02X}{'' if data == 0x02 else ' ⚠ 不是 0x02（没切成功）'}"
                 f"  总线 ID={data2}")
    elif cmd == 0x10:                   # speed：data = 生效转速（0.1rpm，带符号）
        extra = (f"  {i32(rsp, 1) / 10.0:+.1f} rpm"
                 f"（总线 ID{data2}{'，已停' if i32(rsp, 1) == 0 else ''}）")
    elif cmd == 0x11:                   # accel：data = 生效的加速时间（ms/rpm）
        extra = f"  加速时间={data}ms/rpm" + ("（按 1ms 处理）" if data == 0 else "")
    elif cmd == 0x12:                   # drive：data = 原样回两个基准，data2 = 看门狗 ms
        v1, v2 = pair_unpack(u32(rsp, 1))
        extra = (f"  电机1={v1 / 10.0:+.1f}rpm 电机2={v2 / 10.0:+.1f}rpm"
                 f"  看门狗={data2}ms" + ("（不启用）" if data2 == 0 else ""))
    elif cmd == 0x13:                   # trim：data = 0/1，data2 = 左轮序号，data3 = 当前 trim
        if data:
            extra = f"  里程差闭环=开  左轮=电机{data2}  当前trim={data3 / 10.0:+.1f}rpm"
        else:
            extra = "  里程差闭环=关"

    print(f"{args.cmd}({arg}){who} → {ST_TEXT.get(rsp[0], rsp[0])}  data={data}{extra}")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="电机控制上位机（两台直驱电机，共用一条 USART10）",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    proto.add_port_args(ap)

    sub = proto.add_subparsers(ap, dest="action", required=True)

    p = sub.add_parser("status", help="两台电机的里程/位置/故障码 + 电源 + 电压")
    p.add_argument("--motor", type=int, default=0, help=f"只看这一台（1..{N_MOTORS}）")
    p.set_defaults(func=cmd_status)

    # 每条电机命令一个子命令（名字/参数含义和 CTRL 表一致）
    shared = argparse.ArgumentParser(add_help=False)
    shared.add_argument("arg", nargs="?", type=lambda s: int(s, 0), default=None,
                        help="参数（含义见上面各子命令的说明；不填用默认值）")
    shared.add_argument("--motor", type=int, default=0,
                        help=f"作用于哪一台（1..{N_MOTORS}）；不填 = 设备默认"
                             f"（失能/急停管全部，使能/走位管 1 号机；drive 时它是看门狗）")

    for name in sorted(CTRL):
        sub.add_parser(name, parents=[shared], help=CTRL_HELP.get(name, "")).set_defaults(
            func=cmd_motor, cmd=name)

    for name in sorted(CTRL_ARG):
        sub.add_parser(name, parents=[shared], help=CTRL_HELP.get(name, "")).set_defaults(
            func=cmd_motor, cmd=name)

    args = ap.parse_args()
    return proto.serve(args, args.func)


if __name__ == "__main__":
    sys.exit(main())
