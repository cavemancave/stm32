#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两轮差速小车的命令行测试框架（跑在 `tools/ota.py` 的协议层之上）。

它只干一件事：**让你一条命令就让某一台电机按某个转速转**，先把台架摸清楚，
再谈小车控制。轮子用**速度环**（0x02）——为什么不是位置环/电流环，见
`README.md` 的「两轮差速小车（速度环）」一节和 `Device/Motor/inc/motor_ctrl.h` 里的说明。

串口默认 /dev/ttyACM0（Linux 的 USB-CDC）；CH340/CP210x 那类一般是 /dev/ttyUSB0，
用 `--port` 指定或设环境变量 CAR_PORT。

典型用法（每一步都单独跑，串口一次只开一个程序）：

    python tools/car.py setup                 # ① 上电 + 两台都使能 + 切速度环
    python tools/car.py status                #    看两台的模式/位置/里程/故障码
    python tools/car.py id                    # ② 识别左右轮（依次点动两台，问你是哪边动）
    python tools/car.py spin --motor 1 --rpm 20 --seconds 2    # ③ 单轮点动
    python tools/car.py drive --left 20 --right 20 --seconds 3  # ④ 两轮一起（按 id 存的映射）
    python tools/car.py stop                  # ⑤ 停车（--disable 连使能/电源一起切）
    python tools/car.py watch --seconds 5     #    看两台实际转速（由编码器算出来的）

⚠ 安全：速度环下电机**一旦给了转速就会一直转**（不像位置环走到位自己停）。
   所以每个动作都是"给转速 → 等一会 → 一定给 0 停下"（Ctrl+C 也会在 finally 里停），
   但**手别离开电源开关**，第一次测试先把轮子架空。跑远之前想加"断联停"再说
   （需要固件周期性重发转速 + 电机开 0x10 断联功能）。
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ota      # noqa: E402  （复用 ota.py 里的 Device / COBS / CRC / 端序那几个小工具）

# ---- 协议里的 CTRL 子命令（要和 Device/Ota/inc/ota_layout.h 对齐）----------
CTRL_DISABLE    = 0x00
CTRL_ENABLE     = 0x01
CTRL_STOP       = 0x05
CTRL_PWR        = 0x08
CTRL_SPEED_LOOP = 0x0F
CTRL_SPEED      = 0x10
CTRL_ACCEL      = 0x11
CTRL_DRIVE      = 0x12      # 两台一起走（设备侧背靠背发两条 0x64）+ 看门狗
CTRL_TRIM       = 0x13      # 里程差闭环：arg = 0 关 / 1|2 开且这个数就是左轮

MODE_NAMES = {0x00: "开环", 0x01: "电流环", 0x02: "速度环", 0x03: "位置环"}

# 速度环给定值的量纲：±3800 ↔ ±380rpm（1 单位 = 0.1rpm）
RPM_MAX = 380.0

# "哪台电机是左轮"这个小配置：id 命令会写它，drive 命令读它
MAP_PATH = os.path.join(HERE, "car_map.json")
DEFAULT_MAP = {"left": 1, "right": 2, "invert_left": False, "invert_right": False}


# ---------------------------------------------------------------------------
# 收发（薄薄一层，把 struct.pack 和回帧解析收在一处）
# ---------------------------------------------------------------------------
def ctrl(dev, cmd, arg=0, motor=0, timeout=8.0):
    """发一条 CTRL 命令，返回 (status, data, data2)。

    motor = 0 表示"不指定"：设备侧对安全类命令（失能/急停）作用于全部电机，
    对运动类命令默认作用于 1 号机 —— 两台车上的命令一定要显式给 --motor。
    """
    payload = struct.pack("<BI", cmd, arg & 0xFFFFFFFF) + (bytes([motor]) if motor else b"")
    rsp = dev.request(ota.T_CTRL, payload, timeout=timeout, retries=2)

    if rsp is None:
        raise ota.OtaError(f"ctrl cmd={cmd:#04x} 没有回复（设备在跑吗？串口对不对？）")

    data = ota.i32(rsp, 1)                      # 带符号：转速会是负数
    data2 = ota.u32(rsp, 5) if len(rsp) > 8 else 0
    return rsp[0], data, data2


def read_status(dev, motor):
    """读一台电机的状态：里程 / 位置 / 故障码 / 当前模式。"""
    rsp = dev.request(ota.T_STATUS, bytes([motor]), timeout=3.0, retries=3)

    if rsp is None:
        raise ota.OtaError("STATUS 没有回复（设备在跑吗）")
    if rsp[0] != ota.ST_OK:
        return None                 # 这台没答上（没上电 / 没接线 / ID 不是它）

    pos = ota.u16(rsp, 5)
    return {
        "mileage": ota.i32(rsp, 1),
        "pos": pos,
        "deg": pos * 360.0 / 32768.0,
        "fault": rsp[7],
        "mode": rsp[8],
        "pwr": rsp[9] if len(rsp) > 9 else 0,
    }


# ---------------------------------------------------------------------------
# 常用动作
# ---------------------------------------------------------------------------
def power_on(dev):
    """电机电源（PC14）上电：设备会等 500 ms 让电源轨稳定。"""
    ctrl(dev, CTRL_PWR, 1, timeout=6.0)


def ensure_speed_loop(dev, motor, accel=None):
    """使能 + 切速度环。设备侧切完会自己用 0x75/0x76 复核模式值，我们再看回帧里的值。

    ⚠ 超时给得宽（设备侧"上电等 500ms + 使能重试最多 5s"都在这条命令里），
      但那只在"电机没上电/没接线"时才真的等满 —— 正常情况一发就回。
    """
    st, mode, bus_id = ctrl(dev, CTRL_SPEED_LOOP, 0, motor, timeout=25.0)

    if st != ota.ST_OK or mode != 0x02:
        raise ota.OtaError(
            f"电机{motor} 切速度环失败：status={ota.ST_TEXT.get(st, st)}、"
            f"回帧模式={mode:#04x}（要 0x02）。\n"
            f"    先 `car.py status` 看它答不答得上；不答就是没上电/没接线/ID 不对。")

    if accel is not None:
        set_accel(dev, motor, accel)

    return bus_id


def set_accel(dev, motor, ms_per_rpm):
    """速度环加速时间：每 1rpm 多少 ms（0 按 1 处理）。越大起步越柔。"""
    st, data, _ = ctrl(dev, CTRL_ACCEL, int(ms_per_rpm), motor)

    if st != ota.ST_OK:
        raise ota.OtaError(f"电机{motor} 设加速时间失败（{ota.ST_TEXT.get(st, st)}）")

    return data


def set_speed(dev, motor, rpm):
    """给一台电机一个转速（rpm，带符号）。

    设备侧发 0x64 之前会自己确认它在速度环，不在就自动切过去 —— 因为同一个数值
    在电流环里是电流、在位置环里是目标位置，"停车"给错环甚至会让轮子转回 0°。
    """
    if abs(rpm) > RPM_MAX:
        raise ota.OtaError(f"转速 {rpm} rpm 超范围（±{RPM_MAX:.0f} rpm）")

    st, data, _ = ctrl(dev, CTRL_SPEED, int(round(rpm * 10)), motor, timeout=15.0)

    if st != ota.ST_OK:
        raise ota.OtaError(f"电机{motor} 给转速失败（{ota.ST_TEXT.get(st, st)}）")

    return data / 10.0


def stop_all(dev):
    """两台都给 0 转速（速度环下 0 = 停）。尽量都试一遍，别因为一台失败就不管另一台。"""
    for motor in (1, 2):
        try:
            set_speed(dev, motor, 0)
            print(f"  电机{motor}: 已停")
        except ota.OtaError as e:
            print(f"  ⚠ 电机{motor} 停车失败：{e}")


def set_speeds_together(dev, speeds, watchdog_ms=0, quiet=False):
    """{电机号: rpm} → **一条** DRIVE 命令（设备侧背靠背发两条 0x64）。

    为什么不用两次 ctrl speed：那条路每台都要"无线往返一次 + 先查一次模式"，
    而且两条 0x64 中间夹着上位机的循环 —— 实测两台起步能差 30~80ms，
    低速时就是"先动的那侧把车拽歪一下"。合成一条之后只剩总线串行的 ~10ms。

    watchdog_ms > 0：设备自己兜底 —— 超过这么久没收到新的 DRIVE 就把两台停掉
    （松手/断线/上位机被强杀都会停，不用指望最后那条 stop 命令发得出去）。
    quiet=True：成功时不打那行摘要（遥控 20 Hz 时用，否则刷屏）。
    """
    raw = {}

    for motor, rpm in speeds.items():
        if abs(rpm) > RPM_MAX:
            raise ota.OtaError(f"电机{motor} 转速 {rpm} rpm 超范围（±{RPM_MAX:.0f} rpm）")
        raw[int(motor)] = int(round(rpm * 10))

    arg = ota.pair_pack(raw.get(1, 0), raw.get(2, 0))
    wd_units = 0 if watchdog_ms <= 0 else max(1, min(255, int(watchdog_ms) // 50))

    st, data, data2 = ctrl(dev, CTRL_DRIVE, arg, wd_units, timeout=20.0)

    if st == ota.ST_PARAM:
        # 老固件没有这条命令：退回"一台一台发"（起步差回来，但至少能动）
        print("  ⚠ 固件不认 DRIVE（旧版本？）—— 退回一台一台发，起步会差一点")
        for motor, rpm in speeds.items():
            set_speed(dev, motor, rpm)
        return

    if st != ota.ST_OK:
        raise ota.OtaError(f"DRIVE 失败：{ota.ST_TEXT.get(st, st)}")

    if not quiet:
        v1, v2 = ota.pair_unpack(data)
        print(f"  一起走：电机1={v1 / 10.0:+.1f} 电机2={v2 / 10.0:+.1f} rpm"
              f"（看门狗 {data2}ms{'，不启用' if data2 == 0 else ''}）")


def ensure_trim(dev, left_motor):
    """告诉设备"哪台是左轮"：里程差闭环靠它才能知道往哪边补。

    很便宜（一条命令、不碰电机），所以 setup/drive/teleop 每次都先发一遍 ——
    免得"换了车/改了 car_map.json，设备里还是老的左轮"这种静默错误（那种情况下车
    会越走越歪，而且没人报错）。
    （闭环只在"直行类"动作下积分，转圈/走弧设备自己会关，所以这里不用管动作。）
    返回 True = 配上了。
    """
    st, on, left = ctrl(dev, CTRL_TRIM, int(left_motor), 0, timeout=6.0)

    if (st != ota.ST_OK) or (on != 1) or (left != int(left_motor)):
        print(f"  ⚠ 里程差闭环没配上（status={ota.ST_TEXT.get(st, st)}，状态={on}，左轮={left}）"
              f" —— 车还能走，但不会自己修直线")
        return False

    print(f"  里程差闭环：左轮=电机{left}（只在直行类动作下生效）")
    return True


def hold(dev, speeds, seconds):
    """按 {电机号: rpm} 跑 seconds 秒，**无论如何**（包括 Ctrl+C）最后都停下来。

    两台一起动时走**一条** DRIVE 命令（起步差 ~10ms）；只动一台时用 ctrl speed。
    seconds <= 0 = 一直跑，直到 Ctrl+C（那种情况不给看门狗：本来就要人看着）。
    """
    if not speeds:
        return

    # 看门狗 = 本段时长 + 1.5s：正常跑完会自己发停；上位机挂了/断线了设备也能自己停
    wd_ms = 0 if seconds <= 0 else int(seconds * 1000) + 1500
    started = {}

    try:
        if len(speeds) > 1:
            set_speeds_together(dev, speeds, wd_ms)
            started = dict(speeds)
        else:
            for motor, rpm in speeds.items():
                started[motor] = set_speed(dev, motor, rpm)
                print(f"  电机{motor}: {started[motor]:+.1f} rpm")

        end = None if seconds <= 0 else time.time() + seconds

        if end is None:
            print("  一直转（--seconds 0）：按 Ctrl+C 停（会先停车再退出）")

        try:
            while end is None or time.time() < end:
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n  Ctrl+C：停车")
    finally:
        # 停车也尽量走一条命令（两台一起 + 取消看门狗）
        if started:
            try:
                if len(started) > 1:
                    set_speeds_together(dev, {m: 0.0 for m in started}, 0, quiet=True)
                    print("  两台已停（一条命令，两台一起）")
                else:
                    for motor in started:
                        set_speed(dev, motor, 0)
                    print(f"  电机{list(started)[0]}: 已停")
            except ota.OtaError as e:
                print(f"  ⚠ 停车失败：{e}")


# ---------------------------------------------------------------------------
# 左右轮映射（id 命令写、drive 命令读）
# ---------------------------------------------------------------------------
def load_map():
    try:
        with open(MAP_PATH, "r", encoding="utf-8") as f:
            m = dict(DEFAULT_MAP)
            m.update(json.load(f))
            return m
    except FileNotFoundError:
        return dict(DEFAULT_MAP)
    except Exception as e:
        print(f"⚠ 读 {MAP_PATH} 失败（{e}），先用默认 {DEFAULT_MAP}")
        return dict(DEFAULT_MAP)


def save_map(m):
    with open(MAP_PATH, "w", encoding="utf-8") as f:
        json.dump(m, f, ensure_ascii=False, indent=2)
        f.write("\n")


def wheel_speeds(rpm_left, rpm_right, m):
    """(左轮 rpm, 右轮 rpm) → {电机号: rpm}，顺带处理左右定义和"反了"（invert）。

    差速底盘就两个自由度：
      直行   left = right（同号同值）
      原地转 left = -right
      转弯   left ≠ right（值越大那侧越快）
    """
    l_motor, r_motor = int(m["left"]), int(m["right"])
    l_rpm = -rpm_left if m.get("invert_left") else rpm_left
    r_rpm = -rpm_right if m.get("invert_right") else rpm_right
    return {l_motor: l_rpm, r_motor: r_rpm}


# ---------------------------------------------------------------------------
# 子命令
# ---------------------------------------------------------------------------
def cmd_setup(dev, args):
    """上电 + 两台都使能并切到速度环（小车开跑前的"点火"）。"""
    print("① 电机电源上电（PC14）…")
    power_on(dev)

    print(f"② 两台使能 + 切速度环{'，加速时间 %d ms/rpm' % args.accel if args.accel is not None else ''}…")
    for motor in range(1, ota.N_MOTORS + 1):
        bus_id = ensure_speed_loop(dev, motor, args.accel)
        print(f"  电机{motor}（总线 ID{bus_id}）：速度环 OK")

    print("③ 里程差闭环（走直线用）：把左轮告诉设备")
    ensure_trim(dev, load_map()["left"])

    cmd_status(dev, args)


def cmd_status(dev, args):
    """看两台电机：在哪台、什么模式、位置/里程/故障码。"""
    motor_arg = getattr(args, "motor", 0)
    motors = [motor_arg] if motor_arg else list(range(1, ota.N_MOTORS + 1))

    for motor in motors:
        st = read_status(dev, motor)

        if st is None:
            print(f"  电机{motor}: ❌ 没答上（没上电 / 没接线 / ID 不是它？）")
            continue

        mode = MODE_NAMES.get(st["mode"], f"0x{st['mode']:02X}")
        print(f"  电机{motor}: 模式={mode}  位置={st['pos']}（{st['deg']:.1f}°）"
              f"  里程={st['mileage']} 圈  故障码=0x{st['fault']:02X}"
              f"  电机电源={'ON' if st['pwr'] else 'OFF'}")


def cmd_spin(dev, args):
    """单轮点动：只动一台，用来确认接线/方向/转速对不对。"""
    print(f"电机{args.motor} 转 {args.rpm:+.1f} rpm，持续 {args.seconds:.1f}s"
          f"（Ctrl+C 可提前停）")

    before = read_status(dev, args.motor)
    hold(dev, {args.motor: args.rpm}, args.seconds)
    after = read_status(dev, args.motor)

    if before and after:
        turns = after["mileage"] - before["mileage"]
        print(f"  里程变化 {turns:+d} 圈，位置 {before['deg']:.1f}° → {after['deg']:.1f}°"
              f"（正 rpm 时位置/里程该往正方向走；反了就是这个电机的相序/映射要反）")


def pos_delta(p_from, p_to):
    """两个位置值之间"实际转了多少计数"：位置一圈是 0~32767 且 32767 与 0 是同一点，
    所以按**最短路径**折一下（和固件里 motor_drive.c 用的是同一套算法）。"""
    d = p_to - p_from

    if d > 16384:
        d -= 32768
    elif d < -16384:
        d += 32768

    return d


def cmd_drive(dev, args):
    """两轮一起跑（按 id 存的左右映射）。直行就给一样的值，原地转就给一正一负。

    两台走的是**同一条** DRIVE 命令（起步差 ~10ms）；收尾会把两台各自走了多少计数打出来
    —— 两台之差就是"这一段歪了多少"（差值大就说明闭环没开好或者轮径差太大）。
    """
    m = load_map()
    speeds = wheel_speeds(args.left, args.right, m)

    print(f"左轮={args.left:+.1f} rpm、右轮={args.right:+.1f} rpm"
          f"（映射：左=电机{m['left']}{'（反向）' if m.get('invert_left') else ''}、"
          f"右=电机{m['right']}{'（反向）' if m.get('invert_right') else ''}）")

    if len(speeds) == 1:
        print("  ⚠ 左右映射指向了同一台电机！先跑 `car.py id` 或手改 "
              f"{os.path.basename(MAP_PATH)}")

    ensure_trim(dev, m["left"])

    before = {mm: read_status(dev, mm) for mm in speeds}
    hold(dev, speeds, args.seconds)
    after = {mm: read_status(dev, mm) for mm in speeds}

    moved = {}
    for mm in speeds:
        if before.get(mm) and after.get(mm):
            moved[mm] = pos_delta(before[mm]["pos"], after[mm]["pos"])

    if len(moved) == len(speeds) and moved:
        for mm, d in moved.items():
            print(f"  电机{mm}: 走了 {d:+d} 计数（{d * 360.0 / 32768:+.1f}°）")

        if len(moved) == 2:
            d1, d2 = moved[m["left"]], moved[m["right"]]
            diff = d1 - d2
            base = max(abs(d1), abs(d2))
            print(f"  左−右 = {diff:+d} 计数（{diff * 360.0 / 32768:+.2f}°）"
                  f"：这就是这一段的航向漂移量（除上轴距就是车头转过的角度）")

            if base and abs(diff) > base * 0.05:
                print("  ⚠ 两台差超过 5%：先看 `setup` 的闭环配上没有、"
                      "再考虑轮径/地面是不是差太多")


def cmd_stop(dev, args):
    """停车。默认只给 0 转速（保持在速度环里，还能马上再开）；
    --disable 会连失能 + 断电机电源一起做（收工/要动线的时候用）。"""
    print("停车：")
    stop_all(dev)

    if args.disable:
        print("失能 + 断电：")
        st, _, _ = ctrl(dev, CTRL_DISABLE, 0, 0, timeout=10.0)
        print(f"  {'OK' if st == ota.ST_OK else ota.ST_TEXT.get(st, st)}")
        print("  ⚠ 下次要动之前先 `car.py setup`（或 ctrl pwron + enable）")


def cmd_id(dev, args):
    """识别哪台电机是左轮、哪一侧要反向：**逐台**点动、逐台提问。

    「左」= **站在车尾、朝车头方向看**时的左手边（驾驶员的左手边）。
    如果轮子已经装上、车已落地：先把车架空（或垫起来），不然它会跑。

    ⚠ 必须**逐台**问：实测见过 电机1 → 右轮、电机2 → 左轮，而且两台的转向还相反
      （两台电机是镜像装的）。所以「只问一次哪边动了」这种问法根本表达不了。
    """
    print("这一步会依次点动两台电机，请盯着车看**哪一侧的轮子**在转。\n")

    # ⚠ 0 在 spin/drive 里是「一直转」—— 在这条命令里没有意义，只会看起来像卡住
    if args.seconds <= 0:
        raise ota.OtaError("id 的 --seconds 必须 > 0（0 = 一直转，只对 spin/drive 有意义）")

    print("先上电 + 两台都切到速度环：")
    power_on(dev)
    for motor in range(1, ota.N_MOTORS + 1):
        ensure_speed_loop(dev, motor)
        print(f"  电机{motor}: 速度环 OK")
    print()

    for motor in range(1, ota.N_MOTORS + 1):
        print(f"===== 现在动 **电机{motor}**（总线 ID{motor}）：{args.rpm:+.0f} rpm "
              f"持续 {args.seconds:.1f}s =====")
        hold(dev, {motor: args.rpm}, args.seconds)
        time.sleep(1.0)         # 留一点空档，好区分"哪一段是哪台"

    m = load_map()

    manual = ("想手动记（或者答错了要改）：\n"
              "    python tools/car.py map --left <左轮电机号> --right <右轮电机号>"
              " [--invert-left] [--invert-right]\n"
              "    （--invert-xxx = 那侧给正 rpm 是把车往后推的；要取消反向用 --no-invert-xxx）")

    if not sys.stdin.isatty() or args.no_input:
        print(f"\n（非交互环境，跳过提问）刚才那两段就是答案，照着记：\n{manual}")
        return

    print("\n下面**逐台**问（两台接的位置/方向经常不一样，所以不能只问一次）：")
    print("  视角统一：**站在车尾、朝车头方向看** —— 你的左手边就是「左轮」。")

    answers = {}        # 电机号 → ("l"/"r", "f"/"b")

    try:
        for motor in range(1, ota.N_MOTORS + 1):
            side = ask(f"电机{motor}（{args.rpm:+.0f}rpm）转的时候，是哪一侧的轮子在转？"
                       f" [l]左边 / [r]右边 / [s]没看清", ("l", "r", "s"))

            if side == "s":
                continue

            direction = ask("那侧轮子是往前还是往后转？（「往前」= 车头那个方向）"
                            " [f]往前 / [b]往后", ("f", "b"))

            answers[motor] = (side, direction)
    except (EOFError, KeyboardInterrupt):
        print(f"\n（没答完，映射不变）\n{manual}")
        return

    lefts = [n for n, (s, _) in answers.items() if s == "l"]
    rights = [n for n, (s, _) in answers.items() if s == "r"]

    if (len(lefts) != 1) or (len(rights) != 1):
        print(f"\n⚠ 这次答的没法直接记下来（左边={lefts}、右边={rights}）："
              f"要么两侧都答成了同一侧（接线/轮子是不是有问题？），要么有一台没答。")
        print(manual)
        return

    m["left"], m["right"] = lefts[0], rights[0]
    # "invert" 的含义 = 这台电机**给正 rpm 时把车往后推** → 记下来，之后 drive 会自动取反
    m["invert_left"] = (answers[lefts[0]][1] == "b")
    m["invert_right"] = (answers[rights[0]][1] == "b")
    save_map(m)

    print(f"\n已记下 → {MAP_PATH}")
    print(f"  左轮 = 电机{m['left']}"
          f"{'（正 rpm 是往后的 → 已记成反向）' if m['invert_left'] else ''}")
    print(f"  右轮 = 电机{m['right']}"
          f"{'（正 rpm 是往后的 → 已记成反向）' if m['invert_right'] else ''}")
    print(f"  {json.dumps(m, ensure_ascii=False)}")
    print("\n验证两条（车架空，或者留出两米）：\n"
          "    python tools/car.py drive --left 20 --right 20 --seconds 1.5   "
          "# 该往**车头**方向直走\n"
          "    python tools/car.py drive --left 20 --right -20 --seconds 1.5  "
          "# 该**原地左转**（车头转向左边）")
    print(manual)


def ask(prompt, allowed):
    """问一个问题，只接受 allowed 里的答案（大小写不敏感）。"""
    while True:
        ans = input(f"{prompt} : ").strip().lower()

        if ans in allowed:
            return ans

        print(f"    只能答 {'/'.join(allowed)}")


def cmd_watch(dev, args):
    """按固定间隔读两台的状态，并用**编码器**算出实际转速（验证速度环跟得上没有）。

    转速是拿 (里程 x 一圈 + 位置) 的差分算的：位置一圈是 0~32767、到顶会翻回 0
    （同时里程 +1），所以差分要按"最短路径"折一下。
    """
    prev = {}
    t0 = time.time()

    print(f"每 {args.interval:.2f}s 读一次（Ctrl+C 停）。转速由编码器差分算出来，"
          f"和 `ctrl speed` 给的值对比着看")

    try:
        while True:
            elapsed = time.time() - t0

            for motor in range(1, ota.N_MOTORS + 1):
                st = read_status(dev, motor)

                if st is None:
                    print(f"[{elapsed:6.1f}s] 电机{motor}: ❌ 没答上")
                    prev.pop(motor, None)
                    continue

                angle = st["mileage"] * 32768 + st["pos"]      # 连续角度（计数）
                rpm_txt = "   ---  "

                if motor in prev:
                    prev_t, prev_angle = prev[motor]
                    dt = time.time() - prev_t
                    da = angle - prev_angle

                    # 位置在一圈内翻转过（0 ↔ 32767）时差分会有 ±32768 的假跳
                    if da > 16384:
                        da -= 32768
                    elif da < -16384:
                        da += 32768

                    if dt > 0:
                        rpm_txt = f"{da / 32768.0 / dt * 60.0:+7.1f}"

                prev[motor] = (time.time(), angle)
                mode = MODE_NAMES.get(st["mode"], f"0x{st['mode']:02X}")
                print(f"[{elapsed:6.1f}s] 电机{motor}: {rpm_txt} rpm   "
                      f"位置={st['pos']:5d}（{st['deg']:5.1f}°）  里程={st['mileage']:+3d}  "
                      f"模式={mode}  故障码=0x{st['fault']:02X}")

            print()

            if args.seconds and elapsed + args.interval > args.seconds:
                break

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("停止观察（电机没有被停，要停请 `car.py stop`）")


def cmd_map(dev, args):
    """看/改左右映射（不想跑 id 的时候直接手写）。"""
    m = load_map()

    if args.left is not None:
        m["left"] = args.left
    if args.right is not None:
        m["right"] = args.right
    if args.invert_left is not None:
        m["invert_left"] = args.invert_left
    if args.invert_right is not None:
        m["invert_right"] = args.invert_right

    if any(v is not None for v in (args.left, args.right, args.invert_left, args.invert_right)):
        if m["left"] == m["right"]:
            raise ota.OtaError("左轮和右轮不能是同一台电机（left == right）")
        save_map(m)
        print(f"已更新 {MAP_PATH}")

    print(f"左轮 = 电机{m['left']}"
          f"{'（反向）' if m.get('invert_left') else ''}，"
          f"右轮 = 电机{m['right']}"
          f"{'（反向）' if m.get('invert_right') else ''}")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="两轮差速小车测试框架（速度环）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--port", default=ota.DEFAULT_PORT,
                    help=f"电脑这边的串口，默认 {ota.DEFAULT_PORT}（也可以设环境变量 CAR_PORT）")
    ap.add_argument("--baud", type=int, default=921600)

    sub = ap.add_subparsers(dest="action", required=True)

    p = sub.add_parser("setup", aliases=["scan"], help="上电 + 两台使能 + 切速度环")
    p.add_argument("--accel", type=int, default=None,
                   help="顺便设加速时间（每 1rpm 多少 ms，0..255）；不填就不动")
    p.set_defaults(func=cmd_setup)

    p = sub.add_parser("status", help="两台电机的模式/位置/里程/故障码")
    p.add_argument("--motor", type=int, default=0, help="只看这一台（1/2）")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("id", help="识别哪台是左轮（依次点动两台，然后逐台问你）")
    p.add_argument("--rpm", type=float, default=15.0, help="点动转速 rpm（默认 15）")
    p.add_argument("--seconds", type=float, default=1.5, help="每台转多久（默认 1.5s）")
    p.add_argument("--no-input", action="store_true", help="不提问，只转（脚本里用）")
    p.set_defaults(func=cmd_id)

    p = sub.add_parser("spin", help="单轮点动（只动一台）")
    p.add_argument("--motor", type=int, required=True, choices=range(1, ota.N_MOTORS + 1),
                   help="动哪一台（1/2）")
    p.add_argument("--rpm", type=float, default=15.0, help="转速 rpm，带符号（默认 15）")
    p.add_argument("--seconds", type=float, default=2.0, help="转多久（默认 2s；0 = 一直转到 Ctrl+C）")
    p.set_defaults(func=cmd_spin)

    p = sub.add_parser("drive", help="两轮一起跑（按 id 存的映射）")
    p.add_argument("--left", type=float, default=0.0, help="左轮 rpm（带符号）")
    p.add_argument("--right", type=float, default=0.0, help="右轮 rpm（带符号）")
    p.add_argument("--seconds", type=float, default=2.0, help="跑多久（默认 2s；0 = 一直转到 Ctrl+C）")
    p.set_defaults(func=cmd_drive)

    p = sub.add_parser("stop", help="停车（--disable 连失能 + 断电）")
    p.add_argument("--disable", action="store_true", help="顺带失能两台并断开电机电源")
    p.set_defaults(func=cmd_stop)

    p = sub.add_parser("watch", help="看两台的实际转速（编码器差分算的）")
    p.add_argument("--interval", type=float, default=0.2, help="采样间隔秒（默认 0.2）")
    p.add_argument("--seconds", type=float, default=0.0, help="看多久（默认 = 一直看到 Ctrl+C）")
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser("map", help="看/改左右轮映射（不跑 id 时的备用手段）")
    p.add_argument("--left", type=int, choices=(1, 2), default=None)
    p.add_argument("--right", type=int, choices=(1, 2), default=None)
    p.add_argument("--invert-left", dest="invert_left", action="store_true", default=None,
                   help="左轮：这台电机给正 rpm 时是把车往后推的（之后 drive 会取反）")
    p.add_argument("--no-invert-left", dest="invert_left", action="store_false", default=None,
                   help="左轮：取消反向")
    p.add_argument("--invert-right", dest="invert_right", action="store_true", default=None,
                   help="右轮：同上")
    p.add_argument("--no-invert-right", dest="invert_right", action="store_false", default=None,
                   help="右轮：取消反向")
    p.set_defaults(func=cmd_map)

    args = ap.parse_args()

    # 只有要碰电机的子命令才需要串口；map 是纯本地的小配置
    if args.action == "map":
        try:
            return cmd_map(None, args) or 0
        except ota.OtaError as e:
            print(f"\n❌ {e}", file=sys.stderr)
            return 1

    try:
        ota.check_port(args.port)
        dev = ota.Device(args.port, args.baud, quiet=False)
    except ota.OtaError as e:
        print(f"\n❌ {e}", file=sys.stderr)
        return 1
    except Exception as e:                      # 权限 / 被占用 / 拔线
        print(f"\n❌ 打不开 {args.port}：{e}", file=sys.stderr)
        return 1

    rc = 0
    try:
        args.func(dev, args)
    except ota.OtaError as e:
        print(f"\n❌ {e}", file=sys.stderr)
        rc = 1
    except KeyboardInterrupt:
        print("\n中断。⚠ 电机可能还在转 —— 跑一次 `car.py stop` 或按板子上的复位")
        rc = 1
    except EOFError:
        print("\n❌ 读不到输入（终端不是交互式的，或者输入被关了）。\n"
              "   要脚本化就用 `car.py id --no-input`，或者直接 `car.py map ...` 写映射。",
              file=sys.stderr)
        rc = 1
    finally:
        dev.close()

    return rc


if __name__ == "__main__":
    sys.exit(main())
