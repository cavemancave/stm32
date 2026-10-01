#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两轮差速小车上位机：左右轮识别 / 差速走 / 看门狗 / 键盘遥控，一个脚本全包。

轮子用**速度环**（0x02）——为什么不是位置环，见 docs/car.md。
协议层在 `tools/proto.py`；**固件升级**用 `tools/ota.py`；**单台电机**用 `tools/motor.py`。

串口**不用指定**：系统上只有一个 USB 串口时脚本会自己用它（Windows 的 COM* 和 Linux
的 /dev/ttyACM*、/dev/ttyUSB* 都认）。要换就 `--port COM7`（写在子命令后面也行，
如 `car.py status --port COM7`），或者设环境变量 CAR_PORT。

典型用法（每一步都单独跑，串口一次只开一个程序）：

    python tools/car.py setup                 # ① 上电 + 两台使能 + 切速度环 + 配闭环
    python tools/car.py status                #    看两台的模式/位置/里程/故障码/电源
    python tools/car.py id                    # ② 识别左右轮（依次点动两台，问你是哪边动）
    python tools/car.py spin --motor 1 --rpm 20 --seconds 2    # ③ 单轮点动
    python tools/car.py drive --left 20 --right 20 --seconds 3  # ④ 两轮一起（按 id 存的映射）
    python tools/car.py teleop --speed 30     # ⑤ 键盘遥控：按住就走、松手就停
    python tools/car.py stop --disable        # ⑥ 收工：停车 + 失能（断电另发 motor.py pwoff）

⚠ 安全：速度环下电机**一旦给了转速就会一直转**（不像位置环走到位自己停）。
   所以每个动作都是"给转速 → 等一会 → 一定给 0 停下"（Ctrl+C 也会在 finally 里停），
   但**手别离开电源开关**，第一次测试先把轮子架空。
"""

from __future__ import annotations

import argparse
import json
import os
import select
import struct
import sys
import time

try:                                # teleop 才需要；Windows 上没有这两个模块
    import termios
    import tty
except ImportError:                 # pragma: no cover
    termios = tty = None

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import proto    # noqa: E402  （协议层：Device / COBS / CRC / 端序 / 帧类型）

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

# "哪台电机是左轮"这个小配置：id 命令会写它，drive/teleop 读它
MAP_PATH = os.path.join(HERE, "car_map.json")
DEFAULT_MAP = {"left": 1, "right": 2, "invert_left": False, "invert_right": False}


# ---------------------------------------------------------------------------
# 收发（薄薄一层，把 struct.pack 和回帧解析收在一处）
# ---------------------------------------------------------------------------
def ctrl(dev, cmd, arg=0, motor=0, timeout=8.0):
    """发一条 CTRL 命令，返回 (status, data, data2)。

    motor = 0 表示“不指定”：设备侧对安全类命令（失能/急停）作用于全部电机，
    对运动类命令默认作用于 1 号机 —— 车上的命令一定要显式给电机号。
    """
    payload = struct.pack("<BI", cmd, arg & 0xFFFFFFFF) + (bytes([motor]) if motor else b"")
    rsp = dev.request(proto.T_CTRL, payload, timeout=timeout, retries=2)

    if rsp is None:
        raise proto.OtaError(f"ctrl cmd={cmd:#04x} 没有回复（设备在跑吗？串口对不对？）")

    data = proto.i32(rsp, 1)                        # 带符号：转速会是负数
    data2 = proto.u32(rsp, 5) if len(rsp) > 8 else 0
    return rsp[0], data, data2


def read_status(dev, motor):
    """读一台电机的状态：里程 / 位置 / 故障码 / 当前模式 / 电机电源。"""
    rsp = dev.request(proto.T_STATUS, bytes([motor]), timeout=3.0, retries=3)

    if rsp is None:
        raise proto.OtaError("STATUS 没有回复（设备在跑吗）")
    if rsp[0] != proto.ST_OK:
        return None                 # 这台没答上（没上电 / 没接线 / ID 不是它）

    pos = proto.u16(rsp, 5)
    return {
        "mileage": proto.i32(rsp, 1),
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

    ⚠ 超时给得宽（设备侧“上电等 500ms + 使能重试最多 5s”都在这条命令里），
      但那只在“电机没上电/没接线”时才真的等满 —— 正常情况一发就回。
    """
    st, mode, bus_id = ctrl(dev, CTRL_SPEED_LOOP, 0, motor, timeout=25.0)

    if st != proto.ST_OK or mode != 0x02:
        raise proto.OtaError(
            f"电机{motor} 切速度环失败：status={proto.ST_TEXT.get(st, st)}、"
            f"回帧模式={mode:#04x}（要 0x02）。\n"
            f"    先 `car.py status` 看它答不答得上；不答就是没上电/没接线/ID 不对。")

    if accel is not None:
        set_accel(dev, motor, accel)

    return bus_id


def set_accel(dev, motor, ms_per_rpm):
    """速度环加速时间：每 1rpm 多少 ms（0 按 1 处理）。越大起步越柔。"""
    st, data, _ = ctrl(dev, CTRL_ACCEL, int(ms_per_rpm), motor)

    if st != proto.ST_OK:
        raise proto.OtaError(f"电机{motor} 设加速时间失败（{proto.ST_TEXT.get(st, st)}）")

    return data


def set_speed(dev, motor, rpm):
    """给一台电机一个转速（rpm，带符号）。

    设备侧发 0x64 之前会自己确认它在速度环，不在就自动切过去 —— 因为同一个数值
    在电流环里是电流、在位置环里是目标位置，“停车”给错环甚至会让轮子转回 0°。
    """
    if abs(rpm) > RPM_MAX:
        raise proto.OtaError(f"转速 {rpm} rpm 超范围（±{RPM_MAX:.0f} rpm）")

    st, data, _ = ctrl(dev, CTRL_SPEED, int(round(rpm * 10)), motor, timeout=15.0)

    if st != proto.ST_OK:
        raise proto.OtaError(f"电机{motor} 给转速失败（{proto.ST_TEXT.get(st, st)}）")

    return data / 10.0


def stop_all(dev):
    """两台都给 0 转速（速度环下 0 = 停）。尽量都试一遍，别因为一台失败就不管另一台。"""
    for motor in (1, 2):
        try:
            set_speed(dev, motor, 0)
            print(f"  电机{motor}: 已停")
        except proto.OtaError as e:
            print(f"  ⚠ 电机{motor} 停车失败：{e}")


def set_speeds_together(dev, speeds, watchdog_ms=0, quiet=False):
    """{电机号: rpm} → **一条** DRIVE 命令（设备侧背靠背发两条 0x64）。

    为什么不用两次 ctrl speed：那条路每台都要“无线往返一次 + 先查一次模式”，
    而且两条 0x64 中间夹着上位机的循环 —— 实测两台起步能差 30~80ms，
    低速时就是“先动的那侧把车拽歪一下”。合成一条之后只剩总线串行的 ~10ms。

    watchdog_ms > 0：设备自己兜底 —— 超过这么久没收到新的 DRIVE 就把两台停掉
    （松手/断线/上位机被强杀都会停，不用指望最后那条 stop 命令发得出去）。
    quiet=True：成功时不打那行摘要（遥控 20 Hz 时用，否则刷屏）。
    """
    raw = {}

    for motor, rpm in speeds.items():
        if abs(rpm) > RPM_MAX:
            raise proto.OtaError(f"电机{motor} 转速 {rpm} rpm 超范围（±{RPM_MAX:.0f} rpm）")
        raw[int(motor)] = int(round(rpm * 10))

    arg = proto.pair_pack(raw.get(1, 0), raw.get(2, 0))
    wd_units = 0 if watchdog_ms <= 0 else max(1, min(255, int(watchdog_ms) // 50))

    st, data, data2 = ctrl(dev, CTRL_DRIVE, arg, wd_units, timeout=20.0)

    if st == proto.ST_PARAM:
        # 老固件没有这条命令：退回“一台一台发”（起步差回来，但至少能动）
        print("  ⚠ 固件不认 DRIVE（旧版本？）—— 退回一台一台发，起步会差一点")
        for motor, rpm in speeds.items():
            set_speed(dev, motor, rpm)
        return

    if st != proto.ST_OK:
        raise proto.OtaError(f"DRIVE 失败：{proto.ST_TEXT.get(st, st)}")

    if not quiet:
        v1, v2 = proto.pair_unpack(data)
        print(f"  一起走：电机1={v1 / 10.0:+.1f} 电机2={v2 / 10.0:+.1f} rpm"
              f"（看门狗 {data2}ms{'，不启用' if data2 == 0 else ''}）")


def ensure_trim(dev, left_motor):
    """告诉设备“哪台是左轮”：里程差闭环靠它才能知道往哪边补。

    很便宜（一条命令、不碰电机），所以 setup/drive/teleop 每次都先发一遍 ——
    免得“换了车/改了 car_map.json，设备里还是老的左轮”这种静默错误（那种情况下车
    会越走越歪，而且没人报错）。
    （闭环只在“直行类”动作下积分，转圈/走弧设备自己会关，所以这里不用管动作。）
    返回 True = 配上了。
    """
    st, on, left = ctrl(dev, CTRL_TRIM, int(left_motor), 0, timeout=6.0)

    if (st != proto.ST_OK) or (on != 1) or (left != int(left_motor)):
        print(f"  ⚠ 里程差闭环没配上（status={proto.ST_TEXT.get(st, st)}，状态={on}，左轮={left}）"
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
            except proto.OtaError as e:
                print(f"  ⚠ 停车失败：{e}")


# ---------------------------------------------------------------------------
# 左右轮映射（id 命令写、drive/teleop 读）
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
    """(左轮 rpm, 右轮 rpm) → {电机号: rpm}，顺带处理左右定义和“反了”（invert）。

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
# 键盘遥控（teleop）：按住就走、松手就停
# ---------------------------------------------------------------------------
# kitty 键盘协议：`CSI > flags u`。flag 2 = 上报事件类型(按下/重复/抬起)，
# flag 8 = 所有键都按转义序列上报（否则普通字母还是当文本发，拿不到“抬起”）。
KITTY_ON = b"\x1b[>10u"
KITTY_OFF = b"\x1b[<u"

# 我们关心的键：字符 → kitty 的 keycode（小写字母的 ASCII 就是它的 keycode）
KEYS = {"w": 119, "a": 97, "s": 115, "d": 100, " ": 32, "q": 113}
CODE_TO_KEY = {v: k for k, v in KEYS.items()}


class Keys:
    """把终端变成“能拿到按下/抬起”的按键源。

    进入/退出用 with，保证异常退出时终端属性也还原（否则终端会一直留在 raw 模式）。
    """

    def __init__(self, release_ms):
        self.fd = sys.stdin.fileno()
        self.release_s = max(0.05, release_ms / 1000.0)
        self.down = {}          # 键 → 最近一次“看到它”的时刻
        self.kitty = None       # None = 还没看出来；True = 收到过带事件类型的按键
        self.buf = ""
        self._saved = None

    def __enter__(self):
        if termios is None:
            raise proto.OtaError("teleop 需要类 Unix 终端（termios/tty）—— Windows 请用 WSL，"
                                 "或者改用 `car.py drive`/`spin` 逐条发命令")
        self._saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)          # 不用回车；Ctrl+C 仍然是信号（ISIG 还开着）
        # 请求带“抬起”事件的按键上报（不支持的终端会忽略这串，退回①）
        os.write(sys.stdout.fileno(), KITTY_ON)
        return self

    def __exit__(self, *exc):
        try:
            os.write(sys.stdout.fileno(), KITTY_OFF)
        except Exception:
            pass
        if self._saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._saved)

    # -- 解析 ---------------------------------------------------------------
    def _press(self, key):
        if key in KEYS:
            self.down[key] = time.monotonic()

    def _kitty_key(self, body):
        """kitty 协议的 `CSI code:alt ; mods:event ; text u` 里 body 的部分。

        ⚠ 事件类型（1 按下 / 2 重复 / 3 抬起）是挂在**第二个字段的 `:` 后面**
          （`97;1:3u` = a 抬起），不是第一个字段 —— 曾经按“第一个字段带 `:` 才是事件”
          来解析，结果抬起被当成按下，**松手不停车**。
          没带 modifiers 的简写形式（`97:3u`）也认：那种情况下 `:` 后面的 1/2/3 才是事件
          （备用键码是个大得多的 unicode 码点，不会混）。
        """
        fields = body.split(";")
        event = 1
        code_text = fields[0].split(":")[0]

        if len(fields) >= 2 and ":" in fields[1]:
            try:
                event = int(fields[1].split(":", 1)[1] or "1")
            except ValueError:
                event = 1
        elif (len(fields) == 1) and (":" in fields[0]):
            c, _, ev = fields[0].partition(":")
            if ev in ("1", "2", "3"):
                code_text, event = c, int(ev)

        try:
            key = CODE_TO_KEY.get(int(code_text))
        except ValueError:
            return

        if key is None:
            return

        self.kitty = True

        if event == 3:                  # 抬起：真事件，立刻删
            self.down.pop(key, None)
        else:
            self._press(key)

    def _feed(self, text):
        i = 0

        while i < len(text):
            ch = text[i]

            if ch == "\x1b" and i + 1 < len(text) and text[i + 1] == "[":
                j = i + 2

                while j < len(text) and not ("\x40" <= text[j] <= "\x7e"):
                    j += 1

                if j >= len(text):
                    break               # 序列还没收完：剩下的下一拍再来

                if text[j] == "u":
                    self._kitty_key(text[i + 2:j])

                i = j + 1
                continue

            if ch.isprintable() and ch != "\x1b":
                self._press(ch.lower())

            i += 1

    def poll(self, timeout):
        """读一次输入，返回“当前按着”的键集合。"""
        try:
            r, _, _ = select.select([self.fd], [], [], timeout)
        except InterruptedError:
            r = []

        if r:
            try:
                data = os.read(self.fd, 64).decode("latin-1")
            except OSError:
                data = ""

            if data:
                self._feed(data)

        # kitty 模式有真的“抬起”事件；退回模式只能靠“多久没再看到它”来推断松开
        if self.kitty is not True:
            now = time.monotonic()
            for key in [k for k, t in self.down.items() if now - t > self.release_s]:
                del self.down[key]

        return set(self.down)


def keys_to_wheels(pressed, speed, turn):
    """按键集合 → (左轮 rpm, 右轮 rpm)。

    「左转」= 车头往左 = 左轮慢/后退、右轮快/前进（差速底盘就这一条规则）。
    前后左右可以叠加：w+d = 一边前进一边往右拐（左轮快、右轮慢）。
    """
    if " " in pressed:                          # 空格 = 急停（按住期间就是 0）
        return 0.0, 0.0

    fwd = (1 if "w" in pressed else 0) - (1 if "s" in pressed else 0)
    rot = (1 if "a" in pressed else 0) - (1 if "d" in pressed else 0)

    return fwd * speed - rot * turn, fwd * speed + rot * turn


# ---------------------------------------------------------------------------
# 子命令
# ---------------------------------------------------------------------------
def cmd_setup(dev, args):
    """上电 + 两台都使能并切到速度环（小车开跑前的“点火”）。"""
    print("① 电机电源上电（PC14）…")
    power_on(dev)

    accel = getattr(args, "accel", None)
    print(f"② 两台使能 + 切速度环{'，加速时间 %d ms/rpm' % accel if accel is not None else ''}…")
    for motor in range(1, proto.N_MOTORS + 1):
        bus_id = ensure_speed_loop(dev, motor, accel)
        print(f"  电机{motor}（总线 ID{bus_id}）：速度环 OK")

    print("③ 里程差闭环（走直线用）：把左轮告诉设备")
    ensure_trim(dev, load_map()["left"])

    cmd_status(dev, args)


def cmd_status(dev, args):
    """看两台电机：在哪台、什么模式、位置/里程/故障码。"""
    motor_arg = getattr(args, "motor", 0)
    motors = [motor_arg] if motor_arg else list(range(1, proto.N_MOTORS + 1))

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
    """两个位置值之间“实际转了多少计数”：位置一圈是 0~32767 且 32767 与 0 是同一点，
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
    —— 两台之差就是“这一段歪了多少”（差值大就说明闭环没开好或者轮径差太大）。
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
    """停车。默认只给 0 转速（保持在速度环里，还能马上再开）。

    两个开关各管一件事，互不影响：
      --disable   失能（电机不使劲，但还通着电、还能读状态）
      --poweroff  断电机电源（PC14）
    收工就都写：`car.py stop --disable --poweroff`（或再 `motor.py pwoff`）。
    """
    print("停车：")
    stop_all(dev)

    if args.disable:
        print("失能（不碰电源）：")
        st, _, _ = ctrl(dev, CTRL_DISABLE, 0, 0, timeout=10.0)
        print(f"  {'OK' if st == proto.ST_OK else proto.ST_TEXT.get(st, st)}")

    if args.poweroff:
        print("断电机电源（PC14）：")
        st, _, _ = ctrl(dev, CTRL_PWR, 0, 0, timeout=10.0)
        print(f"  {'OK' if st == proto.ST_OK else proto.ST_TEXT.get(st, st)}")

    if args.disable or args.poweroff:
        print("  ⚠ 下次要动之前先 `car.py setup`（或 motor.py pwron + enable）")


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
        raise proto.OtaError("id 的 --seconds 必须 > 0（0 = 一直转，只对 spin/drive 有意义）")

    print("先上电 + 两台都切到速度环：")
    power_on(dev)
    for motor in range(1, proto.N_MOTORS + 1):
        ensure_speed_loop(dev, motor)
        print(f"  电机{motor}: 速度环 OK")
    print()

    for motor in range(1, proto.N_MOTORS + 1):
        print(f"===== 现在动 **电机{motor}**（总线 ID{motor}）：{args.rpm:+.0f} rpm "
              f"持续 {args.seconds:.1f}s =====")
        hold(dev, {motor: args.rpm}, args.seconds)
        time.sleep(1.0)         # 留一点空档，好区分“哪一段是哪台”

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
        for motor in range(1, proto.N_MOTORS + 1):
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
    （同时里程 +1），所以差分要按“最短路径”折一下。
    """
    prev = {}
    t0 = time.time()

    print(f"每 {args.interval:.2f}s 读一次（Ctrl+C 停）。转速由编码器差分算出来，"
          f"和给的值对比着看")

    try:
        while True:
            elapsed = time.time() - t0

            for motor in range(1, proto.N_MOTORS + 1):
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
            raise proto.OtaError("左轮和右轮不能是同一台电机（left == right）")
        save_map(m)
        print(f"已更新 {MAP_PATH}")

    print(f"左轮 = 电机{m['left']}"
          f"{'（反向）' if m.get('invert_left') else ''}，"
          f"右轮 = 电机{m['right']}"
          f"{'（反向）' if m.get('invert_right') else ''}")


def cmd_teleop(dev, args):
    """键盘遥控：按住就走、松手就停（w/a/s/d，空格急停，q 退出）。

    ⚠ “松手就停”在终端里没有天然事件：终端只在**按下**（和自动重复）时发字符，
      **松开什么都不发**。所以两条路都备着，跑起来时会打印用的是哪一种：
        ① kitty 键盘协议（VS Code / kitty / wezterm 支持）：抬起事件 → 立刻停；
        ② 自动重复超时推断：超过 --release-ms 没再收到那个字母就算松手。
      另外还有**看门狗**兜底（--watchdog-ms）：松手/断线/上位机挂了设备自己停。
    """
    turn = args.speed / 2.0 if args.turn is None else args.turn
    tick = 1.0 / max(1.0, args.rate)

    m = load_map()
    print("== 遥控前的一次性准备 ==")
    print("① 上电 + 两台进速度环")
    power_on(dev)
    for motor in range(1, proto.N_MOTORS + 1):
        bus_id = ensure_speed_loop(dev, motor, args.accel)
        print(f"  电机{motor}（总线 ID{bus_id}）：速度环 OK")

    print("② 左右映射 + 里程差闭环")
    print(f"  左轮 = 电机{m['left']}{'（反向）' if m.get('invert_left') else ''}，"
          f"右轮 = 电机{m['right']}{'（反向）' if m.get('invert_right') else ''}"
          f"   ← tools/car_map.json")

    if args.no_trim:
        ctrl(dev, CTRL_TRIM, 0, 0, timeout=6.0)
        print("  里程差闭环：已关（--no-trim）")
    else:
        ensure_trim(dev, m["left"])

    print(f"\n== 遥控中 ==\n"
          f"  w/s = 前进/后退 {args.speed:.0f}rpm，a/d = 原地转 {turn:.0f}rpm，空格 = 急停，q = 退出\n"
          f"  发送 {args.rate:.0f}Hz，看门狗 {args.watchdog_ms}ms（松手/断线设备自己停）")

    if not sys.stdin.isatty():
        raise proto.OtaError("这个终端不能实时读按键（stdin 不是 tty）—— 在 VS Code 的终端里跑，"
                             "别用管道/重定向")

    keys = Keys(args.release_ms)
    told_mode = False
    failures = 0
    last_line = ""
    idle_sent = False       # 上一拍已经“发过一次 0 且当时就停着”（那就别每 50ms 再发一遍）

    try:
        with keys:
            while True:
                pressed = keys.poll(tick)

                if (not told_mode) and (keys.kitty is not None) and pressed:
                    told_mode = True
                    print("  [输入] " + ("kitty 协议：能收到真正的「抬起」事件"
                                        if keys.kitty else
                                        f"自动重复推断：松开 {args.release_ms}ms 后停车"))

                if "q" in pressed:
                    break

                left, right = keys_to_wheels(pressed, args.speed, turn)
                speeds = wheel_speeds(left, right, m)
                moving = bool(pressed) and (left != 0.0 or right != 0.0)

                # 停着的时候就别再刷了：目标已经是 0，设备不需要心跳
                # （设备侧还没停的靠它自己的看门狗；停好之后就安安静静待着）
                if (not moving) and idle_sent:
                    continue

                try:
                    set_speeds_together(dev, speeds, args.watchdog_ms, quiet=True)
                    failures = 0
                    idle_sent = not moving
                except proto.OtaError as e:
                    failures += 1
                    print(f"\n  ⚠ 命令发不出去（第 {failures} 次）：{e}")
                    if failures >= 5:
                        print("  连着 5 次失败：退出（设备侧的看门狗会自己把车停下来）")
                        return 1
                    time.sleep(0.2)
                    continue

                mode = "停" if not pressed else "+".join(sorted(pressed))
                line = (f"  [{mode:<6}] 左={left:+6.1f} 右={right:+6.1f} rpm"
                        f"  →  电机1={speeds.get(1, 0.0):+6.1f} 电机2={speeds.get(2, 0.0):+6.1f}")

                if line != last_line:
                    sys.stdout.write("\r" + " " * len(last_line) + "\r" + line)
                    sys.stdout.flush()
                    last_line = line
    except KeyboardInterrupt:
        print("\n  Ctrl+C")
    finally:
        print()
        try:
            # 停车：两台一起，并把看门狗清掉
            set_speeds_together(dev, {1: 0.0, 2: 0.0}, 0)
            print("  已停车")
            if args.disable:
                st, _, _ = ctrl(dev, CTRL_DISABLE, 0, 0, timeout=10.0)
                print(f"  已失能（不碰电源）：{proto.ST_TEXT.get(st, st)}")
            if args.poweroff:
                st, _, _ = ctrl(dev, CTRL_PWR, 0, 0, timeout=10.0)
                print(f"  已断电机电源（PC14）：{proto.ST_TEXT.get(st, st)}")
            if not (args.disable and args.poweroff):
                print("  （收工就把两件都做：`car.py stop --disable --poweroff`）")
        except proto.OtaError as e:
            print(f"  ⚠ 停车失败：{e} —— 按板子复位，或拔电机电源")

    return 0


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="两轮差速小车上位机（速度环：识别左右轮 / 差速走 / 键盘遥控）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    proto.add_port_args(ap)

    sub = proto.add_subparsers(ap, dest="action", required=True)

    p = sub.add_parser("setup", aliases=["scan"], help="上电 + 两台使能 + 切速度环 + 配闭环")
    p.add_argument("--accel", type=int, default=None,
                   help="顺便设加速时间（每 1rpm 多少 ms，0..255）；不填就不动")
    p.set_defaults(func=cmd_setup)

    p = sub.add_parser("status", help="两台电机的模式/位置/里程/故障码")
    p.add_argument("--motor", type=int, default=0, help=f"只看这一台（1..{proto.N_MOTORS}）")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("id", help="识别哪台是左轮（依次点动两台，然后逐台问你）")
    p.add_argument("--rpm", type=float, default=15.0, help="点动转速 rpm（默认 15）")
    p.add_argument("--seconds", type=float, default=1.5, help="每台转多久（默认 1.5s）")
    p.add_argument("--no-input", action="store_true", help="不提问，只转（脚本里用）")
    p.set_defaults(func=cmd_id)

    p = sub.add_parser("spin", help="单轮点动（只动一台）")
    p.add_argument("--motor", type=int, required=True, choices=range(1, proto.N_MOTORS + 1),
                   help="动哪一台（1/2）")
    p.add_argument("--rpm", type=float, default=15.0, help="转速 rpm，带符号（默认 15）")
    p.add_argument("--seconds", type=float, default=2.0, help="转多久（默认 2s；0 = 一直转到 Ctrl+C）")
    p.set_defaults(func=cmd_spin)

    p = sub.add_parser("drive", help="两轮一起跑（按 id 存的映射）")
    p.add_argument("--left", type=float, default=0.0, help="左轮 rpm（带符号）")
    p.add_argument("--right", type=float, default=0.0, help="右轮 rpm（带符号）")
    p.add_argument("--seconds", type=float, default=2.0, help="跑多久（默认 2s；0 = 一直转到 Ctrl+C）")
    p.set_defaults(func=cmd_drive)

    p = sub.add_parser("stop", help="停车（--disable 失能 / --poweroff 断电）")
    p.add_argument("--disable", action="store_true", help="顺带失能两台（不碰电源）")
    p.add_argument("--poweroff", action="store_true", help="顺带断开电机电源（PC14）")
    p.set_defaults(func=cmd_stop)

    p = sub.add_parser("watch", help="看两台的实际转速（编码器差分算的）")
    p.add_argument("--interval", type=float, default=0.2, help="采样间隔秒（默认 0.2）")
    p.add_argument("--seconds", type=float, default=0.0, help="看多久（默认 = 一直看到 Ctrl+C）")
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser("teleop", help="键盘遥控：按住就走、松手就停（w/a/s/d，空格急停，q 退出）")
    p.add_argument("--speed", type=float, default=30.0, help="前进/后退的 rpm（默认 30）")
    p.add_argument("--turn", type=float, default=None, help="原地转的 rpm（默认 = --speed 的一半）")
    p.add_argument("--rate", type=float, default=20.0, help="发送频率 Hz（默认 20，即 50ms 一条）")
    p.add_argument("--watchdog-ms", type=int, default=400,
                   help="松手/断线后设备自己停车的超时（默认 400ms；0 = 不启用，不推荐）")
    p.add_argument("--release-ms", type=int, default=250,
                   help="终端不支持 key-up 时，多久没再收到某个键就算松开了（默认 250ms）")
    p.add_argument("--accel", type=int, default=None,
                   help="顺便设加速时间（每 1rpm 多少 ms）；不填就不动")
    p.add_argument("--no-trim", action="store_true", help="关掉里程差闭环（纯开环）")
    p.add_argument("--disable", action="store_true", help="退出时顺带失能（不碰电源）")
    p.add_argument("--poweroff", action="store_true", help="退出时顺带断开电机电源（PC14）")
    p.set_defaults(func=cmd_teleop)

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

    # map 是纯本地的小配置，不碰串口
    return proto.serve(args, args.func, needs_port=(args.action != "map"),
                       interrupt_msg="⚠ 电机可能还在转 —— 跑一次 `car.py stop` 或按板子复位")


if __name__ == "__main__":
    sys.exit(main())
