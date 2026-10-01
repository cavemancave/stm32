#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WASD 键盘遥控：**按住就走、松开就停**，两条命令一条心跳。

    python tools/car_teleop.py                            # w/a/s/d，空格急停，q 退出
    python tools/car_teleop.py --speed 40 --turn 25       # 快一点 / 转得慢一点
    python tools/car_teleop.py --no-trim                  # 关掉里程差闭环，看纯开环漂多少
    python tools/car_teleop.py --rate 15 --watchdog-ms 400

按键（可组合，比如 w+d = 一边前进一边右转）：

    w 前进      s 后退      a 左转      d 右转      空格 急停      q 退出

⚠⚠ "松开就停止"在终端里没有天然的事件：
    终端只在**按下**（和自动重复）时发字符，**松开什么都不发**，没有 key-up 事件。
  所以这里两条路都备着，启动时打印用的是哪一种：
    ① **kitty 键盘协议**（`CSI > 10 u`，VS Code / kitty / wezterm 等支持）：
       按下/重复/抬起三个事件会带上类型（`CSI code;mods:event u`），抬起立刻停 —— 真事件；
    ② **自动重复超时推断**（不支持上面的协议时）：还按着的话终端会一直重复发那个字母
       （典型 20~30 次/秒），超过 --release-ms 没再收到 ⇒ 认为松开了。
       注意：如果系统把"按键重复延迟"设得很长，第一次按下的反应还是即时的（我们不等重复），
       只有"松开"会晚一个 --release-ms。

另一个安全网是**看门狗**：每条 DRIVE 命令都带"多久没收到新命令就自己停"
（默认 400ms，比一个心跳周期略长）。所以松手、断线、无线模块掉电、笔记本睡死，
车都会自己停，而不是靠最后那条 stop 命令发得出去。

⚠ 第一次请把车架空（或让人扶住），先小转速（--speed 15）试：遥控时车会按你按下的
  每一个键立刻动，方向反了就回 `car.py id` 重认映射。
"""

import argparse
import os
import select
import sys
import termios
import time
import tty

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ota      # noqa: E402
import car      # noqa: E402

# kitty 键盘协议：`CSI > flags u`。flag 2 = 上报事件类型(按下/重复/抬起)，
# flag 8 = 所有键都按转义序列上报（否则普通字母还是当文本发，拿不到"抬起"）。
KITTY_ON  = b"\x1b[>10u"
KITTY_OFF = b"\x1b[<u"

# 我们关心的键：字符 → kitty 的 keycode（小写字母的 ASCII 就是它的 keycode）
KEYS = {"w": 119, "a": 97, "s": 115, "d": 100, " ": 32, "q": 113}
CODE_TO_KEY = {v: k for k, v in KEYS.items()}


class Keys:
    """把终端变成"能拿到按下/抬起"的按键源。

    进入/退出用 with，保证异常退出时终端属性也还原（否则终端会一直留在 raw 模式）。
    """

    def __init__(self, release_ms):
        self.fd = sys.stdin.fileno()
        self.release_s = max(0.05, release_ms / 1000.0)
        self.down = {}          # 键 → 最近一次"看到它"的时刻
        self.kitty = None       # None = 还没看出来；True = 收到过带事件类型的按键
        self.buf = ""
        self._saved = None

    def __enter__(self):
        self._saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)          # 不用回车；Ctrl+C 仍然是信号（ISIG 还开着）
        # 请求带"抬起"事件的按键上报（不支持的终端会忽略这串，退回①）
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
          （`97;1:3u` = a 抬起），不是第一个字段 —— 曾经按"第一个字段带 `:` 才是事件"
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
        """读一次输入，返回"当前按着"的键集合。"""
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

        # kitty 模式有真的"抬起"事件；退回模式只能靠"多久没再看到它"来推断松开
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


def main():
    ap = argparse.ArgumentParser(
        description="WASD 键盘遥控（长按就走，松手就停）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--port", default=ota.DEFAULT_PORT, help=f"串口（默认 {ota.DEFAULT_PORT}）")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--speed", type=float, default=30.0, help="前进/后退的 rpm（默认 30）")
    ap.add_argument("--turn", type=float, default=None,
                    help="原地转的 rpm（默认 = --speed 的一半）")
    ap.add_argument("--rate", type=float, default=20.0, help="发送频率 Hz（默认 20，即 50ms 一条）")
    ap.add_argument("--watchdog-ms", type=int, default=400,
                    help="松手/断线后设备自己停车的超时（默认 400ms；0 = 不启用，不推荐）")
    ap.add_argument("--release-ms", type=int, default=250,
                    help="终端不支持 key-up 时，多久没再收到某个键就算松开了（默认 250ms）")
    ap.add_argument("--accel", type=int, default=None,
                    help="顺手设加速时间（每 1rpm 多少 ms）；不填就不动")
    ap.add_argument("--no-trim", action="store_true", help="关掉里程差闭环（纯开环）")
    ap.add_argument("--disable", action="store_true", help="退出时顺带失能 + 断电机电源")
    args = ap.parse_args()

    turn = args.speed / 2.0 if args.turn is None else args.turn
    tick = 1.0 / max(1.0, args.rate)
    wd_units = 0 if args.watchdog_ms <= 0 else max(1, min(255, args.watchdog_ms // 50))

    try:
        ota.check_port(args.port)
        dev = ota.Device(args.port, args.baud, quiet=False)
    except ota.OtaError as e:
        print(f"\n❌ {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"\n❌ 打不开 {args.port}：{e}", file=sys.stderr)
        return 1

    m = car.load_map()
    rc = 0

    print("== 遥控前的一次性准备 ==")
    print("① 上电 + 两台进速度环")
    car.power_on(dev)
    for motor in range(1, ota.N_MOTORS + 1):
        bus_id = car.ensure_speed_loop(dev, motor, args.accel)
        print(f"  电机{motor}（总线 ID{bus_id}）：速度环 OK")

    print("② 左右映射 + 里程差闭环")
    print(f"  左轮 = 电机{m['left']}{'（反向）' if m.get('invert_left') else ''}，"
          f"右轮 = 电机{m['right']}{'（反向）' if m.get('invert_right') else ''}"
          f"   ← tools/car_map.json")

    if args.no_trim:
        car.ctrl(dev, car.CTRL_TRIM, 0, 0, timeout=6.0)
        print("  里程差闭环：已关（--no-trim）")
    else:
        car.ensure_trim(dev, m["left"])

    print(f"\n== 遥控中 ==\n"
          f"  w/s = 前进/后退 {args.speed:.0f}rpm，a/d = 原地转 {turn:.0f}rpm，空格 = 急停，q = 退出\n"
          f"  发送 {args.rate:.0f}Hz，看门狗 {args.watchdog_ms}ms（松手/断线设备自己停）")

    if not sys.stdin.isatty():
        print("\n❌ 这个终端不能实时读按键（stdin 不是 tty）—— 在 VS Code 的终端里跑，"
              "别用管道/重定向。", file=sys.stderr)
        dev.close()
        return 1

    keys = Keys(args.release_ms)
    told_mode = False
    failures = 0
    last_line = ""
    idle_sent = False       # 上一拍已经"发过一次 0 且当时就停着"（那就别每 50ms 再发一遍）

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
                speeds = car.wheel_speeds(left, right, m)
                moving = bool(pressed) and (left != 0.0 or right != 0.0)

                # 停着的时候就别再刷了：目标已经是 0，设备不需要心跳
                # （设备侧还没停的靠它自己的看门狗；停好之后就安安静静待着）
                if (not moving) and idle_sent:
                    continue

                try:
                    car.set_speeds_together(dev, speeds, args.watchdog_ms, quiet=True)
                    failures = 0
                    idle_sent = not moving
                except ota.OtaError as e:
                    failures += 1
                    print(f"\n  ⚠ 命令发不出去（第 {failures} 次）：{e}")
                    if failures >= 5:
                        print("  连着 5 次失败：退出（设备侧的看门狗会自己把车停下来）")
                        rc = 1
                        break
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
            # 停车：两台一起，并把看门狗清掉（否则那条 0 也会带上超时，无所谓但没必要）
            car.set_speeds_together(dev, {1: 0.0, 2: 0.0}, 0)
            print("  已停车")
            if args.disable:
                st, _, _ = car.ctrl(dev, car.CTRL_DISABLE, 0, 0, timeout=10.0)
                print(f"  失能 + 断电：{ota.ST_TEXT.get(st, st)}")
            else:
                print("  （没断电：下次 `car.py stop --disable`，或加 --disable 让遥控退出时就断）")
        except ota.OtaError as e:
            print(f"  ⚠ 停车失败：{e} —— 按板子复位，或拔电机电源")
        dev.close()

    return rc


if __name__ == "__main__":
    sys.exit(main())
