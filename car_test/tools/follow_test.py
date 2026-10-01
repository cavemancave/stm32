#!/usr/bin/env python3
"""位置跟随的一键实测（**把设备日志一起收下来**）。

为什么要单独写这个脚本：
  `ctrl follow` / `ctrl spin` 各自是一个独立进程，**回帧一收到进程就退出、串口一关**，
  之后设备打的 `[follow]` 1 Hz 行（以及任何 ABORT / WARN）就**没人读、直接丢了**。
  于是现象看起来是"命令发出去了、电机没动、日志里什么都没有"，但其实设备说了话。

  这个脚本把整段序列放在**同一个进程**里，串口全程开着，日志原样打出来，
  最后还会把停止跟随时的回帧（实测周期 / 平均跟随误差）和每个任务的栈余量一起收。

用法（串口默认 /dev/ttyACM0，Linux 下 USB-CDC 就是它）：
    python tools/follow_test.py                                  # 默认：跟随 + 2 号机 30°/s 转 8 s
    python tools/follow_test.py --port /dev/ttyUSB0               # 换个口（CH340/CP210x 那类）
    python tools/follow_test.py --spin 0                          # 不自动转：用手拖 2 号机（真遥操作）
    python tools/follow_test.py --spin 300 --lead 0               # 对比：关掉前馈看“纯跟随”的稳态滞后

⚠ **转速不要给超过 300（30°/s）太多**：实测这颗电机在位置环里的速度上限只有 ~45°/s
  （命令 90°/s 它只跑到 42°/s，而且一直顶在最大速度上）。顶在最大速度上 = 反作用力矩最大，
  直驱台架会被拖着动。

看什么：
  `rate=`      实测跟随频率（一拍 3~4 条帧、同一总线一问一答，42 Hz 上下就是上限）
  `lag avg/max` 1 号机落后 2 号机多少度 —— **这就是"跟得紧不紧"的答案**
           （lead=500 时应该接近 0；lead=0 时会稳定落后一截，那就是纯跟随的稳态误差）
  `miss=/glitch=/overrun=`  丢帧 / 位置被丢 / 一拍没跑完 的次数（都该是 0）
  `hwm=`       跟随任务还剩多少字节栈
  `lead=`      当前生效的前馈提前量
"""
import argparse
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ota   # noqa: E402   （复用同一个目录里的协议层，不重复实现）

CTRL_FOLLOW, CTRL_SPIN, CTRL_STACKS, CTRL_LEAD = 0x0B, 0x0C, 0x0D, 0x0E


def ctrl(dev, cmd, arg, timeout=25.0):
    """发一条 CTRL 命令并把回帧的三个数据字都打出来（不受"在不在跑"的影响）。"""
    rsp = dev.request(ota.T_CTRL, struct.pack("<BI", cmd, arg), timeout=timeout, retries=2)

    if rsp is None:
        print(f"❌ ctrl cmd={cmd:#04x} arg={arg}：没有回复（设备在跑吗？）")
        return None

    data = ota.u32(rsp, 1)
    data2 = ota.u32(rsp, 5) if len(rsp) > 8 else 0
    data3 = ota.i32(rsp, 9) if len(rsp) > 12 else 0

    extra = ""
    if cmd == CTRL_FOLLOW:
        extra = (f"  跟随={'运行中' if data else '已停止'}"
                 f"  实测周期={data2 / 1000.0:.2f}ms（{1000000.0 / data2:.1f}Hz）" if data2 else
                 f"  跟随={'运行中' if data else '已停止'}")
        if data3:
            extra += f"  平均落后={data3 * 360.0 / 32768:+.1f}°"
    elif cmd == CTRL_SPIN:
        extra = f"  2 号机（ID{data2}）转速={data / 10.0:.1f}°/s" if data else "  2 号机已停"

    print(f"── ctrl cmd={cmd:#04x} arg={arg} → {ota.ST_TEXT.get(rsp[0], rsp[0])}  "
          f"data={data} data2={data2} data3={data3}{extra}")
    return data, data2, data3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=ota.DEFAULT_PORT,
                    help=f"电脑这边的串口（默认 {ota.DEFAULT_PORT}；也可以设环境变量 CAR_PORT）")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--spin", type=int, default=300,
                    help="2 号机自动转速，单位 0.1°/s（默认 300 = 30.0°/s；0 = 不自动转，手拖）。"
                         "⚠ 实测位置环速度上限 ~45°/s（450），给大了只会让它顶在最大速度上")
    ap.add_argument("--seconds", type=float, default=8.0, help="自动转/观察多久（默认 8 s）")
    ap.add_argument("--period", type=int, default=25, help="跟随周期 ms（默认 25）")
    ap.add_argument("--lead", type=int, default=None,
                    help="速度前馈提前量 ms（0 = 纯跟随，~500 抵消稳态滞后）；不填就用设备默认")
    args = ap.parse_args()

    dev = ota.Device(args.port, args.baud, quiet=False)

    try:
        print(f"== 1) 开始跟随（周期 {args.period} ms）==")
        ctrl(dev, CTRL_FOLLOW, args.period)

        if args.lead is not None:
            print(f"== 1b) 设置前馈提前量 {args.lead} ms ==")
            ctrl(dev, CTRL_LEAD, args.lead)
        dev.pump(0.5)

        if args.spin:
            print(f"== 2) 让 2 号机以 {args.spin / 10.0:.1f}°/s 自动转 ==")
            ctrl(dev, CTRL_SPIN, args.spin)
        else:
            print("== 2) 不自动转：请现在用手拖 2 号机 ==")

        print(f"== 3) 观察 {args.seconds:.0f}s —— 下面就是设备日志 ==")
        dev.pump(args.seconds)

        print("== 4) 停止跟随（回帧里带实测周期和平均跟随误差）==")
        ctrl(dev, CTRL_FOLLOW, 0)
        dev.pump(1.5)

        print("== 5) 各任务栈余量（hwm 越大越安全）==")
        ctrl(dev, CTRL_STACKS, 0)
        dev.pump(1.0)
    finally:
        dev.close()

    print("完成。若 lag 一直在长/glitch 在涨，把上面 [follow] 行发我。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
