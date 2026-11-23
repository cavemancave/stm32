"""uart_rx_dma 硬件测试：正常功能 / 粘包 / 帧中断 / CRC 失败 / 性能统计。

用法:
    python tools/test_uart.py --port COM12 --baud 921600
    python tools/test_uart.py --list
    python tools/test_uart.py --perf-count 1000 --tests perf

依赖: pip install pyserial
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

from proto import (
    ASYNC_TYPES,
    ERR_NAMES,
    MSG_CALC_SQSUM_REQ,
    MSG_CALC_SQSUM_RSP,
    MSG_CALC_SUM_REQ,
    MSG_CALC_SUM_RSP,
    MSG_CALC_WEIGHTED_REQ,
    MSG_CALC_WEIGHTED_RSP,
    MSG_HEARTBEAT,
    MSG_NACK,
    MSG_PING_REQ,
    MSG_PONG_RSP,
    MSG_STATS_REQ,
    MSG_STATS_RSP,
    Nack,
    SerialLink,
    StatsPage,
    autodetect_port,
    calc_payload,
    decode_calc,
    decode_heartbeat,
    decode_nack,
    decode_pong,
    decode_stats,
    encode,
    list_serial_ports,
    type_name,
)

PASS = 0
FAIL = 0
RESULTS: list[dict] = []


def section(title: str) -> None:
    print()
    print("=" * 72)
    print(title)
    print("=" * 72)


def check(cond: bool, message: str, detail: str = "") -> bool:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [ OK ] {message}")
    else:
        FAIL += 1
        print(f"  [FAIL] {message}")
        if detail:
            print(f"         {detail}")
    RESULTS.append({"ok": bool(cond), "message": message, "detail": detail})
    return bool(cond)


def show_frame(frame, prefix: str = "         ") -> None:
    print(f"{prefix}<{frame.name}> len={len(frame.payload)} raw={frame.raw.hex(' ')}")


def expect_nack(link: SerialLink, code: int, timeout: float = 1.0) -> Optional[Nack]:
    frame = link.wait_frame({MSG_NACK}, timeout)
    if frame is None:
        return None
    nack = decode_nack(frame)
    if nack.code != code:
        print(f"         NACK code={nack.code_name} (期望 {ERR_NAMES.get(code)})")
    return nack


# --------------------------------------------------------------------------- #
# 测试项
# --------------------------------------------------------------------------- #
def test_ping(link: SerialLink) -> None:
    section("1. 正常功能：PING / PONG")
    link.drain()
    ok_count = 0
    rtts = []
    for i in range(5):
        t0 = time.perf_counter()
        link.send(MSG_PING_REQ)
        frame = link.wait_frame({MSG_PONG_RSP}, 1.0)
        if frame is None:
            check(False, f"ping #{i + 1} 有响应")
            continue
        rx_ms = (time.perf_counter() - t0) * 1000.0
        pong = decode_pong(frame)
        rtts.append(rx_ms)
        ok_count += 1
        check(
            pong.proc_us < 5000,
            f"ping #{i + 1}: uptime={pong.uptime_ms}ms proc={pong.proc_us}us rtt={rx_ms:.2f}ms",
            f"raw={frame.raw.hex(' ')}",
        )
    check(ok_count == 5, f"PING 5/5 有响应（实际 {ok_count}）")
    if rtts:
        print(f"         RTT min/avg/max = {min(rtts):.2f} / {statistics.mean(rtts):.2f} / {max(rtts):.2f} ms")


def test_calc(link: SerialLink) -> None:
    section("2. 正常功能：三个计算消息")
    link.drain()
    cases = [
        ("算数和", MSG_CALC_SUM_REQ, MSG_CALC_SUM_RSP, 1, 2, 3),
        ("算数和(负数)", MSG_CALC_SUM_REQ, MSG_CALC_SUM_RSP, -5, 10, -20),
        ("算数和(零)", MSG_CALC_SUM_REQ, MSG_CALC_SUM_RSP, 0, 0, 0),
        ("平方和", MSG_CALC_SQSUM_REQ, MSG_CALC_SQSUM_RSP, 2, 3, 4),
        ("平方和(大数饱和)", MSG_CALC_SQSUM_REQ, MSG_CALC_SQSUM_RSP, 32767, -32768, 12345),
        ("加权和", MSG_CALC_WEIGHTED_REQ, MSG_CALC_WEIGHTED_RSP, 1, 2, 3),
        ("加权和(负数)", MSG_CALC_WEIGHTED_REQ, MSG_CALC_WEIGHTED_RSP, -100, 50, -25),
        ("加权和(大数)", MSG_CALC_WEIGHTED_REQ, MSG_CALC_WEIGHTED_RSP, 32767, 32767, 32767),
    ]
    INT32_MAX = 0x7FFFFFFF
    INT32_MIN = -0x80000000

    for name, req_type, rsp_type, a, b, c in cases:
        if req_type == MSG_CALC_SUM_REQ:
            expected = a + b + c
        elif req_type == MSG_CALC_SQSUM_REQ:
            expected = a * a + b * b + c * c
        else:
            expected = a + 2 * b + 3 * c
        clamped = False
        if expected > INT32_MAX:
            expected = INT32_MAX
            clamped = True
        elif expected < INT32_MIN:
            expected = INT32_MIN
            clamped = True

        link.send(req_type, calc_payload(a, b, c))
        frame = link.wait_frame({rsp_type}, 1.0)
        if frame is None:
            check(False, f"{name} a={a} b={b} c={c} 有响应")
            continue
        rsp = decode_calc(frame)
        check(
            rsp.result == expected and rsp.status == (1 if clamped else 0),
            f"{name} a={a} b={b} c={c} -> {rsp.result} (期望 {expected}), status={rsp.status}",
            f"proc={rsp.proc_us}us raw={frame.raw.hex(' ')}",
        )
        check(0 < rsp.proc_us < 5000, f"{name} 处理时间 {rsp.proc_us}us 合理")


def test_sticky(link: SerialLink) -> None:
    section("3. 粘包：多帧一次写入 / 带前导垃圾 / 分两次写入")
    link.drain()

    raws = [
        encode(MSG_CALC_SUM_REQ, calc_payload(1, 2, 3)),
        encode(MSG_CALC_SQSUM_REQ, calc_payload(2, 3, 4)),
        encode(MSG_CALC_WEIGHTED_REQ, calc_payload(1, 2, 3)),
        encode(MSG_PING_REQ),
    ]
    expected = [(MSG_CALC_SUM_RSP, 6), (MSG_CALC_SQSUM_RSP, 29), (MSG_CALC_WEIGHTED_RSP, 14), (MSG_PONG_RSP, None)]

    # 4 帧拼成一次写
    link.send_many(raws)
    frames = link.collect_frames(4, 2.0)
    check(len(frames) == 4, f"粘包 4 帧收到 4 个响应（实际 {len(frames)}）")
    for i, (frame, (rsp_type, value)) in enumerate(zip(frames, expected)):
        if frame.type != rsp_type:
            check(False, f"粘包第 {i + 1} 帧类型 {frame.name} (期望 {type_name(rsp_type)})")
            continue
        if value is None:
            check(True, f"粘包第 {i + 1} 帧 {frame.name}")
        else:
            rsp = decode_calc(frame)
            check(rsp.result == value, f"粘包第 {i + 1} 帧结果 {rsp.result} (期望 {value})")

    # 前导垃圾 + 3 帧
    link.drain()
    link.send_raw(b"\x00\x11\x22" + b"".join(raws[:3]))
    frames = link.collect_frames(3, 2.0)
    check(len(frames) == 3, f"前导垃圾后收到 3 个响应（实际 {len(frames)}）")
    check(
        [f.type for f in frames] == [MSG_CALC_SUM_RSP, MSG_CALC_SQSUM_RSP, MSG_CALC_WEIGHTED_RSP],
        "前导垃圾后响应类型/顺序正确",
        f"实际 {[f.name for f in frames]}",
    )

    # 同一帧分两次写，间隔 < 超时
    link.drain()
    raw = encode(MSG_PING_REQ)
    link.send_raw(raw[:3])
    time.sleep(0.003)
    link.send_raw(raw[3:])
    frame = link.wait_frame({MSG_PONG_RSP}, 1.0)
    check(frame is not None, "一帧拆两次写入仍能正确解析")


def test_interrupt(link: SerialLink) -> None:
    section("4. 帧中断：半帧后停住，超时应回 NACK_TIMEOUT 并恢复")
    link.drain()

    raw = encode(MSG_CALC_SUM_REQ, calc_payload(7, 8, 9))
    link.send_raw(raw[:8])  # 只发 SOF/TYPE/LEN/3 个 payload 字节 + 1 个 CRC 字节
    time.sleep(0.10)        # > APP_FRAME_TIMEOUT_MS(20ms)
    nack = expect_nack(link, 4, 1.0)
    if nack is None:
        check(False, "残帧后收到 NACK_TIMEOUT")
    else:
        check(nack.type == MSG_CALC_SUM_REQ, f"NACK 里保留了残帧 type=0x{nack.type:02X}")
        check(nack.raw.startswith(raw[:3]), f"NACK 原始字节={nack.raw.hex(' ')}")

    link.send(MSG_PING_REQ)
    check(link.wait_frame({MSG_PONG_RSP}, 1.0) is not None, "残帧后状态机恢复，PING 正常")

    # 残帧后紧接着发新帧（不等超时）：新帧可能被当成残帧剩余字节，但最终要能重新同步
    link.drain()
    link.send_raw(raw[:10])
    time.sleep(0.05)
    nack = expect_nack(link, 4, 1.0)
    check(nack is not None, "第二次残帧也报超时")
    link.send(MSG_PING_REQ)
    check(link.wait_frame({MSG_PONG_RSP}, 1.0) is not None, "第二次残帧后仍可恢复")


def test_crc(link: SerialLink) -> None:
    section("5. CRC32 校验失败")
    link.drain()

    raw = bytearray(encode(MSG_PING_REQ))
    raw[-2] ^= 0xFF  # 改 CRC 最后一个字节，保留合法帧尾
    link.send_raw(bytes(raw))
    nack = expect_nack(link, 2, 1.0)
    if nack is None:
        check(False, "坏 CRC 收到 NACK_CRC")
    else:
        check(nack.code == 2, f"坏 CRC 错误码={nack.code_name}")
        check(nack.type == MSG_PING_REQ, f"NACK 保留 type=0x{nack.type:02X}")

    link.send(MSG_PING_REQ)
    check(link.wait_frame({MSG_PONG_RSP}, 1.0) is not None, "CRC 失败后恢复正常")

    # 坏帧 + 好帧粘在一起：先 NACK，再正常响应
    link.drain()
    bad = bytearray(encode(MSG_CALC_SUM_REQ, calc_payload(1, 1, 1)))
    bad[-2] ^= 0x55
    good = encode(MSG_PING_REQ)
    link.send_raw(bytes(bad) + good)
    nack = expect_nack(link, 2, 1.0)
    check(nack is not None, "粘包中的坏 CRC 被识别")
    check(link.wait_frame({MSG_PONG_RSP}, 1.0) is not None, "坏帧后的好帧仍被处理")


def test_bad_frames(link: SerialLink) -> None:
    section("6. 非法长度 / 非法帧尾 / 未知类型 / payload 长度不符")
    link.drain()

    # LEN > 8
    link.send_raw(bytes([0xA5, MSG_CALC_SUM_REQ, 9]))
    nack = expect_nack(link, 1, 1.0)
    check(nack is not None, "LEN=9 收到 NACK_BAD_LEN")

    link.send(MSG_PING_REQ)
    check(link.wait_frame({MSG_PONG_RSP}, 1.0) is not None, "非法 LEN 后恢复")

    # 非法帧尾：CRC 正确，EOF 改 0x00
    raw = bytearray(encode(MSG_PING_REQ))
    raw[-1] = 0x00
    link.send_raw(bytes(raw))
    nack = expect_nack(link, 3, 1.0)
    check(nack is not None, "错误帧尾收到 NACK_TAIL")

    link.send(MSG_PING_REQ)
    check(link.wait_frame({MSG_PONG_RSP}, 1.0) is not None, "非法帧尾后恢复")

    # 未知类型
    link.send(0x55, b"")
    nack = expect_nack(link, 5, 1.0)
    check(nack is not None, "未知类型收到 NACK_UNKNOWN_TYPE")

    # payload 长度不符（calc 要求 6 字节）
    link.send(MSG_CALC_SUM_REQ, b"\x01\x00\x02\x00\x03")
    nack = expect_nack(link, 6, 1.0)
    check(nack is not None, "calc payload 长度不符收到 NACK_PAYLOAD_LEN")

    link.send(MSG_PING_REQ)
    check(link.wait_frame({MSG_PONG_RSP}, 1.0) is not None, "非法消息后恢复")


def query_stats(link: SerialLink, page: int) -> Optional[StatsPage]:
    link.send(MSG_STATS_REQ, bytes([page]))
    frame = link.wait_frame({MSG_STATS_RSP}, 1.0)
    if frame is None:
        return None
    return decode_stats(frame)


def test_stats(link: SerialLink) -> None:
    section("7. 统计上报")
    link.drain()
    labels = [
        ("page0", ["valid_frames", "calc_frames", "ping_frames"]),
        ("page1", ["crc_err", "len_err", "tail_err"]),
        ("page2", ["timeout_err", "ring_drop_bytes", "uart_err"]),
        ("page3", ["proc_avg_us", "proc_min_us", "proc_max_us"]),
        ("page4", ["tx_frames", "unknown_type_err", "payload_len_err"]),
    ]
    for page, (label, names) in enumerate(labels):
        st = query_stats(link, page)
        if st is None:
            check(False, f"stats page {page} 有响应")
            continue
        check(st.page == page, f"stats page {page} 页号正确")
        print(f"         page{page}: " + ", ".join(f"{n}={v}" for n, v in zip(names, st.values)))

    # valid_frames 应随一次 ping 增加
    before = query_stats(link, 0)
    link.send(MSG_PING_REQ)
    link.wait_frame({MSG_PONG_RSP}, 1.0)
    after = query_stats(link, 0)
    if before and after:
        check(after.values[0] >= before.values[0] + 1,
              f"valid_frames {before.values[0]} -> {after.values[0]}")
    else:
        check(False, "stats 前后查询成功")


@dataclass
class PerfResult:
    count: int = 0
    elapsed_s: float = 0.0
    frames_per_s: float = 0.0
    tx_bytes: int = 0
    rx_bytes: int = 0
    tx_util_pct: float = 0.0
    rx_util_pct: float = 0.0
    rtt_min_ms: float = 0.0
    rtt_avg_ms: float = 0.0
    rtt_max_ms: float = 0.0
    proc_min_us: int = 0
    proc_avg_us: float = 0.0
    proc_max_us: int = 0
    stats_before: dict = field(default_factory=dict)
    stats_after: dict = field(default_factory=dict)


def test_perf(link: SerialLink, count: int, baud: int) -> PerfResult:
    section(f"8. 性能统计：连续 {count} 帧")
    result = PerfResult()
    link.drain()

    before = {p: query_stats(link, p) for p in (0, 1, 2)}
    result.stats_before = {p: (s.values if s else None) for p, s in before.items()}

    # 先测顺序 RTT（30 次），避免和批量吞吐混在一起
    rtts = []
    for _ in range(30):
        t0 = time.perf_counter()
        link.send(MSG_PING_REQ)
        if link.wait_frame({MSG_PONG_RSP}, 1.0) is not None:
            rtts.append((time.perf_counter() - t0) * 1000.0)
    if rtts:
        result.rtt_min_ms = min(rtts)
        result.rtt_avg_ms = statistics.mean(rtts)
        result.rtt_max_ms = max(rtts)
    check(len(rtts) == 30, f"顺序 RTT 30/30 成功（实际 {len(rtts)}）")

    # 准备混合计算请求
    requests = []
    expected = []
    ops = [
        (MSG_CALC_SUM_REQ, MSG_CALC_SUM_RSP),
        (MSG_CALC_SQSUM_REQ, MSG_CALC_SQSUM_RSP),
        (MSG_CALC_WEIGHTED_REQ, MSG_CALC_WEIGHTED_RSP),
    ]
    for i in range(count):
        req_type, rsp_type = ops[i % 3]
        a = (i % 201) - 100
        b = (i % 101) - 50
        c = (i % 51) - 25
        raw = encode(req_type, calc_payload(a, b, c))
        requests.append(raw)
        if req_type == MSG_CALC_SUM_REQ:
            expected.append((rsp_type, a + b + c, False))
        elif req_type == MSG_CALC_SQSUM_REQ:
            expected.append((rsp_type, a * a + b * b + c * c, False))
        else:
            expected.append((rsp_type, a + 2 * b + 3 * c, False))

    proc_us: list[int] = []
    got = 0
    tx_bytes = 0
    rx_bytes = 0
    last_frame_ts = 0.0
    t0 = time.perf_counter()
    batch = count  # 一次发完，测链路/固件吞吐；分批会被 RTT 主导
    try:
        for start in range(0, count, batch):
            chunk = requests[start:start + batch]
            tx_bytes += link.send_many(chunk)
            frames = link.collect_frames(len(chunk), 3.0)
            for frame in frames:
                last_frame_ts = frame.ts
                if frame.type == MSG_NACK:
                    nack = decode_nack(frame)
                    print(f"         [warn] 收到 NACK {nack.code_name} type=0x{nack.type:02X}")
                    continue
                rsp = decode_calc(frame)
                proc_us.append(rsp.proc_us)
                rx_bytes += len(frame.raw)
                idx = got
                exp_type, exp_value, _ = expected[idx]
                if frame.type != exp_type:
                    print(f"         [warn] 第 {idx} 帧类型 {frame.name} 期望 {type_name(exp_type)}")
                elif rsp.result != exp_value:
                    print(f"         [warn] 第 {idx} 帧结果 {rsp.result} 期望 {exp_value}")
                got += 1
    finally:
        # 丢帧时 collect_frames 会等到 3 s 超时；吞吐应按最后一个响应的到达时间算
        elapsed = (last_frame_ts - t0) if last_frame_ts > t0 else (time.perf_counter() - t0)

    result.count = got
    result.elapsed_s = elapsed
    result.frames_per_s = got / elapsed if elapsed > 0 else 0.0
    result.tx_bytes = tx_bytes
    result.rx_bytes = rx_bytes
    result.tx_util_pct = (tx_bytes * 10.0 / elapsed / baud * 100.0) if elapsed > 0 else 0.0
    result.rx_util_pct = (rx_bytes * 10.0 / elapsed / baud * 100.0) if elapsed > 0 else 0.0
    if proc_us:
        result.proc_min_us = min(proc_us)
        result.proc_avg_us = statistics.mean(proc_us)
        result.proc_max_us = max(proc_us)

    check(got == count, f"批量 {count} 帧全部收到响应（实际 {got}）")

    after = {p: query_stats(link, p) for p in (0, 1, 2)}
    result.stats_after = {p: (s.values if s else None) for p, s in after.items()}

    if all(before[p] and after[p] for p in (0, 1, 2)):
        dv = after[0].values[0] - before[0].values[0]
        dcrc = after[1].values[0] - before[1].values[0]
        dlen = after[1].values[1] - before[1].values[1]
        dtail = after[1].values[2] - before[1].values[2]
        dtimeout = after[2].values[0] - before[2].values[0]
        dring = after[2].values[1] - before[2].values[1]
        duart = after[2].values[2] - before[2].values[2]
        check(dv >= got, f"valid_frames 增量 {dv} >= {got}")
        check(dcrc == 0, f"crc_err 增量 {dcrc} (期望 0)")
        check(dlen == 0, f"len_err 增量 {dlen} (期望 0)")
        check(dtail == 0, f"tail_err 增量 {dtail} (期望 0)")
        check(dtimeout == 0, f"timeout_err 增量 {dtimeout} (期望 0)")
        check(dring == 0, f"ring_drop_bytes 增量 {dring} (期望 0)")
        check(duart == 0, f"uart_err 增量 {duart} (期望 0)")

    print()
    print("         结果                值")
    print("         ------------------- -----------------------------")
    print(f"         帧数                {result.count}")
    print(f"         总耗时              {result.elapsed_s * 1000:.1f} ms")
    print(f"         吞吐                {result.frames_per_s:.1f} 帧/s")
    print(f"         TX 字节 / 占用      {result.tx_bytes} B / {result.tx_util_pct:.1f}%")
    print(f"         RX 字节 / 占用      {result.rx_bytes} B / {result.rx_util_pct:.1f}%")
    print(f"         RTT min/avg/max     {result.rtt_min_ms:.2f} / {result.rtt_avg_ms:.2f} / {result.rtt_max_ms:.2f} ms")
    print(f"         固件处理时间        {result.proc_min_us} / {result.proc_avg_us:.1f} / {result.proc_max_us} us")
    return result


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main() -> int:
    parser = argparse.ArgumentParser(description="uart_rx_dma USART1 测试")
    parser.add_argument("--port", help="串口，例如 COM12；不填自动找 CH343")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--list", action="store_true", help="列出串口")
    parser.add_argument("--tests", default="all",
                        help="逗号分隔: ping,calc,sticky,interrupt,crc,bad,stats,perf,all")
    parser.add_argument("--perf-count", type=int, default=500)
    parser.add_argument("--json", default=os.path.join(os.path.dirname(__file__), "test_report.json"))
    args = parser.parse_args()

    if args.list:
        for p in list_serial_ports():
            print(f"{p['device']:<8} {p['description']}  VID:PID={p['vid']:04X}:{p['pid']:04X}"
                  if p["vid"] is not None else f"{p['device']:<8} {p['description']}")
        return 0

    port = args.port or autodetect_port()
    if not port:
        print("找不到串口。可用 --list 查看，或 --port COMx 指定。")
        return 2

    print(f"串口: {port} @ {args.baud} 8N1")
    print(f"时间: {datetime.now().isoformat(timespec='seconds')}")
    link = SerialLink(port, args.baud)
    perf: Optional[PerfResult] = None
    try:
        # 链路自检：等心跳（固件每秒发一次）
        hb = link.wait_frame({MSG_HEARTBEAT}, 2.5, skip_async=False)
        if hb is None:
            print()
            print("没收到心跳。请检查：")
            print("  1) 固件波特率和 --baud 是否一致（本工程默认 921600）")
            print("  2) USB-TTL 的 TX 是否接 PA10、RX 是否接 PA9、GND 是否共地")
            print("  3) 板子是否已复位运行")
            print("尝试发 PING...")
            link.send(MSG_PING_REQ)
            if link.wait_frame({MSG_PONG_RSP}, 1.0) is None:
                print("PING 也无响应，退出。")
                return 2
        else:
            h = decode_heartbeat(hb)
            print(f"心跳: uptime={h.uptime_ms}ms valid={h.valid_frames} uart_err={h.uart_errors}")

        tests = args.tests.strip().lower()
        run_all = tests == "all"
        selected = {t.strip() for t in tests.split(",")} if not run_all else set()

        if hb is not None or True:
            if run_all or "ping" in selected:
                test_ping(link)
            if run_all or "calc" in selected:
                test_calc(link)
            if run_all or "sticky" in selected:
                test_sticky(link)
            if run_all or "interrupt" in selected:
                test_interrupt(link)
            if run_all or "crc" in selected:
                test_crc(link)
            if run_all or "bad" in selected:
                test_bad_frames(link)
            if run_all or "stats" in selected:
                test_stats(link)
            if run_all or "perf" in selected:
                perf = test_perf(link, args.perf_count, args.baud)

    finally:
        link.close()

    section("汇总")
    total = PASS + FAIL
    print(f"通过 {PASS}/{total}，失败 {FAIL}")
    if perf is not None:
        print(f"性能: {perf.frames_per_s:.1f} 帧/s, RTT avg {perf.rtt_avg_ms:.2f} ms, "
              f"固件处理 avg {perf.proc_avg_us:.1f} us")

    report = {
        "port": port,
        "baud": args.baud,
        "time": datetime.now().isoformat(timespec="seconds"),
        "passed": PASS,
        "failed": FAIL,
        "results": RESULTS,
        "perf": perf.__dict__ if perf else None,
    }
    with open(args.json, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(f"报告: {args.json}")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
