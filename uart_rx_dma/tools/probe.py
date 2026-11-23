"""快速硬件探测：在指定波特率周期性发 PING，打印收到的原始字节。

用法:
    python tools/probe.py --port COM12 --baud 921600 --seconds 2.5 --repeat 20
"""
import argparse
import struct
import sys
import time
import zlib

import serial


def frame(ftype: int, payload: bytes = b"") -> bytes:
    body = bytes([ftype, len(payload)]) + payload
    crc = zlib.crc32(body) & 0xFFFFFFFF
    return bytes([0xA5]) + body + struct.pack("<I", crc) + bytes([0x5A])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM12")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=0.8)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--interval", type=float, default=0.1)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1)
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        deadline = time.monotonic() + args.seconds
        next_tx = time.monotonic()
        sent = 0
        data = bytearray()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if sent < args.repeat and now >= next_tx:
                ser.write(frame(0x10))
                ser.flush()
                sent += 1
                next_tx = now + args.interval
            chunk = ser.read(256)
            if chunk:
                data.extend(chunk)
        print(f"{args.port} @ {args.baud}: sent {sent} ping, got {len(data)} bytes")
        print(data.hex(" "))
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
