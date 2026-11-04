#!/usr/bin/env python3
"""Nucleo の代わりに UART(仮想シリアル)で応答するテスト用スクリプト(docs/can_protocol.md v0.4 の「UART」)。

  python3 fake_nucleo_serial.py                       # /tmp/fake_nucleo に仮想シリアルを作る
  ros2 run aicontest_can_bridge serial_bridge_node --ros-args -p port:=/tmp/fake_nucleo

  COMMAND パケット(駆動輪の目標角速度)を受け取り、10 ms ごとに TELEMETRY パケット(駆動輪・計測輪2つ・IMU・状態)を返す。
  0.5 秒ごとに TEXT パケット(printf の代わり)も送る。機体の動きの計算は fake_nucleo.py(CAN 版)と同じ。
  --corrupt 0.05 で 5% のパケットを 1 バイト壊す(CRC で捨てられることの確認)
"""
import argparse
import math
import os
import random
import select
import struct
import time
import tty

from fake_nucleo import COUNTS_PER_REV, GYRO_BIAS, ODOM, ODOM_R, body_twist, row

TYPE_COMMAND, TYPE_TELEMETRY, TYPE_TEXT = 0x01, 0x10, 0x20


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data):
    out, code_pos, code = bytearray([0]), 0, 1
    for b in data:
        if b == 0:
            out[code_pos] = code
            code_pos, code = len(out), 1
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_pos] = code
                code_pos, code = len(out), 1
                out.append(0)
    out[code_pos] = code
    return bytes(out)


def cobs_decode(data):
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0:
            return None
        for _ in range(code - 1):
            if i >= len(data):
                return None
            out.append(data[i])
            i += 1
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def packet(ptype, data):
    raw = bytes([ptype]) + data
    return cobs_encode(raw + struct.pack("<H", crc16(raw))) + b"\x00"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link", default="/tmp/fake_nucleo", help="仮想シリアルへのリンクを作る場所")
    ap.add_argument("--tau", type=float, default=0.05, help="駆動輪の追従の時定数[s]")
    ap.add_argument("--corrupt", type=float, default=0.0, help="送るパケットを壊す確率")
    args = ap.parse_args()

    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    slave_name = os.ttyname(slave)
    if os.path.islink(args.link) or os.path.exists(args.link):
        os.remove(args.link)
    os.symlink(slave_name, args.link)
    print(f"fake_nucleo_serial: {args.link} -> {slave_name}", flush=True)

    target, actual = [0.0] * 4, [0.0] * 4
    counts = [0.0] * len(ODOM)
    heading, last_cmd = 0.0, 0.0
    rx, n_cmd = bytearray(), 0
    dt = 0.001
    t_next = time.monotonic()
    tick = 0
    while True:
        r, _, _ = select.select([master], [], [], 0)
        if r:
            try:
                chunk = os.read(master, 4096)
            except OSError:
                chunk = b""
            for b in chunk:
                if b != 0:
                    rx.append(b)
                    continue
                dec = cobs_decode(bytes(rx))
                rx.clear()
                if dec is None or len(dec) < 3 or crc16(dec[:-2]) != struct.unpack("<H", dec[-2:])[0]:
                    continue
                if dec[0] == TYPE_COMMAND and len(dec) == 11:
                    target = [v / 100.0 for v in struct.unpack("<4h", dec[1:9])]
                    last_cmd = time.monotonic()
                    n_cmd += 1
        now = time.monotonic()
        if now < t_next:
            time.sleep(min(0.0005, t_next - now))
            continue
        t_next += dt
        tick += 1
        watchdog = (time.monotonic() - last_cmd) > 0.1
        goal = [0.0] * 4 if watchdog else target
        a = dt / (args.tau + dt)
        actual = [x + a * (g - x) for x, g in zip(actual, goal)]
        vx, vy, wz = body_twist(actual)
        odom_rad_s = [sum(c * v for c, v in zip(row(*o), (vx, vy, wz))) / ODOM_R for o in ODOM]
        counts = [c + w * dt / (2 * math.pi) * COUNTS_PER_REV for c, w in zip(counts, odom_rad_s)]
        heading = (heading + wz * dt + math.pi) % (2 * math.pi) - math.pi
        out = b""
        if tick % 10 == 0:
            tel = struct.pack("<4h", *[max(-32768, min(32767, round(v * 100))) for v in actual])
            for i in range(len(ODOM)):
                tel += struct.pack("<if", int(counts[i]), odom_rad_s[i] * 60 / (2 * math.pi))
            tel += struct.pack("<ff", wz + GYRO_BIAS + random.gauss(0, 0.003), heading)
            tel += struct.pack("<BB", 1 if watchdog else 0, 0x30)
            pkt = bytearray(packet(TYPE_TELEMETRY, tel))
            if args.corrupt > 0 and random.random() < args.corrupt:
                k = random.randrange(len(pkt) - 1)
                pkt[k] = (pkt[k] ^ 0x5A) or 1
            out += pkt
        if tick % 500 == 0:
            out += packet(TYPE_TEXT, f"fake nucleo: commands {n_cmd}, target {[round(t, 2) for t in target]}".encode())
        if out:
            try:
                os.write(master, out)
            except OSError:
                pass


if __name__ == "__main__":
    main()
