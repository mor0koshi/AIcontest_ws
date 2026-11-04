#!/usr/bin/env python3
"""Nucleo の代わりに CAN に応答するテスト用スクリプト(docs/can_protocol.md v0.3)。

  0x300(目標角速度)を受け取り、10 ms ごとに 0x310〜0x314 を返す(計測輪2つ: 0x311 前後向き・0x312 横向き、0x313 IMU)。
  駆動輪は一次遅れで目標に追従し、機体の動き → 計測輪の回転 をメカナム/オムニの式で計算する
  (寸法は aicontest_rover の URDF の仮の値と同じ)。

  python3 fake_nucleo.py vcan0
  python3 fake_nucleo.py vcan0 --c610-traffic      # C610 のフィードバック(0x201〜0x204 を各1kHz)も流す
  python3 fake_nucleo.py vcan0 --drop 0.05         # 5% のフレームを送らない(取りこぼしの再現)
"""
import argparse
import math
import random
import socket
import struct
import time

CAN_FRAME = struct.Struct("<IB3x8s")

# ---- 機体の寸法(urdf/rover.urdf.xacro の仮の値)----
MECANUM_R_EFF = 0.03 / math.sqrt(2)  # メカナム半径 / √2
WX, WY = 0.06, 0.085
DRIVE = [  # (x, y, ローラー軸の向き) 0x300 の並び = fl, fr, rl, rr
    (WX, WY, -math.pi / 4),
    (WX, -WY, math.pi / 4),
    (-WX, WY, math.pi / 4),
    (-WX, -WY, -math.pi / 4),
]
ODOM_R = 0.024
ODOM = [(0.0, 0.05, 0.0), (-0.06, 0.0, math.pi / 2)]  # 前後向き(l)、横向き(s)
GYRO_BIAS = math.radians(0.3)  # IMU のずれ(Pi 側で校正されることの確認用)
COUNTS_PER_REV = 1440.0  # aicontest_can_bridge の odom_counts_per_rev と合わせる(E6B2-CWZ6C 360 P/R × 4)


def row(x, y, yaw):
    """omni の式: 車輪の周速 = row · [vx, vy, wz]"""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c, s, -c * y + s * x)


def solve3(a, b):
    """3x3 連立方程式(ガウスの消去法)"""
    m = [list(a[i]) + [b[i]] for i in range(3)]
    for i in range(3):
        p = max(range(i, 3), key=lambda k: abs(m[k][i]))
        m[i], m[p] = m[p], m[i]
        for k in range(3):
            if k != i:
                f = m[k][i] / m[i][i]
                m[k] = [m[k][j] - f * m[i][j] for j in range(4)]
    return [m[i][3] / m[i][i] for i in range(3)]


def body_twist(wheel_rad_s):
    """駆動輪の角速度 → 機体速度(最小二乗)"""
    rows = [row(*d) for d in DRIVE]
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(3)] for i in range(3)]
    atb = [sum(r[i] * w * MECANUM_R_EFF for r, w in zip(rows, wheel_rad_s)) for i in range(3)]
    return solve3(ata, atb)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("interface", nargs="?", default="vcan0")
    ap.add_argument("--c610-traffic", action="store_true", help="0x201〜0x204 を各1kHzで流す")
    ap.add_argument("--drop", type=float, default=0.0, help="Nucleo→Pi フレームを送らない確率")
    ap.add_argument("--tau", type=float, default=0.05, help="駆動輪の追従の時定数[s]")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.interface,))
    sock.setblocking(False)

    def send(can_id, data):
        if args.drop > 0 and random.random() < args.drop:
            return
        sock.send(CAN_FRAME.pack(can_id, len(data), data.ljust(8, b"\0")))

    target = [0.0] * 4
    actual = [0.0] * 4
    counts = [0.0] * len(ODOM)
    heading = 0.0
    last_cmd = 0.0
    dt_ctrl = 0.001
    t_next_ctrl = time.monotonic()
    tick = 0
    print(f"fake_nucleo on {args.interface} (c610_traffic={args.c610_traffic}, drop={args.drop})", flush=True)

    while True:
        # 受信(0x300 のみ使う)
        while True:
            try:
                raw = sock.recv(16)
            except BlockingIOError:
                break
            can_id, dlc, data = CAN_FRAME.unpack(raw)
            if (can_id & 0x7FF) == 0x300 and dlc >= 8:
                target = [v / 100.0 for v in struct.unpack("<4h", data)]
                last_cmd = time.monotonic()

        now = time.monotonic()
        if now < t_next_ctrl:
            time.sleep(max(0.0, t_next_ctrl - now))
            continue
        t_next_ctrl += dt_ctrl
        tick += 1

        # 1 kHz: ウォッチドッグ、駆動輪の追従、計測輪の積算
        watchdog = (time.monotonic() - last_cmd) > 0.1
        goal = [0.0] * 4 if watchdog else target
        a = dt_ctrl / (args.tau + dt_ctrl)
        actual = [x + a * (g - x) for x, g in zip(actual, goal)]
        vx, vy, wz = body_twist(actual)
        odom_rad_s = [sum(c * v for c, v in zip(row(*o), (vx, vy, wz))) / ODOM_R for o in ODOM]
        counts = [c + w * dt_ctrl / (2 * math.pi) * COUNTS_PER_REV for c, w in zip(counts, odom_rad_s)]

        if args.c610_traffic:
            for mid in range(4):
                send(0x201 + mid, struct.pack(">hhhB", 0, int(actual[mid] * 36 * 60 / (2 * math.pi)), 0, 30))

        # 100 Hz: Pi へ 0x310 → 0x311 → 0x312 → 0x313 → 0x314
        heading = (heading + wz * dt_ctrl + math.pi) % (2 * math.pi) - math.pi
        if tick % 10 == 0:
            send(0x310, struct.pack("<4h", *[max(-32768, min(32767, round(v * 100))) for v in actual]))
            for i in range(len(ODOM)):
                rpm = odom_rad_s[i] * 60 / (2 * math.pi)
                send(0x311 + i, struct.pack("<if", int(counts[i]), rpm))
            send(0x313, struct.pack("<ff", wz + GYRO_BIAS + random.gauss(0, 0.003), heading))  # IMU
            send(0x314, struct.pack("<BB", 1 if watchdog else 0, 0xF0))  # CALIB_STAT: sys 3, gyro 3


if __name__ == "__main__":
    main()
