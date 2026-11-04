#!/usr/bin/python3
"""Nucleo(UART 版のファームウェア)の USB を PC から見るツール。ROS なしで使える。

USB は Pi との通信(バイナリのパケット)に使うので、ふつうのシリアルモニターでは printf が読めない。
このツールはパケットを解読して、printf の文字と、計測値(駆動輪・計測輪・IMU・状態)を表示する。

  python3 tools/nucleo_monitor.py                    # /dev/ttyACM0(Pi なら /dev/nucleo)
  python3 tools/nucleo_monitor.py /dev/ttyACM1
  python3 tools/nucleo_monitor.py --drive 0.0        # 0 の指令を送り続ける(ウォッチドッグの停止を解く。手動操縦の確認は不要)
  python3 tools/nucleo_monitor.py --drive 3.0        # 全輪 3 rad/s の指令(車輪を浮かせてから!)

表示(0.2 秒ごと): 駆動輪の実速度 [rad/s]、計測輪の累積カウントと rpm、IMU の旋回の速さ [deg/s] と向き [deg]、状態フラグ
取り決めは docs/can_protocol.md の「UART」。
"""
import argparse
import math
import os
import select
import struct
import sys
import termios
import time

TYPE_COMMAND, TYPE_TELEMETRY, TYPE_TEXT = 0x01, 0x10, 0x20
BAUDS = {115200: termios.B115200, 230400: termios.B230400, 460800: termios.B460800, 921600: termios.B921600}


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


def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attr = termios.tcgetattr(fd)
    attr[0] = 0                                  # iflag
    attr[1] = 0                                  # oflag
    attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attr[3] = 0                                  # lflag
    attr[4] = attr[5] = BAUDS[baud]
    attr[6][termios.VMIN] = 0
    attr[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def flags_text(f):
    out = []
    if f & 1:
        out.append("watchdog-stop")
    for m in range(4):
        if f >> (1 + m) & 1:
            out.append(f"C610-ID{m + 1}-lost")
    if f & (1 << 5):
        out.append("IMU-lost")
    return ",".join(out) or "OK"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="/dev/nucleo" if os.path.exists("/dev/nucleo") else "/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=460800, choices=sorted(BAUDS))
    ap.add_argument("--drive", type=float, default=None, help="全輪にこの角速度 [rad/s] の指令を 50Hz で送る(車輪を浮かせてから)")
    args = ap.parse_args()

    fd = open_port(args.port, args.baud)
    print(f"{args.port} {args.baud} bps  (Ctrl+C で終わる)", flush=True)
    buf = bytearray()
    tel = None
    n_tel = n_bad = 0
    t_print = t_cmd = time.monotonic()
    try:
        while True:
            r, _, _ = select.select([fd], [], [], 0.02)
            if r:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    chunk = b""
                for b in chunk:
                    if b != 0:
                        if len(buf) < 512:
                            buf.append(b)
                        continue
                    dec = cobs_decode(bytes(buf))
                    buf.clear()
                    if not dec or len(dec) < 3 or crc16(dec[:-2]) != struct.unpack("<H", dec[-2:])[0]:
                        n_bad += 1
                        continue
                    ptype, data = dec[0], dec[1:-2]
                    if ptype == TYPE_TEXT:
                        print(f"[nucleo] {data.decode(errors='replace')}", flush=True)
                    elif ptype == TYPE_TELEMETRY and len(data) >= 34:
                        tel = data
                        n_tel += 1
            now = time.monotonic()
            if args.drive is not None and now - t_cmd >= 0.02:
                t_cmd = now
                raw = max(-32768, min(32767, round(args.drive * 100)))
                body = bytes([TYPE_COMMAND]) + struct.pack("<4h", raw, raw, raw, raw)
                os.write(fd, cobs_encode(body + struct.pack("<H", crc16(body))) + b"\x00")
            if tel is not None and now - t_print >= 0.2:
                t_print = now
                drive = [v / 100.0 for v in struct.unpack("<4h", tel[0:8])]
                c0, rpm0 = struct.unpack("<if", tel[8:16])
                c1, rpm1 = struct.unpack("<if", tel[16:24])
                rate, heading = struct.unpack("<ff", tel[24:32])
                flags, calib = tel[32], tel[33]
                imu = "NaN" if math.isnan(rate) else f"{math.degrees(rate):+6.1f} deg/s {math.degrees(heading):+7.1f} deg"
                print(f"drive {' '.join(f'{v:+5.1f}' for v in drive)} | odom0 {c0:+8d} ({rpm0:+6.1f} rpm) odom1 {c1:+8d} ({rpm1:+6.1f} rpm) | "
                      f"imu {imu} cal {calib:02X} | {flags_text(flags)} | {n_tel} pkt, {n_bad} bad", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
