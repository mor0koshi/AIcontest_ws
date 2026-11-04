#!/usr/bin/python3
"""chair_detector.py の試験(ROS なし)。A1 をまねたスキャンをランダムな配置で作って、椅子を見つけられるか調べる。

  python3 test_chair_detection.py --trials 100 --seconds 5
"""
import argparse
import math
import os
import random
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import chair_detector  # noqa: E402
import make_map  # noqa: E402
import random_course  # noqa: E402


def ray_hit(ox, oy, a, segs, circles, rmax):
    dx, dy = math.cos(a), math.sin(a)
    best = rmax
    for (x1, y1), (x2, y2) in segs:
        vx, vy = x2 - x1, y2 - y1
        det = dx * vy - dy * vx
        if abs(det) < 1e-12:
            continue
        t = ((x1 - ox) * vy - (y1 - oy) * vx) / det
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / det
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    for cx, cy, r in circles:
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - r * r
        disc = b * b - c
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if t > 0:
                best = min(best, t)
    return best


def simulate_scan(world, pose, rng, seconds, rate=2000, hz=7.0, noise=0.008, dropout=0.03, rmin=0.15, rmax=12.0, pose_err=(0.01, 0.5)):
    """A1 をまねたスキャン: 1秒あたり rate 点、hz 回転、回転ごとに角度の位相がずれる。
    返す点は「ロボットが思っている自分の位置」(pose に誤差を足したもの)で地図座標に直したもの"""
    f = world["field"]
    W, H = f["width"], f["height"]
    segs = [((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))]
    circles = [(world["cone"]["x"], world["cone"]["y"], world["cone"]["radius"])]
    circles += random_course_legs(world)
    x, y, th = pose
    bx, by, bth = x + rng.gauss(0, pose_err[0]), y + rng.gauss(0, pose_err[0]), th + math.radians(rng.gauss(0, pose_err[1]))
    pts = []
    per_rev = int(rate / hz)
    for rev in range(int(seconds * hz)):
        phase = rng.uniform(0, 2 * math.pi / per_rev)
        for k in range(per_rev):
            if rng.random() < dropout:
                continue
            a = phase + 2 * math.pi * k / per_rev
            r = ray_hit(x, y, th + a, segs, circles, rmax)
            if r >= rmax or r < rmin:
                continue
            r += rng.gauss(0, noise)
            pts.append((bx + r * math.cos(bth + a), by + r * math.sin(bth + a)))
    return pts, (bx, by)


def random_course_legs(world):
    sx, sy, leg_r = chair_detector.chair_square(world)
    out = []
    for k in world["chairs"]:
        for qx, qy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
            lx, ly = make_map.rotate(qx * sx / 2, qy * sy / 2, k.get("yaw", 0))
            out.append((k["x"] + lx, k["y"] + ly, leg_r))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=50)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--pose-error", type=float, nargs=2, default=[0.01, 0.5], help="自己位置の誤差 [m, 度](標準偏差)")
    ap.add_argument("--min-legs", type=int, default=3)
    args = ap.parse_args()
    base = yaml.safe_load(open(os.path.join(make_map.package_src_dir(), "config", "map", "course.yaml")))
    rng = random.Random(args.seed)
    found = total = false_pos = 0
    pos_err, yaw_err, legs_hist = [], [], {}
    for t in range(args.trials):
        world = random_course.random_layout(base, rng)
        s = world["start"]
        pts, sensor = simulate_scan(world, (s["x"], s["y"], math.radians(s["yaw"])), rng, args.seconds, pose_err=tuple(args.pose_error))
        dets = chair_detector.detect_chairs(pts, world, sensor, min_legs=args.min_legs)
        used = set()
        for k in world["chairs"]:
            total += 1
            best = None
            for i, d in enumerate(dets):
                e = math.hypot(d["x"] - k["x"], d["y"] - k["y"])
                if i not in used and e < 0.08 and (best is None or e < best[1]):
                    best = (i, e)
            if best:
                used.add(best[0])
                found += 1
                d = dets[best[0]]
                pos_err.append(best[1])
                ye = (d["yaw_deg"] - k["yaw"] % 90.0 + 45.0) % 90.0 - 45.0
                yaw_err.append(abs(ye))
                legs_hist[d["legs"]] = legs_hist.get(d["legs"], 0) + 1
        false_pos += len(dets) - len(used)
    pe = sorted(pos_err) or [0]
    ye = sorted(yaw_err) or [0]
    print(f"{args.trials} layouts, scan {args.seconds} s from the start pose, pose error {args.pose_error}, min legs {args.min_legs}")
    print(f"chairs found: {found}/{total} ({100.0 * found / total:.1f}%), false detections: {false_pos}")
    print(f"position error: mean {100 * sum(pe) / len(pe):.1f} cm, 95% {100 * pe[int(0.95 * (len(pe) - 1))]:.1f} cm, max {100 * pe[-1]:.1f} cm")
    print(f"yaw error: mean {sum(ye) / len(ye):.1f} deg, 95% {ye[int(0.95 * (len(ye) - 1))]:.1f} deg, max {ye[-1]:.1f} deg")
    print(f"legs used per detection: {dict(sorted(legs_hist.items()))}")


if __name__ == "__main__":
    main()
