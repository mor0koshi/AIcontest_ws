#!/usr/bin/python3
"""ルート(make_map.py が作る state_graph の course.md)の検査。test_routes.py と make_map.py(ルートの自動修正)が使う。

check_route(c, path) → {"errs": [...], "warns": [...], "n": set_pose の数, "implicated": {関係する椅子の番号(0始まり)}}
  errs(problems):
    1. 続く2つの set_pose の位置が 0.1m 未満(natto の経路計画が「前と同じ目標」とみなすことがある)
    2. 途中の点(_向き・_手前・_離れる・_よける・_回る)が壁・クレーター・コーン・椅子に近すぎる
    3. 続く2つの点を結ぶ直線が、両端より椅子に近づく(入口 → 出口は通り抜ける椅子を除く。holonomic モードは調べない)
    4. 椅子ごとに 入口 → 出口 がそろっていて、椅子の中心をはさんでいる
  warns(warning):
    直線がクレーターを横切り、そのクレーターのすぐそばに椅子がある(A* がクレーターを回り込むとき、椅子とのすき間を通ることがある)
"""
import math
import re

import make_map

POSE = re.compile(r"--> (\S+) : set_pose\(x = ([-\d.]+), y = ([-\d.]+), yaw = ([-\d.]+)\)")
CHAIR_NAME = re.compile(r"椅子(\d+)_")


def _chairs_in(*names):
    out = set()
    for n in names:
        m = CHAIR_NAME.match(n)
        if m:
            out.add(int(m.group(1)) - 1)
    return out


def check_route(c, path):
    errs, warns, implicated = [], [], set()
    poses = [(m.group(1), float(m.group(2)), float(m.group(3)), float(m.group(4))) for m in POSE.finditer(open(path).read())]
    s = c["start"]
    start = ("スタート", s["x"], s["y"], s.get("yaw", 0))
    prev = start
    for p in poses:
        d = math.hypot(p[1] - prev[1], p[2] - prev[2])
        if d < 0.1:
            errs.append(f"{prev[0]} → {p[0]}: only {d * 100:.1f} cm apart")
            implicated |= _chairs_in(prev[0], p[0])
        prev = p
    margin = float(c.get("clearance_margin", 0.03))
    for name, x, y, _ in poses:
        if name.endswith(("_向き", "_手前", "_手前2", "_離れる", "_よける")) or "_回る" in name:
            g = make_map.pose_clearance(c, x, y)
            # 入口の手前(軸の上)と、出口から軸に沿って離れる点は、余白なしでも置く(回るだけなので)
            need = 0.0 if name.endswith(("入口_手前", "入口_手前2", "_離れる")) else margin
            if g < need - 0.002:  # 座標は小数3桁に丸めて書くので 2mm の許容
                errs.append(f"{name} ({x:.2f},{y:.2f}) clearance {g * 100:.1f} cm")
                implicated |= _chairs_in(name)
    half = make_map.chair_half(c)
    chairs = c.get("chairs") or []

    def chair_gap(x, y, skip):  # 椅子(skip 以外)との最小のすき間と、その椅子。壁・クレーター・コーンは A* が避けるので見ない
        return min(((make_map.dist_point_square(x, y, k["x"], k["y"], k.get("yaw", 0), half) - make_map.ROBOT_R, j)
                    for j, k in enumerate(chairs) if j != skip), default=(9.0, None))

    forward = make_map.forward_mode(c)
    prev = start
    for p in (poses if forward else []):  # holonomic モードは途中の点を置かず A* に任せる
        m = re.match(r"椅子(\d+)_出口$", p[0])
        skip = int(m.group(1)) - 1 if m else None
        a, b = prev[1:3], p[1:3]
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.01))
        lim = min(0.0, chair_gap(*a, skip)[0], chair_gap(*b, skip)[0]) - 0.01
        g, j = min((chair_gap(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, skip) for k in range(1, n)),
                   default=(9.0, None))
        if g < lim:
            errs.append(f"{prev[0]} → {p[0]}: straight line comes {g * 100:.1f} cm close to chair {j + 1} (ends {lim * 100 + 1:.1f})")
            implicated |= _chairs_in(prev[0], p[0]) | {j}
        prev = p
    r_crater = float(c["crater_radius"]) + float(c.get("crater_margin", 0.0))
    prev = start
    for p in (poses if forward else []):
        a, b = prev[1:3], p[1:3]
        vx, vy = b[0] - a[0], b[1] - a[1]
        L2 = vx * vx + vy * vy
        for cx, cy in c.get("craters") or []:
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((cx - a[0]) * vx + (cy - a[1]) * vy) / L2))
            if math.hypot(a[0] + t * vx - cx, a[1] + t * vy - cy) >= r_crater + make_map.ROBOT_R:
                continue
            for j, k in enumerate(chairs):
                gap = make_map.dist_point_square(cx, cy, k["x"], k["y"], k.get("yaw", 0), half) - r_crater
                if gap < 2 * make_map.ROBOT_R + 0.10:
                    warns.append(f"{prev[0]} → {p[0]}: crosses the crater ({cx:.2f},{cy:.2f}); chair {j + 1} is only "
                                 f"{gap * 100:.0f} cm from it (A* may squeeze between them)")
                    implicated |= _chairs_in(prev[0], p[0]) | {j}
        prev = p
    names = [p[0] for p in poses]
    for i, k in enumerate(chairs, start=1):
        ent = [p for p in poses if p[0] == f"椅子{i}_入口"]
        ext = [p for p in poses if p[0] == f"椅子{i}_出口"]
        if len(ent) != 1 or len(ext) != 1:
            errs.append(f"chair {i}: entry/exit missing")
            continue
        (_, ex, ey, _), (_, xx, xy, _) = ent[0], ext[0]
        mx, my = (ex + xx) / 2, (ey + xy) / 2
        if math.hypot(mx - k["x"], my - k["y"]) > 0.01:
            errs.append(f"chair {i}: entry/exit not centered on the chair")
        if names.index(f"椅子{i}_出口") != names.index(f"椅子{i}_入口") + 1:
            errs.append(f"chair {i}: something between entry and exit")
    return {"errs": errs, "warns": warns, "n": len(poses), "implicated": implicated}
