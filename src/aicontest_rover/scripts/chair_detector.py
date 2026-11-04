#!/usr/bin/python3
"""LiDAR の点から椅子(四つ足)を見つける(案1: スタート時に全体をスキャンして、椅子の位置と向きを出す)。

  points: 地図座標の点 [(x, y), ...](スタート位置で数秒分のスキャンを、自己位置で地図座標に直して貯めたもの)
  detect_chairs(points, course, sensor_xy) → [{"x", "y", "yaw", "legs"}, ...](見つかった脚の数が多い順)

手順:
  1. 壁・コーン(地図に載っている、LiDAR に映る物)の近くの点を除く
  2. 残りの点を 3cm でつながるまとまりに分け、大きさが脚くらい(半径 3cm 以下)のものを「脚の候補」にする
     (点は脚の表面の LiDAR 側に乗るので、脚の半径だけ奥にずらして中心にする)
  3. 脚の候補から、椅子の脚の配置(1辺 0.33m の正方形、course.yaml の chair から計算)に並ぶ組を探す
     - 2本の間隔が 1辺 → 正方形は2通り(どちら側に伸びるか)/ 対角線 → 1通り
     - 予想した残りの角に脚の候補があれば数える。3本以上見えたものを椅子とする
  4. 近い仮説はまとめ、脚の位置に正方形を当てはめ直して中心と向き(0〜90°)を出す

ROS には依存しない(テスト: test_chair_detection.py)。
"""
import math


def chair_square(course):
    ch = course.get("chair", {})
    leg = float(ch.get("leg_size", 0.02))
    side_x = float(ch.get("inner_x", 0.31)) + leg
    side_y = float(ch.get("inner_y", 0.31)) + leg
    return side_x, side_y, leg / 2.0


def _seg_dist(px, py, x1, y1, x2, y2):
    vx, vy = x2 - x1, y2 - y1
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x1) * vx + (py - y1) * vy) / L2))
    return math.hypot(x1 + t * vx - px, y1 + t * vy - py)


def remove_known(points, course, margin=0.04):
    """壁・コーンの近くの点と、コースの外の点を除く"""
    f = course["field"]
    W, H = float(f["width"]), float(f["height"])
    walls = [((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))]
    cone = course.get("cone")
    out = []
    for x, y in points:
        if not (margin < x < W - margin and margin < y < H - margin):
            continue
        if any(_seg_dist(x, y, a[0], a[1], b[0], b[1]) < margin for a, b in walls):
            continue
        if cone and math.hypot(x - cone["x"], y - cone["y"]) < cone["radius"] + margin:
            continue
        out.append((x, y))
    return out


def cluster(points, link=0.03):
    """link 以内でつながる点のまとまり(格子で近所だけ調べる)"""
    cell = {}
    for i, (x, y) in enumerate(points):
        cell.setdefault((int(x // link), int(y // link)), []).append(i)
    seen = [False] * len(points)
    groups = []
    for i in range(len(points)):
        if seen[i]:
            continue
        seen[i] = True
        stack, g = [i], []
        while stack:
            k = stack.pop()
            g.append(k)
            cx, cy = int(points[k][0] // link), int(points[k][1] // link)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for j in cell.get((cx + dx, cy + dy), []):
                        if not seen[j] and math.hypot(points[j][0] - points[k][0], points[j][1] - points[k][1]) <= link:
                            seen[j] = True
                            stack.append(j)
        groups.append([points[k] for k in g])
    return groups


def leg_candidates(points, sensor_xy, leg_r, min_points=3, max_radius=0.035, max_extent=0.10):
    """小さなまとまり → 脚の中心の候補 [(x, y, 点の数)]
    何回分ものスキャンを重ねると、距離のノイズで脚の点が奥行き方向に ±3cm ほど散る。いちばん遠い点ではなく
    8割の点が中心から max_radius 以内か(と、全体の広がりが max_extent 以内か)で判定する(ROS 上のシミュレーションで、
    いちばん遠い点で判定したら脚が1本も残らなかった)"""
    cands = []
    for g in cluster(points):
        if len(g) < min_points:
            continue
        mx = sorted(p[0] for p in g)[len(g) // 2]
        my = sorted(p[1] for p in g)[len(g) // 2]
        dist = sorted(math.hypot(p[0] - mx, p[1] - my) for p in g)
        if dist[int(0.8 * (len(dist) - 1))] > max_radius or dist[-1] > max_extent:
            continue
        inl = [p for p in g if math.hypot(p[0] - mx, p[1] - my) <= max_radius]
        mx = sum(p[0] for p in inl) / len(inl)
        my = sum(p[1] for p in inl) / len(inl)
        vx, vy = mx - sensor_xy[0], my - sensor_xy[1]
        r = math.hypot(vx, vy) or 1.0
        cands.append((mx + leg_r * vx / r, my + leg_r * vy / r, len(g)))
    return cands


def _fit_square(legs, side):
    """脚の位置(角の番号つき)に正方形を当てはめて、中心と向きを出す。legs: [(corner_index, x, y)]"""
    h = side / 2.0
    template = {0: (h, h), 1: (-h, h), 2: (-h, -h), 3: (h, -h)}
    # 向き: 各脚の「中心からの角度 − テンプレートの角度」の平均(中心は脚の平均で近似してから求め直す)
    cx = sum(x for _, x, _ in legs) / len(legs)
    cy = sum(y for _, _, y in legs) / len(legs)
    for _ in range(3):
        s = c = 0.0
        for k, x, y in legs:
            tx, ty = template[k]
            a = math.atan2(y - cy, x - cx) - math.atan2(ty, tx)
            s += math.sin(a)
            c += math.cos(a)
        yaw = math.atan2(s, c)
        cs, sn = math.cos(yaw), math.sin(yaw)
        cx = sum(x - (cs * template[k][0] - sn * template[k][1]) for k, x, _ in legs) / len(legs)
        cy = sum(y - (sn * template[k][0] + cs * template[k][1]) for k, _, y in legs) / len(legs)
    rms = math.sqrt(sum((x - (cx + cs * template[k][0] - sn * template[k][1])) ** 2 + (y - (cy + sn * template[k][0] + cs * template[k][1])) ** 2
                        for k, x, y in legs) / len(legs))
    return cx, cy, yaw, rms


def detect_chairs(points, course, sensor_xy, tol=0.035, min_legs=3):
    side_x, side_y, leg_r = chair_square(course)
    side = (side_x + side_y) / 2.0
    pts = remove_known(points, course)
    cands = leg_candidates(pts, sensor_xy, leg_r)
    hyps = []
    n = len(cands)
    for i in range(n):
        for j in range(i + 1, n):
            ax, ay, _ = cands[i]
            bx, by, _ = cands[j]
            d = math.hypot(bx - ax, by - ay)
            corners = []
            if abs(d - side) < tol:  # 辺 → 正方形は2通り
                ux, uy = (bx - ax) / d, (by - ay) / d
                for sgn in (1, -1):
                    nx, ny = -uy * sgn * side, ux * sgn * side
                    corners.append([(ax, ay), (bx, by), (bx + nx, by + ny), (ax + nx, ay + ny)])
            elif abs(d - side * math.sqrt(2)) < tol * 1.4:  # 対角線 → 1通り
                mx, my = (ax + bx) / 2, (ay + by) / 2
                hx, hy = (bx - ax) / 2, (by - ay) / 2
                corners.append([(ax, ay), (mx - hy, my + hx), (bx, by), (mx + hy, my - hx)])
            for sq in corners:
                # 角の並びを反時計回りにそろえる(テンプレートと同じ向き)
                area = sum(sq[k][0] * sq[(k + 1) % 4][1] - sq[(k + 1) % 4][0] * sq[k][1] for k in range(4))
                if area < 0:
                    sq = [sq[0], sq[3], sq[2], sq[1]]
                matched = []
                for k, (qx, qy) in enumerate(sq):
                    best = min(cands, key=lambda c: math.hypot(c[0] - qx, c[1] - qy))
                    if math.hypot(best[0] - qx, best[1] - qy) < tol:
                        matched.append((k, best[0], best[1]))
                if len(matched) >= min_legs:
                    cx, cy, yaw, rms = _fit_square(matched, side)
                    hyps.append({"x": cx, "y": cy, "yaw": yaw, "legs": len(matched), "rms": rms})
    # 近い仮説をまとめる(脚の数が多く、当てはまりが良いものを残す)
    hyps.sort(key=lambda h: (-h["legs"], h["rms"]))
    chairs = []
    for h in hyps:
        if all(math.hypot(h["x"] - c["x"], h["y"] - c["y"]) > side * 0.6 for c in chairs):
            chairs.append(h)
    for c in chairs:
        c["yaw_deg"] = math.degrees(c["yaw"]) % 90.0  # 正方形なので 90° ごとに同じ
    return chairs
