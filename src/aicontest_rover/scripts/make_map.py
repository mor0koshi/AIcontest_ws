#!/usr/bin/python3
"""config/map/course.yaml から、地図とルートを作る。

  ros2 run aicontest_rover make_map.py
    → config/map/line_segments.csv, config/map/circles.csv(natto_map 用の地図)
    → state_graph/course.md(natto_state_machine 用のルート)

  ros2 run aicontest_rover make_map.py course.yaml   # 入力を指定(出力先は同じ)

natto の地図の高さ(z)の決まり:
  線分: z は「下端〜上端」。どちらかが0なら経路計画の地図に入る。
        LiDARの高さが下端〜上端の間にあれば LiDAR シミュレーターに映る。
  円:   z=0 なら経路計画の地図に入り、LiDAR シミュレーターにも映る。
→ 壁は z 0〜壁の高さ の線分、クレーター(床)は z 0〜0 の線分の多角形、
  コーンと椅子の脚は z=0 の円にする。

ルート: スタートで5秒待つ → 椅子ごとに 入口 → 出口(椅子の真下を一直線に通り抜ける)→ ゴール
  入口で向きをそろえ、椅子の中はロボットの前後方向だけで進む(メカナムは真横に滑りやすいため)
  drive_mode: forward(既定)では、椅子と椅子の間も進む向きを向いてから前進(後退)する。途中の点:
    _離れる   出口から軸に沿って離れる          _よける  出口から椅子の横の面に沿って離れる
    _回る1〜5 椅子の外側を回る                  _向き    進む向きへ回りながら少し進む
    _手前2    入口の 0.30m 手前(軸の上)       _手前    入口の 0.15m 手前(突入の向き。ここで止まって脚を測る)
  auto_route: true なら、回る順番と通り抜ける向きを動的計画法で決める(移動距離・回転・椅子やクレーターを
  横切る回り道・窮屈な通り方のコスト)
"""
import math
import os
import sys

import yaml


def package_src_dir():
    """symlink-install でもソースを指すように、このスクリプトの場所からたどる"""
    here = os.path.realpath(__file__)
    pkg = os.path.dirname(os.path.dirname(here))  # scripts/ の1つ上
    if os.path.isdir(os.path.join(pkg, "config", "map")):
        return pkg
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory("aicontest_rover")


def rotate(x, y, deg):
    t = math.radians(deg)
    return x * math.cos(t) - y * math.sin(t), x * math.sin(t) + y * math.cos(t)


def make_map(c, out_dir):
    f = c["field"]
    W, H, WALL = float(f["width"]), float(f["height"]), float(f["wall_height"])
    segs = []
    for (x1, y1), (x2, y2) in [((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))]:
        segs.append((x1, y1, 0.0, x2, y2, WALL))

    # クレーターは crater_margin だけ大きく描く(経路計画は外形が縁に重ならないことしか見ないので、
    # 測定の誤差や自己位置の誤差で触れないように余白をとる)
    r = float(c["crater_radius"]) + float(c.get("crater_margin", 0.0))
    for cx, cy in c.get("craters") or []:
        n = 16
        pts = [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n)) for i in range(n + 1)]
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            segs.append((x1, y1, 0.0, x2, y2, 0.0))

    circles = []
    cone = c.get("cone")
    if cone:
        circles.append((cone["x"], cone["y"], cone["radius"]))

    ch = c.get("chair", {})
    leg = float(ch.get("leg_size", 0.02))
    hx = float(ch.get("inner_x", 0.31)) / 2 + leg / 2  # 脚の中心までの距離
    hy = float(ch.get("inner_y", 0.31)) / 2 + leg / 2
    for chair in c.get("chairs") or []:
        for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
            lx, ly = rotate(sx * hx, sy * hy, chair.get("yaw", 0))
            circles.append((chair["x"] + lx, chair["y"] + ly, leg / 2))

    with open(os.path.join(out_dir, "line_segments.csv"), "w") as fp:
        fp.write("start_x,start_y,start_z,end_x,end_y,end_z\n")
        for s in segs:
            fp.write(",".join(f"{v:.4f}" for v in s) + "\n")
    with open(os.path.join(out_dir, "circles.csv"), "w") as fp:
        fp.write("center_x,center_y,center_z,radius,start_angle,end_angle\n")
        for x, y, rad in circles:
            fp.write(f"{x:.4f},{y:.4f},0,{rad:.4f},0,6.283185307\n")
    return len(segs), len(circles)


def angle_diff(a, b):
    """a - b を -180〜180 度に"""
    return (a - b + 180.0) % 360.0 - 180.0


# ==================== ルートの自動計画 ====================
ROBOT_R = math.hypot(0.09, 0.10)  # 機体(180x200mm)を囲む円の半径。向きによらず安全側
CHAIR_HALF = 0.31 / 2 + 0.02       # 椅子の外形の半分(course.yaml の値で上書き)


def chair_half(c):
    ch = c.get("chair", {})
    leg = float(ch.get("leg_size", 0.02))
    return max(float(ch.get("inner_x", 0.31)), float(ch.get("inner_y", 0.31))) / 2 + leg


def dist_point_square(px, py, cx, cy, yaw_deg, half):
    """点と、中心(cx,cy)・向き yaw・半辺 half の正方形との距離(内側なら 0)"""
    lx, ly = rotate(px - cx, py - cy, -yaw_deg)
    return math.hypot(max(abs(lx) - half, 0.0), max(abs(ly) - half, 0.0))


def pose_clearance(c, x, y, skip_chair=None):
    """機体の中心が (x,y) のときの、壁・クレーター・コーン・ほかの椅子との最小のすき間 [m]"""
    f = c["field"]
    W, H = float(f["width"]), float(f["height"])
    gaps = [x - ROBOT_R, W - x - ROBOT_R, y - ROBOT_R, H - y - ROBOT_R]
    r = float(c["crater_radius"]) + float(c.get("crater_margin", 0.0))
    gaps += [math.hypot(x - cx, y - cy) - r - ROBOT_R for cx, cy in c.get("craters") or []]
    cone = c.get("cone")
    if cone:
        gaps.append(math.hypot(x - cone["x"], y - cone["y"]) - cone["radius"] - ROBOT_R)
    half = chair_half(c)
    for j, ch in enumerate(c.get("chairs") or []):
        if j != skip_chair:
            gaps.append(dist_point_square(x, y, ch["x"], ch["y"], ch.get("yaw", 0), half) - ROBOT_R)
    return min(gaps)


def segment_crater_penalty(c, a, b):
    """a→b の直線がクレーターを横切るなら、回り道の分を足す(だいたいの値)"""
    r = float(c["crater_radius"]) + float(c.get("crater_margin", 0.0)) + ROBOT_R
    pen = 0.0
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    for cx, cy in c.get("craters") or []:
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((cx - ax) * vx + (cy - ay) * vy) / L2))
        d = math.hypot(ax + t * vx - cx, ay + t * vy - cy)
        if d < r:
            pen += 2.0 * (r - d)
    return pen


def segment_chair_penalty(c, a, b):
    """a→b の直線が椅子を横切る(余白 clearance_margin を保てない)なら、回り道の分を足す(だいたいの値)。
    両端がもともと椅子に近い(入口・出口)ときは、その近さより近づく分だけを数える"""
    half = chair_half(c)
    margin = max(float(c.get("clearance_margin", 0.03)), float(c.get("travel_margin", 0.05)))
    reach = half * math.sqrt(2.0) + ROBOT_R + margin  # 椅子の中心からこれより遠い直線は調べない
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    L = math.hypot(vx, vy)
    if L < 1e-6:
        return 0.0
    n = max(1, int(L / 0.03))
    pen = 0.0
    for k in c.get("chairs") or []:
        cx, cy, yaw = k["x"], k["y"], k.get("yaw", 0)
        t = max(0.0, min(1.0, ((cx - ax) * vx + (cy - ay) * vy) / (L * L)))
        if math.hypot(ax + t * vx - cx, ay + t * vy - cy) > reach:
            continue
        gap = lambda x, y: dist_point_square(x, y, cx, cy, yaw, half) - ROBOT_R
        lim = min(margin, gap(ax, ay), gap(bx, by)) - 0.005
        g = min(gap(ax + vx * i / n, ay + vy * i / n) for i in range(1, n)) if n > 1 else lim
        if g < lim:
            pen += 2.0 * (lim - g) + 0.3  # 回り道 + 回る点で止まって向きを変える手間
    return pen * float(c.get("chair_cross_weight", 1.0))  # ルートの自動修正で、横切るのをもっと嫌うときに大きくする


def chair_options(c, i, margin):
    """椅子 i の通り抜け方の候補 [(pass, heading, entry, exit)]。入口・出口がぶつからないものだけ"""
    ch = c["chairs"][i]
    d = float(c.get("pass_distance", 0.35))
    allow_backward = bool(c.get("allow_backward", True))
    base = float(ch.get("yaw", 0))
    fixed = ch.get("pass")
    if ch.get("force_pass") is not None:  # auto_route: true でも、この椅子だけ通り抜ける向きを決める(本番でルートを直すとき)
        passes = [float(ch["force_pass"])]
    elif fixed is not None and not c.get("auto_route", False):
        passes = [float(fixed)]
    else:
        passes = [base + k * 90.0 for k in range(4)]
    # 壁際などで入口・出口が置けないときは、椅子に近づけて再挑戦する
    # (ロボットの奥行きの半分 0.09 + 椅子の外形の半分 0.175 = 0.265 より近いと椅子に触れる)
    dists = [d] + [x for x in (0.31, 0.28) if x < d]
    for dd in dists:
        out = []
        for p in passes:
            ux, uy = math.cos(math.radians(p)), math.sin(math.radians(p))
            entry = (ch["x"] - ux * dd, ch["y"] - uy * dd)
            exit_ = (ch["x"] + ux * dd, ch["y"] + uy * dd)
            if min(pose_clearance(c, *entry, skip_chair=i), pose_clearance(c, *exit_, skip_chair=i)) < margin:
                continue
            for h in ([p, p + 180.0] if allow_backward else [p]):
                out.append((angle_diff(p, 0.0), angle_diff(h, 0.0), entry, exit_))
        if out:
            if dd < d:
                print(f"note: 椅子{i + 1} は入口・出口を中心から {dd:.2f} m に近づけた(壁などが近い)", file=sys.stderr)
            return out
    return []


def option_penalty(c, o, margin):
    """窮屈な通り方を重くする: 入口の手前(入口から 0.15m / 0.30m 手前の軸の上)や、出口の先(0.15m)に
    ロボットを置けない通り方は、壁と椅子のすき間などを回り込まないとたどり着けないことが多い。
    出口の先にも横にも置けない(囲まれたすき間に出る)通り方は、特に重くする"""
    p, _, entry, exit_ = o
    ux, uy = math.cos(math.radians(p)), math.sin(math.radians(p))
    pen = 0.0
    if pose_clearance(c, entry[0] - ux * 0.15, entry[1] - uy * 0.15) < margin:
        pen += 0.5
    if pose_clearance(c, entry[0] - ux * 0.30, entry[1] - uy * 0.30) < 0.0:
        pen += 0.5
    if pose_clearance(c, exit_[0] + ux * 0.15, exit_[1] + uy * 0.15) < 0.0:
        pen += 0.3
        # 出口の先も横(椅子の面に沿って 0.3m)も置けない = 壁・クレーター・椅子に囲まれたすき間に出てしまう。
        # 抜け出すにはクレーターの余白を横切るなどしかない(シミュレーションで、抜け出すときに椅子の脚に触れた)
        if all(pose_clearance(c, exit_[0] + sg * uy * 0.30, exit_[1] - sg * ux * 0.30) < margin for sg in (1.0, -1.0)):
            pen += 1.5
    return pen


def forward_mode(c):
    """drive_mode: forward … 移動の前にその場で進む向きを向いてから前進(後退)する。横には動かない(横滑り対策)
       drive_mode: holonomic … 向きを変えながら斜め・横にも動く(メカナムの本来の動き)"""
    return str(c.get("drive_mode", "forward")) == "forward"


def travel_heading(c, a, ha, b, hb):
    """forward モードで a → b を進むときのロボットの向き(前進 φ か後退 φ+180 の、回転が少ないほう)"""
    phi = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    opts = [phi] + ([phi + 180.0] if bool(c.get("allow_backward", True)) else [])
    return angle_diff(min(opts, key=lambda t: abs(angle_diff(t, ha)) + abs(angle_diff(hb, t))), 0.0)


def _move_cost(c, a, ha, b, hb, rot_weight):
    """a(向き ha)から b(向き hb)へ動く手間 [m 相当]。
    forward モード: 距離 + 回転(a でその場で進む向きへ、b で hb へ)。止まって回るたびに turn_penalty を足す
    どちらのモードも、直線がクレーターや椅子を横切るときは回り道の分を足す
    holonomic モード: 横向きに進む分は lateral_weight 倍に重くする(メカナムは横に滑るため)"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    dist = math.hypot(dx, dy)
    if forward_mode(c) and dist > 0.03:
        t = travel_heading(c, a, ha, b, hb)
        turns = [abs(angle_diff(t, ha)), abs(angle_diff(hb, t))]
        pen = float(c.get("turn_penalty", 0.1)) * sum(1 for x in turns if x > 3.0)
        return (dist + segment_crater_penalty(c, a, b) + segment_chair_penalty(c, a, b)
                + rot_weight * math.radians(sum(turns)) + pen)
    lat = 0.0
    if dist > 1e-6:
        phi = math.degrees(math.atan2(dy, dx))
        lat = 0.5 * (abs(math.sin(math.radians(phi - ha))) + abs(math.sin(math.radians(phi - hb))))
    lateral_weight = float(c.get("lateral_weight", 1.5))
    return (dist * (1.0 + lateral_weight * lat) + segment_crater_penalty(c, a, b) + segment_chair_penalty(c, a, b)
            + rot_weight * math.radians(abs(angle_diff(hb, ha))))


def plan_route(c):  # noqa: C901
    """椅子を回る順番と、それぞれの通り抜け方を決める。
    auto_route: true  → 順番も向きも自動(移動距離 + 回転の重み × 回転角 が最小。全組み合わせを動的計画法で調べる)
    auto_route: false → course.yaml の順番と pass のまま(前進/後退だけは回転が少ないほう)
    戻り値 [(椅子の番号, pass, heading, entry, exit)], 推定コスト"""
    chairs = c.get("chairs") or []
    margin = float(c.get("clearance_margin", 0.03))
    rot_weight = float(c.get("rotation_weight", 0.25))  # 1 rad 回る手間を何 m 進む手間とみなすか
    s = c.get("start") or {"x": 0.5, "y": 0.5, "yaw": 0}
    g = c["goal"]
    start, hs = (s["x"], s["y"]), float(s.get("yaw", 0))
    goal, hg = (g["x"], g["y"]), float(g.get("yaw", 0))
    opts = [chair_options(c, i, margin) for i in range(len(chairs))]
    opt_pen = [[option_penalty(c, o, margin) for o in oo] for oo in opts]
    memo = {}

    def move_cost(c_, a, ha, b, hb, rw):  # 同じ組み合わせを何度も計算しない(動的計画法で同じ区間が何度も出る)
        key = (a, ha, b, hb)
        if key not in memo:
            memo[key] = _move_cost(c_, a, ha, b, hb, rw)
        return memo[key]
    for i, o in enumerate(opts):
        if not o:
            raise RuntimeError(f"椅子{i + 1} ({chairs[i]['x']}, {chairs[i]['y']}) はどの向きでも入口か出口がぶつかる。pass_distance や配置を確認")
    pass_len = 2.0 * float(c.get("pass_distance", 0.35))

    if not c.get("auto_route", False):
        plan, pos, h, cost = [], start, hs, 0.0
        for i, o in enumerate(opts):
            kbest = min(range(len(o)), key=lambda k: move_cost(c, pos, h, o[k][2], o[k][1], rot_weight) + opt_pen[i][k])
            best = o[kbest]
            cost += move_cost(c, pos, h, best[2], best[1], rot_weight) + opt_pen[i][kbest] + pass_len
            plan.append((i,) + best)
            pos, h = best[3], best[1]
        return plan, cost + move_cost(c, pos, h, goal, hg, rot_weight)

    n = len(chairs)
    INF = float("inf")
    dp = {}  # (訪問済みの集合, 最後の椅子, 候補番号) → (コスト, ひとつ前の状態)
    for i in range(n):
        for k, o in enumerate(opts[i]):
            dp[(1 << i, i, k)] = (move_cost(c, start, hs, o[2], o[1], rot_weight) + opt_pen[i][k] + pass_len, None)
    for mask in range(1, 1 << n):
        for i in range(n):
            if not mask & (1 << i):
                continue
            for k, oi in enumerate(opts[i]):
                cur = dp.get((mask, i, k))
                if cur is None:
                    continue
                for j in range(n):
                    if mask & (1 << j):
                        continue
                    nm = mask | (1 << j)
                    for l, oj in enumerate(opts[j]):
                        v = cur[0] + move_cost(c, oi[3], oi[1], oj[2], oj[1], rot_weight) + opt_pen[j][l] + pass_len
                        if v < dp.get((nm, j, l), (INF,))[0]:
                            dp[(nm, j, l)] = (v, (mask, i, k))
    full = (1 << n) - 1
    best_key, best_cost = None, INF
    for i in range(n):
        for k, o in enumerate(opts[i]):
            e = dp.get((full, i, k))
            if e is None:
                continue
            v = e[0] + move_cost(c, o[3], o[1], goal, hg, rot_weight)
            if v < best_cost:
                best_key, best_cost = (full, i, k), v
    plan, key = [], best_key
    while key is not None:
        _, i, k = key
        plan.append((i,) + opts[i][k])
        key = dp[key][1]
    plan.reverse()
    return plan, best_cost


def fit_crater_margin(c):
    """ルートが作れないときは、クレーターの余白(crater_margin)を半分 → 0 と減らして作り直す。c を書き換える"""
    m0 = float(c.get("crater_margin", 0.0))
    for m in [m0, m0 / 2.0, 0.0]:
        c["crater_margin"] = m
        try:
            plan_route(c)
            if m < m0:
                print(f"warning: ルートが作れなかったので crater_margin を {m0:.2f} → {m:.2f} m に減らした(クレーターに注意)", file=sys.stderr)
            return
        except RuntimeError:
            if m == 0.0:
                raise


def make_route(c, path):
    """椅子ごとに 入口 → 出口。メカナムは真横に滑りやすいので、椅子の中はロボットの前後方向だけで進む。"""
    plan, cost = plan_route(c)
    wait = float(c.get("entry_wait_sec", 0.0))  # 入口で止まる時間(止まっている間に自己位置が正確になる)
    steps = []  # (状態名, x, y, yaw) または (状態名, 待ち時間)
    s0 = c.get("start") or {"x": 0.5, "y": 0.5, "yaw": 0}
    pos, head = (s0["x"], s0["y"]), angle_diff(float(s0.get("yaw", 0)), 0.0)

    TURN = 0.15  # 向きを変えながら進む距離 [m]。natto の経路計画は 0.1m 以内の目標を「前と同じ」とみなして
    #              無視することがあるので、その場で回る目標(同じ位置で向きだけ違う)は作らない

    def go(name, target, final_yaw, approach=None, leave=None, wait_at_approach=0.0):
        """pos から target へ。forward モードのとき:
           leave(通り抜けの軸の向き)があれば、pos は椅子の出口:
             1. 軸に沿ってさらに TURN(置けなければ 0.12m)進みながら、次へ進む向きに回る(_離れる)
             2. そこから次の点へまっすぐ向かうと、いま出た椅子の角をかすめるときは
                (次の点が椅子の後ろ側にある、壁際で 1 が置けない など)、軸と直角に椅子の横の面に沿って
                離れてから向かう(_よける)
           approach があれば: 入口の TURN 手前の軸の上まで前進(後退)し、軸に沿って入口へ入りながら final_yaw に回る
                           (横から斜めに入口へ近づくと、向きはそろっても軸から横にずれたまま脚の列に入ってしまうため)
           なければ: 進む向きへ回りながら TURN 進む → target の TURN 手前まで前進 → final_yaw へ回りながら target へ
           途中で椅子を横切るときは、椅子の外側の点をつないで回る(_回る1〜5、最短経路)"""
        nonlocal pos, head
        margin = float(c.get("clearance_margin", 0.03))
        # 椅子と椅子の間を動くときの余白(直線・回る点)。椅子の間では自己位置の誤差が 3〜5cm あるので、
        # 3cm では回る点へ向かう直線が椅子の角に触れた(シミュレーション)
        tmargin = max(margin, float(c.get("travel_margin", 0.05)))

        def free(x, y):  # 途中の点が壁・クレーター・コーン・椅子に近すぎないか
            return pose_clearance(c, x, y) >= margin

        def seg_ok(a, b, chair=None):
            """a → b の直線の途中で、障害物との余白(tmargin)を保てるか(両端がもともと近いときは、その近さまで許す)。
            自己位置の誤差の分の余白。余白 0 で判定すると、椅子の角を 1cm でかすめる直線を通してしまった(シミュレーション)。
            chair に番号を指定するとその椅子だけ、"all" で壁以外の障害物(椅子・クレーター・コーン)を調べる"""
            if chair is None:
                gap = lambda x, y: pose_clearance(c, x, y)
            elif chair == "all":
                return not blocking(a, b, first=True)  # 障害物ごとに、その障害物と両端の近さで判定する
            elif chair == "chairs":
                return not blocking(a, b, circles=False, first=True)
            else:
                k = c["chairs"][chair]
                gap = lambda x, y: dist_point_square(x, y, k["x"], k["y"], k.get("yaw", 0), chair_half(c)) - ROBOT_R
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.02))
            lim = min(margin, gap(*a), gap(*b)) - 0.005
            return all(gap(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) >= lim for i in range(1, n))

        # 障害物(椅子 = 正方形、クレーター・コーン = 円)ごとの「機体を囲む円とのすき間」
        obstacles = []
        for k in c.get("chairs") or []:
            obstacles.append(("chair", k["x"], k["y"], k.get("yaw", 0), chair_half(c)))
        # クレーター・コーン(円)は、回る点の直線が横切らないかの判定にだけ使う(回る点の候補は椅子のまわりだけ)
        r_crater = float(c["crater_radius"]) + float(c.get("crater_margin", 0.0))
        for x0, y0 in c.get("craters") or []:
            obstacles.append(("circle", x0, y0, 0.0, r_crater))
        if c.get("cone"):
            obstacles.append(("circle", c["cone"]["x"], c["cone"]["y"], 0.0, float(c["cone"]["radius"])))

        def obstacle_gap(o, x, y):
            if o[0] == "chair":
                return dist_point_square(x, y, o[1], o[2], o[3], o[4]) - ROBOT_R
            return math.hypot(x - o[1], y - o[2]) - o[4] - ROBOT_R

        def chairs_gap(x, y):
            return min((obstacle_gap(o, x, y) for o in obstacles), default=9.0)

        def blocking(a, b, circles=True, first=False):
            """a → b の直線で余白を保てない障害物(circles=False なら椅子だけ調べる。first=True なら1つ見つけたら終わり)"""
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(L / 0.02))
            out = []
            for o in obstacles:
                if not circles and o[0] != "chair":
                    continue
                # 直線から遠い障害物は調べない(外接円 + 機体 + 余白より遠い)
                bound = (o[4] * math.sqrt(2.0) if o[0] == "chair" else o[4]) + ROBOT_R + margin + 0.01
                t = 0.0 if L < 1e-9 else max(0.0, min(1.0, ((o[1] - a[0]) * (b[0] - a[0]) + (o[2] - a[1]) * (b[1] - a[1])) / (L * L)))
                if math.hypot(a[0] + (b[0] - a[0]) * t - o[1], a[1] + (b[1] - a[1]) * t - o[2]) > bound:
                    continue
                lim = min(margin, obstacle_gap(o, *a), obstacle_gap(o, *b)) - 0.005
                if any(obstacle_gap(o, a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) < lim for i in range(1, n)):
                    out.append(o)
                    if first:
                        break
            return out

        def detour_path(a, b, check="all", max_points=5):
            """a → b の直線が椅子を横切るとき、椅子の外側の点をつないで回る道を探す(最短経路、A* 探索)。
            点の候補: 椅子の4つの角の外と4つの辺のまん中の外(椅子から tmargin、壁との間が狭いときのためにもう少し近い点も)、
            check="all" のときはクレーター・コーンのまわり 8方向も(クレーターを横切る道しかないと、A* がクレーターを
            回り込んだ先で椅子の脚に触れた: シミュレーション)。
            点と点の直線は余白 margin を保つこと(check="all" ならクレーター・コーンも横切らない)。
            経路計画(A*)に任せると、forward モードでは向きを直線に合わせたまま回り込むので横に動き、横滑りで近くの椅子の
            脚に近づく(椅子の下を斜めに抜けてしまったこともあった: シミュレーション)。回る点を置けば、そこでいったん
            向きを変えて前進だけで進める。1回回るごとに 0.15m 分のコストを足して、点の少ない道を選ぶ。見つからなければ None"""
            import heapq
            L0 = math.hypot(b[0] - a[0], b[1] - a[1])
            nodes = [a, b]
            for o in obstacles:
                if o[0] != "chair" and check != "all":
                    continue
                # a・b からあまりに遠い障害物は使わない(だ円の中だけ)
                if math.hypot(o[1] - a[0], o[2] - a[1]) + math.hypot(o[1] - b[0], o[2] - b[1]) > L0 + 2.0:
                    continue
                for extra in (0.02, 0.0):
                    if o[0] == "chair":
                        H = o[4] + ROBOT_R + tmargin + extra
                        pts = [rotate(sx * H, sy * H, o[3]) for sx, sy in
                               ((1, 1), (1, -1), (-1, 1), (-1, -1), (1, 0), (-1, 0), (0, 1), (0, -1))]
                    else:  # クレーター・コーンのまわり 8方向
                        R = o[4] + ROBOT_R + margin + extra
                        pts = [(R * math.cos(k * math.pi / 4), R * math.sin(k * math.pi / 4)) for k in range(8)]
                    for dx, dy in pts:
                        q = (o[1] + dx, o[2] + dy)
                        if free(*q):
                            nodes.append(q)
            dist = lambda i, j: math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
            edge_cache = {}

            def edge(i, j):
                key = (min(i, j), max(i, j))
                if key not in edge_cache:
                    edge_cache[key] = dist(i, j) >= 0.15 and seg_ok(nodes[i], nodes[j], chair=check)
                return edge_cache[key]

            best_g = {0: 0.0}
            prev = {}
            heap = [(dist(0, 1), 0.0, 0, 0)]  # (f, g, 点の番号, 回った点の数)
            closed = set()
            while heap:
                f, g, u, k = heapq.heappop(heap)
                if u in closed:
                    continue
                if u == 1:
                    path = []
                    while u != 0:
                        u = prev[u]
                        if u != 0:
                            path.append(nodes[u])
                    return path[::-1]
                closed.add(u)
                if k >= max_points:
                    continue
                for v in range(1, len(nodes)):
                    if v in closed or v == u:
                        continue
                    ng = g + dist(u, v) + (0.15 if v != 1 else 0.0)
                    if ng < best_g.get(v, float("inf")) and edge(u, v):
                        best_g[v] = ng
                        prev[v] = u
                        heapq.heappush(heap, (ng + dist(v, 1), ng, v, k + (1 if v != 1 else 0)))
            return None

        def hop(label, q):
            """pos から q へ: 進む向きへ回りながら TURN 進む → q まで前進(後退)"""
            nonlocal pos, head
            dq = math.hypot(q[0] - pos[0], q[1] - pos[1])
            t = travel_heading(c, pos, head, q, head)
            a = (pos[0] + (q[0] - pos[0]) / dq * TURN, pos[1] + (q[1] - pos[1]) / dq * TURN)
            if abs(angle_diff(t, head)) > 3.0 and dq > TURN + 0.12 and free(*a):
                steps.append((f"{name}_{label}向き", a[0], a[1], t))
                pos, head = a, t
            steps.append((f"{name}_{label}", q[0], q[1], t))
            pos, head = q, t

        if not forward_mode(c):
            steps.append((name, target[0], target[1], final_yaw))
            pos, head = target, final_yaw
            return
        if approach is not None:
            # 入口の手前の点(軸の上)は余白なしで判定する(クレーターの余白はもともと地図に入っている)。
            # TURN で置けなければ 0.12m で試す(natto は 0.1m 以内の目標を同じとみなすので、それより近づけない)
            ex, ey = target
            ax, ay = approach
            vx, vy = (ex - ax) / TURN, (ey - ay) / TURN  # 軸の向き(単位ベクトル)
            approach = next(((ex - vx * off, ey - vy * off) for off in (TURN, 0.12)
                             if pose_clearance(c, ex - vx * off, ey - vy * off) >= 0.0), None)
        nxt = approach if approach is not None else target
        if leave is not None:
            ux, uy, j = leave  # 通り抜けの軸、いま出た椅子の番号
            dep = None  # 1. 軸に沿って離れる点(余白ありで置けなければ、余白なしで)
            for off, need in ((TURN, margin), (0.12, margin), (TURN, 0.0), (0.12, 0.0)):
                q = (pos[0] + ux * off, pos[1] + uy * off)
                if pose_clearance(c, *q) >= need:
                    dep = q
                    break
            if dep is not None and math.hypot(nxt[0] - dep[0], nxt[1] - dep[1]) <= 0.12:
                dep = None
            base = dep if dep is not None else pos
            side = None  # 2. 軸と直角に離れる点
            if not seg_ok(base, nxt, chair=j):
                best = None
                for sgn in (1.0, -1.0):
                    for sd in (0.15, 0.20, 0.25, 0.30, 0.35, 0.40):
                        q = (base[0] - uy * sgn * sd, base[1] + ux * sgn * sd)
                        dq = math.hypot(nxt[0] - q[0], nxt[1] - q[1])
                        if free(*q) and dq > 0.12 and seg_ok(base, q) and seg_ok(q, nxt, chair=j):
                            if best is None or sd + dq < best[0]:
                                best = (sd + dq, q)
                            break  # この向きでは、いちばん近い点で十分
                if best is not None:
                    side = best[1]
            if dep is not None:
                t = travel_heading(c, dep, head, side if side is not None else nxt, final_yaw)
                steps.append((f"{name}_離れる", dep[0], dep[1], t))
                pos, head = dep, t
            if side is not None:
                t = travel_heading(c, pos, head, side, head)
                steps.append((f"{name}_よける", side[0], side[1], t))
                pos, head = side, t
        # 3. 次の点へまっすぐ向かうと椅子やクレーターを横切るときは、外側の点をつないで回る(5点まで、最短経路)。
        #    入口へ向かうときは、入口の 2×TURN 手前(_手前2)を目指す
        aim = nxt
        if approach is not None:
            vx, vy = target[0] - approach[0], target[1] - approach[1]
            vn = math.hypot(vx, vy) or 1.0
            far0 = (approach[0] - vx / vn * TURN, approach[1] - vy / vn * TURN)
            if pose_clearance(c, *far0) >= 0.0:
                aim = far0
        if not seg_ok(pos, aim, chair="all"):
            # クレーター・コーンも横切らない道を探し、なければ(椅子を横切るときだけ)椅子だけを見て探す。
            # クレーターだけを横切る直線も A* に任せない: A* がクレーターを回り込んだ先で椅子の脚に触れた(シミュレーション)
            path = detour_path(pos, aim)
            if path is None and not seg_ok(pos, aim, chair="chairs"):
                path = detour_path(pos, aim, check="chairs")
            for n_detour, q in enumerate(path or []):
                hop(f"回る{n_detour + 1}", q)
        d = math.hypot(nxt[0] - pos[0], nxt[1] - pos[1])
        if d > 0.12:
            t = travel_heading(c, pos, head, nxt, final_yaw)
            ux, uy = (nxt[0] - pos[0]) / d, (nxt[1] - pos[1]) / d
            a = (pos[0] + ux * TURN, pos[1] + uy * TURN)
            if abs(angle_diff(t, head)) > 3.0 and d > TURN + 0.12 and free(*a):
                steps.append((f"{name}_向き", a[0], a[1], t))
                pos, head = a, t
            if approach is not None:
                # 入口の 2×TURN 手前まで進む向きのまま前進し、そこから TURN 進むあいだに突入の向きへ回る。
                # 入口の TURN 手前(approach)では向きがそろっている → 止まって脚を測る → 入口までまっすぐ前進。
                # (回りながら進むと横に数 cm ふくらむので、大きく回るのは脚から遠い場所で済ませる)
                ex, ey = target
                vx, vy = (ex - approach[0]), (ey - approach[1])
                vn = math.hypot(vx, vy) or 1.0
                far = (approach[0] - vx / vn * TURN, approach[1] - vy / vn * TURN)
                if (abs(angle_diff(final_yaw, t)) > 3.0 and pose_clearance(c, *far) >= 0.0
                        and math.hypot(far[0] - pos[0], far[1] - pos[1]) > 0.12):
                    steps.append((f"{name}_手前2", far[0], far[1], t))
                    steps.append((f"{name}_手前", approach[0], approach[1], final_yaw))
                    pos, head = approach, final_yaw
                elif abs(angle_diff(final_yaw, t)) > 3.0:
                    # 軸の上の _手前2 が置けない(壁やほかの椅子が近い): 来た向きの直線の上で、手前の TURN 手前から
                    # 回り始め、_手前 には突入の向きで着く(入口の手前 → 入口はまっすぐ。測るときも向きがそろっている)
                    da = math.hypot(approach[0] - pos[0], approach[1] - pos[1])
                    b = None
                    if da >= TURN + 0.12:
                        b = (approach[0] - (approach[0] - pos[0]) / da * TURN, approach[1] - (approach[1] - pos[1]) / da * TURN)
                        if pose_clearance(c, *b) < 0.0:
                            b = None
                    if b is not None:
                        steps.append((f"{name}_手前2", b[0], b[1], t))
                    steps.append((f"{name}_手前", approach[0], approach[1], final_yaw))
                    pos, head = approach, final_yaw
                else:
                    steps.append((f"{name}_手前", approach[0], approach[1], t))
                    pos, head = approach, t
                if wait_at_approach > 0.0:
                    # 入口の手前(脚の列から十分離れた軸の上)で止まって脚を測る → 補正を受けてから入口まで
                    # 進むあいだに横のずれと向きを直す(入口で測ると、ずれたまま斜めに脚の列を横切ってしまう)
                    steps.append((f"{name[:-3]}_停止", wait_at_approach))
                    waited[0] = True
            elif abs(angle_diff(final_yaw, t)) > 3.0 and d >= 2.0 * TURN + 0.15:
                b = (target[0] - ux * TURN, target[1] - uy * TURN)
                if free(*b):
                    steps.append((f"{name}_手前", b[0], b[1], t))
                    pos, head = b, t
        steps.append((name, target[0], target[1], final_yaw))
        pos, head = target, final_yaw

    leave = None
    waited = [False]
    for i, p, h, entry, exit_ in plan:
        ux, uy = math.cos(math.radians(p)), math.sin(math.radians(p))  # 通り抜けの軸(入口 → 出口)
        waited[0] = False
        go(f"椅子{i + 1}_入口", entry, h, approach=(entry[0] - ux * TURN, entry[1] - uy * TURN), leave=leave, wait_at_approach=wait)
        if wait > 0.0 and not waited[0]:
            steps.append((f"椅子{i + 1}_停止", wait))  # 手前の点が置けないときは入口で止まって測る
        steps.append((f"椅子{i + 1}_出口", exit_[0], exit_[1], h))
        pos, head = exit_, h
        leave = (ux, uy, i)
    g = c["goal"]
    if math.hypot(g["x"] - pos[0], g["y"] - pos[1]) < 0.12:
        # 最後の出口がゴールのすぐそば: natto は 0.1m 以内の目標を「前と同じ」とみなすことがあるので、移動せずに終わる
        goal_step = None
    else:
        go("ゴール", (g["x"], g["y"]), angle_diff(float(g.get("yaw", 0)), 0.0), leave=leave)
        goal_step = steps.pop()  # 最後の「ゴール」は [*] への遷移として書く

    lines = [
        "# ルート(make_map.py が config/map/course.yaml から自動生成。直接編集しない)",
        "スタートで5秒待つ → 椅子ごとに入口 → 出口(椅子の真下を通り抜けてサンプルの上を通過)→ ゴール。"
        + (" 移動の前にその場で進む向きを向く(drive_mode: forward、横滑り対策)。" if forward_mode(c) else ""),
        f"順番: {' → '.join(str(i + 1) for i, *_ in plan)}(推定の移動コスト {cost:.1f} m 相当)",
        "",
        "```mermaid",
        "stateDiagram-v2",
        "",
        "[*] --> 待機 : wait(duration_sec = 5.0)",
    ]
    prev = "待機"
    for st in steps:
        if len(st) == 2:
            lines.append(f"{prev} --> {st[0]} : wait(duration_sec = {st[1]:.1f})")
        else:
            name, x, y, yaw = st
            lines.append(f"{prev} --> {name} : set_pose(x = {x:.3f}, y = {y:.3f}, yaw = {yaw:.1f})")
        prev = st[0]
    if goal_step is None:
        lines.append(f"{prev} --> [*] : wait(duration_sec = 0.5)")
    else:
        lines.append(f"{prev} --> [*] : set_pose(x = {goal_step[1]:.3f}, y = {goal_step[2]:.3f}, yaw = {goal_step[3]:.1f})")
    lines.append("```")
    with open(path, "w") as fp:
        fp.write("\n".join(lines) + "\n")
    return plan, cost


def validate_course(c):
    """course.yaml の入力ミスを探す(本番で配置を書き写したあと)。戻り値: 注意のリスト"""
    out = []
    f = c.get("field", {})
    W, H = float(f.get("width", 5.0)), float(f.get("height", 4.0))
    chairs = c.get("chairs") or []
    half = chair_half(c)
    if len(chairs) != 8:
        out.append(f"椅子が {len(chairs)} 脚(8脚の前提)")
    for i, k in enumerate(chairs):
        x, y = float(k["x"]), float(k["y"])
        if not (0.0 < x < W and 0.0 < y < H):
            out.append(f"椅子{i + 1} ({x}, {y}) がコースの外(x 0〜{W}、y 0〜{H})")
        elif min(x, W - x, y, H - y) < half:
            out.append(f"椅子{i + 1} ({x}, {y}) が壁にめり込んでいる(中心が壁から {min(x, W - x, y, H - y) * 100:.0f}cm)")
        yaw = float(k.get("yaw", 0.0))
        if abs(yaw) > 360.0:
            out.append(f"椅子{i + 1} の向き {yaw} が大きすぎる(度で書く)")
        for j in range(i):
            d = math.hypot(x - float(chairs[j]["x"]), y - float(chairs[j]["y"]))
            if d < 2 * half:
                out.append(f"椅子{j + 1} と椅子{i + 1} が重なっている(中心どうし {d * 100:.0f}cm)。どちらかの入力ミス?")
            elif d < 2 * half + 2 * ROBOT_R:
                out.append(f"椅子{j + 1} と椅子{i + 1} のすき間がロボットより狭い(中心どうし {d * 100:.0f}cm)")
        for cx, cy in c.get("craters") or []:
            if dist_point_square(cx, cy, x, y, yaw, half) < float(c.get("crater_radius", 0.21)):
                out.append(f"椅子{i + 1} とクレーター ({cx}, {cy}) が重なっている")
    yaws = [float(k.get("yaw", 0.0)) for k in chairs]
    if chairs and all(abs(v) <= 3.2 for v in yaws) and any(0.01 < abs(v - round(v)) for v in yaws):
        out.append("椅子の向きがすべて ±3.2 以下で小数がある。ラジアンで書いていないか(度で書く)")
    s0 = c.get("start")
    if s0:
        g = pose_clearance(c, float(s0["x"]), float(s0["y"]))
        if g < -0.05:
            out.append(f"スタート ({s0['x']}, {s0['y']}) が壁・椅子・クレーターに重なっている(すき間 {g * 100:.0f}cm)")
    for x, y in c.get("craters") or []:
        if not (0.0 < x < W and 0.0 < y < H):
            out.append(f"クレーター ({x}, {y}) がコースの外")
    return out


def make_route_auto(c, path, max_rounds=3, verbose=True):
    """make_route のあと、ルートを検査(route_check.py)して problems / warning が出たら、関係する椅子の通り抜ける向きを
    90°ずつ変えて(force_pass)作り直し、いちばん良いものを path に書く。戻り値: (plan, cost, 向きを変えた椅子 {番号: pass}, 検査結果)
    本番で手で force_pass を付ける作業(contest_day.md)を自動でやる。1回の作り直しに約1.5秒、多くて数十秒"""
    import copy
    import tempfile
    import route_check

    def score(r):
        return 10 * len(r["errs"]) + len(r["warns"])

    plan, cost = make_route(c, path)
    res = route_check.check_route(c, path)
    if score(res) == 0:
        return plan, cost, {}, res
    forced = {j: float(k["force_pass"]) for j, k in enumerate(c.get("chairs") or []) if k.get("force_pass") is not None}
    best = (score(res), cost, dict(forced), plan, res)
    with tempfile.TemporaryDirectory() as td:
        tmp = os.path.join(td, "course.md")
        for _ in range(max_rounds):
            cur_plan, cur_forced = best[3], best[2]
            round_best = None
            for i in sorted(best[4]["implicated"] - set(cur_forced)):
                now = [q[1] for q in cur_plan if q[0] == i]
                if not now:
                    continue
                for k in (1, 2, 3):
                    fp = angle_diff(now[0] + 90.0 * k, 0.0)
                    c2 = copy.deepcopy(c)
                    for j, v in list(cur_forced.items()) + [(i, fp)]:
                        c2["chairs"][j]["force_pass"] = v
                    try:
                        plan2, cost2 = make_route(c2, tmp)
                    except RuntimeError:
                        continue
                    r2 = route_check.check_route(c2, tmp)
                    cand = (score(r2), cost2, {**cur_forced, i: fp}, plan2, r2)
                    if round_best is None or cand[:2] < round_best[:2]:
                        round_best = cand
            # 回る順番も変えてみる: 椅子を横切る移動をもっと嫌って計画し直す(壁ぎわの椅子など、向きだけでは直らないとき)
            for w in (3.0, 10.0):
                c2 = copy.deepcopy(c)
                c2["chair_cross_weight"] = w
                for j, v in cur_forced.items():
                    c2["chairs"][j]["force_pass"] = v
                try:
                    plan2, cost2 = make_route(c2, tmp)
                except RuntimeError:
                    continue
                r2 = route_check.check_route(c2, tmp)
                cand = (score(r2), cost2, dict(cur_forced), plan2, r2, w)
                if round_best is None or cand[:2] < round_best[:2]:
                    round_best = cand
            if round_best is None or round_best[:2] >= best[:2]:
                break
            best = round_best
            if best[0] == 0:
                break
    c3 = copy.deepcopy(c)
    for j, v in best[2].items():
        c3["chairs"][j]["force_pass"] = v
    if len(best) > 5:
        c3["chair_cross_weight"] = best[5]
    plan, cost = make_route(c3, path)
    res = route_check.check_route(c3, path)
    new = {j: v for j, v in best[2].items() if j not in forced}
    if len(best) > 5:
        new["order"] = best[5]  # 回る順番を変えた(椅子を横切る重み)
    if verbose and new:
        print("ルートの検査で問題が出たので、ルートを変えた: "
              + "、".join(f"椅子{j + 1} の向き → {v:.1f}°" if j != "order" else f"回る順番(chair_cross_weight: {v:.0f})"
                         for j, v in sorted(new.items(), key=lambda kv: str(kv[0])))
              + f"(残りの問題 {len(res['errs'])}、注意 {len(res['warns'])})", file=sys.stderr)
    return plan, cost, new, res


def main():
    import argparse
    pkg = package_src_dir()
    ap = argparse.ArgumentParser(description="course.yaml から地図とルートを作る")
    ap.add_argument("course", nargs="?", default=os.path.join(pkg, "config", "map", "course.yaml"))
    ap.add_argument("--out", help="出力先のフォルダ(省略するとパッケージの config/map と state_graph を上書き)")
    args = ap.parse_args()
    c = yaml.safe_load(open(args.course))

    map_dir = args.out or os.path.join(pkg, "config", "map")
    route_path = os.path.join(args.out, "course.md") if args.out else os.path.join(pkg, "state_graph", "course.md")
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        if os.path.realpath(args.course) != os.path.realpath(os.path.join(args.out, "course.yaml")):
            yaml.safe_dump(c, open(os.path.join(args.out, "course.yaml"), "w"), allow_unicode=True, sort_keys=False)
    for w in validate_course(c):
        print(f"注意(入力): {w}", file=sys.stderr)
    fit_crater_margin(c)  # 先にルートが作れる余白を決めてから、その余白で地図を描く
    nseg, ncirc = make_map(c, map_dir)
    plan, cost, changed, res = make_route_auto(c, route_path)

    print(f"map:   {map_dir}/line_segments.csv ({nseg} segments), circles.csv ({ncirc} circles)")
    print(f"route: {route_path}")
    print(f"  順番 {' → '.join(str(i + 1) for i, *_ in plan)}、推定コスト {cost:.1f} m 相当({'自動' if c.get('auto_route') else 'course.yaml の順番'})")
    for i, p, h, entry, exit_ in plan:
        kind = "前進" if abs(angle_diff(h, p)) < 1 else "後退"
        print(f"  椅子{i + 1}: 通り抜ける向き {p:6.1f}°、ロボットの向き {h:6.1f}°({kind})")
    s = c.get("start")
    if s:
        print(f"  start ({s['x']}, {s['y']}, {s.get('yaw', 0)}deg)(launch がここから自己位置推定の初期位置を読む)")
    if changed:
        print("  自動で変えたところ: " + "、".join(f"椅子{j + 1}(force_pass: {v:.1f})" if j != "order" else f"回る順番(chair_cross_weight: {v:.0f})"
                                            for j, v in sorted(changed.items(), key=lambda kv: str(kv[0]))))
    for e in res["errs"]:
        print(f"  problem: {e}")
    for w in res["warns"]:
        print(f"  warning: {w}")
    if res["errs"] or res["warns"]:
        print("  → 上の区間は RViz で特に注意して見る(contest_day.md 3)")


if __name__ == "__main__":
    main()
