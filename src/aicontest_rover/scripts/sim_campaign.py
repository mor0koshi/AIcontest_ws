#!/usr/bin/python3
"""ランダムな配置でシミュレーションを何回も自動で回して、成功率などを表にまとめる。

  ros2 run aicontest_rover sim_campaign.py --runs 20 --out ~/sim_results/run1
  ros2 run aicontest_rover sim_campaign.py --runs 10 --chair-error 0.03 --launch-args odom_wheels:=3

1回ごとに:
  1. ルールに沿ったランダムな配置を作る(本当の配置 = world)
  2. 人が測ったときの誤差(--chair-error など)を足した配置を作り、ロボットにはこちらを渡す(measured)
  3. 置き位置の誤差もランダムに入れて sim_realistic.launch.xml を起動する
  4. 本当の配置を基準に、椅子の真下を通ったか・脚やクレーターに触れたか・自己位置の誤差・時間を記録する
  5. ゴールしたら(または時間切れで)止めて、プロセスをすべて片付けてから次へ
結果: OUT/results.csv(1回 1行)、OUT/summary.md(まとめ)、OUT/runNNN/(配置・ログ)
"""
import argparse
import bisect
import copy
import csv
import math
import os
import random
import signal
import subprocess
import sys
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import make_map  # noqa: E402
import random_course  # noqa: E402

HX, HY = 0.09, 0.10  # 機体の外形の半分(奥行き 180mm × 幅 200mm)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rect_clearance(x, y, th, cx, cy, r):
    dx, dy = cx - x, cy - y
    lx, ly = dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)
    return math.hypot(max(abs(lx) - HX, 0.0), max(abs(ly) - HY, 0.0)) - r


def legs_of(c):
    ch = c.get("chair", {})
    leg = float(ch.get("leg_size", 0.02))
    hx = float(ch.get("inner_x", 0.31)) / 2 + leg / 2
    hy = float(ch.get("inner_y", 0.31)) / 2 + leg / 2
    out = []
    for k in c["chairs"]:
        for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
            lx, ly = make_map.rotate(sx * hx, sy * hy, k.get("yaw", 0))
            out.append((k["x"] + lx, k["y"] + ly, leg / 2))
    return out


def measured_layout(world, rng, chair_err, chair_yaw_err, crater_err):
    """本当の配置に、人が測ったときの誤差を足す"""
    m = copy.deepcopy(world)
    for k in m["chairs"]:
        k["x"] = round(k["x"] + rng.gauss(0, chair_err), 3)
        k["y"] = round(k["y"] + rng.gauss(0, chair_err), 3)
        k["yaw"] = round(k.get("yaw", 0) + rng.gauss(0, chair_yaw_err), 1)
    m["craters"] = [[round(x + rng.gauss(0, crater_err), 3), round(y + rng.gauss(0, crater_err), 3)] for x, y in m["craters"]]
    return m


class Evaluator:
    """本当の配置を基準に、走りを記録する(rclpy のノード)"""

    def __init__(self, node, world, traj_path=None):
        from geometry_msgs.msg import PoseStamped
        self.world = world
        # 走った跡(本当の位置・ロボットが思っている位置・脚とのすき間)を 0.1 秒ごとに書く。失敗の原因を調べる用
        self.traj = open(traj_path, "w") if traj_path else None
        if self.traj:
            self.traj.write("t,t_epoch,x,y,yaw_deg,est_x,est_y,est_yaw_deg,leg_gap_cm\n")  # t_epoch は launch.log の時刻と同じ
        self.t_traj = -1.0
        self.est = None
        self.legs = legs_of(world)
        self.chairs = [(k["x"], k["y"]) for k in world["chairs"]]
        self.craters = [tuple(c) for c in world["craters"]]
        self.crater_r = float(world["crater_radius"])
        self.goal = (world["goal"]["x"], world["goal"]["y"])
        self.chair_best = [9.0] * len(self.chairs)
        self.leg_min, self.crater_min = 9.0, 9.0
        self.leg_at, self.crater_at = "", ""
        self.leg_contacts, self.crater_contacts = 0, 0
        self._in_leg, self._in_crater = False, False
        self.hist_t, self.hist_p = [], []
        self.loc_err = []
        self.corrections = 0
        self.t_all_chairs = None
        self.t_goal = None
        self.last = None
        self.still_since = None
        node.create_subscription(PoseStamped, "/simulation/pose", self.on_sim, 10)
        node.create_subscription(PoseStamped, "/localization/current_pose", self.on_est, 10)
        node.create_subscription(PoseStamped, "/localization/pose_correction", self.on_corr, 10)
        self.t0 = time.time()

    def on_corr(self, msg):
        self.corrections += 1

    def on_sim(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.position
        th = yaw_of(m.pose.orientation)
        self.hist_t.append(t)
        self.hist_p.append((p.x, p.y))
        if len(self.hist_t) > 4000:
            del self.hist_t[:2000]
            del self.hist_p[:2000]
        for i, (cx, cy) in enumerate(self.chairs):
            self.chair_best[i] = min(self.chair_best[i], math.hypot(p.x - cx, p.y - cy))
        d = min(rect_clearance(p.x, p.y, th, *l) for l in self.legs)
        if self.traj and t - self.t_traj >= 0.1:
            self.t_traj = t
            e = self.est or (float("nan"),) * 3
            now_w = time.time()
            self.traj.write(f"{now_w - self.t0:.2f},{now_w:.2f},{p.x:.4f},{p.y:.4f},{math.degrees(th):.2f},"
                            f"{e[0]:.4f},{e[1]:.4f},{math.degrees(e[2]):.2f},{d * 100:.2f}\n")
        if d < self.leg_min:
            self.leg_at = f"({p.x:.2f},{p.y:.2f},{math.degrees(th):.0f}deg,t={time.time() - self.t0:.0f}s)"
        self.leg_min = min(self.leg_min, d)
        if d < 0 and not self._in_leg:
            self.leg_contacts += 1
        self._in_leg = d < 0
        d = min((rect_clearance(p.x, p.y, th, cx, cy, self.crater_r) for cx, cy in self.craters), default=9.0)
        if d < self.crater_min:
            self.crater_at = f"({p.x:.2f},{p.y:.2f},{math.degrees(th):.0f}deg,t={time.time() - self.t0:.0f}s)"
        self.crater_min = min(self.crater_min, d)
        if d < 0 and not self._in_crater:
            self.crater_contacts += 1
        self._in_crater = d < 0
        now = time.time() - self.t0
        if self.t_all_chairs is None and all(b < 0.05 for b in self.chair_best):
            self.t_all_chairs = now
        moved = self.last is None or math.hypot(p.x - self.last[0], p.y - self.last[1]) > 0.002
        self.last = (p.x, p.y)
        if moved:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now
        # ゴールの判定は launch のログ(ルートの最後 /_exit まで状態遷移したか)で行う(main 側)

    def on_est(self, m):
        self.est = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        i = bisect.bisect_left(self.hist_t, t)
        cand = [k for k in (i - 1, i) if 0 <= k < len(self.hist_t)]
        if not cand:
            return
        k = min(cand, key=lambda k: abs(self.hist_t[k] - t))
        if abs(self.hist_t[k] - t) < 0.05:
            self.loc_err.append(math.hypot(m.pose.position.x - self.hist_p[k][0], m.pose.position.y - self.hist_p[k][1]))

    def result(self):
        if self.traj:
            self.traj.close()
            self.traj = None
        e = sorted(self.loc_err) or [0.0]
        passed = sum(1 for b in self.chair_best if b < 0.05)
        return {
            "chairs_passed": passed,
            "worst_chair_cm": round(max(self.chair_best) * 100, 1),
            "leg_min_cm": round(self.leg_min * 100, 1),
            "leg_contacts": self.leg_contacts,
            "leg_min_at": self.leg_at,
            "crater_min_cm": round(self.crater_min * 100, 1),
            "crater_contacts": self.crater_contacts,
            "crater_min_at": self.crater_at,
            "loc_err_mean_cm": round(sum(e) / len(e) * 100, 1),
            "loc_err_95_cm": round(e[int(0.95 * (len(e) - 1))] * 100, 1),
            "loc_err_max_cm": round(e[-1] * 100, 1),
            "corrections": self.corrections,
            "goal_time_s": round(self.t_goal, 1) if self.t_goal else "",
        }


def kill_group(proc):
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=15)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        except Exception:
            pass


def leftover_ros():
    out = subprocess.run(["pgrep", "-f", "component_container|sim_realism|ros2 launch aicontest_rover"], capture_output=True, text=True).stdout
    return [int(x) for x in out.split()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=10)
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--out", default=os.path.expanduser("~/sim_results/campaign"))
    ap.add_argument("--chair-error", type=float, default=0.015, help="椅子の位置を測るときの誤差(標準偏差 m)")
    ap.add_argument("--chair-yaw-error", type=float, default=2.0, help="椅子の向きを測るときの誤差(標準偏差 度)")
    ap.add_argument("--crater-error", type=float, default=0.02, help="クレーターの位置を測るときの誤差(標準偏差 m)")
    ap.add_argument("--place-error", type=float, default=0.03, help="ロボットを置くときの位置の誤差(一様分布の幅 ±m)")
    ap.add_argument("--place-yaw-error", type=float, default=3.0, help="ロボットを置くときの向きの誤差(±度)")
    ap.add_argument("--timeout", type=float, default=300.0, help="1回の上限 [s]")
    ap.add_argument("--drive-mode", choices=["forward", "holonomic"], help="course.yaml の drive_mode を上書き")
    ap.add_argument("--no-align", action="store_true", help="入口での補正(chair_aligner)を使わない")
    ap.add_argument("--launch-args", nargs="*", default=["odom_wheels:=2fs", "use_imu:=true"], help="sim_realistic.launch.xml に渡す引数")
    args = ap.parse_args()

    import rclpy
    rclpy.init()
    os.makedirs(args.out, exist_ok=True)
    base = yaml.safe_load(open(os.path.join(make_map.package_src_dir(), "config", "map", "course.yaml")))
    if args.drive_mode:
        base["drive_mode"] = args.drive_mode
    if args.no_align:
        args.launch_args = list(args.launch_args) + ["chair_align:=false"]
    rng = random.Random(args.seed)
    rows = []
    fields = ["run", "seed", "status", "chairs_passed", "worst_chair_cm", "leg_min_cm", "leg_contacts", "leg_min_at", "crater_min_cm", "crater_contacts", "crater_min_at",
              "loc_err_mean_cm", "loc_err_95_cm", "loc_err_max_cm", "corrections", "goal_time_s", "final_to_goal_cm", "place_err", "launch_args"]
    csv_path = os.path.join(args.out, "results.csv")
    with open(csv_path, "w", newline="") as fp:
        csv.DictWriter(fp, fieldnames=fields).writeheader()

    for r in range(args.runs):
        run_dir = os.path.join(args.out, f"run{r:03d}")
        world_dir, meas_dir = os.path.join(run_dir, "world"), os.path.join(run_dir, "measured")
        os.makedirs(world_dir, exist_ok=True)
        os.makedirs(meas_dir, exist_ok=True)
        world = random_course.random_layout(base, rng)
        meas = measured_layout(world, rng, args.chair_error, args.chair_yaw_error, args.crater_error)
        try:
            make_map.fit_crater_margin(meas)
            make_map.make_route_auto(meas, os.path.join(meas_dir, "course.md"), verbose=False)  # 本番と同じく、検査して自動で直す
        except RuntimeError as e:
            print(f"run {r}: route planning failed: {e}", flush=True)
            continue
        for d, c in [(world_dir, world), (meas_dir, meas)]:
            yaml.safe_dump(c, open(os.path.join(d, "course.yaml"), "w"), allow_unicode=True, sort_keys=False)
            make_map.make_map(c, d)
        pe = (rng.uniform(-args.place_error, args.place_error), rng.uniform(-args.place_error, args.place_error),
              rng.uniform(-args.place_yaw_error, args.place_yaw_error))

        for attempt in range(2):
            # シミュレーターの起動に失敗することがまれにある(composable node の読み込みが時間切れになる)。
            # 60秒たっても動き出さない(/待機 から先へ進まない)ときは、同じ配置でもう一度だけやり直す
            for pid in leftover_ros():
                try:
                    os.kill(pid, signal.SIGKILL)
                except Exception:
                    pass
            cmd = ["ros2", "launch", "aicontest_rover", "sim_realistic.launch.xml", "launch_visualization:=false",
                   f"course_dir:={meas_dir}", f"world_dir:={world_dir}",
                   f"place_error_x:={pe[0]:.3f}", f"place_error_y:={pe[1]:.3f}", f"place_error_yaw_deg:={pe[2]:.2f}"] + args.launch_args
            log = open(os.path.join(run_dir, "launch.log"), "w")
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            node = rclpy.create_node(f"campaign_eval_{r}_{attempt}")
            ev = Evaluator(node, world, traj_path=os.path.join(run_dir, "traj.csv"))
            status = "timeout"
            log_path = os.path.join(run_dir, "launch.log")
            t_exit = None
            started = False
            try:
                while time.time() - ev.t0 < args.timeout:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    # ルートの最後(/_exit)まで状態遷移したら、ロボットが止まるのを少し待って終わる
                    if t_exit is None and int((time.time() - ev.t0) * 20) % 20 == 0:
                        with open(log_path, errors="ignore") as lf:
                            text = lf.read()
                        started = started or "from state /待機" in text
                        if "to state /_exit" in text:
                            t_exit = time.time()
                        if not started and time.time() - ev.t0 > 60.0:
                            status = "sim_error"
                            break
                    if t_exit is not None and time.time() - t_exit > 3.0:
                        status = "goal"
                        ev.t_goal = ev.t_goal or round(t_exit - ev.t0, 1)
                        break
                    if proc.poll() is not None:
                        status = "launch_exited"
                        break
            finally:
                kill_group(proc)
                log.close()
                node.destroy_node()
            if status != "sim_error":
                break
            print(f"run {r}: the simulator did not start (attempt {attempt + 1}), retrying", flush=True)
            time.sleep(5)
        res = ev.result()
        gx, gy = world["goal"]["x"], world["goal"]["y"]
        near_goal = ev.last is not None and math.hypot(ev.last[0] - gx, ev.last[1] - gy) < 0.10
        res["final_to_goal_cm"] = round(math.hypot(ev.last[0] - gx, ev.last[1] - gy) * 100, 1) if ev.last else ""
        ok = (status == "goal" and near_goal and res["chairs_passed"] == len(world["chairs"])
              and res["leg_contacts"] == 0 and res["crater_contacts"] == 0)
        row = {"run": r, "seed": args.seed, "status": ("OK" if ok else "NG") + f"({status})", **res,
               "place_err": f"{pe[0]:+.3f},{pe[1]:+.3f},{pe[2]:+.1f}", "launch_args": " ".join(args.launch_args)}
        rows.append(row)
        with open(csv_path, "a", newline="") as fp:
            csv.DictWriter(fp, fieldnames=fields).writerow(row)
        print(f"run {r:3d}: {row['status']:16s} chairs {res['chairs_passed']}/8 leg_min {res['leg_min_cm']:5.1f}cm "
              f"contacts leg {res['leg_contacts']} crater {res['crater_contacts']} loc95 {res['loc_err_95_cm']}cm "
              f"corr {res['corrections']} t {res['goal_time_s']}s", flush=True)
        time.sleep(3)

    rclpy.try_shutdown()
    n = len(rows)
    if n == 0:
        return
    ok = sum(1 for r in rows if r["status"].startswith("OK"))
    times = sorted(r["goal_time_s"] for r in rows if r["goal_time_s"] != "")
    legmins = sorted(r["leg_min_cm"] for r in rows)
    with open(os.path.join(args.out, "summary.md"), "w") as fp:
        fp.write(f"# シミュレーション結果のまとめ\n\n- 回数: {n}(seed {args.seed})\n- 起動引数: {' '.join(args.launch_args)}\n- 走り方: {base.get('drive_mode', 'forward')}\n")
        fp.write(f"- 誤差: 椅子の位置 σ{args.chair_error * 100:.1f}cm・向き σ{args.chair_yaw_error}°、クレーター σ{args.crater_error * 100:.1f}cm、"
                 f"置き位置 ±{args.place_error * 100:.0f}cm・±{args.place_yaw_error}°\n\n")
        fp.write(f"| 項目 | 結果 |\n|---|---|\n| 成功(8脚通過・ゴール・接触なし) | {ok}/{n}({100.0 * ok / n:.0f}%) |\n")
        fp.write(f"| 椅子の脚に接触した回 | {sum(1 for r in rows if r['leg_contacts'] > 0)}/{n} |\n")
        fp.write(f"| クレーターに接触した回 | {sum(1 for r in rows if r['crater_contacts'] > 0)}/{n} |\n")
        fp.write(f"| ゴールまで行けた回 | {len(times)}/{n} |\n")
        if times:
            fp.write(f"| ゴールまでの時間(中央値 / 最大) | {times[len(times) // 2]} / {times[-1]} 秒 |\n")
        fp.write(f"| 脚との最短距離(最小 / 中央値) | {legmins[0]} / {legmins[len(legmins) // 2]} cm |\n")
        fp.write(f"\n1回ごとの結果: results.csv\n")
    print(open(os.path.join(args.out, "summary.md")).read())


if __name__ == "__main__":
    main()
