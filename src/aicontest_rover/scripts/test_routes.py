#!/usr/bin/python3
"""ルート生成(make_map.py)の自動テスト(ROS なし)。ランダムな配置で、作ったルートに問題がないか調べる。

  python3 test_routes.py --trials 200
  python3 test_routes.py --course ~/AIcontest_ws/src/aicontest_rover/config/map/course.yaml   # 本番の配置だけ

調べること:
  1. 続く2つの set_pose の位置が 0.1m 以上離れている
     (natto の astar_planner は 0.1m 以内の目標を「前と同じ」とみなし、向きが違っても経路を作らないことがある)
  2. 途中の点(_向き・_手前)が壁・クレーター・コーン・ほかの椅子に近すぎない
  3. 椅子ごとに 入口 → (停止) → 出口 がそろっていて、入口と出口が椅子の中心をはさんで反対側にある
  4. 続く2つの点を結ぶ直線が、両端より椅子に近づかない(椅子の出口から斜めに動いて、いま出た椅子の
     角をかすめるルートなどを見つける。入口 → 出口は通り抜ける椅子を除いて調べる。
     壁・クレーター・コーンは地図に入っていて A* が避けるので調べない)
"""
import argparse
import math
import os
import random
import re
import sys
import tempfile

import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import make_map  # noqa: E402
import route_check  # noqa: E402
import random_course  # noqa: E402

POSE = re.compile(r"--> (\S+) : set_pose\(x = ([-\d.]+), y = ([-\d.]+), yaw = ([-\d.]+)\)")


def check(c, path):
    r = route_check.check_route(c, path)
    return r["errs"], r["n"], r["warns"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=100)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--drive-mode", default=None)
    ap.add_argument("--course", help="ランダムな配置ではなく、この course.yaml(本番の配置)のルートだけを調べる")
    ap.add_argument("--no-auto", action="store_true", help="ルートの自動修正(問題が出た椅子の向きを変える)をしない")
    args = ap.parse_args()
    base = yaml.safe_load(open(args.course or os.path.join(make_map.package_src_dir(), "config", "map", "course.yaml")))
    if args.drive_mode:
        base["drive_mode"] = args.drive_mode
    if args.course:
        with tempfile.TemporaryDirectory() as td:
            make_map.fit_crater_margin(base)
            path = os.path.join(td, "course.md")
            if args.no_auto:
                make_map.make_route(base, path)
                errs, n, warns = check(base, path)
            else:
                _, _, changed, r = make_map.make_route_auto(base, path, verbose=False)
                errs, n, warns = r["errs"], r["n"], r["warns"]
                for j, v in sorted(changed.items(), key=lambda kv: str(kv[0])):
                    print(f"  (auto) chair {j + 1}: force_pass {v:.1f}" if j != "order" else f"  (auto) re-ordered (chair_cross_weight {v:.0f})")
        for e in errs:
            print("  " + e)
        for w in warns:
            print("  warning: " + w)
        print(f"{args.course}: {'OK' if not errs else f'{len(errs)} problems'} ({n} set_pose)")
        sys.exit(1 if errs else 0)
    rng = random.Random(args.seed)
    bad, steps = 0, []
    with tempfile.TemporaryDirectory() as td:
        for t in range(args.trials):
            c = random_course.random_layout(base, rng)
            make_map.fit_crater_margin(c)
            path = os.path.join(td, "course.md")
            if args.no_auto:
                make_map.make_route(c, path)
                errs, n, _ = check(c, path)
            else:
                _, _, _, r = make_map.make_route_auto(c, path, verbose=False)
                errs, n = r["errs"], r["n"]
            steps.append(n)
            if errs:
                bad += 1
                if bad <= 5:
                    print(f"layout {t}: " + "; ".join(errs[:4]))
    print(f"{args.trials - bad}/{args.trials} routes OK (drive_mode {base.get('drive_mode', 'forward')}), set_pose per route: "
          f"median {sorted(steps)[len(steps) // 2]}, max {max(steps)}")


if __name__ == "__main__":
    main()
