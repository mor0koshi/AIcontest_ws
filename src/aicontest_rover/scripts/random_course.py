#!/usr/bin/python3
"""ランダムなコース配置を作って、ルートの自動計画が成り立つか調べる(本番の配置はランダムなので)。

  ros2 run aicontest_rover random_course.py --trials 500            # 500通り試して、成功率を出す
  ros2 run aicontest_rover random_course.py --trials 1 --write out/  # 1通り作って course.yaml・地図・ルートを out/ に書く

配置のルール(仮、ルール確定後に合わせる):
  - 椅子8脚(向きは 0〜90° のランダム)、クレーター2つ(直径42cm)、中央にゴールのコーン
  - スタート位置と向きもランダム
  - 椅子どうし・椅子とクレーター・スタート・コーンが近すぎない
"""
import argparse
import copy
import math
import os
import random
import sys
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import make_map  # noqa: E402


def random_layout(base, rng, n_chairs=8, n_craters=2, chair_yaw_any=True):
    c = copy.deepcopy(base)
    W, H = c["field"]["width"], c["field"]["height"]
    cone = (c["cone"]["x"], c["cone"]["y"])
    for _ in range(10000):
        # スタート位置と向きもランダム(本番は毎回変わる。壁から 0.3m 以上、コーンから 0.8m 以上)
        while True:
            start = (rng.uniform(0.3, W - 0.3), rng.uniform(0.3, H - 0.3))
            if math.dist(start, cone) > 0.8:
                break
        c["start"] = {"x": round(start[0], 3), "y": round(start[1], 3), "yaw": round(rng.uniform(-180, 180), 1)}
        chairs, craters, ok = [], [], True
        for _ in range(n_craters):
            for _ in range(200):
                p = (rng.uniform(0.4, W - 0.4), rng.uniform(0.4, H - 0.4))
                if math.dist(p, start) > 0.6 and math.dist(p, cone) > 0.6 and all(math.dist(p, q) > 0.7 for q in craters):
                    craters.append(p)
                    break
            else:
                ok = False
        for _ in range(n_chairs):
            for _ in range(500):
                p = (rng.uniform(0.45, W - 0.45), rng.uniform(0.45, H - 0.45))
                if (math.dist(p, start) > 0.7 and math.dist(p, cone) > 0.7
                        and all(math.dist(p, q) > 0.6 for q in craters)
                        and all(math.dist(p, (q["x"], q["y"])) > 0.8 for q in chairs)):
                    yaw = rng.uniform(0, 90) if chair_yaw_any else 0.0
                    chairs.append({"x": round(p[0], 3), "y": round(p[1], 3), "yaw": round(yaw, 1)})
                    break
            else:
                ok = False
        if ok:
            c["craters"] = [[round(x, 3), round(y, 3)] for x, y in craters]
            c["chairs"] = chairs
            c["auto_route"] = True
            return c
    raise RuntimeError("could not place layout")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=200)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--axis-aligned", action="store_true", help="椅子の向きを 0° に固定")
    ap.add_argument("--write", help="最後の配置の course.yaml・地図・ルートをこのフォルダに書く")
    args = ap.parse_args()

    base_path = os.path.join(make_map.package_src_dir(), "config", "map", "course.yaml")
    base = yaml.safe_load(open(base_path))
    rng = random.Random(args.seed)
    ok, fails, costs, times = 0, [], [], []
    for t in range(args.trials):
        c = random_layout(base, rng, chair_yaw_any=not args.axis_aligned)
        t0 = time.time()
        try:
            make_map.fit_crater_margin(c)
            plan, cost = make_map.plan_route(c)
            ok += 1
            costs.append(cost)
        except RuntimeError as e:
            fails.append(str(e))
        times.append(time.time() - t0)
        if args.write and t == args.trials - 1:
            os.makedirs(args.write, exist_ok=True)
            yaml.safe_dump(c, open(os.path.join(args.write, "course.yaml"), "w"), allow_unicode=True, sort_keys=False)
            make_map.make_map(c, args.write)
            make_map.make_route(c, os.path.join(args.write, "course.md"))
    print(f"{ok}/{args.trials} layouts planned ({100.0 * ok / args.trials:.1f}%), "
          f"planning time mean {1000 * sum(times) / len(times):.0f} ms, max {1000 * max(times):.0f} ms")
    if costs:
        costs.sort()
        print(f"route cost [m-equivalent]: median {costs[len(costs) // 2]:.1f}, max {costs[-1]:.1f}")
    for f in fails[:5]:
        print("  fail:", f)


if __name__ == "__main__":
    main()
