#!/usr/bin/python3
"""スタート位置で LiDAR をしばらく貯めて椅子を見つけ、course.yaml の chairs を作る(案1)。

  # real.launch.xml(またはシミュレーション)を起動して、自己位置が落ち着いてから(ロボットは止めたまま)
  ros2 run aicontest_rover scan_chairs.py --seconds 5 --out ~/AIcontest_ws/src/aicontest_rover/config/map/course.yaml
  ros2 run aicontest_rover make_map.py      # 地図とルートを作り直す → launch を起動し直す

- 入力の course.yaml(--course、既定はパッケージのもの)のクレーター・スタート・ゴールなどはそのまま、chairs だけ置き換える
- 見つからなかった椅子は手で足す(--keep-manual を付けると、入力の chairs のうち、見つかった椅子と重ならないものを残す)
- 椅子の位置は「ロボットが思っている自分の位置」を基準にするので、先に自己位置推定を落ち着かせる(誤差 1cm・0.5° で椅子の誤差は平均約2cm)。
  残りの誤差は各椅子の入口での補正(chair_aligner.py)が直す
- LiDAR の取り付け位置・向きは --lidar-x/--lidar-y/--lidar-yaw(config/lidar.yaml・URDF と合わせる)
- 細い脚に当てるため、A1 は angle_compensate を切った生の角度のほうがよい(1° の決まった角度に丸めると、
  遠くの脚がいつも点と点のすき間に入ることがある)
"""
import argparse
import math
import os
import sys
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import chair_detector  # noqa: E402
import make_map  # noqa: E402


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--course", default=os.path.join(make_map.package_src_dir(), "config", "map", "course.yaml"))
    ap.add_argument("--out", required=True, help="書き出す course.yaml(--course と同じでもよい)")
    ap.add_argument("--keep-manual", action="store_true", help="入力の chairs のうち、見つからなかったものを残す")
    ap.add_argument("--lidar-x", type=float, default=0.0)
    ap.add_argument("--lidar-y", type=float, default=0.0)
    ap.add_argument("--lidar-yaw", type=float, default=0.0, help="度")
    args = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    course = yaml.safe_load(open(args.course))
    rclpy.init()
    node = rclpy.create_node("scan_chairs")
    state = {"pose": None, "pts": [], "scans": 0, "moved": False, "start": None}

    def on_pose(m):
        p = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))
        if state["start"] is None:
            state["start"] = p
        elif math.hypot(p[0] - state["start"][0], p[1] - state["start"][1]) > 0.02:
            state["moved"] = True
        state["pose"] = p

    def on_scan(m):
        if state["pose"] is None:
            return
        x, y, th = state["pose"]
        c, s = math.cos(th), math.sin(th)
        lyaw = math.radians(args.lidar_yaw)
        for k, r in enumerate(m.ranges):
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                a = m.angle_min + k * m.angle_increment + lyaw
                bx, by = args.lidar_x + r * math.cos(a), args.lidar_y + r * math.sin(a)
                state["pts"].append((x + c * bx - s * by, y + s * bx + c * by))
        state["scans"] += 1

    node.create_subscription(PoseStamped, "/localization/current_pose", on_pose, 10)
    node.create_subscription(LaserScan, "/sensing/lidar/laserscan/front", on_scan, qos_profile_sensor_data)
    print(f"collecting {args.seconds} s of scans (keep the robot still)...", flush=True)
    t0 = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if t0 is None and state["pose"] is not None and state["scans"] > 0:
            t0 = time.time()
        if t0 is not None and time.time() - t0 > args.seconds:
            break
    if state["moved"]:
        print("warning: the robot moved during the scan; results may be off", file=sys.stderr)
    x, y, th = state["pose"]
    sensor = (x + math.cos(th) * args.lidar_x - math.sin(th) * args.lidar_y, y + math.sin(th) * args.lidar_x + math.cos(th) * args.lidar_y)
    dets = chair_detector.detect_chairs(state["pts"], course, sensor)
    print(f"{state['scans']} scans, {len(state['pts'])} points → {len(dets)} chairs")
    for d in dets:
        print(f"  ({d['x']:.3f}, {d['y']:.3f}) yaw {d['yaw_deg']:.1f} deg, legs {d['legs']}, fit {d['rms'] * 100:.1f} cm")
    chairs = [{"x": round(d["x"], 3), "y": round(d["y"], 3), "yaw": round(d["yaw_deg"], 1)} for d in dets]
    if args.keep_manual:
        for k in course.get("chairs") or []:
            if all(math.hypot(k["x"] - d["x"], k["y"] - d["y"]) > 0.2 for d in dets):
                chairs.append({kk: k[kk] for kk in ("x", "y", "yaw") if kk in k})
                print(f"  kept manual chair ({k['x']}, {k['y']})")
    course["chairs"] = chairs
    course["auto_route"] = True  # 見つけた順は回る順ではないので、順番と向きは自動で決める
    yaml.safe_dump(course, open(args.out, "w"), allow_unicode=True, sort_keys=False)
    print(f"wrote {args.out} ({len(chairs)} chairs). next: ros2 run aicontest_rover make_map.py {args.out}")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
