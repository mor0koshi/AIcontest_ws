#!/usr/bin/python3
"""椅子の入口で止まっているときに、LiDAR で近くの椅子の脚を測り、自己位置のずれを求める(案2)。

  入口(椅子の中心から約 0.35m)では、脚までの距離が 0.2〜0.5m なので、A1 でも 1点の間隔が 1cm 以下になり、
  脚の位置を約 1cm で測れる。地図上の脚の位置(course.yaml)と、実際に見えた脚の位置の差から、
  「椅子に対するロボットの本当の位置と向き」を求めて /localization/pose_correction に出す。
  pose_extrapolator.py がそれを受け取り、椅子を通り抜けるあいだ /localization/current_pose を補正する。

  - 椅子の位置を手で測ったときの誤差と、自己位置推定の誤差の両方を打ち消せる(大事なのは椅子との相対位置)
  - 自己位置推定(natto_mcl)には触らない(/initialpose を送ると粒子が ±10cm・±30° にばらまかれ直すため)

  入力: /sensing/lidar/laserscan/front(LiDAR はロボットの中心、向き 0 の前提: config/lidar.yaml)
        /localization/current_pose、/localization/wheel_odometry/odometry(止まっているかの判定)
        /goal_pose(どの椅子に入ろうとしているかの判定)
        course.yaml(椅子の位置・向き・脚の間隔と太さ)
"""
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def fit_rigid_2d(src, dst):
    """dst ≈ R(th) src + t となる (tx, ty, th) を最小二乗で求める(2点以上)"""
    n = len(src)
    sx = sum(p[0] for p in src) / n
    sy = sum(p[1] for p in src) / n
    dx = sum(p[0] for p in dst) / n
    dy = sum(p[1] for p in dst) / n
    a = b = 0.0
    for (x1, y1), (x2, y2) in zip(src, dst):
        x1, y1, x2, y2 = x1 - sx, y1 - sy, x2 - dx, y2 - dy
        a += x1 * x2 + y1 * y2
        b += x1 * y2 - y1 * x2
    th = math.atan2(b, a)
    c, s = math.cos(th), math.sin(th)
    return dx - (c * sx - s * sy), dy - (s * sx + c * sy), th


class ChairAligner(Node):
    def __init__(self):
        super().__init__("chair_aligner")
        p = self.declare_parameter
        course_path = p("course_path", "").value
        self.trigger_min = p("trigger_min_dist", 0.22).value   # 椅子の中心からこの範囲にいて止まっていたら測る
        self.trigger_max = p("trigger_max_dist", 0.55).value   # 入口の手前(中心から約 0.50m)で測るので 0.55 まで
        self.n_scans = p("scans_per_measurement", 4).value     # 何回分のスキャンを使うか(A1 は約7Hz)
        self.gate = p("association_gate", 0.10).value          # 地図から予想した脚の位置から、この距離以内の点を使う
        self.max_shift = p("max_correction_m", 0.12).value     # これより大きい補正は信用しない
        self.max_rot = math.radians(p("max_correction_deg", 10.0).value)
        self.max_rms = p("max_residual_m", 0.015).value
        self.search = p("search_m", 0.14).value                # 脚が見つからないとき、予想位置をずらして探す範囲
        self.max_shift_searched = p("max_correction_searched_m", 0.18).value  # ずらして見つけたときの補正の上限
        self.max_shift_strong = p("max_correction_strong_m", 0.15).value      # 脚3本以上がよく合っているときの上限
        self.max_rot_strong = math.radians(p("max_correction_strong_deg", 15.0).value)
        self.axis_gate = p("axis_gate_m", 0.10).value          # 目標がないとき: ロボットが軸からこれ以上横にずれていたら測らない
        self.goal_axis_gate = p("goal_axis_gate_m", 0.02).value  # 目標が軸からこれ以上ずれていたら、入口の手前の点ではない
        self.goal_reach = p("goal_reach_m", 0.10).value        # ロボットが目標からこれ以上離れていたら、まだ着いていない
        self.range_min = p("range_min", 0.15).value
        self.range_max = p("range_max", 0.8).value
        # LiDAR の取り付け位置と向き(ロボットの中心から。config/lidar.yaml・URDF と合わせる)
        self.still_window = p("still_window_sec", 0.3).value  # この時間の位置の変化で「止まっている」を判定
        self.still_dist = p("still_dist_m", 0.003).value
        self.still_rot = math.radians(p("still_rot_deg", 0.5).value)
        self.odom_hist = []
        self.lidar = (p("lidar_x", 0.0).value, p("lidar_y", 0.0).value, math.radians(p("lidar_yaw_deg", 0.0).value))

        c = yaml.safe_load(open(course_path))
        ch = c.get("chair", {})
        self.leg_r = float(ch.get("leg_size", 0.02)) / 2.0
        hx = float(ch.get("inner_x", 0.31)) / 2.0 + self.leg_r
        hy = float(ch.get("inner_y", 0.31)) / 2.0 + self.leg_r
        self.chairs = []
        for k in c.get("chairs") or []:
            th = math.radians(float(k.get("yaw", 0.0)))
            legs = []
            for sx, sy in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
                lx, ly = sx * hx, sy * hy
                legs.append((k["x"] + lx * math.cos(th) - ly * math.sin(th), k["y"] + lx * math.sin(th) + ly * math.cos(th)))
            self.chairs.append(((k["x"], k["y"]), legs, th))

        self.pub = self.create_publisher(PoseStamped, "/localization/pose_correction", 10)
        # 補正に使った椅子の中心(pose_extrapolator.py が、椅子を通り抜けたあとに補正を戻すのに使う)
        self.pub_chair = self.create_publisher(PointStamped, "/localization/pose_correction_chair", 10)
        self.create_subscription(PoseStamped, "/localization/current_pose", self.on_pose, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)  # 状態機械の set_pose が出す目標
        self.create_subscription(Odometry, "/localization/wheel_odometry/odometry", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/sensing/lidar/laserscan/front", self.on_scan, qos_profile_sensor_data)

        self.pose = None
        self.goal = None
        self.still_since = None
        self.scans = []
        self.done_chair = None  # 測り終えた椅子(離れるまで測り直さない)
        self.scan_chair = None  # self.scans を貯めている椅子
        self.get_logger().info(f"{len(self.chairs)} chairs from {course_path}")

    def on_pose(self, msg):
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation))

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def on_odom(self, msg):
        """止まっているかは、速度ではなく「直近 0.3 秒の位置の変化」で判定する
        (計測輪の速度はノイズやカウントの粗さで揺れるので、速度のしきい値だとすぐ途切れる)"""
        now = self.get_clock().now().nanoseconds * 1e-9
        q = msg.pose.pose
        self.odom_hist.append((now, q.position.x, q.position.y, yaw_of(q.orientation)))
        while self.odom_hist and now - self.odom_hist[0][0] > self.still_window:
            self.odom_hist.pop(0)
        if not self.odom_hist or now - self.odom_hist[0][0] < self.still_window * 0.8:
            return
        t0, x0, y0, a0 = self.odom_hist[0]
        moved = max(math.hypot(x - x0, y - y0) for _, x, y, _ in self.odom_hist)
        turned = max(abs(wrap(a - a0)) for _, _, _, a in self.odom_hist)
        if moved < self.still_dist and turned < self.still_rot:
            if self.still_since is None:
                self.still_since = now
        else:
            self.still_since = None
            self.scans = []

    def target_chair(self):
        """これから入る椅子を選ぶ。いちばん近い椅子を選ぶと、椅子どうしが近いときに隣の椅子を測ってしまい、
        その椅子の測定誤差に合わせて補正してしまう(シミュレーションで脚に接触した原因)。

        ルートの入口の手前の点(と入口)は、必ず椅子の通り抜けの軸の上に置いてある。そこで、
        状態機械が最後に出した目標(/goal_pose)が「ある椅子の軸の上で、中心から測る範囲の距離」にあり、
        ロボットがその目標に着いている(goal_reach_m 以内)ときだけ、その椅子を測る。
        目標を受け取っていないとき(手動で動かしているときなど)は、ロボットの位置で同じように判定する
        (自己位置の誤差の分、軸からのずれを axis_gate_m まで許す)"""
        x, y, _ = self.pose
        if self.goal is not None:
            if math.hypot(self.goal[0] - x, self.goal[1] - y) > self.goal_reach:
                return None  # まだ目標に着いていない(途中で止まっているだけ)
            rx, ry, gate = self.goal[0], self.goal[1], self.goal_axis_gate
        else:
            rx, ry, gate = x, y, self.axis_gate
        best = None
        for i, ((cx, cy), _, th) in enumerate(self.chairs):
            dx, dy = rx - cx, ry - cy
            d = math.hypot(dx, dy)
            if not (self.trigger_min <= d <= self.trigger_max):
                continue
            u = math.cos(th) * dx + math.sin(th) * dy
            v = -math.sin(th) * dx + math.cos(th) * dy
            lateral = min(abs(u), abs(v))  # 4つの通り抜け方向のうち、近い軸からの横のずれ
            if lateral <= gate and (best is None or lateral < best[1]):
                best = (i, lateral)
        return None if best is None else best[0]

    def on_scan(self, msg):
        if self.pose is None or not self.chairs:
            return
        x, y, _ = self.pose
        if self.done_chair is not None:
            cx, cy = self.chairs[self.done_chair][0]
            if math.hypot(cx - x, cy - y) > self.trigger_max + 0.1:
                self.done_chair = None  # 椅子から離れたら、もう一度測れるようにする
        i = self.target_chair()
        if i is None or i == self.done_chair:
            self.scans = []
            return
        if self.scan_chair != i:
            self.scans = []  # 別の椅子のスキャンを混ぜない
            self.scan_chair = i
        if self.still_since is None:
            return
        pts = []
        lx0, ly0, lyaw = self.lidar
        for k, r in enumerate(msg.ranges):
            if math.isfinite(r) and self.range_min <= r <= self.range_max:
                a = msg.angle_min + k * msg.angle_increment + lyaw
                pts.append((lx0 + r * math.cos(a), ly0 + r * math.sin(a)))  # ロボット基準
        self.scans.append(pts)
        if len(self.scans) < self.n_scans:
            return
        believed = self.pose
        result = self.measure(i, believed)
        self.scans = []
        self.done_chair = i
        if result is None:
            return
        tx, ty, th, n_legs, rms = result
        # 測った脚 = M・(予想した脚)、M = (本当の位置)^-1 ∘ (推定した位置) → 本当の位置 = 推定した位置 ∘ M^-1
        bx, by, byaw = believed
        c, s = math.cos(-th), math.sin(-th)
        ix, iy = -(c * tx - s * ty), -(s * tx + c * ty)  # M^-1 の並進
        cb, sb = math.cos(byaw), math.sin(byaw)
        true = (bx + cb * ix - sb * iy, by + sb * ix + cb * iy, wrap(byaw - th))
        ch = PointStamped()
        ch.header.stamp = self.get_clock().now().to_msg()
        ch.header.frame_id = "map"
        ch.point.x, ch.point.y = self.chairs[i][0]
        self.pub_chair.publish(ch)  # 補正より先に出す
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.pose.position.x, out.pose.position.y = true[0], true[1]
        out.pose.orientation.z, out.pose.orientation.w = math.sin(true[2] / 2.0), math.cos(true[2] / 2.0)
        self.pub.publish(out)
        self.get_logger().info(
            f"chair {i + 1}: {n_legs} legs, rms {rms * 100:.1f} cm → correction "
            f"({(true[0] - bx) * 100:+.1f}, {(true[1] - by) * 100:+.1f}) cm, {math.degrees(wrap(true[2] - byaw)):+.1f} deg")

    def associate(self, i, believed, shift=(0.0, 0.0)):
        """地図上の脚 → ロボットから見た予想位置。予想位置(+shift)から gate 以内の点のまとまりを脚とみなす。
        戻り値: (予想位置のリスト, 測った位置のリスト)"""
        bx, by, byaw = believed
        c, s = math.cos(byaw), math.sin(byaw)
        pred, meas = [], []
        for lx, ly in self.chairs[i][1]:
            dx, dy = lx - bx, ly - by
            px, py = c * dx + s * dy, -s * dx + c * dy
            if not (self.range_min + 0.02 <= math.hypot(px, py) <= self.range_max - 0.05):
                continue
            qx, qy = px + shift[0], py + shift[1]
            near = [q for scan in self.scans for q in scan if math.hypot(q[0] - qx, q[1] - qy) < self.gate]
            if len(near) < 3:
                continue
            # 外れ値を除く: 中央値から 3cm 以内
            mx = sorted(q[0] for q in near)[len(near) // 2]
            my = sorted(q[1] for q in near)[len(near) // 2]
            near = [q for q in near if math.hypot(q[0] - mx, q[1] - my) < 0.03]
            if len(near) < 3:
                continue
            cx = sum(q[0] for q in near) / len(near)
            cy = sum(q[1] for q in near) / len(near)
            # 点は脚の表面(LiDAR から見て手前側)に乗るので、脚の半径だけ奥にずらして中心にする
            vx, vy = cx - self.lidar[0], cy - self.lidar[1]
            rr = math.hypot(vx, vy)
            cx, cy = cx + self.leg_r * vx / rr, cy + self.leg_r * vy / rr
            pred.append((px, py))
            meas.append((cx, cy))
        return pred, meas

    def search_shift(self, i, believed):
        """脚が見つからないとき(椅子の位置の測定誤差 + 自己位置の誤差が gate を超えたとき): 予想した脚の位置を
        ±search_m の範囲で 2cm ずつずらし、脚の近く(3cm)に入る点がいちばん多いずらし方を探す。3本以上の脚に点が
        乗るときだけ使う(2本だと別の脚や壁と取り違えても合ってしまうことがあるため)"""
        bx, by, byaw = believed
        c, s = math.cos(byaw), math.sin(byaw)
        legs = []
        for lx, ly in self.chairs[i][1]:
            dx, dy = lx - bx, ly - by
            px, py = c * dx + s * dy, -s * dx + c * dy
            if self.range_min + 0.02 <= math.hypot(px, py) <= self.range_max - 0.05:
                legs.append((px, py))
        if len(legs) < 3:
            return None
        R = self.search + 0.04
        pts = [q for scan in self.scans for q in scan if any(math.hypot(q[0] - px, q[1] - py) < R for px, py in legs)]
        n = int(round(self.search / 0.02))
        best = None
        for ix in range(-n, n + 1):
            for iy in range(-n, n + 1):
                sx, sy = ix * 0.02, iy * 0.02
                counts = [sum(1 for q in pts if (q[0] - px - sx) ** 2 + (q[1] - py - sy) ** 2 < 0.03 ** 2) for px, py in legs]
                n_legs = sum(1 for k in counts if k >= 3)
                if n_legs < 3:
                    continue
                key = (n_legs, sum(counts), -math.hypot(sx, sy))
                if best is None or key > best[0]:
                    best = (key, (sx, sy))
        return None if best is None else best[1]

    def measure(self, i, believed):
        pred, meas = self.associate(i, believed)
        searched = False
        if len(meas) < 2:
            shift = self.search_shift(i, believed)
            if shift is not None:
                pred, meas = self.associate(i, believed, shift)
                searched = len(meas) >= 3
                if searched:
                    self.get_logger().info(f"chair {i + 1}: legs found after shifting the prediction by "
                                           f"({shift[0] * 100:+.0f}, {shift[1] * 100:+.0f}) cm")
                else:
                    pred, meas = [], []
        if len(meas) < 2:
            self.get_logger().warn(f"chair {i + 1}: only {len(meas)} legs found, no correction")
            return None
        tx, ty, th = fit_rigid_2d(pred, meas)
        ct, st = math.cos(th), math.sin(th)
        rms = math.sqrt(sum((ct * p[0] - st * p[1] + tx - m[0]) ** 2 + (st * p[0] + ct * p[1] + ty - m[1]) ** 2
                            for p, m in zip(pred, meas)) / len(meas))
        # 脚3本以上がよく合っている(rms が小さい)ときは、大きな補正も信用する(椅子の向きの測定誤差が 10°を超えることがある)
        strong = len(meas) >= 3 and rms <= self.max_rms
        max_shift = self.max_shift_searched if searched else (self.max_shift_strong if strong else self.max_shift)
        max_rot = self.max_rot_strong if strong else self.max_rot
        if math.hypot(tx, ty) > max_shift or abs(th) > max_rot or rms > self.max_rms:
            self.get_logger().warn(f"chair {i + 1}: correction rejected (shift {math.hypot(tx, ty) * 100:.1f} cm, "
                                   f"rot {math.degrees(th):.1f} deg, rms {rms * 100:.1f} cm)")
            return None
        return tx, ty, th, len(meas), rms


def main():
    rclpy.init()
    node = ChairAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
