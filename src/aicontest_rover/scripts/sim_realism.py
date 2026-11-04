#!/usr/bin/python3
"""シミュレーションに「現実の誤差」を入れるノード(sim_realistic.launch.xml で使う)。

natto の簡易シミュレーターは、車輪が指令どおりに回って滑らず、計測輪も再現しない。
このノードを間に挟んで、実機に近い条件にする。

  1. 滑り   : /command_joint_states → (機体の速度に直して、方向ごとに効きを落とす)→ /simulation/command_joint_states
              シミュレーターはこの「実際に効いた分」で動く。メカナムは特に横方向が効かない
  2. 計測輪 : /simulation/pose(真の位置)を微分 → 計測輪3つの回転 + 誤差(半径の誤差・ノイズ・一瞬の空転)
              → /joint_states(駆動輪 + 計測輪。実機の aicontest_can_bridge と同じ形)
  3. LiDAR  : /simulation/lidar/front_raw → ノイズ・取りこぼし・最小距離 → /sensing/lidar/laserscan/front

車輪の位置と向きは robot_state_publisher の TF(base_link → *_base_link)から読む(URDF と同じ値を使う)。
"""
import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def solve3(a, b):
    m = [list(a[i]) + [b[i]] for i in range(3)]
    for i in range(3):
        p = max(range(i, 3), key=lambda k: abs(m[k][i]))
        m[i], m[p] = m[p], m[i]
        if abs(m[i][i]) < 1e-12:
            return [0.0, 0.0, 0.0]
        for k in range(3):
            if k != i:
                f = m[k][i] / m[i][i]
                m[k] = [m[k][j] - f * m[i][j] for j in range(4)]
    return [m[i][3] / m[i][i] for i in range(3)]


class OU:
    """ゆっくり変化するランダムなゆらぎ(平均0、標準偏差 sigma、時定数 tau)"""

    def __init__(self, sigma, tau):
        self.sigma, self.tau, self.x = sigma, max(tau, 1e-3), 0.0

    def step(self, dt):
        a = math.exp(-dt / self.tau)
        self.x = a * self.x + self.sigma * math.sqrt(1.0 - a * a) * random.gauss(0.0, 1.0)
        return self.x


class SimRealism(Node):
    def __init__(self):
        super().__init__("sim_realism")
        p = self.declare_parameter
        self.drive_names = list(p("drive_wheel_names", ["mecanum_fl", "mecanum_fr", "mecanum_rl", "mecanum_rr"]).value)
        self.drive_radius = p("drive_wheel_radius", 0.0212132).value  # r/√2(mecanum.yaml と同じ)
        self.odom_names = list(p("odom_wheel_names", ["odom_wheel_l", "odom_wheel_r", "odom_wheel_s"]).value)
        self.odom_radius = p("odom_wheel_radius", 0.024).value
        # 滑り(機体の速度のうち実際に効く割合)
        self.k = [p("slip_efficiency_x", 0.97).value, p("slip_efficiency_y", 0.70).value, p("slip_efficiency_yaw", 0.92).value]
        fluct = p("slip_fluctuation", 0.05).value
        self.k_fluct = [OU(fluct, 0.5) for _ in range(3)]
        self.cross = OU(p("cross_drift", 0.03).value, 0.3)  # 前後に動くと横に流れる割合
        # 計測輪の誤差
        self.odom_scale = list(p("odom_scale_error", [0.015, -0.01, 0.02]).value)  # 半径の誤差(+1.5% など)。計測輪の並び順に対応
        self.odom_scale += [0.0] * max(0, len(self.odom_names) - len(self.odom_scale))
        self.odom_noise = p("odom_noise_rad_s", 0.05).value
        self.odom_skip_rate = p("odom_skip_per_sec", 0.3).value  # 1秒あたりに計測輪が一瞬空転する回数
        self.odom_skip_dur = p("odom_skip_duration", 0.08).value
        # LiDAR
        self.scan_noise = p("lidar_noise_m", 0.01).value
        self.scan_dropout = p("lidar_dropout", 0.03).value
        self.scan_min = p("lidar_range_min", 0.15).value
        # IMU(ジャイロの z 軸だけ)。/imu に出す(使うかはオドメトリ側の設定)
        self.gyro_bias = math.radians(p("gyro_bias_deg_s", 0.5).value)        # 最初のずれ(校正前)
        self.gyro_walk = math.radians(p("gyro_bias_walk_deg_s", 0.02).value)  # ずれが 1秒あたり変わる量(標準偏差)
        self.gyro_noise = math.radians(p("gyro_noise_deg_s", 0.3).value)      # 1サンプルのノイズ(標準偏差)
        self.gyro_scale = p("gyro_scale_error", 0.005).value                  # 倍率の誤差(0.5%)
        random.seed(p("seed", 0).value or None)

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.drive_rows = None  # [(c, s, -c*y + s*x)]
        self.odom_rows = None

        self.pub_cmd = self.create_publisher(JointState, "/simulation/command_joint_states", qos_profile_sensor_data)
        self.pub_js = self.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, "/imu", qos_profile_sensor_data)
        self.pub_scan = self.create_publisher(LaserScan, "/sensing/lidar/laserscan/front", qos_profile_sensor_data)
        self.create_subscription(JointState, "/command_joint_states", self.on_cmd, qos_profile_sensor_data)
        self.create_subscription(JointState, "/simulation/drive_joint_states", self.on_drive, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/simulation/pose", self.on_pose, 10)
        self.create_subscription(LaserScan, "/simulation/lidar/front_raw", self.on_scan, qos_profile_sensor_data)

        self.drive_state = None
        self.cmd_vel = None
        self.prev_pose = None
        self.odom_pos = [0.0] * 3
        self.skip_until = [0.0] * 3
        self.last_cmd_t = None
        self.get_logger().info(
            f"slip efficiency x/y/yaw = {self.k}, odom scale error = {self.odom_scale}, "
            f"lidar noise {self.scan_noise} m, dropout {self.scan_dropout}, min {self.scan_min} m")

    # ---- 車輪の配置(TF から一度だけ読む)----
    def rows(self, names):
        out = []
        for n in names:
            t = self.tf.lookup_transform("base_link", f"{n}_base_link", rclpy.time.Time())
            x, y = t.transform.translation.x, t.transform.translation.y
            yaw = yaw_of(t.transform.rotation)
            c, s = math.cos(yaw), math.sin(yaw)
            out.append((c, s, -c * y + s * x))
        return out

    def ensure_geometry(self):
        if self.drive_rows is not None and self.odom_rows is not None:
            return True
        try:
            self.drive_rows = self.rows(self.drive_names)
            self.odom_rows = self.rows(self.odom_names)
            self.get_logger().info("wheel geometry loaded from TF")
            return True
        except Exception as e:  # TF がまだ来ていない
            self.get_logger().info(f"waiting for TF: {e}", throttle_duration_sec=2.0)
            return False

    # ---- 1. 滑り ----
    def on_cmd(self, msg):
        if not self.ensure_geometry():
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.01 if self.last_cmd_t is None else min(max(now - self.last_cmd_t, 1e-3), 0.2)
        self.last_cmd_t = now
        w = {n: v for n, v in zip(msg.name, msg.velocity)}
        if not all(n in w for n in self.drive_names):
            return
        # モーターのエンコーダーは「車輪が回った分」を数える(ロボットが滑っても指令どおりに回る)
        self.cmd_vel = [w[n] for n in self.drive_names]
        # 車輪の角速度 → 機体の速度(最小二乗)
        rows = self.drive_rows
        ata = [[sum(r[i] * r[j] for r in rows) for j in range(3)] for i in range(3)]
        atb = [sum(r[i] * w[n] * self.drive_radius for r, n in zip(rows, self.drive_names)) for i in range(3)]
        vx, vy, wz = solve3(ata, atb)
        # 方向ごとに効きを落とし、ゆらぎと横流れを足す
        f = [max(0.0, k * (1.0 + o.step(dt))) for k, o in zip(self.k, self.k_fluct)]
        drift = self.cross.step(dt) * abs(vx)
        v = (vx * f[0], vy * f[1] + drift, wz * f[2])
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = list(self.drive_names)
        out.position = [0.0] * len(rows)
        out.velocity = [sum(a * b for a, b in zip(r, v)) / self.drive_radius for r in rows]
        self.pub_cmd.publish(out)

    def on_drive(self, msg):
        self.drive_state = msg

    # ---- 2. 計測輪 ----
    def on_pose(self, msg):
        if not self.ensure_geometry():
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x, y, th = msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation)
        if self.prev_pose is None or t <= self.prev_pose[0]:
            self.prev_pose = (t, x, y, th)
            return
        t0, x0, y0, th0 = self.prev_pose
        dt = t - t0
        self.prev_pose = (t, x, y, th)
        if dt > 0.2:
            return
        dxw, dyw, dth = x - x0, y - y0, wrap(th - th0)
        thm = th0 + dth / 2.0
        vx = (dxw * math.cos(thm) + dyw * math.sin(thm)) / dt
        vy = (-dxw * math.sin(thm) + dyw * math.cos(thm)) / dt
        wz = dth / dt

        # IMU: 真の旋回の速さ × (1 + 倍率の誤差) + ずれ(ゆっくり変わる)+ ノイズ
        self.gyro_bias += random.gauss(0.0, self.gyro_walk * math.sqrt(dt))
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = "base_link"
        imu.orientation_covariance[0] = -1.0  # 向きは出さない
        imu.angular_velocity.z = wz * (1.0 + self.gyro_scale) + self.gyro_bias + random.gauss(0.0, self.gyro_noise)
        self.pub_imu.publish(imu)

        # 駆動輪: 実機のエンコーダーと同じく「滑る前の指令どおりの回転」(0.5秒指令がなければ止まっている)
        names, pos, vel = list(self.drive_names), [0.0] * len(self.drive_names), [0.0] * len(self.drive_names)
        if self.cmd_vel is not None and self.last_cmd_t is not None:
            if self.get_clock().now().nanoseconds * 1e-9 - self.last_cmd_t < 0.5:
                vel = list(self.cmd_vel)
        for j, (r, n) in enumerate(zip(self.odom_rows, self.odom_names)):
            w = (r[0] * vx + r[1] * vy + r[2] * wz) / self.odom_radius
            w *= 1.0 + self.odom_scale[j]
            if t < self.skip_until[j]:
                w *= 0.3  # 一瞬の空転(段差・浮き)
            elif random.random() < self.odom_skip_rate * dt:
                self.skip_until[j] = t + self.odom_skip_dur
            w += random.gauss(0.0, self.odom_noise)
            self.odom_pos[j] += w * dt
            names.append(n)
            pos.append(self.odom_pos[j])
            vel.append(w)
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name, out.position, out.velocity = names, pos, vel
        self.pub_js.publish(out)

    # ---- 3. LiDAR ----
    def on_scan(self, msg):
        out = msg
        rmin = max(self.scan_min, msg.range_min)
        rng = []
        for r in msg.ranges:
            if not math.isfinite(r) or r >= msg.range_max or r < rmin or random.random() < self.scan_dropout:
                rng.append(float("inf"))
            else:
                rng.append(r + random.gauss(0.0, self.scan_noise))
        out.ranges = rng
        out.range_min = float(rmin)
        self.pub_scan.publish(out)


def main():
    rclpy.init()
    node = SimRealism()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
