#!/usr/bin/python3
"""計測輪(2〜3輪)+ 駆動輪 のオドメトリ(重み付き最小二乗)。

natto_wheel_odometry は「omni の式で 3輪以上」でないと解けない(未知数が vx, vy, ωz の3つ)。
計測輪が2つの場合は、計測輪だけでは分からない成分を、重みを小さくした駆動輪(メカナム)の回転で補う。
  計測輪2つ(前後向きを左右に2つ): vx と ωz が計測輪から、vy は駆動輪から
  計測輪2つ(前後向き1 + 横向き1): vx と vy が主に計測輪から、ωz は主に駆動輪から
  計測輪3つ: ほぼ計測輪だけで決まる(natto_wheel_odometry とほぼ同じ)
  use_gyro: true … ωz は IMU のジャイロ(/imu の angular_velocity.z)を重く使う。計測輪の左右の間隔が狭いと
                   向きの誤差が大きいので、計測輪2つ(前後向き1 + 横向き1)+ IMU の組み合わせを想定
                   ジャイロのずれ(バイアス)は、計測輪が止まっているあいだの平均から求め続ける(スタートや椅子の入口で止まるとき)

入力  /joint_states(駆動輪 + 計測輪。実機は aicontest_can_bridge、シミュレーションは sim_realism.py)
出力  odometry(nav_msgs/Odometry、natto_mcl が位置の変化量を使う), pose, twist
車輪の位置と向きは TF(command/base_link → command/<名前>_base_link)から読む。
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def solve3(a, b):
    m = [list(a[i]) + [b[i]] for i in range(3)]
    for i in range(3):
        p = max(range(i, 3), key=lambda k: abs(m[k][i]))
        m[i], m[p] = m[p], m[i]
        if abs(m[i][i]) < 1e-12:
            return None
        for k in range(3):
            if k != i:
                f = m[k][i] / m[i][i]
                m[k] = [m[k][j] - f * m[i][j] for j in range(4)]
    return [m[i][3] / m[i][i] for i in range(3)]


class WheelOdometryWLS(Node):
    def __init__(self):
        super().__init__("wheel_odometry")
        p = self.declare_parameter
        self.odom_names = list(p("odom_wheel_names", ["odom_wheel_l", "odom_wheel_r", "odom_wheel_s"]).value)
        self.odom_radius = p("odom_wheel_radius", 0.024).value
        self.drive_names = list(p("drive_wheel_names", ["mecanum_fl", "mecanum_fr", "mecanum_rl", "mecanum_rr"]).value)
        self.drive_radius = p("drive_wheel_radius", 0.0212132).value
        self.drive_weight = p("drive_weight", 0.05).value  # 駆動輪の重み(計測輪 = 1)。滑るので小さく
        self.timeout = p("timeout_sec", 0.1).value
        self.odom_frame = p("odom_frame_id", "odom").value
        self.base_frame = p("base_frame_id", "base_link").value
        self.use_gyro = p("use_gyro", False).value
        self.gyro_weight = p("gyro_weight", 1000.0).value          # ジャイロの重み(計測輪 = 1)
        self.still_speed = p("still_wheel_speed", 0.003).value     # 計測輪の周速がこれ以下なら止まっている [m/s]
        self.still_time = p("still_time_sec", 0.5).value            # これだけ止まっていたらジャイロのずれを測り直す
        self.gyro = []          # 直近のジャイロ [(時刻, 値)]
        self.gyro_bias = 0.0
        self.bias_ready = False
        self.still_since = None

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.rows = None  # [(name, radius, weight, (c, s, -c*y + s*x))]

        self.pub_odom = self.create_publisher(Odometry, "odometry", qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(PoseStamped, "pose", qos_profile_sensor_data)
        self.pub_twist = self.create_publisher(TwistStamped, "twist", qos_profile_sensor_data)
        self.create_subscription(JointState, "joint_states", self.on_js, qos_profile_sensor_data)
        if self.use_gyro:
            self.create_subscription(Imu, p("gyro_topic", "/imu").value, self.on_imu, qos_profile_sensor_data)

        self.x = self.y = self.th = 0.0
        self.last_t = None
        self.get_logger().info(f"odom wheels {self.odom_names} (r={self.odom_radius}), drive wheels weight {self.drive_weight}, gyro {'on' if self.use_gyro else 'off'}")

    def load_rows(self):
        rows = []
        for names, radius, weight in [(self.odom_names, self.odom_radius, 1.0), (self.drive_names, self.drive_radius, self.drive_weight)]:
            if weight <= 0.0:
                continue
            for n in names:
                t = self.tf.lookup_transform("command/base_link", f"command/{n}_base_link", rclpy.time.Time())
                x, y = t.transform.translation.x, t.transform.translation.y
                yaw = yaw_of(t.transform.rotation)
                c, s = math.cos(yaw), math.sin(yaw)
                rows.append((n, radius, weight, (c, s, -c * y + s * x)))
        self.rows = rows
        self.get_logger().info("wheel geometry loaded from TF")

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gyro.append((t, msg.angular_velocity.z))
        while self.gyro and t - self.gyro[0][0] > 2.0:
            self.gyro.pop(0)

    def update_gyro_bias(self, t, still):
        """計測輪が止まっているあいだのジャイロの平均 = ずれ"""
        if not still:
            self.still_since = None
            return
        if self.still_since is None:
            self.still_since = t
            return
        if t - self.still_since < self.still_time:
            return
        samples = [g for tg, g in self.gyro if tg >= self.still_since + 0.1]
        if len(samples) < 10:
            return
        mean = sum(samples) / len(samples)
        if not self.bias_ready:
            self.gyro_bias, self.bias_ready = mean, True
            self.get_logger().info(f"gyro bias calibrated: {math.degrees(mean):+.3f} deg/s")
        else:
            self.gyro_bias += 0.2 * (mean - self.gyro_bias)

    def gyro_rate(self, t0, t1):
        """t0〜t1 のジャイロの平均(ずれを引いた値)。なければ None"""
        samples = [g for tg, g in self.gyro if t0 < tg <= t1]
        if not samples:
            return None
        return sum(samples) / len(samples) - self.gyro_bias

    def on_js(self, msg):
        if self.rows is None:
            try:
                self.load_rows()
            except Exception as e:
                self.get_logger().info(f"waiting for TF: {e}", throttle_duration_sec=2.0)
                return
        vel = {n: v for n, v in zip(msg.name, msg.velocity)}
        if not all(n in vel for n in self.odom_names):
            return  # 計測輪の入っていないメッセージは使わない
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        if dt <= 0.0 or dt > self.timeout:
            self.last_t = t  # 間が空いたら積算せずに合わせ直す
            return
        t_prev = self.last_t
        self.last_t = t
        if self.use_gyro:
            # 止まっている = 計測輪も駆動輪(モーターの実速度)も止まっている。計測輪だけだと、その場でゆっくり回る(約 3°/s 以下)
            # ときも止まっているように見え、その回転をジャイロのずれとして引いてしまう
            radius = {n: r for n, r, _, _ in self.rows}
            radius.update({n: self.drive_radius for n in self.drive_names})
            still = all(abs(vel[n] * radius[n]) < self.still_speed for n in self.odom_names)
            still = still and all(abs(vel[n] * radius[n]) < 2.0 * self.still_speed for n in self.drive_names if n in vel)
            self.update_gyro_bias(t, still)

        ata = [[0.0] * 3 for _ in range(3)]
        atb = [0.0] * 3
        for n, radius, w, r in self.rows:
            if n not in vel:
                continue
            b = vel[n] * radius
            for i in range(3):
                atb[i] += w * r[i] * b
                for j in range(3):
                    ata[i][j] += w * r[i] * r[j]
        if self.use_gyro and self.bias_ready:
            g = self.gyro_rate(t_prev - 0.005, t + 0.005)
            if g is not None:
                ata[2][2] += self.gyro_weight
                atb[2] += self.gyro_weight * g
        sol = solve3(ata, atb)
        if sol is None:
            self.get_logger().warn("odometry is not solvable with these wheels", throttle_duration_sec=2.0)
            return
        vx, vy, wz = sol

        thm = self.th + wz * dt / 2.0
        self.x += (vx * math.cos(thm) - vy * math.sin(thm)) * dt
        self.y += (vx * math.sin(thm) + vy * math.cos(thm)) * dt
        self.th = math.atan2(math.sin(self.th + wz * dt), math.cos(self.th + wz * dt))

        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)
        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = vx, vy, wz
        self.pub_odom.publish(od)
        ps = PoseStamped()
        ps.header = od.header
        ps.pose = od.pose.pose
        self.pub_pose.publish(ps)
        tw = TwistStamped()
        tw.header.stamp = msg.header.stamp
        tw.header.frame_id = self.base_frame
        tw.twist = od.twist.twist
        self.pub_twist.publish(tw)


def main():
    rclpy.init()
    node = WheelOdometryWLS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
