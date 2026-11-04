#!/usr/bin/python3
"""椅子の周りだけ速度を落とす。

natto の chassis_calculator が出す車輪の指令(/omni/command_joint_states)を受け取り、
ロボットが椅子の中心から zone_radius 以内にいるあいだは、機体の速さが slow_speed 以下・旋回が slow_yaw_rate 以下に
なるように4輪の指令を同じ割合で縮めて /omni/command_joint_states_limited に出す(進む向きは変わらない)。
joint_state_merger が読むトピックの一覧(launch の joint_state_topics)を limited に差し替えて使う。natto 本体は変更なし。

経路追従(holonomic_pure_pursuit)の最高速度は1つしか決められないので、場所によって速さを変えるためにこの方法にした。
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
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


class ZoneSpeedLimiter(Node):
    def __init__(self):
        super().__init__("zone_speed_limiter")
        p = self.declare_parameter
        course_path = p("course_path", "").value
        self.zone_radius = p("zone_radius", 0.45).value     # 椅子の中心からこの距離以内で遅くする [m]
        self.slow_speed = p("slow_speed", 0.15).value       # 椅子の周りでの最高速度 [m/s]
        self.slow_yaw = p("slow_yaw_rate", 0.5).value       # 椅子の周りでの最高旋回速度 [rad/s]
        self.wheel_names = list(p("wheel_names", ["mecanum_fl", "mecanum_fr", "mecanum_rl", "mecanum_rr"]).value)
        self.wheel_radius = p("wheel_radius", 0.0212132).value  # mecanum.yaml と同じ(r/√2)
        c = yaml.safe_load(open(course_path)) if course_path else {}
        self.chairs = [(k["x"], k["y"]) for k in (c.get("chairs") or [])]

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.rows = None
        self.pose = None
        self.inside = False
        self.pub = self.create_publisher(JointState, p("output_topic", "/omni/command_joint_states_limited").value, qos_profile_sensor_data)
        self.create_subscription(JointState, p("input_topic", "/omni/command_joint_states").value, self.on_cmd, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/localization/current_pose", self.on_pose, 10)
        self.get_logger().info(f"{len(self.chairs)} chairs: within {self.zone_radius} m → max {self.slow_speed} m/s, {self.slow_yaw} rad/s")

    def on_pose(self, msg):
        self.pose = (msg.pose.position.x, msg.pose.position.y)

    def load_rows(self):
        rows = []
        for n in self.wheel_names:
            t = self.tf.lookup_transform("command/base_link", f"command/{n}_base_link", rclpy.time.Time())
            x, y = t.transform.translation.x, t.transform.translation.y
            yaw = yaw_of(t.transform.rotation)
            c, s = math.cos(yaw), math.sin(yaw)
            rows.append((c, s, -c * y + s * x))
        self.rows = rows

    def on_cmd(self, msg):
        scale = 1.0
        near = self.pose is not None and any(math.hypot(self.pose[0] - x, self.pose[1] - y) < self.zone_radius for x, y in self.chairs)
        if near != self.inside:
            self.inside = near
            self.get_logger().info("entering chair zone: slow" if near else "leaving chair zone")
        if near:
            if self.rows is None:
                try:
                    self.load_rows()
                except Exception as e:
                    self.get_logger().warn(f"waiting for TF: {e}", throttle_duration_sec=2.0)
            vel = {n: v for n, v in zip(msg.name, msg.velocity)}
            if self.rows is not None and all(n in vel for n in self.wheel_names):
                ata = [[sum(r[i] * r[j] for r in self.rows) for j in range(3)] for i in range(3)]
                atb = [sum(r[i] * vel[n] * self.wheel_radius for r, n in zip(self.rows, self.wheel_names)) for i in range(3)]
                v = solve3(ata, atb)
                if v is not None:
                    speed, wz = math.hypot(v[0], v[1]), abs(v[2])
                    if speed > self.slow_speed:
                        scale = min(scale, self.slow_speed / speed)
                    if wz > self.slow_yaw:
                        scale = min(scale, self.slow_yaw / wz)
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = [v * scale for v in msg.velocity]
        out.effort = list(msg.effort)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ZoneSpeedLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
