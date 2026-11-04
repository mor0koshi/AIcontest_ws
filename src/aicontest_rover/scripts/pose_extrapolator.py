#!/usr/bin/python3
"""自己位置を「MCL の推定 + その後にオドメトリで進んだ分」で、オドメトリと同じ頻度(約50〜100Hz)で出す。

natto_mcl は LiDAR のスキャンが届いたとき(A1 は約7Hz)にしか位置を出さないので、そのままだと
経路追従は最大 0.14 秒古い位置で動くことになる(0.5m/s なら 7cm)。
このノードは、MCL の位置が届いたときのオドメトリを覚えておき、それ以降にオドメトリで進んだ分を足して出す。

  入力  /localization/mcl_pose(natto_mcl の pose を remap)、/localization/wheel_odometry/odometry
        /localization/pose_correction(chair_aligner.py が椅子の脚から求めた「本当の位置」)
  出力  /localization/current_pose(natto の経路計画・経路追従・行動管理が使う)

補正: pose_correction を受け取ったら、そのときの出力との差(地図座標での回転+平行移動)を覚えて出力に足す。MCL には触らない。
補正は「その椅子に合わせた」位置なので、椅子を通り抜けたら戻す: 補正に使った椅子の中心(/localization/pose_correction_chair)から
release_radius_m(0.40m、出口 0.35m の少し先)より離れたら、そこから fade_distance_m(0.3m)進むあいだに少しずつ 0 に戻す
(椅子の真下 0.25m 以内を一度通ったあとだけ)。一気に戻すと位置が飛んで急に曲がり、続けすぎると、椅子の向きを大きく測り間違えたときに
隣の椅子に対してずれたまま近づいた(シミュレーション、14°測り間違えた椅子)。
椅子の中心が来ないときは、これまでどおり correction_distance(1.1m)進むか correction_timeout_sec(15秒)で戻す。
hold_mcl_during_correction を true にすると、補正中は MCL の新しい推定を使わずオドメトリだけで進める(既定は false。計測輪の向きの誤差で悪化したため)。
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseExtrapolator(Node):
    def __init__(self):
        super().__init__("pose_extrapolator")
        self.max_extrapolation = self.declare_parameter("max_extrapolation_sec", 0.5).value
        self.corr_dist = self.declare_parameter("correction_distance", 1.1).value  # 入口の手前(0.15m)+ 通り抜け(0.7m)より長く
        self.corr_time = self.declare_parameter("correction_timeout_sec", 15.0).value
        # true: 補正中は MCL の更新を止めてオドメトリだけで進める。計測輪の左右の間隔が狭い(10cm)と向きの誤差が大きく、
        # シミュレーションでは悪化した(脚に接触)ので既定は false
        self.hold_during_correction = self.declare_parameter("hold_mcl_during_correction", False).value
        self.release_r = self.declare_parameter("release_radius_m", 0.40).value
        self.fade = self.declare_parameter("fade_distance_m", 0.30).value
        self.create_subscription(PointStamped, "/localization/pose_correction_chair", self.on_corr_chair, 10)
        self.create_subscription(PoseStamped, "/localization/pose_correction", self.on_correction, 10)
        self.corr = None      # (回転, 平行移動 x, y, 補正した位置 x, y, 時刻)
        self.corr_chair = None  # 補正に使った椅子の中心 (x, y, 受信時刻)
        self.chair_for_corr = None  # いまの補正の椅子の中心 (x, y)
        self.passed = False   # 椅子の真下を通ったか
        self.last_raw = None  # 補正をかける前の最新の出力
        self.pub = self.create_publisher(PoseStamped, "/localization/current_pose", 10)
        self.create_subscription(PoseStamped, "/localization/mcl_pose", self.on_mcl, 10)
        self.create_subscription(Odometry, "/localization/wheel_odometry/odometry", self.on_odom, qos_profile_sensor_data)
        self.odom = None      # 最新のオドメトリ (x, y, yaw)
        self.anchor = None    # (MCL の位置 (x, y, yaw), そのときのオドメトリ (x, y, yaw), 受信時刻)
        self.frame = "map"

    def on_mcl(self, msg):
        self.frame = msg.header.frame_id or "map"
        if self.corr is not None and self.hold_during_correction and self.anchor is not None:
            return  # 補正中(椅子を通り抜けるあいだ)は MCL の更新を使わず、補正した位置 + オドメトリで進める
        p = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation))
        if self.odom is None:
            self.publish(p, msg.header.stamp)
            return
        self.anchor = (p, self.odom, self.get_clock().now())
        self.publish(p, msg.header.stamp)

    def on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(msg.pose.pose.orientation))
        if self.anchor is None:
            return
        (mx, my, myaw), (ax, ay, ayaw), t_anchor = self.anchor
        age = (self.get_clock().now() - t_anchor).nanoseconds * 1e-9
        if age > self.max_extrapolation and self.corr is None:
            # MCL の更新が来ていない。出力を止めると、経路追従が「止まっている」と思って走り続けて危ない。
            # なので出し続ける(オドメトリだけで進める)が、警告は出す
            self.get_logger().warn(f"no MCL update for {age:.1f} s: pose from odometry only", throttle_duration_sec=1.0)
        # MCL の位置を受け取ってから進んだ分(オドメトリ座標 → ロボット基準)
        ox, oy, oyaw = self.odom
        dx, dy = ox - ax, oy - ay
        c, s = math.cos(ayaw), math.sin(ayaw)
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        dyaw = oyaw - ayaw
        # それを MCL の位置に足す(ロボット基準 → 地図座標)
        c, s = math.cos(myaw), math.sin(myaw)
        self.publish((mx + c * lx - s * ly, my + s * lx + c * ly, myaw + dyaw), self.get_clock().now().to_msg())

    def on_corr_chair(self, msg):
        self.corr_chair = (msg.point.x, msg.point.y, self.get_clock().now())
        # 補正のほうが先に届いたとき(別のトピックなので順番は決まらない)
        if self.corr is not None and self.chair_for_corr is None and (self.get_clock().now() - self.corr[5]).nanoseconds * 1e-9 < 1.0:
            self.chair_for_corr, self.passed = (msg.point.x, msg.point.y), False

    def on_correction(self, msg):
        if self.last_raw is None:
            return
        self.chair_for_corr, self.passed = None, False
        if self.corr_chair is not None and (self.get_clock().now() - self.corr_chair[2]).nanoseconds * 1e-9 < 1.0:
            self.chair_for_corr = self.corr_chair[:2]
        tx, ty, tyaw = msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation)
        bx, by, byaw = self.last_raw
        rot = math.atan2(math.sin(tyaw - byaw), math.cos(tyaw - byaw))
        c, s = math.cos(rot), math.sin(rot)
        self.corr = (rot, tx - (c * bx - s * by), ty - (s * bx + c * by), tx, ty, self.get_clock().now())
        self.get_logger().info(f"correction applied: ({(tx - bx) * 100:+.1f}, {(ty - by) * 100:+.1f}) cm, {math.degrees(rot):+.1f} deg")

    def publish(self, p, stamp):
        self.last_raw = p
        if self.corr is not None:
            rot, cx, cy, ax, ay, t0 = self.corr
            c, s = math.cos(rot), math.sin(rot)
            q = (c * p[0] - s * p[1] + cx, s * p[0] + c * p[1] + cy, p[2] + rot)
            f = 1.0  # 補正をどれだけ効かせるか(1 = 全部、0 = なし)
            if self.chair_for_corr is not None:
                d = math.hypot(q[0] - self.chair_for_corr[0], q[1] - self.chair_for_corr[1])
                if d < 0.25:
                    self.passed = True
                if self.passed and d > self.release_r:
                    f = max(0.0, 1.0 - (d - self.release_r) / self.fade)
            elif math.hypot(q[0] - ax, q[1] - ay) > self.corr_dist:
                f = 0.0
            if (self.get_clock().now() - t0).nanoseconds * 1e-9 > self.corr_time:
                f = 0.0
            if f <= 0.0:
                self.corr = None
                self.get_logger().info("correction released")
            else:
                p = (p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]), p[2] + f * rot)
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.frame
        out.pose.position.x, out.pose.position.y = p[0], p[1]
        out.pose.orientation.z, out.pose.orientation.w = math.sin(p[2] / 2.0), math.cos(p[2] / 2.0)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PoseExtrapolator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
