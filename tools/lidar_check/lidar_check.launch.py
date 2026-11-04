"""RPLIDAR A1 の点群をRVizで確認する。

使い方:
  source /opt/ros/jazzy/setup.bash
  ros2 launch ~/AIcontest_ws/tools/lidar_check/lidar_check.launch.py
  ros2 launch ~/AIcontest_ws/tools/lidar_check/lidar_check.launch.py serial_port:=/dev/ttyUSB1
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        Node(
            package="rplidar_ros",
            executable="rplidar_composition",  # このバージョンでは rplidar_node ではない
            name="rplidar_node",
            output="screen",
            parameters=[{
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": 115200,  # A1
                "frame_id": "lidar_front",
                "angle_compensate": True,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", os.path.join(HERE, "lidar_check.rviz")],
            output="log",
        ),
    ])
