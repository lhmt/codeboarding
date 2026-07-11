"""LIO only: LiDAR + IMU odometry."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG_DIR = os.environ.get("SLAM_CONFIG_DIR", os.path.join(os.getcwd(), "configs"))


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lio_node",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[os.path.join(CONFIG_DIR, "lidar_imu.yaml")],
            ),
        ]
    )
