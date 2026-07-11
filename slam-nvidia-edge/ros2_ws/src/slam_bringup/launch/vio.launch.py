"""VIO only: camera + IMU odometry."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG_DIR = os.environ.get("SLAM_CONFIG_DIR", os.path.join(os.getcwd(), "configs"))


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vio_node",
                executable="vio_node",
                name="vio_node",
                output="screen",
                parameters=[os.path.join(CONFIG_DIR, "camera_imu.yaml")],
            ),
        ]
    )
