"""Health aggregation only — attach to an already-running stack."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG_DIR = os.environ.get("SLAM_CONFIG_DIR", os.path.join(os.getcwd(), "configs"))


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="health_monitor_node",
                executable="health_monitor_node",
                name="health_monitor_node",
                output="screen",
                parameters=[os.path.join(CONFIG_DIR, "health.yaml")],
            ),
        ]
    )
