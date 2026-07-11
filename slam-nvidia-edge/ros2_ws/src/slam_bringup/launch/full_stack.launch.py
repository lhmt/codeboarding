"""Full local stack: frontend + VIO + LIO + fusion + map + health + recorder."""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG_DIR = os.environ.get("SLAM_CONFIG_DIR", os.path.join(os.getcwd(), "configs"))
LOG_DIR = os.environ.get("SLAM_LOG_DIR", os.path.join(os.getcwd(), "logs"))


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tensor_frontend_node",
                executable="tensor_frontend_node",
                name="tensor_frontend_node",
                output="screen",
                parameters=[os.path.join(CONFIG_DIR, "nvidia_runtime.yaml")],
            ),
            Node(
                package="vio_node",
                executable="vio_node",
                name="vio_node",
                output="screen",
                parameters=[os.path.join(CONFIG_DIR, "camera_imu.yaml")],
            ),
            Node(
                package="lio_node",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[os.path.join(CONFIG_DIR, "lidar_imu.yaml")],
            ),
            Node(
                package="fusion_node",
                executable="fusion_node",
                name="fusion_node",
                output="screen",
                parameters=[os.path.join(CONFIG_DIR, "fusion.yaml")],
            ),
            Node(
                package="map_node",
                executable="map_node",
                name="map_node",
                output="screen",
            ),
            Node(
                package="health_monitor_node",
                executable="health_monitor_node",
                name="health_monitor_node",
                output="screen",
                parameters=[os.path.join(CONFIG_DIR, "health.yaml")],
            ),
            Node(
                package="recorder_node",
                executable="recorder_node",
                name="recorder_node",
                output="screen",
                parameters=[{"output_dir": LOG_DIR}],
            ),
        ]
    )
