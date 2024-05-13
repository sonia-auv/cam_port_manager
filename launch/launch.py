import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    auv = os.getenv("AUV", "")

    config = os.path.join(
        get_package_share_directory("cam_port_manager"), "config", f"{auv}_config.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="cam_port_manager",
                executable="cam_port_manager",
                parameters=[config]
            )
        ]
    )
