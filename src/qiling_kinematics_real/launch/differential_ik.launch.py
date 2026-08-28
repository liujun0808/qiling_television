from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("qiling_kinematics")) / "config" / "differential_ik.yaml"
    return LaunchDescription([
        Node(
            package="qiling_kinematics",
            executable="differential_ik_node",
            name="qiling_differential_ik",
            output="screen",
            parameters=[str(config)],
        )
    ])
