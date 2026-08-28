from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("qiling_kinematics"))
    ik_config = package_share / "config" / "differential_ik.yaml"
    demo_config = package_share / "config" / "pose_target_demo.yaml"
    return LaunchDescription([
        Node(
            package="qiling_kinematics",
            executable="differential_ik_node",
            name="qiling_differential_ik",
            output="screen",
            parameters=[str(ik_config)],
        ),
        Node(
            package="qiling_kinematics",
            executable="pose_target_demo",
            name="qiling_pose_target_demo",
            output="screen",
            parameters=[str(demo_config)],
        ),
    ])
