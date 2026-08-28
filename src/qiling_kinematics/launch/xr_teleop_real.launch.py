from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("qiling_kinematics"))
    ik_config = package_share / "config" / "differential_ik.yaml"
    bridge_config = package_share / "config" / "xr_pose_clutch_bridge.yaml"
    pxrea_config = package_share / "config" / "xrobotoolkit_pxrea_adapter.yaml"

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
            executable="xr_pose_clutch_bridge",
            name="qiling_xr_pose_clutch_bridge",
            output="screen",
            parameters=[str(bridge_config)],
        ),
        Node(
            package="qiling_kinematics",
            executable="xrobotoolkit_pxrea_adapter",
            name="qiling_xrobotoolkit_pxrea_adapter",
            output="screen",
            parameters=[str(pxrea_config)],
        ),
    ])
