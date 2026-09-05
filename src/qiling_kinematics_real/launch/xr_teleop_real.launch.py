from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("qiling_kinematics_real"))
    ik_config = package_share / "config" / "differential_ik.yaml"
    bridge_config = package_share / "config" / "xr_pose_clutch_bridge.yaml"
    pxrea_config = package_share / "config" / "xrobotoolkit_pxrea_adapter.yaml"
    pxrea_adapter_respawn = LaunchConfiguration("pxrea_adapter_respawn")

    return LaunchDescription([
        DeclareLaunchArgument(
            "pxrea_adapter_respawn",
            default_value="true",
            description=(
                "Restart the PXREA adapter after an unexpected exit. "
                "The combined teleop/recording launcher disables this so "
                "Ctrl-C cannot leave a delayed respawn behind."
            ),
        ),
        Node(
            package="qiling_kinematics_real",
            executable="differential_ik_real_node",
            name="qiling_differential_ik_real",
            output="screen",
            parameters=[str(ik_config)],
        ),
        Node(
            package="qiling_kinematics_real",
            executable="xr_pose_clutch_bridge_real",
            name="qiling_xr_pose_clutch_bridge_real",
            output="screen",
            parameters=[str(bridge_config)],
        ),
        Node(
            package="qiling_kinematics_real",
            executable="xrobotoolkit_pxrea_adapter_real",
            name="qiling_xrobotoolkit_pxrea_adapter_real",
            output="screen",
            respawn=pxrea_adapter_respawn,
            respawn_delay=1.0,
            parameters=[str(pxrea_config)],
        ),
        Node(
            package="qiling_kinematics_real",
            executable="o6_trigger_state_node",
            name="qiling_o6_trigger_state",
            output="screen",
            parameters=[str(package_share / "config" / "o6_trigger_state.yaml")],
        ),
        Node(
            package="qiling_kinematics_real",
            executable="o6_command_adapter_node",
            name="qiling_o6_command_adapter",
            output="screen",
            parameters=[str(package_share / "config" / "o6_command_adapter.yaml")],
        ),
    ])
