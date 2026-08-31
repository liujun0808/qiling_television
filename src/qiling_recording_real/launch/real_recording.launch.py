from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    package_share = Path(get_package_share_directory("qiling_recording_real"))
    config_path = Path(LaunchConfiguration("config_file").perform(context)).expanduser()
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    stream_config = config["stream"]
    cameras = config["cameras"]
    nodes = []

    for camera in cameras.values():
        # D405 uses depth_module.color_profile while D435i uses
        # rgb_camera.color_profile. Supplying both keeps one YAML format for
        # all three devices; unsupported inactive profiles are ignored by the
        # driver because depth is disabled.
        parameters = {
            "serial_no": camera["serial"],
            "enable_color": True,
            "enable_depth": False,
            "enable_infra": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,
            "enable_accel": False,
            "enable_motion": False,
            "enable_rgbd": False,
            "rgb_camera.color_profile": (
                f'{stream_config["width"]}x{stream_config["height"]}x{stream_config["fps"]}'
            ),
            "rgb_camera.color_format": stream_config["color_format"],
            "depth_module.color_profile": (
                f'{stream_config["width"]}x{stream_config["height"]}x{stream_config["fps"]}'
            ),
            "depth_module.color_format": stream_config["color_format"],
            "pointcloud.enable": False,
            "align_depth.enable": False,
            "publish_tf": False,
        }
        nodes.append(
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace=camera["camera_namespace"],
                name=camera["camera_name"],
                output="screen",
                parameters=[parameters],
            )
        )

    nodes.append(
        Node(
            package="qiling_recording_real",
            executable="episode_recorder",
            name="qiling_episode_recorder",
            output="screen",
            parameters=[{"config_file": str(config_path)}],
        )
    )
    return nodes


def generate_launch_description():
    default_config = (
        Path(get_package_share_directory("qiling_recording_real"))
        / "config"
        / "real_recording.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=str(default_config),
            description="YAML configuration for cameras and episode recording",
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
