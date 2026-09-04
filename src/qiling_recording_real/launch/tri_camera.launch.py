"""Launch the three recording RealSense RGB streams without the episode recorder.

The profile and serial numbers are read from real_recording.yaml so capture,
rollout and recording always use the same physical camera mapping.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _camera_nodes(context):
    config_path = Path(LaunchConfiguration("config_file").perform(context)).expanduser()
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    cameras = config.get("cameras", {})
    if set(cameras) != {"head", "left", "right"}:
        raise RuntimeError("real_recording.yaml must define exactly head, left and right cameras")

    nodes = []
    requested_profiles = []
    for camera_key in ("head", "left", "right"):
        camera = cameras[camera_key]
        width = int(camera.get("source_width", camera.get("width", 640)))
        height = int(camera.get("source_height", camera.get("height", 480)))
        fps = int(camera.get("source_fps", camera.get("fps", 30)))
        output_profile = (
            int(camera.get("width", width)),
            int(camera.get("height", height)),
            int(camera.get("fps", fps)),
        )
        if (width, height, fps) != output_profile:
            raise RuntimeError(
                f"{camera_key} camera source profile {width}x{height}@{fps}Hz does not "
                f"match recording profile {output_profile[0]}x{output_profile[1]}@{output_profile[2]}Hz")
        profile = f"{width}x{height}x{fps}"
        requested_profiles.append(
            f"{camera['camera_name']}({camera['serial']})={width}x{height}@{fps}Hz")
        nodes.append(Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace=camera["camera_namespace"],
            name=camera["camera_name"],
            output="screen",
            parameters=[{
                "camera_name": camera["camera_name"],
                "camera_namespace": camera["camera_namespace"],
                "serial_no": str(camera["serial"]),
                "enable_color": True,
                "enable_depth": False,
                "enable_infra": False,
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_gyro": False,
                "enable_accel": False,
                "enable_motion": False,
                "enable_rgbd": False,
                # D435i uses rgb_camera; D405 uses depth_module.color_profile.
                # Supplying both keeps the three-camera mapping in one config.
                "rgb_camera.color_profile": profile,
                "rgb_camera.color_format": str(camera.get("color_format", "RGB8")),
                "depth_module.color_profile": profile,
                "depth_module.color_format": str(camera.get("color_format", "RGB8")),
                "pointcloud.enable": False,
                "align_depth.enable": False,
                "publish_tf": False,
            }],
        ))
    return [
        LogInfo(
            msg="Tri-camera profile requests: " + ", ".join(requested_profiles)
            + "; native RGB only (no resize)",
        ),
        *nodes,
    ]


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
            description="Camera serial/profile configuration; source and recording profiles must match.",
        ),
        OpaqueFunction(function=_camera_nodes),
    ])
