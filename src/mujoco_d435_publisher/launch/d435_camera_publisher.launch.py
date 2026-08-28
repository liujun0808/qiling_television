from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_d435_publisher")
    default_config = os.path.join(pkg_share, "config", "d435_camera_publisher.yaml")

    config_file = LaunchConfiguration("config_file")
    publish_camera_extrinsic_tf = LaunchConfiguration("publish_camera_extrinsic_tf")
    camera_base_frame = LaunchConfiguration("camera_base_frame")
    camera_frame = LaunchConfiguration("camera_frame")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="D435 camera publisher parameter yaml file.",
        ),
        DeclareLaunchArgument(
            "publish_camera_extrinsic_tf",
            default_value="true",
            description="Publish the fixed base_link -> D435 camera extrinsic TF.",
        ),
        DeclareLaunchArgument(
            "camera_base_frame",
            default_value="base_link",
            description="Parent frame for the D435 camera extrinsic TF.",
        ),
        DeclareLaunchArgument(
            "camera_frame",
            default_value="d435_camera",
            description="Child camera frame for the D435 camera extrinsic TF.",
        ),
        Node(
            package="mujoco_d435_publisher",
            executable="d435_camera_publisher",
            name="d435_camera_publisher",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="d435_camera_extrinsic_tf_publisher",
            output="screen",
            arguments=[
                "--x",
                "0.0240925853",
                "--y",
                "0.0044939526",
                "--z",
                "0.6924775167",
                "--qx",
                "-0.6533206256",
                "--qy",
                "0.6617666165",
                "--qz",
                "-0.2427124326",
                "--qw",
                "0.2762748278",
                "--frame-id",
                camera_base_frame,
                "--child-frame-id",
                camera_frame,
            ],
            condition=IfCondition(publish_camera_extrinsic_tf),
        ),
    ])
