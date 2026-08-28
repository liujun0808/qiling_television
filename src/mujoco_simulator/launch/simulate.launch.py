from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import os
import yaml
from datetime import datetime

def generate_launch_description():

    # 获取yaml路径以读取urdf文件
    mujoco_pkg_path = get_package_share_directory('mujoco_simulator')
    yaml_path = mujoco_pkg_path + "/config/simulate.yaml"
    with open(yaml_path, "r", encoding="utf-8") as file:
        yaml_config = yaml.safe_load(file)
    # MuJoCo 使用 new_scene/scene_S4_40DOF_fullbody.xml；
    # robot_state_publisher 使用与完整 MuJoCo 关节状态对应的 fullbody URDF。
    robot_pkg_path = get_package_share_directory('qi_robot_description')
    model_path = yaml_config["mujoco_simulator"]["modelPath"]
    expected_model_path = os.path.join(
        robot_pkg_path, "new_scene", "scene_S4_40DOF_fullbody.xml")
    urdf_path = os.path.join(
        robot_pkg_path, "urdf", "s4_40DOF_fullbody.urdf")

    if model_path != "package://qi_robot_description/new_scene/scene_S4_40DOF_fullbody.xml":
        raise ValueError(
            "simulate.yaml 的 modelPath 必须指向 "
            "package://qi_robot_description/new_scene/scene_S4_40DOF_fullbody.xml")

    if not os.path.exists(expected_model_path):
        raise FileNotFoundError(f"MuJoCo模型文件未找到: {expected_model_path}")

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF文件未找到: {urdf_path}")

    # Foxglove（可选）：未安装 foxglove_bridge 时跳过，避免启动失败
    foxglove_launch = None
    try:
        foxglove_pkg_path = get_package_share_directory('foxglove_bridge')
        xml_launch_path = PathJoinSubstitution([
            foxglove_pkg_path, 'launch', 'foxglove_bridge_launch.xml'
        ])
        foxglove_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(xml_launch_path),
            launch_arguments={
                "output": "log"
            }.items()
        )
    except PackageNotFoundError:
        pass

    # 获取当前时间，用于确定bags的保存路径
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_folder_path = "bags/" + f"{timestamp}"

    actions = [
        Node(
            package='mujoco_simulator',  # 你的包名
            executable='simulate',  # 你的可执行文件名
            name='mujoco_simulator_node',  # 节点名称
            output='both',
            emulate_tty=True,
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
    ]
    if foxglove_launch is not None:
        actions.append(foxglove_launch)

    return LaunchDescription(actions)
