#!/usr/bin/env bash
set -Eeuo pipefail

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "错误：未找到 ROS 2 Humble。请先在 Ubuntu 22.04 上安装 ROS 2 Humble。" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  adb \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libglfw3-dev \
  libsimde-dev \
  libyaml-cpp-dev \
  pkg-config \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-yaml \
  ros-humble-ament-cmake \
  ros-humble-ament-cmake-gtest \
  ros-humble-kdl-parser \
  ros-humble-pinocchio \
  ros-humble-proxsuite \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-robot-state-publisher \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-generator-dds-idl \
  ros-humble-tf2-ros \
  ros-humble-urdf

echo "ROS 2、Pinocchio、ProxSuite、MuJoCo GUI 编译依赖和 ADB 已安装。"
echo "下一步：安装 XRoboToolkit PC Service，再运行 src/scripts/build_xrtele.sh。"
