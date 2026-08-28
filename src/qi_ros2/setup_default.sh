#!/bin/bash
echo "Setup qi ros2 environment with default interface"
source /opt/ros/humble/setup.bash
source $HOME/Work/qi_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
