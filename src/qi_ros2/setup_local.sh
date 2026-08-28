#!/bin/bash
echo "Setup qi ros2 simulation environment"
source /opt/ros/humble/setup.bash
source $HOME/Work/qi_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp47s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
