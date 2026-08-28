Qi robot ROS2 support

# Introduction
Qi SDK implements an easy-to-use robot communication mechanism based on Cyclonedds, which enable developers to achieve robot communication and control (**Supports Qi Shennong Humanoid Robot**).

DDS is used in ROS2 as a communication mechanism. Therefore, the underlying layers of Qi Robots can be compatible with ROS2. ROS2 msg can be directly used for communication and control of Qi Shennong Humanoid Robot without wrapping the SDK interface.

# Configuration
## System requirements
Tested systems and ROS2 distro
|systems|ROS2 distro|
|Ubuntu 22.04|humble|

Taking ROS2 humble as an example, if you need another version of ROS2, replace "humble" with the current ROS2 version name in the corresponding place:

The installation of ROS2 humble can refer to: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

ctrl+alt+T open the terminal, clone the repository: https://github.com/qi-robotics/qi_ros2

```bash
mkdir Work
cd ~/Work
git clone https://github.com/qi-robotics/qi_ros2
```
where:
- **cyclonedds_ws**: The workspace of Qi ros2 package. The msg for Qi robot are supplied in the subfolder cyclonedds_ws/qi/qi.
- **example**: The workspace of some examples.


## Install Qi ROS2 package

### 1. Dependencies
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
```

### 2. Compile cyclone dds
The cyclonedds version of Qi robot is 0.10.5. To communicate with Qi robots using ROS2, it is necessary to change the dds implementation. See：https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html

Before compiling cyclonedds, please ensure that ros2 environment has **NOT** been sourced when starting the terminal. Otherwise, it may cause errors in compilation.

If "source/opt/ros/humble/setup. bash" has been added to the ~/.bashrc file when installing ROS2, it needs to be commented out:

```bash
sudo apt install gedit
sudo gedit ~/.bashrc
``` 
```bash
# source /opt/ros/humble/setup.bash 
```


Compile cyclone-dds
```bash
cd ~/Work/qi_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds #Compile cyclone-dds package
```

### 3. Compile qi package
After compiling cyclone-dds, ROS2 dependencies is required for compilation of the qi package. Therefore, before compiling, it is necessary to source the environment of ROS2.
```bash
source /opt/ros/humble/setup.bash # source ROS2 environment
colcon build # Compile all packages in the workspace
```

## Connect to Qi robot

### 1. Network configuration
TODO: add network configuration content here


### 2. Connect to Simulation
If your computer is not connected to the robot but you still want to use Qi ROS2 for simulation and other functions, you can use the local loopback "lo" as the network interface.
```bash
source ~/Work/qi_ros2/setup_local.sh # use "lo" as the network interface
```
or
```bash
source ~/Work/qi_ros2/setup_default.sh # No network network interface specified 
```

### 2. Connect and test
After completing the above configuration, it is recommended to restart the computer before conducting the test.

Ensure that the network of robot is connected correctly, open a terminal and input:  
```bash
source ~/Work/qi_ros2/setup.sh
ros2 topic list
```
TODO: add topic list info


### 3. Examples
Open a terminal and input:
```bash
source ~/Work/qi_ros2/setup.sh
cd ~/Work/qi_ros2/example
colcon build
```
After compilation, run in the terminal:
```bash
source install/setup.bash
ros2 run qi_ros2_example write_low_state 
```
You can see a low state writer simulate writing low state information to the topic /lowstate from the terminal:
```bash
[INFO] [1750822989.289125378] [low_state_writer]: Low state writer started, publishing at 100Hz...
[INFO] [1750822990.289375401] [low_state_writer]: Published LowState - Time: 1.00s, IMU Roll: -0.000, Motor[0] Pos: 0.476
[INFO] [1750822991.289467564] [low_state_writer]: Published LowState - Time: 2.00s, IMU Roll: 0.000, Motor[0] Pos: -0.294
[INFO] [1750822992.309418842] [low_state_writer]: Published LowState - Time: 3.01s, IMU Roll: -0.003, Motor[0] Pos: -0.286
[INFO] [1750822993.309586466] [low_state_writer]: Published LowState - Time: 4.01s, IMU Roll: 0.003, Motor[0] Pos: 0.478
[INFO] [1750822994.329627606] [low_state_writer]: Published LowState - Time: 5.01s, IMU Roll: -0.003, Motor[0] Pos: -0.009
[INFO] [1750822995.330035400] [low_state_writer]: Published LowState - Time: 6.01s, IMU Roll: 0.003, Motor[0] Pos: -0.473
[INFO] [1750822996.339475881] [low_state_writer]: Published LowState - Time: 7.01s, IMU Roll: -0.003, Motor[0] Pos: 0.301
[INFO] [1750822997.339549653] [low_state_writer]: Published LowState - Time: 8.01s, IMU Roll: 0.003, Motor[0] Pos: 0.286
```
Then run in another terminal:
```bash
source ~/Work/qi_ros2/setup.sh
cd ~/Work/qi_ros2/example
source install/setup.bash
ros2 run qi_ros2_example read_low_state 
```
You can see a low state reader that reading robot state from /lowstate topic.
```bash
[INFO] [1750823400.264865158] [low_state_reader]: ---
[INFO] [1750823400.275733154] [low_state_reader]: Machine Mode: 1
[INFO] [1750823400.276090879] [low_state_reader]: IMU - Roll: 0.051, Pitch: -0.043, Yaw: 0.031
[INFO] [1750823400.276202724] [low_state_reader]: IMU - Gyro: [-0.086, 0.025, 0.003], Accel: [0.255, -0.258, 9.912]
[INFO] [1750823400.276434662] [low_state_reader]: Motor[0] - Mode: 1, Pos: 0.152, Vel: 0.898, Torque: 0.606, Temp: 45°C
[INFO] [1750823400.276550649] [low_state_reader]: Motor[1] - Mode: 1, Pos: 0.277, Vel: -0.915, Torque: 0.938, Temp: 45°C
[INFO] [1750823400.276638753] [low_state_reader]: Motor[2] - Mode: 1, Pos: -0.498, Vel: 0.109, Torque: -1.918, Temp: 50°C
[INFO] [1750823400.276720569] [low_state_reader]: Motor[3] - Mode: 1, Pos: 0.345, Vel: 1.023, Torque: 1.746, Temp: 54°C
[INFO] [1750823400.276798145] [low_state_reader]: CRC: 0x01C98DE6
```
You can also try to play with the low_level_ctrl node to see how to send control command to the robot.