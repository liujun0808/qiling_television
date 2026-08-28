# mujoco_simulator

## 1. 介绍

本仓库基于mujoco3.3.0的simulate示例程序进行二次开发,在原程序的基础上添加了ROS2通信接口与模型信息输出功能。可以在不改动代码的基础上支持各种MJCF模型


## 2. 安装教程

安装ROS2,推荐使用鱼香ROS一键安装脚本

```shell
wget http://fishros.com/install -O fishros && . fishros
```

安装依赖

```shell
sudo apt-get install libglfw3-dev gobjc++
```

新建ros2工作空间

```shell
cd ~/project # 换成您自己的路径
mkdir -p mujoco_simulator/src
cd mujoco_simulator/src
```

克隆本仓库和相关仓库到ros2工作空间中

```shell
# mujoco仿真库本体
git clone https://gitee.com/coralab/mujoco_simulator.git
# 实验室机器人描述文件库
git clone https://gitee.com/coralab/robot_description_coral.git 
# 实验室ROS2自定义通信接口库
git clone https://gitee.com/coralab/robot_interface_coral.git
```

开始编译

```shell
cd ..
colcon build
```

如果编译成功就说明安装没有问题

## 3. 使用说明



### 3.1 启动mujoco仿真

仓库的主要超参数保存在`config/simulate.yaml`中,首次执行需要确定其中参数是否正确,其中的MJCF文件路径是绝对路径,需要改成正确的路径

```yaml
mujoco_simulator_node:
  ros__parameters:

    # mjcf文件的路径 !需要改成您的路径!
    modelPath: "/home/coral-jyz/project/coral_rl_deploy/src/robot_description_coral/ShenNong/mjcf/scene_ShenNong.xml"

    # 电机状态与命令topic
    lowStateTopic: "human_low_state" # 共享内存硬件信息topic
    jointCommandsTopic: "human_joint_command" # 共享内存电机命令topic

    # 加载模型后是否暂停
    unPauseService: "/unpause_mujoco" # 启动服务名称
    initPauseFlag: true # 1是会暂停,0是不会暂停

    # 是否输出模型信息表格
    modelTableFlag: true # 1是会,0是不会

```
每次进行修改都需要再次进行编译

```shell
colcon build
```

确认参数无误后使用下面的命令启动仿真

```shell
source install/setup.bash
ros2 launch mujoco_simulator simulate.launch.py 
```

成功启动后的界面如下图所示

![](https://picgo-nanhaibei.oss-cn-beijing.aliyuncs.com/20250403180035.png)

默认启动mujoco后仿真处于暂停状态,此时可以按空格键进行仿真,也可以在调用ROS2服务启动仿真

在C++中使用服务启动仿真的示例代码如下

```C++
// 包含需要的头文件
#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/empty.hpp>

// 声明服务客户端
rclcpp::Client<std_srvs::srv::Empty>::SharedPtr mujocoUnpauseClient;
.mujocoUnpauseClient = this->create_client<std_srvs::srv::Empty>(.mujocoUnpauseName);

// 调用服务启动mujoco仿真
auto unpause = std::make_shared<std_srvs::srv::Empty::Request>();
.mujocoUnpauseClient->async_send_request(unpause);
```


### 3.2 MJCF文件的导出

#### 3.2.1 从URDF导出MJCF

以实验室的神农机器人为例,假设现在已经拿到从solidworks中导出的urdf文件,首先需要在`robot`标签下添加mesh文件路径

```xml
<robot
  name="ShenNong">
  <mujoco>
    <compiler
    	meshdir="../meshes/"
    	balanceinertia="true"
    	discardvisual="false" />
  </mujoco>
  <link
    name="base_link">
  <!-- 下面的link和joint省略 -->
```

然后打开mujoco程序,无论是mujoco原始仓库的`simulate`程序还是本仓库的launch都可以,这里以本仓库为例

首先用本仓库的launch文件启动mujoco

```shell
ros2 launch mujoco_simulator simulate.launch.py 
```
然后在文件夹中把urdf文件拖到mujoco界面中,成功加载后的界面如下图所示

![](https://picgo-nanhaibei.oss-cn-beijing.aliyuncs.com/20250404155834.png)

如果加载URDF时出现下面的报错,一般是某个mesh面数超过200000,mujoco的compiler不支持,需要手动减少mesh面数后再尝试导入。减少面数的方法见[这篇博文]((https://blog.csdn.net/qq_37389133/article/details/125050981))

```
Error: number of faces should be between 1 and 200000 in STL file '/home/coral-jyz/project/robot-urdf/ShenNong/urdf/../meshes/base_link.STL'; perhaps this is an ASCII file?
```

此时按下空格键启动仿真,可能会发现机器人关节乱动

![](https://picgo-nanhaibei.oss-cn-beijing.aliyuncs.com/20250404160016.png)

这一般是由于机器人的碰撞体积发生干涉,我们需要简化urdf中的碰撞体积。

下面给出一个修改例子,原始urdf中link标签的内容如下,我们需要用简单几何体替换mesh作为碰撞体积

```xml
  <link
    name="LL1_link">
    <inertial>
    <!-- ... -->
    </inertial>
    <visual>
      <!-- ... -->
    </visual>
    <!-- 👇这里是碰撞体积标签👇 -->
    <collision> 
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://robot_urdf/ShenNong/meshes/LL1_link.STL" />
      </geometry>
    </collision>
    <!-- 👆这里是碰撞体积标签👆 -->
  </link>
```

作如下修改

```xml
  <link
    name="LL1_link">
    <inertial>
    <!-- ... -->
    </inertial>
    <visual>
      <!-- ... -->
    </visual>
    <collision>
        <origin
            xyz="-0.0028 3.5e-05 -0.002062"
            rpy="0 0 0"/>
        <geometry>
            <!-- 使用圆柱替代原来的碰撞体积 -->
            <cylinder radius="0.06" length="0.05"/>
        </geometry>
    </collision>
  </link>
```

碰撞体积的简化方式视任务而定。在行走任务中,可以使用长方体作为base_link和脚掌的碰撞体积,其他所有link的碰撞体积都可以删除

tips:在mujoco中按下数字`0`和`1`分别是打开/隐藏碰撞体积与可视化mesh

#### 3.2.2 MJCF文件的完善

**添加base_link**

当机器人物理仿真正常后,可以点击mujoco界面左上角的`Save xml`导出MJCF文件了,文件会生成在打开mujoco的终端所在的路径下。需要注意,mujoco默认将base_link与第二个link合并,为了保持正确的link树,需要手动创建base_link

以神农机器人为例,导出的MJCF的body部分如下,第一个body是`LL1_link`而不是`base_link`

```xml
 <worldbody>
    <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.752941 0.752941 0.752941 1" mesh="base_link_sim"/>
    <geom type="mesh" rgba="0.752941 0.752941 0.752941 1" mesh="base_link_sim"/>
    <geom size="0.0075 0.0075 0.002" pos="0 0 -0.004" quat="0.999998 0 0 -0.002" type="box" contype="0" conaffinity="0" group="1" density="0"/>
    <body name="LL1_link" pos="0.04142 0.119977 -0.266431">
      <inertial pos="-0.0028 3.5e-05 -0.002062" quat="-0.0066869 0.703683 0.0022928 0.710479" mass="1.905" diaginertia="0.00597639 0.00442126 0.00244036"/>
      <joint name="l_leg_hpx" pos="0 0 0" axis="1 0 0" range="-0.2 0.4" actuatorfrcrange="-160 160"/>
      <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="0.752941 0.752941 0.752941 1" mesh="LL1_link"/>
      <geom type="mesh" rgba="0.752941 0.752941 0.752941 1" mesh="LL1_link"/>
      <body name="LL2_link" pos="0 0 -0.0473">
      <!-- ... -->
<worldbody/>
```

需要作如下修改

```xml
<worldbody>
    <body name="base_link" pos="0 0 1.2">
      <inertial pos="0 0 0" quat="1 0 0 0" mass="12.42979" diaginertia="0.61166 0.544877 0.139043" />
      <joint name="float_base_joint" type="free" limited="false" actuatorfrclimited="false"/>
      <geom type="mesh" contype="0" conaffinity="0" group="1" density="0" rgba="1 1 1 1"
        mesh="base_link_sim" />
      <geom size="0.0075 0.0075 0.002" pos="0 0 -0.004" quat="0.999998 0 0 -0.002" type="box"
        contype="0" conaffinity="0" group="1" density="0" />
        <!-- 记得在这里添加site,后面要用 -->
      <site name="imu" size="0.01" pos="0 0 0" />
      <body name="LL1_link" pos="0.04142 0.119977 -0.266431">
        <inertial pos="-0.0028 3.5e-05 -0.002062" quat="-0.0066869 0.703683 0.0022928 0.710479"
          mass="1.905" diaginertia="0.00597639 0.00442126 0.00244036" />
        <joint name="l_leg_hpx" pos="0 0 0" axis="1 0 0" range="-0.2 0.4"
          actuatorfrcrange="-160 160" frictionloss="0.2" damping="0.1"/>
        <geom type="mesh" contype="0" conaffinity="0" group="1" density="0"
          rgba="0.752941 0.752941 0.752941 1" mesh="LL1_link" />
        <geom size="0.06 0.025" pos="-0.0028 3.5e-05 -0.002062" type="cylinder"
          rgba="0.752941 0.752941 0.752941 1" />
        <body name="LL2_link" pos="0 0 -0.0473">
        <!-- ... -->
```

其中`base_link`的`pos`指机器人初始生成的位置,可自行给定,`inertial`标签下的`pos`,`quat`,`mass`可直接从URDF中读取,`diaginertia`有三个元素,分别是URDF中惯性矩阵的`ixx`,`iyy`,`izz`

**添加actuator与sensor**

在MJCF中添加`<actuator>`与`<sensor>`标签后,才能发送电机命令与读取传感器状态

以神农机器人为例,在`<mujoco>`标签下添加下面的内容

```xml
  <!-- 力控执行器 -->
  <actuator>
    <motor name="M_l_leg_hpx" joint="l_leg_hpx" ctrlrange="-160 160" />
    <motor name="M_l_leg_hpz" joint="l_leg_hpz" ctrlrange="-120 120" />
    <motor name="M_l_leg_hpy" joint="l_leg_hpy" ctrlrange="-160 160" />
    <!-- ... -->
  </actuator>

  <sensor>
    <!-- 电机位置传感器 -->
    <jointpos name="l_leg_hpx_pos" joint="l_leg_hpx" />
    <jointpos name="l_leg_hpz_pos" joint="l_leg_hpz" />
    <jointpos name="l_leg_hpy_pos" joint="l_leg_hpy" />
    <!-- ... -->

    <!-- 电机速度传感器 -->
    <jointvel name="l_leg_hpx_vel" joint="l_leg_hpx" />
    <jointvel name="l_leg_hpz_vel" joint="l_leg_hpz" />
    <jointvel name="l_leg_hpy_vel" joint="l_leg_hpy" />
    <!-- ... -->

    <!-- 电机力矩传钢琴 -->
    <jointactuatorfrc name="l_leg_hpx_torque" joint="l_leg_hpx" />
    <jointactuatorfrc name="l_leg_hpz_torque" joint="l_leg_hpz" />
    <jointactuatorfrc name="l_leg_hpy_torque" joint="l_leg_hpy" />
    <!-- ... -->

    <!-- 建立一个返回四元数的传感器 -->
    <framequat name="imu_quat" objtype="site" objname="imu" />
    <!-- 建立一个返回角速度的传感器 -->
    <gyro name="imu_gyro" site="imu" />
    <!-- 建立一个返回线加速度的传感器 -->
    <accelerometer name="imu_acc" site="imu" />
    <!-- 返回3D真实位置 -->
    <framepos name="frame_pos" objtype="site" objname="imu" />
    <!-- 返回3D真实线速度 -->
    <framelinvel name="frame_vel" objtype="site" objname="imu" />
  </sensor>
```

添加完成后,用mujoco打开该MJCF,在`Option`中开启`Sensor`,在右侧边栏中打开`Control`,如果看到如下图所示的画面,说明标签添加成功

![](https://picgo-nanhaibei.oss-cn-beijing.aliyuncs.com/20250405104500.png)

**给关节添加阻尼摩擦力**

默认的关节阻尼和摩擦都是0,可能会导致控制发散。在`<joint>`标签下添加摩擦`frictionloss`和阻尼`damping`可以解决该问题

```xml
<joint name="l_leg_hpx" pos="0 0 0" axis="1 0 0" range="-0.2 0.4"
       actuatorfrcrange="-160 160" frictionloss="0.2" damping="0.1"/>
```

**添加天空和地板**

上诉的MJCF只包含机器人本身,启动物理仿真后机器人会坠入无尽虚空,为了解决该问题,需要在MJCF中添加地板和天空

具体的添加方式再次不赘述,详见实验室的`robot_description_coral`仓库

完成了上述工作后,我们就得到了完善的MJCF文件,可以开始仿真了


