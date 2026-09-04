# qiling_television 项目交接文档

更新时间：2026-08-28

## 1. 项目目标

本项目目标是使用 Meta Quest 3 和 XRoboToolkit 遥操自研人形双臂机器人，并在后续阶段记录三路相机、双臂状态和末端位姿，最终转换为 LeRobot 数据集。

当前已确认的机器人约定：

- 双臂各 7 个关节；
- O6 灵巧手独立于双臂 IK 控制；
- 机器人身体 SDK 接口为 26 维；
- ROS2 Humble；
- 手臂底层为 MIT 模式；
- 手臂控制接口最高 50 Hz；
- 固定 base_link，腿部不参与 IK 控制；
- 当前使用的 Pinocchio 模型是 src/qi_robot_description/urdf/s4_dual_arm.urdf；
- 暂时不实现躯干避障；数据录制已独立实现第一版，后续继续做真机验证和 LeRobot
  数据质量检查。

## 2. 当前代码包

~~~text
src/
├── qi_robot_description/
│   └── urdf/s4_dual_arm.urdf
├── qiling_kinematics/       # 已验证的 MuJoCo 仿真包
├── qiling_kinematics_real/  # 真机专用包，当前主要实现内容
├── common_msgs/mit_msgs/    # MITJointCommands、MITLowState
├── topic_convertor/         # MIT ROS2 命令与 SDK DDS 接口转换
├── qi_ros2/                 # Qi DDS ROS2 消息包（LowCmd/LowState）
├── mujoco_simulator/
└── mujoco_d435_publisher/
~~~

真机包不启动 MuJoCo，也不应依赖 MuJoCo 运行时。

## 3. qiling_kinematics_real 已完成内容

真机包位置：

~~~text
src/qiling_kinematics_real/
~~~

### 3.1 IK 核心

从仿真包复制并独立出来，保留当前已验证的：

- Pinocchio 双臂运动学；
- ProxSuite/ProxQP 差分 IK；
- 位置优先、姿态次优先的层级 IK；
- 肘部拟人化零空间调节；
- 关节限位和速度限制；
- 奇异位形速度缩放；
- 工作空间保护；
- 目标滤波、死区和加速度限制；
- Pinocchio 重力补偿；
- MIT 位置、速度、kp、kd、eff 输出。

### 3.2 真机状态输入

真机 IK 直接订阅：

~~~text
/human_lower_state
mit_msgs/msg/MITLowState
~~~

消息中的 joint_states.position 固定为 26 维，按以下索引读取：

~~~text
0..11   腿部，忽略
12..18  左臂 7 个关节
19..25  右臂 7 个关节
~~~

当前实现要求：

- position 数组长度严格为 26；
- 所有位置值必须是有限值；
- 左右臂状态必须同时收到后才开始输出命令；
- 不再依赖 /joint_states 作为 IK 输入。

### 3.3 真机命令输出

真机 IK 发布：

~~~text
/human_lower_command
mit_msgs/msg/MITJointCommands
~~~

命令数组严格为 26 维：

~~~text
0..11   腿部命令全部为零
12..18  左臂命令
19..25  右臂命令
~~~

双臂命令字段：

~~~text
pos：IK 积分得到的关节参考位置
vel：当前控制参考速度
kp：默认 40
kd：默认 2
eff：Pinocchio 重力补偿前馈力矩
~~~

根据用户确认，底层 SDK 只使用后面的手臂数据控制手臂，因此当前不修改腿部零命令逻辑。

### 3.4 真实腕部 FK

真实腕部位姿已经集成到真机 IK 节点内。IK 根据真实关节状态计算 Pinocchio FK，并发布：

~~~text
/teleop/left_wrist_state
/teleop/right_wrist_state
geometry_msgs/msg/PoseStamped
~~~

这两个话题供 Grip 离合器建立相对位姿锚点使用，不需要单独的 FK 节点。

### 3.5 Quest 输入适配

xrobotoolkit_pxrea_adapter_real 使用：

~~~text
/opt/apps/roboticsservice/SDK/include/PXREARobotSDK.h
/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so
~~~

输出：

~~~text
/xr/left_controller_pose
/xr/right_controller_pose
geometry_msgs/msg/PoseStamped

/xr/controller_joy
sensor_msgs/msg/Joy
~~~

当前 Joy 映射：

~~~text
axes[0]：左 Trigger
axes[1]：右 Trigger
axes[2]：左 Grip
axes[3]：右 Grip

buttons[0]：左 X
buttons[1]：右 A
buttons[4]：左 Grip 数字状态
buttons[5]：右 Grip 数字状态
~~~

### 3.6 O6 当前状态层（不是实际电机控制）

已新增 o6_trigger_state_node。

输入：

~~~text
/xr/controller_joy
~~~

输出：

~~~text
/teleop/o6_trigger_state
std_msgs/msg/UInt8
~~~

位定义：

~~~text
bit 0：左 O6 闭合
bit 1：右 O6 闭合
~~~

当前代码控制规则：

~~~text
左 Trigger 按住：左 O6 闭合
左 Trigger 松开：左 O6 张开

右 Trigger 按住：右 O6 闭合
右 Trigger 松开：右 O6 张开
~~~

当前已加入：

- 闭合阈值 0.75；
- 释放阈值 0.25；
- 去抖时间 0.08 秒；
- Joy 超时自动释放。

注意：当前只输出内部 bitmask 状态，还没有转换成真实 O6 SDK 命令，因为 O6 统一命令话题和消息类型尚未提供。

因此，当前真机包尚未实现 O6 灵巧手的实际闭合/张开控制。`topic_convertor` 也明确
只转换 26 个身体电机，不包含 O6。当前节点仍读取 Trigger；用户最终要求的
“按住左右手柄前方的鼠标状按钮闭合，松开按钮张开”尚未落地，后续需要修改输入
映射并增加 O6 实际命令适配器。

## 4. 真机启动文件

文件：

~~~text
src/qiling_kinematics_real/launch/xr_teleop_real.launch.py
~~~

启动命令：

~~~bash
cd /home/ub/project/qiling_television
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qiling_kinematics_real xr_teleop_real.launch.py
~~~

该 launch 启动：

~~~text
xrobotoolkit_pxrea_adapter_real
xr_pose_clutch_bridge_real
differential_ik_real_node
o6_trigger_state_node
~~~

## 5. XRoboToolkit PC Service

真机 PC 已安装并确认存在：

~~~text
/opt/apps/roboticsservice/runService.sh
/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so
~~~

真机 PC 后台脚本：

~~~text
src/scripts/start_xrobotoolkit_real.sh
src/scripts/stop_xrobotoolkit_real.sh
~~~

启动：

~~~bash
bash src/scripts/start_xrobotoolkit_real.sh
~~~

停止：

~~~bash
bash src/scripts/stop_xrobotoolkit_real.sh
~~~

脚本行为：

- RoboticsServiceProcess 使用 Qt offscreen 模式；
- RobotLinuxDemo.x86_64 使用 batchmode 和 nographics；
- 不显示 GUI；
- 使用 PID 文件停止进程；
- 日志位于 XDG_RUNTIME_DIR/qiling_xrobotoolkit/，没有 XDG_RUNTIME_DIR 时位于 /tmp/qiling_xrobotoolkit/。

用户已经在真机 PC 上验证：

~~~text
RoboticsServiceProcess 启动成功
RobotLinuxDemo.x86_64 启动成功
两个进程可以正常停止
~~~

## 6. 完整真机控制链路

双臂控制最终链路：

~~~text
Quest 3
  ↓ 网络
XRoboToolkit PC Service
  ↓ PXREA SDK
xrobotoolkit_pxrea_adapter_real
  ↓
/xr/left_controller_pose
/xr/right_controller_pose
/xr/controller_joy
  ↓
xr_pose_clutch_bridge_real
  ↓
/teleop/left_wrist_target
/teleop/right_wrist_target
  ↓
differential_ik_real_node
  ↓
/human_lower_command，26 维 MITJointCommands
  ↓
topic_convertor
  ↓
DDS lowcmd
  ↓
机器人 SDK 和双臂电机
~~~

状态链路：

~~~text
机器人 DDS lowstate
  ↓
topic_convertor
  ↓
/human_lower_state
  ↓
differential_ik_real_node
~~~

重要说明：

qiling_kinematics_real 已经负责发布 26 维 /human_lower_command，但 topic_convertor 必须运行，命令才会继续进入 DDS lowcmd。当前真机 launch 尚未把 topic_convertor 自动启动起来。

### 6.1 Qi 消息包与 topic_convertor

Qi 消息包实际位置：

~~~text
src/qi_ros2/cyclonedds_ws/src/qi/qi
~~~

包名为 `qi`，提供 `qi/msg/LowCmd`、`qi/msg/LowState` 以及电机和 IMU 消息。
当前已经在本工作空间编译成功；`topic_convertor` 也已在 source `qi` 环境后编译成功。

转换节点负责：

~~~text
qi/LowState  lowstate  →  /human_lower_state   MITLowState
/human_lower_command   →  lowcmd              qi/LowCmd
~~~

转换节点默认关闭命令桥接，真机必须显式打开：

~~~bash
ros2 run topic_convertor topic_converter_node --ros-args \
  -p expected_motor_count:=26 \
  -p enable_state_bridge:=true \
  -p enable_command_bridge:=true \
  -p strict_command_size:=true
~~~

## 7. 已完成的验证

当前开发工作空间已经验证：

- qiling_kinematics_real 编译成功；
- 8 个核心 GTest 全部通过；
- Pinocchio 成功加载 s4_dual_arm.urdf；
- 模型 nq=14、nv=14；
- 真机 IK 节点可以启动；
- 真机 launch 文件可以被 ROS2 解析；
- XRoboToolkit 无 GUI 启动和停止脚本正常；
- 真机 IK 已改为 /human_lower_state 输入；
- 输出已改为 26 维命令；
- O6 状态节点已编译，但仅为内部 bitmask 状态层，尚无 O6 实际命令适配器；
- `qi` 消息包和 `topic_convertor` 均已编译成功。

由于当前开发环境的 ROS 日志目录权限限制，运行测试时可以临时使用：

~~~bash
export ROS_LOG_DIR=/tmp/qiling_television_ros_log
~~~

这不是项目代码问题。

## 8. 尚未完成的内容

### 8.1 topic_convertor 的真机验证与启动整合

topic_convertor 已在当前工作空间编译成功，但尚未在真实机器人网络上验证。需要确认：

~~~text
enable_state_bridge=true
enable_command_bridge=true
expected_motor_count=26
strict_command_size=true
~~~

它负责：

~~~text
/human_lower_command → DDS lowcmd
DDS lowstate → /human_lower_state
~~~

当前仍未将它加入 `qiling_kinematics_real` 的 launch，原因是它属于 DDS/身体接口层，
应在确认网络接口和 CycloneDDS 配置后再统一编排。后续需要：

1. 单独启动并验证已有 topic_convertor；
2. 确认真实 `lowstate`/`lowcmd` 话题和 DDS 网络接口；
3. 再将 topic_convertor 加入真机总启动流程；
4. 保留独立停止脚本和异常退出清理。

### 8.2 O6 真实 SDK 命令适配

需要用户提供：

- O6 统一状态话题名称；
- O6 状态消息类型；
- O6 统一命令话题名称；
- O6 命令消息类型；
- 左右 O6 的关节排列；
- 闭合/张开目标值；
- 是否需要位置、速度、力矩或模式字段。

Quest 前方按钮和录制按钮在 `/xr/controller_joy` 中的当前索引为左 X=`buttons[0]`、
右 A=`buttons[1]`、左 Y=`buttons[2]`、右 B=`buttons[3]`。
O6 仍使用 Trigger，最终输入语义为：

~~~text
左手柄前方按钮按住 → 左 O6 闭合
左手柄前方按钮松开 → 左 O6 张开
右手柄前方按钮按住 → 右 O6 闭合
右手柄前方按钮松开 → 右 O6 张开
~~~

适配器需要包含按钮边沿/长按判定、少量延迟和去抖、Joy 超时自动张开、home 未完成
时强制张开，以及 O6 命令发布频率和字段限幅。不能把 O6 命令塞入当前 26 维身体
`LowCmd`，除非 SDK 明确提供包含 O6 的统一新消息。

之后实现：

~~~text
/teleop/o6_trigger_state
  ↓
o6_command_adapter_real
  ↓
O6 真实 SDK 控制话题
~~~

### 8.3 真机实际状态接口核对

第一次运行真机时必须检查：

~~~bash
ros2 topic type /human_lower_state
ros2 topic echo /human_lower_state --once
~~~

确认：

- position 数组为 26；
- 12～18 是左臂；
- 19～25 是右臂；
- 单位是弧度；
- 状态没有 NaN/Inf；
- 状态频率满足控制要求。

### 8.4 真机命令核对

电机使能前检查：

~~~bash
ros2 topic echo /human_lower_command --once
ros2 topic hz /human_lower_command
~~~

确认：

- 命令长度为 26；
- 0～11 所有字段为零；
- 12～18 为左臂；
- 19～25 为右臂；
- kp、kd 和 eff 数值正确；
- 发布频率约为 50 Hz。

### 8.5 真机安全测试

推荐顺序：

1. 启动 XRoboToolkit 后台服务；
2. 只启动 xrobotoolkit_pxrea_adapter_real；
3. 确认 Quest 位姿、Grip、Trigger 和 X/A 数据；
4. 启动 topic_convertor，确认 /human_lower_state；
5. 启动真机 IK，但先不使能电机；
6. 观察 /human_lower_command；
7. 低速、低增益、单臂测试；
8. 双臂测试；
9. 最后接入 O6 控制。

不要在状态顺序、单位和命令链路未确认前直接使能机器人。

### 8.6 录制功能

已新增 `qiling_recording_real` 的第一版真机 RGB episode 录制链路。录制器独立于
遥操控制链，只订阅数据，并仅通过 IK 服务请求“保持/回 home”，不发布机器人控制指令。
当前配置记录：

- 头部 D435i、左腕 D405、右腕 D405 的 RGB 图像（640×480@20 Hz，JPEG 压缩）；
- 双臂实际关节位置；
- 双臂关节速度；
- 双臂实际末端位姿；
- O6 目标状态（当前仍没有可用的真实电机反馈话题）；
- 双臂关节位置目标和末端位姿目标，转换时可选择任一种作为 action；
- 每个 episode 的语言任务标签；
- 机器人处于 home/READY 后，左 Y 开始一个 episode；
- 左 X 标记当前 episode 成功并结束，数据才从 `.pending` 移入正式目录；
- 右 A 标记当前 episode 失败并结束，临时数据直接删除；
- 成功/失败后 IK 锁存当前实测姿态；右 B 请求从该姿态直接五次多项式回 home，
  不经过启动时过渡点；
- 回 home 完成并重新发布 READY 后，左 Y 才能开始下一条 episode。

每个 episode 的 MCAP 保存压缩 RGB、选定的遥操状态和派生的关节数据；录制中的 episode
位于 `recordings/.pending/`，只有成功 episode 才移动到正式目录。事件写入
`events.jsonl`，配置、相机序列号和任务标签写入 `session.yaml`。录制器只从
`/human_lower_state`、`/human_lower_command` 提取关节位置/速度和位置目标，不保存
原始 MIT 消息中的力矩、`kp`、`kd`、`vel` 等底层字段，派生 `JointState.effort` 为空。
每个成功 episode 独立保存为一个 MCAP 目录；录制中的 episode 位于 `.pending`，失败或
中断时删除，不进入转换输入。旧的 `convert_to_lerobot.py` 已删除；当前采用“准入清单 → ROS 导出中间数据 → LeRobot 打包”的两阶段转换链路，默认主 action 为
14 维双臂关节位置目标，同时保留末端位姿 action 供后续选择。

## 9. 按键约定

~~~text
左 Grip：左臂离合器
右 Grip：右臂离合器

当前临时实现：左/右 Trigger 分别控制左/右 O6 状态位；尚未连接实际 O6 电机。

最终要求：
左手柄前方按钮按住：左 O6 闭合
左手柄前方按钮松开：左 O6 张开
右手柄前方按钮按住：右 O6 闭合
右手柄前方按钮松开：右 O6 张开

左 Y：home/READY 后开始一个 episode
左 X：当前 episode 成功并保存
右 A：当前 episode 失败并删除
右 B：成功/失败后请求直接回 home
~~~

## 10. 后续实现顺序

1. 在真机网络环境中 source `qi` 和 CycloneDDS 配置；
2. 启动 topic_convertor，仅验证 `/human_lower_state` 和 DDS `lowstate`；
3. 单独验证 Quest 位姿、Grip、前方按钮和 X/A 话题映射；
4. 启动真机 IK，观察 home 状态机和 26 维命令，不使能电机；
5. 验证实测起点、过渡点、home 和 Grip 释放后的遥操解锁；
6. 在急停可用、低增益、低速度条件下进行单臂真机测试；
7. 进行双臂位置和姿态测试；
8. 获取 O6 消息定义，按最终按钮语义实现 O6 实际命令适配；
9. 验证 O6 单独闭合、张开、超时释放和 home 门控；
10. 将 topic_convertor、XRoboToolkit、真机 IK 和 O6 统一到总启动流程；
11. 在真机上验证三路相机和 recorder 的 640×480@20 Hz 采集；
12. 使用语言标签录制成功/失败 episode，转换为 LeRobot Dataset v3；
13. 做图像、观测、action 时间对齐和数据集回放检查。

## 11. 当前新增：仿真启动 Home 流程

已在仿真用 `qiling_kinematics` 中加入自动启动 home 状态机，现已将相同逻辑同步到
`qiling_kinematics_real`。两个包均由各自的 IK 节点独立管理 home，不依赖 MuJoCo
内部关键帧。

启动顺序：

```text
MuJoCo 启动并保持暂停
  -> 调用 /unpause_mujoco
  -> MuJoCo 保持所有非浮动可驱动关节为零位
  -> 启动对应包的 xr_teleop_real.launch.py
  -> 等待左右臂完整状态
  -> 安全过渡点
  -> home
  -> 每侧 Grip 释放一次
  -> 该侧进入遥操
```

当前 home（单位：度）：

```text
左臂  [0,  10, 0, -90, 0, 0, 0]
右臂  [0, -10, 0, -90, 0, 0, 0]
```

当前仿真过渡点（单位：弧度）：

```text
左臂  [1.00,  1.20, 0, -1.20, 0, 0, 0]
右臂  [1.00, -1.20, 0, -1.20, 0, 0, 0]
```

MuJoCo 已取消旧的 `teleop_start -> keyframe -> teleop_home` 内部过渡。其职责仅为：

- 启动时清零非浮动可驱动关节；
- 在运动学节点发出首个命令前保持零位；
- 执行 `/human_lower_command`。

仿真中 O6 命令槽在 home 阶段保持零值；真机中 O6 使用独立状态接口，并由
`/teleop/startup_home_complete` 门控，在 home 完成前强制输出零状态。当前 home
参数位于：

```text
src/qiling_kinematics/config/differential_ik.yaml       # 仿真
src/qiling_kinematics_real/config/differential_ik.yaml  # 真机
```

状态机日志应依次出现：

```text
MOVE_TO_TRANSITION
SETTLE_AT_TRANSITION
MOVE_TO_HOME
SETTLE_AT_HOME
COMPLETE
```

仿真包的 9 个测试和真机包的 8 个测试均已通过。真机包还完成了无硬件节点启动检查，
节点能加载 `s4_dual_arm.urdf` 并以 50 Hz 运行。真机尚未接通电机执行验证；首次上机
必须先观察 `/human_lower_command`，确认 26 维索引、单位、MIT 增益和腿部零字段，
再在急停可用、低增益条件下单臂测试。

## 12. 真机启动 Home 实现

真机启动命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qiling_kinematics_real xr_teleop_real.launch.py
```

真机 IK 节点的执行顺序为：

```text
等待 /human_lower_state 左右臂状态
  -> 首次实测关节位置
  -> 五次多项式移动到过渡点（2.5 s）
  -> 过渡点稳定（默认 0.30 s）
  -> 五次多项式移动到 home（3.0 s）
  -> home 稳定（默认 0.30 s）
  -> 发布 startup_home_complete=true
  -> 每侧 Grip 释放一次
  -> 进入该侧位姿遥操
```

当前真机 home（弧度，来自示教时的 26 维 `lowstate`；已排除 0..11 腿部数据）为：

```text
左臂 [0.0104905777,  0.6509880424, -0.1741435826, -1.7084382772,
      0.0738155171, -0.0963225737, -0.0936522484]
右臂 [-0.0749599487, -0.6475547552,  0.1073853672, -1.2964446545,
     -0.0009536889, -0.4251545072,  0.4945830405]
```

启动阶段的过渡点不变；重复录制流程的回 home 仍从当前实测姿态直接插值到上述
home，不经过该过渡点。

真机输出仍为唯一的 26 维 `/human_lower_command`，其中 0..11 使用实测腿部位置，12..18
为左臂，19..25 为右臂。home 阶段只给双臂发布轨迹参考，所有其他命令字段为零；
O6 由 `o6_trigger_state_node` 独立处理，但在 home 完成前被强制置零。

注意：真机 home 的起点采用第一帧完整实测状态，而不是盲目假定机器人处于全零。
如果状态在 home 期间超时，节点进入 `FAULT`，并保持当前实测臂位，不再继续向
home 运动。只有在实际安全验证后，才允许把该流程用于电机使能状态。

## 13. 后续实施阶段与验收标准

### Phase A：Qi/DDS 通信确认

目标是确认身体底层状态和命令链路，不启动遥操和 O6。

1. 在机器人主机 source ROS2、Qi 消息包和 CycloneDDS 网络配置；
2. 启动身体底层状态发布；
3. 启动 `topic_convertor`，确认 `/human_lower_state` 能稳定收到 26 个电机；
4. 检查位置、速度、力矩的单位和左右臂索引；
5. 暂不打开命令桥接，避免误发电机命令。

验收：`/human_lower_state` 稳定，频率满足要求，26 维数据有限且索引正确。

### Phase B：真机 home 空载验证

1. 保持电机未使能或处于可控低增益状态；
2. 启动 `qiling_kinematics_real`，不连接 Quest 也可以测试 home；
3. 观察 `WAITING_FOR_STATE` 到 `COMPLETE` 的日志；
4. 使用 rosbag 或 `ros2 topic echo` 检查 26 维命令；
5. 确认腿部 0..11 始终为零，双臂仅按预期轨迹运动；
6. 若发生状态超时，确认节点进入 `FAULT` 并保持当前实测位置。

验收：双臂完成“实测起点 → 过渡点 → home”，无碰撞、无突跳，O6 状态始终为零。

### Phase C：Quest 输入与双臂单臂测试

1. 启动 XRoboToolkit 无 GUI 服务；
2. 单独启动 PXREA 适配器，确认左右位姿和 `/xr/controller_joy`；
3. 检查左右 Grip 只控制对应侧离合器；
4. home 完成后释放每侧 Grip 一次；
5. 先只测试左臂，再只测试右臂，最后测试双臂；
6. 逐步验证平移、旋转、奇异位形限速、目标超时和急停行为。

验收：不按 Grip 时保持，按住 Grip 后位姿连续跟随，释放后停止并回到安全保持状态。

### Phase D：O6 实际控制

1. 确认 O6 的 ROS2 话题、消息类型、关节数量和左右手索引；
2. 将当前 Trigger 临时逻辑改为最终的前方按钮长按逻辑；
3. 新增独立 `o6_command_adapter_real`；
4. 将闭合/张开状态转换为 O6 所需的目标字段；
5. 加入按键去抖、延迟、Joy 超时自动张开和 home 门控；
6. 先单独测试 O6，再与双臂遥操同时运行。

验收：O6 不影响双臂 26 维命令；按钮按住闭合、松开张开；失联或 home 未完成时保持安全状态。

### Phase E：统一启动与停止

整合以下组件：

~~~text
XRoboToolkit 服务
topic_convertor
qiling_kinematics_real
O6 command adapter
qiling_recording_real recorder
~~~

统一脚本需要具备：启动顺序、PID 管理、日志目录、异常退出清理和停止时的安全输出。
在没有通过 Phase A～D 前，不要把所有节点直接合并为一个自动启动命令。

### Phase F：三路相机与数据录制

录制阶段再接入头部 D435、左右腕部 D405，并同步记录：

- 三路图像；
- 双臂实际关节位置和速度；
- 双臂实际末端位姿；
- O6 目标状态（真实反馈接入前不作为可靠观测）；
- 双臂关节位置目标、末端位姿目标（转换时选择 action）；
- 不记录关节力矩等 MIT 底层字段；
- 语言任务标签；
- 时间戳、episode 编号和任务标签；
- 左 Y 在 home/READY 后开始一个 episode；左 X 成功并保存，右 A 失败并删除；随后
  右 B 请求直接回 home，回到 READY 后再按左 Y 开始下一条。

录制原始的精选 ROS2 数据，不强行在实时控制线程内生成 LeRobot 格式。录制完成后
使用独立转换脚本生成 LeRobot Dataset v3，并检查图像、状态和 action 的时间对齐。
