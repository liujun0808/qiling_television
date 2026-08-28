# qiling_kinematics_real

真机专用双臂末端位姿遥操包。该包与 MuJoCo 仿真包独立，使用固定
`base_link` 的 `s4_dual_arm.urdf`，通过 Pinocchio + ProxSuite 执行双臂
层级差分 IK。

## 当前真机接口

状态输入：

```text
/human_lower_state   mit_msgs/msg/MITLowState
```

状态消息中的 `joint_states.position` 固定为 26 个身体电机，真机包只读取：

```text
12..18  左臂 7 关节
19..25  右臂 7 关节
```

命令输出：

```text
/human_lower_command   mit_msgs/msg/MITJointCommands
```

命令固定为 26 维。0..11 腿部字段全部为零，12..18 和 19..25 分别为双臂
的 MIT 位置/速度/增益/前馈力矩命令。O6 不进入该命令，使用独立接口。

## 启动

先启动 XRoboToolkit PC Service 和 Quest 端数据发送，再加载 ROS2 环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

启动真机遥操链路：

```bash
ros2 launch qiling_kinematics_real xr_teleop_real.launch.py
```

启动后，`differential_ik_real_node` 会自动执行一次安全回 home 流程。它等待
`/human_lower_state` 中左右臂状态都有效，然后以首次实测关节位置为起点，按
以下顺序发布 26 维 MIT 命令：

```text
实测当前位置 → 过渡点 → home → 等待稳定 → 开放遥操
```

当前 home（弧度）为：

```text
left:  [0,  0.1745329252, 0, -1.5707963268, 0, 0, 0]
right: [0, -0.1745329252, 0, -1.5707963268, 0, 0, 0]
```

过渡点为：

```text
left:  [1.0,  1.2, 0, -1.2, 0, 0, 0]
right: [1.0, -1.2, 0, -1.2, 0, 0, 0]
```

过渡点和 home 之间使用五次多项式轨迹，默认分别为 2.5 s 和 3.0 s，
到点后还需保持 0.30 s。home 阶段忽略 Quest 的目标位姿和 Grip；home 完成
后，每只手还必须先释放一次 Grip，之后才允许该侧进入遥操，防止手柄已经按住
时产生目标跳变。home 状态通过以下话题发布，`true` 表示已完成：

```text
/teleop/startup_home_complete   std_msgs/msg/Bool
```

O6 状态节点在收到该话题为 `true` 之前强制输出零状态，因此 home 期间不会
因手柄输入改变 O6 状态。O6 仍然是独立于 26 维双臂命令的接口。

启动文件包含：

```text
xrobotoolkit_pxrea_adapter_real  Quest 位姿/按键 → ROS2
xr_pose_clutch_bridge_real       Grip 离合器与相对位姿目标
differential_ik_real_node        /human_lower_state → 26 维身体命令
o6_trigger_state_node             Trigger → 独立 O6 状态位
```

O6 内部状态话题暂定为：

```text
/teleop/o6_trigger_state   std_msgs/msg/UInt8
```

其中 bit 0 表示左 O6 闭合，bit 1 表示右 O6 闭合。统一 O6 SDK 命令消息
确认后，再增加独立的 O6 命令适配器。

真机还必须单独运行 `topic_convertor`，它负责把 ROS2 的 26 维 MIT 命令转到
DDS `lowcmd`，并把 DDS `lowstate` 转成 `/human_lower_state`。参数必须打开双向
桥接：

```bash
ros2 run topic_convertor topic_converter_node --ros-args \
  -p expected_motor_count:=26 \
  -p enable_state_bridge:=true \
  -p enable_command_bridge:=true \
  -p strict_command_size:=true
```

该命令要求当前 ROS 环境已经能找到 `qi` 消息包；若 `ros2 run` 找不到
`topic_convertor` 或构建时找不到 `qiConfig.cmake`，先在机器人上的 DDS/SDK
工作空间构建并 source 对应工作空间。

## 安全说明

首次连接真机时应先只运行 PXREA 适配器，确认 Quest 位姿和按键话题正常，
再启动 IK 和底层命令转换节点。第一次执行 home 时建议：

1. 机器人周围清空，双臂前方尤其不能有桌面或人员；
2. 先降低 `command_kp` 或在底层设置限幅，并准备急停；
3. 观察日志中的 `MOVE_TO_TRANSITION`、`SETTLE_AT_TRANSITION`、
   `MOVE_TO_HOME`、`SETTLE_AT_HOME`、`COMPLETE` 状态；
4. 确认左右臂状态频率稳定、26 维命令顺序和身体 SDK 的 MIT 参数含义；
5. 确认腿部零字段不会影响底层腿部控制器，再逐步提高控制增益。

若状态在 home 过程中超时，节点进入 `FAULT` 并将臂部参考保持在当前实测位置，
不会继续向 home 运动。`startup_home_enabled` 可暂时设为 `false` 做通信检查，
但真机正式遥操前应保持为 `true`。
