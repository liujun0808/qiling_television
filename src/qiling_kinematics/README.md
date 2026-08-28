# qiling_kinematics

## Hierarchical DIK Phase 0

分层差分 IK 的 Phase 0 设计已经冻结，规范文件为：

- docs/hierarchical_dik_phase0_contract.md
- config/hierarchical_differential_ik.yaml

冻结层级为：位置严格高于姿态，姿态在位置 null-space 中运行，elbow swivel 在完整 wrist null-space 中运行。Phase 0 YAML 尚未被当前运行节点读取；它作为后续数学重构的参数契约。

## Hierarchical DIK Phase 1

Phase 1 已完成控制核心拆分和左右独立状态修复，详见：

- docs/hierarchical_dik_phase1_implementation.md

当前运行节点已经使用独立的 HierarchicalDIKSolver 和 ArmControlState，但 solver 数学仍是旧版平级 6D QP。严格位置优先、LOCAL_WORLD_ALIGNED 和 elbow swivel 将在后续阶段实现。

## Hierarchical DIK Phase 2

Phase 2 已完成 Pinocchio/MuJoCo 运动学和坐标系验证，详见：

- docs/hierarchical_dik_phase2_implementation.md

新增 `DualArmKinematics` 统一提供 base_link 方向基下的 `LOCAL_WORLD_ALIGNED` wrist/elbow FK 与 Jacobian，并通过左右各 7 列中心有限差分验证。MuJoCo 与 Pinocchio 的 home/扰动构型 FK 回归测试还发现并修正了 MuJoCo shoulder-yaw 轴和 O6 hand mount 朝向与 URDF 不一致的问题。

Phase 2 完成时运行节点仍保持 Phase 1 legacy `LOCAL + log6` 路径；该运行路径已在 Phase 3 被替换。

## Hierarchical DIK Phase 3

Phase 3 已将运行节点切换为严格 wrist-position QP，详见：

- docs/hierarchical_dik_phase3_implementation.md

当前运行路径使用 `DualArmKinematics` 的 `LOCAL_WORLD_ALIGNED` 3×7 position Jacobian 和 base_link position error。期望线速度按三维向量范数限幅，并保留每关节速度与一步位置 box 约束。Orientation 和 elbow 在本阶段强制关闭；target orientation 只用于 diagnostics，不进入 QP。

当前双臂位置差分 IK 完全使用 C++：

- Pinocchio：加载 `qi_robot_description/urdf/s4_dual_arm.urdf`，计算 FK、末端位姿和 Jacobian。
- ProxQP：每个 50 Hz 控制周期分别求解左右两个 7 维 `qdot` QP，带关节位置 margin 和速度 box 约束。
- 积分：使用实测并裁切后的控制周期更新 q reference，再发布到现有 40 维 MIT 命令接口。

## Hierarchical DIK Phase 4

Phase 4 已上线严格 wrist-orientation null-space QP，详见：

- docs/hierarchical_dik_phase4_implementation.md

当前运行路径先求解 Phase 3 的位置主任务，再对 `J_position` 做 SVD。仅当位置 Jacobian 满秩时，使用其固定 4 维 null-space basis 求解姿态增量：

```text
qdot_pose = qdot_position + N_position * z_orientation
```

姿态误差采用沿 `base_link` 轴表达的 `Log3(R_target * R_current^T)`。姿态层继续满足最终关节速度和一步位置边界，并在接受结果前检查它对位置任务造成的速度退化。位置 Jacobian 降秩、姿态输入无效、次级 QP 失败或位置退化超限时，只丢弃该周期姿态增量并回退到有效的 `qdot_position`；不会把单独的姿态层失败升级为手臂 FAULT。Elbow 仍保持关闭。

## Hierarchical DIK Phase 4.1

Phase 4.1 修复了 joint-limit margin 被错误当成第二套硬边界的问题，并加入奇异值诊断，详见：

- docs/hierarchical_dik_phase4_1_implementation.md

URDF `q_min/q_max` 现在是唯一硬位置边界；`joint_limit_margin_rad` 是连续速度阻尼区。进入 margin 后只逐渐降低朝硬限位方向的允许速度，朝关节范围内部的恢复速度始终保留。硬限位容差内的小幅仿真超调会得到强制向内的一步恢复边界，不会因为越过软 margin 而进入 `INVALID_BOUNDS`。

运行日志新增 position/full-wrist rank、最小奇异值、条件数、limit damper 状态和最近硬限位距离。完整 wrist Jacobian 使用 `characteristic_length_m` 对角速度行进行尺度统一。当前只诊断和告警，不在 Phase 4.1 自动缩放 position/orientation 速度。

## ROS 接口

## 启动时 Home 流程

`xr_teleop_real.launch.py` 当前在仿真闭环中也承担启动 Home。启动后，差分 IK 节点
会等待左右臂各收到一份完整且新鲜的 7 关节状态，然后由同一个 `/human_lower_command`
发布者依次执行：

```text
当前测量姿态（仿真为全零）
    -> 安全过渡点
    -> home
    -> 每侧 Grip 释放一次后允许该侧遥操
```

Home 关节目标为左臂 `[0, 10, 0, -90, 0, 0, 0]`、右臂 `[0, -10, 0, -90, 0, 0, 0]`
（单位：度）。当前仿真验证用过渡点为：

```text
left : [1.00,  1.20, 0.00, -1.20, 0, 0, 0]
right: [1.00, -1.20, 0.00, -1.20, 0, 0, 0]
```

过渡点和 home 使用五次多项式轨迹，并在目标附近等待稳定。Home 阶段会忽略 Quest
目标和 Grip；O6 对应命令的位置、速度和前馈保持零，并使用小的保持增益固定在零位。
任何一侧状态超时或目标未达到稳定误差，流程会
进入故障保持，不会自动开放遥操。

MuJoCo 只负责执行外部 MIT 命令：它启动时将所有非浮动可驱动关节置零，并在收到
运动学节点的首个命令前保持零位；解除暂停不会再读取或重置旧 keyframe。

推荐仿真启动顺序：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujoco_simulator simulate.launch.py
ros2 service call /unpause_mujoco std_srvs/srv/Empty "{}"
ros2 launch qiling_kinematics xr_teleop_real.launch.py
```

观察 `qiling_differential_ik` 日志中的 `startup home phase=`，应依次看到
`MOVE_TO_TRANSITION`、`SETTLE_AT_TRANSITION`、`MOVE_TO_HOME`、`SETTLE_AT_HOME`、
`COMPLETE`。仿真启动时的旧 MuJoCo keyframe home 流程已经关闭。

输入：

- `/joint_states`：使用左右臂 14 个关节名和位置。
- `/teleop/left_wrist_target`：`geometry_msgs/msg/PoseStamped`。
- `/teleop/right_wrist_target`：`geometry_msgs/msg/PoseStamped`。

差分 IK 状态输出：

- `/teleop/left_wrist_state`：`geometry_msgs/msg/PoseStamped`。
- `/teleop/right_wrist_state`：`geometry_msgs/msg/PoseStamped`。

目标位姿的 `header.frame_id` 应为 `base_link`。Quest bridge 后续只需要把重定位/缩放后的左右控制器末端位姿发布到这两个话题，不应在 ROS 回调中直接发布电机命令。

输出：

- `/human_lower_command`：40 个 `mit_msgs/msg/MITJointCommand`。
- 当前只填左右臂：左臂索引 12--18，右臂索引 26--32。
- 腿部索引 0--11 保持零；O6 索引 19--25、33--39 留给后续 O6 adapter。

## 安装与编译

正式主机需要安装系统依赖（不要从 Conda 启动 ROS 2 控制节点）：

```bash
sudo apt install ros-humble-proxsuite libsimde-dev
source /opt/ros/humble/setup.bash
colcon build --packages-select qiling_kinematics --symlink-install
source install/setup.bash
```

运行：

```bash
ros2 launch qiling_kinematics differential_ik.launch.py
```

当前节点不会凭空生成 Quest 目标；在 Quest bridge 接入前，可用两个 `PoseStamped` 话题做离线/仿真测试。首次收到有效机器人关节状态后，未收到目标的一侧会保持当前 FK 位姿，避免节点启动时跳动。

## 双臂仿真闭环测试

先启动 MuJoCo 并解除暂停：

```bash
ros2 launch mujoco_simulator simulate.launch.py
ros2 service call /unpause_mujoco std_srvs/srv/Empty "{}"
```

然后启动 IK 和平滑目标测试源：

```bash
ros2 launch qiling_kinematics closed_loop_test.launch.py
```

该测试源会从当前双臂末端位姿建立锚点，再以小幅正弦轨迹发布目标，用于验证：

```text
JointState -> pose_target_demo -> PoseStamped
           -> Pinocchio + ProxQP
           -> /human_lower_command -> MuJoCo
```

它只用于仿真验收，后续由 Quest/XR bridge 替换 `pose_target_demo`。

## Quest/XR 输入桥接

`xr_pose_clutch_bridge` 是与具体 XR SDK 解耦的 C++ 桥接层。Quest/XR backend 需要发布：

- `/xr/left_controller_pose`：左手柄位姿，`PoseStamped`；
- `/xr/right_controller_pose`：右手柄位姿，`PoseStamped`；
- `/xr/controller_joy`：`sensor_msgs/msg/Joy`，默认 buttons[4] 为左 Grip、buttons[5] 为右 Grip。

桥接层以 50 Hz 发布左右末端目标。Grip 按下时记录“当前机器人末端位姿 + 当前手柄位姿”作为锚点，Grip 松开时对应手臂回到实测末端位姿并停止跟随。重新按下 Grip 会重新锚定，不会产生跳变。XR 输入或机器人状态超时后，该侧进入保持状态。

当前没有把 XRoboToolkit/Quest SDK 直接放入本包；SDK 适配器只需要把数据转换成上述三个 ROS 话题即可。

## Hierarchical DIK Phase 5.0 / 5.1 / 5.2

Phase 5.0 已加入只读 anthropomorphic elbow 几何诊断，详见：

- docs/hierarchical_dik_phase5_0_implementation.md
- docs/hierarchical_dik_phase5_1_implementation.md

模块根据 `shoulder -> current wrist` 轴，将当前 elbow 投影到垂直平面，并融合左右侧 outward、轻微 downward 和 teleop home direction。它维护 previous preferred direction，在投影退化时按 previous、home 顺序回退，并拒绝生成随机正交方向。

Phase 5.2 已将 elbow 作为完整 6D wrist 任务之后的 1D tertiary task 接入。只有完整 wrist Jacobian 满秩且远离奇异位形时，才执行：

```text
qdot_final = qdot_pose + n_wrist * z_elbow
```

其中 `n_wrist` 是 `6x7` wrist Jacobian 的唯一零空间方向，`z_elbow` 由单变量 ProxQP 结合 swivel、rest posture、joint centering 和 smoothness 求解。它继续经过 joint velocity box、position degradation、orientation degradation 三重检查；失败时回退到 `qdot_pose`。详见：

- docs/hierarchical_dik_phase5_2_implementation.md

日志中的 `elbow_status`、`elbow_applied`、`elbow_scale` 和两类 elbow degradation 用于区分“肘部任务未启用/几何无效/奇异淡出/QP 回退/成功执行”。

## XRoboToolkit PXREA 真实输入

当前主机安装的 XRoboToolkit SDK 路径为：

```text
/opt/apps/roboticsservice/SDK/include/PXREARobotSDK.h
/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so
```

已加入 `xrobotoolkit_pxrea_adapter`。该节点使用 `PXREAInit` 注册 SDK 回调，解析官方控制器状态中的：

```text
Controller.left/right.pose
Controller.left/right.grip
Controller.left/right.trigger
```

适配器以约 90 Hz 发布最新 XR 帧；`xr_pose_clutch_bridge` 仍以 50 Hz 读取最新帧并输出末端目标。`/xr/controller_joy` 中的数据约定为：

```text
axes[0]：左 trigger
axes[1]：右 trigger
axes[2]：左 grip 模拟值
axes[3]：右 grip 模拟值
buttons[4]：左 Grip，大于阈值时为 1
buttons[5]：右 Grip，大于阈值时为 1
```

启动真实 XR 输入链路前，先运行 PC Service，并在 Quest 3 客户端中连接机器人主机、打开 Controller Tracking 和 Send：

```bash
/opt/apps/roboticsservice/runService.sh
```

然后启动 MuJoCo 并解除暂停：

```bash
ros2 launch mujoco_simulator simulate.launch.py
ros2 service call /unpause_mujoco std_srvs/srv/Empty "{}"
```

另一个终端启动真实 Quest 链路：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qiling_kinematics xr_teleop_real.launch.py
```

真实输入测试时不要同时启动 `xr_pose_demo`。建议先检查输入话题，再按住 Grip：

```bash
ros2 topic hz /xr/left_controller_pose
ros2 topic hz /xr/right_controller_pose
ros2 topic echo /xr/controller_joy --once
```

当前 PXREA 适配器发布的是 `xr_origin` 原始坐标系。第一次真实测试只验证左右手柄、Grip 和位姿变化是否稳定；确认方向后，再配置 `xr_origin` 到 `base_link` 的固定旋转和尺度，不直接复用 UR5 示例中的坐标变换。

使用内置测试输入验证桥接和 MuJoCo：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 终端 1：启动 MuJoCo 并解除暂停
ros2 launch mujoco_simulator simulate.launch.py
ros2 service call /unpause_mujoco std_srvs/srv/Empty "{}"

# 终端 2：启动差分 IK、XR clutch bridge 和模拟手柄输入
ros2 launch qiling_kinematics xr_teleop_demo.launch.py
```

内置 demo 前 2 秒保持 Grip 松开，随后同时按住左右 Grip 并做小幅平移/旋转，12 秒后释放 Grip。真实 Quest backend 接入时，保留 `xr_pose_clutch_bridge`，停止 `xr_pose_demo` 即可。
