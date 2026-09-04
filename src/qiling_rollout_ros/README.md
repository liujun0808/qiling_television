# qiling_rollout_ros

主机侧 ROS 2 Humble / Python 3.10 包。它负责真机状态与相机接入、归位/rollout/abort 状态机、
MIT 命令安全边界、重力前馈，以及与独立 XVLA worker 的本机 IPC。完整操作步骤见根目录
[`ROLLOUT_WORKFLOW.md`](../../ROLLOUT_WORKFLOW.md)。

## 包结构

```text
qiling_rollout_ros/
├── config/rollout_host.yaml        # ROS 参数与机器人安全参数
├── launch/rollout_host.launch.py   # bridge launch 文件
├── qiling_rollout_ros/
│   └── rollout_ros_bridge.py       # 运行节点与状态机
├── package.xml                     # ROS 运行依赖
└── setup.py                        # ament_python 安装定义
```

## 节点与接口

节点名：`/qiling_rollout_ros_bridge`

订阅：

- `/human_lower_state`：26 维机器人关节状态；启动归位和 abort hold 都依赖它。
- 三路 `sensor_msgs/Image`：仅在 `ROLLOUT` 状态创建订阅。

发布：

- `/human_lower_command`：26 维 `mit_msgs/MITJointCommands`，仅 armed 状态下发布。
- `/handscmd`：`qi/msg/HandsCmd`，用于 O6。

服务：

- `/rollout/finish`，`std_srvs/srv/Trigger`：人工确认任务完成，直接回 home 后结束。
- `/rollout/abort`，`std_srvs/srv/Trigger`：停止推理并进入 `ABORT_HOLD`。

## 状态机

Armed：

```text
WAITING_FOR_STATE → MOVE_TO_TRANSITION → SETTLE_AT_TRANSITION
→ MOVE_TO_HOME → SETTLE_AT_HOME → WAIT_BEFORE_ROLLOUT → ROLLOUT
```

完成：

```text
ROLLOUT → RETURN_DIRECT_TO_HOME → SETTLE_RETURN_HOME → FINISHED
```

中断：

```text
ROLLOUT / timeout → ABORT_HOLD
ABORT_HOLD + state stale → ABORT_FAULT
```

## 配置要点

`config/rollout_host.yaml` 定义 26 电机索引（腿 `0..11`、左臂 `12..18`、右臂 `19..25`）、双臂
home/过渡点、关节限位、MIT `kp/kd`、Pinocchio 重力前馈、O6 位置、超时和 IPC 参数。

该包不启动 `topic_convertor`，也不运行 Quest、XR bridge 或 differential IK；这些命令源必须与 rollout
互斥，避免多个发布者同时写入 `/human_lower_command`。
