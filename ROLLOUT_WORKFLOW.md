# Qiling XVLA Rollout 操作流程

本文是主机侧 XVLA 真机 rollout 的唯一操作说明。ROS bridge 与 XVLA worker 的目录职责分别见
[`src/qiling_rollout_ros/README.md`](src/qiling_rollout_ros/README.md) 和
[`rollout/README.md`](rollout/README.md)。

## 1. 前置条件

- 真机 PC：机器人 SDK、三路相机节点和 DDS 通信已正常运行。
- 主机：已构建当前工作空间，能收到 `/human_lower_state` 和三路 RGB。
- 仅允许一个节点发布 `/human_lower_command`：rollout 时必须停止 Quest 遥操、XR bridge、差分 IK 和其他命令发布者。
- 现场清空，实体急停可用。
- 主机和真机处于相同 ROS/DDS Domain；`topic_convertor` 全系统只允许启动一份。

当前三路相机话题：

```text
/camera_head/head_camera/color/image_raw
/camera_left/left_camera/color/image_raw
/camera_right/right_camera/color/image_raw
```

## 2. 构建和启动顺序

首次修改代码或切换主机时构建：

```bash
cd /home/ub/program/qiling_television
source /opt/ros/humble/setup.bash
colcon build --packages-select qiling_rollout_ros
source install/setup.bash
```

真机 PC：先启动机器人 SDK，再在一个已 source 工作空间的终端启动三路相机。该 launch 只启动
RealSense 相机，不会启动 episode recorder；三路画面为 `640×480 @ 30 Hz`。

```bash
cd /home/ub/program/qiling_television
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qiling_recording_real tri_camera.launch.py
```

终端 1（主机）：启动一次 DDS/ROS 命令转换。真机 PC 保持机器人 SDK 与上述相机节点运行。

```bash
cd /home/ub/program/qiling_television
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run topic_convertor topic_converter_node --ros-args \
  -p expected_motor_count:=26 \
  -p enable_state_bridge:=true \
  -p enable_command_bridge:=true \
  -p strict_command_size:=true
```

终端 2：先以 shadow 验证链路。shadow 不会发布 `/human_lower_command` 或 `/handscmd`。

```bash
cd /home/ub/program/qiling_television
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qiling_rollout_ros rollout_host.launch.py
```

终端 3：启动 XVLA worker。它使用 `lerobot051` 环境并从
`rollout/config/xvla_rollout.yaml` 读取 checkpoint、任务文本和 IPC 配置。

```bash
cd /home/ub/program/qiling_television
bash rollout/scripts/start_xvla_worker.sh
```

确认三路图像、`/human_lower_state`、shadow 日志和模型候选 action 正常后，停止 shadow bridge 与 worker。

## 3. Armed rollout

重新启动 bridge，并显式设置 `armed`：

```bash
cd /home/ub/program/qiling_television
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qiling_rollout_ros rollout_host.launch.py execution_mode:=armed
```

然后启动 worker：

```bash
cd /home/ub/program/qiling_television
bash rollout/scripts/start_xvla_worker.sh
```

Armed 状态机如下：

```text
WAITING_FOR_STATE
  → 当前实测双臂姿态 → 过渡点 → home
  → 双臂 home 到位且稳定
  → 保持 home 等待 10 秒
  → 创建图像订阅和 worker IPC
  → ROLLOUT
```

启动归位和 home 后的 10 秒等待期间只订阅 `/human_lower_state`；不会将图像或观测交给模型。等待结束后才
开始模型推理。等待时间由 `rollout_start_delay_sec` 配置，当前为 10 秒。

运行时控制约束：

- 双臂 MIT 命令频率：50 Hz；腿部保持实测位置。
- 左臂固定在 home，右臂使用模型 action。
- XVLA action 按训练数据的 30 Hz 消费；每次 worker 返回 8 个 action。
- 右臂目标经过关节限位、速度限制；双臂使用 Pinocchio 重力前馈。
- 左 O6 保持张开；右 O6 由模型 action 第 8 维通过滞回阈值控制。

## 4. 正常完成

当前模型没有 done/success 输出。操作员确认任务完成后，在另一个已 source 工作空间的终端调用：

```bash
ros2 service call /rollout/finish std_srvs/srv/Trigger "{}"
```

完成状态机：

```text
ROLLOUT
  → 停止接收模型 action、清空 action queue
  → 当前实测双臂姿态直接平滑回 home
  → home 到位且稳定
  → 停止状态/图像订阅、worker 推理和 MIT/O6 命令发布
```

完成后不会向双臂发布回零命令。bridge 保留为终态诊断进程；下一次任务需重新启动 bridge 和 worker。

## 5. 异常中断与超时

发现异常抓取姿态但没有即时碰撞风险时，调用：

```bash
ros2 service call /rollout/abort std_srvs/srv/Trigger "{}"
```

正式进入 `ROLLOUT` 后，`max_rollout_duration_sec` 也会开始计时；当前配置为 45 秒。超时自动进入同样的
`ABORT_HOLD`，归位时间不计入超时。

```text
人工 abort 或 rollout 超时
  → 停止图像订阅和 XVLA worker 推理
  → 清空未执行 action
  → 锁存 abort 瞬间的双臂实测关节角
  → ABORT_HOLD：50 Hz hold MIT + 重力前馈
```

`ABORT_HOLD` 不会自动回 home、回零或张开 O6；O6 保持 abort 前状态。它会保留 `/human_lower_state`
订阅以维持 hold。若状态反馈丢失，则进入 `ABORT_FAULT` 并停止继续发布控制命令；应按现场风险立即使用实体急停。

发生碰撞风险或人身风险时，优先使用**实体急停**。不要把直接关闭 rollout 终端当成安全中断方式：底层可能保持最后一帧 MIT 指令。

Abort 后由人工确认安全，再切换回遥操处理姿态或重启新的 rollout。

## 6. 常用诊断

```bash
# 查看 rollout 状态机日志
ros2 topic echo /rosout

# 检查状态与相机话题是否存在
ros2 topic hz /human_lower_state
ros2 topic hz /camera_head/head_camera/color/image_raw
ros2 topic hz /camera_left/left_camera/color/image_raw
ros2 topic hz /camera_right/right_camera/color/image_raw

# 查看 bridge 的服务
ros2 service list | rg '/rollout/(finish|abort)'
```

bridge 的结构化运行日志写入 `rollout/records/rollout_*.jsonl`。重点观察 `rollout_phase`、
`command_blocked`、`action_chunk_received`、`worker_error` 和 `gravity_disabled_after_error` 事件。

## 7. 主要配置位置

- `src/qiling_rollout_ros/config/rollout_host.yaml`：ROS 话题、26 电机映射、双臂 home/过渡点、MIT 参数、重力前馈、O6、超时和服务名称。
- `rollout/config/xvla_rollout.yaml`：模型 checkpoint、任务文本、GPU、worker IPC。
- `src/qiling_rollout_ros/qiling_rollout_ros/rollout_ros_bridge.py`：归位、rollout、finish 和 abort 状态机。
- `rollout/worker/xvla_worker.py`：Python 3.12 下的 LeRobot XVLA 推理进程。
