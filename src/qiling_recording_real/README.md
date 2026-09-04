# qiling_recording_real

真机 RGB episode 录制包。录制器只订阅话题，不发布任何机器人控制指令。

## 按键与单 episode 流程

所有按键都只响应上升沿。每个 episode 都必须经历“home → 开始 → 成功/失败 → 回
home”，不会自动开始下一条：

- 机器人处于 READY/home 后，左 Y：开始一个 episode。
- 录制期间，左 X：标记成功并结束当前 episode；数据才会保存。
- 录制期间，右 A：标记失败并结束当前 episode；数据被删除，不进入正式目录。
- 成功/失败后机器人先锁存当前实测姿态，进入保持；右 B：请求从当前姿态直接
  五次多项式插值回 home，不经过启动时的过渡点。
- 回 home 期间不录制相机；收到 READY 后，按左 Y 才能开始下一条。
- 左右 Grip、Trigger：不参与录制控制。

当前 `/xr/controller_joy` 约定为：左 X=`buttons[0]`、右 A=`buttons[1]`、
左 Y=`buttons[2]`、右 B=`buttons[3]`。

## 启动

录制期间需要让 ROS RealSense 节点独占相机。若 `camera-ws.service` 正在运行，先执行：

```bash
sudo systemctl stop camera-ws.service
```

然后按以下顺序启动：

```bash
ros2 run topic_convertor topic_converter_node
ros2 launch qiling_kinematics_real xr_teleop_real.launch.py
ros2 launch qiling_recording_real real_recording.launch.py
```

## 只启动三路相机（rollout / 画面检查）

以下 launch 只启动头部 D435i、左腕 D405、右腕 D405，不启动 episode recorder，也不发布任何
机器人命令。它从 `config/real_recording.yaml` 读取已确认的序列号与映射，并强制检查三路均为
原生 RGB `640×480@30Hz`：

```bash
ros2 launch qiling_recording_real tri_camera.launch.py
```

也可直接运行包装脚本：

```bash
bash src/qiling_recording_real/scripts/start_tri_cameras.sh
```

三个输出话题为：

```text
/camera_head/head_camera/color/image_raw
/camera_left/left_camera/color/image_raw
/camera_right/right_camera/color/image_raw
```

`real_recording.launch.py` 已经会启动同样的三路相机，因此录制时二者只能选择一个，不能同时启动。

录制器启动后处于 idle 状态。按左 Y 开始一个 episode；成功或失败标记只关闭当前
episode。成功/失败后按右 B 回 home，回到 READY 后按左 Y 开始下一条。

录制器通过 `/teleop/request_recording_hold` 和 `/teleop/request_home` 调用 IK 节点的
服务，仅请求状态切换，不发布任何机器人关节控制话题。

## 语言标签

每个 episode 必须有非空任务描述。可以在按左 Y 开始前发布：

```bash
ros2 topic pub --once /recording/language std_msgs/msg/String \
  "{data: '将物体放入盒子'}"
```

也可以在 `config/real_recording.yaml` 的 `language.default_task` 中填写固定任务。
左 Y 触发时会把当前任务锁存到该 episode；录制期间发布的新语言不会修改当前 episode，
可作为下一条 episode 的任务。没有任务标签时，当前配置会拒绝开始 episode。

## 输出

默认输出目录是：

```text
/home/coral/liujun/qiling_television/recordings/
```

录制中的 episode 临时位于 `recordings/.pending/`；只有成功 episode 才会移动到正式目录。
每个正式 episode 的 `rosbag/` 是 MCAP 文件，三路 RGB 均以相机原生
640×480@30Hz 的 `sensor_msgs/msg/CompressedImage`（JPEG）保存；不裁剪、不缩放、不做
软件旋转，也不做软件限帧。图像消息保留原始帧的 header 时间戳，bag 写入时间为该原始帧
的本机接收时间。开始录制前会核验三路实际输入都是 640×480，若驱动回退到其他 profile，
则拒绝启动该条 episode。`events.jsonl` 保存开始、成功、失败、停止等事件，
`session.yaml` 保存状态、分辨率、频率、按键、任务标签、话题配置以及每路相机的
接收/压缩/写入/丢帧计数。失败/中断 episode 不会留下正式 episode 目录。

三路相机使用独立的有界队列和 JPEG 工作线程；ROS 图像回调只更新新鲜度和入队，
不压缩也不写 MCAP。采样线程使用单调时钟独立按 50Hz 排程，原子地生成右臂 observation、
右臂关节 action、右手二值 gripper action 三条消息，再放入最高优先级队列。JPEG worker
只负责编码和放入最低优先级图像队列；唯一的 MCAP writer 线程串行序列化/写入所有队列。
因此相机编码、MCAP I/O 不会拖慢 50Hz 状态/动作采样。按左 Y 前要求三路相机最近0.5秒内
均收到 640×480 图像。录制期间每2秒会打印每路相机的
`recv/enc/write/drop_rate/drop_q/drop_writer/error` 统计。

底层状态和命令回调只缓存最新数据。独立的 50 Hz 采样线程使用同一个采样时间戳，成组
写入右臂 observation、右臂关节 action 和右手二值 gripper action，从而避免
`/human_lower_state` 的高频输入直接决定数据频率。超过 0.1 秒没有更新的源数据不会
被重复写入，并会在日志和 `session.yaml` 中累计为 skipped sample。

录制的数据字段为：

- RGB 图像；
- 右臂 7 维实际关节位置、7 维关节速度和实际末端位姿；
- 右臂 7 维关节位置目标和末端位姿目标，可分别作为两种 action 来源；
- 右手二值开合目标：`0=open`、`1=closed`，由 O6 状态位的 bit 1 派生；
- 右手柄/遥操模式等用于回溯的事件与状态。

左臂关节、左腕状态/目标和左控制器位姿不写入正式 episode。`/xr/controller_joy`
仍会保留，因为 Y/X/A/B 录制按键位于该消息中。当前没有可靠的 O6 位置反馈，
`right_gripper_closed` 是命令目标而不是实测手指位置。

明确不录制关节力矩、`kp`、`kd`、`vel` 等 MIT 底层命令字段。录制器虽然读取
`/human_lower_state` 和 `/human_lower_command`，但只从中派生位置/速度消息，原始
`MITLowState`、`MITJointCommands` 不会写入 MCAP；派生 `JointState.effort` 也保持为空。

## 转换为 LeRobot

当前转换链路只保留两阶段实现：

```text
成功 episode MCAP
  ↓ build_training_admission_manifest.py（生成可用连续片段清单）
  ↓ export_manifest_to_intermediate.py（ROS Python 3.10）
intermediate：JPEG + q/dq/q_target/O6 标签
  ↓ pack_intermediate_to_lerobot.py（lerobot051 Python 3.12）
LeRobot v3 数据集
```

`export_manifest_to_intermediate.py` 是唯一读取 MCAP/ROS bag 的转换阶段；
`pack_intermediate_to_lerobot.py` 是唯一依赖 LeRobot 的打包阶段。旧的历史双臂直转脚本与已废弃的
人工 review/promotion 脚本已删除，避免被误用于当前右臂 + O6 数据结构。

## 注意事项

`/home/coral/start_cameras.sh` 和 `camera_ws_server.py` 中旧的左右 D405 映射与当前
确认的映射相反，不能用于本录制流程。当前配置使用：

- 头部 D435i：`135122070003`
- 左手 D405：`352122273604`
- 右手 D405：`409122273836`

当前流程请求三路 RealSense 原生 RGB 640×480@30Hz。深度、红外和IMU均关闭。
`session.yaml` 中相机 `source_*` 和 `width/height/fps` 必须完全一致；它们分别记录
驱动请求与落盘规范，不代表两次图像处理。

## 离线质检

每条成功 episode 录制完成后，建议在不启动任何 ROS 节点的情况下运行：

```bash
ros2 run qiling_recording_real check_episode_quality \
  recordings/episode_YYYYMMDD_HHMMSS_mmm
```

它会逐项输出三路 JPEG 分辨率、实际 RGB 频率和最大帧间隔、右臂 observation/action/
gripper 的原子 50Hz 批次、字段维度、队列/写入错误以及任务语言标签。帧率、帧间隔、
原子批次和写队列异常仅输出 `WARN`，不会阻断转换；缺 topic、空数据、JPEG 损坏或字段
格式错误仍会以 `OVERALL: FAIL` 标记。检查只读取 MCAP，不会发布任何控制消息。
