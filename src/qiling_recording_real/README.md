# qiling_recording_real

真机 RGB episode 录制包。录制器只订阅话题，不发布任何机器人控制指令。

## 按键

- 左 Y：开始新的 episode；只有按键上升沿生效。
- 右 B：写入成功事件并立即停止当前 episode。
- 右 A：写入失败事件并立即停止当前 episode。
- 左右 Grip、Trigger：不参与录制控制。

当前 `/xr/controller_joy` 约定为：左 Y=`buttons[2]`、右 B=`buttons[3]`、右 A=`buttons[1]`。

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

录制器启动后处于 idle 状态。按左 Y 创建 episode；按右 B 或右 A 完成并停止。

## 语言标签

每个 episode 必须有非空任务描述。可以在按左 Y 之前发布：

```bash
ros2 topic pub --once /recording/language std_msgs/msg/String \
  "{data: '将物体放入盒子'}"
```

也可以在 `config/real_recording.yaml` 的 `language.default_task` 中填写固定任务。
按左 Y 时会把当前任务写入 episode；录制期间再次发布非空字符串也会保留在 MCAP
中。没有任务标签时，当前配置会拒绝开始录制。

## 输出

默认输出目录是：

```text
/home/coral/liujun/qiling_television/recordings/
```

每个 episode 的 `rosbag/` 是 MCAP 文件，三路 RGB 以
`sensor_msgs/msg/CompressedImage`（JPEG）保存；图像消息保留相机原始 header 时间戳，
bag 写入时间为本机接收时间。`events.jsonl` 保存开始、成功、失败、停止等事件，
`session.yaml` 保存序列号、分辨率、频率、按键、任务标签和话题配置。

录制的数据字段为：

- RGB 图像；
- 双臂实际关节位置、关节速度和实际末端位姿；
- 双臂关节位置目标和末端位姿目标，可分别作为两种 action 来源；
- Quest/遥操模式等用于回溯的事件与状态。

明确不录制关节力矩、`kp`、`kd`、`vel` 等 MIT 底层命令字段。录制器虽然读取
`/human_lower_state` 和 `/human_lower_command`，但只从中派生位置/速度消息，原始
`MITLowState`、`MITJointCommands` 不会写入 MCAP；派生 `JointState.effort` 也保持为空。

## 转换为 LeRobot

录制完成后，修改 `config/lerobot_conversion.yaml` 的输出目录和 `action_mode`，然后：

```bash
ros2 run qiling_recording_real convert_to_lerobot \
  --config-file /home/coral/liujun/qiling_television/src/qiling_recording_real/config/lerobot_conversion.yaml
```

`action_mode: joint`（默认）使用 14 维双臂关节位置目标作为主 action，
`action_mode: eef` 使用 14 维双臂末端位姿目标；`include_alternate_actions: true`
会把另一种 action 也保存在数据集字段中，便于后续选择。`action_mode: both` 会将
末端位姿和关节位置拼接成一个 28 维主 action，通常只有在训练策略明确需要时才使用。

转换器按相机接收时间戳做离线近似对齐，并通过 LeRobot Dataset v3 API 生成 Parquet
和 MP4，不在实时录制过程中直接写 LeRobot 文件。需要在转换环境预先安装支持
Dataset v3 的 LeRobot；转换器不会自动安装依赖。

## 注意事项

`/home/coral/start_cameras.sh` 和 `camera_ws_server.py` 中旧的左右 D405 映射与当前
确认的映射相反，不能用于本录制流程。当前配置使用：

- 头部 D435i：`135122070003`
- 左手 D405：`352122273604`
- 右手 D405：`409122273836`

三路相机均配置为 RGB 640×480@20 Hz，关闭深度、红外和 IMU。若某台设备实际不接受
该 profile，RealSense 节点会在启动日志中报告 profile 错误，不会静默改用其他频率。
