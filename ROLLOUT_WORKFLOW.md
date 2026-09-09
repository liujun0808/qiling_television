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

真机本地三路 raw 相机话题：

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

真机 PC：先启动机器人 SDK，再运行一键脚本。脚本先启动三路 RealSense，相机保持
`640×480 @ 30 Hz`；等待 5 秒后自动启动 `15 Hz、JPEG quality=80` 的 rollout 压缩传输。
脚本不会启动 episode recorder，并保持前台运行；按 Ctrl+C 会同时关闭相机和压缩节点。

```bash
cd /home/ub/program/qiling_television
bash src/scripts/start_rollout_cameras.sh
```

如需临时覆盖压缩图像频率或 JPEG 质量，可在启动脚本前设置环境变量：

```bash
ROLLOUT_IMAGE_RATE_HZ=20.0 ROLLOUT_JPEG_QUALITY=85 \
  bash src/scripts/start_rollout_cameras.sh
```

主机 bridge 实际订阅以下压缩话题：

```text
/qiling_rollout/camera_head/image/compressed
/qiling_rollout/camera_left/image/compressed
/qiling_rollout/camera_right/image/compressed
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
- XVLA action 按训练数据的 30 Hz 消费；worker 返回原生完整 32 步 chunk。bridge 维持 12 步
  （400 ms）未来队列，剩余 8 步（约 267 ms）时提前请求新推理。新 chunk 只保护最近 3 步旧动作，
  替换更远的旧预测，并在替换边界融合 4 步右臂关节目标。
- XVLA 输出是 30 Hz 关节位置 sample；bridge 仅在相邻两条已通过队列校验的 sample 之间线性插值，
  再在每个 50 Hz MIT tick 施加关节限位、最大速度和最大加速度限制。队列没有下一条已排程 action
  或 action 超时时，冻结当前 `q_ref`，绝不外推模型轨迹。双臂使用 Pinocchio 重力前馈。
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

正式进入 `ROLLOUT` 后，`max_rollout_duration_sec` 也会开始计时；当前配置为 200 秒。超时自动进入同样的
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
ros2 topic hz /qiling_rollout/camera_head/image/compressed
ros2 topic hz /qiling_rollout/camera_left/image/compressed
ros2 topic hz /qiling_rollout/camera_right/image/compressed

# 实测压缩后跨 Wi-Fi 的带宽
ros2 topic bw /qiling_rollout/camera_head/image/compressed
ros2 topic bw /qiling_rollout/camera_left/image/compressed
ros2 topic bw /qiling_rollout/camera_right/image/compressed

# 查看 bridge 的服务
ros2 service list | rg '/rollout/(finish|abort)'
```

bridge 的结构化运行日志写入 `rollout/records/rollout_*.jsonl`。重点观察 `rollout_phase`、
`command_blocked`、`action_chunk_received`、`worker_error` 和 `gravity_disabled_after_error` 事件。

## 7. 主要配置位置

- `src/qiling_rollout_ros/config/rollout_host.yaml`：ROS 话题、26 电机映射、双臂 home/过渡点、MIT 参数、右臂速度/加速度限制、重力前馈、O6、超时和服务名称。
- `rollout/config/xvla_rollout.yaml`：模型 checkpoint、任务文本、GPU、worker IPC。
- `src/qiling_rollout_ros/qiling_rollout_ros/rollout_ros_bridge.py`：归位、rollout、finish 和 abort 状态机。
- `rollout/worker/xvla_worker.py`：Python 3.12 下的 LeRobot XVLA 推理进程。

## 8. 完整数据与控制 Pipeline

### 8.1 需要区分的四个频率

当前系统中的 15 Hz、30 Hz、约 5～6 Hz 和 50 Hz 分别描述不同环节：

| 频率 | 含义 | 周期 |
|---|---|---:|
| 15 Hz | 三路 JPEG 图像跨 Wi-Fi 的默认输出频率 | 66.67 ms |
| 30 Hz | 真机 raw 相机采集频率，也是 XVLA chunk 内相邻 action 的时间基准 | 33.33 ms |
| 约 5～6 Hz | 推理约 80 ms 且链路稳定时，生成新 chunk 的名义频率 | 约 170～200 ms |
| 50 Hz | bridge 生成并发布机器人 MIT 关节命令的频率 | 20 ms |

`30 Hz action` 不表示模型每秒运行 30 次。XVLA 每运行一次就预测 32 个未来动作点，每个动作点间隔
33.33 ms，所以完整 chunk 表示约 `32 / 30 = 1.067 s` 的未来轨迹。bridge 利用动作块缓存，不需要
每 33.33 ms 重新运行一次模型。

当前 12/8/3 队列设置下，队列从 12 步下降到 8 步后请求推理。新 chunk 到达时只保留最近 3 步，
所以一次新观测通常在“推理耗时 + 3 个保护步”后影响机器人：

```text
80 ms + 3 / 30 s ≈ 180 ms
```

对应约 5～6 Hz，只是时延正常时的估算，不是固定定时器频率。模型仍一次预测 32 步；15 Hz 只是
输入图像传输频率，不会把 action 的 30 Hz 时间基准改成 15 Hz。若图像暂时不新鲜或网络拥塞，模型
调用间隔会更长。

### 8.2 完整链路

```text
真机三路相机：640×480 RGB raw，30 Hz（只在真机本地消费）
  → JPEG quality 80，限频15 Hz
真机 SDK：/lowstate，1 kHz（约1 ms/帧）
        │
        │ ROS 2 DDS / Wi-Fi，ROS_DOMAIN_ID=16
        ▼
主机 topic_convertor
  /lowstate → /human_lower_state，收到即转换，不重新定频
        │
        ▼
主机 rollout bridge
  保存三路最新图像和最新26维关节状态
  订阅三路CompressedImage并解码为RGB
  检查状态年龄≤200 ms、每路图像年龄≤300 ms
        │
        │ 127.0.0.1:29551 本机 IPC
        ▼
XVLA worker（Python 3.12 / CUDA）
  输入：三路RGB + 右臂实测q(7) + 固定任务文本
  一次推理约72～80 ms
  输出：[32,8] absolute action chunk
        │
        ▼
异步 action scheduler
  action时间基准30 Hz，每步33.33 ms
  最多保留12步=400 ms，剩8步≈267 ms时请求新推理
  保留最近3步、替换更远旧动作、在边界融合4步关节目标
        │
        ▼
50 Hz参考生成器
  相邻30 Hz关节目标线性插值
  → 关节位置裁切
  → 最大速度限制
  → 最大加速度限制
        │
        ▼
/human_lower_command：26维 MITJointCommands，50 Hz
        │
        ▼
主机 topic_convertor：收到即转换
  /human_lower_command → /lowcmd
        │
        │ ROS 2 DDS / Wi-Fi
        ▼
真机 SDK：接收约50 Hz低层命令并控制双臂
```

这里的状态频率和控制频率是两条独立链路：真机 SDK 以 1 kHz 发布 `/lowstate`；`topic_convertor`
收到一帧就转换一帧，本身不主动降频，因此 `/human_lower_state` 的上限跟随实际收到的状态流。rollout
bridge只缓存最新状态，并在自己的50 Hz控制定时器中每20 ms读取一次最新值，用于安全检查、重力补偿和
MIT命令生成。最终 `/human_lower_command` 与 `/lowcmd` 仍为约50 Hz。跨Wi-Fi后主机实际收到的状态频率
可能低于1 kHz，应以 `ros2 topic hz /lowstate` 和 `ros2 topic hz /human_lower_state` 的实测结果为准。

### 8.3 观测内容

bridge 进入 `ROLLOUT` 后才创建三路图像订阅和 worker IPC。每次送入模型的观测为：

```text
observation.images.head   头部RGB图像
observation.images.left   左腕RGB图像
observation.images.right  右腕RGB图像
observation.state         右臂7个实测关节位置，单位rad
task                      固定任务文本
```

三路 JPEG 在 bridge 中解码为 RGB 并分别保存最新帧，不做严格硬件时间同步。worker 无法取得有效观测时，
每 20 ms 重新询问一次。
只有以下条件同时成立才发起新推理：

- `/human_lower_state` 年龄不超过 200 ms；
- 三路图像均存在，且每路接收年龄不超过 300 ms；
- 动作队列不超过 8 步；
- 没有另一次推理正在执行。

### 8.4 XVLA action 的含义

worker 返回 `[32,8]`：

```text
action[0:7]  右臂7关节绝对目标位置，单位rad
action[7]    右侧O6开合值
```

action 是绝对关节位置，不是关节增量、关节速度或末端位姿。bridge 不直接执行全部 32 步，而是根据
12步执行窗口、实际推理时延和动作时间戳选取可用片段。

新 chunk 到达时，调度器仅保留旧队列最靠近当前时刻的 3 步，删除其余远期旧预测。新 chunk 会跳过
相对于本次观测已经过期的模型步，然后补足到 12 步。替换边界的前 4 步对右臂 7 个关节进行线性权重
渐进融合；O6 第 8 维不插值，继续按开合阈值离散处理。这使新观测约在 3 个保护步后生效，同时避免
直接清空整个队列造成关节目标跳变。

### 8.5 30 Hz action 到 50 Hz MIT参考

模型动作点间隔33.33 ms，而机器人命令间隔20 ms。bridge在每个50 Hz控制周期中，在前后两条已排程
的30 Hz绝对关节目标之间做线性插值，然后执行安全限制。

当前右臂限制为：

```yaml
right_max_velocity_rad_s: [0.60, 0.70, 0.70, 1.00, 0.70, 0.80, 0.70]
right_max_acceleration_rad_s2: [1.40, 1.80, 1.80, 2.40, 1.40, 2.00, 1.40]
```

每个 20 ms 控制周期允许的参考位移/速度变化按各关节数组分别计算。例如第 1 关节最大位移为
`0.60×0.02=0.012 rad`，第 4 关节最大位移为 `1.00×0.02=0.020 rad`。

如果没有下一条已排程action、队列耗尽或action超过500 ms没有更新，参考生成器保持当前 `q_ref` 并
将内部参考速度清零，不会外推未经模型确认的轨迹。

### 8.6 发送给真机的26维命令

bridge每20 ms发布一次 `/human_lower_command`：

| 电机范围 | 控制内容 |
|---|---|
| 腿部0～11 | `pos=实测位置`，`kp=0`，`kd=0`，不参与rollout控制 |
| 左臂12～18 | 始终保持left home，`kp=40`，`kd=2` |
| 右臂19～25 | 50 Hz平滑后的右臂 `q_ref`，`kp=40`，`kd=2` |

rollout期间每个20 ms周期都使用双臂实测关节角，通过Pinocchio计算14维重力补偿前馈并写入
`eff`。`topic_convertor`随后把MIT字段映射为 `/lowcmd` 的 `q/dq/tau/kp/kd`，转换节点没有自己的
周期，因此输出频率跟随输入，约为50 Hz。

O6不按50 Hz重复发送。模型第8维满足 `≥0.70` 时闭合，满足 `≤0.05` 时张开，中间区域保持原状态；
只有开合状态发生变化时才发布一次 `/handscmd`。左侧O6在rollout中保持张开。

### 8.7 一次稳态补充动作块的例子

假设当前动作队列刚好剩8步，三路图像和状态均满足时效要求：

```text
t=0 ms
bridge锁存三路最新解码RGB、右臂q(7)和任务文本，发给worker。
队列还可继续执行约267 ms。

t=0～77 ms
XVLA在GPU上生成32步动作；机器人并未等待，仍执行旧队列。

t≈77 ms
新chunk返回。旧队列通常还剩5～6步。bridge只保护其中最近3步，删除其余旧动作；随后跳过新chunk中
已经过期的前部动作，在边界融合4步，并把队列补足到12步。

t≈170～200 ms
3个保护步执行完成，新chunk中融合后的动作开始影响右臂。

全过程
30 Hz动作点每33.33 ms推进一次；bridge每20 ms插值、限位、计算重力补偿并发布MIT命令。
```

如果某一路图像年龄超过300 ms，worker会每20 ms重试但不会发起推理；此时队列继续消耗。若队列最终
耗尽，50 Hz参考生成器冻结当前位置，待有效图像和新chunk恢复后再从零参考速度继续运动。
