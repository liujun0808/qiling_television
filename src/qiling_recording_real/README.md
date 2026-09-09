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

当前模式是**完整 episode 一对一转换**：每条通过结构检查的原始 episode 对应一条
LeRobot episode，不抽取可用片段，不因图像时间间隔而拆分 episode。

```text
成功 episode MCAP
  ↓ build_training_admission_manifest.py（结构检查，生成完整 episode 清单）
  ↓ export_manifest_to_intermediate.py（ROS Python 3.10）
intermediate：JPEG + q/dq/q_target/O6 标签
  ↓ pack_intermediate_to_lerobot.py（lerobot051 Python 3.12）
LeRobot v3 数据集
```

`export_manifest_to_intermediate.py` 是唯一读取 MCAP/ROS bag 的转换阶段；
`pack_intermediate_to_lerobot.py` 是唯一依赖 LeRobot 的打包阶段。旧的历史双臂直转脚本与已废弃的
人工 review/promotion 脚本已删除，避免被误用于当前右臂 + O6 数据结构。

以头部相机第一帧到最后一帧为时间范围，保留头部帧，其他相机、状态与 action 按最近时间戳
对齐。当前导出器的 `--camera-tolerance-sec` 和 `--state-tolerance-sec` 仅为兼容参数，不执行
超差丢帧。最近帧对齐不能修复源数据丢帧。缺必需 topic、空流、数据损坏等结构问题仍会拒绝整条
episode。JSON 中沿用 `training_segments` / `segment_id` 字段名，每项实际代表完整 episode。

### 选择模式：从指定 episode 开始连续 N 条

“往下 20 个”定义为：按目录名中的录制时间升序排列，从指定 episode **本身开始计数**，取连续
20 条；不是按文件管理器当前显示顺序或文件修改时间排列。时间不连续、跨录制批次也照常计数。

下面的选择命令只创建软链接，不复制或修改原始 MCAP。仅选择正式的 `episode_YYYYMMDD_HHMMSS_mmm`
目录，不选择 `.pending`。起点不存在、不足指定条数或选择内目录缺失元数据时直接报错，不会悄悄
少选或用后面的条目补位。

先在主机终端设置本次转换参数（后续命令在同一终端按顺序执行）：

```bash
cd /home/ub/program/qiling_television
# 在系统 ROS Python 环境中执行前两阶段，勿使用 Conda Python 读取 MCAP。
source /opt/ros/humble/setup.bash
source install/setup.bash

export CONVERT_START=episode_20260904_214028_679
export CONVERT_COUNT=20
# 每次转换使用新的工作目录与数据集目录，避免覆盖先前结果。
export CONVERT_WORK="$PWD/conversion_runs/from_214028_20"
export CONVERT_OUTPUT="$PWD/lerobot_dataset_from_214028_20"
export CONVERT_REPO_ID=local/qiling_right_arm_o6_from_214028_20
```

创建选择目录并打印所选的 20 条名单：

```bash
/usr/bin/python3 - <<'PY'
import os
import re
from pathlib import Path

root = Path('recordings').resolve()
start = os.environ['CONVERT_START']
count = int(os.environ['CONVERT_COUNT'])
work = Path(os.environ['CONVERT_WORK'])
if count <= 0:
    raise SystemExit('CONVERT_COUNT 必须大于 0')
episodes = sorted(
    (p for p in root.iterdir()
     if p.is_dir() and re.fullmatch(r'episode_\d{8}_\d{6}_\d{3}', p.name)),
    key=lambda p: p.name,
)
names = [p.name for p in episodes]
if start not in names:
    raise SystemExit(f'找不到起始 episode：{start}')
offset = names.index(start)
selected = episodes[offset:offset + count]
if len(selected) != count:
    raise SystemExit(f'从 {start} 起只有 {len(selected)} 条，需要 {count} 条')
for p in selected:
    if not (p / 'session.yaml').is_file() or not (p / 'rosbag').is_dir():
        raise SystemExit(f'episode 结构不完整：{p}')
if work.exists():
    raise SystemExit(f'工作目录已存在，请更换 CONVERT_WORK：{work}')
inputs = work / 'input'
inputs.mkdir(parents=True)
for number, p in enumerate(selected, 1):
    (inputs / p.name).symlink_to(p, target_is_directory=True)
    print(f'{number:02d}. {p.name}')
print(f'已选择 {len(selected)} 条，选择目录：{inputs}')
PY
```

选择失败时先修正参数，不要继续后续步骤。以后选其他批次，只修改 `CONVERT_START`、
`CONVERT_COUNT` 和三个输出标识即可。选择目录创建后名单已经固定，后续新录制的 episode 不会自动加入。

### 第一步：生成完整 episode 清单

```bash
/usr/bin/python3 \
  src/qiling_recording_real/scripts/build_training_admission_manifest.py \
  "$CONVERT_WORK/input" \
  --config-file src/qiling_recording_real/config/real_recording.yaml \
  --output "$CONVERT_WORK/manifest.json"
```

检查生成的 `manifest.md` / `manifest.json`。下面的命令确认指定的 N 条均通过检查，每条只有一个
完整导出项；若不通过，应先处理错误，不能把“选了 20 条”当作“成功转换了 20 条”：

```bash
/usr/bin/python3 - <<'PY'
import json
import os
from pathlib import Path

m = json.loads((Path(os.environ['CONVERT_WORK']) / 'manifest.json').read_text())
n = int(os.environ['CONVERT_COUNT'])
items = m['training_segments']
assert len(items) == n, f'要求 {n} 条，实际通过 {len(items)} 条；请查看 manifest.md'
assert len({s['source_episode'] for s in items}) == n, '检测到重复 episode'
assert all(e['tier'] == 'all_recorded' and len(e['segments']) == 1 for e in m['episodes'])
print(f'确认 {n} 条完整 episode 可以导出')
PY
```

### 第二步：导出 MCAP 为中间文件

```bash
/usr/bin/python3 \
  src/qiling_recording_real/scripts/export_manifest_to_intermediate.py \
  "$CONVERT_WORK/manifest.json" \
  --output-root "$CONVERT_WORK/intermediate"
```

输出为每条 episode 的 JPEG、`data.npz` 和来源信息；原始 recordings 不变。

### 第三步：使用 lerobot051 打包 LeRobot v3

```bash
/home/ub/miniconda3/envs/lerobot051/bin/python \
  src/qiling_recording_real/scripts/pack_intermediate_to_lerobot.py \
  "$CONVERT_WORK/intermediate" \
  --output-root "$CONVERT_OUTPUT" \
  --repo-id "$CONVERT_REPO_ID" \
  --fps 30
```

不需要编译，也不需要启动 SDK、相机、遥操或 rollout。`--repo-id` 用作数据集标识，这条命令
仅写入本地，不上传。两个转换脚本都拒绝覆盖已存在的输出目录；重跑请使用新的输出路径。

最终包含三路视频、右臂 `observation.state(7)`、`observation.velocity(7)` 和
`action(8) = 右臂目标关节角(7) + O6 开闭目标(1)`。检查实际输出条数：

```bash
/usr/bin/python3 - <<'PY'
import json
import os
from pathlib import Path

info = json.loads((Path(os.environ['CONVERT_OUTPUT']) / 'meta/info.json').read_text())
print('episodes:', info['total_episodes'], 'frames:', info['total_frames'], 'fps:', info['fps'])
assert info['total_episodes'] == int(os.environ['CONVERT_COUNT'])
PY
```

### 全量模式：转换 `recordings/` 中全部完整 episode

先停止录制，且在下列命令执行期间不要新建 episode。本模式直接扫描 `recordings/` 下所有名称符合
`episode_YYYYMMDD_HHMMSS_mmm` 的目录，不创建软链接；每条原始 episode 仍是一条 LeRobot
episode。它不会覆盖前述“从指定 episode 开始”的转换结果。

“全部”指当前右臂 + O6 数据格式下所有**成功且结构完整**的 episode。若存在未保存、失败、缺少
必需 topic、JPEG 损坏或旧数据格式的条目，清单检查会在核对步骤中明确报出并停止，避免静默漏转。
先修复或移走这些无效条目，再重新执行全量转换。

```bash
cd /home/ub/program/qiling_television
source /opt/ros/humble/setup.bash
source install/setup.bash

# 改为新的名字即可重做一次全量转换；不要使用已经存在的目录。
export ALL_CONVERT_WORK="$PWD/conversion_runs/all_episodes_v1"
export ALL_CONVERT_OUTPUT="$PWD/lerobot_dataset_all_episodes_v1"
export ALL_CONVERT_REPO_ID=local/qiling_right_arm_o6_all_episodes_v1

if [ -e "$ALL_CONVERT_WORK" ] || [ -e "$ALL_CONVERT_OUTPUT" ]; then
  echo '全量工作目录或输出数据集目录已存在；请改用新的 *_vN 名字。' >&2
  exit 1
fi
mkdir -p "$ALL_CONVERT_WORK"

# 仅计数正式 episode 目录；.pending 和其他辅助文件不会参与转换。
export ALL_SOURCE_COUNT="$(/usr/bin/python3 - <<'PY'
import re
from pathlib import Path

pattern = re.compile(r'episode_\d{8}_\d{6}_\d{3}')
root = Path('recordings')
print(sum(p.is_dir() and pattern.fullmatch(p.name) is not None for p in root.iterdir()))
PY
)"
if [ "$ALL_SOURCE_COUNT" -le 0 ]; then
  echo 'recordings/ 下没有可转换的正式 episode。' >&2
  exit 1
fi
echo "本次将检查并转换 $ALL_SOURCE_COUNT 条 episode"
```

第一步生成全量清单：

```bash
/usr/bin/python3 \
  src/qiling_recording_real/scripts/build_training_admission_manifest.py \
  recordings \
  --config-file src/qiling_recording_real/config/real_recording.yaml \
  --output "$ALL_CONVERT_WORK/manifest.json"
```

必须进行下面的核对。它要求 `recordings/` 中的每条正式 episode 都恰好生成一条完整导出项；若打印
拒绝条目，先查看 `"$ALL_CONVERT_WORK/manifest.md"` 的原因并处理，不要直接继续打包：

```bash
/usr/bin/python3 - <<'PY'
import json
import os
from pathlib import Path

manifest = json.loads((Path(os.environ['ALL_CONVERT_WORK']) / 'manifest.json').read_text())
expected = int(os.environ['ALL_SOURCE_COUNT'])
episodes = manifest['episodes']
segments = manifest['training_segments']
rejected = [
    f"{item['source_episode']}: {', '.join(item.get('reasons', []))}"
    for item in episodes if item['tier'] != 'all_recorded'
]
if rejected:
    raise SystemExit(
        '以下 episode 不能按当前完整 episode 规则转换：\n- '
        + '\n- '.join(rejected)
        + '\n请查看 manifest.md 后处理，再重新运行。')
assert len(episodes) == expected, (len(episodes), expected)
assert len(segments) == expected, (len(segments), expected)
assert len({item['source_episode'] for item in segments}) == expected
assert all(item['tier'] == 'all_recorded' and len(item['segments']) == 1 for item in episodes)
print(f'确认 {expected} 条完整 episode 均可导出')
PY
```

第二步导出 MCAP，第三步打包为 LeRobot v3：

```bash
/usr/bin/python3 \
  src/qiling_recording_real/scripts/export_manifest_to_intermediate.py \
  "$ALL_CONVERT_WORK/manifest.json" \
  --output-root "$ALL_CONVERT_WORK/intermediate"

/home/ub/miniconda3/envs/lerobot051/bin/python \
  src/qiling_recording_real/scripts/pack_intermediate_to_lerobot.py \
  "$ALL_CONVERT_WORK/intermediate" \
  --output-root "$ALL_CONVERT_OUTPUT" \
  --repo-id "$ALL_CONVERT_REPO_ID" \
  --fps 30
```

完成后检查数据集实际条数：

```bash
/usr/bin/python3 - <<'PY'
import json
import os
from pathlib import Path

info = json.loads((Path(os.environ['ALL_CONVERT_OUTPUT']) / 'meta/info.json').read_text())
expected = int(os.environ['ALL_SOURCE_COUNT'])
print('episodes:', info['total_episodes'], 'frames:', info['total_frames'], 'fps:', info['fps'])
assert info['total_episodes'] == expected, (info['total_episodes'], expected)
PY
```

### 其他选择方式

- 指定单条：使用相同流程，将 `CONVERT_START` 设为该条名字、`CONVERT_COUNT=1`。
- 指定连续多条：使用上面的起始名字 + 条数模式。
- 全量：使用上面的“全量模式”整套命令；它会动态计数并要求每条源 episode 都被完整转换。
- 不连续的几条：在新的 `CONVERT_WORK/input` 里仅放入所需 episode 的绝对路径软链接，再执行
  三个阶段；将 `CONVERT_COUNT` 改为实际选择数量。

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
