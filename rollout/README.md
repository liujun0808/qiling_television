# XVLA Worker 目录

此目录承载与 ROS2 隔离的 Python 3.12 / `lerobot051` XVLA 推理侧。完整操作步骤见根目录
[`ROLLOUT_WORKFLOW.md`](../ROLLOUT_WORKFLOW.md)。

## 架构职责

```text
qiling_rollout_ros bridge（系统 Python 3.10 / ROS2）
  最新三路 RGB + 右臂 q
        ⇅ localhost authenticated IPC
xvla_worker.py（lerobot051 / Python 3.12）
  XVLA 推理、预处理、后处理
        ↓ action [right q(7), O6(1)]
qiling_rollout_ros bridge
```

worker 不导入 ROS2。它只连接本机 IPC，加载训练 checkpoint 旁保存的 pre/postprocessor，使图像重命名、
RGB 预处理和 action 反归一化与训练保持一致。

## 文件

- `config/xvla_rollout.yaml`：checkpoint、任务文本、GPU、HF cache 和 IPC 参数。
- `worker/xvla_worker.py`：持续获取最新观测并执行 XVLA 推理；bridge 返回 `finished` 后正常退出。
- `scripts/start_xvla_worker.sh`：通过 `lerobot051` 环境启动 worker。
- `records/`：ROS bridge 写入的 JSONL 运行日志；默认不保存图像。

## 接口约束

- IPC 仅可绑定 `127.0.0.1`/`localhost`/`::1`。
- 输入观测：head、left、right 三路 RGB 和右臂关节角 `q(7)`。
- 输出：`[N, 8]` action chunk，每行为右臂目标关节角 `q_target(7)` 与 O6 二值控制值 `1`。
- checkpoint 必须是 LeRobot `pretrained_model` 目录，并包含相应 processors。
