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
RGB 预处理和 action 反归一化与训练保持一致。bridge 依据自身未来 action 队列的水位才提供新观测；
worker 每次返回完整 XVLA chunk，并携带观测/推理时间戳。bridge 保留尚未执行的旧计划，将新计划按
推理时延对齐后追加到未来队列，因此它不是“新结果立即清空旧 chunk”的逻辑。
worker 默认在连接 bridge 前执行一次合成观测 warm-up，避免首个真实 action 触发 CUDA 初始化延迟。

## 文件

- `config/xvla_rollout.yaml`：checkpoint、任务文本、GPU、HF cache 和 IPC 参数。
- `worker/xvla_worker.py`：仅在 bridge 请求补充队列时推理，回传完整 XVLA chunk；bridge 返回 `finished` 后正常退出。
- `scripts/start_xvla_worker.sh`：通过 `lerobot051` 环境启动 worker。
- `scripts/prepare_benchmark_intermediate.sh`、`scripts/benchmark_xvla_offline.py`：无真机的录制回放与真实
  XVLA 推理时延测量；执行步骤见 `OFFLINE_VALIDATION.md`。
- `records/`：ROS bridge 写入的 JSONL 运行日志；默认不保存图像。

## 接口约束

- IPC 仅可绑定 `127.0.0.1`/`localhost`/`::1`。
- 输入观测：head、left、right 三路 RGB 和右臂关节角 `q(7)`。
- 输出：当前 XVLA 原生 `[32, 8]` action chunk，每行为右臂目标关节角 `q_target(7)` 与 O6 二值控制值 `1`。
- 这是一种时延感知的异步队列执行，不是官方模型级 Real-Time Chunking (RTC)；当前 XVLA 不提供
  LeRobot RTC 所需的策略接口。
- checkpoint 必须是 LeRobot `pretrained_model` 目录，并包含相应 processors。
