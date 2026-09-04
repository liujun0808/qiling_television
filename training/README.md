# Qiling XVLA 训练

训练数据为 `lerobot_dataset_v2`：132 个连续轨迹、20,734 帧、三路 RGB、7 维右臂状态/速度和 8 维动作。

所有参数位于 `configs/xvla_full_finetune.yaml`，并附中文注释。启动器强制使用 `lerobot051` 与 `dataset.video_backend=pyav`；后者是必要设置，默认 TorchCodec 当前无法解码视频。

先检查训练命令、GPU、数据集路径而不启动训练：

```bash
cd /home/ub/program/qiling_television
bash training/scripts/train_xvla.sh --dry-run
```

启动全量微调：

```bash
cd /home/ub/program/qiling_television
bash training/scripts/train_xvla.sh
```

产物将写入 YAML 中的 `training.output_dir`。该目录已存在时启动器会停止，防止覆盖已有实验。

`xvla.freeze_vision_encoder=false`、`freeze_language_encoder=false`、`train_policy_transformer=true`、`train_soft_prompts=true` 共同表示全量微调；XVLA 优化器自动对 VLM 使用 1/10 学习率。
