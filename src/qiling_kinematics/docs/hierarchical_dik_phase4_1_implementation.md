# Hierarchical DIK Phase 4.1 安全加固记录

状态：已完成  
完成日期：2026-08-27

## 1. 修复目标

Phase 4 实机式 MuJoCo 测试出现：

```text
position-primary QP INVALID_BOUNDS
```

并且释放、重新按下 Grip 后仍立即进入 FAULT。根因不是 orientation null-space QP，而是位置主任务把：

```text
q_min + margin
q_max - margin
```

错误地作为必须在一个周期内返回的硬边界。当 measured q 进入 margin 较深且单周期最大速度不足以返回时，计算得到 `lower > upper`。

这与 Phase 0 冻结契约冲突。冻结语义规定 URDF limit 是硬边界，margin 只是 velocity damper influence zone。

## 2. 新的硬边界

每个关节先使用 URDF 硬限位构造一步预测约束：

```text
lower_hard = (q_min - q_measured) / dt
upper_hard = (q_max - q_measured) / dt

lower = max(-velocity_max, lower_hard)
upper = min(+velocity_max, upper_hard)
```

只要 measured q 位于 URDF 硬范围内，零速度总是可行，因此不会因为进入软 margin 而产生不可行 QP。

`hard_limit_tolerance_rad` 默认是 `0.005 rad`。容差内的小幅 MuJoCo 超调仍构造指向范围内部的一步恢复边界；超过容差才返回 `INVALID_BOUNDS` 并沿用原有 FAULT 语义。

## 3. 连续软限位阻尼

定义到下、上硬限位的距离：

```text
d_lower = q_measured - q_min
d_upper = q_max - q_measured
```

margin 内的朝外速度比例为：

```text
scale = clamp(
    joint_limit_damper_gain * distance / joint_limit_margin_rad,
    0,
    1)
```

对应朝外速度上限从 margin 外的完整速度连续降到硬限位处的零。相反方向的恢复速度不被 damper 关闭。

因此：

- 刚进入 margin 不会发生速度跳变；
- 深入 margin 不会产生 `lower > upper`；
- 到达硬限位后不能继续向外；
- 到达或轻微越过硬限位后仍可向内恢复。

## 4. SVD 诊断

Position 诊断直接对 3×7 Jacobian 计算：

```text
rank(J_position)
sigma_position_min
sigma_position_max
condition_position
```

完整 wrist 诊断使用 characteristic-length scaling：

```text
J_wrist_scaled = [J_position; characteristic_length_m * J_angular]
```

记录：

```text
rank(J_wrist_scaled)
sigma_wrist_min
sigma_wrist_max
condition_wrist
```

默认参数：

```yaml
characteristic_length_m: 0.25
position_sigma_warn: 0.05
wrist_sigma_warn: 0.08
rank_threshold: 0.0001
```

Phase 4.1 只输出诊断和节流告警，不根据奇异值改变目标速度。后续 Phase 5 使用 `sigma_wrist_min` 控制 elbow 任务淡出。

## 5. 日志解释

正常日志示例字段：

```text
damper=false
sigma_position=...
sigma_wrist=...
```

进入任一关节 margin 后：

```text
damper=true
limit_distance=...
```

`limit_distance` 是七个关节中最近的 URDF 硬限位距离。接近零表示已非常靠近硬限位；小于零但没有超过 `-hard_limit_tolerance_rad` 表示处于允许恢复的小幅超调区。

靠近奇异构型时会输出：

```text
near singularity: position rank=... sigma_min=... cond=...;
wrist rank=... sigma_min=... cond=...
```

其中 full-wrist sigma 低而 position sigma 正常，通常表示仍能保持位置，但姿态或下一阶段 elbow 的可用空间正在退化。

## 6. 测试覆盖

新增测试验证：

- soft margin 内 QP 始终可行；
- margin 内朝外速度连续降低；
- 硬限位处禁止继续向外；
- 硬限位处保留向内恢复速度；
- 容差内小幅硬限位超调不会返回 `INVALID_BOUNDS`；
- 明显越过 URDF 硬限位仍安全失败；
- position Jacobian rank、奇异值和条件数正确；
- 真实 S4 左右臂完整 wrist rank 和最小奇异值有效。

完整回归：

```text
8 test programs
43 tests
0 errors
0 failures
```

## 7. 下一阶段

Phase 5 开始实现 anthropomorphic elbow swivel，但仍按小步上线：

1. 先实现 shoulder-elbow-wrist 几何和 home swivel reference 的只读诊断。
2. 验证左右 outward/downward 符号、连续性和退化判定。
3. 再构造完整 wrist 的 1D null-space elbow QP。
4. 使用 `sigma_wrist_min` 在接近奇异构型时连续淡出，rank 小于 6 时完全关闭。
5. 检查 elbow 增量对 wrist position 和 orientation 的退化，失败时回退 Phase 4 pose 结果。
