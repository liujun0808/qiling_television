# Hierarchical DIK Phase 4 实现记录

状态：已完成  
完成日期：2026-08-27

## 1. 本阶段范围

Phase 4 在已经验收的 Level 1 wrist-position QP 后加入严格 Level 2 wrist-orientation QP。

冻结的优先级为：

```text
wrist position > wrist orientation
```

Orientation 只能使用 `null(J_position)`，不能重新与 position 放进同一个加权 6D 目标。Elbow、本体姿态和 O6 不在本阶段求解。

## 2. 姿态误差与速度目标

当前 wrist FK 和 Jacobian 均使用 Pinocchio `LOCAL_WORLD_ALIGNED`，线速度和角速度方向基均为 `base_link`。姿态误差为：

```text
R_error = R_target * R_current^T
e_orientation = Log3(R_error)
omega_desired = norm_saturate(
    rotation_gain * e_orientation,
    max_angular_velocity_rps)
```

这避免把局部 wrist 轴误差直接混入 base_link 方向基。

## 3. 严格 null-space 层级

位置主任务先输出 `qdot_position`。对 3×7 的位置 Jacobian 做完整 SVD：

```text
J_position = U * S * V^T
N_position = V.rightCols(4)
```

第一版仅在 `rank(J_position) == 3` 时启用姿态层。姿态 QP 的变量为 4 维 `z_orientation`：

```text
qdot_pose = qdot_position + N_position * z_orientation
J_orientation_reduced = J_angular * N_position
```

目标函数为：

```text
min 0.5 * ||J_orientation_reduced * z_orientation
           - (omega_desired - J_angular * qdot_position)||^2
  + 0.5 * orientation_regularization * ||z_orientation||^2
  + 0.5 * orientation_smoothness_weight
          * ||qdot_pose - qdot_previous||^2
```

姿态层不是再次求完整 7 维 qdot，因此其数学结构不能主动改变位置主任务。

## 4. 最终边界与验收门

Phase 3 根据每关节速度限制、measured q、dt、软限位 margin 得到最终速度边界：

```text
qdot_lower <= qdot_pose <= qdot_upper
```

在 reduced QP 中转换为：

```text
qdot_lower - qdot_position
<= N_position * z_orientation
<= qdot_upper - qdot_position
```

求解后仍执行 finite、最终 box bound 和位置退化检查：

```text
||J_position * (qdot_pose - qdot_position)||
<= max_orientation_position_degradation_mps
```

默认允许阈值为 `0.0005 m/s`，主要用于拒绝数值异常，不用于允许姿态层有意牺牲位置。

## 5. 降级与故障语义

以下情况只关闭当前周期的 orientation 增量：

- `rank(J_position) != 3`；
- orientation error 或 angular Jacobian 非 finite；
- orientation reduced QP 未成功；
- 最终 qdot 越界；
- 位置退化检查失败。

这些情况下输出仍为已经成功的 `qdot_position`，该侧状态机保持 ACTIVE，不累计 primary-QP fault。只有位置主任务失败，才沿用 Phase 1 的连续失败计数和 HOLD/FAULT 语义。

日志分别输出 primary solver 状态、orientation solver 状态、orientation 是否应用以及位置退化量，便于区分“整臂停止”和“姿态层安全降级”。

## 6. 当前参数

```yaml
orientation_task_enabled: true
rotation_gain: 3.0
max_angular_velocity_rps: 0.80
orientation_regularization: 0.0001
orientation_smoothness_weight: 0.001
rank_threshold: 0.0001
max_orientation_position_degradation_mps: 0.0005
```

`orientation_task_enabled: false` 可显式回到 Phase 3 position-only 路径，用于 A/B 排障。

## 7. 测试覆盖

合成 Jacobian 测试覆盖：

- base_link X/Y/Z 三个姿态轴均可在位置 null-space 中跟踪；
- 姿态增量不改变 position primary；
- orientation target 不变时，姿态层可抵消 position primary 引入的 wrist 角速度；
- 角速度向量范数限制生效；
- 最终每关节速度和一步位置边界生效；
- position Jacobian 降秩时回退 position-only；
- 非 finite 姿态输入时回退 position-only。

真实 S4 运动学测试覆盖：

- 使用 teleop home 构型；
- 左右臂各自验证 base_link X/Y/Z 旋转方向；
- 左右臂各自验证 base_link X/Y/Z 平移时保持 wrist orientation；
- 验证 orientation 被实际应用；
- 验证姿态增量位于真实 wrist-position Jacobian 的 null-space。

完整回归结果：

```text
8 test programs
39 tests
0 errors
0 failures
```

## 8. 下一阶段

Phase 5 实现 Level 3 anthropomorphic elbow swivel：

- 构造带 characteristic length 的完整 wrist Jacobian；
- 检查完整 wrist rank 和最小奇异值；
- 仅在完整 wrist null-space 中加入 elbow swivel；
- 接近奇异构型时连续淡出，rank 小于 6 时完全关闭；
- 同一剩余冗余方向内加入弱 rest posture、joint centering 和 smoothness；
- 分别验证对 wrist position 和 orientation 的退化量，失败时回退到 Phase 4 的 `qdot_pose`。
