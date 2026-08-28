# Hierarchical DIK Phase 5.2 实现记录

状态：已完成

## 1. 本阶段目标

将 Phase 5.0/5.1 的 anthropomorphic elbow 几何结果接入差分 IK，但不允许它重新引入此前已经修复的“姿态影响位置”问题。

因此本阶段的严格优先级为：

```text
Level 1: wrist position
Level 2: wrist orientation in position null-space
Level 3: elbow swivel / rest posture / joint centering in full wrist null-space
```

对于每条 7 DoF 手臂，完整 wrist Jacobian 为 characteristic-length 缩放后的 `6×7` 矩阵。满秩时它最多留下一个冗余自由度，Phase 5.2 只使用这个 1D 方向。

## 2. 实际求解结构

前两层保持 Phase 4 的求解方式：

```text
qdot_position = PositionQP(J_position)
qdot_pose     = qdot_position + N_position * z_orientation
```

Phase 5.2 在此基础上计算完整腕部零空间向量 `n_wrist`：

```text
J_wrist = [ J_position          ]
           [ characteristic_len * J_angular ]

J_wrist * n_wrist = 0
qdot_final = qdot_pose + n_wrist * z_elbow
```

`z_elbow` 是一个标量，由单变量 ProxQP 求解。它同时考虑：

- 当前 signed swivel error 对应的 elbow swivel 速度；
- 当前位置相对于 home/rest pose 的弱偏好；
- 关节范围中心的弱偏好；
- 对上一周期速度的 smoothness 偏好；
- ProxQP 正则项。

swivel 误差的局部 Jacobian 由 elbow 和 wrist 的线速度 Jacobian 计算。肩部参考点固定在 `base_link`，wrist 轴的变化和 elbow 投影方向的变化都纳入当前周期的局部线性化；preferred direction 在当前周期冻结，并在下一周期重新计算。

## 3. 安全门与回退

elbow 任务只有同时满足以下条件才会执行：

1. position 层成功；
2. orientation 层成功；
3. elbow geometry 有效，当前肩腕距离和肘部投影半径没有退化；
4. 完整 wrist Jacobian 的 rank 为 6；
5. `wrist_sigma_min > elbow_sigma_disable`；
6. 输入 Jacobian、swivel error、home pose 均为 finite；
7. elbow 单变量 QP 成功并给出有限解。

在 `elbow_sigma_disable < wrist_sigma_min < elbow_sigma_fade_start` 区间内，elbow 速度和弱偏好会线性淡出。这样接近奇异位形时，系统先放弃冗余塑形，再继续保持腕部 position/orientation。

elbow 增量还要通过三重检查：

```text
qdot_lower <= qdot_pose + n_wrist*z_elbow <= qdot_upper
||J_position * delta_elbow|| <= max_elbow_position_degradation
||J_angular  * delta_elbow|| <= max_elbow_orientation_degradation
```

任一检查失败时，本周期只返回 `qdot_pose`，整体控制仍保持有效，不会因为 tertiary task 失败而让手臂进入 FAULT。

## 4. 运行参数

配置位于 `config/differential_ik.yaml`：

```yaml
elbow_task_enabled: true
elbow_swivel_gain: 2.0
max_elbow_swivel_velocity_rps: 0.80
elbow_posture_weight: 0.08
elbow_joint_centering_weight: 0.03
elbow_smoothness_weight: 0.05
elbow_regularization: 0.0001
elbow_sigma_fade_start: 0.08
elbow_sigma_disable: 0.04
max_elbow_position_degradation_mps: 0.0005
max_elbow_orientation_degradation_rps: 0.002
```

建议第一轮 MuJoCo 联调保留 `elbow_task_enabled: true`，观察日志中的：

```text
elbow_status
elbow_applied
elbow_scale
elbow_position_degradation_mps
elbow_orientation_degradation_rps
wrist_rank
wrist_sigma_min
```

若需要隔离验证 Phase 4 pose 控制，可临时将 `elbow_task_enabled` 设为 `false`；这只关闭 Level 3，不改变 position/orientation 两层。

启动日志应显示：

```text
Differential IK Phase 5.2 ready ... elbow control=ON
```

## 5. 测试覆盖

新增或更新的测试验证：

- 合成 7 DoF 模型中 elbow 只沿完整 wrist null-space 运动；
- elbow 增量对 position 和 orientation 的 Jacobian 退化为零；
- 完整 wrist rank 不足时 elbow 被拒绝并回退；
- elbow swivel Jacobian 对肩腕轴周向肘部运动的符号和数值正确；
- 原有 position、orientation、限位、Pinocchio/MuJoCo FK 对照测试不受影响。

当前测试结果：

```text
9 test programs
54 tests
0 errors
0 failures
```

## 6. 本阶段的能力边界

该模块仍然不是对操作者真实肘部的跟踪，因为 Quest 当前输入只有左右控制器 6D 位姿和 Grip。它实现的是：

```text
在 wrist target 已经确定后，利用 7 DoF 冗余选择更自然、连续、向外的机器人肘部构型。
```

所以本阶段的验收重点是：腕部目标不被破坏、肘部不突然翻转、接近奇异位形安全降级。真实 Quest 运行时还需要通过日志和 MuJoCo 画面确认 outward 符号、速度和 home 偏好是否符合实际手臂外观；必要时只调整上述权重和淡出阈值，不改变层级结构。

