# Hierarchical DIK Phase 3 实现记录

状态：已完成  
完成日期：2026-08-27

## 1. 本阶段范围

Phase 3 只上线严格 Level 1 wrist-position QP。Orientation 和 elbow 在运行时强制关闭，避免在位置主任务尚未独立验收前重新引入耦合。

## 2. Position-primary 数学

全部量沿 `base_link` 轴表达：

~~~text
e_p = p_target - p_current
v_des = norm_saturate(Kp * e_p, max_linear_velocity)
~~~

每侧求解 7 变量 QP：

~~~text
min 0.5 * ||J_position * qdot - v_des||^2
  + 0.5 * position_regularization * ||qdot||^2
  + 0.5 * position_smoothness_weight * ||qdot - qdot_previous||^2
~~~

约束：

- 每关节独立 `max_joint_velocity_rps`。
- 基于 measured q、dt、软限位 margin 计算的一步位置 box bound。
- 非 finite 输入、非法边界和 QP failure 都返回零 qdot。
- ProxQP 输出再次显式裁切到 box bound。

不可达或大跳变目标不会按位置误差大小无限增速，期望笛卡尔速度的向量范数始终不超过 `max_linear_velocity_mps`。

## 3. 运行节点迁移

`differential_ik_node` 已迁移到 Phase 2 的 `DualArmKinematics`：

- 删除节点内部重复的 Pinocchio model/data、frame id 和 q/v index 管理。
- JointState 继续严格按名字映射左右各 7 个关节。
- 每周期只执行一次双臂 FK/Jacobian 更新。
- wrist state 和 QP 使用同一份 LWA 运动学结果。
- position error 使用 target/current 在 base_link 下的平移差。
- 运行时调用 `solvePositionPrimary()`，不再调用 `solveLegacyFullPose()`。
- target orientation 仍被接收并用于 diagnostics，但不进入 QP。
- `PositionInput` 只暴露 3×7 Jacobian，在接口层面不存在 angular Jacobian 通道。

Phase 1 legacy solver 暂时保留用于回归测试，不再位于控制运行路径。

## 4. Phase 3 参数

当前运行配置：

~~~yaml
orientation_task_enabled: false
position_gain: 3.0
max_linear_velocity_mps: 0.20
position_regularization: 0.0001
position_smoothness_weight: 0.001
joint_limit_margin_rad: 0.08
max_joint_velocity_rps: [1.0, 1.0, 1.0, 1.2, 1.2, 1.2, 1.2]
~~~

`orientation_task_enabled=true` 在 Phase 3 也只会产生警告，运行节点仍强制 position-only。

## 5. 新增测试

Position-primary solver：

- 零误差输出零速度。
- base_link 正负 X/Y/Z 单轴方向正确。
- 非目标轴速度接近零。
- 不可达目标的笛卡尔速度按向量范数限幅。
- 每关节不同速度限位均生效。
- 一步关节位置边界阻止继续向限位外运动。
- 非法边界安全失败并输出零速度。
- NaN 输入安全失败并输出零速度。

S4 真实运动学联合测试：

- 使用 home 构型的真实左右 wrist LWA Jacobian。
- 左右手臂分别验证 base_link X/Y/Z 平移方向。
- 验证目标轴速度为正且交叉轴速度接近零。

## 6. 本阶段预期现象

由于 orientation 被明确关闭：

- 平移时 wrist orientation 可能随最小范数关节解发生变化。
- 旋转 Quest 手柄不会驱动机器人 wrist orientation。
- 此现象不是 Phase 3 的失败；本阶段只验收位置方向、稳定性、限速和安全边界。

最终要求的“位置严格高于姿态”将在 Phase 4 把 orientation 放入 `null(J_position)` 后实现。

## 7. 下一阶段

Phase 4：严格 Level 2 wrist-orientation null-space QP。

- 计算 `N_position = null(J_position)`。
- 使用 `R_target * R_current^T` 的 base_link `Log3` 姿态误差。
- 只在 `N_position` 中求解 orientation 增量。
- 保留最终速度 box bounds。
- 求解后检查 `J_position * (qdot_pose - qdot_position)`。
- 超过位置退化阈值或 secondary QP 失败时回退到 position-only 结果。
- elbow 仍保持关闭。
