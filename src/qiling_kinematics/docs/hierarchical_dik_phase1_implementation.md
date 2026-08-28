# Hierarchical DIK Phase 1 实现记录

状态：已完成  
完成日期：2026-08-27

## 1. 本阶段范围

Phase 1 只完成控制核心拆分和运行状态安全修复。求解器内部仍使用旧版 LOCAL Jacobian + log6 的平级 6D 加权 QP，尚未进入严格位置优先数学重构。

## 2. 新增核心组件

- HierarchicalDIKSolver：把旧版单臂 7 变量 ProxQP 从 ROS timer callback 中抽离。
- ArmControlState：每侧独立维护 HOLD / ACTIVE / FAULT、q_reference、qdot_previous 和 release-required latch。
- IkDiagnostics：保存每侧状态、solver status、数据年龄、误差、qdot 和求解时间。
- ik_types.hpp：统一 7DoF 向量、6×7 Jacobian 和状态枚举。

## 3. 已修复问题

1. 所有 qdot 默认初始化为零。
2. HOLD 状态禁止 reference integration。
3. Grip 释放时 q_reference 重置到本侧 q_measured，qdot_previous 清零。
4. Target 或 JointState 超时后要求释放 Grip 才能重新进入 ACTIVE。
5. 左右 target、mode、JointState freshness 和 solver failure 独立处理。
6. 任一侧缺少 target 不再阻止另一侧求解。
7. 任一侧超时不再重置另一侧 reference。
8. JointState 每侧必须在同一条消息中包含完整 7 个 finite 关节位置才刷新。
9. 节点在两侧都获得过完整 7 关节状态前不发布第一条 40D 命令。
10. QP 失败返回零 qdot，禁止沿用上一周期速度。
11. 连续 QP 失败达到阈值后，本侧进入 latch FAULT。
12. steady-clock 实测控制周期用于积分，并对 dt 进行裁切；严重 stall 进入 HOLD。
13. ProxQP 结果在发布前再次显式裁切到 box bounds，消除求解容差造成的微小越界。
14. 机器人 wrist state 只在对应 JointState fresh 时发布，避免 bridge 把 stale FK 当成新状态。

## 4. 保持不变

- ROS 2 话题名称。
- 40D MITJointCommands 格式和双臂索引。
- Pinocchio 模型 s4_dual_arm.urdf。
- 左右 wrist frame。
- 旧版 6D QP 的权重、阻尼和 posture 公式。
- 50 Hz 控制频率。
- XR bridge 和 PXREA adapter。
- O6 槽位仍保持不生成。

## 5. 测试

新增 ArmControlState 测试：

- HOLD 不积分。
- Grip release 重置 reference 和 velocity。
- Timeout 后必须 release 才能重新 ACTIVE。
- 连续 QP failure latch FAULT。

新增 HierarchicalDIKSolver 测试：

- 零误差输出零速度。
- 非零误差输出有限且受 box 限制的速度。
- 非法 dt 安全失败并返回零速度。
- 非法关节边界安全失败并返回零速度。
- NaN 输入安全失败并返回零速度。

构建和测试结果：

~~~text
qiling_kinematics build: passed
GTest cases: 9 passed
colcon aggregate: 11 tests, 0 errors, 0 failures
node startup smoke test: passed
~~~

## 6. 已知保留问题

当前求解器仍是 Phase 1 的 legacy 路径：

- 仍使用 Pinocchio LOCAL Jacobian。
- 仍使用 log6(current.inverse() * target)。
- Position 与 orientation 仍在一个加权 6D QP 中。
- 仍使用旧 joint-limit margin 一步速度边界。
- 尚未加入 acceleration bounds。
- 尚未加入 bounded q_ref gap。
- 尚未加入 LWA finite-difference 验证。
- 尚未加入 elbow swivel。

这些问题将在后续阶段逐项替换，不应在 Phase 1 中通过调参掩盖。

## 7. 下一阶段

Phase 2：运动学与 Jacobian 验证。

- 将 FK/Jacobian 检查放入可重复测试。
- 验证 wrist 与 elbow frame。
- 验证 LOCAL_WORLD_ALIGNED analytic Jacobian。
- 用有限差分检查左右各 7 列线速度和角速度。
- 对比 Pinocchio 与 MuJoCo 的 wrist FK。
- Phase 2 只证明数学坐标一致，不提前实现严格位置 QP。

