# Hierarchical DIK Phase 2 实现记录

状态：已完成  
完成日期：2026-08-27

## 1. 本阶段范围

Phase 2 建立可重复的运动学和坐标系验证基线，不改变运行节点当前的 Phase 1 legacy 6D QP。严格位置优先 QP 从 Phase 3 开始实现。

## 2. 可复用 Pinocchio 运动学层

新增 `DualArmKinematics`：

- 固定加载 `s4_dual_arm.urdf` 的 14 个可动关节。
- 按 Phase 0 契约固定左右各 7 个关节顺序。
- 固定 shoulder frame 为 shoulder-pitch link 原点。
- 固定 elbow frame 为 elbow link 原点。
- 固定 wrist frame 为 O6 hand-base link。
- 输出左右 wrist FK、elbow FK 和 shoulder FK。
- 输出 `LOCAL_WORLD_ALIGNED` wrist 6×7 Jacobian。
- 输出 `LOCAL_WORLD_ALIGNED` elbow 线速度 3×7 Jacobian。
- 输出按手臂顺序提取的关节上下限。
- 非 finite 构型直接拒绝，不进入运动学计算。

该类已加入 `qiling_dik_core`，Phase 3 会用它替换运行节点中重复的模型索引和 legacy LOCAL Jacobian 代码。

## 3. Jacobian 有限差分验证

左右手臂都在以下构型验证全部 7 列：

1. `teleop_home`。
2. 每个关节均非零变化的确定性扰动构型。

验证内容：

- wrist position 对关节的中心有限差分与 Jacobian 前 3 行一致。
- wrist world angular velocity 的中心有限差分与 Jacobian 后 3 行一致。
- elbow position 对关节的中心有限差分与 elbow Jacobian 一致。
- 最大逐元素误差阈值为 `2e-6`。
- shoulder reference position 在手臂运动时固定为 base_link 中的设计原点。

以上测试证明 Pinocchio Motion/Jacobian 在本项目中的行顺序是 linear 后 angular，并证明后续不能再混用 LOCAL Jacobian 与 base_link 误差。

## 4. MuJoCo 与 Pinocchio 模型一致性

新增回归测试，直接加载真实 MuJoCo scene，并把同一组左右 7DoF 关节数值写入 MuJoCo 和 Pinocchio。测试在 `teleop_home` 与扰动构型比较：

- left/right elbow body pose。
- left/right O6 hand-base body pose。
- MuJoCo world pose 先转换到 MuJoCo `base_link`，再与 Pinocchio base_link pose 对比。

首轮测试发现并量化两处原模型错误：

1. MuJoCo 左右 `shoulder_yaw_joint` 使用 `-Z`，URDF 使用 `+Z`。home 的 elbow orientation 已产生 `0.04 rad` 差异，扰动构型扩大到左 `0.42 rad`、右 `0.28 rad`。
2. MuJoCo 左右 O6 hand mount orientation 与 URDF 固定关节不一致，hand-base orientation 固定相差约 `pi/2`。

MuJoCo 模型现已按作为 IK 权威模型的 URDF 修正：

- 左右 shoulder yaw axis 改为 `0 0 1`。
- 左右 hand-base mount quaternion 改为与 URDF fixed mount 相同的 orientation。

修正后，home 和扰动构型的 4 个 frame 全部通过位置和姿态对比。

## 5. 测试覆盖

Phase 2 新增：

- `test_dual_arm_kinematics`：5 个测试。
- `test_mujoco_pinocchio_fk`：2 个测试。

连同 Phase 1 测试，当前 GTest 共 16 个 case：

- ArmControlState：4。
- legacy HierarchicalDIKSolver：5。
- Pinocchio kinematics/Jacobian：5。
- MuJoCo-Pinocchio FK：2。

## 6. 保持不变

- `differential_ik_node` 运行时仍使用 Phase 1 legacy `LOCAL + log6` 路径。
- 当前 solver 仍把 position 和 orientation 放在同一个加权 6D QP。
- 未加入 elbow swivel、null-space、加速度约束或 bounded q_ref。
- XR、O6、视频和 episode 录制均未改变。

## 7. 下一阶段

Phase 3：实现严格 Level 1 wrist-position QP。

- 运行节点迁移到 `DualArmKinematics` 的 LWA FK/Jacobian。
- 使用 base_link position error。
- 实现 Cartesian linear velocity norm saturation。
- 保留 Level 0 box bounds。
- 此阶段 orientation 和 elbow 均关闭，先单独证明位置控制稳定。
- 增加 X/Y/Z 单轴与不可达目标测试，确认姿态变化不会反向污染位置主任务的定义。
