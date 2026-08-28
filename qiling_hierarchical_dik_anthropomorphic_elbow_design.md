# Qiling Television：分层差分 IK + 拟人 Elbow Null-Space 方案

> 版本：v1.5.2（Phase 0 冻结；Phase 1～Phase 5.2 已实现）  
> 目标平台：Meta Quest 3 + XRoboToolkit + ROS 2 Humble + Pinocchio + ProxQP + MuJoCo / 真实双臂机器人  
> 控制频率：50 Hz  
> 范围：当前阶段只重构“双臂 IK 与姿态冗余控制层”，不替换 XRoboToolkit、不替换 ROS 2 数据链、不引入 PyRoki、不改变 O6 后续接入方向。
>
> Phase 0 的规范性冻结契约位于 src/qiling_kinematics/docs/hierarchical_dik_phase0_contract.md。若本文后续保留的 v1.0 公式与冻结契约冲突，以冻结契约为准。已确认：位置严格高于姿态；elbow 使用 swivel direction；左右臂独立状态机；MuJoCo 使用 bounded q_ref；真实机器人初期使用 measured step。Phase 1～Phase 5.2 的实现记录位于 `src/qiling_kinematics/docs/hierarchical_dik_phase*_implementation.md`。

---

## 0. 文档目的

当前系统已经完成以下基础链路：

```text
Quest 3 Controller
    ↓
XRoboToolkit / PXREA SDK
    ↓
ROS 2 XR Adapter
    ↓
Grip Clutch + Relative Pose Mapping
    ↓
Left / Right Wrist 6D Target
    ↓
Pinocchio + ProxQP Differential IK
    ↓
40D MITJointCommands
    ↓
MuJoCo / Real Robot
```

现有方案能够形成基本闭环，但“高质量遥操”仍存在几个结构性缺口：

1. 每条机械臂为 7DoF，而 wrist 6D task 只约束 6 个自由度，剩余冗余自由度没有明确任务。
2. `posture_weight = 0` 时，肘部/肩部姿态实际上由数值阻尼、当前构型、关节限位等间接决定，无法保证拟人。
3. position 与 orientation 目前只是一个加权 least-squares task，在奇异位形、速度饱和或任务不可完全满足时会互相牺牲。
4. 当前旋转与平移误差量纲不同，直接加权容易造成姿态/位置竞争。
5. `q_target` 积分状态与实际 `q_measured` 可能长期分离。
6. 目前没有显式 elbow/swivel task、关节加速度限制、奇异性自适应阻尼和系统化任务诊断。

因此，新方案采用：

```text
Level 0：安全硬约束
        ↓
Level 1：Wrist Position
        ↓
Position Null Space
        ↓
Level 2：Wrist Orientation
        ↓
Full Wrist Null Space
        ↓
Level 3：Anthropomorphic Elbow Swivel / Posture / Smoothness
        ↓
Safe qdot
        ↓
Bounded q_ref / q_cmd
```

核心目标是：

> **严格优先保证手腕位置；姿态只能使用位置任务的 null-space；肘部只能使用完整 wrist 任务剩余的 1D null-space。**

---

# 1. 设计边界

## 1.1 保留不变的部分

以下模块原则上不推翻：

- XRoboToolkit PC Service。
- PXREA SDK。
- `qiling_xrobotoolkit_pxrea_adapter`。
- 左右 Grip clutch。
- `/teleop/left_wrist_target`。
- `/teleop/right_wrist_target`。
- `/teleop/left_control_mode`。
- `/teleop/right_control_mode`。
- Pinocchio 运动学模型。
- ProxQP C++ 求解路径。
- 50 Hz 外部控制频率。
- MuJoCo 1 kHz 内部仿真。
- 固定 `base_link`。
- 第一阶段冻结腿部。
- 40 维 `/human_lower_command` 接口。
- 后续 command mux 设计。
- O6 后续单独接入。

## 1.2 本次重点重构

重点修改：

```text
differential_ik_node.cpp
```

建议拆分出：

```text
qiling_kinematics/
├── include/qiling_kinematics/
│   ├── hierarchical_dik_solver.hpp
│   ├── anthropomorphic_elbow.hpp
│   └── ik_diagnostics.hpp
│
├── src/
│   ├── hierarchical_dik_solver.cpp
│   ├── anthropomorphic_elbow.cpp
│   └── differential_ik_node.cpp
│
└── config/
    └── hierarchical_differential_ik.yaml
```

第一版可以先不增加新 ROS package。

---

# 2. 任务定义

每条机械臂：

```text
7 个关节
```

手腕完整位姿任务：

```text
位置         3 DoF
姿态         3 DoF
------------------
Wrist Task   6 DoF
```

正常满秩情况下：

```text
7 - 6 = 1 DoF
```

因此每条机械臂理论上存在约 1 个主要冗余自由度。

对于人形 7DoF 手臂，该自由度主要表现为：

```text
elbow swivel
/
肩-肘-腕平面绕“肩到腕轴”的旋转
```

因此新的任务优先级定义为：

```text
Priority 0
安全约束

Priority 1
Wrist Position

Priority 2
Wrist Orientation in null(J_position)

Priority 3
Elbow Swivel / Rest Pose / Joint Centering / Smoothness in null(J_wrist)
```

Priority 2 不允许明显破坏 Priority 1；Priority 3 不允许明显破坏 Priority 1 和 Priority 2。完整 wrist 正常满秩时只剩 1DoF，因此 elbow、rest pose、joint centering 和 smoothness 在 Priority 3 内是一个加权冗余目标，不再声明为多个严格子层级。

---

# 3. 总体控制架构

每侧独立求解，但使用同一套 solver 类。

```text
                     XR Wrist Target
                           │
                           ▼
                  target preprocessing
                           │
                           ▼
                current Pinocchio FK
                           │
                           ▼
           ┌───────────────────────────┐
           │ Level 0: hard constraints │
           │ q limit / vel / accel     │
           └───────────────────────────┘
                           │
                           ▼
           ┌───────────────────────────┐
           │ Level 1: Wrist Position   │
           │ strict primary task       │
           └───────────────────────────┘
                           │
                    qdot_position
                           │
                           ▼
                    null(J_position)
                           │
                           ▼
           ┌───────────────────────────┐
           │ Level 2: Orientation      │
           │ preserve position task    │
           └───────────────────────────┘
                           │
                    qdot_pose
                           │
                           ▼
                    null(J_wrist)
                           │
                           ▼
           ┌───────────────────────────┐
           │ Level 3: Elbow Swivel     │
           │ posture / centering       │
           └───────────────────────────┘
                           │
                    qdot_final
                           │
                           ▼
               bounded q_ref / q_cmd
                           │
                           ▼
                  MIT command / mux
```

---

# 4. 坐标系统一原则

这是实现新 IK 之前必须先固定的规则。

## 4.1 IK 内部统一使用 base_link 表达

新 IK 内部建议全部使用：

```text
Pinocchio ReferenceFrame::LOCAL_WORLD_ALIGNED
```

含义：

- frame 原点仍在 wrist frame。
- 线速度方向用 `base_link` 轴表达。
- 角速度方向用 `base_link` 轴表达。

这样 XR bridge 输出的目标如果已经是：

```text
frame_id = base_link
```

则：

```text
position error
rotation error
Jacobian
desired Cartesian velocity
```

全部在同一个方向基下解释。

禁止以下混用：

```text
LOCAL Jacobian
+
base_link position error
```

或：

```text
LOCAL_WORLD_ALIGNED Jacobian
+
log6(current^-1 * target) 的 body-frame error
```

除非数学上明确完成 Adjoint 转换。

---

## 4.2 新方案不直接使用一个未验证的 6D log6 混合误差

建议明确分开：

### Position

```math
e_p = p_target - p_current
```

均表达在：

```text
base_link
```

### Orientation

定义：

```math
R_err = R_target R_current^T
```

然后：

```math
e_R = Log(R_err)
```

其中：

```text
e_R ∈ R^3
```

也表达在 `base_link` 方向基。

因此：

```text
e_p : meter
e_R : radian
```

二者不直接未经尺度处理地混合。

---

## 4.3 Jacobian reference 必须通过有限差分验证

每侧至少做：

```text
X translation
Y translation
Z translation
X rotation
Y rotation
Z rotation
```

数值雅可比测试。

给定：

```math
q' = q + ε e_i
```

用 FK 得到：

```math
Δp / ε
Δθ / ε
```

与 Pinocchio analytic Jacobian 对应列比较。

建议测试：

```text
ε = 1e-6 ~ 1e-5 rad
```

验收不是只看“方向差不多”，而是：

```text
analytic Jacobian
≈
finite-difference Jacobian
```

---

# 5. Level 0：安全硬约束

Level 0 不属于优化偏好，而是所有 Level 必须满足的约束。

每周期：

```math
dt = 1 / 50 = 0.02 s
```

## 5.1 关节速度限制

每个关节：

```math
-v_{max,i} ≤ qdot_i ≤ v_{max,i}
```

当前系统已有 `max_joint_velocity`，新版建议改成支持 per-joint 配置。

## 5.2 基于 position limit 的一步速度约束

为了保证下一步不会越过：

```text
q_min + margin
q_max - margin
```

计算：

```math
qdot_lower_pos = (q_min + margin - q) / dt
```

```math
qdot_upper_pos = (q_max - margin - q) / dt
```

最终：

```math
qdot_lower = max(-v_max, qdot_lower_pos)
```

```math
qdot_upper = min(+v_max, qdot_upper_pos)
```

## 5.3 加速度限制

新增：

```math
-a_max Δt ≤ qdot_k - qdot_{k-1} ≤ a_max Δt
```

因此：

```math
qdot_lower_acc = qdot_prev - a_max dt
```

```math
qdot_upper_acc = qdot_prev + a_max dt
```

最终：

```math
lb = max(-v_max, qdot_lower_pos, qdot_lower_acc)
```

```math
ub = min(+v_max, qdot_upper_pos, qdot_upper_acc)
```

如果：

```text
lb_i > ub_i
```

立即：

```text
IK invalid → hold
```

不能继续求解。

---

# 6. v1.0 原始 6D Primary 公式（已被 Phase 0 严格层级取代）

> 本节保留原始推导作为历史记录。实现不得继续把 position 和 orientation 放入同一个平级 QP；规范实现见 Phase 0 冻结契约的 Level 1 与 Level 2。

Level 1 是最高优先级运动任务。

目标：

```text
人的手柄移动 → 机器人 wrist position 跟随
人的手柄转动 → 机器人 wrist orientation 跟随
```

## 6.1 期望 Cartesian velocity

Position：

```math
v_p_raw = K_p e_p
```

然后做 norm saturation：

```math
v_p_des = sat_norm(v_p_raw, v_linear_max)
```

Orientation：

```math
ω_raw = K_R e_R
```

然后：

```math
ω_des = sat_norm(ω_raw, ω_max)
```

禁止简单对 xyz 分别裁切，因为那会改变运动方向。

---

## 6.2 Position 与 Rotation 的量纲归一化

不要再直接用 meter 与 radian 比较。

引入 characteristic length：

```math
L_c
```

单位为 meter。

定义：

```math
J_task =
[
    J_linear
    L_c J_angular
]
```

```math
v_task =
[
    v_linear
    L_c ω
]
```

这样上下两部分统一成为：

```text
m/s
```

例如：

```yaml
characteristic_length_m: 0.25
```

只是建议初值，需要根据机械臂尺度标定。

---

## 6.3 Level 1 QP

变量：

```math
qdot ∈ R^7
```

求解：

```math
min  1/2 || W_task (J_task qdot - v_task) ||²
   + λ/2 ||qdot||²
   + w_s/2 ||qdot - qdot_prev||²
```

约束：

```math
lb ≤ qdot ≤ ub
```

第一版建议：

```yaml
primary_position_weight: 1.0
primary_orientation_weight: 1.0
```

先完成量纲统一，再谈 position / orientation 偏好。

---

# 7. Primary Task 的奇异性处理

每周期对 scaled Jacobian：

```math
J_task
```

做 SVD：

```math
J = U Σ V^T
```

记录：

```text
sigma_min
condition_number
rank
```

## 7.1 自适应 damping

建议：

```math
r = clamp((σ_warn - σ_min) / σ_warn, 0, 1)
```

```math
λ = λ_min + λ_gain r²
```

含义：

```text
sigma_min 正常 → damping 很小
进入 warning 区 → damping 平滑增大
接近奇异 → 大 damping，并降低 Cartesian 速度
```

`σ_warn` 必须对 scaled Jacobian 标定。

---

# 8. Primary Null-Space

Level 1 求得：

```text
qdot_primary
```

之后，对当前：

```math
J_task
```

计算 null-space basis：

```math
N ∈ R^(7×k)
```

满足：

```math
J_task N ≈ 0
```

正常满秩：

```text
rank(J_task) = 6
k = 1
```

接近奇异位形时：

```text
rank < 6
k > 1
```

建议用：

```text
Eigen::JacobiSVD
```

判定：

```math
σ_i > σ_rank_threshold
```

为有效 rank。

二级速度形式：

```math
qdot = qdot_primary + N z
```

其中：

```math
z ∈ R^k
```

这就是“手腕优先、肘部只在 null-space 中调”的核心。

---

# 9. 拟人 Elbow Target

当前 Quest 手柄只提供 wrist/controller 6DoF，所以当前无法知道操作者真实肘部的位置。

本方案定义的是：

> **anthropomorphic elbow heuristic**

即：

```text
让机器人肘部保持自然、向外、稳定
```

而不是复制操作者真实肘部。

未来如果获得 Quest body/elbow tracking，可直接替换 elbow target generator，而不需要推翻 Level 2 结构。

---

# 10. v1.0 二连杆 Elbow 几何（已被 Swivel Direction 取代）

> 当前 S4 肩部和腕部不是理想共点球形关节，因此本节的 L1/L2 肘圆只保留为历史思路，不作为严格控制目标。实现使用 Phase 0 契约中的 current shoulder-wrist axis 和 elbow swivel angle。

每侧配置：

```text
shoulder frame
elbow frame
wrist frame
```

实际 frame 名称必须从当前 URDF 检查，不能猜。

定义：

```math
p_s = shoulder position
p_e = elbow position
p_w = wrist target position
```

上臂长度：

```math
L_1
```

前臂长度：

```math
L_2
```

长度建议从 URDF / FK 几何初始化得到，不手填魔法数字。

---

# 11. 几何 elbow target

定义肩到目标腕：

```math
d = p_w - p_s
D = ||d||
e_sw = d / D
```

可达距离先裁切：

```math
|L_1 - L_2| + ε ≤ D ≤ L_1 + L_2 - ε
```

三角形几何：

```math
a = (L_1² - L_2² + D²) / (2D)
```

```math
h = sqrt(max(L_1² - a², 0))
```

肩腕轴上的肘圆中心：

```math
p_c = p_s + a e_sw
```

肘部目标：

```math
p_e_des = p_c + h u
```

其中 `u` 是肩腕轴法平面中的首选拟人方向。

---

# 12. 拟人 elbow direction

机器人 base 坐标：

```text
X：前
Y：左
Z：上
```

左右 outward vector：

```math
o_left  = [0, +1, 0]^T
o_right = [0, -1, 0]^T
```

down：

```math
d_down = [0, 0, -1]^T
```

构造：

```math
u_raw =
w_out o_side
+
w_down d_down
+
w_home u_home
```

其中 `u_home` 来自 `teleop_home` 的 elbow swivel reference。

然后投影到肩腕轴法平面：

```math
u_proj = (I - e_sw e_sw^T) u_raw
```

```math
u = u_proj / ||u_proj||
```

若 `||u_proj||` 太小：

1. 优先使用上一帧 `u_prev`；
2. 再 fallback 到 home reference；
3. 禁止随机选择正交方向，否则 elbow 可能 flip。

---

# 13. Elbow Target 连续性

保存：

```text
u_prev
p_elbow_target_prev
```

做连续性处理：

```math
u_new =
normalize(
    (1 - α) u_prev
    + α u_geometry
)
```

或者对 elbow swivel angle 做一阶低通。

目标：

```text
手柄轻微运动 → elbow target 连续
跨特殊构型 → 不突然 elbow-out / elbow-in 翻转
```

---

# 14. Elbow Velocity Task

误差：

```math
e_e = p_e_des - p_e
```

期望 elbow 速度：

```math
v_e_des = sat_norm(K_e e_e, v_elbow_max)
```

从 Pinocchio 获取 elbow frame 的线速度 Jacobian：

```math
J_e ∈ R^(3×7)
```

与 primary 一致使用：

```text
LOCAL_WORLD_ALIGNED
```

---

# 15. v1.0 Elbow Position Null-Space QP（已被 Swivel QP 取代）

> 后续实现不直接追踪本节的三维 p_e_des，而是在完整 wrist 的 1D null-space 中优化有符号 elbow swivel error。

代入：

```math
qdot = qdot_primary + N z
```

二级变量：

```math
z ∈ R^k
```

目标：

```math
min
    w_elbow / 2
    ||J_e (qdot_primary + N z) - v_e_des||²

  + w_posture / 2
    ||W_q (
        qdot_primary + N z
        - qdot_posture
      )||²

  + w_smooth / 2
    ||qdot_primary + N z - qdot_prev||²

  + w_z / 2
    ||z||²
```

Natural posture velocity：

```math
qdot_posture = -K_q (q - q_rest)
```

建议 `q_rest` 默认取 `teleop_home`。

由于 Level 2 只在 `N` 中运行，posture 不会直接把 wrist 拉回 home。

---

# 16. Level 2 仍必须满足 Level 0

因为：

```math
qdot = qdot_primary + N z
```

所以：

```math
lb - qdot_primary ≤ N z ≤ ub - qdot_primary
```

ProxQP 可直接把 `N` 作为 secondary inequality matrix。

---

# 17. Primary Preservation Check

由于数值 SVD 与浮点误差：

```math
J_task N
```

不可能严格为零。

二级求解后计算：

```math
δ_primary =
J_task(qdot_final - qdot_primary)
```

分别记录：

```text
primary_position_degradation
primary_orientation_degradation
```

如果超过阈值：

```text
qdot_final = qdot_primary
```

保证 secondary 永远不能明显破坏 primary。

---

# 18. Joint Limit：硬约束 + 软回中

硬约束：

```text
Level 0 绝不越过 margin
```

软约束：

```math
q_mid = (q_min + q_max) / 2
```

```math
r_i =
(q_i - q_mid_i)
/
((q_max_i - q_min_i)/2)
```

可构造靠近限位时快速增加的 joint-centering gradient。

原则：

> hard bound 负责安全；soft cost 负责提前离开危险区域。

---

# 19. q_target / q_ref 新策略

不允许：

```text
qdot 在 q_measured 上计算
但长期无界积分到旧 q_target
```

推荐 bounded reference integrator：

```math
q_ref_candidate =
q_ref_prev + qdot_final dt
```

```math
e_ref =
q_ref_candidate - q_measured
```

限制：

```math
|e_ref_i| ≤ q_ref_gap_max_i
```

若超过：

```math
q_ref =
q_measured
+
clamp(e_ref, -gap_max, +gap_max)
```

真实机器人第一阶段可使用更保守模式：

```math
q_cmd = q_measured + qdot_final dt
```

先消除 command runaway 风险。

---

# 20. Hold / Clutch 状态机

每侧：

```text
HOLD
ACTIVE
FAULT
```

## HOLD

进入条件：

```text
Grip released
XR timeout
JointState timeout
Target timeout
QP failure
invalid number
safety violation
```

行为：

```text
qdot = 0
qdot_prev = 0
q_ref = q_measured
```

## ACTIVE entry

Grip 新按下：

```text
robot_anchor = current robot wrist
xr_anchor = current XR controller
q_ref = q_measured
qdot_prev = 0
```

第一帧 target 必须约等于当前 robot wrist。

## FAULT

出现：

```text
NaN / Inf
lb > ub
严重 joint-limit violation
连续 QP failure
q_ref gap 超限
primary error 长时间发散
```

进入 FAULT：

```text
停止 active
hold current
要求 release Grip 后重新进入
```

---

# 21. 双臂策略

第一版继续：

```text
Left arm solver
Right arm solver
```

两个独立 7DoF HQP。

共享：

```text
参数结构
safety policy
diagnostics 格式
```

后续再升级 14DoF coupled QP，实现：

```text
inter-arm collision
relative hand constraint
bimanual object constraint
```

---

# 22. XR → Robot Target 的前置要求

bridge 输出 `/teleop/*_wrist_target` 必须满足：

1. `frame_id = base_link`。
2. position 与 orientation 来自严格定义的 SE(3) 映射。
3. 左右手共享同一个 XR-world → robot-base 基础变换。
4. 左右差异只来自 controller handedness、controller→wrist 标定和机械结构镜像。
5. 不继续依赖大量左右不同的 world-axis sign patch。
6. 如果最终要求 1:1 wrist orientation following，`rotation_scale` 应逐步标定到 1.0。
7. 在 IK 调参前先证明 target PoseStamped 自身正确。

---

# 23. 推荐 ROS 2 输入输出

现有输入：

```text
/joint_states

/teleop/left_wrist_target
/teleop/right_wrist_target

/teleop/left_control_mode
/teleop/right_control_mode
```

现有输出：

```text
/human_lower_command
```

建议增加：

```text
/teleop/left_elbow_state
/teleop/right_elbow_state

/teleop/left_elbow_target
/teleop/right_elbow_target
```

可先使用：

```text
geometry_msgs/msg/PointStamped
```

---

# 24. 每周期 Diagnostics

至少监控：

```text
dt

target_age
joint_state_age

position_error_norm
rotation_error_norm

desired_linear_velocity_norm
desired_angular_velocity_norm

sigma_min
condition_number
jacobian_rank
adaptive_damping

qdot_primary
qdot_secondary
qdot_final

qdot_max

q_ref_minus_q_norm
q_ref_minus_q_max

elbow_error_norm
elbow_target_position

primary_degradation_after_secondary

QP1 status
QP1 solve_us

QP2 status
QP2 solve_us

control_mode
```

ROS 日志降频到约 1 Hz，内部统计每周期更新。

---

# 25. v1.0 YAML 示例（已由 Phase 0 配置取代）

> 本节仅保留旧参数名作为迁移参考。后续实现读取 src/qiling_kinematics/config/hierarchical_differential_ik.yaml。

建议文件：

```text
src/qiling_kinematics/config/hierarchical_differential_ik.yaml
```

示例：

```yaml
qiling_differential_ik:
  ros__parameters:

    control_rate_hz: 50.0
    target_frame: base_link

    target_timeout_sec: 0.25
    joint_state_timeout_sec: 0.20

    position_gain: 3.0
    rotation_gain: 3.0

    max_linear_velocity_mps: 0.25
    max_angular_velocity_rps: 0.80

    characteristic_length_m: 0.25

    primary_position_weight: 1.0
    primary_orientation_weight: 1.0

    primary_regularization: 1.0e-4
    primary_smoothness_weight: 1.0e-3

    singularity_adaptive_damping: true
    damping_min: 1.0e-4
    damping_gain: 2.0e-2

    # 必须根据 scaled Jacobian 实测再定
    sigma_warn: 0.08
    sigma_rank_threshold: 1.0e-4

    joint_limit_margin: 0.08

    max_joint_velocity: 1.0
    max_joint_acceleration: 3.0

    q_ref_gap_max: 0.10

    elbow_task_enabled: true

    elbow_gain: 2.0
    max_elbow_velocity_mps: 0.20

    elbow_weight: 1.0
    posture_weight: 0.10
    secondary_smoothness_weight: 0.05

    elbow_outward_weight: 1.0
    elbow_downward_weight: 0.30
    elbow_home_reference_weight: 0.80

    elbow_direction_filter_alpha: 0.15

    # 必须替换成 URDF 实际 frame 名称
    left_shoulder_frame: ""
    left_elbow_frame: ""
    left_wrist_frame: "LH_hand_base_link"

    right_shoulder_frame: ""
    right_elbow_frame: ""
    right_wrist_frame: "RH_hand_base_link"

    max_secondary_position_degradation_mps: 0.005
    max_secondary_orientation_degradation_rps: 0.02

    max_position_error_m: 0.25
    max_rotation_error_rad: 1.20

    qp_max_iter: 80
    qp_eps_abs: 1.0e-5
```

以上参数仅是第一轮仿真起始值，不是最终真实机器人参数。

---

# 26. C++ Solver 类建议

```cpp
class HierarchicalDIKSolver
{
public:
    struct Result
    {
        bool success;

        Eigen::VectorXd qdot_primary;
        Eigen::VectorXd qdot_secondary;
        Eigen::VectorXd qdot_final;

        double position_error;
        double rotation_error;

        double sigma_min;
        double condition_number;

        double elbow_error;
        double primary_degradation;

        int primary_qp_status;
        int secondary_qp_status;
    };

    Result solve(
        const Eigen::VectorXd& q,
        const Eigen::VectorXd& qdot_prev,
        const pinocchio::SE3& wrist_target,
        double dt);
};
```

内部拆分：

```cpp
computeKinematics();
computeWristError();
computeSafeVelocityBounds();
solvePrimaryQP();
computeNullspace();
computeAnthropomorphicElbowTarget();
solveSecondaryQP();
verifyPrimaryPreservation();
applySafetyFallback();
```

不要把所有逻辑继续堆在 ROS timer callback 中。

---

# 27. AnthropomorphicElbow 类建议

```cpp
class AnthropomorphicElbow
{
public:
    enum class Side
    {
        Left,
        Right
    };

    struct Output
    {
        bool valid;
        Eigen::Vector3d target_position;
        Eigen::Vector3d preferred_direction;
        double geometric_height;
    };

    Output compute(
        const Eigen::Vector3d& shoulder,
        const Eigen::Vector3d& wrist_target,
        const Eigen::Vector3d& current_elbow,
        const Eigen::Vector3d& home_direction);
};
```

内部维护：

```text
previous preferred direction
```

避免 elbow flip。

---

# 28. 每周期伪代码

```cpp
void controlStep()
{
    if (!joint_state_valid) {
        hold();
        return;
    }

    if (!target_valid || mode != ACTIVE) {
        hold();
        return;
    }

    q = buildQFromJointNames();

    forwardKinematics(q);

    wrist_current = getWristPose();
    J_wrist = getWristJacobianLWA();

    elbow_current = getElbowPosition();
    J_elbow = getElbowLinearJacobianLWA();

    shoulder_position = getShoulderPosition();

    e_pos = target.p - current.p;
    e_rot = Log(target.R * current.R.transpose());

    v_des = saturateNorm(Kp * e_pos, v_max);
    w_des = saturateNorm(Kr * e_rot, w_max);

    J_task = stack(
        J_linear,
        L_char * J_angular
    );

    v_task = concat(
        v_des,
        L_char * w_des
    );

    lb, ub = velocityPositionAccelerationBounds(
        q,
        qdot_prev,
        dt
    );

    if (!boundsValid(lb, ub)) {
        hold();
        return;
    }

    svd = SVD(J_task);
    lambda = adaptiveDamping(svd.sigma_min);

    qdot_primary = solvePrimaryQP(
        J_task,
        v_task,
        lb,
        ub,
        lambda
    );

    if (!valid(qdot_primary)) {
        hold();
        return;
    }

    N = computeNullspaceBasis(J_task, svd);

    elbow_target = elbow_model.compute(
        shoulder_position,
        wrist_target.translation(),
        elbow_current,
        home_elbow_direction
    );

    if (elbow_target.valid && N.cols() > 0) {

        v_elbow_des =
            saturateNorm(
                K_elbow *
                (elbow_target.position - elbow_current),
                elbow_vmax
            );

        z = solveSecondaryQP(
            qdot_primary,
            N,
            J_elbow,
            v_elbow_des,
            q,
            q_rest,
            qdot_prev,
            lb,
            ub
        );

        qdot_final =
            qdot_primary + N * z;
    }
    else {
        qdot_final = qdot_primary;
    }

    if (!primaryPreserved(
            J_task,
            qdot_primary,
            qdot_final)) {

        qdot_final = qdot_primary;
    }

    if (!finiteAndSafe(qdot_final)) {
        hold();
        return;
    }

    q_ref =
        updateBoundedReference(
            q_ref,
            q,
            qdot_final,
            dt
        );

    publishArmCommand(q_ref);

    qdot_prev = qdot_final;
}
```

---

# 29. 为什么 elbow task 用几何位置而不是只用 joint posture

只做：

```math
q → q_rest
```

只能表示：

```text
机器人倾向 home
```

但不同 wrist target 下，自然 elbow 本身应随空间位置变化。

使用 shoulder-elbow-wrist 几何：

```text
手伸远 → 肘自然展开
手靠近身体 → 肘自然弯曲
左臂 → 肘倾向左外
右臂 → 肘倾向右外
```

`q_rest` 只作为 tertiary bias。

因此：

```text
elbow geometry
>
joint rest posture
```

---

# 30. 为什么不把 elbow 直接加到原来的单个 QP

不建议：

```math
w_wrist E_wrist
+
w_elbow E_elbow
```

直接全部塞进同一个 QP。

否则：

```text
w_elbow 稍大 → wrist 被牺牲
w_elbow 太小 → elbow 基本不起作用
```

null-space 明确表达：

```text
先把 wrist 做好
再用剩余自由度处理 elbow
```

---

# 31. Orientation Tracking 验收

如果最终目标是 1:1 wrist orientation following，`rotation_scale` 最终应逐步达到：

```text
1.0
```

建议：

```text
0.50
→ 0.65
→ 0.80
→ 1.00
```

每一级都执行 X/Y/Z 单轴测试，并记录：

```text
target position drift
actual wrist position drift
```

---

# 32. 旧版分阶段实施记录

> 本节保留 v1.0 的历史阶段划分，不再作为实施顺序。当前实施以 Phase 0 冻结契约及已确认的 Phase 1～13 顺序为准。

## Phase A：只重构 Primary 6D，不开 elbow

设置：

```yaml
elbow_task_enabled: false
```

完成：

1. `LOCAL_WORLD_ALIGNED` 统一。
2. position error 与 orientation error 分离。
3. characteristic length scaling。
4. norm velocity saturation。
5. joint acceleration bounds。
6. adaptive damping。
7. bounded q_ref。
8. diagnostics。

目标：

> 先证明 wrist 6D 本身能稳定、准确工作。

## Phase B：Primary 单臂单元测试

不用 Quest，程序生成：

```text
+5 cm X
+5 cm Y
+5 cm Z

+10° Rx
+10° Ry
+10° Rz

position + rotation combined
```

左右臂分别执行。

## Phase C：开启 Elbow Null-Space

用同一个 wrist target，从不同初始 elbow 构型开始。

验收：

```text
最终 wrist pose 基本一致
但 elbow 都收敛到同一类自然外展构型
```

## Phase D：扩大 workspace

测试：

```text
前伸
侧伸
上抬
下探
跨身体中线附近
接近完全伸直
```

重点记录：

```text
elbow flip
sigma_min
joint limit
wrist tracking
```

## Phase E：接 Quest

```text
先左臂
再右臂
最后双臂
```

每个动作只验证一个自由度。

## Phase F：真实机器人

```text
单臂
↓
低 max velocity
↓
低 max acceleration
↓
低 command kp
↓
position-only 小范围
↓
6D wrist
↓
elbow null-space
↓
双臂
```

---

# 33. 建议仿真验收指标

以下为第一版建议阈值，不是最终硬规格。

Static wrist target：

```text
position error < 5 ~ 10 mm
orientation error < 2 ~ 3 deg
```

Pure translation：

```text
orientation unintended change < 2 deg
```

Pure rotation：

```text
position unintended change < 5 ~ 10 mm
```

Elbow posture：

```text
elbow target error < 20 ~ 40 mm
```

但 wrist 永远优先于 elbow。

Reference gap：

```math
max_i |q_ref_i - q_i|
```

超过配置阈值持续一定时间则 hold。

---

# 34. Elbow Flip 专项测试

移动 wrist 沿轨迹：

```text
前方
→ 上方
→ 身体侧方
→ 回前方
```

记录：

```text
preferred elbow direction
elbow position
nullspace vector
z
```

检查：

```text
u_t · u_(t-1)
```

若突然接近 `-1`，说明 preferred direction 翻转，必须保持上一帧 branch。

---

# 35. Null-Space Basis 连续性

SVD 输出的 null vector：

```text
n
```

每帧可能变成：

```text
-n
```

二者数学上等价，但控制上会造成符号跳变。

满秩 7DoF 单臂第一版可做：

```cpp
if (n.dot(n_prev) < 0.0) {
    n = -n;
}
```

对于 `k > 1` 时再做更一般的 subspace continuity。

---

# 36. Singularity 下的 Elbow 策略

当：

```text
sigma_min < sigma_elbow_disable
```

应逐渐降低：

```text
elbow_weight
```

而不是增强。

定义：

```math
w_elbow_eff =
w_elbow * f(σ_min)
```

正常构型 `f≈1`，接近奇异时 `f→0`。

同时增大 primary damping。

---

# 37. 手腕 orientation 与 elbow 必须解耦理解

Priority 1：

```text
wrist position + wrist orientation
```

Priority 2：

```text
elbow/swivel
```

因此以后不再通过：

```text
调 rotation_weight
```

去间接修肘部姿态。

---

# 38. 与 teleop_home 的关系

`teleop_home` 可作为：

```text
q_rest
home elbow direction
```

但不能成为硬目标。

正确语义：

```text
wrist task 决定“手要去哪、朝哪”
home posture 决定“存在多个解时更喜欢哪一个”
```

---

# 39. Debug 可视化

MuJoCo / RViz 建议显示：

```text
current wrist
target wrist

current elbow
desired elbow

shoulder-to-wrist axis
preferred elbow direction
```

必须能够肉眼区分：

```text
target 错了
还是 IK 错了
```

---

# 40. 首轮代码实施顺序

```text
Step 1
建立 HierarchicalDIKSolver 类

Step 2
保持 elbow OFF
重写 6D error/reference consistency

Step 3
加入 characteristic length

Step 4
加入 Cartesian velocity norm saturation

Step 5
加入 acceleration bound

Step 6
加入 bounded q_ref

Step 7
加入 SVD diagnostics + adaptive damping

Step 8
跑完 Primary 单元测试

Step 9
实现 shoulder/elbow/wrist geometry

Step 10
实现 null-space basis

Step 11
实现 Level 2 elbow QP

Step 12
加入 posture / joint centering

Step 13
做 elbow flip test

Step 14
重新接 Quest

Step 15
双臂

Step 16
真实机器人
```

---

# 41. 本阶段不要同时做的事情

暂时不要同时：

- 接 O6 最终控制。
- 改数据录制格式。
- 做视频回传。
- 改 MuJoCo 腿部逻辑。
- 换 PyRoki。
- 换全身 IK。
- 引入复杂碰撞优化。
- 改真实机器人驱动接口。

否则无法定位效果变化来自哪一层。

---

# 42. 失败回退规则

```text
secondary QP fail
→ 使用 qdot_primary

primary QP fail
→ hold

state invalid
→ hold

safety violation
→ fault / hold
```

绝不能：

```text
继续使用上一帧 qdot
```

---

# 43. ProxQP 使用建议

Primary：

```text
7 variables
box constraints
```

Secondary：

```text
k variables
正常 k=1
```

50 Hz 下计算量很小。

优先保证：

```text
确定性
warm-start
预分配
finite checks
```

同时记录：

```text
mean solve time
max solve time
P99 solve time
```

---

# 44. 预期改善

## 位置跟随

通过：

```text
统一 frame
独立 position error
Cartesian velocity saturation
```

减少方向和耦合问题。

## 姿态跟随

通过：

```text
明确 Log3 orientation error
characteristic-length scaling
primary 6D task
```

减少 meter/radian 权重靠感觉的问题。

## 肘部姿态

通过：

```text
primary null-space
+
shoulder-elbow-wrist geometry
+
home anthropomorphic reference
```

主动利用 7DoF 冗余。

## 平滑

通过：

```text
joint acceleration limit
qdot smoothness
continuous elbow direction
bounded q_ref
```

减少抖动和积累。

## 奇异位形

通过：

```text
SVD monitor
adaptive damping
secondary fade-out
```

让系统安全退化。

---

# 45. 本方案明确不能解决的事情

本方案不能做到：

> “机器人肘部严格复制操作者真实肘部。”

因为当前输入没有 human elbow pose。

当前实现的是：

```text
anthropomorphic elbow selection
```

即：

> 在 wrist target 已知时，从机器人的冗余解中选择更自然、更稳定、更类似人形手臂的解。

未来如果获得：

```text
Quest body tracking
camera human pose
IMU elbow tracker
```

只需替换：

```text
p_elbow_des
```

Level 2 结构不需要推翻。

---

# 46. 最终目标状态

```text
Quest Controller 6D
        │
        ▼
Robot Wrist Target 6D
        │
        ▼
┌────────────────────────┐
│ Priority 1             │
│ Wrist Position         │
│ Wrist Orientation      │
└────────────────────────┘
        │
        ▼
Primary Null Space
        │
        ▼
┌────────────────────────┐
│ Priority 2             │
│ Elbow Out / Natural    │
│ Home Posture           │
│ Joint Centering        │
│ Smoothness             │
└────────────────────────┘
        │
        ▼
Velocity / Accel / Limits
        │
        ▼
Bounded q_ref
        │
        ▼
Robot
```

最终设计原则：

> **手腕任务决定“手要去哪、朝哪”；null-space 决定“肘和肩用什么姿态完成它”。**

---

# 47. 与现有工程的迁移结论

当前工程不需要推倒重来。

保留：

```text
XRoboToolkit
PXREA adapter
ROS topics
clutch
MuJoCo
Pinocchio
ProxQP
40D command interface
```

重构：

```text
differential IK formulation
frame/error consistency
redundancy resolution
elbow task
singularity management
q_ref generation
diagnostics
```

第一阶段成功标准不是“看起来更顺”，而是能够用日志证明：

```text
1. wrist position error 收敛
2. wrist orientation error 收敛
3. pure rotation 不再明显带走 position
4. pure translation 不再明显带走 orientation
5. elbow 在 null-space 中向自然构型收敛
6. secondary 不明显破坏 primary
7. q_ref 不与 q_measured 长期分离
8. singularity 可检测并安全退化
```

只有这些通过后，再进入 O6、双臂协作和真实机器人高质量遥操。
