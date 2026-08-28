# Qiling Hierarchical DIK Phase 0 冻结契约

状态：已冻结  
契约版本：phase0-v1  
适用范围：单侧 7DoF 手臂、双侧独立运行、50 Hz 外部控制  
生效日期：2026-08-27

本文档是后续分层差分 IK 实现的规范性依据。如果旧设计文档、现有 differential_ik_node.cpp 或旧 differential_ik.yaml 与本文档冲突，以本文档为准。

Phase 0 只冻结接口、数学层级、状态机、安全语义和验收标准，不改变当前运行节点。

---

## 1. 已冻结的设计结论

1. Wrist position 严格高于 wrist orientation。
2. Orientation 只能在 position Jacobian 的 null-space 中求解。
3. Elbow 只能在完整 wrist 6D Jacobian 的 null-space 中求解。
4. Elbow 使用 shoulder-wrist 轴上的 swivel direction / angle，不使用理想二连杆几何作为严格三维目标。
5. Elbow 偏好固定为“向外 + 轻微向下 + teleop_home reference”，不使用 Quest 摇杆调整。
6. 左右手臂使用完全独立的 HOLD / ACTIVE / FAULT 状态机。
7. MuJoCo 使用 bounded_integrator reference。
8. 真实机器人第一阶段使用 measured_step reference，即 q_cmd = q_measured + qdot × dt。
9. 第一版只在完整 wrist Jacobian rank=6 时启用 1D elbow null-space。
10. 接近奇异位形时逐渐关闭 elbow；rank<6 时完全关闭 elbow。
11. 当前阶段不接 O6、不改视频、不改数据录制、不改腿部控制。

---

## 2. 规范任务层级

~~~text
Level 0
实际关节硬限位、limit damper、速度、加速度、finite、freshness
        |
        v
Level 1
Wrist Position QP
        |
        v
null(J_position)
        |
        v
Level 2
Wrist Orientation QP
        |
        v
null([J_position; Lc * J_angular])
        |
        v
Level 3
Elbow Swivel + Rest Posture + Joint Centering + Smoothness
        |
        v
safe qdot
        |
        +--> MuJoCo: bounded q_ref
        |
        +--> Real robot initial: q_measured + qdot * dt
~~~

Level 1、Level 2、Level 3 都必须满足 Level 0。

### 2.1 Level 1：位置

全部量使用 base_link 方向基：

~~~text
e_position = p_target - p_current
v_desired = norm_saturate(Kp * e_position, max_linear_velocity)
~~~

位置 QP 输出 qdot_position。其结果是后续任务必须保持的最高运动优先级结果。

### 2.2 Level 2：姿态

~~~text
R_error = R_target * transpose(R_current)
e_orientation = Log3(R_error)
omega_desired = norm_saturate(Kr * e_orientation, max_angular_velocity)
~~~

设 N_position 为 J_position 的 null-space basis：

~~~text
qdot_pose = qdot_position + N_position * z_orientation
~~~

Orientation QP 必须继续满足最终速度边界。求解后检查：

~~~text
norm(J_position * (qdot_pose - qdot_position))
<= max_orientation_position_degradation_mps
~~~

检查失败时丢弃 orientation 结果，回退 qdot_position。

### 2.3 Level 3：elbow redundancy

完整 wrist Jacobian：

~~~text
J_wrist = [J_position; characteristic_length * J_angular]
~~~

正常 rank=6 时只使用 1D null-space：

~~~text
qdot_final = qdot_pose + N_wrist * z_elbow
~~~

Level 3 内部目标按以下顺序理解：

1. elbow swivel 是主要冗余目标。
2. q_rest、joint centering、smoothness 是同一 1D 冗余方向中的弱偏好或正则项。
3. 不把 posture 描述为 elbow 后的另一个严格层级，因为正常情况下完整 wrist task 只剩 1DoF。

求解后必须分别检查：

~~~text
norm(J_position * (qdot_final - qdot_pose))
<= max_elbow_position_degradation_mps

norm(J_angular * (qdot_final - qdot_pose))
<= max_elbow_orientation_degradation_rps
~~~

检查失败时丢弃 Level 3，回退 qdot_pose。

---

## 3. 固定坐标表达

IK 内部统一使用：

~~~text
Pinocchio ReferenceFrame::LOCAL_WORLD_ALIGNED
~~~

以下量全部按 base_link 轴表达：

- wrist position error。
- wrist orientation error。
- linear Jacobian。
- angular Jacobian。
- desired linear velocity。
- desired angular velocity。
- elbow point 和 swivel direction。

禁止继续使用 LOCAL Jacobian 配合 base_link 误差，也禁止使用 body-frame log6 误差配合 LOCAL_WORLD_ALIGNED Jacobian。

---

## 4. 固定 frame

| 语义 | 左侧 | 右侧 |
|---|---|---|
| shoulder reference | left_shoulder_pitch_link 原点 | right_shoulder_pitch_link 原点 |
| elbow point | left_elbow_link 原点 | right_elbow_link 原点 |
| wrist control frame | LH_hand_base_link | RH_hand_base_link |

shoulder reference 只是 swivel heuristic 的固定近似肩点，不解释为 pitch/roll/yaw 三轴的严格公共交点。

Elbow swivel 使用 current wrist position，不使用远端 wrist target position。这样 elbow preference 随实际 wrist 运动连续变化，不会被不可达目标提前拉动。

---

## 5. 固定关节顺序

左臂：

~~~text
0 left_shoulder_pitch_joint
1 left_shoulder_roll_joint
2 left_shoulder_yaw_joint
3 left_elbow_joint
4 left_wrist_roll_joint
5 left_wrist_pitch_joint
6 left_wrist_yaw_joint
~~~

右臂：

~~~text
0 right_shoulder_pitch_joint
1 right_shoulder_roll_joint
2 right_shoulder_yaw_joint
3 right_elbow_joint
4 right_wrist_roll_joint
5 right_wrist_pitch_joint
6 right_wrist_yaw_joint
~~~

JointState 必须按 name 映射，不能依赖消息数组顺序。每侧只有在本侧 7 个 position 都存在、finite 且 fresh 时才能进入 ACTIVE。

MITJointCommands 固定映射：

~~~text
left arm:  12..18
left O6:   19..25
right arm: 26..32
right O6:  33..39
~~~

Phase 0 不改变当前 40 维接口。O6 仍不由 arm IK 生成。

---

## 6. 固定 rest pose

关节顺序与第 5 节一致。

~~~text
left_q_rest:
[-0.45, 0.30, 0.02, -1.40, 1.40, 0.0, 0.0]

right_q_rest:
[-0.45, -0.30, 0.02, -1.40, 1.40, 0.0, 0.0]
~~~

用途：

- 计算 home elbow reference direction。
- Level 3 中的弱 posture preference。
- 测试初始构型。

q_rest 不是硬目标，不能在 Grip 松开后主动拉动机械臂。

---

## 7. Elbow swivel 定义

使用：

~~~text
p_s = shoulder reference position
p_e = current elbow position
p_w = current wrist position
~~~

肩腕轴：

~~~text
e_sw = normalize(p_w - p_s)
P = I - e_sw * transpose(e_sw)
~~~

当前 elbow direction：

~~~text
r_elbow = P * (p_e - p_s)
u_current = normalize(r_elbow)
~~~

固定偏好：

~~~text
outward_left  = [0, +1, 0]
outward_right = [0, -1, 0]
downward      = [0, 0, -1]
~~~

~~~text
u_raw =
    outward_weight * outward_side
  + downward_weight * downward
  + home_weight * u_home

u_desired = normalize(P * u_raw)
~~~

有符号 swivel error：

~~~text
phi_error = atan2(
    e_sw dot (u_current cross u_desired),
    u_current dot u_desired)
~~~

退化处理顺序：

1. 首选当前几何产生的连续方向。
2. 投影退化时使用 previous desired direction。
3. previous 无效时使用投影后的 home direction。
4. 仍无效时关闭本周期 elbow task。
5. 禁止随机生成正交方向。

用于可视化的 elbow target 可以保持当前轴向位置和投影半径，只改变投影方向；该点不是严格三维控制目标。

---

## 8. 左右独立状态机

每侧分别维护：

~~~text
HOLD
ACTIVE
FAULT
~~~

### 8.1 HOLD

进入条件包括：

- Grip released。
- 本侧 target timeout。
- 本侧 JointState timeout。
- 本侧 target 尚未建立。
- 正常启动。

行为：

~~~text
qdot = 0
qdot_previous = 0
q_reference = q_measured
clear solver warm-start when required
~~~

HOLD 中绝不积分 qdot。

### 8.2 ACTIVE

进入条件：

- 本侧 7 个关节状态完整、finite、fresh。
- 本侧 target finite、fresh、frame_id=base_link。
- Grip 从 released 变为 pressed。
- 当前没有 latch fault。

进入动作：

~~~text
q_reference = q_measured
qdot_previous = 0
first target approximately equals current wrist pose
~~~

### 8.3 FAULT

触发条件包括：

- NaN / Inf。
- 超过真实关节硬限位。
- velocity bounds 出现 lb > ub。
- 连续 primary QP failure。
- q_ref gap 持续超限。
- position error 持续发散。
- 控制周期严重超时。

行为：

~~~text
qdot = 0
qdot_previous = 0
q_reference = q_measured
fault latched while Grip remains pressed
~~~

恢复必须经过：

~~~text
Grip release
FAULT -> HOLD
下一次 Grip press
HOLD -> ACTIVE
~~~

一侧 FAULT、超时或未收到目标，不得阻止另一侧正常运行。

---

## 9. 安全边界语义

### 9.1 硬边界

URDF q_min / q_max 是实际硬边界。预测下一步不得越过实际硬边界。

### 9.2 joint_limit_margin

margin 是速度阻尼区，不是要求一个周期内返回的第二套硬边界。

进入 margin 后：

- 逐渐降低继续朝限位方向的允许速度。
- 增强 joint-centering preference。
- 保留有限、受加速度约束的离开限位速度。

超过真实硬边界进入 FAULT。

### 9.3 速度与加速度

每侧支持 7 个 per-joint velocity 和 acceleration 参数。

加速度约束：

~~~text
qdot_previous - acceleration_max * dt
<= qdot
<= qdot_previous + acceleration_max * dt
~~~

失败或无效状态不能沿用上一周期 qdot。

### 9.4 dt

控制频率目标为 50 Hz，nominal dt 为 0.02 s，但积分和加速度边界使用 steady clock 测得的实际 dt。

dt 必须裁切到配置范围。超过 stall threshold 时，本侧进入 HOLD 或 FAULT，不使用异常大 dt 积分。

---

## 10. Reference 输出策略

### 10.1 MuJoCo

模式：

~~~text
bounded_integrator
~~~

~~~text
q_ref_candidate = q_ref_previous + qdot_final * dt
q_ref_gap = q_ref_candidate - q_measured
q_ref = q_measured + clamp(q_ref_gap, -gap_max, +gap_max)
~~~

gap 持续超限触发 FAULT。

### 10.2 真实机器人初期

模式：

~~~text
measured_step
~~~

~~~text
q_cmd = q_measured + qdot_final * dt
~~~

第一轮真实机器人测试不累计长期 reference。只有跟踪误差和链路延迟验证稳定后，才重新评估 bounded_integrator。

---

## 11. 奇异性策略

分别诊断：

- J_position rank 和 sigma_min。
- J_angular * N_position 的有效 rank。
- 完整 scaled J_wrist rank、sigma_min 和 condition number。

规则：

1. Position rank 异常：只运行带自适应阻尼和降速的 position，关闭所有低层任务。
2. Orientation 在 position null-space 中不可完整满足：保留 position，orientation 做 best effort。
3. 完整 J_wrist rank=6 且 sigma 正常：允许 1D elbow task。
4. sigma 进入 warning：增大 primary damping、降低 Cartesian 速度、逐渐降低 elbow weight。
5. rank<6 或 sigma 低于 elbow disable threshold：关闭 elbow。
6. 第一版不使用 k>1 secondary null-space。

---

## 12. 失败回退

~~~text
Elbow QP fail
-> qdot_pose

Elbow preservation check fail
-> qdot_pose

Orientation QP fail
-> qdot_position

Position preservation check fail
-> qdot_position

Position QP fail
-> HOLD

invalid state / bounds / finite check
-> HOLD or latched FAULT
~~~

禁止使用上一周期 qdot 作为求解失败回退。

---

## 13. Phase 验收门槛

### 13.1 运动学

- 左右 wrist 和 elbow analytic Jacobian 必须通过 finite-difference 检查。
- epsilon 建议 1e-6 到 1e-5 rad。
- 所有误差与 Jacobian 必须在相同 base_link 方向基下解释。

### 13.2 Position-only

- 静态 position error：第一轮目标小于 5 到 10 mm。
- target 停止后不持续漂移。
- qdot、q_ref、joint limit damper 连续。

### 13.3 Orientation under position priority

- Pure rotation 的 MuJoCo position drift：目标小于 5 mm，第一轮不得超过 10 mm。
- Orientation 加入后的位置任务速度退化不得超过配置阈值。
- Orientation 不可达时必须优先保留 position。

### 13.4 Elbow

- Full wrist rank=6 时才启用。
- 不发生 elbow direction flip。
- Level 3 对 position 和 orientation 的速度退化不得超过配置阈值。
- Elbow 失败只能回退完整 wrist 结果。

### 13.5 状态机

- Grip released 时不积分。
- 一侧 timeout/fault 不影响另一侧。
- 重新 Grip 必须重新 anchor。
- target、JointState 或 QP 失效时不使用旧 qdot。

### 13.6 实时性

- 每周期记录 QP1/QP2/QP3 solve time。
- 记录 mean、max、P99。
- 总 IK 计算必须明显低于 20 ms 控制周期；第一轮工程门槛为 P99 小于 5 ms。

---

## 14. Phase 0 配置

冻结配置位于：

~~~text
config/hierarchical_differential_ik.yaml
~~~

该 YAML 是后续实现的仿真起始值，不由当前 differential_ik_node.cpp 读取。参数值需要在各实现阶段按测试结果标定，但参数名称、单位和语义已经冻结。

---

## 15. Phase 0 明确不做

- 不修改 differential_ik_node.cpp 的运行行为。
- 不启用新的 launch。
- 不改变 /human_lower_command。
- 不接 O6。
- 不修改 XR 坐标映射。
- 不改变 MuJoCo home。
- 不开始真实机器人控制。

下一阶段是 Phase 1：拆分 solver/state/diagnostics，并消除当前 hold 分支未初始化 qdot、左右目标互相阻塞等状态问题。
