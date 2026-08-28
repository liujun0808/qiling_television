# Hierarchical DIK Phase 6：抖动保护与重力前馈

状态：已实现，待 MuJoCo 与真机分阶段验证

## 1. 本阶段范围

本阶段针对末端接近工作空间极限、奇异位形或目标信号微抖时的关节抖动，加入四层保护；同时加入 Pinocchio 重力补偿。

本阶段明确不加入：

- 摩擦补偿；
- 惯性项补偿；
- 科氏力/离心力补偿；
- 基于完整逆动力学的加速度前馈。

## 2. 四层抖动保护

### Level 1：目标低通与误差死区

`differential_ik_node` 对 Quest 目标位置采用一阶低通，对姿态采用四元数最短弧 slerp。默认时间常数为 `0.04 s`。位置误差小于 `0.003 m`、姿态误差小于 `0.01 rad` 时不生成对应任务速度，避免视觉/手柄噪声持续驱动关节。

释放 Grip 或目标超时会清空滤波器；重新接管时第一帧直接建立滤波初值，避免重新按下 Grip 后出现明显的滞后跳变。

### Level 2：奇异度速度缩放与姿态降级

位置任务使用平移 Jacobian 最小奇异值，姿态任务使用带 characteristic length 缩放的完整 `6×7` wrist Jacobian 最小奇异值。

在 slowdown 阈值到 stop 阈值之间线性降低任务速度；达到 stop 阈值时不再注入不可靠的姿态速度。位置仍然是第一优先级。完整 wrist rank 不足时，姿态和肘部次任务回退，保留已经求出的平移结果。

### Level 3：限位、加速度与不可达目标保护

每周期的 QP 同时满足：

```text
q_min <= q_measured + qdot * dt <= q_max
|qdot| <= max_joint_velocity
|qdot - qdot_previous| <= max_joint_acceleration * dt
```

URDF 位置限位仍是硬约束，限位附近使用连续 velocity damper。若目标误差持续较大，同时已经触及硬限位或平移奇异阈值，持续 `0.25 s` 后自动进入 HOLD，停止继续追逐不可达目标；释放并重新按下 Grip 后才能再次接管。

### Level 4：命令输出变化率保护

MuJoCo 的 `d.ctrl` 在最终进入物理引擎前增加了可配置的变化率限制：

```text
|ctrl[k] - ctrl[k-1]| <= maxControlRateNmPerSec * simulation_dt
```

它用于仿真侧复现真实驱动器常见的 torque/command slew limit，避免 QP、PD 或重力前馈的单周期异常值直接变成大扭矩跳变。真机接入时，等价限制必须在实际 MIT/ROS 2 控制适配层再次保留，不能只依赖 MuJoCo。

## 3. 重力补偿数据流

控制周期先用当前双臂关节状态计算 Pinocchio RNEA 的零速度、零加速度结果：

```text
tau_g = rnea(q, v=0, a=0)
```

然后按 URDF 中的 7+7 关节顺序提取双臂力矩，并写入 `MITJointCommands.commands[i].eff`。当前命令语义为：

```text
tau = kp * (q_ref - q) + kd * (qdot_ref - qdot) + eff
eff = clamp(gravity_torque_scale * tau_g, effort_limit)
```

MuJoCo 的命令回调也已经将 `eff` 加到其 PD 输出中，因此仿真和真机消息路径保持一致。O6 固定关节没有进入 Pinocchio 的 14 DoF 重力计算，也不会被误发 arm gravity effort。

可调参数：

```yaml
gravity_compensation_enabled: true
gravity_torque_scale: 1.0
gravity_torque_limit_scale: 1.0
```

初次真机验证建议先使用 `gravity_torque_scale: 0.3`，确认 `eff` 符号和单位后逐步增加到 `1.0`；若驱动 SDK 已经在底层自动做重力补偿，则必须关闭这里的补偿，避免重复叠加。

## 4. 运行时关键参数

```yaml
target_filter_enabled: true
target_filter_time_constant_sec: 0.04
position_error_deadband_m: 0.003
orientation_error_deadband_rad: 0.01
position_sigma_slowdown_start: 0.05
position_sigma_stop: 0.003
wrist_sigma_slowdown_start: 0.08
wrist_sigma_stop: 0.005
max_joint_acceleration_rps2: [3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0]
workspace_guard_enabled: true
workspace_guard_error_m: 0.03
workspace_guard_timeout_sec: 0.25
```

日志中的 `speed=(position,orientation)`、`sigma_position`、`sigma_wrist`、`limit_distance` 和 `solver` 用于判断保护是否生效。`HOLD` 的原因 `unreachable target or workspace boundary` 表示工作空间保护锁存，而不是 QP 数值错误。

## 5. 验证结果与真机注意事项

已通过：

- qiling_kinematics 全部 GTest；
- S4 URDF 的 Pinocchio FK/Jacobian 与 MuJoCo 对照；
- 重力力矩有限性与 URDF effort limit 检查；
- MuJoCo simulator 和 qiling_kinematics 编译。

真机前仍需确认：

1. SDK 的 `eff` 单位、正方向和底层是否已经包含重力补偿；
2. 实际发送频率是否稳定在不超过 50 Hz；
3. 在低负载、低速、无接触条件下逐步提高重力比例；
4. 真机适配层是否有独立的命令限幅、变化率限制和急停策略。
