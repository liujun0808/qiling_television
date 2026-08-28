# Hierarchical DIK Phase 5.0 实现记录

状态：已完成  
完成日期：2026-08-27

## 1. 阶段边界

本阶段只实现 anthropomorphic elbow swivel 几何和运行时诊断，不实现 elbow QP，不改变最终关节速度：

```text
elbow geometry -> diagnostics only
qdot / command output -> unchanged from Phase 4.1
```

这样可以先确认肘部偏好方向和 frame 约定，再进入完整 wrist null-space 的控制实现。

## 2. 几何定义

所有向量均沿 `base_link` 轴表达：

```text
p_s = shoulder frame origin
p_e = current elbow frame origin
p_w = current wrist frame origin
e_sw = normalize(p_w - p_s)
P = I - e_sw * e_sw^T
u_current = normalize(P * (p_e - p_s))
```

这里使用 current wrist，而不是远端 target wrist，避免不可达目标提前把 elbow preference 拉向不连续位置。

左右侧固定偏好为：

```text
outward_left  = [0, +1, 0]
outward_right = [0, -1, 0]
downward      = [0,  0, -1]
```

首选方向：

```text
u_raw = outward_weight * outward
      + downward_weight * downward
      + home_weight * u_home

u_preferred = normalize(P * u_raw)
```

home direction 从配置的 `left_q_rest/right_q_rest` 通过真实 Pinocchio FK 计算，不手填一个与模型无关的向量。

## 3. 连续性和退化

方向计算遵循：

1. 使用当前构型投影后的几何偏好。
2. 若投影退化，使用上一帧 preferred direction 的重新投影。
3. 若上一帧不可用，使用 home direction 的重新投影。
4. 若肩腕轴距离或肘部投影半径退化，本次输出 `valid=false`。
5. 不随机生成正交方向。

如果新的 preferred direction 与上一帧方向点积为负，会翻转到连续分支，避免数值投影导致 elbow direction 突然翻面。

输出的可视化 elbow target 只保持当前肩腕轴上的 wrist 距离和当前投影半径，改变投影方向：

```text
p_elbow_target = p_s + ||p_w-p_s|| * e_sw
                 + radius_current * u_preferred
```

该点目前不作为控制目标。

## 4. 运行时接入

`differential_ik_node` 启动时读取：

```yaml
left_q_rest: [-0.45, 0.30, 0.02, -1.40, 1.40, 0.0, 0.0]
right_q_rest: [-0.45, -0.30, 0.02, -1.40, 1.40, 0.0, 0.0]
elbow_geometry_diagnostic_enabled: true
elbow_outward_weight: 1.0
elbow_downward_weight: 0.30
elbow_home_weight: 0.80
min_shoulder_wrist_distance_m: 0.05
min_elbow_projection_radius_m: 0.02
```

每侧 Grip ACTIVE 且 target fresh 时，节点计算几何结果并写入节流日志。Grip release 会清除 previous direction，重新接管不会继承上一次操作的旧 elbow 分支。

启动日志会明确显示：

```text
elbow geometry=DIAGNOSTIC, elbow control=OFF
```

运行日志中的 `Phase 5 geometry` 字段包括：

```text
valid
swivel_error
preferred=(x,y,z)
projection radius
shoulder-wrist distance
```

验证左右方向时，观察 `preferred.y`：正常情况下左侧偏正、右侧偏负。`swivel_error` 是当前肘部方向到 preferred direction 的有符号角度，不代表已经执行了 elbow 控制。

## 5. 测试覆盖

新增 `test_anthropomorphic_elbow`，覆盖：

- 左右侧使用相反的 base_link Y outward 方向；
- shoulder-wrist 平面投影半径保持正确；
- 投影退化时按 previous、home 顺序 fallback；
- 不生成随机方向；
- shoulder-wrist 轴过短时安全返回 invalid；
- 真实 S4 teleop home 左右方向 finite 且 outward-biased。

完整回归：

```text
9 test programs
48 tests
0 errors
0 failures
```

## 6. Phase 5.1 已完成

已补充 elbow 几何的离线扰动和 flip 专项测试，确认：

- 左右侧 preferred direction 始终符号正确；
- wrist target 变化时方向连续；
- 接近肩腕轴退化时安全关闭；
- 不会因姿态任务变化而把 elbow direction 突然翻转。

这些测试均已通过。详细记录见：

- docs/hierarchical_dik_phase5_1_implementation.md

下一步 Phase 5.2 才把 elbow swivel 作为完整 wrist rank=6 时的 1D null-space 次任务接入，并保留 position/orientation 双重退化回退。
