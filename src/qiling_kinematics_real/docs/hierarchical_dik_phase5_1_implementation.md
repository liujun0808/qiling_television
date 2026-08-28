# Hierarchical DIK Phase 5.1 实现记录

状态：已完成  
完成日期：2026-08-27

## 1. 本阶段范围

本阶段只验证 Phase 5.0 的 elbow 几何方向、退化处理和连续性，不将 elbow 几何结果接入任何控制 QP。

## 2. 新增安全语义

当当前 elbow 在 shoulder–wrist 轴上的投影半径小于 `min_elbow_projection_radius_m` 时，输出：

```text
current_direction_valid = false
valid = false
```

preferred direction 可以保留用于诊断，但不会被未来 elbow controller 当作可执行 target。这样避免在没有定义 swivel angle 的构型中生成伪造的 elbow 运动。

## 3. 新增测试

### 合成几何

- 左右侧 base_link Y outward 方向相反。
- 投影退化时 previous/home fallback 顺序正确。
- 肩腕轴过短时安全无效。
- 肘部投影半径过短时安全无效。
- shoulder–wrist 轴跨越 outward 投影退化点时 preferred direction 不翻转。

### 真实 S4 模型

- 使用 `s4_dual_arm.urdf` 和当前 teleop home。
- 左右臂各进行 80 个小幅关节扰动构型。
- 每一帧验证 preferred direction 与上一帧的点积为正。
- 每一帧验证左右 outward 方向符号保持正确。
- 每一帧验证输出 finite 且 elbow 几何有效。

## 4. 当前结论

Phase 5.0 的几何定义在合成输入和真实 S4 home/扰动输入下均连续。当前可以确认：

```text
left  preferred.y > 0
right preferred.y < 0
```

但这只证明了 elbow target generator 的几何和分支选择正确，不代表机器人已经开始主动向外移动肘部。

## 5. 回归结果

```text
9 test programs
51 tests
0 errors
0 failures
```

## 6. 下一阶段

Phase 5.2：把 elbow swivel 接入完整 wrist null-space：

1. 构造 characteristic-length scaled 的完整 6×7 wrist Jacobian。
2. 仅在 rank=6 且最小奇异值足够大时计算 1D null-space。
3. 将 swivel angle error 转换成 elbow secondary velocity。
4. 加入 joint centering、rest posture 和 smoothness 的弱偏好。
5. 检查 elbow 增量对 position 和 orientation 的退化。
6. 失败、降秩或接近奇异时回退到 Phase 4 pose output。
