# Qiling 双臂 XR 遥操（`xrtele` 分支）

本分支用于使用 Meta Quest 3 手柄遥操 Qiling 人形机器人的双臂和 O6 灵巧手，包含：

- XRoboToolkit Quest 3 控制器输入；
- Pinocchio + ProxSuite C++ 分层差分 IK；
- MuJoCo 双臂遥操仿真；
- ROS 2 Humble 真机状态/命令桥接；
- 双臂安全过渡点、home 和 Grip 离合器；
- 真机 Trigger 控制左右 O6 开合。

当前遥操链路使用 Quest 3 的控制器位姿和按键，不负责把 MuJoCo 或真机相机画面发送到头显。

## 1. 安全要求

真机模式会发布 26 维 MIT 命令并自动执行“当前姿态 → 过渡点 → home”。首次运行必须：

1. 清空机器人双臂、身体和桌面周围的障碍物；
2. 确认双臂关节索引、方向和 URDF 与真机一致；
3. 准备硬件急停，并安排一人随时监护；
4. 停止其他 `/human_lower_command` 和 `/handscmd` 发布者；
5. 先完成 MuJoCo 仿真，再连接真机。

## 2. 系统要求

- Ubuntu 22.04 x86_64；
- ROS 2 Humble；
- Meta Quest 3 和左右 Touch 控制器；
- XRoboToolkit PC Service 1.0.0；
- 仿真主机建议连接 HDMI/DP 显示器，并在本地图形桌面中运行；
- 真机 PC 与 Quest 3 位于同一局域网。

控制程序为 ROS 2/C++，不要在 Conda 环境中启动或编译。

## 3. 克隆 `xrtele` 分支

```bash
git clone --branch xrtele --single-branch \
  https://github.com/liujun0808/qiling_television.git
cd qiling_television
chmod +x src/scripts/*.sh
```

## 4. 安装 ROS 2 和工程依赖

先按 [ROS 2 Humble 官方文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
安装 Ubuntu 22.04 对应的 ROS 2 Humble，然后执行：

```bash
./src/scripts/install_xrtele_dependencies.sh
```

该脚本安装编译工具、Pinocchio、ProxSuite、CycloneDDS、MuJoCo GUI 系统依赖和 ADB，
但不会安装机器人底层 SDK，也不会安装 XRoboToolkit。

## 5. 安装 XRoboToolkit PC Service

仿真主机和真机 PC 都需要安装。先从
[XRoboToolkit PC Service v1.0.0](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/tag/v1.0.0)
下载 Ubuntu 22.04 amd64 安装包，然后执行：

```bash
sudo apt install ./XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

安装后必须存在：

```text
/opt/apps/roboticsservice/runService.sh
/opt/apps/roboticsservice/SDK/include/PXREARobotSDK.h
/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so
```

检查：

```bash
ls -l /opt/apps/roboticsservice/runService.sh
ls -l /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so
```

工程中的 C++ Quest 适配器直接链接该 SDK，因此必须先安装 PC Service，再编译工程。

## 6. 在 Quest 3 安装 XRoboToolkit App

Quest 3 客户端基于官方
[XRoboToolkit-Unity-Client-Quest](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest)
构建。本分支已经附带可直接侧载的 Quest APK：

```text
third_party/xrobotoolkit/quest/XRoboToolkit-Quest-1.0.1.apk
```

该文件包名为 `com.xrobotoolkit.client.quest`，SHA256 为：

```text
fa9fd5036a92f6377db77838cc6098cf3b42890d29a9e39b018f7f7706843ddf
```

### 6.1 开启开发者模式和 USB 调试

1. 在 Meta 账号中启用开发者组织；
2. 在 Meta Horizon 手机 App 中为 Quest 3 开启 Developer Mode；
3. 重启 Quest 3，通过 USB 连接 Ubuntu 主机；
4. 在头显中允许 USB 调试，并勾选始终允许该主机。

检查连接：

```bash
adb devices -l
```

设备状态必须为 `device`；如果是 `unauthorized`，重新在头显中确认授权。

### 6.2 安装 APK

```bash
(cd third_party/xrobotoolkit/quest && sha256sum -c SHA256SUMS)
adb install -r -g third_party/xrobotoolkit/quest/XRoboToolkit-Quest-1.0.1.apk
```

`-r` 表示覆盖升级并尽量保留应用数据，`-g` 表示安装时授予可授予权限。安装后在 Quest 3
的“未知来源/Unknown Sources”中打开 `XRoboToolkit-Quest`。

此 APK 使用 Android Debug 证书签名，适合开发侧载。如果头显中已有同包名、不同签名的
版本，`adb install -r` 会报告 `INSTALL_FAILED_UPDATE_INCOMPATIBLE`。确认不需要保留旧应用
数据后，可执行：

```bash
adb uninstall com.xrobotoolkit.client.quest
adb install -g third_party/xrobotoolkit/quest/XRoboToolkit-Quest-1.0.1.apk
```

卸载会清除 XRoboToolkit Quest App 保存的 PC 地址和设置。APK 的构建来源和校验信息见
[`third_party/xrobotoolkit/quest/README.md`](third_party/xrobotoolkit/quest/README.md)。

## 7. 编译本工程

```bash
./src/scripts/build_xrtele.sh
```

该脚本只编译遥操所需包，避免工作区内其他目录影响构建。完成后可检查：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix qiling_kinematics
ros2 pkg prefix qiling_kinematics_real
ros2 pkg prefix mujoco_simulator
```

## 8. 启动 XRoboToolkit 与 Quest 3

在运行遥操的 Ubuntu 主机上启动无 GUI 后台服务：

```bash
./src/scripts/start_xrobotoolkit_real.sh
```

确认两个进程存在：

```bash
pgrep -af '/opt/apps/roboticsservice/RoboticsServiceProcess'
pgrep -af 'RobotLinuxDemo.x86_64'
```

然后在 Quest 3 中：

1. 打开 `XRoboToolkit-Quest`；
2. 选择或手动输入运行 PC Service 的 Ubuntu 主机 IP；
3. 确认状态显示已连接/`WORKING`；
4. 打开 `Controller Tracking`；
5. 打开 `Send`；
6. 本项目不要开启 `Switch w/ A Button`，避免右手 A 键改变发送状态。

官方要求 Quest 与 PC 位于同一网络，并先运行 PC 端服务/3D 应用，再打开头显客户端。

停止服务：

```bash
./src/scripts/stop_xrobotoolkit_real.sh
```

## 9. MuJoCo 仿真遥操

推荐在带 HDMI/DP 显示器、键盘和鼠标的 Ubuntu 22.04 主机上运行。MuJoCo `simulate`
是图形程序，不推荐通过纯 SSH、无显示器或不稳定的远程桌面进行首次仿真遥操。

启动顺序：

1. 启动 XRoboToolkit PC Service；
2. Quest 3 连接并打开 `Controller Tracking` 和 `Send`；
3. 一键启动 MuJoCo、解除暂停并启动仿真 IK：

```bash
./src/scripts/start_sim_teleop.sh
```

机器人从全零状态自动经过安全过渡点到达仿真 home。日志中应依次出现：

```text
MOVE_TO_TRANSITION
SETTLE_AT_TRANSITION
MOVE_TO_HOME
SETTLE_AT_HOME
COMPLETE
```

home 完成后，左右 Grip 都要先松开一次。之后按住左/右 Grip，才会控制对应手臂。
按 `Ctrl+C` 会停止仿真遥操和 MuJoCo，XRoboToolkit PC Service 需用独立停止脚本关闭。

如果暂时没有 Quest，可使用内置模拟手柄输入：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mujoco_simulator simulate.launch.py
```

另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /unpause_mujoco std_srvs/srv/Empty '{}'
ros2 launch qiling_kinematics xr_teleop_demo.launch.py
```

## 10. 真机遥操

真机 PC 上完成前述依赖、PC Service 和工程编译。机器人底层 SDK 的安装和启动不属于本脚本
管理范围，必须先由用户按机器人文档启动，并确保它能够发布/接收 Qi DDS 数据。

设置 ROS 网络。默认 Domain ID 为 `16`，需要修改时在启动命令前覆盖：

```bash
export ROS_DOMAIN_ID=16
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

完整顺序：

1. 启动机器人 SDK；
2. 启动 XRoboToolkit PC Service；
3. Quest 3 连接真机 PC，打开 `Controller Tracking` 和 `Send`；
4. 清空双臂周围环境并准备急停；
5. 启动真机遥操：

```bash
./src/scripts/start_real_teleop.sh
```

脚本会按顺序执行：

```text
topic_convertor
  → 等待 /human_lower_state
  → qiling_kinematics_real
  → 当前实测姿态
  → 安全过渡点
  → home
  → 遥操
```

30 秒内没有收到 `/human_lower_state` 时，脚本会退出且不会启动 IK。可临时延长等待时间：

```bash
QILING_STATE_WAIT_SEC=60 ./src/scripts/start_real_teleop.sh
```

按 `Ctrl+C` 会停止真机遥操和 `topic_convertor`，不会命令机器人回到全零位。

## 11. 手柄控制

- 左 Grip：按住时遥操左臂，松开后左臂保持；
- 右 Grip：按住时遥操右臂，松开后右臂保持；
- 左 Trigger（真机）：按住闭合左 O6，松开张开；
- 右 Trigger（真机）：按住闭合右 O6，松开张开。

当前 MuJoCo 遥操包只接管双臂，Trigger/O6 命令由真机包的独立 `/handscmd` 适配器处理。

Grip 按下瞬间使用“当前机器人末端位姿 + 当前手柄位姿”重新建立锚点，因此可以多次松开、
调整手柄位置后再按下，不应产生末端目标跳变。

## 12. 运行检查与排错

检查 Quest 数据：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /xr/left_controller_pose
ros2 topic hz /xr/right_controller_pose
ros2 topic echo /xr/controller_joy --once
```

检查真机状态和命令：

```bash
ros2 topic hz /human_lower_state
ros2 topic info -v /human_lower_command
ros2 topic info -v /handscmd
```

XRoboToolkit 日志：

```bash
tail -f "${XDG_RUNTIME_DIR:-/tmp}/qiling_xrobotoolkit/robotics_service.log"
tail -f "${XDG_RUNTIME_DIR:-/tmp}/qiling_xrobotoolkit/robot_linux_demo.log"
```

常见问题：

- 编译提示找不到 `libPXREARobotSDK.so`：先安装 XRoboToolkit PC Service；
- Quest 没有控制器数据：确认连接为 `WORKING`，并打开 `Controller Tracking` 和 `Send`；
- 真机一直等待状态：检查机器人 SDK、`ROS_DOMAIN_ID`、网卡、防火墙和 CycloneDDS；
- home 完成但手臂不跟随：先完整释放一次对应 Grip，再重新按住；
- MuJoCo 无法打开窗口：在有 HDMI/DP 显示器的本地图形会话运行并检查 `DISPLAY`。

更详细的 IK 和节点接口说明见：

- `src/qiling_kinematics/README.md`；
- `src/qiling_kinematics_real/README.md`；
- `src/mujoco_simulator/README.md`。
