# XRoboToolkit Quest 客户端

本目录包含为 Meta Quest 3 构建的 XRoboToolkit 客户端：

```text
文件：XRoboToolkit-Quest-1.0.1.apk
Android 包名：com.xrobotoolkit.client.quest
versionName：1.0.1
versionCode：1
最低 Android SDK：32
目标 Android SDK：35
CPU ABI：arm64-v8a
SHA256：fa9fd5036a92f6377db77838cc6098cf3b42890d29a9e39b018f7f7706843ddf
```

该 APK 于 2026-08-25 使用官方
[XRoboToolkit-Unity-Client-Quest](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest)
源码和 Unity `2021.3.45f2` 构建。APK Signature Scheme v2 校验通过，但使用 Android Debug
证书签名，适合开发部署和侧载，不应当作正式商店发布签名包。

安装：

```bash
adb devices -l
adb install -r -g XRoboToolkit-Quest-1.0.1.apk
```

校验：

```bash
sha256sum XRoboToolkit-Quest-1.0.1.apk
```

如果 Quest 上已有同包名但签名不同的版本，覆盖安装会返回
`INSTALL_FAILED_UPDATE_INCOMPATIBLE`。确认不需要保留旧应用数据后，可先执行：

```bash
adb uninstall com.xrobotoolkit.client.quest
adb install -g XRoboToolkit-Quest-1.0.1.apk
```

卸载会清除该应用保存的服务器地址和其他设置。
