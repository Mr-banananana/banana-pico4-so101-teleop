# Banana PICO 4 + SO-ARM101 遥操作

## 下载

```powershell
git clone https://github.com/Mr-banananana/banana-pico4-so101-teleop.git
cd banana-pico4-so101-teleop
```

## 安装（Windows）

需要 Python 3.12。打开 PowerShell，在项目目录运行：

```powershell
py -3.12 -m venv .venv-so101-hardware
.\.venv-so101-hardware\Scripts\python.exe -m pip install -r requirements-pico-teleop.txt
```

## 仿真试用

```powershell
.\.venv-so101-hardware\Scripts\python.exe -m scripts.pico_debug
```

要用内存舵机完整演示（不会连接或移动真机）：

```powershell
.\.venv-so101-hardware\Scripts\python.exe -m scripts.pico_debug --virtual-hardware
```

PICO 通过 USB 调试连接电脑需要安装 Android Platform Tools（adb），并在头显中允许 USB 调试。通过 PICO 浏览器打开页面时，使用启动器显示的头显地址。

## 驱动真机

先复制配置模板，并填写自己的板卡信息：

```powershell
Copy-Item .\configs\pico-hardware.example.json .\configs\pico-hardware.local.json
Copy-Item .\configs\pico-real-calibration.example.json .\configs\pico-real-calibration.local.json
Copy-Item .\configs\pico-control-reference.example.json .\configs\pico-control-reference.local.json
```

模板不能直接驱动机械臂；按页面提示完成实机标定和控制参考记录。

确认控制板、电源和六个电机已连接后，先做只读反馈检查：

```powershell
.\.venv-so101-hardware\Scripts\python.exe -m scripts.pico_hardware_check --read-motors
```

确认六轴反馈正常后启动真机页面：

```powershell
.\.venv-so101-hardware\Scripts\python.exe -m scripts.pico_debug --real
```

本机配置、运行日志和虚拟环境已由 `.gitignore` 排除。项目采用 MIT 许可；SO-ARM101 模型和 Feetech SDK 的许可见各自目录。
