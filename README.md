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

连接 SO101 控制板、六个电机和对应电源。安装好 PICO USB 调试所需的 Android Platform Tools，并在头显中允许 USB 调试。启动真机页面：

```powershell
.\.venv-so101-hardware\Scripts\python.exe -m scripts.pico_debug --real
```

在电脑打开的遥操作页面中选择 LeRobot 导出的 SO101 标定 JSON，点击「导入标定文件」。程序会只读识别唯一的六轴控制板，并检查电机编号、偏移、限位和供电读数。若检查不匹配或发现多个控制板，导入会停止并显示原因。

导入成功后，确认机械臂已固定、空载且工作区清空，勾选工位确认并点击「按当前姿态接管电机」。接着戴上 PICO，在透视页面把右手柄竖直、松开握持键和扳机，短按 A 记录手柄参考；松开 A 后再短按一次授权。按住握持键移动机械臂，松开保持，按 B 停止。

标定与自动生成的运行参考保存在本机 `artifacts/pico-teleop`，不会提交到 Git。项目采用 MIT 许可；SO-ARM101 模型和 Feetech SDK 的许可见各自目录。
