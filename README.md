# SO101 + PICO 4 遥操作

本目录是从个人开发项目中整理出的独立源码副本，包含 PICO 透视网页、SO101 位姿与翻滚控制、夹爪接触保持、自动工作区准备，以及 STS3215 串口后端。它不包含开发者本机的电机标定、控制板序列号、PICO 会话令牌、运行日志或训练数据。

默认控制方式为“坐标 + 独立翻滚”：手柄位置控制末端，手柄翻滚控制腕部自转；手柄俯仰、偏航不进入姿态目标。夹爪为扣一下抓取、再扣一下张开。夹爪接触保持由位置反馈推断，不是力传感器闭环。

## 软件

- Python 3.12（推荐）
- pip install -r requirements-pico-teleop.txt
- 真机串口使用 pyserial（已列入依赖）；PICO 设备调试可选 Android Platform Tools（ADB）
- SO101 网页模型使用 MuJoCo XML/STL。模型原始来源和许可见 assets/so101/README.md 与 assets/so101/LICENSE。
- `configs/pico-sts3215-reference.json` 是软件仿真用的通用、未标定舵机响应参考，不是实机标定或性能承诺。
- Feetech SDK 固定版本及上游许可见 vendor/ftservo/source.json、vendor/ftservo/LICENSE。

## 启动

- 仿真/浏览器预览：python -m scripts.pico_teleop
- 内存舵机软件链路：python -m scripts.pico_teleop --virtual-hardware
- 真机入口：先完成本机标定和供电/工位核对，再运行 python -m scripts.pico_teleop --real --debug。默认配置模板不能直接驱动机械臂。

网页由本机服务提供。服务为每次会话随机生成令牌；不要把运行 URL、artifacts/ 或 *.local.json 上传到公开仓库。

## 本机配置

首次整理提供 `configs/*.example.json`，其中没有有效的板卡指纹、零位、关节方向或手柄参考，因此模板不应被视为可运行的真机标定。复制对应模板为 `.local.json` 后，使用自己的设备信息及校准流程重新生成有效配置。`.gitignore` 默认排除这些本机文件。

SO101 机构模型以其上游文件许可为准；其许可与本项目 MIT 许可分开。发布前应保留各第三方源文件的版权和许可声明。
