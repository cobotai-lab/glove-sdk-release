# Glovity 使用者 README

版本日期：2026-07-01

## 1. 你需要使用哪些东西

正式使用通常只需要：

| 设备/软件 | 作用 |
|---|---|
| 左手手套 | 采集左手 IMU 数据 |
| 右手手套 | 采集右手 IMU 数据 |
| 无线接收器 | 接收无线数据并连接电脑 |
| `glove-sdk` 程序 | 显示 手部骨骼可视化界面 |

推荐固件已经放在：

```text
02_Firmware/30Hz_Product
```

软件主程序在：

```text
glove-sdk-release
```

该文件夹包含可直接运行的 `glove-sdk` 程序，无需安装 Python。手部模型文件已内置，无需额外下载。电脑需要 NVIDIA GPU + CUDA 驱动，不支持纯 CPU 运行。

## 2. 正常启动流程

1. 给左右手手套上电。
2. 将接收器插入电脑 USB。
3. 确认电脑识别到接收器串口，例如 Linux 上的 `/dev/ttyACM0`。
4. 进入软件主程序目录：

```bash
cd glove-sdk-release
```

5. 根据下面命令启动 `glove-sdk` 可视化程序。

如果使用 Text 模式直连手套，左手示例：

```bash
./glove-sdk --port /dev/ttyACM0 --side left --text
```

右手示例：

```bash
./glove-sdk --port /dev/ttyACM0 --side right --text
```

如果使用接收器二进制链路，请以软件部门最终提供的启动参数为准。当前接收器参数为：

```text
baud = 2000000
frame = 168 bytes
```

## 3. 首次使用校准

每只手第一次使用前，需要做一次 SDK 姿态校准，约 30 秒。

校准左手：

```bash
./glove-sdk --calibrate --port /dev/ttyACM0 --side left --text
```

校准右手：

```bash
./glove-sdk --calibrate --port /dev/ttyACM0 --side right --text
```

按照命令行提示完成三步动作：

1. 手掌朝下水平，手指伸直自然张开，按 Enter。
2. 手腕向上转 90 度，指尖朝上，按 Enter。
3. 手腕向下转 90 度，指尖朝下，按 Enter。

校准完成后会生成对应的校准文件，例如：

```text
imu_calibration_left.npy
imu_calibration_right.npy
```

请不要删除这些文件。

## 4. 磁力计校准

当前推荐磁力计校准方式不需要重新烧录固件。请使用本交付包内的 USB 触发脚本。

左手校准：

```powershell
cd 03_Test_Debug_Tools\Integrated_USB_MagCalibration\Host_Tester
python .\integrated_mag_calibrate.py --port COM29 --duration 35
```

右手校准：

```powershell
cd 03_Test_Debug_Tools\Integrated_USB_MagCalibration\Host_Tester
python .\integrated_mag_calibrate.py --port COM26 --duration 35
```

操作步骤：

1. 运行命令后，按 Enter 开始。
2. 缓慢旋转手套约 35 秒，尽量覆盖不同方向。
3. 脚本会自动停止并保存。
4. 保存完成后，手套会自动回到正常工作模式。

系统可能显示：

```text
$GLOVITY,CAL,STARTED
$GLOVITY,CAL,COMPLETE
$GLOVITY,MODE,NORMAL
```

## 5. 程序界面说明

启动后窗口分为两部分：

| 区域 | 说明 |
|---|---|
| 左侧 | 3D 手部骨骼实时渲染，可用鼠标旋转、缩放、平移 |
| 右侧 | 参数控制面板 |

右侧常见控制项：

| 控件 | 作用 |
|---|---|
| `Flip Direction` | 翻转手指弯曲方向和坐标轴 |
| `Coordinate Preset` | 坐标系旋转预设 |
| `DIP Coupling` | DIP 关节跟随 PIP 的比例 |
| `Finger Splay` | 手指张开角度微调 |
| `Wrist Yaw Fix` | 手腕偏航修正 |
| `Reset Parameters` | 重置所有参数 |
| `FPS` | 实时帧率 |

## 6. 电量和自动关机

手套会自动监测电池电压。

如果电池电压连续低于 3.50V，手套会自动关机。关机前可能打印：

```text
battery low, auto power off
```

如果手套电池正在下降，并且 15 分钟内都没有连接到接收器，手套也会自动关机，以避免长时间空耗电。

如果手套插在电脑或充电设备上，电池电压通常不会下降，这种情况下不会因为“没有接收器”而自动关机。但电压低于 3.50V 时仍会保护关机。

## 7. 常见现象

| 现象 | 说明 |
|---|---|
| COM34 打开后像乱码 | 正常。接收器输出的是 168 字节二进制帧，不是文本 |
| 手套 USB 口有文本输出 | 正常。正式 30Hz 固件会保留 USB Text 调试输出 |
| 磁校准时正常数据暂停 | 正常。校准期间手套会暂停正常回传，保存后恢复 |
| 找不到校准文件 | 需要先做首次 IMU/骨骼校准 |
| 串口 Permission denied | Linux 下需要加入 `dialout` 用户组后重新登录 |
| 没有 GPU 无法启动 | 当前软件主程序需要 NVIDIA GPU + CUDA 驱动 |


## 8. 固件编译说明（开发者参考）

如需重新编译固件，请注意以下关键要求：

### 编译环境

- Arduino CLI 或 Arduino IDE 2.x
- ESP32 开发板支持包（esp32:esp32）

### 关键编译参数

所有 ESP32-S3 固件（手套和接收器）编译时 **必须** 添加 `CDCOnBoot=cdc` 参数，否则固件无法正常启动。

arduino-cli 编译命令示例：

```bash
# 手套固件（左手）
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc 02_Firmware/30Hz_Product/LeftHand_TX_USB30

# 接收器固件
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc 02_Firmware/30Hz_Product/Receiver_RX
```

### 接收器串口说明

接收器固件使用 `Serial0`（UART0 硬件串口）输出 2Mbps 二进制帧数据，而非 `Serial`（USB CDC）。
这是因为 ESP32-S3 的 `Serial` 默认映射到 USB CDC，不映射到 UART0 硬件引脚。

## 9. 不建议用户操作的内容

- 不要随意烧录 SDK 原始目录里的旧版示例固件。
- 不要删除校准生成的 `.npy` 文件。
- 不要用普通串口监视器解释 COM34 的二进制输出。
- 不要在磁力计校准过程中拔掉手套 USB。
