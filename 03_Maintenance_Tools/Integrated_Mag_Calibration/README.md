# Glovity 磁力计维护校准工具

本目录保留正式磁力计维护校准工具。

研发人员已经在出厂前完成初始磁力计校准。用户不需要在首次开机时执行磁力计校准。

## 什么时候需要校准

建议在以下情况执行：

- 每半个月到一个月做一次例行维护。
- 手套在固定姿态下方向持续漂移。
- 使用过程中出现明显数据偏移或方向不稳定。
- 更换使用环境后方向明显不稳定。
- 技术人员要求执行维护校准。

## 使用前准备

电脑需要能运行 Python，并安装串口库：

```powershell
pip install pyserial
```

## 常用命令

进入工具目录：

```powershell
cd 03_Maintenance_Tools\Integrated_Mag_Calibration
```

只扫描当前连接设备：

```powershell
python .\integrated_mag_calibrate.py --auto --scan-only
```

校准左手：

```powershell
python .\integrated_mag_calibrate.py --auto --side left --duration 35
```

校准右手：

```powershell
python .\integrated_mag_calibrate.py --auto --side right --duration 35
```

左右手依次校准：

```powershell
python .\integrated_mag_calibrate.py --auto --all --duration 35
```

## 操作动作

1. 按工具提示开始。
2. 在约 35 秒内缓慢旋转手套，让手套覆盖多个方向。
3. 等待工具显示完成并保存。
4. 保存完成前不要断开连接或关闭设备。

