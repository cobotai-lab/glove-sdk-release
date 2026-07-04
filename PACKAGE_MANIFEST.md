# Package Manifest

版本日期：2026-07-04

## Included

| 路径 | 内容 |
|---|---|
| `01_Software/glove-sdk` | 软件主程序及其运行依赖 |
| `02_Device_Flasher/Glovity_Flasher.exe` | 三设备正式固件烧录与配对工具，内嵌左手、右手、接收器固件 |
| `03_Maintenance_Tools/Integrated_Mag_Calibration/integrated_mag_calibrate.py` | 正式磁力计维护校准工具 |
| `README.md` | GitHub 首页使用者手册 |
| `Glovity 使用者操作手册_v20260704.4.docx` | 正式 Word 使用者手册 |

## Excluded

| 类别 | 原因 |
|---|---|
| Arduino 工程源码 | 面向用户发布包不开放设备源码 |
| 5Hz 调试固件 | 非正式产品流程 |
| 纯文本调试固件 | 非正式产品流程 |
| 旧版独立磁力计校准 | 已由正式维护校准工具替代 |
| 本地三串口链路测试工具 | 开发调试工具，不进入用户发布包 |
| Reference 目录 | 开发参考资料，不进入用户发布包 |
| 旧版说明文档 | 避免用户看到冲突流程 |
