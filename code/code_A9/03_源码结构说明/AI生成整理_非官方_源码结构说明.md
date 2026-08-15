# A9 源码结构说明

## 核心目录

| 目录 | 作用 | 阅读建议 |
| --- | --- | --- |
| `Main` | 启动、版本号和主入口 | 先看启动顺序与固件版本 |
| `MeasurementSystem` | 传感器测量、状态和定位相关 | 结合 GPS、光流、激光和 IMU 阅读 |
| `ControlSystem` | 姿态、位置、角速度和控制链路 | 高风险，先读不改 |
| `Modes` | 飞行模式、任务和自定义模式 | 上层开发的主要入口 |
| `Communication` | MAVLink、串口和通信处理 | 嵌入式与 AI 的接口入口 |
| `Drivers` | UART、CAN、I²C、PWM、SD 和外设驱动 | 确认端口和硬件映射 |
| `FreeRTOS` | 任务、调度和系统运行环境 | 理解任务并发关系 |
| `HAL_Library` | STM32 HAL | 通常不直接修改 |
| `CMSIS` | Cortex/设备支持 | 通常不直接修改 |
| `Objects` | 编译输出、HEX、AXF 等 | 不作为源码提交重点 |

## 关键链路

```text
Drivers / Sensors
      ↓
MeasurementSystem
      ↓
Modes / Communication
      ↓
ControlSystem
      ↓
PWM output / Motors
```

## 源码状态

已解压归档的主要版本：20231029、20241114、20250615，以及 20250526 巡线版本。其他版本的压缩包原件保留在 `D:\Flight`，本上传包优先保留可直接检索的解压源码。

## 编译注意

- 原工程目标为 STM32H743VITx。
- 原工程记录使用 ARM Compiler 6 系列和 STM32H7 DFP 3.1.0。
- 当前团队主要使用 CLion/Codex/Vibe 编辑和理解代码；构建前要确认 CMake/Keil 工程是否已经完成等价配置。
- `Objects`、`Listings` 和调试数据库属于生成或工具文件，应谨慎提交。
