# AI生成整理_非官方_A9 纯软件工具箱说明

> 工具源码来自项目协作目录，本文档为 AI 生成整理，不是厂家官方工具手册。

这是面向博睿/ACFLY A9 学习和开发准备的离线工具集合。当前包含：

- `simulate`：本机 UDP MAVLink 遥测仿真器，对 `COMMAND_LONG` 只返回模拟 ACK。
- `mission-check`：任务 JSON 的坐标、围栏、高度、速度、PWM 参数预检查。
- `log-report`：把博睿日志分析软件导出的 CSV 生成 HTML、PNG、CSV、JSON 报告。
- `firmware-diff`：比较两个已解压源码目录，或两个 HEX 文件的哈希和地址范围。
- `self-test`：运行 pytest 单元测试。
- `scan-demo`：模拟 AI 发送目标、扫描完成、危险、重规划、漏扫补扫和地图完成事件。

源码目录比较也可以把两个已解压目录拖到 `5-比较两个A9源码目录.bat` 上。

## 快速开始

在此目录打开终端。不要直接照抄其他电脑的绝对路径：

```powershell
python -m a9tools --help
python -m a9tools mission-check examples\mission_sample.json --html reports\mission_report.html
python -m a9tools log-report examples\flight_sample.csv --out reports\flight_sample
python -m a9tools scan-demo
python -m a9tools self-test
```

启动仿真器后，可让你的 MAVLink 学习程序连接 `udp://127.0.0.1:14560`。默认不连接真实串口，不执行真实解锁、起飞或烧录。

`scan-demo` 是 AI 组与嵌入式组的接口联调原型。它只模拟事件和状态转换，不代表 A9 官方报文，也不直接控制 A9。事件包括 `NEXT_TARGET`、`SCAN_DONE`、`COVERAGE_GAP`、`HAZARD`、`MAP_COMPLETE`、定位丢失、人工接管和链路丢失。

## 输入约定

任务文件顶层使用 `mission` 数组；单个任务点可使用 `TAKEOFF`、`WAYPOINT`、`LAND`、`RTL/RTH`、`HOVER`、`PWM`。带位置的点使用 `lat`、`lon`、`alt`，PWM 使用 `channel` 和 `value`。`geofence` 是可选矩形围栏。

日志报告当前接收 CSV。原始 `.aclog` 请先使用本机已有的博睿日志分析软件导出 CSV；工具不猜测私有二进制格式。

源码差异比较要求两个目录已解压。HEX 比较只做哈希、校验和地址范围分析，不写芯片。

## 安全边界

本工具箱用于学习、仿真、离线预检查和版本审计。A9 真机操作仍应以 A9 用户手册、GCS PRO、工作室规定和现场安全流程为准。示例中的高度/速度限制是项目预检查策略，不代表固件全部硬限制。
