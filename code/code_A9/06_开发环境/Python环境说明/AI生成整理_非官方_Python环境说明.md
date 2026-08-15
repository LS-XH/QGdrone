# Python 环境说明

原始资料中的 Python 虚拟环境不上传。协作者在 Python 3.12 中重新创建环境。

建议依赖：

```text
pyserial
pymavlink
numpy
pandas
matplotlib
opencv-python
pytest
```

现有协作工具位于 `06_开发环境/AI生成整理_非官方_A9软件工具`，包括协议模拟、任务检查、日志报告、源码比较和单元测试。

创建环境示例：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

A9 实际协议以项目中的交互协议为准；`pymavlink` 和 MAVSDK 不能假定自动支持全部 A9 自定义消息。
