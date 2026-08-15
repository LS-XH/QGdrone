# STM32 工具说明

## 当前工具分工

- CLion/Codex/Vibe：代码阅读、修改、脚本和协作。
- Keil μVision：原 A9 工程兼容验证；不是团队日常编辑器。
- STM32CubeProgrammer：ST-LINK/SWD、HEX 校验和芯片级工具。
- STM32CubeMX/CubeIDE：通用 STM32 学习和参考，不作为 A9 原工程的默认重建工具。

## A9 工程兼容记录

- 目标：STM32H743VITx。
- 原工程：ARM Compiler 6 系列。
- 原工程记录：STM32H7 DFP 3.1.0。
- 编译成功、下载成功和飞行安全是三个不同结论，不能混为一谈。
