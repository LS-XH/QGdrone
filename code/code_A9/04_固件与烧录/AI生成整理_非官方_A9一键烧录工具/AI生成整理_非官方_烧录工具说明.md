# AI生成整理_非官方_ACFLY A9 一键烧录工具说明

> 这是项目协作工具的整理说明，不是博睿官方烧录工具说明。实际烧录前必须核对 A9 型号、固件来源和硬件连接。

## 入口

- `0-检查A9烧录环境.bat`：只检查 CLI 和设备列举，不烧录。
- `1-ST-LINK烧录A9.bat`：连接 A9 的 SWJ/SWD 和 ST-LINK 后使用。
- `2-USB烧录A9.bat`：只有 A9 进入 STM32 USB DFU/Bootloader 后使用。
- `3-拖拽HEX到这里.bat`：把 `.hex` 拖到文件上，再选择 ST-LINK 或 USB。

## 默认固件

无参数运行时，会从本仓库的 `04_固件与烧录/A9固件_来源未确认` 中选择修改时间最新的 `Firmware_A9_*.hex`。当前应优先人工确认：

```text
04_固件与烧录/A9固件_来源未确认/1.17.86/Firmware_A9_1.17.86_20260304_1.hex
```

如果要烧录源码构建出的 `ACFly.hex`，请先确认它来自 `02_A9源码` 中的 A9 工程，再通过拖拽入口选择。

## 安全限制

工具会拦截 K9、K9Teach、F570、BootLoader、AC-RID 和 Wheeltec 路径中的固件，也不会执行 mass erase、Read Protection、Option Bytes 或解锁操作。

烧录前会检查 Intel HEX 地址是否落在 A9 应用区 `0x08040000` 到 `0x081FFFFF`，并要求输入 `YES` 确认。

## USB 说明

A9 普通 USB 连接、GCS PRO 连接和 STM32 USB-DFU 不是一回事。USB 入口识别不到 `usb1` 时，不要反复插拔或尝试其他固件；改用已确认的 GCS PRO 升级流程，或使用 ST-LINK/SWD。

## 日志

日志默认保存在工具目录下的 `logs`：

```text
`logs`
```

本工具没有在没有硬件确认的情况下执行真实烧录。
