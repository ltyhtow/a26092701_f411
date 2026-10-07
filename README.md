# STM32F411 平衡车

板载 Type-C 虚拟串口使用 [USB CDC 指南](USB_CDC_GUIDE.md)：25 MHz 晶振、`UsbWheelTest` / `UsbEncoderTest` / `UsbDebug` 独立配置。**USB 固件要求将原 PA11 的右电机控制线移到 PB5**，PA11/PA12 专供 USB。

CLion 日常运行目标保持 `a26092701_f411`，共享运行配置默认选择 `UsbWheelTest`。按用户约定，后续完成并验证的任务自动提交到本地 Git；具体约定见 [AGENTS.md](AGENTS.md)。

悬空自动转轮及编码器记录使用 [WheelTest 指南](WHEEL_TEST_GUIDE.md)：专用固件上电等待，电脑端握手后执行单轮/双轮正反转，结束或失联锁存停机。

参数查询、停机调参、完成回执、异常状态与 AT24C08N 保存恢复见 [TUNING_GUIDE.md](TUNING_GUIDE.md)。EEPROM 默认关闭；`EepromDebug` 预设启用独立 I2C2（PB10/PB9），底层使用固定版本的 MIT 许可 LibDriver。

当前结构与验证结论见 [本次解耦记录](ARCHITECTURE_REFACTOR_20261007.md)。旧交接文档和审查报告保留历史背景，涉及路径、任务、供电状态的旧描述不能作为当前实现依据。

## 构建与验证

在工程根目录用 PowerShell 7，确保 ARM GCC、CMake、Ninja、Python 可用：

```powershell
cmake --preset Debug
cmake --build --preset Debug

cmake --preset EncoderTest
cmake --build --preset EncoderTest

pwsh -NoProfile -File tools\run_static_checks.ps1 -IncludeDiagnostics
```

验证入口包含 host 回归、正常 Debug/Release、EncoderTest、EepromDebug、UsbDebug/UsbWheelTest/UsbEncoderTest、PWM/GPIO 电机诊断、IMU UART 诊断和 IMU+电机组合构建，**不烧录、不打开实车串口**。验证产物在 `build/static-fix`，日常预设产物在 `build/<预设名>`。

固件每次构建会生成 GCC `.su` 栈用量报告，并自动检查 IMUFusion 已知 AHRS 调用链的栈预算（需要 Python）。这项检查覆盖本次 Debug 栈不足问题，不能代替完整调用链审查和实车验证；故障现场与复查命令见 [HardFault 记录](HARDFAULT_DEBUG.md)。

手推操作与数据记录见 [ENCODER_PUSH_TEST.md](ENCODER_PUSH_TEST.md)。正常固件默认禁止输出，需有效已校准姿态、稳定资格和显式使能。EncoderTest 锁存电机停止且忽略运动命令。供电已由用户确认恢复；新架构的实际波形、蓝牙链路和闭环效果仍需实车验证。

## 分层

- `middleware/`：标准 C 核心、领域契约、协议 codec/engine。没有 STM32/FreeRTOS include。
- `platform/freertos/`：任务、队列、锁、时间和失败钩子；FreeRTOSConfig 也在此。
- `platform/stm32f411/`：电机、编码器、IMU、UART DMA 硬件后端与板配置。
- `platform/include/`：标准类型的板级接口。
- `app/`：启动装配、协议到领域的转换；`app/diagnostics/` 为整车诊断。
- `Core/`：CubeMX 板级初始化、中断与目标运行库；不再放协议服务或业务任务。
- `cmake/modules.cmake`：固件与主机测试共用的核心库目标。

协议保持 `显式小端 payload → LwPKT → LwRB → UART DMA / USB CDC`。传输后端由构建预设选择，协议说明见 [middleware/serial/README.md](middleware/serial/README.md)。禁止将 UTF-8 调试文本直接混入发送缓冲。

边界复核：

```powershell
python tools\check_architecture.py --compile-commands build\static-fix\firmware-Debug\compile_commands.json
python tools\audit_architecture.py
```

换 MCU 时实现新的板级后端；换 RTOS 时替换任务/同步适配，并核对使用该 RTOS 同步机制的硬件传输后端。算法与线协议核心无需引入对应平台头。CubeMX 重生成之后必须保留当前自定义模块目标、配置位置和应用接线，并完整运行验证入口。
