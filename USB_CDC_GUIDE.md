# 板载 USB 虚拟串口

适用于用户已确认的 25 MHz 外部晶振 F411 开发板。USB 配置使用 TinyUSB 的 CDC ACM 设备类，电脑通过开发板 Type-C 数据口通信。线协议仍为显式二进制 payload → LwPKT → LwRB → USB CDC；不发送 UTF-8 调试文本。

## 先处理 PA11 冲突

当前 UART 接线将 PA11 用作右电机驱动输入 BIN2，而 F411 USB 必须使用 PA11（D−）、PA12（D+）。[ST 官方引脚表](https://www.st.com/resource/en/datasheet/stm32f411ce.pdf)确认 PB5 可作为 AF2/TIM3_CH2。用户已确认 PB5 空闲。

**烧录 USB 固件前，断电，将原接 PA11 的右电机控制线改接 PB5。PA11 不再连接电机驱动输入。**

| 信号 | 原 UART 配置 | USB 配置 |
| --- | --- | --- |
| 左 IN1 | PA8 | PA8 |
| 左 IN2 | PA9 | PA9 |
| 右 IN1 | PA10 | PA10 |
| 右 IN2 | PA11 | **PB5** |
| USB D− / D+ | 未启用 | PA11 / PA12，使用板载 Type-C 数据连线 |

USB 不启用 PA9 的 VBUS 检测或 PA10 的 ID 功能，这两个脚继续控制电机。USB 模式的紧急停机会关闭 TIM1 和 TIM3，并将四路电机输入拉低，不接管 PA11/PA12。若以后刷回原 UART 固件，需要同时恢复右 IN2 的 PA11 接线。

使用能传数据的 Type-C 线连接电脑。USB 仅负责开发板供电和数据；驱动板仍按现有方案接电机动力电源。

## 构建和烧录

工程根目录 PowerShell：

```powershell
cmake --preset UsbWheelTest
cmake --build --preset UsbWheelTest
```

烧录 **`build\UsbWheelTest\a26092701_f411.hex`**。CLion 使用对应 CMake profile 和同目录 ELF；普通 Debug/UsbDebug 发普通遥测，不执行这套自动转轮测试。

CLion 的运行入口统一为 **`a26092701_f411`**，共享配置在 `.run/a26092701_f411.run.xml`。`UsbWheelTest` 是 CMake 预设，不是另一个可执行目标。当前电脑已启用并选择 `UsbWheelTest - UsbWheelTest`（分别表示配置预设和构建预设），绑定原 ST-LINK 调试器；其他库仍作为编译依赖保留。

当前 CLion 用户设置已关闭自动生成运行配置，避免库重新填满运行菜单；这是 IDE 设置，会影响该 CLion 下其他项目的自动生成行为，不随 Git 同步。可在“设置 → 高级设置”恢复自动生成。修改前的本机配置备份在 `build/static-fix/clion-config-before/`。如果工具栏还显示旧配置，重新打开工程后再选 `a26092701_f411`。

其他独立配置：

| 预设 | 功能 |
| --- | --- |
| UsbWheelTest | 悬空转轮测试，收到 START 前保持 WAIT |
| UsbEncoderTest | 只测编码器，锁存禁止电机输出 |
| UsbDebug | 普通控制、遥测和调参，保持原显式使能条件 |
| WheelTest / EncoderTest / Debug | 原 UART 传输与原电机接线 |

USB 配置时钟为 HSE 25 MHz，PLL M=25、N=192、P=2、Q=4，CPU=96 MHz，USB=48 MHz。原 UART 配置保持 HSI 100 MHz。PWM 的 ARR=4999、占空比尺度和限幅保持一致，因此 USB 配置的 PWM 频率为 **19.2 kHz**，原 UART 为 20 kHz；RTOS 和 HAL 毫秒时基依实际时钟配置。

## 电脑端使用

烧录并正常启动后，在设备管理器的“端口”中查看新增的 USB 串行设备，或运行：

```powershell
python -m serial.tools.list_ports -v
```

COM 号由 Windows 分配，不一定还是 COM10。保持车轮悬空，将下面 COM12 替换为新端口：

```powershell
python tools\wheel_test.py --port COM12 --usb-cdc --csv build\wheel-test\usb01.csv --raw-output build\wheel-test\usb01.bin
```

`--usb-cdc` 在打开端口时置 DTR，表示电脑程序已打开 CDC 会话。原 UART 模式继续保持 DTR/RTS 为低。USB 的“115200”是兼容串口 API 的线编码，不是 USB 总线速率。脚本依然只在收到合法 WAIT、编码器就绪且输出为零时发送一次 START；随后约 21 秒完成流程。

只读编码器采集对应 UsbEncoderTest：

```powershell
python tools\encoder_monitor.py --port COM12 --usb-cdc --seconds 30 --csv build\wheel-test\usb-push01.csv
```

UsbDebug 的调参继续使用 `tools\tuner.py`；其 pyserial 打开方式默认置 DTR。例如 `python tools\tuner.py --port COM12 status`。保存 EEPROM 仍需要启用原 EEPROM 配置及完成接线，USB 本身不会启用存储。

CDC 关闭、USB 复位、断线和挂起会使旧会话失效；传输环和解析器半帧在任务侧清理，迟到的 TX 完成不能消耗新传输数据。重连不会重放离线积压的命令。转轮测试保持原 750 ms 心跳超时和终态锁存，不会因重连自动恢复。

为了连硬件端点中尚未完成的数据也一并取消，**关闭已打开的 CDC 端口，或在会话期间 USB 挂起，会触发 USB 重新枚举**：设备断开至少 100 ms 后再连接，COM 口可能短暂消失。等 Windows 中端口重新出现后再运行下一条命令；不要将这个现象误判为 MCU 重启。等待由非阻塞状态机完成，电机任务继续执行原有失联停机逻辑。

## Windows 已枚举但未出现 COM：ADB 驱动误绑定

2026-10-07 22:56 的实机记录确认父设备 `USB\VID_CAFE&PID_4001` 已正常配置并启动，使用 `usbccgp`。先前的“设备设置未迁移”消息不能单独判定当前设备失败。

只读查询进一步确认其 `MI_00` 子接口报告合法 CDC 兼容 ID `USB\Class_02&SubClass_02`，但实际绑定成 **Android ADB Interface**，服务为 `WinUSB`、驱动为 `oem92.inf`（原名 `android_winusb.inf`）。Windows 内置 `usbser.inf` 同样匹配，排名均为 `00FF2004`，但 ADB 包被选中。因此此时没有 COM 口，不需要重刷固件或改 VID/PID；该 ADB 包匹配的是整个 CDC 类，改 PID 也不能可靠规避。

在设备管理器对**这个板子的子接口**操作：

1. 找到 `Android ADB Interface`，在“属性 → 详细信息 → 硬件 ID”确认包含 `USB\VID_CAFE&PID_4001&MI_00`。
2. 选择“更新驱动程序 → 浏览我的电脑以查找驱动程序 → 让我从计算机上的可用驱动程序列表中选取”。
3. 选择 Microsoft 的 **USB 串行设备 / USB Serial Device**。如列表中没有，使用“从磁盘安装”，指定 `C:\Windows\INF\usbser.inf`。
4. 完成后在“端口（COM 和 LPT）”查看新增 COM 号；如 Windows 请求管理员授权，需要由本机用户完成。

只更换此 `MI_00` 接口的驱动，不要修改父级 USB Composite Device，也不需要删除整机 ADB 驱动包。CDC 使用 Windows 内置串口驱动，依据见 [Microsoft Usbser 文档](https://learn.microsoft.com/en-us/windows-hardware/drivers/usbcon/usb-driver-installation-based-on-compatible-ids)。

换驱动后可只读复查（不打开端口、不启动轮子）：

```powershell
pwsh -NoProfile -File tools\usb_status.ps1
```

预期子接口为 `Class: Ports`、`Service: usbser`、`DriverInf: usbser.inf`，名称含 COM 号。脚本退出码 0 表示串口驱动已绑定，1 表示板子存在但未找到串口驱动，2 表示板子当前未连接；这不等于帧通信或编码器测试已通过。

## 本次 COM10 超时的实际记录

`build/wheel-test/run01.bin` 共 1740 字节：按项目真实帧格式解析出 **34 个有效 0x81 普通遥测帧，没有 0x88 WheelTest 状态帧**，另有 8 次 CRC 错误和 3 次帧尾错误。它证明当时能接收部分有效串口数据，同时固件模式与脚本不匹配；不能据此宣布串口模块完全损坏或链路完全正常。脚本现在会在这个情形下提示选择 WheelTest/UsbWheelTest。

若没有出现新 COM 口，依次核对：已改开 PA11 电机线、使用数据线、实际刷入 UsbWheelTest、程序没有停在断点或错误处理、25 MHz HSE 正常启动。若出现 COM 口但没有 WAIT，核对 `--usb-cdc` 和固件预设，再保留新的原始记录；不要放宽状态心跳超时来掩盖通信问题。

实机现已确认能枚举父设备及 CDC 子接口；Windows 串口驱动正确绑定、持续传输、拔插后的停机，以及两轮/编码器表现仍需继续验收。Agent 未打开实车端口、烧录或发送电机命令。

## 软件验证记录

- `tools/run_static_checks.ps1 -IncludeDiagnostics`：93/93 项 CTest 通过，12 种固件配置构建通过，日志 `build/static-fix/usb-full-regression.log`。
- 额外使用 CLion 同版本 GCC 14.3.1 构建 UsbWheelTest 与 USB Release 均通过。
- USB 传输后端包含 9 组场景；真实 USB 包装层与外设/协议栈桩包含 7 组场景，覆盖 DTR、64 字节分包、尾包 ZLP、迟到完成、初始化失败、重复打开、取消端点和 USBRST/ENUMDNE 间隔。
- USB 电机引脚及停机新增 9 个场景，原 UART 电机和诊断测试继续通过。
- TinyUSB 固定版本 0.21.0（MIT），提交 `dae3f9a366bfcddbf9dcf1b48d7500286a849539`，保留原始许可证；见 `middleware/usb_device/third_party/tinyusb/UPSTREAM.md`。

`.ioc` 仍描述旧 UART 接线；USB 是 CMake 中的条件板级配置。CubeMX 重生成时需要保留这里的时钟、引脚、后端选择和 IRQ 接线，再运行完整验证。
