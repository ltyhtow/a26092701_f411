# 双轮自动编码器测试

使用板载 Type-C 虚拟串口时，先按 [USB CDC 指南](USB_CDC_GUIDE.md)将 PA11 电机线移到 PB5，烧录 `UsbWheelTest`，运行脚本时加 `--usb-cdc`。下文接线表和 `WheelTest` 预设对应原 USART2 方案。

若上电进入 HardFault，参见 [故障现场记录与 CLion 复查](HARDFAULT_DEBUG.md)。当前固件可直接监视 `g_fault_snapshot`；务必确认调试配置指向 WheelTest 对应的 ELF。

这是单独的 `WheelTest` 固件和 PC 客户端：车轮悬空后，由电脑启动一次低占空比转轮流程，自动记录编码器，取代手推。不开平衡闭环、不用 IMU 姿态调节电机，也不修改保存的 PID 参数。软件已做模拟测试；是否实际转动及方向正确仍由实车验证，程序结束不代表编码器通过验收。

## 接线和构建

USB 转串口使用 **3.3V TTL 信号**，115200、8N1：

| USB 转串口 | F411 |
| --- | --- |
| RX | PA2，USART2 TX |
| TX | PA3，USART2 RX |
| GND | GND，共地 |

本测试必须接 TX 和 RX，因为 MCU 需要收到 START、心跳和 STOP。适配器 VCC 不接开发板 3V3，不能把 5V 接入 3V3，也不要用串口模块给电机供电。开发板和驱动板使用已恢复正常的供电方案；电机需要它自己的动力供电。只连接表内三条串口线，DTR/RTS 不接复位或电机使能。

在项目根目录运行：

```powershell
cmake --preset WheelTest
cmake --build --preset WheelTest
```

烧录 `build\WheelTest\a26092701_f411.hex`，不要选 `EncoderTest` 或普通 `Debug`。本次 Agent 不打开串口、不烧录、不启动实车。

## 执行一次测试

固定车体、保证两轮悬空且周边无缠绕物，保留能够切断电机动力电源的操作空间。确认串口工具未占用端口。

```powershell
python -m pip install pyserial
[System.IO.Ports.SerialPort]::GetPortNames()
python tools\wheel_test.py --port COM7 --csv build\wheel-test\run01.csv --raw-output build\wheel-test\run01.bin
```

把 COM7 改成实际串口号。**这条命令会启动电机测试**：收到有效 `WAIT`、编码器就绪且增量有效、电机未锁存的状态帧后，自动发送一次 START。主机等待 COUNTDOWN 回执；未收到回执会失败，不会重发 START。固件上电自己不启动电机。

| 顺序 | 阶段 | 时长 | 目标 PWM（左 / 右） |
| --- | --- | --- | --- |
| 1 | COUNTDOWN，倒数 | 5 秒 | 0 / 0 |
| 2 | LEFT_FORWARD，左轮正向 | 3 秒 | +1200 / 0 |
| 3 | REST_LEFT，停轮 | 1 秒 | 0 / 0 |
| 4 | RIGHT_FORWARD，右轮正向 | 3 秒 | 0 / +1200 |
| 5 | REST_RIGHT，停轮 | 1 秒 | 0 / 0 |
| 6 | BOTH_FORWARD，双轮正向 | 3 秒 | +1500 / +1500 |
| 7 | REST_FORWARD，停轮 | 2 秒 | 0 / 0 |
| 8 | BOTH_REVERSE，双轮反向 | 3 秒 | -1500 / -1500 |
| 9 | DONE，停机锁存 | 持续 | 0 / 0 |

正/反向是软件命令方向，尚未保证等于车体实际前进/后退方向。输出每 10ms 最多变化 50；PWM 频率约 20kHz，1200 对应约 24%，最高 1500 对应约 30%。单次固件流程从 START 开始约 21 秒，结束后 PC 再采集 1 秒停机数据并退出。低占空比未克服电机死区时，可能不转；不要直接以此认定编码器坏了，也不要在无人观察时提高输出。

主机每 200ms 发送二进制心跳。固件 750ms 未收到心跳会锁存停机；主机超过 1 秒没有**新的有效 0x88 状态帧**会停止测试，持续收到编码器帧也不能代替状态确认。反复收到同一时间戳或旧状态同样不会延长超时。异常后不会自动恢复运行。

`Ctrl+C` 会尽力发送 STOP 并关闭串口，进程返回 130。断线时 STOP 可能发不出去，控制台会明确报告；“STOP 已写入串口”也不等于板端已经确认停机。需要立即停轮时可以切断电机动力电源。

再次测试应确认停机、复位专用固件，并换一组输出文件名。运行中、DONE 或 ABORTED 的固件不会被客户端自动重新启动。默认主机总超时 45 秒，可用 `--timeout 45` 显式设置。

## 记录和判断

固件标称每 50ms（20Hz）采样，并在阶段边界额外采样；实际间隔以每帧 `sample_period_ms` 为准。脚本每 0.5 秒显示最新阶段、左右增量和累计计数，生成：

- `run01.csv`：每个通过 LwPKT 校验和版本检查的编码器载荷，以及有效标志、原始计数、A/B 电平等全部现有字段。
- `run01.status.csv`：每个通过校验且 MCU 时间戳前进的 WheelTest 状态、阶段、PWM、心跳年龄、设备/主机时间和终止原因。
- `run01.bin`：串口读取到的全部原始字节，包括坏帧和未知帧。可以用于离线复查。

文件只新建，不覆盖已有结果。编码器 CSV 的 `latest_status_*` 是**接收顺序上最近的状态**，不是同一时刻采样：`encoder_minus_status_ms` 为两个设备时间戳之差（有符号、允许回绕），`status_receive_age_ms` 为最近有效状态在主机上的接收年龄。准确对照阶段时结合独立状态 CSV，不要把这些字段当成同步电流/PWM测量。

检查重点：

1. 左轮单独输出时左侧增量连续变化，右轮应基本不计数；右轮阶段相反。如果计数通道相反，先检查左右接线/映射。
2. 两轮正向阶段都应有稳定非零计数；反向后每轮增量符号应各自反转。结合实际轮子方向核对正负定义，不能只靠 PWM 符号认定方向正确。
3. 停轮阶段允许惯性带来的短暂增量；轮子实际停稳后，增量应恢复接近 0。观察 `sample_errors`、`tx_dropped` 与有效位。
4. 轮子实际转动而对应计数始终为 0，才重点检查编码器供电、A/B 接线、输入引脚和定时器。肉眼不转时计数为 0 不足以诊断编码器。
5. 本测试是开环 PWM，两轮计数不同不一定是故障，也不能自动标定每圈计数、轮径或 PID。每圈计数仍需轮子标记与实际圈数作为参考。

退出码 0 只表示完整观察了 COUNTDOWN 至 DONE、收到编码器载荷且 DONE 原因正常、报告零 PWM 和停机锁存。客户端按设备时间核对每个阶段的开始/结束范围、阶段内经过时间、剩余时间，DONE 必须报告累计 21000ms、剩余 0；不靠主机收到数据所花的时间推测完整运行。ABORTED、阶段丢失、计时矛盾、CRC/版本异常导致超时、串口异常等返回非零。结束时 STOP 写入失败也返回非零。将三份记录交给 Agent 后，可以继续分析通道、方向和计数一致性。

## 二进制协议与离线回归

本轮已通过 **80/80 项 CTest、9/9 种固件构建配置**。其中 WheelTest 覆盖纯序列器、17 个任务场景、专用/旧停止命令路由、真实 C LwPKT fixture 和 PC 20 项回归（互通项无跳过）。固件 ELF 已核对存在 WheelTest 任务和电机驱动，未包含平衡任务、IMU 采样任务或参数服务启动入口；未对实车烧录或打开串口。

完整复验命令：`pwsh -NoProfile -File tools\run_static_checks.ps1 -IncludeDiagnostics`。

串口发送使用本项目 LwPKT 帧（AA / CMD / LEN / payload / CRC8 / 55），不发送 UTF-8 测试文本。固件沿用 `serial_protocol_try_send → lwpkt_write → LwRB → UART DMA`；主机复用 `tuner.build_frame` 和 `encoder_monitor.StreamParser`，CRC覆盖 CMD、LEN、payload。

- 命令 `0x06`，4 字节 `<BBBB`：version=1、seq、action（START=1、STOP=2、HEARTBEAT=3）、reserved=0。
- 编码器 `0x87`，56 字节，保持 [原编码器格式](ENCODER_PUSH_TEST.md)。
- 状态 `0x88`，32 字节 `<BBBBIIhhIHHII`：version、seq、phase、reason、timestamp_ms、phase_elapsed_ms、left_pwm、right_pwm、heartbeat_age_ms、flags、reserved、remaining_ms、test_elapsed_ms。
- flags：bit0 编码器就绪、bit1 电机停机锁存、bit2 编码器增量有效、bit3 已收到心跳；phase 为上表顺序 1～9，WAIT=0、ABORTED=10；reason：NONE=0、USER_STOP=1、LINK_TIMEOUT=2、SENSOR_ERROR=3、DRIVER_FAULT=4、DEADLINE=5。

无需硬件的主机回归：

```powershell
python -m unittest discover -s tools -p test_wheel_test_client.py -v
```

测试中的串口和时间均为替身，覆盖完整流程、提前 DONE/设备计时不一致、阶段边界、坏 CRC、错误版本、旧/重复状态、非 WAIT 拒绝启动、丢失 ACK、心跳与状态超时、掉线、Ctrl+C、部分写入、文件保护和 CSV/原始数据留存。完整静态检查还通过 `WHEEL_PROTOCOL_FIXTURE` 指向真实 C codec → LwPKT → LwRB 生成的 101 字节样本，验证 Python 的三种命令逐字节一致、REVERSE/DONE 的全部字段以及任意分片解析；未指定该环境变量时，仅此互通测试跳过。
