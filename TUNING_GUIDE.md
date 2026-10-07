# F411 参数调试、异常回执与 AT24C08N 持久化

本次实现把参数修改、结果回执、异常状态查询和可选 EEPROM 保存连起来。`SET` 只更新经过校验的 RAM 参数；只有显式 `SAVE` 才写 EEPROM。没有参数指令会自动使能电机。当前实车还没有完成该链路验收，原先蓝牙串口数据无法解析的问题也不能靠软件回归认定已解决。

## EEPROM 总线与接线

**地址上可以与 MPU6050 共用 I2C；本工程采用独立 I2C2。** 现有 MPU6050 驱动使用 0x68/0x69，AT24C08 的 A2 低时使用 0x50～0x53，两者不冲突。不过共享同一个 HAL I2C handle 需要统一互斥和事务调度，EEPROM 访问及其故障处理不能拖延 200 Hz 姿态/平衡控制。独立 I2C2 让 MPU6050 保持当前 I2C1，参数存储由后台任务单独负责。

用户确认的实物型号是 **AT24C08N**。本实现按 AT24C08 的 1024 字节容量、16 字节写页和 bank 地址方式驱动。所核查的官方资料 `doc0180.pdf` 描述 AT24C08A 家族；其中的 5 ms 写周期规格不能直接证明实物“N”丝印的具体代际或真伪。适配层每页写后至少等待 12 ms，并让出 RTOS 执行时间，仍须实测读写验证。[Microchip/Atmel 官方家族资料](https://ww1.microchip.com/downloads/en/DeviceDoc/doc0180.pdf)

| 模块引脚 | 当前固件接线 | 说明 |
|---|---|---|
| VCC | 开发板正常 3.3 V | 模块上拉也随 VCC 供电 |
| GND | 开发板 GND | 共地 |
| SCL | PB10 | I2C2，AF4，100 kHz |
| SDA | PB9 | I2C2，AF9 |
| A2 | GND | 默认；设高时需同时修改 `EEPROM_AT24C08_A2=1` |
| WP | GND | 允许保存；接 VCC 时写保护，但仍可读 |
| A0、A1 | 可固定接地 | 对 AT24C08 家族不作为独立器件地址选择位 |

A2=0 的 7 位地址范围是 0x50～0x53，A2=1 是 0x54～0x57；这四个地址对应同一芯片的不同 256 字节 bank，不是四颗 EEPROM。HAL/LibDriver 回调使用左移后的地址，外部扫描工具通常显示 7 位地址。[官方地址与 WP 说明](https://ww1.microchip.com/downloads/en/DeviceDoc/doc0180.pdf)

用户提供的原理图标出 R2、R3 均为 **10 kΩ**，分别将 SCL、SDA 上拉到模块 VCC。接线按模块丝印核对，不从商品照片的朝向猜排针顺序；跳帽的高低端也需按实际板子确认。

```text
开发板正常 3V3 ─── 模块 VCC
                  ├── R2 10k ─── SCL ─── PB10 (I2C2)
                  └── R3 10k ─── SDA ─── PB9  (I2C2)
开发板 GND ─────── 模块 GND、A2、WP
```

固件不启用 MCU 内部上拉。先使用模块现有上拉与短线；是否调整阻值取决于实测上升沿和总线电容，不要在未核对时重复叠加上拉。当前接线应让上拉终点保持 3.3 V。

底层复用 [LibDriver AT24CXX](https://github.com/libdriver/at24cxx) 的实际芯片驱动，没有另写页访问驱动。上游固定提交与 MIT 声明见 [第三方依赖记录](middleware/eeprom/THIRD_PARTY.md)。本次没有替用户选择整个项目的 copyleft 许可证；分发时应保留这些第三方声明。

## 固件与工具准备

普通 Debug/Release 中 `EEPROM_ENABLED` 默认关闭。未启用或没接 EEPROM 时，存储显示不可用，RAM 查询/调参仍可使用；不会因 EEPROM 缺失调用硬件致命错误处理。专用 EncoderTest 等诊断固件不用于参数调试，需烧录正常控制固件。

启用 EEPROM 后，上电会在保持停机的维护阶段自动读取并应用最新有效记录；空片或读失败时保留编译默认值并报告存储状态。此过程不写 EEPROM，也不恢复使能状态。SET/SAVE/LOAD/DEFAULTS 从读取当前配置到实际应用或存储完成都持有维护门，期间 ARM/PID 被拒绝，退出时清除延迟运动指令。若控制任务未完成配置请求，后台继续等待并保持停机；GET/STATUS 仍可查询 BUSY，PC 超时不会自动重试。

接线确认后可构建启用 EEPROM 的正常固件：

```powershell
cmake --preset EepromDebug
cmake --build --preset EepromDebug
```

烧录仍是独立步骤，运行上述命令只构建固件。要改变 A2 跳帽配置，保持代码中 `EEPROM_AT24C08_A2` 与接线一致。

PC 串口工具只依赖 pyserial，解码与离线测试不需要它：

```powershell
python -m pip install pyserial
python tools\tuner.py --help
python tools\test_tuner.py
```

默认 115200、8N1，等待回执 3 秒；`--baud` 和 `--timeout` 在子命令前设置。下列 COM6 仅是示例，按实际连接替换。工具不会主动设置 DTR/RTS，也不会发送 ARM、运动或系统复位命令。

## 读参数、改 RAM、显式保存

先查询状态与当前值：

```powershell
python tools\tuner.py --port COM6 status
python tools\tuner.py --port COM6 get balance.kp
python tools\tuner.py --port COM6 get mechanical_zero
python tools\tuner.py --port COM6 get max_pwm
```

在固件处于 `DISARMED` 时修改参数。以下示例将速度 Kp 保持为当前尚未整定时使用的 0，不代表完成了实车 PID 整定：

```powershell
python tools\tuner.py --port COM6 set velocity.kp 0
python tools\tuner.py --port COM6 get velocity.kp
```

回执 `result=OK` 表示已在控制任务安全点完成应用，尚未写入 EEPROM。需要保留的参数逐项读回确认后，再显式保存：

```powershell
python tools\tuner.py --port COM6 --timeout 3 save
python tools\tuner.py --port COM6 status
```

`load` 读取最新有效持久化记录并申请安全应用；`defaults` 恢复 RAM 中的编译默认参数，不擦除 EEPROM。两者都不会使能电机。

```powershell
python tools\tuner.py --port COM6 load
python tools\tuner.py --port COM6 defaults
```

参数变更操作需要停机；忙、校准中、已使能或其他不满足条件的状态会被拒绝。不要把收到任意遥测帧当作调参成功；工具只接受本次请求匹配的完成回执。上电和载入后仍需要原有有效校准、稳定姿态与明确使能流程。

| Key | 工具参数名 | 含义与当前校验 |
|---:|---|---|
| 1 | `balance.kp` | 直立比例；有限值，绝对值不大于 10000 |
| 2 | `balance.kd` | 直立角速度反馈；同上 |
| 3 | `velocity.kp` | 速度比例；同上，当前默认 0 |
| 4 | `velocity.ki` | 速度积分；同上，当前默认 0 |
| 5 | `velocity.integral_limit` | 积分累积量限幅，0～30000 |
| 6 | `velocity.lpf_alpha` | 速度低通系数，0～1 |
| 7 | `velocity.max_output` | 速度环输出限幅；不超过板级 PWM 上限，级联模式还受倾角包络约束 |
| 8 | `turn.kp` | 转向比例；有限值，绝对值不大于 10000 |
| 9 | `turn.kd` | 转向微分；同上 |
| 10 | `turn.max_output` | 差分输出限幅，0～板级 PWM 上限 |
| 11 | `mechanical_zero` | 机械零点角，绝对值不超过 15° 且小于跌倒阈值 |
| 12 | `deadband_left` | 左轮死区补偿，0～当前 `max_pwm` |
| 13 | `deadband_right` | 右轮死区补偿，0～当前 `max_pwm` |
| 14 | `max_pwm` | 1～板级上限；当前 F411 板级默认上限为 4500 |

数字键 1～14 也可使用。`mechanical_zero_pitch`/`zero` 是机械零点别名；`left.deadband`、`right.deadband` 分别是左右死区别名。参数以 signed Q16.16 传输，分辨率 1/65536；NaN、Inf 和超出表示范围的输入会在 PC 侧拒绝，其余完整配置关系由固件检查。上述上限是输入保护，不是可直接使用的实车 PID 范围。

## 回执、故障与存储状态

工具输出完成结果、控制状态、故障位、存储标志、返回值和 RAM 配置版本。`revision` 是 RAM 参数变更版本，不是 EEPROM 写入次数或持久化序号。GET/SET 的 `value` 是参数值；STATUS 的 `value` 是整数存储错误码，其他操作不依赖此值。

| `result` | 含义 |
|---|---|
| `OK` | 请求已完成；保存内容未变化时也算成功 |
| `BUSY` | worker/队列忙，未获得完成确认 |
| `INVALID` | 字段、范围或整组配置不合法 |
| `UNSUPPORTED` | 此操作/参数不支持，或当前诊断固件不支持调参 |
| `UNSAFE` | 当前控制状态不允许该变更 |
| `STORAGE_ERROR` | 存储访问、完整性或读回验证失败 |
| `NO_SAVED` | 没有可载入的有效记录 |
| `NOT_READY` | 对应服务尚未就绪 |

控制状态 0/1/2/3 分别为 `DISARMED`、`CALIBRATING`、`ARMED`、`FALLEN`。`faults` 是独立的运动保护位，可以组合：

| Bit | 名称 | 含义 |
|---:|---|---|
| 0 | FALL_PITCH | 俯仰超限 |
| 1 | FALL_ROLL | 横滚超限 |
| 2 | CMD_TIMEOUT | 运动指令看门狗超时 |
| 3 | SENSOR_INVALID | 传感器异常、数据无效或过期等 |
| 4 | CALIBRATION_FAIL | 校准失败 |
| 5 | EMERGENCY_STOP | 急停保护 |

`storage_flags` 的 bit0～5 分别是 `CONFIGURED`（构建已启用）、`READY`（器件访问就绪）、`VALID`（存在已知有效记录）、`DIRTY`（当前 RAM 与持久状态不一致）、`BUSY`、`ERROR`。EEPROM 错误通过这些状态与回执报告，不伪装成 MPU6050 故障或运动致命故障。记录不可用时继续使用经过校验的 RAM 配置。

STATUS 存储错误码：0 表示没有存储错误；1～7 分别为未启用、未就绪、参数错误、初始化失败、读失败、写失败、错误调用上下文。记录层 0x103/0x104/0x105/0x106/0x107 分别为记录无效、I/O 失败、读回校验失败、参数配置无效、调用参数错误。空片本身不是硬件故障；接了 WP 写保护却请求 SAVE，可能返回读回验证失败。

上电读取了有效记录但控制任务未能应用时，错误码为 `0x200 + SERIAL_RESPONSE_*`。READY 表示最近一次初始化成功；后续 I/O 错误会清除 READY，下次显式保存/载入重新探测。VALID 表示最近确认有有效记录，不能保证拔掉芯片后仍然可访问。

回执超时不等于操作一定失败：可能已经完成，但蓝牙丢了回执；也可能保存已提交而最后一次读回失败。工具不自动重发，先用 STATUS/GET 确认。协议 sequence 只有 8 位，随机值用于降低旧回执撞号概率，不提供安全认证；不要同时运行多个写参数客户端。

## 持久化与协议实现

固件发送链路仍为 `serial_protocol_try_send → codec → lwpkt_write → LwRB → UART DMA`。PC 端严格按相同二进制 LwPKT 配置生成命令，复用 `encoder_monitor.py` 的 CRC8 与增量解析器；串口不发送 UTF-8 文本。

```text
请求：AA 04 08 [version, sequence, operation, key, value(Q16.16 LE)] CRC8 55
回执：AA 83 18 [24 字节回执载荷] CRC8 55
```

CRC8 覆盖 CMD、长度和 payload，采用项目实际 LwPKT 的反射多项式 0x8C、初值 0。请求载荷 Python 格式 `<BBBBi`；回执为 `<BBBBBBHiIII`，依次为版本、序列、原请求命令、结果、key、控制状态、存储标志、Q16 返回值、故障位、RAM revision、毫秒时间戳。线格式由显式编解码决定，不依赖 C 结构体对齐或主机大小端。

存储使用两个 256 字节槽，占 EEPROM 地址 0～511，512～1023 保留。每个槽保存 magic、schema、32 位序号、长度、CRC32、14 个 Q16.16 参数和最终提交标志。写另一个槽前先使其提交标志无效，再写内容并读回比较，最后写提交标志。旧有效槽始终保留；重启只选择 schema、CRC、范围全部通过的最新记录。相同编码内容不会重复写，修改 RAM 不消耗 EEPROM 写寿命。不同槽在记录提交时的字节断电情况由主机测试覆盖。

## 后续实车验收

本轮离线验证：**59/59 CTest 项通过，8/8 固件构建配置通过**，包括 EepromDebug。测试覆盖真实 LibDriver 的跨页/bank访问，参数存储的 149 个模拟断电字节边界，控制任务的实际应用回执与维护门，以及 PC tuner 的 16 项模拟串口用例。开源文件 SHA-256 与固定上游提交记录一致。没有打开实际串口、烧录或向实物 EEPROM 写入。

复验：`pwsh -NoProfile -File tools\run_static_checks.ps1 -IncludeDiagnostics`。启用 EEPROM 的独立产物位于 `build\EepromDebug\a26092701_f411.hex`，日常不开 EEPROM 的 Debug/Release 仍可 RAM 调参。

1. 先确认正常控制固件通过现有串口连接能收到匹配 STATUS 回执。若仍不能识别 LwPKT，继续原来的 USB 转串口对照检查，暂不判定 EEPROM 或编码器坏。
2. EEPROM 未接入时确认 RAM GET/SET 可用，SAVE/LOAD 有明确不可用结果，停机保护照常工作。
3. 停机后接好模块，启用 EepromDebug；STATUS 检查 READY。空片首次 LOAD 应明确没有有效记录。
4. 对一项参数 SET、GET、SAVE，再改 RAM、LOAD、GET，核对保存值恢复，过程始终不自动使能。
5. 完成软件保存测试后，再安排独立的断电恢复、WP 写保护、拔除 EEPROM 与总线异常验收；这些需要现场条件，主机模拟测试不代替实物结果。
