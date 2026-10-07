# F411 手推编码器测试

> 当前已完成模块重构：测试任务在 `app/diagnostics`，硬件后端在 `platform/stm32f411`，协议使用显式小端 codec。下面的 EncoderTest 预设、串口参数和 0x87 字节格式保持不变。当前源码结构见 [README.md](README.md)。

这套测试用于观察左右编码器计数、方向和原始 A/B 电平。PC 工具只接收二进制帧，不发送使能、电机或调参命令。**目前只完成软件检查；没有连接实车、打开串口或烧录。**

## 测试前

- 用户已确认此前 LDO 故障及后续 3V3 恢复。实车检查仍应在正常供电下进行；此前约 2V 条件下的记录不能作为可靠硬件验收依据。
- 使用本次新增的专用编码器测试固件；该模式报告电机停机锁存状态。手推前切断电机动力电源，保留所需逻辑电源并共地，让轮子能够自由转动。
- 串口为项目的 USART2，115200、8N1。USB 转串口的 RX 接 PA2（板端 TX），GND 共地。本工具不发命令，适配器 TX 可以不接 PA3。Type-C 供电不等于已经连接 USART2 数据。
- 左编码器：TIM2，A=PA5、B=PB3；右编码器：TIM5，A=PA0、B=PA1。实际线序、电平及每圈计数需要实测。

专用固件使用已有的 ARM 工具链和 Ninja，在工程根目录构建：

```powershell
cmake --preset EncoderTest
cmake --build --preset EncoderTest
```

产物为 `build\EncoderTest\a26092701_f411.hex` 和 `.bin`。该模式只运行协议、EncoderPush 和状态灯任务，不启动 IMU/平衡闭环，忽略收到的运动命令，并锁存电机停机。**本次未烧录**；后续需明确选用这份专用产物，不要把原有 Debug/Release 闭环固件当作手推测试固件。

## Windows 采集

在工程根目录用 PowerShell 执行。只有实时串口采集需要安装 pyserial：

```powershell
python -m pip install pyserial
[System.IO.Ports.SerialPort]::GetPortNames()
python tools\encoder_monitor.py --port COM7 --baud 115200 --seconds 15 --csv build\encoder-push\01-still.csv --raw-output build\encoder-push\01-still.bin
```

把 `COM7` 换成实际串口；程序不会自动选串口。输出文件只新建，不覆盖已有结果；再次测试请换文件名。Ctrl+C 可提前结束，已收到的数据仍会保存并生成概要。程序不发送应用数据；部分 USB 转串口驱动在开口时仍可能改变 DTR/RTS 电平，不要把这些线接到复位或使能端。

每个工况**独立录制 15～30 秒**，把上方命令的时长和两个输出文件名改成对应行；不要把不同工况混成一段再猜方向。“向前”以车轮推动小车前进的方向为准：

| 文件名前缀 | 时长 | 操作与观察 |
| --- | --- | --- |
| `01-still` | 15 秒 | 两轮静止，delta 应以 0 为主；记录有无抖动 |
| `02-left-only` | 20 秒 | 仅左轮先向前、再反向慢转，记录换向时间；左 delta 符号应反转，右轮保持静止 |
| `03-right-only` | 20 秒 | 仅右轮先向前、再反向慢转，记录换向时间；右 delta 符号应反转，左轮保持静止 |
| `04-forward` | 20 秒 | 手推小车向前；经极性配置修正后，两轮 delta 应同为正 |
| `05-backward` | 20 秒 | 手推小车向后；两轮 delta 应同为负 |
| `06-one-turn-left` / `06-one-turn-right` | 各 30 秒 | 分别做单轮完整一圈，比较首尾 total 差；正反向、重复测量另存文件 |
| `07-still-after` | 15 秒 | 两轮再次静止，观察 delta 是否重新归零 |

若要测每圈计数，给轮子做标记，另外录制“静止 → 完整转 N 圈 → 静止”。用首尾稳定时的 **total 差 / N** 计算每轮输出轴一圈的 X4 计数，并重复正反向测量。不要把当前代码里的默认反相值、轮径或减速比当作实测结果。

## 如何读数

工具每秒显示最近一帧，CSV 保存每个通过校验的 0x87 载荷。固件目标采样周期为 50ms（20Hz），以实际 `sample_period_ms` 为准。

- `raw_left/raw_right`：32 位定时器原始计数，不做方向修正；越过 0 时回绕属于计数器行为。
- `delta_left/delta_right`：本采样间隔的 X4 增量，已经按固件极性配置修正。这里是 **counts/sample**，不是闭环任务的 counts/nominal-5ms，也不是转速或米每秒。
- `total_left/total_right`：本次固件运行中的有符号累计计数。原始计数与修正后计数的符号可能不同。
- `AB L=.. R=..`：当前 A/B 电平快照。50ms 快照看不到所有脉冲，不能代替示波器或逻辑分析仪判断相位、毛刺和边沿质量。
- `flags`：bit0=本次增量有效，bit1=编码器初始化成功，bit2=固件报告电机停机锁存，bit3=左反相，bit4=右反相。需要核对 bit0～2；初始化或有效标志缺失时不要用该帧判定方向。软件锁存标志不是电机驱动板实际断电的证明。
- `tx_dropped/sample_errors`：设备自启动以来的累计发送丢弃与采样错误计数。概要里的 CRC/帧尾错误是解析器发现的坏候选帧次数，不等于精确的物理丢包数。
- 序列缺口只给出模 256 的缺口下界；重启、乱序和整圈序号回绕另行记录。不能据此证明串口完全无丢包。

没有计数变化时，先明确自己是否转动了对应轮子，再核对供电、接线和 A/B 信号。**软件不会因“没有脉冲”就判定编码器坏了，也不会自动给出“方向、精度全部通过”。**

## 离线回放与回归

本轮已通过 28/28 项 CTest（含 Python 18 项用例）、真实 LwPKT/LwRB 组帧互通、手推任务 4 种场景；EncoderTest 和正常 Debug/Release 固件均构建通过。EncoderTest ELF 中没有平衡任务、IMU 采样任务或正常电机输出函数。上述均为软件验证，未打开实车串口、未烧录。

完整回归可在工程根目录运行 `pwsh -NoProfile -File tools\run_static_checks.ps1`，然后运行前述 EncoderTest 预设构建。CTest 自动先生成真实 C 帧样本，再交给 Python 做互通校验。

离线回放只用 Python 标准库，可以重新生成 CSV，同时保留原始文件：

```powershell
python tools\encoder_monitor.py --input build\encoder-push\01-still.bin --csv build\encoder-push\01-still-replay.csv
python -m unittest discover -s tools -p test_encoder_monitor.py -v
```

退出码：0=读到了有效编码器载荷；1=文件/串口等操作失败；2=没有有效编码器载荷。退出码 0 只代表解析到了数据，不代表实车验收通过。

真实 LwPKT C 写入器互通测试使用本次构建生成的 fixture：

```powershell
$env:ENCODER_LWPKT_FIXTURE = (Resolve-Path -LiteralPath 'build\static-fix\encoder_lwpkt_fixture.bin').Path
python -m unittest discover -s tools -p test_encoder_monitor.py -v
Remove-Item Env:\ENCODER_LWPKT_FIXTURE
```

协议依据是仓库的 `lwpkt_opts.h`、`lwpkt_opt.h` 和 `lwpkt.c`，不是旧文档：`AA 87 38 [56 字节载荷] CRC8 55`，共 61 字节；CMD 为单字节，长度字段是受最大载荷 64 限制的单字节 varint。CRC8 初值 0、反射多项式 0x8C、无最终异或，覆盖 CMD、长度和载荷，不含首尾字节。载荷按 little-endian `'<BBHIIIIiiIqqII'` 解包。AA/55 出现在载荷中是正常数据，工具按长度和 CRC 处理。
