# STM32F411CEU6 两轮自平衡小车项目交接文档 (Agent Handoff)

**更新时间**：2026-10-07  
**固件工程根目录**：`C:\Users\30496\CLionProjects\a26092701_f411`  
**核心芯片**：STM32F411CEU6 (WeAct BlackPill, ARM Cortex-M4F @ 100MHz, 128KB RAM, 512KB Flash)  
**开发与构建环境**：Windows 11 + CMake + Ninja + arm-none-eabi-gcc 14.3.1 (CubeCLT 1.20.0) + FreeRTOS CMSIS-V2  

---

## 1. 硬件平台与关键工程背景

1. **芯片迁移背景**：
   - 原项目基于 STM32C562CET6 (Cortex-M33)，因芯片物理损坏烧毁，已**全量迁移至 STM32F411CEU6** 核心板。
   - FreeRTOS 堆大小已扩展为 48 KB（`configTOTAL_HEAP_SIZE = 48 * 1024`），硬件 FPU 已完全开启并使能浮点寄存器上下文保护（`configENABLE_FPU 1`）。
2. **远程调试方案彻底废弃**：
   - 早期文档中提及的安徽端/河南端 GDB 端口 3333 与网络串口桥 8080 **已彻底废弃**。
   - 现阶段所有构建、烧录（ST-Link SWD / USB DFU）以及串口遥测采集均为**本地主机环境直接操作**。
3. **硬件供电损坏与降级运行模式**：
   - **严重警示**：单片机核心板板载 3.3V LDO 供电损坏，实际输出电压被拉低至 **~2.0V**。
   - MPU6050 目前仍能在 2.0V 下进行 I2C 通信与姿态解算；
   - **电机编码器完全失效**：实测推车时霍尔/光电比较器在 2V 供电下无法产生电平翻转，脉冲计数恒定为 0；
   - **算法降级对策**：为防止缺少速度反馈导致速度环积分饱和飞车，当前固件已将速度环增益归零（`velocity_loop.kp = 0.0f; velocity_loop.ki = 0.0f;`），小车运行在**纯直立 PD 平衡模式**。

---

## 2. 硬件引脚与外设映射全景 (Pinout Mapping)

| 功能模块 | 引脚 | 外设复用 | 对端设备 / 功能说明 | 硬件与电平特性 |
| :--- | :---: | :---: | :--- | :--- |
| **左轮电机 (A路) AIN1** | `PA8` | TIM1_CH1 (PWM) | AT8236 驱动 `AIN1` | 20 kHz PWM (ARR=4999), 占空比 0~100% |
| **左轮电机 (A路) AIN2** | `PA9` | TIM1_CH2 (PWM) | AT8236 驱动 `AIN2` | 20 kHz PWM |
| **右轮电机 (B路) BIN1** | `PA10` | TIM1_CH3 (PWM) | AT8236 驱动 `BIN1` | 20 kHz PWM |
| **右轮电机 (B路) BIN2** | `PA11` | TIM1_CH4 (PWM) | AT8236 驱动 `BIN2` | 20 kHz PWM |
| **左编码器 A相** | `PA5` | TIM2_CH1 (AF1) | 左轮测速编码器 A 相 | 32-bit 定时器, X4 模式, N8 数字滤波 |
| **左编码器 B相** | `PB3` | TIM2_CH2 (AF1) | 左轮测速编码器 B 相 | 32-bit 定时器, X4 模式, N8 数字滤波 |
| **右编码器 A相** | `PA0` | TIM5_CH1 (AF2) | 右轮测速编码器 A 相 | 32-bit 定时器, X4 模式, N8 数字滤波 |
| **右编码器 B相** | `PA1` | TIM5_CH2 (AF2) | 右轮测速编码器 B 相 | 32-bit 定时器, X4 模式, N8 数字滤波 |
| **MPU6050 SCL** | `PB6` | I2C1_SCL (开漏) | MPU6050 SCL | 400 kHz Fast Mode, 初始化带 9 脉冲防锁死清总线 |
| **MPU6050 SDA** | `PB7` | I2C1_SDA (开漏) | MPU6050 SDA | 400 kHz Fast Mode |
| **MPU6050 INT** | `PB1` | EXTI1 (输入上拉) | MPU6050 数据就绪中断 | 上升沿触发检测 |
| **遥测串口 TX** | `PA2` | USART2_TX (AF7) | 蓝牙透传 / 串口模块 RX | 115200 8N1, DMA1 Stream 6 发送 |
| **遥测串口 RX** | `PA3` | USART2_RX (AF7) | 蓝牙透传 / 串口模块 TX | 115200 8N1, DMA1 Stream 5 接收 |
| **板载状态指示灯** | `PC13` | GPIO_Output (推挽) | 板载 LED (SYS_LED) | 低电平点亮; 慢闪: 解算正常; 快闪: 传感器异常 |
| **电池电压采样** | `PB0` | ADC1_IN8 (模拟) | 分压电阻网络 `VIN/11` | 12-bit ADC |
| **SWD 调试** | `PA13` / `PA14` | SWDIO / SWCLK | ST-Link / DAPLink | SWD 调试烧录接口 |

---

## 3. 电机物理旋转极性真值表（更正后的底盘参考系）

根据底盘重新装配后的实际物理朝向，经直流直驱测试与校正：

| 动作目标 | 左轮 A路驱动 (`PA8 / PA9`) | 右轮 B路驱动 (`PA10 / PA11`) | 驱动函数输入参数 |
| :--- | :--- | :--- | :--- |
| **真实前进 (Forward)** | `PA8 = PWM`, `PA9 = 0` | `PA10 = 0`, `PA11 = PWM` | `left_pwm > 0, right_pwm > 0` |
| **真实后退 (Backward)**| `PA8 = 0`, `PA9 = PWM` | `PA10 = PWM`, `PA11 = 0` | `left_pwm < 0, right_pwm < 0` |
| **就地左转 (Turn Left)** | `PA8 = 0`, `PA9 = PWM` (后退) | `PA10 = 0`, `PA11 = PWM` (前进) | `left_pwm < 0, right_pwm > 0` |
| **就地右转 (Turn Right)**| `PA8 = PWM`, `PA9 = 0` (前进) | `PA10 = PWM`, `PA11 = 0` (后退) | `left_pwm > 0, right_pwm < 0` |
| **电机停转 (Stop)** | `PA8 = 0`, `PA9 = 0` | `PA10 = 0`, `PA11 = 0` | `left_pwm = 0, right_pwm = 0` |

---

## 4. 串口通信协议与线格式 (LwPKT Binary Protocol)

本项目**严禁**向串口直接输出未经组帧的裸 ASCII 字符串。所有通信必须通过 `serial_protocol_send()` 由 LwPKT 封装发送：

### 4.1 数据帧封装格式
```text
[START: 0xAA] [CMD: 1B] [LEN: 1~2B varint] [DATA: Payload] [CRC8: 1B] [STOP: 0x55]
```

### 4.2 核心业务命令集 (CMD)
| 命令字 | 方向 | 负载结构体 | 大小 | 说明 |
| :---: | :---: | :--- | :---: | :--- |
| `0x01` | 上位机 -> MCU | `motion_command_t` | 16B | 速度与角速度控制指令 (含 500ms 看门狗超时) |
| `0x02` | 上位机 -> MCU | `pid_config_command_t` | 20B | 动态在线整定 PID 参数 |
| `0x03` | 上位机 -> MCU | `system_command_t` | 4B | 紧急停机、使能、清除故障、复位 |
| `0x81` | MCU -> 上位机 | `motion_telemetry_t` | 32B | 50Hz 平衡与运动遥测 (姿态、转速、PWM、状态) |
| `0x84` | MCU -> 上位机 | `imu_telemetry_t` | 24B | MPU6050 原始加速度与角速度遥测 |
| `0x85` | MCU -> 上位机 | `imu_diagnostic_t` | 24B | I2C 与 WHO_AM_I 硬件自检诊断信息 |
| `0x86` | MCU -> 上位机 | `imu_attitude_telemetry_t` | 52B | Fusion AHRS 姿态四元数、欧拉角与陀螺零偏 |

---

## 5. FreeRTOS 任务架构与调度拓扑

系统共运行 5 个协作式 FreeRTOS 任务：

1. **`ProtocolTask` (优先级 2, 512 words)**:
   - 维护 `lwpkt` 状态机；
   - 监听 USART2 DMA 接收中断，拆包后分发至控制队列；
   - 驱动 TX Ring Buffer 与 UART DMA 连续发送。
2. **`ImuTask` (优先级 3, 384 words)**:
   - 5 ms (200 Hz) 定时触发；
   - 读取 MPU6050 原始传感器数据并放入 `imu_fusion_input_queue`。
3. **`ImuFusionTask` (优先级 3, 512 words)**:
   - 200 Hz 运行 x-io Technologies Fusion AHRS 姿态解算法；
   - 完成静态陀螺零偏校准与加速度重力投影解算；
   - 将最新姿态与角速度输出写入 `imu_attitude_queue`。
4. **`BalanceTask` (优先级 3, 512 words)**:
   - 5 ms (200 Hz) 周期执行；
   - 运行 **4 状态安全有限状态机 (Safety FSM)**：`DISARMED` / `CALIBRATING` / `ARMED` / `FALLEN`；
   - 跌倒保护判定：$|\theta_{pitch}| > 35^\circ$ 或 $|\theta_{roll}| > 30^\circ$ 立即断开 PWM 输出切入 `FALLEN`；
   - 扶正自恢复：当车身倾角恢复至 $|\theta_{pitch}| \le 5^\circ$ 且角速度平稳时自动重新使能 `ARMED`；
   - 直立平衡闭环：计算 $u_{bal} = K_p \cdot (\theta - \theta_0) + K_d \cdot \omega_y$ 并驱动电机；
   - 以 50 Hz（4 分频）发布 `0x81` 运动遥测包。
5. **`Task1` (优先级 1, 256 words)**:
   - 心跳指示任务：根据系统解算状态动态控制 PC13 LED 闪烁周期（传感器故障快闪 100ms，静止校准中 250ms，正常运行慢闪 500ms）。

---

## 6. 本地常用构建与调试命令

### 编译工程
```powershell
& "C:\ST\STM32CubeCLT_1.20.0\CMake\bin\cmake.exe" --build "C:\Users\30496\CLionProjects\a26092701_f411\build\Debug"
```

### ST-Link SWD 烧录
```powershell
& "C:\ST\STM32CubeCLT_1.20.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -w "C:\Users\30496\CLionProjects\a26092701_f411\build\Debug\a26092701_f411.hex" -v -rst
```

### USB DFU 烧录 (按住 BOOT0 插入 USB)
```powershell
& "C:\ST\STM32CubeCLT_1.20.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=USB1 -w "C:\Users\30496\CLionProjects\a26092701_f411\build\Debug\a26092701_f411.hex" -v -rst
```
