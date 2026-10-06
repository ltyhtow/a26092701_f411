# STM32F411CEU6 平衡车引脚与接线表 (方案 A)

本工程针对 **WeAct STM32F411CEU6 (BlackPill)** 核心板设计，由原 STM32C562 架构无缝迁移而来。

---

## 1. 核心引脚全景映射表

| 序号 | 功能模块 | 引脚 | 复用功能 | 外部连接 / 对端器件 | 备注 |
| :---: | :--- | :---: | :--- | :--- | :--- |
| 1 | **板载指示灯** | `PC13` | GPIO_Output (推挽) | 板载 LED (SYS_LED) | 低电平亮，初始 500ms 翻转心跳 |
| 2 | **电机1 AIN1** | `PA8` | TIM1_CH1 (PWM) | AT8236 `AIN1` | 20 kHz PWM (0~4999 对应 0~100%) |
| 3 | **电机1 AIN2** | `PA9` | TIM1_CH2 (PWM) | AT8236 `AIN2` | 20 kHz PWM |
| 4 | **电机2 BIN1** | `PA10` | TIM1_CH3 (PWM) | AT8236 `BIN1` | 20 kHz PWM |
| 5 | **电机2 BIN2** | `PA11` | TIM1_CH4 (PWM) | AT8236 `BIN2` | 20 kHz PWM |
| 6 | **编码器1 A相** | `PA5` | TIM2_CH1 (AF1) | 编码器 1 A 相 | X4 编码器模式，N8 滤波，原线不用动 |
| 7 | **编码器1 B相** | `PB3` | TIM2_CH2 (AF1) | 编码器 1 B 相 | X4 编码器模式，N8 滤波，原线不用动 |
| 8 | **编码器2 A相** | `PA0` | TIM5_CH1 (AF2) | 编码器 2 A 相 | X4 编码器模式，N8 滤波，原线不用动 |
| 9 | **编码器2 B相** | `PA1` | TIM5_CH2 (AF2) | 编码器 2 B 相 | X4 编码器模式，N8 滤波，原线不用动 |
| 10 | **电池分压采样** | `PB0` | ADC1_IN8 (模拟) | 分压电阻网络 `VIN/11` | 12-bit ADC，原线不用动 |
| 11 | **MPU6050 中断** | `PB1` | EXTI1 (输入上拉) | MPU6050 `INT` | 上升沿中断，原线不用动 |
| 12 | **MPU6050 SCL** | `PB6` | I2C1_SCL (开漏上拉) | MPU6050 `SCL` | 400 kHz 快速模式 |
| 13 | **MPU6050 SDA** | `PB7` | I2C1_SDA (开漏上拉) | MPU6050 `SDA` | 400 kHz 快速模式，原线不用动 |
| 14 | **遥测串口 TX** | `PA2` | USART2_TX (AF7) | 蓝牙/串口模块 `RX` | 115200 8N1，DMA 发送 |
| 15 | **遥测串口 RX** | `PA3` | USART2_RX (AF7) | 蓝牙/串口模块 `TX` | 115200 8N1，DMA 接收 |
| 16 | **SWD 调试** | `PA13` | SWDIO | ST-Link / DAPLink | SWD 调试 |
| 17 | **SWD 时钟** | `PA14` | SWCLK | ST-Link / DAPLink | SWD 调试 |
| 18 | **公共地** | `GND` | 电源地 | 驱动板 / 传感器 / 串口共地 | **务必共地** |
| 19 | **逻辑供电** | `3.3V` | 供电输出 | 传感器供电 | **【硬件异常警示】当前板载 3.3V 损坏，实测仅输出 ~2.0V**。MPU6050 仍可勉强工作，但电机霍尔/光电编码器无法翻转，实测脉冲恒定为 0。系统当前工作在纯直立 PD 模式，速度环已禁用。 |

---

## 2. 电机旋转方向与极性约定（底盘更正后的物理参考系）

经过实际直流驱动测试与底盘重新组装后的物理视角标定：

| 运动目标 | 左轮 A路 (PA8 / PA9) | 右轮 B路 (PA10 / PA11) | 软件 `motor_driver_set_output` |
| :--- | :--- | :--- | :--- |
| **真正前进 (Forward)** | `PA8 = PWM`, `PA9 = 0` | `PA10 = 0`, `PA11 = PWM` | `left_pwm > 0, right_pwm > 0` |
| **真正后退 (Backward)** | `PA8 = 0`, `PA9 = PWM` | `PA10 = PWM`, `PA11 = 0` | `left_pwm < 0, right_pwm < 0` |
| **就地左转 (Turn Left)** | `PA8 = 0`, `PA9 = PWM` (后退) | `PA10 = 0`, `PA11 = PWM` (前进) | `left_pwm < 0, right_pwm > 0` |
| **就地右转 (Turn Right)**| `PA8 = PWM`, `PA9 = 0` (前进) | `PA10 = PWM`, `PA11 = 0` (后退) | `left_pwm > 0, right_pwm < 0` |
| **急停 / 刹车** | `PA8 = 0`, `PA9 = 0` | `PA10 = 0`, `PA11 = 0` | `left_pwm = 0, right_pwm = 0` |

---

## 3. 定时器参数与频率说明

### 电机驱动 (TIM1)
- 挂载总线：APB2（Timer 时钟频率：100 MHz）
- 预分频器（Prescaler）：`0`
- 自动重装载值（Period/ARR）：`4999`
- PWM 输出频率：`100 MHz / (1 * 5000) = 20.000 kHz`
- 占空比计算：`0` 对应 0%，`2500` 对应 50%，`5000` 对应 100%。

### 编码器接口 (TIM2 & TIM5)
- 挂载总线：APB1（Timer 频率 100 MHz）
- 模式：`TIM_ENCODERMODE_TI12`（4倍频计数模式，在双通道的上升沿和下降沿均计数）
- 计数器位宽：32 位（ARR = `0xFFFFFFFF`），不会轻易溢出
- 输入滤波：8 级数字滤波（有效抑制电机火花和引线干扰）

---

## 4. 本地构建与烧录命令

```powershell
# 1. 编译工程
& "C:\ST\STM32CubeCLT_1.20.0\CMake\bin\cmake.exe" --build "C:\Users\30496\CLionProjects\a26092701_f411\build\Debug"

# 2. 烧录固件 (ST-Link SWD 模式)
& "C:\ST\STM32CubeCLT_1.20.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -w "C:\Users\30496\CLionProjects\a26092701_f411\build\Debug\a26092701_f411.hex" -v -rst

# 3. 烧录固件 (USB DFU 模式：按住 BOOT0 插入 USB 后松开)
& "C:\ST\STM32CubeCLT_1.20.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=USB1 -w "C:\Users\30496\CLionProjects\a26092701_f411\build\Debug\a26092701_f411.hex" -v -rst
```
