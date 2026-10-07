# 本次上电 HardFault 记录与复查

已收到的旧现场（后因用户复位清除）：

| 寄存器 | 值 | 可以得出的结论 |
| --- | --- | --- |
| CFSR | 0x00020000 | UsageFault INVSTATE，执行状态无效 |
| HFSR | 0x40000000 | FORCED，其他异常升级为 HardFault |
| CPACR | 0x00F00000 | CP10/CP11 已开启，不能归因于没有启用 FPU |
| VTOR | 0 | 单凭 0 不能认定错误；地址 0 与 Flash 的向量内容已实测一致 |
| MMFAR/BFAR | 0xE000EDF8 | CFSR 的 MMARVALID/BFARVALID 均未置位，这两个值不能当作故障访问地址 |

用户读取 0x00000000 和 0x08000000 都得到 0x20020000；读取两处 SVC 向量（+0x2C）都得到 0x0800F811。这个入口与当时 `build/Debug` ELF 的 SVC_Handler=0x0800F810 匹配，而当时 WheelTest 的入口为 0x0800A521。

CLion `.idea/workspace.xml` 当时也显示普通 Debug 配置启用、WheelTest 配置禁用。应先明确正在运行哪份固件；运行普通 Debug 不会执行悬空轮测试。用户随后发现电机驱动板电源未开启，但这个事实**不能单独证明 INVSTATE 根因**。

## 新现场：PendSV 恢复 IMU 上下文时跳到 0

用户从暂停的普通 Debug 固件读取到：

```text
g_fault_snapshot.magic           = 0x4641554c
exception_number                 = 3
exc_return                       = 0xffffffe1
original_msp / frame_address     = 0x2001ff78
original_psp                     = 0x200032f0
frame_valid / frame_extended     = 1 / 1
cfsr / hfsr                      = 0x00020000 / 0x40000000
stacked_lr / stacked_pc          = 0 / 0
stacked_xpsr                     = 0x4000000e
pxCurrentTCB                     = 0x20003410 (IMU)
pxCurrentTCB->pxStack             = 0x20002e08
pxCurrentTCB->pxTopOfStack        = 0x2000328c
*(uint32_t*)0x200032ac            = 0
```

对应旧 `build/Debug` ELF，PendSV 在 `0x0800fe20` 从保存栈恢复 r4-r11 和 LR，第 9 个字（`0x200032ac`）应为 EXC_RETURN，实际为 0。随后错误地走浮点恢复分支，PSP 从 `0x200032b0` 增加 64 字节到 `0x200032f0`，与快照吻合。最后 `0x0800fe36` 的 `bx lr` 跳到 0，触发 INVSTATE。xPSR 的异常号 14 表示 PendSV，T 位为 0；这是故障现场，不是把停机循环本身当成根因。

## 已确认的缺陷与修复

当前故障 ELF 使用 GCC 14.3.1、`-O0`。反汇编和同版本编译器的 `.su` 报告分别确认：

| 同时在栈上的调用层 | 栈占用（字节） |
| --- | ---: |
| imu_fusion_task_entry | 152 |
| imu_fusion_update | 232 |
| FusionAhrsUpdateNoMagnetometer | 56 |
| FusionAhrsUpdate | 2552 |
| 四层合计 | **2992** |

其中 `FusionAhrsUpdate` 的函数入口为 push 12 字节，再 `subw sp, sp, #2540`。旧任务只分配 **512 words = 2048 字节**，单个函数就已超预算。其内部调用 `FusionAhrsRestart` 时另占 72 字节，浮点上下文保存另需 204 字节（再留对齐空间）。Fusion 的强制内联在 `-O0` 下生成大量临时变量，不能用 Release 的栈占用估计 Debug。

用户随后实测确认 IMUFusion TCB 为 `0x20003d60`，栈底为 `0x20003558`，保存栈顶为 `0x20003bbc`，融合上下文为 `0x20003478`。旧 2048 字节栈的线程入口 SP 为 `0x20003d50`，减去上述四层 2992 字节后，AHRS 帧基址为 `0x200031a0`。`0x08013d56` 的四元数临时结果写入地址为 `0x200031a0 + 2536 - 2268 = 0x200032ac`，恰好覆盖 IMU 保存的 EXC_RETURN。初始四元数 `(1,0,0,0)` 使该结果为零；其后三个浮点结果以及另一份四元数副本也与内存转储吻合。这将 AHRS 栈越界与本次 PendSV 恢复零返回值直接对应起来。

本次读取还确认两个任务栈底的前 64 字节均保持 `0xa5a5a5a5`。AHRS 大帧跨过底部标记，标记所在区域对应未执行的磁力计分支临时变量，故未被写入；不能据此否定越界。

修复将 IMUFusion 栈单独增到 **1024 words = 4096 字节**。保留调试优化等级、第三方算法和异常停机机制。每次固件构建自动生成 `.su`，并检查上述四层调用链，加 208 字节上下文、512 字节子调用/余量和 8 字节任务起始余量：本次为 **3720 <= 4096**。该检查会拒绝旧 2048 字节配置、缺失报告和动态栈记录；它覆盖已知路径预算，不是所有路径的栈安全证明。额外 2048 字节由现有 FreeRTOS 堆分配，因此链接器静态 RAM 总量不增加，运行时剩余堆会减少。

当前 FreeRTOS 10.3.1 的模式 2 仅检查栈底四个填充值。大帧可跨过检查位置，在其他地址写临时变量，因此没有进入溢出钩子不代表没有越界；[FreeRTOS 官方说明](https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/09-Memory-management/02-Stack-usage-and-stack-overflow-checking)也未保证检测所有溢出。

保持当前 CLion 暂停时，可在 **GDB 控制台**执行下面一行读取剩余现场。使用代码块复制，避免 Markdown 将星号、下划线、箭头转义：

```gdb
source C:/Users/30496/CLionProjects/a26092701_f411/tools/fault-context.gdb
```

脚本只读取现有变量和内存，不运行目标函数、不恢复执行、不复位。根因分析使用原 `build/Debug` ELF；修复验证使用独立目录 `build/static-fix/fault-stack-after`，当前调试 ELF 保留原样。故障对应的 ELF、map 和编译命令另备份在 `build/static-fix/hardfault-origin`，后续重建 Debug 不会覆盖这份诊断依据。

验证结果：同 CLion 的 GCC 14.3.1 Debug 构建和栈预算检查通过；`tools/run_static_checks.ps1 -IncludeDiagnostics` 的 **82 项 CTest 全部通过**（新增门禁测试内部含 12 个案例），9 个固件配置全部构建通过。日志在 `build/static-fix/hardfault-regression.log`。故障对应旧 Debug ELF 的 SHA256 为 `700CB13D39893208C8125B9A463FAA82CD67F6F173C5ED54C85FC9D578347414`。

## 保存现场后的烧录与复查

1. 正常连接驱动板/逻辑电源，保持两轮悬空。
2. 普通 Debug 修复验收应先重新编译对应 Debug 配置，再启动新的调试会话，观察 IMU 校准完成后的持续运行。悬空转轮测试则在 CLion 启用生成目录为 `build/WheelTest` 的 CMake 配置，并选择其 `a26092701_f411.elf`。不要仅修改源码后仍调试另一目录的 ELF。
3. 当前专用构建命令仍是 `cmake --preset WheelTest` 和 `cmake --build --preset WheelTest`；HEX 位于 `build/WheelTest/a26092701_f411.hex`。
4. 新固件上电保持 WAIT，不会自行转动。正常时按 WHEEL_TEST_GUIDE.md 使用电脑端脚本开始测试。
5. 若再进入异常，在监视或 Alt+F8 展开 `g_fault_snapshot`，先不要复位；该变量存在 SRAM，复位会清除。

优先提供这些字段：

```text
g_fault_snapshot.magic
g_fault_snapshot.exception_number
g_fault_snapshot.cfsr
g_fault_snapshot.hfsr
g_fault_snapshot.exc_return
g_fault_snapshot.frame_valid
g_fault_snapshot.stacked_pc
g_fault_snapshot.stacked_lr
g_fault_snapshot.stacked_xpsr
g_fault_snapshot.original_msp
g_fault_snapshot.original_psp
```

`magic=0x4641554C` 表示采集完成；`frame_valid=1` 表示栈范围/布局可读取，不保证内容没有损坏。若 frame_valid=0，不使用清零的 PC/LR 推断根因。magic=0 且 `g_fault_capture_guard` 非零表示采集未完成或被更高优先级故障打断。

## 诊断改动与验证边界

五个 Cortex fault 入口改为 naked 汇编，保存原始 EXC_RETURN/MSP/PSP，再切换独立故障栈。C helper 首先锁存电机停止，之后保存 SCB 和堆栈记录，不使用串口或 RTOS。无效地址、堆栈错误位和异常返回值均先检查；嵌套故障不覆盖已完成的首份记录。

根据 [ARM Cortex-M4 官方手册 Figure 2-3](https://documentation-service.arm.com/static/5f2ac4ab60a93e65927bbdbf)，基本帧和浮点扩展帧都以 R0 开始；浮点区域在 xPSR 后方，不能把 SP 加 18 个字再读取 PC。实现和测试已按此布局核对。

故障记录器本身是诊断增强。后续现场已据此确认 PendSV 恢复出的 EXC_RETURN 为零，并定位、修正了 IMUFusion 确定存在的栈容量不足。**修复后的实机运行尚未验收**；本次没有重刷或复位目标板，也没有连接第二个 GDB 客户端干扰 CLion 现场。
