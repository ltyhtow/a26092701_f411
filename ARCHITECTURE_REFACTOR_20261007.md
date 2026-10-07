# 全模块解耦实施与验证记录

2026-10-07；基于本会话已有静态修复与 EncoderTest 工作区。用户授权自主完成，总并发始终不超过 4（主代理 + 控制、协议、硬件三个工作分支）。代码已实际修改与构建；没有烧录、操作串口或修改旧 C562 工程，没有提交 Git。

## 完成范围

此前审查的八组边界均已落实，不再只是待办建议：

| 原问题 | 实际处理 | 验证 |
| --- | --- | --- |
| 全局头目录、没有模块构建边界 | 共用 `cmake/modules.cmake` 定义独立核心库，精确 PUBLIC/PRIVATE include | 同一目标用于 ARM 与 host；源码和编译参数边界门禁 |
| app_freertos 混合协议/业务/板级操作 | 拆为 App 装配、App 协议路由、纯协议 engine、FreeRTOS adapter、板状态/故障入口 | 真实路由、adapter 生命周期和满队列停机回归 |
| 控制服务依赖 RTOS/wire/Fusion | 新增标准类型 `robot_types.h` 和显式 context 的 `balance_runtime`；task 仅作适配 | 纯 runtime 无 stub 编译；RTOS adapter 单独测试 |
| Fusion 类型污染公共姿态头 | 输出改为中立 `attitude_sample_t`；Fusion context 不透明，私有类型仅在实现内 | public API host 测试，不向消费者公开 Fusion include |
| weak/strong 隐式电机绑定 | 删除两端符号，组合根明确注入 `motor_driver_set_output` | 缺必要 hook 启动失败；并发停止测试 |
| 设备核心和引脚/诊断混合 | 后端移到 platform；编码器纯计数算法抽成独立 context；三诊断任务移 App，通过板接口操作 | 计数器无 stub 测试，原硬件/诊断行为回归 |
| payload 依赖 native struct ABI | 八种消息按固定偏移显式 LE encode/decode | golden bytes、真实 LwPKT、Python 18 项互通；packed ABI 变体 |
| tick 隐含等于毫秒 | 统一 `app_clock` + 可移植单调时间累计器，保留分数余量与回绕 | 100/1000/1024/2000Hz、tick 和毫秒回绕、周期取整测试 |

硬件驱动仍需要 HAL、寄存器和 RTOS 同步机制；这些依赖已经成为明确适配层职责。这里没有声称整份固件可以不带 MCU/RTOS 运行，也没有为了统计数字把所有接口包装成一个庞大 OSAL。

## 当前核心与依赖

实际自有核心源为 **8 个 C 文件、2084 物理行**：

- balance_controller、safety_fsm、balance_runtime
- encoder_counter
- imu_fusion
- serial_codec、serial_protocol_engine
- monotonic_clock

它们组织为 6 个实体核心库：`balance_core`、`encoder_core`、`imu_fusion_core`、`serial_codec`、`protocol_engine`、`monotonic_clock`，另有 `robot_contracts` 接口目标。可移植第三方库也各自建目标，Fusion 依赖为实现私有。

重构前 middleware 有 13 个 C 文件，其中 10 个依赖 MCU/RTOS；重构后 middleware 的 8 个自有 C 实现均不再包含或依赖 MCU/RTOS 头。当前检查覆盖其 24 个 C/H 文件及 11 个 App C/H 文件；真实 ARM 与 host 编译数据库中，核心编译命令没有 Core、Drivers、FreeRTOS 或测试 stub 头路径。

这是抽出了核心并移动了适配器，**不是消灭了硬件代码**。当前自有总量为 39 个 C、42 个头文件，5590 物理行；其余 11 个 C 在 App/FreeRTOS 适配层，17 个 C 为 STM32/板级实现，3 个是启动/运行库基础设施。新旧模块划分不同，不能把总 include 边数直接当作解耦得分。

业务核心之间现在只通过领域契约发生跨目录引用：control→contracts、mpu6050→contracts；旧的 control→serial/Fusion 实现、encoder→motor/serial/mpu 配置依赖已经移到正确的应用/适配边界。

## 关键 API 与兼容性

- `balance_runtime_t` 为显式 context，可同时创建独立实例。正常调用顺序是 begin→输入事件→finish→处理校准 effect→prepare_output→写 PWM。最终时效检查必须在与停止请求相同的排他区域内执行。
- `balance_task` 位于 `platform/freertos`，领域队列元素是 motion/pid/system request，不含线协议 version、Q16 格式或 vendor 类型。
- `serial_protocol_send/try_send` 仍接受当前 native DTO，平台 adapter 经纯 codec 编码到字节数组后调用 LwPKT；不会复制 DTO 原始内存上网，也不会直接写 UTF-8 到 LwRB。
- 保留命令 0x01/0x02/0x03、遥测 0x81/0x84/0x85/0x86/0x87 的原字节布局。0x82/0x83 未定义 payload，明确拒绝，未擅自造新语义。
- Protocol engine 注入 rings、回调和时间，每轮有字节与结果数量预算；RTOS adapter 持有 mutex/task/transport，运行中拒绝 start/stop 生命周期操作。
- Fusion context 的内存由调用者分配，算法不自行分配。FreeRTOS task 在启动前分配，并在 task 创建失败或 stop 时释放；公共头只暴露大小查询与 opaque 指针。
- `app_synctasks_init()` 只允许调度器启动前的一次启动尝试；失败先锁存停机并清理，重试需复位，避免重复装配破坏已创建资源。
- 时钟频率不可在运行中改变，需至少每次 32 位 tick 回绕内观测一次；当前周期任务满足。不同 tick 率会产生不同周期量化结果：例如 100Hz 无法精确调度 5ms，时间正确换算不等于硬件仍以 200Hz 运行。

## 安全行为保留

- 正常上电保持停机，要求有效、已校准、非 STARTUP 的姿态及持续稳定资格后显式 Arm。
- 20ms 姿态/执行时效、重复/未来/乱序时间戳、最终写前重查、NaN/Inf、编码器异常、同周期停止优先等保护保留。
- Reset 与 Arm 分离；CLEAR_FAULT 不带 ENABLE 可以清故障但保持停止。
- 校准必须看见新的 CALIBRATING→健康结果，15s 超时；失效数值不会污染 Fusion 内部状态，正常数据可恢复处理。
- fatal 停机保持板级寄存器直达，关闭 TIM1 并将输入置低；不经过普通消息队列或调度器。
- EncoderTest 不启动平衡或 IMU 任务，忽略所有运动输入；电机诊断等待/运行期间 fatal 后不可重新启动。
- 通信超时仍让目标运动回零并保留直立；与硬件故障锁存的用途不同。

## 交叉审查中修正的收尾问题

1. 不透明 Fusion 内存引入生命周期所有权：补 stop、任务创建失败释放和状态复位。
2. 同时启用 IMU 与电机诊断时句柄覆盖：独立保存 IMU 诊断句柄。
3. 应用重复初始化会覆盖队列：入口增加启动前/单次尝试限制。
4. 已知设备方向遥测回流会被误计队列满：路由明确忽略这类消息。
5. app_diagnostics 公共头的 RTOS/应用配置依赖未发布：修正 PUBLIC usage requirements，其余保持 PRIVATE。
6. IMU 诊断仍用 HAL 字段名并携带前一次错误：公共类型改为 transport 字段，HAL 后端每次更新，采样后取诊断，再映射原 wire 字段；失败清理保留原错误。

## 验证结果

本轮完整入口：

```powershell
pwsh -NoProfile -File tools\run_static_checks.ps1 -IncludeDiagnostics
```

- **46/46 CTest 项通过**，包括纯 runtime、adapter、计数器、motor/encoder/IMU/UART 后端、协议 codec/engine、opaque Fusion、时钟、应用路由与边界检查；Python 18 项互通包含在 encoder_monitor 项中。
- **7/7 ARM 配置构建通过**：Debug、Release、EncoderTest、PWM 电机诊断、GPIO 电机诊断、IMU UART 诊断、IMU+电机诊断。
- 所有主机自有测试/核心以 `-Wall -Wextra -Werror` 编译；codec 另以 `-fpack-struct=1` 验证 native ABI 改变不影响线上 golden bytes。
- 核心以同一 CMake 目标在 ARM 与 host 构建。运行时/硬件替身只用于 adapter 测试，未通过给核心塞 stub 来掩盖平台依赖。
- ELF 核查包括 RTOS SysTick/SVC/PendSV、TIM11/HAL 周期回调、UART DMA 回调和 fatal stop，避免改成静态库后意外保留空弱回调。

核查产物在 `build/static-fix`；当前审查统计在 `build/architecture-refactor/audit`。此前 `build/architecture-audit` 保存重构前快照。

## 工具与文档

- `tools/check_architecture.py` 纳入 CTest，并检查真实 ARM 编译数据库；防止之后重新把 HAL/RTOS/Fusion 细节引入核心。
- `tools/audit_architecture.py` 支持旧结构与新分层，输出逐文件 CSV、依赖位置、哈希和 JSON；新统计不会覆盖重构前快照。
- 9 个旧分析脚本改为必须显式传输入路径，移除私人桌面/会话路径。旧 decode_telemetry 的无依据“链路完全健康”结论已删除；新 encoder_monitor 的 CRC 解析方式不变。
- `README.md` 是当前入口；此前报告和交接文档保留历史背景。

## 尚不能离线证明的内容

用户已确认 3V3 恢复，但本轮没有测量硬件。UART/蓝牙链路、引脚波形、真实时间裕量、编码器方向与比例、机械零点、死区、PID 及稳定站立仍需实车验收。速度环默认保持 0/0，未猜测参数或宣称完成调参。硬件独立看门狗、电池保护与完整调参产品功能也不属于本次解耦验收。

移植到新 MCU/RTOS 仍需实现或替换相应硬件/同步后端，再做目标机验收；本次交付的是隔离平台依赖且可独立测试的核心，不是未经验证的第二块板移植版本。
