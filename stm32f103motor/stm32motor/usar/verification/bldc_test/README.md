# BLDC 模块可重放测试

本目录用 ARMCC 将真实 `usar/bldc_motor.c` 以 `MOTOR_OPEN_LOOP_TEST=0` 编译，明确验证保留的正常霍尔换相模式；不覆盖或旁路任何保护配置。在 Unicorn 中执行 Cortex-M3 Thumb 指令；仅替换 HAL、外设寄存器存储和输入注入。未复制换相或状态机算法。独立的定时开环测试见 `../open_loop_test/`。

在固件工程根目录（`stm32f103motor/stm32motor`）的 PowerShell 中运行，确保 Python 已加入 PATH：

```powershell
python .\usar\verification\bldc_test\run_bldc_tests.py
```

依赖：`D:\Keil_v5\ARM\ARMCC\bin` 中的 `armcc.exe`、`armlink.exe`，以及 `%TEMP%\stm32motor-can-test-python` 中的 `unicorn`、`pyelftools`。其他环境可用 `ARMCC_BIN`、`STM32_TEST_PYTHON_DEPS` 指定位置。每次运行重新编译；所有用例通过返回 0，否则返回 1。

`results.json` 保存模块与配置 SHA256、正常配置及独立旧版对照的完整构建命令、25 个用例的通过状态和关键寄存器/状态快照；`build.log` 保存编译链接输出。每个用例创建全新的模拟器实例，按 ELF 具有 `SHF_ALLOC` 属性的 section 执行地址载入数据并将 `SHT_NOBITS` 清零，正确处理 ARMCC 的 RW scatter-copy 布局。

覆盖：64 点 ADC 平均边界、ADC=0、PA1/PA2 双通道隔离与母线保护、仅收到半组数据不消费、DMA 错误立即停机且禁止直接清除、PB0:PA7:PA6 霍尔位序对应六路物理输入、正反六步换相、预充边界、负数最小值限幅、反转滑行后重新预充、499/500 ms 超时与 uint32 时间回绕、PB12 低电平上电即锁存 fault=5、运行时低电平软件兜底（不注入 Break IRQ）、Break 中断锁存/清除及启动前残留标志、低电平拒绝清故障、高电平不自动清除故障、显式清除后 BIE 重开且输出保持关闭、收到新命令重启后再次触发保护、ADC 缺失 40/41 tick 及恢复后的 63/64 组新样本边界、Hall 0/7、Hall 不变 65534/65535 tick 堵转、过压启动闭锁和 HAL 初始化失败。

模拟中 TIM2 CNT 每次读取递增，`Test_Advance` 每 20 个 FastTick 推进 1 ms。霍尔用例把逻辑位按 PB0→bit2、PA7→bit1、PA6→bit0 映射到模拟 GPIO，实际编译执行 `read_hall()`，再核对换相相位。模拟 HAL 记录应用传入的 DMA 缓冲区和长度；仅有软件触发时，测试才向该缓冲区写入 PA1/PA2 并调用完成回调，同时令 DR 保留第二路数据、EOC 已清除，验证应用不再依赖单通道 DR/EOC。半组注入只写 PA1、不调用完成回调。堵转用例每 100 ms 更新有效命令，使其不先触发 500 ms 通讯超时。故障清除后验证 BKE 保持启用、BIE 重新启用、BIF 清除、MOE 仍关闭，必须收到新命令才进入预充；重启后再次注入 PB12 低和 Break IRQ，验证仍能锁存 fault=5 并关断。上电低电平用例在首次 Bldc_Init 前注入 PB12=0。测试边界预置 CubeMX 应生成的 BKE=1；模拟的 Break 硬件动作只在 BKE=1 时执行，不模拟真实异步电气响应。GPIO 软件兜底用例只拉低 PB12，不模拟硬件清 MOE 或调用 IRQ，以独立验证软件低有效判断。

## Break 快照与 MOE 同步读对照

新增断言覆盖五种 Break 来源：初始化低电平、周期采样低电平、IRQ、开输出前检查和开输出后检查。检查第一次快照保留、失败清故障不删除证据、成功清除和初始化重置证据。IRQ 测试按“捕获→HAL 清 BIF→回调锁存”的顺序模拟，确认保存的 BIF=1 不会被后续清标志或低电平周期检查覆盖；只有 BIF 与 BIE 都有效才捕获。

`synthetic_moe_readback_delay_old_vs_current` 使用**明确人工模型**：MOE 从 0 写 1 后，必须发生至少两次 TIM2 CNT 读取，MOE 才能读回 1。对照源是随工程提供的 `fixtures/pre_moe_sync/` 中四个原样保存的文件（旧 `bldc_motor.c`、旧头文件、旧 harness 及固定配置），运行不依赖 `usar/backup`。来源与 SHA256 见 fixture 的 README；构建到本目录独立的 `bldc_moe_baseline.axf`，不覆盖正常构建，也不将 fixture 用于刷机。模型验证旧代码立即读回导致 fault=5、新代码确实经过计数器等待后才能成功；**它不是 STM32 周期精确模型，也不证明本次实板故障的根因**。报告记录旧版来源及 SHA256。

`short_break_during_moe_wait_still_stops` 在等待期间人工注入一次低脉冲（BIF 置位、MOE 清除），下一次计数器读时让 PB12 恢复高，确认等待不会绕开保护、ENABLE_POSTCHECK 快照仍捕获到 Break 并停机。测试仅对边界寄存器和 GPIO 做注入，不替代示波器和实板验证。

本测试验证应用状态与寄存器写入，未模拟实际栅极/PWM 波形、定时器预装载转移、ADC 模拟噪声与采样相位、真实 Break 电气响应、异步 IRQ 抢占以及 Hall 毛刺；这些仍需上板验证。
