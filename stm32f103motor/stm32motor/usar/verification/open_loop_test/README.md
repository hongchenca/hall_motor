# 定时开环六步换相测试

此测试实际编译并执行 `usar/bldc_motor.c`，不复制换相状态机。ARMCC 构建不覆盖 `MOTOR_OPEN_LOOP_TEST`，启动断言要求当前默认配置为 1；正常霍尔换相版本由 `../bldc_test/` 覆盖，该套件明确使用模式 0。

在固件工程根目录（`stm32f103motor/stm32motor`）的 PowerShell 中运行，确保 Python 已加入 PATH：

```powershell
python .\usar\verification\open_loop_test\run_open_loop_tests.py
```

依赖 ARMCC 5、Python 的 `unicorn` 和 `pyelftools`。默认编译器位于 `D:\Keil_v5\ARM\ARMCC\bin`，可用 `ARMCC_BIN` 覆盖；Python 依赖可安装到当前解释器，或通过 `STM32_TEST_PYTHON_DEPS` 指定额外搜索路径。外设模拟复用 `../bldc_test/harness.c` 和 `mock/`，独立构建到本目录，不覆盖 Keil 固件或正常模式的测试产物。

## 覆盖范围

- 有效启动命令后立即进入 `BLDC_RUNNING` 并输出第 1 步；无预充、对齐或加速阶段。
- 正、反转完整六步 CCER/CCMR 输出；换相严格间隔 2000 个 50 us tick，即 100 ms。
- Hall 为 0、7、固定值或连续变化时，开环相序不受影响；持续运行超过 5 秒也不自动停机或锁存故障。
- 有符号电压命令入口的 10% 占空比上限，包括 `-32768`；低于最小占空比停止。
- 零电压命令和紧急停止立即关闭输出；反向命令先全关断滑行，再从反向第 1 步启动。
- 命令 499/500 ms 超时与时钟回绕。
- PB12 低有效、Break IRQ、残留 BIF、MOE 等待中的短脉冲、清除后再次触发；清故障不要求有效霍尔，但仍要求零目标、健康供电/ADC 和 PB12 高。
- ADC 40/41 tick 缺失边界、DMA 故障、欠压/过压启动闭锁及运行停机、HAL 初始化失败。

`results.json` 记录源码/配置 SHA256、实际构建命令、用例结果和寄存器证据；`build.log` 记录构建输出。这些文件均为可重新生成的本地产物，不纳入源码版本控制。

该模拟不验证实际电机转动、失步、电流、温升、机械转速或真实 PWM/栅极波形，也不覆盖应用层 USART3 的命令解析及 100 ms Hall 打印。占空比上限不是电流限制；实板测试仍需独立确认。
