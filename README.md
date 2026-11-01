# STM32F103 六步电机驱动

当前工程位于 `stm32f103motor/stm32motor`，包含 STM32CubeMX 配置、Keil 工程、HAL/CMSIS 依赖、业务源码及可重放的软件回归测试。

## 功能

- BLDC 霍尔六步换相，以及可选的定时开环六步测试模式；支持 CAN 或 USART3 占空比指令。
- PB12 / TIM1_BKIN 低有效硬件保护、故障锁存及 `BKIN` 诊断；没有旁路开关。
- 母线电压与 PA2 双通道 ADC DMA 采样。
- PA4/PA5 方向控制、PB1 PWM 的有刷直流电机控制。
- PB8 舵机输出、USART1 RS485 与 MT_6V10 Modbus 编码器反馈。

霍尔接线：PA6=HA、PA7=HB、PB0=HC；当前编码为 `hall = (HC << 2) | (HB << 1) | HA`。电机实际霍尔/相线关系仍待实板验证，不应把软件回归通过视作整板或电机运行验收。

## 当前测试配置

`usar/motor_config.h` 默认 `MOTOR_OPEN_LOOP_TEST=1`：发送 `MOTOR START` 后从第一步开始定时六步换相，每步固定 100 ms；运行时每 100 ms 从 USART3 打印一次霍尔值。默认占空比 10%，可用 `MOTOR SPEED` 调整；`MOTOR STOP` 立即关闭输出。PB12 / BKIN、母线电压、ADC 和通信超时保护仍生效。此模式不是反电动势闭环无感控制，也没有电流闭环；应空载、限流短测。

发送 `MOTOR START` 即可按默认速度启动；可选 `MOTOR SPEED 8..10` 和 `MOTOR DIR FWD|REV`。停止发送 `MOTOR STOP`。修改 `MOTOR_OPEN_LOOP_TEST=0` 并重新构建、烧录即可恢复霍尔换相。完整接线与保护说明见业务目录文档。

## 构建

使用 Keil MDK / ARM Compiler 5 打开：

```text
stm32f103motor/stm32motor/MDK-ARM/stm32motor.uvprojx
```

也可以在 PowerShell 中从工程目录运行：

```powershell
cd .\stm32f103motor\stm32motor
powershell -ExecutionPolicy Bypass -File .\usar\verification\rebuild.ps1
```

构建脚本的默认 Keil 路径可通过 `-Uv4` 参数覆盖。HEX 由构建生成在 `MDK-ARM/stm32motor/stm32motor.hex`，不作为源码提交。

## 文档与测试

- [引脚、通信命令与保护说明](stm32f103motor/stm32motor/usar/README.md)
- [BLDC 回归测试](stm32f103motor/stm32motor/usar/verification/bldc_test/README.md)
- [定时开环测试](stm32f103motor/stm32motor/usar/verification/open_loop_test/README.md)
- [有刷电机回归测试](stm32f103motor/stm32motor/usar/verification/brushed_test/README.md)
- [CAN 协议回归测试](stm32f103motor/stm32motor/usar/verification/can_test/README.md)
- [MT_6V10 回归测试](stm32f103motor/stm32motor/usar/verification/mt6v10_test/README.md)

测试所需的 Python 依赖与 ARMCC 配置见各测试目录说明。日志、结果 JSON、编译中间文件及本地历史备份不纳入版本控制；必要的 MOE 同步读回归对照源独立保存在测试 `fixtures` 目录中。

HAL/CMSIS 与原工程代码的版权声明保留在对应文件中。
